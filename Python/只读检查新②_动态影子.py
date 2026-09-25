"""新②动态影子检查：读取 Quest 与 JAKA，调用厂商逆解，但绝不发送运动命令。

在 PyCharm 直接点运行只显示说明。现场显式添加 ``--live-readonly`` 后，
本脚本占用本机 UDP 5005，接收 Quest 右手柄；对 Grip 按住期间的相对位姿，
以当前 Tool 1 TCP 为锚点生成候选目标，并调用 JAKA ``kine_inverse``。
仅记录候选解、耗时和异常；不调用 servo_j/servo_p、伺服开关、上电或使能。

为什么仍需此步骤：静止位姿的 50 次逆解不能揭示手柄运动时的分支跳变、
输入丢失和完整循环耗时。影子检查同样不能证明机器人实际运动安全。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Python" / "src"))

from vla_lab.engineering_teleop_live import LevelASettings, compose_target
from vla_lab.quest_vr_input import QuestUdpReceiver, RelativeQuestTracker


SDK_DIRECTORY = Path(os.environ.get("QUEST_JAKA_SDK_DIR", "__SET_QUEST_JAKA_SDK_DIR__"))
REPORT_DIRECTORY = ROOT / "Validation" / "sdk_dynamic_shadow"
CONFIG_PATH = ROOT / "Python" / "config" / "jaka_jog.json"


def checked(result: object, name: str):
    """只接受 SDK 明确成功的二元组；错误码原样写进报告。"""
    if not isinstance(result, tuple) or len(result) < 2 or result[0] != 0:
        raise RuntimeError(f"{name} 失败：{result!r}")
    return result[1]


def six_finite(values: object, name: str) -> tuple[float, ...]:
    if not isinstance(values, (tuple, list)) or len(values) != 6:
        raise ValueError(f"{name} 不是六维值")
    numbers = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError(f"{name} 含 NaN 或无穷大")
    return numbers


def duration_summary(values: list[float]) -> dict[str, float | int] | None:
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "median_ms": statistics.median(ordered),
        "p99_ms": ordered[math.ceil(0.99 * len(ordered)) - 1],
        "max_ms": ordered[-1],
        "over_8ms": sum(value > 8.0 for value in ordered),
    }


class DynamicShadow:
    """纯诊断状态机：手柄离合与坐标映射复用现有代码，SDK 仅调用只读接口。"""

    def __init__(self, robot, mapping: dict[str, str], *, mode: str = "full6d") -> None:
        if mode not in {"full6d", "position-only", "rotation-only"}:
            raise ValueError(f"未知影子模式：{mode}")
        self.robot = robot
        self.mode = mode
        self.tracker = RelativeQuestTracker(
            mapping, grip_on=0.75, grip_off=0.55, trigger_on=0.75, trigger_off=0.55
        )
        # 只用于构造诊断目标，速度字段不会下发给机器人。
        self.settings = LevelASettings(
            radius_mm=1000.0,
            translation_scale=1.0,
            linear_speed_mm_s=50.0,
            rotation_enabled=mode != "position-only",
            rotation_scale=1.0,
            rotation_deg=45.0,
            angular_speed_deg_s=15.0,
            joint_speed_deg_s=15.0,
            xyz_axes=(mode != "rotation-only",) * 3,
            rpy_axes=(True, True, True),
        )
        self.heading_locked = False
        self.anchor_tcp: tuple[float, ...] | None = None
        self.previous_solution: tuple[float, ...] | None = None
        self.previous_target: tuple[float, ...] | None = None

    def process(self, frame, received_perf_s: float) -> dict[str, object]:
        """处理一帧；返回可写 JSON 的诊断，不执行任何候选运动命令。"""
        record: dict[str, object] = {
            "at": datetime.now().astimezone().isoformat(),
            "source": frame.udp_source,
            "tracked": bool(frame.connected and frame.tracked and frame.valid),
            "grip": frame.grip,
            "rotation_valid": frame.rotation_valid,
            "input_age_ms_at_processing": (time.perf_counter() - received_perf_s) * 1000.0,
        }
        if not self.heading_locked:
            if not record["tracked"] or not frame.head_rotation_valid:
                record["event"] = "等待头显和右手柄有效跟踪"
                return record
            record["heading_yaw_deg"] = self.tracker.lock_heading(frame.head_rotation_xyzw)
            self.heading_locked = True
        events = self.tracker.update(frame, 1.0)
        names = [name for name, _ in events]
        record["events"] = names
        if "grip_stop" in names:
            self.anchor_tcp = None
            self.previous_solution = None
            self.previous_target = None
            record["event"] = "Grip 松开或跟踪丢失；诊断锚点已清除"
            return record
        if "grip_start" in names:
            # 只读抓取当前实际 TCP。每次新握持都重新锚定，绝不追赶旧目标。
            self.anchor_tcp = six_finite(
                checked(self.robot.get_actual_tcp_position(), "get_actual_tcp_position(anchor)"),
                "握持起点 TCP",
            )
            self.previous_solution = None
            self.previous_target = None
            record["anchor_tcp"] = self.anchor_tcp
        if self.anchor_tcp is None:
            record["event"] = "等待按住右 Grip"
            return record
        pose_events = [value for name, value in events if name == "pose_delta"]
        if not pose_events:
            record["event"] = "无完整手柄位姿"
            return record
        delta_mm, rotation = pose_events[-1]
        record["hand_delta_mm"] = delta_mm
        if rotation is None and self.mode != "position-only":
            record["event"] = "手柄姿态无效，未进行六维逆解"
            return record
        target = six_finite(compose_target(self.anchor_tcp, delta_mm, rotation, self.settings), "候选 TCP")
        record["target_tcp"] = target
        loop_started = time.perf_counter()
        try:
            tool_id = int(checked(self.robot.get_tool_id(), "get_tool_id"))
            if tool_id != 1:
                raise RuntimeError(f"Tool={tool_id}，不是 Tool 1")
            joints = six_finite(checked(self.robot.get_actual_joint_position(), "get_actual_joint_position"), "关节反馈")
            tcp = six_finite(checked(self.robot.get_actual_tcp_position(), "get_actual_tcp_position"), "TCP 反馈")
            feedback_perf_s = time.perf_counter()
            inverse_started = time.perf_counter()
            solved = six_finite(checked(self.robot.kine_inverse(joints, target), "kine_inverse"), "厂商逆解")
            inverse_ended = time.perf_counter()
            record.update({
                "event": "仅生成厂商逆解；未发送",
                "actual_joints_rad": joints,
                "actual_tcp": tcp,
                "solution_joints_rad": solved,
                "inverse_ms": (inverse_ended - inverse_started) * 1000.0,
                "read_and_inverse_ms": (inverse_ended - loop_started) * 1000.0,
                "input_age_ms_after_inverse": (inverse_ended - received_perf_s) * 1000.0,
                "feedback_age_ms_after_inverse": (inverse_ended - feedback_perf_s) * 1000.0,
                "largest_solution_vs_actual_deg": max(
                    math.degrees(abs(a - b)) for a, b in zip(solved, joints, strict=True)
                ),
            })
            if self.previous_solution is not None:
                record["largest_solution_step_deg"] = max(
                    math.degrees(abs(a - b))
                    for a, b in zip(solved, self.previous_solution, strict=True)
                )
            if self.previous_target is not None:
                record["target_position_step_mm"] = math.dist(target[:3], self.previous_target[:3])
            self.previous_solution, self.previous_target = solved, target
        except Exception as error:
            # 厂商二进制 SDK 可能抛出非标准 Python 异常；诊断脚本只记错误并清锚点。
            record["event"] = "只读诊断失败；本次握持停止评估"
            record["error"] = str(error)
            self.tracker.reset_grip()
            self.anchor_tcp = None
            self.previous_solution = None
            self.previous_target = None
        return record


def run(robot, *, seconds: float, port: int, mode: str = "full6d") -> tuple[dict[str, object], list[dict[str, object]]]:
    """只接收 UDP，且只调用 robot 的读数和逆解；返回全部事件供落盘。"""
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))["quest_vr"]
    observer = DynamicShadow(robot, config["direction_mapping"], mode=mode)
    events: list[dict[str, object]] = []
    source: tuple[str, int] | None = None
    ignored_other_sources = 0
    valid_packets = 0
    started = time.perf_counter()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.settimeout(0.2)
        receiver.bind(("127.0.0.1", port))
        print(f"仅监听 127.0.0.1:{port}，请佩戴头显并轻微移动右手柄；无运动命令。")
        while time.perf_counter() - started < seconds:
            try:
                data, address = receiver.recvfrom(65535)
            except socket.timeout:
                continue
            received_perf_s = time.perf_counter()  # 与 SDK 计时使用同一个高分辨率时钟。
            try:
                frame = QuestUdpReceiver._frame(json.loads(data.decode("utf-8")), f"{address[0]}:{address[1]}")
                valid_source = bool(frame.connected and frame.tracked and frame.valid)
                if source is None:
                    if not valid_source:
                        continue
                    source = address
                elif address != source:
                    ignored_other_sources += 1
                    continue
                if valid_source:
                    valid_packets += 1
                events.append(observer.process(frame, received_perf_s))
            except Exception as error:
                # 坏包或解析器异常只记诊断；这里没有可继续发送的运动命令。
                events.append({"event": "UDP 包或只读处理无效", "error": str(error)})
    solved = [event for event in events if "solution_joints_rad" in event]
    errors = [event for event in events if "error" in event]
    summary = {
        "schema": "jaka_vendor_ik_dynamic_shadow.v1",
        "created_at": datetime.now().astimezone().isoformat(),
        "duration_s": seconds,
        "control_mode": mode,
        "udp_port": port,
        "locked_source": f"{source[0]}:{source[1]}" if source else None,
        "valid_quest_packets": valid_packets,
        "ignored_other_source_packets": ignored_other_sources,
        "solved_frames": len(solved),
        "diagnostic_errors": len(errors),
        "inverse": duration_summary([float(item["inverse_ms"]) for item in solved]),
        "read_and_inverse": duration_summary([float(item["read_and_inverse_ms"]) for item in solved]),
        "input_age_after_inverse": duration_summary([float(item["input_age_ms_after_inverse"]) for item in solved]),
        "max_solution_step_deg": max((float(item.get("largest_solution_step_deg", 0)) for item in solved), default=None),
        "max_solution_vs_actual_deg": max((float(item["largest_solution_vs_actual_deg"]) for item in solved), default=None),
        "movement_commands_sent": 0,
        "safety_acceptance": "not_assessed",
        "interpretation": "动态只读诊断，不证明 servo_j 节拍、滤波器、物理运动或停机安全",
    }
    return summary, events


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-readonly", action="store_true", help="显式启用 Quest+JAKA 只读动态检查")
    parser.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", "127.0.0.1"))
    parser.add_argument("--sdk-dir", type=Path, default=SDK_DIRECTORY)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument(
        "--mode", choices=("full6d", "position-only", "rotation-only"), default="full6d",
        help="只读目标类型；用于分离平移与姿态造成的逆解变化",
    )
    args = parser.parse_args(argv)
    if not args.live_readonly:
        print("默认不连接机器人。现场只读检查请添加 --live-readonly --seconds 10。")
        return 0
    if not math.isfinite(args.seconds) or not 3 <= args.seconds <= 60:
        parser.error("--seconds 必须为 3—60")
    if not 1 <= args.port <= 65535:
        parser.error("--port 必须为 1—65535")
    sdk_dir = args.sdk_dir.resolve()
    if not sdk_dir.is_dir():
        parser.error(f"未找到 JAKA SDK：{sdk_dir}")
    if os.name == "nt":
        os.add_dll_directory(str(sdk_dir))
    sys.path.insert(0, str(sdk_dir))
    import jkrc  # 只有显式 --live-readonly 才导入二进制 SDK。

    robot = jkrc.RC(args.host)
    logged_in = False
    try:
        login_result = robot.login()
        if not isinstance(login_result, tuple) or not login_result or login_result[0] != 0:
            raise RuntimeError(f"login 失败：{login_result!r}")
        logged_in = True
        tool_id = int(checked(robot.get_tool_id(), "get_tool_id"))
        if tool_id != 1:
            raise RuntimeError(f"当前 Tool={tool_id}，需要 Tool 1")
        if not bool(checked(robot.is_in_pos(), "is_in_pos")):
            raise RuntimeError("机器人尚未静止；拒绝进行动态影子检查")
        if bool(checked(robot.is_in_servomove(), "is_in_servomove")):
            raise RuntimeError("控制器正在伺服模式；先由操作者正常退出该模式")
        summary, events = run(robot, seconds=args.seconds, port=args.port, mode=args.mode)
    finally:
        if logged_in:
            try:
                robot.logout()
            except Exception as error:
                print(f"警告：SDK logout 未确认：{error}", file=sys.stderr)
    REPORT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    stem = datetime.now().strftime("sdk_shadow_%Y%m%d_%H%M%S")
    report_path = REPORT_DIRECTORY / f"{stem}.json"
    events_path = REPORT_DIRECTORY / f"{stem}.jsonl"
    report_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    events_path.write_text("".join(json.dumps(event, ensure_ascii=False) + "\n" for event in events), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"报告：{report_path}\n逐帧记录：{events_path}")
    # 退出 0 仅代表收集到足够诊断数据，不代表允许真机运动。
    return 0 if summary["solved_frames"] >= 20 and summary["diagnostic_errors"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
