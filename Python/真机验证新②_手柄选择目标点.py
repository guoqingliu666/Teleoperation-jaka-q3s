"""用 Quest 手柄选择一个位置终点，再交给 JAKA 控制器规划到达。

本阶段只开放 TCP 平移，不改变姿态：

1. 佩戴头显并保持 Grip、A 键松开；
2. 按住 Grip 时，把当前 Tool 1 TCP 与手柄位置同时记为起点；
3. 移动手柄，最大相对位移固定为 50 mm；
4. 松开 Grip 冻结目标；
5. 只读模式仅输出厂商逆解；真机模式还必须在冻结后再按一次右手 A 键；
6. 程序只发送一次 JAKA ``linear_move``，不逐帧复刻手柄轨迹。

默认运行不连接 Quest 或机器人。程序不负责上电、使能、清报警或切换 Tool。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import socket
import sys
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Python" / "src"))

from vla_lab.engineering_teleop_live import LevelASettings, compose_target
from vla_lab.quest_vr_input import QuestUdpReceiver, RelativeQuestTracker
from vla_lab.vr_robot_visualization import RobotVrBroadcaster


TARGET_SCRIPT = Path(__file__).with_name("真机验证新②_目标点运动.py")
TARGET_SPEC = importlib.util.spec_from_file_location("verified_target_point_gate", TARGET_SCRIPT)
if TARGET_SPEC is None or TARGET_SPEC.loader is None:
    raise RuntimeError(f"无法加载目标点门槛：{TARGET_SCRIPT}")
target_gate = importlib.util.module_from_spec(TARGET_SPEC)
TARGET_SPEC.loader.exec_module(target_gate)

CONFIG_PATH = ROOT / "Python" / "config" / "jaka_jog.json"
REPORT_DIRECTORY = ROOT / "Validation" / "quest_selected_target"
MAX_TARGET_DISTANCE_MM = 50.0
MIN_TARGET_DISTANCE_MM = 5.0
MAX_JOINT_DELTA_DEG = 12.0
SPEED_MM_S = 10.0
QUEST_MAX_AGE_S = 0.15
GRIP_ON = 0.75
GRIP_OFF = 0.55
TARGET_OVERLAY_ADDRESS = ("127.0.0.1", 5007)
MOTION_FEEDBACK_HZ = 20.0


class TargetOverlaySender:
    """只给数字孪生发送目标标记；这个通道不包含任何机器人运动指令。"""

    def __init__(self) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, target, *, phase: str, hold_s: float = 1.0) -> None:
        packet = {
            "schema": "quest_jaka_target_overlay.v1",
            "phase": str(phase),
            "target_tcp_mm_rad": None if target is None else [float(v) for v in target],
            "hold_s": float(hold_s),
        }
        self._socket.sendto(
            json.dumps(packet, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
            TARGET_OVERLAY_ADDRESS,
        )

    def clear(self) -> None:
        self.publish(None, phase="clear", hold_s=0.1)

    def close(self) -> None:
        self._socket.close()


class EndpointSelector:
    """把一段 Grip 手势压缩成一个固定 TCP 位置终点；不拥有运动命令。"""

    def __init__(self, mapping: dict[str, str], overlay: TargetOverlaySender | None = None,
                 speed_mm_s: float = SPEED_MM_S) -> None:
        self.tracker = RelativeQuestTracker(
            mapping,
            grip_on=GRIP_ON,
            grip_off=GRIP_OFF,
            trigger_on=0.75,
            trigger_off=0.55,
        )
        self.settings = LevelASettings(
            radius_mm=MAX_TARGET_DISTANCE_MM,
            translation_scale=1.0,
            linear_speed_mm_s=float(speed_mm_s),
            rotation_enabled=False,
            rotation_scale=1.0,
            rotation_deg=1.0,
            angular_speed_deg_s=1.0,
            joint_speed_deg_s=1.0,
            xyz_axes=(True, True, True),
            rpy_axes=(False, False, False),
        )
        self.heading_locked = False
        self.release_seen = False
        self.anchor_tcp: tuple[float, ...] | None = None
        self.anchor_joints: tuple[float, ...] | None = None
        self.candidate: tuple[float, ...] | None = None
        self.stop_requested = False
        self.overlay = overlay

    def reset_cycle(self) -> None:
        """一次运动完成后清除旧锚点；下一轮必须从当前真机反馈重新锚定。"""
        self.tracker.reset_grip()
        self.anchor_tcp = self.anchor_joints = self.candidate = None
        self.release_seen = True
        if self.overlay is not None:
            self.overlay.clear()

    def process(self, frame, robot) -> str:
        """处理最新 Quest 帧；只读取机器人反馈，返回状态文字。"""
        tracked = bool(frame.connected and frame.tracked and frame.valid)
        if not tracked or time.monotonic() - frame.received_s > QUEST_MAX_AGE_S:
            self.tracker.reset_grip()
            self.anchor_tcp = self.anchor_joints = self.candidate = None
            if self.overlay is not None:
                self.overlay.clear()
            return "等待新鲜且有效的右手柄追踪"
        if not self.heading_locked:
            if not frame.head_rotation_valid:
                return "等待头显姿态有效"
            if frame.grip > GRIP_OFF or frame.button_a:
                return "首次锁定前请松开 Grip 和 A"
            self.tracker.lock_heading(frame.head_rotation_xyzw)
            self.heading_locked = True
            self.release_seen = True
            return "正前方已锁定；按住 Grip 选择位置终点"

        events = self.tracker.update(frame, 1.0)
        names = [name for name, _ in events]
        if "restore_saved_position" in names:
            self.stop_requested = True
            self.tracker.reset_grip()
            self.anchor_tcp = self.anchor_joints = self.candidate = None
            if self.overlay is not None:
                self.overlay.clear()
            return "检测到右手 B：请求结束连续会话"
        if "grip_start" in names:
            if not self.release_seen:
                self.tracker.reset_grip()
                return "重新进入后必须先完整松开一次 Grip"
            joints, tcp = target_gate.require_ready(robot)
            self.anchor_joints, self.anchor_tcp = joints, tcp
            self.candidate = tcp
            self.release_seen = False
            if self.overlay is not None:
                self.overlay.publish(self.candidate, phase="selecting", hold_s=1.0)
        for name, value in events:
            if name == "pose_delta" and self.anchor_tcp is not None:
                delta_mm, _rotation = value
                self.candidate = target_gate.six_finite(
                    compose_target(self.anchor_tcp, delta_mm, None, self.settings),
                    "手柄候选终点",
                )
                if self.overlay is not None:
                    self.overlay.publish(self.candidate, phase="selecting", hold_s=1.0)
        if "grip_stop" in names:
            self.release_seen = True
            if self.anchor_tcp is not None and self.candidate is not None:
                if self.overlay is not None:
                    self.overlay.publish(self.candidate, phase="frozen", hold_s=60.0)
                return "目标已冻结"
            return "Grip 已松开；等待重新选择"
        if self.anchor_tcp is not None and self.candidate is not None:
            distance = math.dist(self.anchor_tcp[:3], self.candidate[:3])
            return f"正在选择目标：相对位移 {distance:.1f} mm"
        return "等待按住 Grip"

    def frozen_target(self) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]] | None:
        if not self.release_seen or self.anchor_tcp is None or self.anchor_joints is None or self.candidate is None:
            return None
        distance = math.dist(self.anchor_tcp[:3], self.candidate[:3])
        if distance < MIN_TARGET_DISTANCE_MM:
            raise RuntimeError(f"手柄终点仅移动 {distance:.1f} mm，小于 {MIN_TARGET_DISTANCE_MM:.0f} mm 门槛")
        if distance > MAX_TARGET_DISTANCE_MM + 1e-6:
            raise RuntimeError("手柄终点超过 50 mm 固定范围")
        return self.anchor_joints, self.anchor_tcp, self.candidate


def wait_for_frozen_target(selector: EndpointSelector, receiver: QuestUdpReceiver, robot, timeout_s: float):
    started = time.monotonic()
    last_status = ""
    while time.monotonic() - started < timeout_s:
        frame = receiver.latest()
        if frame is None:
            if receiver.error:
                raise RuntimeError(f"Quest UDP 接收失败：{receiver.error}")
            time.sleep(0.01)
            continue
        status = selector.process(frame, robot)
        if status != last_status:
            print(status)
            last_status = status
        if selector.stop_requested:
            raise InterruptedError("操作者按右手 B 结束连续会话")
        frozen = selector.frozen_target()
        if frozen is not None:
            return frozen
    raise RuntimeError(f"{timeout_s:.0f} 秒内没有完成一次 Grip 目标选择")


def wait_for_a_confirmation(receiver: QuestUdpReceiver, timeout_s: float) -> None:
    """冻结后要求 A 键先处于松开，再检测一次新的按下沿。"""
    started = time.monotonic()
    released = False
    print("目标已通过逆解。保持 Grip 松开，按一次右手 A 键执行；超时自动取消。")
    while time.monotonic() - started < timeout_s:
        frame = receiver.latest()
        if frame is None:
            time.sleep(0.01)
            continue
        fresh = time.monotonic() - frame.received_s <= QUEST_MAX_AGE_S
        tracked = frame.connected and frame.tracked and frame.valid
        if not fresh or not tracked:
            raise RuntimeError("等待 A 键期间 Quest 追踪或数据流失效")
        if frame.grip > GRIP_OFF:
            raise RuntimeError("目标冻结后 Grip 必须保持松开")
        if not frame.button_a:
            released = True
        elif released:
            return
    raise RuntimeError("等待 A 键确认超时；未发送运动")


def wait_controls_released(receiver: QuestUdpReceiver, timeout_s: float = 10.0) -> None:
    """每次到位后等待 Grip/A/B 全部松开，防止上一轮按钮延续到下一轮。"""
    started = time.monotonic()
    while time.monotonic() - started < timeout_s:
        frame = receiver.latest()
        if frame is None:
            time.sleep(0.01)
            continue
        fresh = time.monotonic() - frame.received_s <= QUEST_MAX_AGE_S
        tracked = frame.connected and frame.tracked and frame.valid
        if fresh and tracked and frame.grip <= GRIP_OFF and not frame.button_a and not frame.button_b:
            return
    raise RuntimeError("到位后 10 秒内未检测到 Grip/A/B 全部松开")


def solve_candidate(robot, reference_joints, target):
    solved = target_gate.six_finite(
        target_gate.checked(robot.kine_inverse(reference_joints, target), "kine_inverse"),
        "手柄终点厂商逆解",
    )
    delta_deg = tuple(
        math.degrees(abs(a - b)) for a, b in zip(solved, reference_joints, strict=True)
    )
    if max(delta_deg) > MAX_JOINT_DELTA_DEG:
        raise RuntimeError(
            f"手柄终点最大关节变化 {max(delta_deg):.3f}° 超过 {MAX_JOINT_DELTA_DEG:.1f}° 门槛"
        )
    return solved, delta_deg


def make_motion_feedback_callback(robot, broadcaster: RobotVrBroadcaster, target):
    """创建运动期间的只读显示回调。

    这里复用②已经登录的 JAKA SDK 连接，只读取控制器实测关节角；不读取手柄目标
    来冒充真机状态，也不调用逆解、servo_j、servo_p 或任何运动接口。
    """
    def publish_measured_joints() -> None:
        joints = target_gate.six_finite(
            target_gate.checked(robot.get_actual_joint_position(), "get_actual_joint_position(display)"),
            "运动中实测关节角",
        )
        state = SimpleNamespace(
            feedback_source="motion_session",
            connected=True,
            powered_on=True,
            enabled=True,
            engineering_servo_active=False,
            tool_id=target_gate.TOOL_ID_REQUIRED,
            joints_rad=joints,
            tcp_pose=None,
            error="",
        )
        broadcaster.publish(
            state,
            armed=False,
            starting=False,
            active=False,
            target_tcp_mm_rad=target,
            robot_simulated=False,
        )

    return publish_measured_joints


def run_continuous_session(args, jkrc, config: dict) -> int:
    """持续多轮目标点遥操作；每轮仍只有一次厂商规划运动命令。"""
    session_started = time.monotonic()
    session_report: dict[str, object] = {
        "schema": "quest_selected_jaka_target_session.v1",
        "created_at": datetime.now().astimezone().isoformat(),
        "mode": "live-session-position-only",
        "max_target_distance_mm": MAX_TARGET_DISTANCE_MM,
        "speed_mm_s": args.speed_mm_s,
        "max_moves": args.max_moves,
        "session_seconds": args.session_seconds,
        "movement_commands_sent": 0,
        "moves": [],
    }
    robot = jkrc.RC(args.host)
    receiver = QuestUdpReceiver("127.0.0.1", args.port)
    overlay = TargetOverlaySender()
    motion_feedback = RobotVrBroadcaster(max_hz=MOTION_FEEDBACK_HZ)
    logged_in = False
    try:
        target_gate.checked(robot.login(), "login")
        logged_in = True
        target_gate.require_ready(robot)
        selector = EndpointSelector(config["direction_mapping"], overlay, args.speed_mm_s)
        print(
            "连续位置目标点遥操作已就绪：Grip 选点并松开，A 执行；"
            "到位后松开全部按钮即可继续，待机时按 B 结束。"
        )
        move_index = 1
        while move_index <= args.max_moves:
            remaining = args.session_seconds - (time.monotonic() - session_started)
            if remaining <= 0:
                print("连续会话已达到时间上限。")
                break
            print(f"\n第 {move_index}/{args.max_moves} 次：等待 Grip 选择终点……")
            selector.reset_cycle()
            try:
                anchor_joints, anchor_tcp, target = wait_for_frozen_target(
                    selector, receiver, robot, remaining
                )
            except InterruptedError as stop:
                session_report["stopped_by_operator"] = str(stop)
                break
            except RuntimeError as error:
                # 会话总时间耗尽才退出；一次手势太小或输入短时异常只取消本轮。
                if time.monotonic() - session_started >= args.session_seconds:
                    print(f"连续会话结束：{error}")
                    break
                print(f"本轮目标已取消：{error}；继续等待新的 Grip 手势。")
                continue
            try:
                solved, delta_deg = solve_candidate(robot, anchor_joints, target)
            except RuntimeError as error:
                print(f"本轮目标未通过厂商逆解门槛：{error}；机器人未运动。")
                continue
            distance = math.dist(anchor_tcp[:3], target[:3])
            print(
                f"终点只读检查通过：位移 {distance:.1f} mm，"
                f"最大关节变化 {max(delta_deg):.2f}°。"
            )
            try:
                wait_for_a_confirmation(receiver, 30.0)
            except RuntimeError as error:
                print(f"本轮执行确认已取消：{error}；机器人未运动。")
                continue
            current_joints, current_tcp = target_gate.require_ready(robot)
            if math.dist(current_tcp[:3], anchor_tcp[:3]) > 1.0:
                raise RuntimeError("选点后机器人 TCP 已变化超过 1 mm；拒绝沿用旧目标")
            solved, execution_delta = solve_candidate(robot, current_joints, target)
            print("A 已确认：开始执行一次真机直线运动……")
            execution = target_gate.execute_target(
                robot,
                target,
                speed_mm_s=args.speed_mm_s,
                expected_distance_mm=math.dist(current_tcp[:3], target[:3]),
                progress_callback=make_motion_feedback_callback(robot, motion_feedback, target),
            )
            after_joints, after_tcp = target_gate.require_ready(robot)
            error_mm = math.dist(after_tcp[:3], target[:3])
            if error_mm > target_gate.FINAL_TRANSLATION_TOLERANCE_MM:
                raise RuntimeError(f"第 {move_index} 次到位误差 {error_mm:.3f} mm 超过 1 mm")
            move_report = {
                "index": move_index,
                "anchor_tcp_mm_rad": anchor_tcp,
                "target_tcp_mm_rad": target,
                "selected_distance_mm": distance,
                "target_joints_rad": solved,
                "precheck_joint_delta_deg": delta_deg,
                "execution_joint_delta_deg": execution_delta,
                "after_joints_rad": after_joints,
                "after_tcp_mm_rad": after_tcp,
                "target_translation_error_mm": error_mm,
                **execution,
            }
            session_report["moves"].append(move_report)
            session_report["movement_commands_sent"] = move_index
            print(
                f"第 {move_index} 次到位：{execution['elapsed_s']:.2f} s，"
                f"误差 {error_mm:.3f} mm。请松开 Grip/A/B。"
            )
            wait_controls_released(receiver, 60.0)
            overlay.clear()
            move_index += 1
    except Exception as error:
        session_report["failure"] = str(error)
    finally:
        session_report["elapsed_s"] = time.monotonic() - session_started
        session_report["quest_source"] = receiver.source_endpoint
        session_report["quest_receiver_error"] = receiver.error
        session_report["ignored_untracked_before_lock"] = receiver.ignored_untracked_before_lock
        session_report["ignored_other_source_packets"] = receiver.ignored_other_source_packets
        receiver.close()
        overlay.clear()
        overlay.close()
        motion_feedback.close()
        if logged_in:
            try:
                robot.logout()
            except Exception as error:
                session_report["logout_warning"] = str(error)
    REPORT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    output = REPORT_DIRECTORY / datetime.now().strftime("session_%Y%m%d_%H%M%S.json")
    output.write_text(json.dumps(session_report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(session_report, ensure_ascii=False, indent=2))
    print(f"连续会话报告：{output}")
    return 0 if session_report.get("failure") is None else 2


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--live-readonly", action="store_true", help="采样一个手柄终点并只读逆解")
    mode.add_argument("--live-target", action="store_true", help="采样、A 键确认后执行一次目标点运动")
    mode.add_argument("--live-session", action="store_true", help="连续多轮位置目标点遥操作")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-moves", type=int, default=10)
    parser.add_argument("--session-seconds", type=float, default=600.0)
    parser.add_argument("--speed-mm-s", type=float, default=SPEED_MM_S)
    parser.add_argument("--sdk-dir", type=Path, default=target_gate.SDK_DIRECTORY)
    args = parser.parse_args(argv)
    if not args.live_readonly and not args.live_target and not args.live_session:
        print("默认不连接。先用 --live-readonly；真机模式还需 --confirm 手柄位置目标点一次运动。")
        return 0
    if args.live_target and args.confirm != "手柄位置目标点一次运动":
        parser.error("--confirm 必须精确填写：手柄位置目标点一次运动")
    if args.live_session and args.confirm != "连续手柄位置目标点遥操作":
        parser.error("--confirm 必须精确填写：连续手柄位置目标点遥操作")
    if not 10.0 <= args.timeout <= 60.0:
        parser.error("--timeout 必须为 10—60 秒")
    if not 1 <= args.max_moves <= 20:
        parser.error("--max-moves 必须为 1—20")
    if not 60.0 <= args.session_seconds <= 1800.0:
        parser.error("--session-seconds 必须为 60—1800 秒")
    if not 5.0 <= args.speed_mm_s <= 30.0:
        parser.error("当前软件速度范围为 5—30 mm/s；新增速度需逐级真机验收")
    sdk_dir = args.sdk_dir.resolve()
    if not sdk_dir.is_dir():
        parser.error(f"未找到 SDK：{sdk_dir}")
    if os.name == "nt":
        os.add_dll_directory(str(sdk_dir))
    sys.path.insert(0, str(sdk_dir))
    import jkrc

    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))["quest_vr"]
    if args.live_session:
        return run_continuous_session(args, jkrc, config)
    report: dict[str, object] = {
        "schema": "quest_selected_jaka_target.v1",
        "created_at": datetime.now().astimezone().isoformat(),
        "mode": "live-target" if args.live_target else "live-readonly",
        "max_target_distance_mm": MAX_TARGET_DISTANCE_MM,
        "speed_mm_s": args.speed_mm_s,
        "movement_commands_sent": 0,
    }
    robot = jkrc.RC(args.host)
    receiver = QuestUdpReceiver("127.0.0.1", args.port)
    overlay = TargetOverlaySender()
    motion_feedback = RobotVrBroadcaster(max_hz=MOTION_FEEDBACK_HZ)
    logged_in = False
    try:
        target_gate.checked(robot.login(), "login")
        logged_in = True
        target_gate.require_ready(robot)
        selector = EndpointSelector(config["direction_mapping"], overlay, args.speed_mm_s)
        anchor_joints, anchor_tcp, target = wait_for_frozen_target(
            selector, receiver, robot, args.timeout
        )
        solved, delta_deg = solve_candidate(robot, anchor_joints, target)
        report.update({
            "quest_source": receiver.source_endpoint,
            "anchor_tcp_mm_rad": anchor_tcp,
            "target_tcp_mm_rad": target,
            "selected_distance_mm": math.dist(anchor_tcp[:3], target[:3]),
            "target_joints_rad": solved,
            "joint_delta_deg": delta_deg,
            "max_joint_delta_deg": max(delta_deg),
        })
        if args.live_target:
            wait_for_a_confirmation(receiver, 15.0)
            current_joints, current_tcp = target_gate.require_ready(robot)
            if math.dist(current_tcp[:3], anchor_tcp[:3]) > 1.0:
                raise RuntimeError("选择目标后机器人 TCP 已变化超过 1 mm；拒绝沿用旧目标")
            solved, delta_deg = solve_candidate(robot, current_joints, target)
            report["execution_joint_delta_deg"] = delta_deg
            print("A 已确认：开始执行一次真机直线运动……")
            report.update(target_gate.execute_target(
                robot,
                target,
                speed_mm_s=args.speed_mm_s,
                expected_distance_mm=math.dist(current_tcp[:3], target[:3]),
                progress_callback=make_motion_feedback_callback(robot, motion_feedback, target),
            ))
        after_tcp = target_gate.six_finite(
            target_gate.checked(robot.get_actual_tcp_position(), "get_actual_tcp_position(after)"),
            "结束 TCP",
        )
        report["after_tcp_mm_rad"] = after_tcp
        report["target_translation_error_mm"] = math.dist(after_tcp[:3], target[:3])
        if args.live_target and report["target_translation_error_mm"] > target_gate.FINAL_TRANSLATION_TOLERANCE_MM:
            raise RuntimeError(
                f"到位后 TCP 误差 {report['target_translation_error_mm']:.3f} mm 超过 1 mm"
            )
    except Exception as error:
        report["failure"] = str(error)
    finally:
        # 必须在 close() 前保存；接收线程关闭后会主动清空 source_endpoint。
        report["quest_source"] = receiver.source_endpoint
        report["quest_receiver_error"] = receiver.error
        report["ignored_untracked_before_lock"] = receiver.ignored_untracked_before_lock
        report["ignored_other_source_packets"] = receiver.ignored_other_source_packets
        receiver.close()
        overlay.clear()
        overlay.close()
        motion_feedback.close()
        if logged_in:
            try:
                robot.logout()
            except Exception as error:
                report["logout_warning"] = str(error)
    REPORT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    prefix = "target" if args.live_target else "readonly"
    output = REPORT_DIRECTORY / datetime.now().strftime(prefix + "_%Y%m%d_%H%M%S.json")
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告：{output}")
    return 0 if report.get("failure") is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
