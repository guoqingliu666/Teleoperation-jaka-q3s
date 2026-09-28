"""新轨迹的真实SDK只读影子；默认不连接，必须明确 --live-readonly。

机械臂保持静止。本入口只读反馈、记录原始手柄和调用JAKA kine_inverse；
没有任何运动/使能/滤波设置接口。不能同时运行②真机控制会话。
"""

from __future__ import annotations

import argparse
from collections import Counter
import configparser
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import sys
import time
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Python" / "src"))
from vla_lab.jaka_telemetry import load_sdk, validate_host
from vla_lab.sampled_follow import checked, check_health
from vla_lab.sdk_owner_lease import (
    SdkOwnerLease,
    request_readonly_handoff,
    reject_legacy_bridge,
)
from vla_lab.quest_vr_input import QuestUdpReceiver
from vla_lab.single_owner_feedback import SingleOwnerFeedback
from vla_lab.trajectory_shadow import TrajectoryShadow
from vla_lab.trajectory_dynamics import inspect_exported_joint_settings
from vla_lab.adaptive_sampling import AdaptiveSampling
from vla_lab.trajectory_horizon import PreviewProfile
from dataclasses import asdict


READONLY_METHODS = frozenset(
    {
        "login",
        "logout",
        "kine_inverse",
        "get_actual_joint_position",
        "get_actual_tcp_position",
        "get_robot_status_simple",
        "get_tool_id",
        "get_user_frame_id",
        "is_in_estop",
        "is_in_collision",
        "is_on_limit",
        "is_in_servomove",
    }
)


class ReadonlyControl:
    """GUI仅能保持/结束只读会话，不能通过stdin转成运动模式。"""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.last_heartbeat = clock()
        self.stopped = False

    def accept(self, line):
        if line.strip() == "HEARTBEAT":
            self.last_heartbeat = self.clock()
        else:
            self.stopped = True  # STOP、EOF或未知命令均只结束。

    def reason(self):
        if self.stopped:
            return "操作者结束测试或界面连接关闭"
        if self.clock() - self.last_heartbeat > 3.0:
            return "界面心跳过期，结束只读测试"
        return None

    def listen(self):
        for line in sys.stdin:
            self.accept(line)
            if self.stopped:
                return
        self.stopped = True


class ReadOnlySDK:
    """显式白名单：误接入旧运动对象时也不能从本入口调用运动/写配置。"""

    def __init__(self, robot, clock=time.perf_counter):
        self.__robot = robot
        self.__clock = clock
        self.__timings = {}

    def timing_report(self):
        return {name: dict(values) for name, values in self.__timings.items()}

    def __getattr__(self, name):
        if name not in READONLY_METHODS:
            raise PermissionError(f"新轨迹只读入口禁止接口：{name}")
        method = getattr(self.__robot, name)

        def timed(*args, **kwargs):
            start = self.__clock()
            try:
                return method(*args, **kwargs)
            finally:
                elapsed = (self.__clock() - start) * 1000.0
                values = self.__timings.setdefault(
                    name, {"count": 0, "total_ms": 0.0, "max_ms": 0.0}
                )
                values["count"] += 1
                values["total_ms"] += elapsed
                values["max_ms"] = max(values["max_ms"], elapsed)

        return timed


def readonly_loop_delay(shadow):
    """待解期间只让出线程，不为每个逆解点额外等待2ms。

    仍然每轮只解一个点，下一轮先检查停止、最新输入和健康状态；
    不批量阻塞求解，也不放宽输入/积压过期保护。
    """
    return 0.0 if shadow.horizon.job is not None else 0.002


def read_local_settings(path):
    """只解析三个字面set值，不执行CMD，不展开环境变量或任何命令。"""
    allowed = {"QUEST_JAKA_HOST", "QUEST_JAKA_SDK_DIR", "QUEST_JAKA_LIMITS_FILE"}
    result = {}
    if not path.is_file():
        return result
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        match = re.fullmatch(r'\s*set\s+"([^=]+)=([^"\r\n]*)"\s*', line, re.IGNORECASE)
        if match is None or match[1].upper() not in allowed:
            continue
        key, value = match[1].upper(), match[2]
        if key in result or any(c in value for c in "%!\x00"):
            raise ValueError("本机配置含重复项或变量展开；请用命令行明确传入参数")
        result[key] = value
    return result


def input_status(frame, now, shadow, *, untracked_before_lock=0):
    """明确空跑原因；不把心跳轮询次数计作手柄帧数。"""
    if frame is None:
        if untracked_before_lock:
            return (
                "已收到UDP，但锁源前右手未有效TRACKED；请佩戴头显并让右手进入追踪范围"
            )
        return "尚无可用帧且未见未追踪帧；请检查①与UDP"
    age = now - frame.received_s
    if not 0 <= age <= 0.15:
        return "UDP数据已过期；检查①和头显"
    if not frame.connected:
        return "右手手柄未连接"
    if not frame.tracked or not frame.valid:
        return "右手未TRACKED或位置无效"
    if not frame.rotation_valid:
        return "右手姿态无效"
    if frame.button_b:
        return "B键停止中"
    if shadow.anchor is not None:
        return "握持采样中；仅计算厂商逆解，不运动"
    if not shadow.heading:
        return "先松Grip并确保头显姿态有效"
    if not shadow.release_seen:
        return "请先完全松开Grip再握住；旧窗口已失效"
    return "已见松Grip；握住右Grip并缓慢移动/转动"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-readonly", action="store_true")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--host")
    parser.add_argument("--sdk-dir", type=Path)
    parser.add_argument("--limits-file", type=Path)
    parser.add_argument("--local-config", type=Path, default=ROOT / "本机配置.cmd")
    parser.add_argument("--ui-control", action="store_true")
    parser.add_argument(
        "--sampling",
        choices=("adaptive", "fixed"),
        default="adaptive",
        help="默认动态采样；fixed仅用于与旧固定密度作只读对照",
    )
    parser.add_argument("--max-check-mm", type=float, default=5.0)
    parser.add_argument("--max-check-deg", type=float, default=1.0)
    parser.add_argument("--medium-pilot", action="store_true",
                        help="与②中等幅真机验收同包络的只读逆解检查；绝不运动")
    args = parser.parse_args(argv)
    if not args.live_readonly:
        print(
            "默认不连接。离线诊断请运行 轨迹连续性诊断.py。真实SDK只读影子需 --live-readonly。"
        )
        return 0
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error("只读影子时长必须为1到120秒")
    if args.medium_pilot and (args.sampling != "adaptive" or
            args.max_check_mm != 5.0 or args.max_check_deg != 1.0):
        parser.error("中等幅只读验收固定使用动态采样5mm/1°")
    try:
        sampling_policy = AdaptiveSampling(
            max_check_mm=args.max_check_mm, max_check_deg=args.max_check_deg
        )
    except ValueError as error:
        parser.error(str(error))
    local = read_local_settings(args.local_config)
    args.host = (
        args.host
        or os.environ.get("QUEST_JAKA_HOST")
        or local.get("QUEST_JAKA_HOST", "")
    )
    args.sdk_dir = Path(
        args.sdk_dir
        or os.environ.get("QUEST_JAKA_SDK_DIR")
        or local.get("QUEST_JAKA_SDK_DIR", "__NOT_CONFIGURED__")
    )
    args.limits_file = Path(
        args.limits_file
        or os.environ.get("QUEST_JAKA_LIMITS_FILE")
        or local.get("QUEST_JAKA_LIMITS_FILE", "__NOT_CONFIGURED__")
    )
    host = validate_host(args.host)
    exported_settings = inspect_exported_joint_settings(args.limits_file)
    config = configparser.ConfigParser(strict=False)
    config.read(args.limits_file, encoding="utf-8-sig")
    limits = [
        (
            float(config[f"JOINT_{i}"]["JOINT_MIN_LIMIT"]),
            float(config[f"JOINT_{i}"]["JOINT_MAX_LIMIT"]),
        )
        for i in range(6)
    ]
    if any(not math.isfinite(lo + hi) or lo >= hi for lo, hi in limits):
        parser.error("关节限位快照无效；不会连接SDK")
    mapping = json.loads(
        (ROOT / "Python/config/jaka_jog.json").read_text(encoding="utf-8")
    )["quest_vr"]["direction_mapping"]
    directory = ROOT / "Validation" / "trajectory_reference"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / datetime.now().strftime("readonly_%Y%m%d_%H%M%S_%f.jsonl")
    robot = lease = receiver = feedback = shadow = None
    logged = False
    failure = None
    cancelled = None
    control = ReadonlyControl() if args.ui_control else None
    if control:
        threading.Thread(
            target=control.listen, daemon=True, name="readonly-ui-control"
        ).start()
    input_counts = Counter()
    with path.open("x", encoding="utf-8", buffering=1) as file:

        def emit(**event):
            event["time_ns"] = time.time_ns()
            file.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
            if event["state"] not in (
                "trajectory_raw_sample",
                "trajectory_reference_span",
                "trajectory_vendor_preview",
            ):
                print(json.dumps(event, ensure_ascii=False), flush=True)

        try:
            # ①可能正在完成一次较慢的只读 SDK 调用。必须等它明确注销并回 ACK；
            # 超时只会拒绝本次检查，不会绕过双连接闸门。
            if not request_readonly_handoff(timeout_s=5.0):
                reject_legacy_bridge()
            else:
                # 控制器注销完成后仍可能短暂拒绝下一次 login。①在3秒后才会
                # 自动重连，这里留1秒会话切换时间，不会形成双连接窗口。
                time.sleep(1.0)
            lease = SdkOwnerLease()
            robot = ReadOnlySDK(load_sdk(args.sdk_dir).RC(host))
            checked(robot.login(), "只读login")
            logged = True
            check_health(robot)
            receiver = QuestUdpReceiver("127.0.0.1", 5005,
                **({"history_size": 32} if args.sampling == "adaptive" else {}))
            feedback = SingleOwnerFeedback(robot)
            shadow = TrajectoryShadow(
                robot,
                mapping,
                limits,
                emit=emit,
                adaptive=args.sampling == "adaptive",
                policy=sampling_policy,
                # 与②的新候选使用相同的速度、段上限和姿态范围。只读仍不发送
                # 任何运动；旧的宽角度数据仅用于离线对照，不冒充同参数验收。
                profile=(PreviewProfile(speed_mm_s=30., acceleration_mm_s2=60.,
                    angular_speed_deg_s=5., max_translation_mm=20., max_rotation_deg=2.)
                    if args.medium_pilot else
                    PreviewProfile(speed_mm_s=150., acceleration_mm_s2=400.,
                    angular_speed_deg_s=30., max_translation_mm=60., max_rotation_deg=6.)),
                rotation_radius_deg=(10. if args.medium_pilot else
                    180. if args.sampling == "adaptive" else 30.),
                position_radius_mm=100. if args.medium_pilot else 1000.,
            )
            end = time.monotonic() + args.seconds
            latest = None
            next_health = 0.0
            next_input_report = 0.0
            last_packet_s = None
            emit(
                state="trajectory_readonly_start",
                message="实机保持静止；先松Grip后握持；只读厂商逆解，不运动",
            )
            emit(state="trajectory_exported_settings", **exported_settings)
            emit(
                state="trajectory_sampling_configuration",
                mode=args.sampling,
                policy=asdict(sampling_policy),
                profile=asdict(shadow.horizon.profile),
                rotation_radius_deg=(10. if args.medium_pilot else
                    180. if args.sampling == "adaptive" else 30.),
                position_radius_mm=100. if args.medium_pilot else 1000.,
                medium_pilot=args.medium_pilot,
                scheduler="bounded_intent_v1" if args.sampling == "adaptive" else "legacy_fixed",
                reference_pipeline=("raw_pose_keypoints" if args.sampling == "adaptive"
                                    else "quintic_reference_comparison"),
                physical_C2_verified=False,
            )
            while time.monotonic() < end:
                if control and control.reason():
                    cancelled = control.reason()
                    break
                if receiver.error:
                    raise RuntimeError(receiver.error)
                if args.sampling == "adaptive" and getattr(receiver,"history_overflow",False):
                    raise RuntimeError("Quest只读输入历史窗口溢出；不能证明轨迹连续")
                frames = (receiver.recent() if args.sampling == "adaptive"
                          and hasattr(receiver,"recent") else [receiver.latest()])
                if time.monotonic() >= next_health:
                    check_health(robot)
                    next_health = time.monotonic() + 0.5
                for fresh in frames:
                    if fresh is None:
                        continue
                    latest = fresh
                    if latest.received_s != last_packet_s:
                        last_packet_s = latest.received_s
                        input_counts["received_packets"] += 1
                        if (latest.connected and latest.tracked and latest.valid
                                and latest.rotation_valid):
                            input_counts["valid_tracked_packets"] += 1
                        if latest.grip <= 0.55:
                            input_counts["released_packets"] += 1
                        if latest.grip >= 0.75:
                            input_counts["held_packets"] += 1
                    shadow.process(latest)
                if not frames:
                    shadow.process(latest)
                feedback.tick()
                if time.monotonic() >= next_input_report:
                    emit(
                        state="trajectory_input_status",
                        message=input_status(
                            latest,
                            time.monotonic(),
                            shadow,
                            untracked_before_lock=receiver.ignored_untracked_before_lock,
                        ),
                        ignored_untracked_before_lock=receiver.ignored_untracked_before_lock,
                        grip=latest.grip if latest else None,
                        trigger=latest.trigger if latest else None,
                        remaining_s=max(0.0, round(end - time.monotonic(), 1)),
                        planning=shadow.horizon.snapshot(),
                        input_counts=dict(input_counts),
                    )
                    next_input_report = time.monotonic() + 0.5
                time.sleep(readonly_loop_delay(shadow))
        except Exception as error:
            failure = str(error)
        finally:
            # 清理失败不跳过其它资源释放，也不掩盖原始错误。
            for resource in (receiver, feedback):
                if resource is not None:
                    try:
                        resource.close()
                    except Exception as error:
                        failure = f"{failure or ''}; 清理失败：{error}"
            if logged:
                try:
                    checked(robot.logout(), "只读logout")
                except Exception as error:
                    failure = f"{failure or ''}; 注销失败：{error}"
            if lease is not None:
                try:
                    lease.close()
                except Exception as error:
                    failure = f"{failure or ''}; 所有权释放失败：{error}"
            # 跑满30秒但没有有效握持/候选，不是验证成功。
            outcome = (
                "failed"
                if failure
                else (
                    "cancelled"
                    if cancelled
                    else (
                        "incomplete"
                        if shadow is None or not shadow.candidates or shadow.blocks
                        else "readonly_candidates_observed"
                    )
                )
            )
            emit(
                state="trajectory_readonly_finished",
                failure=failure,
                outcome=outcome,
                cancelled=cancelled,
                input_counts=dict(input_counts),
                ignored_untracked_before_lock=(
                    receiver.ignored_untracked_before_lock if receiver else 0
                ),
                candidates=shadow.candidates if shadow else 0,
                blocked_windows=shadow.blocks if shadow else 0,
                grip_windows=shadow.strokes if shadow else 0,
                largest_observed_twist_deg=shadow.max_twist_deg if shadow else 0,
                largest_observed_arc_deg=shadow.max_arc_deg if shadow else 0,
                joint_timing=shadow.timing.report() if shadow else None,
                sdk_call_timing=robot.timing_report() if robot else {},
                sampling_summary=shadow.sampling_summary if shadow else {},
                movement_commands_sent=0,
                physical_C2_verified=False,
            )
    print(f"报告：{path}")
    return 1 if failure else 2 if outcome == "incomplete" else 0


if __name__ == "__main__":
    raise SystemExit(main())
