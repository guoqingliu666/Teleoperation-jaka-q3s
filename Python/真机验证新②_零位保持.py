"""新②第一项真机门槛：用 servo_j 保持启动瞬间的当前关节角，不接收手柄目标。

默认运行只显示说明，不连接机器人。只有同时提供 ``--live-servo-hold``、
JAKA 工程师确认的 ``--lpf-cutoff`` 和确认短语才会连接。脚本不提供上电、
使能、清报警、移动到起点或自动恢复；现场状态不满足即拒绝。

这不是遥操作，也不是安全认证。它会实际进入关节伺服并发送“当前关节角”目标，
因此仍属于真机动作试验：独立急停必须由现场人员掌握，人员及物体必须离开范围。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Callable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SDK_DIRECTORY = Path(os.environ.get("QUEST_JAKA_SDK_DIR", "__SET_QUEST_JAKA_SDK_DIR__"))
REPORT_DIRECTORY = PROJECT_ROOT / "Validation" / "sdk_servo_hold"
SERVO_PERIOD_S = 0.008
CONFIRMATION = "保持当前关节，不跟随手柄"


def checked(result: object, name: str):
    if not isinstance(result, tuple) or not result or type(result[0]) is not int or result[0] != 0:
        raise RuntimeError(f"{name} 失败：{result!r}")
    return result[1] if len(result) > 1 else None


def checked_bool(result: object, name: str) -> bool:
    """兼容 JAKA Python SDK 用整数 0/1 表示布尔状态，但拒绝其它值。"""
    value = checked(result, name)
    if isinstance(value, bool):
        return value
    if type(value) is int and value in (0, 1):
        return bool(value)
    raise RuntimeError(f"{name} 返回的状态不是布尔或整数 0/1：{value!r}")


def six_finite(values: object, name: str) -> tuple[float, ...]:
    if not isinstance(values, (tuple, list)) or len(values) != 6:
        raise RuntimeError(f"{name} 不是六维值：{values!r}")
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise RuntimeError(f"{name} 含 NaN 或无穷大")
    return converted


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[math.ceil(fraction * len(ordered)) - 1]


def require_ready(robot) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """全部为 SDK 读取；本函数不改变控制器状态。"""
    simple = checked(robot.get_robot_status_simple(), "get_robot_status_simple")
    if not isinstance(simple, (tuple, list)) or len(simple) < 4:
        raise RuntimeError(f"机器人简要状态格式异常：{simple!r}")
    if not bool(int(simple[2])) or not bool(int(simple[3])):
        raise RuntimeError("机械臂必须已由 JAKA App 上电且使能；本脚本不会代做")
    if not checked_bool(robot.is_in_pos(), "is_in_pos"):
        raise RuntimeError("机械臂尚未静止")
    for method, label in (
        ("is_in_estop", "急停"),
        ("is_in_collision", "碰撞保护"),
        ("is_on_limit", "限位"),
    ):
        if checked_bool(getattr(robot, method)(), method):
            raise RuntimeError(f"机器人处于{label}状态")
    if checked_bool(robot.is_in_servomove(), "is_in_servomove"):
        raise RuntimeError("控制器已在伺服模式；请先正常退出占用程序")
    tool_id = int(checked(robot.get_tool_id(), "get_tool_id"))
    if tool_id != 1:
        raise RuntimeError(f"当前 Tool={tool_id}，不是 Tool 1")
    joints = six_finite(checked(robot.get_actual_joint_position(), "get_actual_joint_position"), "当前关节")
    tcp = six_finite(checked(robot.get_actual_tcp_position(), "get_actual_tcp_position"), "当前 TCP")
    return joints, tcp


def servo_hold(
    robot,
    target_joints: tuple[float, ...],
    *,
    lpf_cutoff: float,
    seconds: float,
    step_num: int = 1,
    command_period_s: float | None = None,
    max_lateness_s: float = 0.004,
    max_call_s: float | None = None,
    clock: Callable[[], float] = time.perf_counter,
    sleeper: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    """发送不变的实测关节角；按实际发送迟到量停机，不积压补发旧目标。

    Python SDK 是同步调用，返回耗时可以略跨过一个 8 ms 周期。一次调用跨周期
    不等于控制器已经失步；真正的门槛是下一发送机会的迟到量。为防止调用一直
    阻塞到会话结束却被误报成功，单次阻塞的兜底上限默认为“周期 + 迟到门槛”。
    """
    if not math.isfinite(lpf_cutoff) or lpf_cutoff <= 0:
        raise ValueError("LPF 截止频率必须为 JAKA 确认的正数")
    if step_num not in (1, 2, 3):
        raise ValueError("step_num 只允许锁定档位 1、2 或 3")
    interpolation_horizon_s = SERVO_PERIOD_S * step_num
    if command_period_s is None:
        command_period_s = interpolation_horizon_s
    if command_period_s <= 0 or command_period_s > interpolation_horizon_s:
        raise ValueError("命令更新周期必须大于 0，且不得超过厂商插补时域")
    servo_period_s = command_period_s
    if max_call_s is None:
        max_call_s = interpolation_horizon_s + max_lateness_s
    if not math.isfinite(max_call_s) or max_call_s <= interpolation_horizon_s:
        raise ValueError("SDK 单次阻塞兜底门槛必须大于厂商插补时域")
    enabled = False
    stop_confirmed = False
    stop_error: str | None = None
    call_ms: list[float] = []
    lateness_ms: list[float] = []
    send_interval_ms: list[float] = []
    queue_depths: list[int] = []
    previous_call_started: float | None = None
    send_count = 0
    skipped_update_slots = 0
    failure: str | None = None
    try:
        checked(robot.servo_move_use_joint_LPF(float(lpf_cutoff)), "servo_move_use_joint_LPF")
        checked(robot.servo_move_enable(True, True), "servo_move_enable(True)")
        enabled = True
        if not checked_bool(robot.is_in_servomove(), "is_in_servomove(start)"):
            raise RuntimeError("伺服启动读回不是 True")
        started = clock()
        frame_index = 0
        while True:
            now = clock()
            if now - started >= seconds:
                break
            deadline = started + frame_index * servo_period_s
            remaining = deadline - now
            if remaining > 0:
                sleeper(remaining)
            now = clock()
            lateness = max(0.0, now - deadline)
            # step_num=2 且每 8 ms 更新时，16 ms 厂商插补时域可覆盖一次偶发慢调用。
            # 此时跳过已经过期的发送槽，不在恢复后密集补发旧目标。
            if command_period_s < interpolation_horizon_s and lateness > max_lateness_s:
                if lateness >= interpolation_horizon_s:
                    raise RuntimeError(f"伺服调度迟到 {lateness * 1000.0:.3f} ms，超过插补时域")
                next_index = math.ceil((now - started) / servo_period_s)
                skipped_update_slots += max(0, next_index - frame_index)
                frame_index = next_index
                deadline = started + frame_index * servo_period_s
                remaining = deadline - now
                if remaining > 0:
                    sleeper(remaining)
                now = clock()
                lateness = max(0.0, now - deadline)
            lateness_ms.append(lateness * 1000.0)
            if lateness > max_lateness_s:
                raise RuntimeError(f"伺服调度迟到 {lateness * 1000.0:.3f} ms")
            call_started = clock()
            if previous_call_started is not None:
                send_interval_ms.append((call_started - previous_call_started) * 1000.0)
            previous_call_started = call_started
            queue_depth = checked(robot.servo_j(target_joints, 0, step_num), "servo_j")
            if queue_depth is not None:
                if type(queue_depth) is not int or not 0 <= queue_depth <= 100:
                    raise RuntimeError(f"servo_j 返回非法队列长度：{queue_depth!r}")
                queue_depths.append(queue_depth)
            call_ended = clock()
            elapsed = call_ended - call_started
            call_ms.append(elapsed * 1000.0)
            send_count += 1
            if elapsed > max_call_s:
                raise RuntimeError(f"servo_j 调用耗时 {elapsed * 1000.0:.3f} ms")
            frame_index += 1
    except Exception as error:
        failure = str(error)
    finally:
        if enabled:
            try:
                checked(robot.servo_move_enable(False, True), "servo_move_enable(False)")
                if checked_bool(robot.is_in_servomove(), "is_in_servomove(stop)"):
                    raise RuntimeError("伺服停止读回不是 False")
                stop_confirmed = True
            except Exception as error:
                stop_error = str(error)
    if stop_error:
        failure = f"{failure + '；' if failure else ''}伺服停止未确认：{stop_error}"
    return {
        "send_count": send_count,
        "skipped_update_slots": skipped_update_slots,
        "step_num": step_num,
        "interpolation_horizon_ms": interpolation_horizon_s * 1000.0,
        "command_period_ms": command_period_s * 1000.0,
        "servo_period_ms": servo_period_s * 1000.0,
        "blocking_call_stop_ms": max_call_s * 1000.0,
        "max_scheduler_lateness_ms": max_lateness_s * 1000.0,
        "servo_call_over_period_count": sum(value > servo_period_s * 1000.0 for value in call_ms),
        "servo_call_ms": {
            "median": statistics.median(call_ms) if call_ms else None,
            "p99": percentile(call_ms, 0.99) if call_ms else None,
            "max": max(call_ms, default=None),
        },
        "send_interval_ms": {
            "median": statistics.median(send_interval_ms) if send_interval_ms else None,
            "p99": percentile(send_interval_ms, 0.99) if send_interval_ms else None,
            "max": max(send_interval_ms, default=None),
        },
        "queue_depth": {
            "count": len(queue_depths),
            "median": statistics.median(queue_depths) if queue_depths else None,
            "max": max(queue_depths, default=None),
            "over_10": sum(value > 10 for value in queue_depths),
        },
        "scheduler_lateness_ms": {
            "median": statistics.median(lateness_ms) if lateness_ms else None,
            "p99": percentile(lateness_ms, 0.99) if lateness_ms else None,
            "max": max(lateness_ms, default=None),
        },
        "stop_confirmed": stop_confirmed,
        "failure": failure,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-servo-hold", action="store_true", help="显式允许零位保持真机试验")
    parser.add_argument("--confirm", default="", help=f"必须精确填写：{CONFIRMATION}")
    parser.add_argument("--lpf-cutoff", type=float, help="JAKA 工程师确认的 servo_j 关节 LPF 截止频率")
    parser.add_argument("--seconds", type=float, default=1.0, help="仅允许 0.5—2.0 秒")
    parser.add_argument("--step-num", type=int, choices=(1, 2, 3), default=1,
                        help="JAKA servo_j 厂商插补周期数；1=8 ms，2=16 ms，3=24 ms")
    parser.add_argument("--command-period-ms", type=float, choices=(8.0, 16.0, 24.0),
                        help="客户端命令更新周期；可与 step_num 的插补时域分开验证")
    parser.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", "127.0.0.1"))
    parser.add_argument("--sdk-dir", type=Path, default=SDK_DIRECTORY)
    args = parser.parse_args(argv)
    if not args.live_servo_hold:
        print("默认不连接、不进入伺服。准备完成后仍需 JAKA 确认 LPF 参数，再显式运行。")
        return 0
    if args.confirm != CONFIRMATION:
        parser.error(f"--confirm 必须精确填写：{CONFIRMATION}")
    if args.lpf_cutoff is None or not math.isfinite(args.lpf_cutoff) or args.lpf_cutoff <= 0:
        parser.error("必须通过 --lpf-cutoff 提供 JAKA 工程师确认的正数；程序没有默认值")
    if not math.isfinite(args.seconds) or not 0.5 <= args.seconds <= 2.0:
        parser.error("--seconds 只允许 0.5—2.0")
    sdk_dir = args.sdk_dir.resolve()
    if not sdk_dir.is_dir():
        parser.error(f"未找到 SDK：{sdk_dir}")
    if os.name == "nt":
        os.add_dll_directory(str(sdk_dir))
    sys.path.insert(0, str(sdk_dir))
    import jkrc

    robot = jkrc.RC(args.host)
    logged_in = False
    report: dict[str, object] = {
        "schema": "jaka_servo_j_current_joint_hold.v1",
        "created_at": datetime.now().astimezone().isoformat(),
        "host": args.host,
        "tool_id_required": 1,
        "lpf_cutoff": args.lpf_cutoff,
        "requested_seconds": args.seconds,
        "step_num": args.step_num,
        "command_period_ms": args.command_period_ms,
        "quest_used": False,
        "target_updates": 0,
    }
    try:
        login_result = robot.login()
        if not isinstance(login_result, tuple) or not login_result or login_result[0] != 0:
            raise RuntimeError(f"login 失败：{login_result!r}")
        logged_in = True
        before_joints, before_tcp = require_ready(robot)
        report["before_joints_rad"] = before_joints
        report["before_tcp"] = before_tcp
        report.update(servo_hold(
            robot,
            before_joints,
            lpf_cutoff=args.lpf_cutoff,
            seconds=args.seconds,
            step_num=args.step_num,
            command_period_s=(args.command_period_ms / 1000.0
                              if args.command_period_ms is not None else None),
        ))
        after_joints = six_finite(
            checked(robot.get_actual_joint_position(), "get_actual_joint_position(after)"),
            "结束关节",
        )
        report["after_joints_rad"] = after_joints
        report["max_observed_joint_change_deg"] = max(
            math.degrees(abs(after - before))
            for before, after in zip(before_joints, after_joints, strict=True)
        )
    except Exception as error:
        report["failure"] = str(error)
    finally:
        if logged_in:
            try:
                robot.logout()
            except Exception as error:
                report["logout_warning"] = str(error)
    REPORT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    output = REPORT_DIRECTORY / (datetime.now().strftime("servo_hold_%Y%m%d_%H%M%S") + ".json")
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告：{output}")
    return 0 if report.get("failure") is None and report.get("stop_confirmed") is True else 2


if __name__ == "__main__":
    raise SystemExit(main())
