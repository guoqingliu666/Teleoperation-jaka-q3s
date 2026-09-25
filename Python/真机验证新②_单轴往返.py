"""新②第二项真机门槛：Tool 1 沿机器人基坐标 +Z 方向 1 mm 平滑往返。

设计目的：在不接入 Quest、不改变工具姿态、不进行自由遥操作的前提下，验证
JAKA 厂商 ``kine_inverse`` 的目标能否由已通过零位保持的 ``servo_j`` 链路
原样、连续地下发。程序不实现自己的逆运动学，也不修补或投影厂商逆解。

默认运行不会连接机器人。``--live-plan-only`` 只读状态并预计算逆解，不发送
运动命令；``--live-single-axis`` 才会执行固定的 1 mm、2 秒往返。程序不负责
上电、使能、清报警或恢复故障，任何门槛失败都会退出伺服且不会自动重试。
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


SDK_DIRECTORY = Path(os.environ.get("QUEST_JAKA_SDK_DIR", "__SET_QUEST_JAKA_SDK_DIR__"))
PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPORT_DIRECTORY = PROJECT_ROOT / "Validation" / "sdk_servo_single_axis"
SERVO_PERIOD_S = 0.008
MAX_LATENESS_S = 0.004
MAX_BLOCKING_CALL_S = SERVO_PERIOD_S + MAX_LATENESS_S
AXIS_INDEX = 2  # JAKA TCP 数组的第 3 项：机器人基坐标系 Z，单位 mm。
DISTANCE_MM = 1.0
DURATION_S = 2.0
MAX_SOLUTION_STEP_DEG = 0.10
MAX_SOLUTION_EXCURSION_DEG = 1.0
MAX_RETURN_ERROR_DEG = 0.05
MAX_RECOVERABLE_COMMUNICATION_GAP_S = 0.032
MAX_CONSECUTIVE_SKIPPED_POINTS = 4
SETTLE_DURATION_S = 1.0
PROFILES = {
    "1mm": {
        "distance_mm": 1.0,
        "duration_s": 2.0,
        "max_solution_excursion_deg": 1.0,
        "confirmation": "单轴Z正向1毫米往返，不使用手柄",
    },
    "5mm": {
        "distance_mm": 5.0,
        "duration_s": 3.0,
        "max_solution_excursion_deg": 2.0,
        "confirmation": "单轴Z正向5毫米往返，不使用手柄",
    },
}


def checked(result: object, name: str):
    """统一检查 JAKA Python SDK 的 ``(错误码, 数据...)`` 返回格式。"""
    if not isinstance(result, tuple) or not result or type(result[0]) is not int or result[0] != 0:
        raise RuntimeError(f"{name} 失败：{result!r}")
    return result[1] if len(result) > 1 else None


def checked_bool(result: object, name: str) -> bool:
    """JAKA Python SDK 现场返回整数 0/1；同时兼容真正的 bool。"""
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


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[math.ceil(fraction * len(ordered)) - 1]


def summary(values: list[float]) -> dict[str, float | None]:
    return {
        "median": statistics.median(values) if values else None,
        "p99": percentile(values, 0.99),
        "max": max(values, default=None),
    }


def require_ready(robot) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """只读检查；不替用户上电、使能、清报警或切换 Tool。"""
    simple = checked(robot.get_robot_status_simple(), "get_robot_status_simple")
    if not isinstance(simple, (tuple, list)) or len(simple) < 4:
        raise RuntimeError(f"机器人简要状态格式异常：{simple!r}")
    if not bool(int(simple[2])) or not bool(int(simple[3])):
        raise RuntimeError("机械臂必须已由 JAKA App 上电且使能")
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
        raise RuntimeError("控制器已在伺服模式；请先退出其它运动程序")
    tool_id = int(checked(robot.get_tool_id(), "get_tool_id"))
    if tool_id != 1:
        raise RuntimeError(f"当前 Tool={tool_id}，不是 Tool 1")
    joints = six_finite(checked(robot.get_actual_joint_position(), "get_actual_joint_position"), "当前关节")
    tcp = six_finite(checked(robot.get_actual_tcp_position(), "get_actual_tcp_position"), "当前 TCP")
    return joints, tcp


def build_plan(
    robot,
    initial_joints: tuple[float, ...],
    initial_tcp: tuple[float, ...],
    *,
    distance_mm: float = DISTANCE_MM,
    duration_s: float = DURATION_S,
    max_solution_excursion_deg: float = MAX_SOLUTION_EXCURSION_DEG,
    step_num: int = 1,
    command_period_s: float | None = None,
    clock: Callable[[], float] = time.perf_counter,
) -> tuple[list[tuple[float, ...]], dict[str, object]]:
    """调用厂商逆解生成固定轨迹；所有逆解都以实测起始关节为参考分支。"""
    if step_num not in (1, 2, 3):
        raise ValueError("step_num 只允许锁定档位 1、2 或 3")
    interpolation_horizon_s = SERVO_PERIOD_S * step_num
    if command_period_s is None:
        command_period_s = interpolation_horizon_s
    if command_period_s <= 0 or command_period_s > interpolation_horizon_s:
        raise ValueError("命令更新周期必须大于 0，且不得超过厂商插补时域")
    servo_period_s = command_period_s
    segments = round(duration_s / servo_period_s)
    plan: list[tuple[float, ...]] = []
    inverse_ms: list[float] = []
    max_step_deg = 0.0
    max_excursion_deg = 0.0
    previous = initial_joints
    for index in range(segments + 1):
        phase = index / segments
        # 升余弦往返：起点、最远点、终点的速度都为 0，避免阶跃目标。
        offset_mm = 0.5 * distance_mm * (1.0 - math.cos(2.0 * math.pi * phase))
        target = list(initial_tcp)
        target[AXIS_INDEX] += offset_mm
        started = clock()
        solved = six_finite(
            checked(robot.kine_inverse(initial_joints, tuple(target)), "kine_inverse"),
            f"第 {index} 帧厂商逆解",
        )
        inverse_ms.append((clock() - started) * 1000.0)
        step_deg = max(math.degrees(abs(a - b)) for a, b in zip(solved, previous, strict=True))
        excursion_deg = max(math.degrees(abs(a - b)) for a, b in zip(solved, initial_joints, strict=True))
        if step_deg > MAX_SOLUTION_STEP_DEG:
            raise RuntimeError(
                f"第 {index} 帧厂商逆解相邻跳变 {step_deg:.6f}° 超过 {MAX_SOLUTION_STEP_DEG:.3f}°"
            )
        if excursion_deg > max_solution_excursion_deg:
            raise RuntimeError(
                f"第 {index} 帧厂商逆解总偏移 {excursion_deg:.6f}° 超过 {max_solution_excursion_deg:.3f}°"
            )
        max_step_deg = max(max_step_deg, step_deg)
        max_excursion_deg = max(max_excursion_deg, excursion_deg)
        plan.append(solved)
        previous = solved
    return_error_deg = max(
        math.degrees(abs(a - b)) for a, b in zip(plan[-1], initial_joints, strict=True)
    )
    if return_error_deg > MAX_RETURN_ERROR_DEG:
        raise RuntimeError(
            f"末帧逆解未回到起始关节分支：{return_error_deg:.6f}° > {MAX_RETURN_ERROR_DEG:.3f}°"
        )
    motion_points = len(plan)
    settle_points = round(SETTLE_DURATION_S / servo_period_s)
    # LPF 截止频率较低，轨迹回到起点后必须继续发送同一末帧，等待滤波输出收敛。
    # 重复的是 JAKA 对起始 TCP 的厂商逆解结果，不创建新的关节目标。
    plan.extend([plan[-1]] * settle_points)
    return plan, {
        "plan_points": len(plan),
        "motion_points": motion_points,
        "settle_points": settle_points,
        "step_num": step_num,
        "interpolation_horizon_ms": interpolation_horizon_s * 1000.0,
        "command_period_ms": command_period_s * 1000.0,
        "servo_period_ms": servo_period_s * 1000.0,
        "motion_duration_s": segments * servo_period_s,
        "settle_duration_s": settle_points * servo_period_s,
        "planned_duration_s": (len(plan) - 1) * servo_period_s,
        "axis": "base_Z",
        "distance_mm": distance_mm,
        "orientation_change": 0.0,
        "inverse_ms": summary(inverse_ms),
        "max_solution_step_deg": max_step_deg,
        "max_solution_excursion_deg": max_excursion_deg,
        "return_solution_error_deg": return_error_deg,
    }


def execute_plan(
    robot,
    plan: list[tuple[float, ...]],
    *,
    lpf_cutoff: float,
    step_num: int = 1,
    command_period_s: float | None = None,
    clock: Callable[[], float] = time.perf_counter,
    sleeper: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    """原样发送预计算的厂商逆解；迟到不补发，失败即退出伺服。"""
    if step_num not in (1, 2, 3):
        raise ValueError("step_num 只允许锁定档位 1、2 或 3")
    interpolation_horizon_s = SERVO_PERIOD_S * step_num
    if command_period_s is None:
        command_period_s = interpolation_horizon_s
    if command_period_s <= 0 or command_period_s > interpolation_horizon_s:
        raise ValueError("命令更新周期必须大于 0，且不得超过厂商插补时域")
    servo_period_s = command_period_s
    max_blocking_call_s = max(
        interpolation_horizon_s + MAX_LATENESS_S,
        MAX_RECOVERABLE_COMMUNICATION_GAP_S,
    )
    enabled = False
    stop_confirmed = False
    stop_error: str | None = None
    failure: str | None = None
    send_count = 0
    call_ms: list[float] = []
    lateness_ms: list[float] = []
    send_interval_ms: list[float] = []
    queue_depths: list[int] = []
    previous_call_started: float | None = None
    skipped_plan_points = 0
    max_consecutive_skipped_points = 0
    max_runtime_sent_step_deg = 0.0
    previous_sent_target: tuple[float, ...] | None = None
    try:
        checked(robot.servo_move_use_joint_LPF(float(lpf_cutoff)), "servo_move_use_joint_LPF")
        checked(robot.servo_move_enable(True, True), "servo_move_enable(True)")
        enabled = True
        if not checked_bool(robot.is_in_servomove(), "is_in_servomove(start)"):
            raise RuntimeError("伺服启动读回不是 True")
        started = clock()
        index = 0
        while index < len(plan):
            deadline = started + index * servo_period_s
            now = clock()
            remaining = deadline - now
            if remaining > 0:
                sleeper(remaining)
            now = clock()
            lateness = max(0.0, now - deadline)
            # 偶发慢调用后直接跳到当前时刻对应的厂商逆解点；绝不密集补发旧目标。
            # 跳点数量和实际相邻下发关节差都受独立门槛限制。
            if lateness > MAX_LATENESS_S:
                next_index = min(math.ceil((now - started) / servo_period_s), len(plan) - 1)
                skipped_now = max(0, next_index - index)
                if skipped_now > MAX_CONSECUTIVE_SKIPPED_POINTS:
                    raise RuntimeError(
                        f"第 {index} 帧通信空窗需连续跳过 {skipped_now} 点，超过门槛 "
                        f"{MAX_CONSECUTIVE_SKIPPED_POINTS}"
                    )
                skipped_plan_points += skipped_now
                max_consecutive_skipped_points = max(max_consecutive_skipped_points, skipped_now)
                index = next_index
                deadline = started + index * servo_period_s
                remaining = deadline - now
                if remaining > 0:
                    sleeper(remaining)
                now = clock()
                lateness = max(0.0, now - deadline)
            lateness_ms.append(lateness * 1000.0)
            if lateness > MAX_LATENESS_S:
                raise RuntimeError(f"第 {index} 帧伺服调度迟到 {lateness * 1000.0:.3f} ms")
            target = plan[index]
            if previous_sent_target is not None:
                runtime_step_deg = max(
                    math.degrees(abs(a - b))
                    for a, b in zip(target, previous_sent_target, strict=True)
                )
                if runtime_step_deg > MAX_SOLUTION_STEP_DEG:
                    raise RuntimeError(
                        f"第 {index} 帧实际相邻下发关节差 {runtime_step_deg:.6f}° 超过 "
                        f"{MAX_SOLUTION_STEP_DEG:.3f}°"
                    )
                max_runtime_sent_step_deg = max(max_runtime_sent_step_deg, runtime_step_deg)
            call_started = clock()
            if previous_call_started is not None:
                send_interval_ms.append((call_started - previous_call_started) * 1000.0)
            previous_call_started = call_started
            # 关键约束：发送值就是 plan 中的 JAKA kine_inverse 原始返回值。
            queue_depth = checked(robot.servo_j(target, 0, step_num), f"第 {index} 帧 servo_j")
            if queue_depth is not None:
                if type(queue_depth) is not int or not 0 <= queue_depth <= 100:
                    raise RuntimeError(f"第 {index} 帧返回非法队列长度：{queue_depth!r}")
                queue_depths.append(queue_depth)
                if queue_depth > 10:
                    raise RuntimeError(f"第 {index} 帧控制器队列长度 {queue_depth} 超过门槛 10")
            elapsed = clock() - call_started
            call_ms.append(elapsed * 1000.0)
            send_count += 1
            previous_sent_target = target
            if elapsed > max_blocking_call_s:
                raise RuntimeError(f"第 {index} 帧 servo_j 阻塞 {elapsed * 1000.0:.3f} ms")
            index += 1
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
        "skipped_plan_points": skipped_plan_points,
        "max_consecutive_skipped_points": max_consecutive_skipped_points,
        "max_runtime_sent_step_deg": max_runtime_sent_step_deg,
        "step_num": step_num,
        "interpolation_horizon_ms": interpolation_horizon_s * 1000.0,
        "command_period_ms": command_period_s * 1000.0,
        "servo_period_ms": servo_period_s * 1000.0,
        "blocking_call_stop_ms": max_blocking_call_s * 1000.0,
        "max_scheduler_lateness_ms": MAX_LATENESS_S * 1000.0,
        "servo_call_over_period_count": sum(value > servo_period_s * 1000.0 for value in call_ms),
        "servo_call_ms": summary(call_ms),
        "send_interval_ms": summary(send_interval_ms),
        "queue_depth": {
            "count": len(queue_depths),
            "median": statistics.median(queue_depths) if queue_depths else None,
            "max": max(queue_depths, default=None),
            "over_10": sum(value > 10 for value in queue_depths),
        },
        "scheduler_lateness_ms": summary(lateness_ms),
        "stop_confirmed": stop_confirmed,
        "failure": failure,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--live-plan-only", action="store_true", help="连接并预计算逆解，不发送运动")
    mode.add_argument("--live-single-axis", action="store_true", help="执行选定的固定单轴往返")
    parser.add_argument("--profile", choices=tuple(PROFILES), default="1mm", help="锁定验收档位")
    parser.add_argument("--step-num", type=int, choices=(1, 2, 3), default=1,
                        help="JAKA servo_j 厂商插补周期数；1=8 ms，2=16 ms，3=24 ms")
    parser.add_argument("--command-period-ms", type=float, choices=(8.0, 16.0, 24.0),
                        help="客户端命令更新周期；可与 step_num 的插补时域分开验证")
    parser.add_argument("--lpf-cutoff", type=float, help="JAKA 工程师确认的关节 LPF 截止频率")
    parser.add_argument("--confirm", default="", help="真机运动必须填写与档位对应的确认短语")
    parser.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", "127.0.0.1"))
    parser.add_argument("--sdk-dir", type=Path, default=SDK_DIRECTORY)
    args = parser.parse_args(argv)
    if not args.live_plan_only and not args.live_single_axis:
        print("默认不连接。先用 --live-plan-only；通过后才可使用 --live-single-axis。")
        return 0
    profile = PROFILES[args.profile]
    if args.live_single_axis and args.confirm != profile["confirmation"]:
        parser.error(f"--confirm 必须精确填写：{profile['confirmation']}")
    if args.lpf_cutoff is None or not math.isfinite(args.lpf_cutoff) or args.lpf_cutoff <= 0:
        parser.error("必须通过 --lpf-cutoff 提供 JAKA 工程师确认的正数")
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
        "schema": "jaka_vendor_ik_single_axis_roundtrip.v1",
        "created_at": datetime.now().astimezone().isoformat(),
        "host": args.host,
        "mode": "live-single-axis" if args.live_single_axis else "live-plan-only",
        "profile": args.profile,
        "step_num": args.step_num,
        "command_period_ms": args.command_period_ms,
        "tool_id_required": 1,
        "lpf_cutoff": args.lpf_cutoff,
        "quest_used": False,
    }
    try:
        checked(robot.login(), "login")
        logged_in = True
        before_joints, before_tcp = require_ready(robot)
        report["before_joints_rad"] = before_joints
        report["before_tcp_mm_rad"] = before_tcp
        plan, plan_report = build_plan(
            robot,
            before_joints,
            before_tcp,
            distance_mm=float(profile["distance_mm"]),
            duration_s=float(profile["duration_s"]),
            max_solution_excursion_deg=float(profile["max_solution_excursion_deg"]),
            step_num=args.step_num,
            command_period_s=(args.command_period_ms / 1000.0
                              if args.command_period_ms is not None else None),
        )
        report.update(plan_report)
        report["movement_commands_sent"] = 0
        if args.live_single_axis:
            execution = execute_plan(
                robot,
                plan,
                lpf_cutoff=args.lpf_cutoff,
                step_num=args.step_num,
                command_period_s=(args.command_period_ms / 1000.0
                                  if args.command_period_ms is not None else None),
            )
            report.update(execution)
            report["movement_commands_sent"] = execution["send_count"]
        after_joints = six_finite(
            checked(robot.get_actual_joint_position(), "get_actual_joint_position(after)"), "结束关节"
        )
        after_tcp = six_finite(
            checked(robot.get_actual_tcp_position(), "get_actual_tcp_position(after)"), "结束 TCP"
        )
        report["after_joints_rad"] = after_joints
        report["after_tcp_mm_rad"] = after_tcp
        report["net_tcp_translation_mm"] = math.dist(before_tcp[:3], after_tcp[:3])
        report["net_joint_change_deg"] = max(
            math.degrees(abs(a - b)) for a, b in zip(before_joints, after_joints, strict=True)
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
    prefix = ("single_axis" if args.live_single_axis else "plan_only") + "_" + args.profile
    output = REPORT_DIRECTORY / (datetime.now().strftime(prefix + "_%Y%m%d_%H%M%S") + ".json")
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告：{output}")
    if args.live_single_axis:
        return 0 if report.get("failure") is None and report.get("stop_confirmed") is True else 2
    return 0 if report.get("failure") is None and report.get("movement_commands_sent") == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
