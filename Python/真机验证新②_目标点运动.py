"""新②目标点运动门槛：只给 JAKA 一个终点，不逐帧复刻手柄轨迹。

设计目的
========
连续遥操作要求 Quest、逆解、Python、网络和控制器在每个 8 ms 周期同步，任何一环
偶发变慢都会影响轨迹。本脚本验证另一条更适合展示级动作的路线：

1. 读取当前 Tool 1 TCP 和关节角；
2. 只用一次 JAKA 厂商 ``kine_inverse`` 检查终点是否存在同分支解；
3. 真机模式只发送一次 JAKA ``linear_move``，速度规划交给控制器；
4. Python 只轮询到位、故障和超时，不持续发送中间目标。

默认运行不连接机器人。``--live-plan-only`` 只读规划且绝不发送运动；
``--live-target`` 才允许执行锁定档位。脚本不负责上电、使能、清报警或切换 Tool。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SDK_DIRECTORY = Path(os.environ.get("QUEST_JAKA_SDK_DIR", "__SET_QUEST_JAKA_SDK_DIR__"))
REPORT_DIRECTORY = PROJECT_ROOT / "Validation" / "sdk_target_point"
TOOL_ID_REQUIRED = 1
MODE_ABS = 0
FINAL_TRANSLATION_TOLERANCE_MM = 1.0
POLL_PERIOD_S = 0.05

# 20 mm 单轴往返已经完成现场验收，因此开放下一档 50 mm 单轴目标。
# 50 mm 仍保持 10 mm/s，并把厂商逆解相对当前关节的最大变化限制在 12°；
# 手柄任意终点仍不在这里隐式开放，必须完成 50 mm 往返后再单独验收。
PROFILES = {
    "z-plus-20mm": {
        "delta_xyz_mm": (0.0, 0.0, 20.0),
        "delta_rpy_deg": (0.0, 0.0, 0.0),
        "speed_mm_s": 10.0,
        "max_joint_delta_deg": 6.0,
        "confirmation": "基坐标Z正向20毫米目标点运动",
    },
    "z-minus-20mm": {
        "delta_xyz_mm": (0.0, 0.0, -20.0),
        "delta_rpy_deg": (0.0, 0.0, 0.0),
        "speed_mm_s": 10.0,
        "max_joint_delta_deg": 6.0,
        "confirmation": "基坐标Z负向20毫米目标点运动",
    },
    "z-plus-50mm": {
        "delta_xyz_mm": (0.0, 0.0, 50.0),
        "delta_rpy_deg": (0.0, 0.0, 0.0),
        "speed_mm_s": 10.0,
        "max_joint_delta_deg": 12.0,
        "confirmation": "基坐标Z正向50毫米目标点运动",
    },
    "z-minus-50mm": {
        "delta_xyz_mm": (0.0, 0.0, -50.0),
        "delta_rpy_deg": (0.0, 0.0, 0.0),
        "speed_mm_s": 10.0,
        "max_joint_delta_deg": 12.0,
        "confirmation": "基坐标Z负向50毫米目标点运动",
    },
}


def checked(result: object, name: str):
    """检查 JAKA Python SDK 的 ``(错误码, 数据...)`` 返回格式。"""
    if not isinstance(result, tuple) or not result or type(result[0]) is not int or result[0] != 0:
        raise RuntimeError(f"{name} 失败：{result!r}")
    return result[1] if len(result) > 1 else None


def checked_bool(result: object, name: str) -> bool:
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


def require_ready(robot) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """只读门槛：不替操作者上电、使能、清报警或切换工具。"""
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
        raise RuntimeError("控制器仍在伺服模式；目标点运动与伺服不能同时运行")
    tool_id = int(checked(robot.get_tool_id(), "get_tool_id"))
    if tool_id != TOOL_ID_REQUIRED:
        raise RuntimeError(f"当前 Tool={tool_id}，不是 Tool {TOOL_ID_REQUIRED}")
    joints = six_finite(checked(robot.get_actual_joint_position(), "get_actual_joint_position"), "当前关节")
    tcp = six_finite(checked(robot.get_actual_tcp_position(), "get_actual_tcp_position"), "当前 TCP")
    return joints, tcp


def build_target(robot, joints, tcp, profile):
    """生成一个终点并只调用一次厂商逆解，用于可达性与分支变化检查。"""
    delta_xyz = tuple(float(value) for value in profile["delta_xyz_mm"])
    delta_rpy = tuple(math.radians(float(value)) for value in profile["delta_rpy_deg"])
    target = tuple(tcp[i] + delta_xyz[i] for i in range(3)) + tuple(
        tcp[3 + i] + delta_rpy[i] for i in range(3)
    )
    solved = six_finite(checked(robot.kine_inverse(joints, target), "kine_inverse"), "终点厂商逆解")
    joint_delta_deg = tuple(math.degrees(abs(a - b)) for a, b in zip(solved, joints, strict=True))
    largest = max(joint_delta_deg)
    if largest > float(profile["max_joint_delta_deg"]):
        raise RuntimeError(
            f"终点厂商逆解最大关节变化 {largest:.3f}° 超过档位门槛 "
            f"{float(profile['max_joint_delta_deg']):.1f}°"
        )
    return target, solved, joint_delta_deg


def execute_target(robot, target, *, speed_mm_s: float, expected_distance_mm: float, clock=time.perf_counter,
                   sleeper=time.sleep, progress_callback=None) -> dict[str, object]:
    """发送一次厂商直线运动，并轮询到位；异常时请求 JAKA ``motion_abort``。

    ``progress_callback`` 只用于把同一 SDK 连接读到的实测关节角送给数字孪生。
    它不能产生运动命令；显示链路异常也不能改变厂商规划运动的执行结果。
    """
    if not 1.0 <= speed_mm_s <= 30.0:
        raise ValueError("第一阶段目标点速度只允许 1—30 mm/s")
    timeout_s = max(5.0, expected_distance_mm / speed_mm_s * 3.0 + 2.0)
    command_sent = False
    abort_sent = False
    display_feedback_samples = 0
    display_feedback_errors = 0
    started = clock()
    try:
        checked(robot.linear_move(target, MODE_ABS, False, speed_mm_s), "linear_move(target point)")
        command_sent = True
        while True:
            if checked_bool(robot.is_in_estop(), "is_in_estop"):
                raise RuntimeError("运动中检测到急停")
            if checked_bool(robot.is_in_collision(), "is_in_collision"):
                raise RuntimeError("运动中检测到碰撞保护")
            if checked_bool(robot.is_on_limit(), "is_on_limit"):
                raise RuntimeError("运动中检测到限位")
            # 运动期间，另一个只读 JAKA 会话可能暂时拿不到反馈。使用当前控制会话
            # 读取实测关节角并交给显示端，避免 Unity 在到位数秒后才跳到终点。
            # 此回调严格位于安全状态检查之后；它的任何异常都只计数，不影响运动。
            if progress_callback is not None:
                try:
                    progress_callback()
                    display_feedback_samples += 1
                except Exception:
                    display_feedback_errors += 1
            if checked_bool(robot.is_in_pos(), "is_in_pos"):
                break
            if clock() - started > timeout_s:
                raise RuntimeError(f"目标点运动 {timeout_s:.1f} 秒仍未到位")
            sleeper(POLL_PERIOD_S)
    except Exception:
        if command_sent:
            try:
                checked(robot.motion_abort(), "motion_abort")
                abort_sent = True
            except Exception:
                pass
        raise
    return {
        "movement_commands_sent": 1,
        "motion_abort_sent": abort_sent,
        "elapsed_s": clock() - started,
        "timeout_s": timeout_s,
        "display_feedback_samples": display_feedback_samples,
        "display_feedback_errors": display_feedback_errors,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--live-plan-only", action="store_true", help="连接并只读规划，不发送运动")
    mode.add_argument("--live-target", action="store_true", help="执行锁定的目标点运动")
    parser.add_argument("--profile", choices=tuple(PROFILES), default="z-plus-20mm")
    parser.add_argument("--confirm", default="", help="真机运动必须填写档位对应的确认短语")
    parser.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", "127.0.0.1"))
    parser.add_argument("--sdk-dir", type=Path, default=SDK_DIRECTORY)
    args = parser.parse_args(argv)
    if not args.live_plan_only and not args.live_target:
        print("默认不连接。先用 --live-plan-only；通过并确认现场后才可使用 --live-target。")
        return 0
    profile = PROFILES[args.profile]
    if args.live_target and args.confirm != profile["confirmation"]:
        parser.error(f"--confirm 必须精确填写：{profile['confirmation']}")
    sdk_dir = args.sdk_dir.resolve()
    if not sdk_dir.is_dir():
        parser.error(f"未找到 SDK：{sdk_dir}")
    if os.name == "nt":
        os.add_dll_directory(str(sdk_dir))
    sys.path.insert(0, str(sdk_dir))
    import jkrc

    report: dict[str, object] = {
        "schema": "jaka_vendor_planned_target_point.v1",
        "created_at": datetime.now().astimezone().isoformat(),
        "host": args.host,
        "mode": "live-target" if args.live_target else "live-plan-only",
        "profile": args.profile,
        "tool_id_required": TOOL_ID_REQUIRED,
        "trajectory_source": "JAKA linear_move planner; Quest trajectory is not replayed",
        "movement_commands_sent": 0,
    }
    robot = jkrc.RC(args.host)
    logged_in = False
    try:
        checked(robot.login(), "login")
        logged_in = True
        joints, tcp = require_ready(robot)
        target, solved, joint_delta_deg = build_target(robot, joints, tcp, profile)
        report.update({
            "before_joints_rad": joints,
            "before_tcp_mm_rad": tcp,
            "target_tcp_mm_rad": target,
            "target_joints_rad": solved,
            "joint_delta_deg": joint_delta_deg,
            "max_joint_delta_deg": max(joint_delta_deg),
            "speed_mm_s": profile["speed_mm_s"],
        })
        if args.live_target:
            distance = math.dist(tcp[:3], target[:3])
            report.update(execute_target(
                robot,
                target,
                speed_mm_s=float(profile["speed_mm_s"]),
                expected_distance_mm=distance,
            ))
        after_tcp = six_finite(checked(robot.get_actual_tcp_position(), "get_actual_tcp_position(after)"), "结束 TCP")
        report["after_tcp_mm_rad"] = after_tcp
        report["target_translation_error_mm"] = math.dist(after_tcp[:3], target[:3])
        if args.live_target and report["target_translation_error_mm"] > FINAL_TRANSLATION_TOLERANCE_MM:
            raise RuntimeError(
                f"到位后 TCP 误差 {report['target_translation_error_mm']:.3f} mm 超过 "
                f"{FINAL_TRANSLATION_TOLERANCE_MM:.1f} mm"
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
    prefix = ("target" if args.live_target else "plan") + "_" + args.profile
    output = REPORT_DIRECTORY / (datetime.now().strftime(prefix + "_%Y%m%d_%H%M%S") + ".json")
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告：{output}")
    return 0 if report.get("failure") is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
