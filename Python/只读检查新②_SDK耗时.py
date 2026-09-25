"""新②上机前的只读 SDK 耗时检查；绝不发运动、上电或使能命令。

直接在 PyCharm 点运行只显示说明，不会连接机器人。只有显式传入
``--live-readonly`` 才会登录已指定控制器，读取 Tool、关节角、TCP 并对
当前 TCP 调用 JAKA 自带 ``kine_inverse``。调用逆解只做计算，不发送结果。

使用前须确保机器人静止、无人处于危险区域，且没有其它程序占用控制器会话。
如果 SDK 调用阻塞，本进程也可能等待；此脚本不能替代现场急停或安全检查。
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


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SDK_DIRECTORY = Path(os.environ.get("QUEST_JAKA_SDK_DIR", "__SET_QUEST_JAKA_SDK_DIR__"))
REPORT_DIRECTORY = PROJECT_ROOT / "Validation" / "sdk_readonly_preflight"


def checked(result: object, label: str):
    if not isinstance(result, tuple) or len(result) < 2 or result[0] != 0:
        raise RuntimeError(f"{label} 失败：{result!r}")
    return result[1]


def measure(method, *args):
    started = time.perf_counter()
    result = method(*args)
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    return result, elapsed_ms


def summary(values):
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "median_ms": statistics.median(ordered),
        "p99_ms": ordered[math.ceil(0.99 * len(ordered)) - 1],
        "max_ms": ordered[-1],
        "over_8ms": sum(value > 8.0 for value in ordered),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-readonly", action="store_true", help="显式允许只读登录与厂商逆解计时")
    parser.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", "127.0.0.1"))
    parser.add_argument("--sdk-dir", type=Path, default=SDK_DIRECTORY)
    parser.add_argument("--samples", type=int, default=50)
    args = parser.parse_args(argv)
    if not args.live_readonly:
        print("默认仅说明，不连接机器人。若现场已准备好，添加 --live-readonly 再运行。")
        return 0
    if not 1 <= args.samples <= 200:
        parser.error("--samples 必须为 1–200")
    sdk_directory = args.sdk_dir.resolve()
    if not sdk_directory.is_dir():
        parser.error(f"未找到 SDK：{sdk_directory}")
    if os.name == "nt":
        os.add_dll_directory(str(sdk_directory))
    sys.path.insert(0, str(sdk_directory))
    import jkrc  # 只在显式 --live-readonly 时导入；不调用任何运动函数。

    robot = jkrc.RC(args.host)
    durations = {"get_actual_joint_position": [], "get_actual_tcp_position": [], "kine_inverse": []}
    largest_inverse_delta_deg = 0.0
    logged_in = False
    try:
        result = robot.login()
        if not isinstance(result, tuple) or not result or result[0] != 0:
            raise RuntimeError(f"JAKA login 失败：{result!r}")
        logged_in = True
        tool_id = checked(robot.get_tool_id(), "get_tool_id")
        if int(tool_id) != 1:
            raise RuntimeError(f"当前 Tool={tool_id}，不是灵巧手 Tool 1；未进行逆解检查")
        for _ in range(args.samples):
            result, elapsed = measure(robot.get_actual_joint_position)
            joints = tuple(float(value) for value in checked(result, "get_actual_joint_position"))
            durations["get_actual_joint_position"].append(elapsed)
            result, elapsed = measure(robot.get_actual_tcp_position)
            pose = tuple(float(value) for value in checked(result, "get_actual_tcp_position"))
            durations["get_actual_tcp_position"].append(elapsed)
            if len(joints) != 6 or len(pose) != 6 or not all(math.isfinite(x) for x in joints + pose):
                raise RuntimeError("SDK 反馈不是有限的六维值")
            result, elapsed = measure(robot.kine_inverse, joints, pose)
            solved = tuple(float(value) for value in checked(result, "kine_inverse"))
            durations["kine_inverse"].append(elapsed)
            if len(solved) != 6 or not all(math.isfinite(x) for x in solved):
                raise RuntimeError("逆解不是有限的六维值")
            largest_inverse_delta_deg = max(
                largest_inverse_delta_deg,
                *(math.degrees(abs(a - b)) for a, b in zip(solved, joints, strict=True)),
            )
    finally:
        if logged_in:
            try:
                robot.logout()
            except Exception as error:
                print(f"警告：SDK logout 未确认：{error}", file=sys.stderr)

    report = {
        "schema": "jaka_vendor_ik_readonly_preflight.v1",
        "created_at": datetime.now().astimezone().isoformat(),
        "host": args.host,
        "sdk_directory": str(sdk_directory),
        "tool_id": int(tool_id),
        "samples": args.samples,
        "methods": {name: summary(values) for name, values in durations.items()},
        "largest_same_pose_inverse_joint_difference_deg": largest_inverse_delta_deg,
        "movement_commands_sent": 0,
        "interpretation": "只读测量，不证明 servo_j 节拍、滤波器或物理停机安全",
    }
    REPORT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    report_path = REPORT_DIRECTORY / (datetime.now().strftime("sdk_readonly_%Y%m%d_%H%M%S") + ".json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告：{report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
