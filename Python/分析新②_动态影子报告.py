"""离线分析新②动态影子 JSONL；不导入 JAKA SDK，也不连接任何设备。"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Python" / "src"))

from vla_lab.quest_vr_input import matmul, rotation_angle_rad, rpy_matrix, transpose


REPORT_DIR = ROOT / "Validation" / "sdk_dynamic_shadow"


def rotation_step_deg(first, second) -> float:
    relative = matmul(transpose(rpy_matrix(tuple(first))), rpy_matrix(tuple(second)))
    return math.degrees(rotation_angle_rad(relative))


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[math.ceil(fraction * len(ordered)) - 1]


def find_latest_jsonl() -> Path:
    candidates = sorted(REPORT_DIR.glob("sdk_shadow_*.jsonl"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"未找到动态影子逐帧记录：{REPORT_DIR}")
    return candidates[-1]


def analyse(path: Path) -> dict[str, object]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    solved = [row for row in rows if "solution_joints_rad" in row]
    if len(solved) < 3:
        raise ValueError("至少需要 3 个连续逆解帧")
    times = [datetime.fromisoformat(row["at"]).timestamp() for row in solved]
    intervals_ms: list[float] = []
    target_position_steps: list[float] = []
    target_rotation_steps: list[float] = []
    joint_steps: list[list[float]] = []
    joint_velocities: list[list[float]] = []
    peak_step: dict[str, object] | None = None
    for index in range(1, len(solved)):
        previous, current = solved[index - 1], solved[index]
        dt = times[index] - times[index - 1]
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError(f"第 {index} 对帧时间戳无效")
        steps = [
            math.degrees(float(b) - float(a))
            for a, b in zip(previous["solution_joints_rad"], current["solution_joints_rad"], strict=True)
        ]
        velocities = [step / dt for step in steps]
        position_step = math.dist(previous["target_tcp"][:3], current["target_tcp"][:3])
        rotation_step = rotation_step_deg(previous["target_tcp"][3:], current["target_tcp"][3:])
        intervals_ms.append(dt * 1000.0)
        joint_steps.append(steps)
        joint_velocities.append(velocities)
        target_position_steps.append(position_step)
        target_rotation_steps.append(rotation_step)
        magnitude = max(abs(value) for value in steps)
        if peak_step is None or magnitude > float(peak_step["max_joint_step_deg"]):
            peak_step = {
                "from": previous["at"],
                "to": current["at"],
                "dt_ms": dt * 1000.0,
                "joint_step_deg": steps,
                "joint_velocity_deg_s": velocities,
                "max_joint_step_deg": magnitude,
                "target_position_step_mm": position_step,
                "target_rotation_step_deg": rotation_step,
            }
    joint_accelerations: list[list[float]] = []
    for index in range(1, len(joint_velocities)):
        # 两个离散速度对应相邻区间；使用两区间中心点之间的时间。
        dt = (times[index + 1] - times[index - 1]) / 2.0
        joint_accelerations.append([
            (current - previous) / dt
            for previous, current in zip(joint_velocities[index - 1], joint_velocities[index], strict=True)
        ])
    max_velocity_per_joint = [
        max(abs(values[joint]) for values in joint_velocities) for joint in range(6)
    ]
    max_acceleration_per_joint = [
        max(abs(values[joint]) for values in joint_accelerations) for joint in range(6)
    ]
    input_ages = [float(row["input_age_ms_after_inverse"]) for row in solved]
    inverse_times = [float(row["inverse_ms"]) for row in solved]
    combined_times = [float(row["read_and_inverse_ms"]) for row in solved]
    return {
        "schema": "jaka_vendor_ik_dynamic_shadow_analysis.v1",
        "source": str(path),
        "created_at": datetime.now().astimezone().isoformat(),
        "solved_frames": len(solved),
        "observed_solution_duration_s": times[-1] - times[0],
        "frame_interval_ms": {
            "median": statistics.median(intervals_ms),
            "p99": percentile(intervals_ms, 0.99),
            "max": max(intervals_ms),
        },
        "inverse_ms": {"median": statistics.median(inverse_times), "max": max(inverse_times)},
        "read_and_inverse_ms": {"median": statistics.median(combined_times), "max": max(combined_times)},
        "input_age_ms_after_inverse": {
            "median": statistics.median(input_ages),
            "max": max(input_ages),
            "over_8ms": sum(value > 8.0 for value in input_ages),
        },
        "max_target_position_step_mm": max(target_position_steps),
        "max_target_rotation_step_deg": max(target_rotation_steps),
        "max_joint_velocity_deg_s_by_j1_to_j6": max_velocity_per_joint,
        "max_joint_acceleration_deg_s2_by_j1_to_j6": max_acceleration_per_joint,
        "max_solution_vs_actual_deg": max(float(row["largest_solution_vs_actual_deg"]) for row in solved),
        "peak_solution_step": peak_step,
        "motion_commands_sent": 0,
        "motion_readiness": "not_accepted",
        "reasons": [
            "尚未测量 servo_j 调用、控制器队列、滤波器和物理停止",
            "至少一帧输入年龄超过 8 ms" if any(value > 8.0 for value in input_ages) else "输入年龄本次未超过 8 ms",
            "厂商逆解候选关节速度/加速度尚无本机固件和负载对应的批准门槛",
            "静止机器人未跟随目标，最大候选解与实测关节差不能作为闭环跟踪证明",
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl", nargs="?", type=Path, help="省略时分析最新 sdk_shadow_*.jsonl")
    args = parser.parse_args(argv)
    source = (args.jsonl or find_latest_jsonl()).resolve()
    result = analyse(source)
    output = source.with_name(source.stem + "_分析.json")
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"分析报告：{output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
