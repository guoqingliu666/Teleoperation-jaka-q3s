"""仅分析已记录的Quest目标；不导入SDK、不连接机器人、不生成可执行运动指令。

本程序回答两个问题：固定时间间隔会留下多远的空间空隙？转角有多急？
固定时间采样只是候选点选择，不等于轨迹速度连续，也不构成真机放行。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics


def load_targets(path: Path):
    """只读取本项目日志的target事件，保留记录顺序；拒绝时间倒退/非有限坐标。"""
    points = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        event = json.loads(line)
        if event.get("state") != "target":
            continue
        stamp = event.get("time_ns")
        target = event.get("target")
        if not isinstance(stamp, int) or not isinstance(target, list) or len(target) != 6:
            raise ValueError(f"第{number}行目标格式错误")
        xyz = tuple(float(x) for x in target[:3])
        if not all(math.isfinite(x) for x in xyz):
            raise ValueError(f"第{number}行坐标非有限数")
        if points and stamp <= points[-1][0]:
            raise ValueError(f"第{number}行时间未严格递增")
        points.append((stamp, xyz))
    if len(points) < 3:
        raise ValueError("至少需要3个目标事件才能评估采样与转角")
    return points


def time_sample(points, period_ms):
    """每到固定时间刻度取当时最新观测值，不向未来外推，也不插值造点。"""
    if not 20 <= period_ms <= 200:
        raise ValueError("仅允许分析20—200ms的采样周期")
    period_ns = round(period_ms * 1_000_000)
    first, last = points[0][0], points[-1][0]
    sampled = []
    index = 0
    for stamp in range(first, last + 1, period_ns):
        while index + 1 < len(points) and points[index + 1][0] <= stamp:
            index += 1
        sampled.append((stamp, points[index][1], stamp - points[index][0]))
    return sampled


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(len(ordered) * fraction) - 1)]


def diagnose(points, period_ms):
    sampled = time_sample(points, period_ms)
    displacements = [tuple(b[1][i] - a[1][i] for i in range(3))
                     for a, b in zip(sampled, sampled[1:])]
    chords = [math.hypot(*delta) for delta in displacements]
    turns = []
    for a, b in zip(displacements, displacements[1:]):
        an, bn = math.hypot(*a), math.hypot(*b)
        if an < .5 or bn < .5:  # 近乎静止时方向由噪声支配，不当作真实转弯。
            continue
        cosine = max(-1.0, min(1.0, sum(x*y for x, y in zip(a, b)) / (an*bn)))
        turns.append(math.degrees(math.acos(cosine)))
    return {
        "period_ms": period_ms,
        "raw_target_events": len(points),
        "sampled_points": len(sampled),
        "duration_s": round((points[-1][0] - points[0][0]) / 1e9, 3),
        "median_chord_mm": round(statistics.median(chords), 3),
        "p95_chord_mm": round(percentile(chords, .95), 3),
        "max_chord_mm": round(max(chords), 3),
        "polyline_length_mm": round(sum(chords), 3),
        "max_apparent_target_speed_mm_s": round(max(chords) * 1000 / period_ms, 1),
        "turns_over_60deg": sum(angle > 60 for angle in turns),
        "reversals_over_120deg": sum(angle > 120 for angle in turns),
        "max_sample_age_ms": round(max(s[2] for s in sampled) / 1e6, 3),
        "interpretation": "离线候选点诊断；未验证碰撞、逆解、关节速度、控制器过渡或停止距离",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="现有会话JSONL；只读")
    parser.add_argument("--period-ms", type=int, nargs="+", default=[33, 50, 100])
    args = parser.parse_args(argv)
    points = load_targets(args.input)
    print(json.dumps({"schema": "quest_time_sampling_offline.v1",
                      "input": str(args.input),
                      "movement_commands_sent": 0,
                      "results": [diagnose(points, p) for p in args.period_ms]},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
