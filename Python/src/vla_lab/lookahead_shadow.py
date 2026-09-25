"""连续跟随的有限前瞻候选生成器；纯几何、离线、零机器人命令。

本模块只把已经观测到的手柄目标整理成受限候选点，并标出急转弯。
它不导入 JAKA SDK，不做逆解，不声称解决奇异点、碰撞或控制器平滑过渡。
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ShadowSettings:
    period_ms: int = 50
    max_chord_mm: float = 8.0
    deadband_mm: float = 0.5
    stop_turn_deg: float = 60.0
    window_points: int = 3

    def __post_init__(self):
        if not 20 <= self.period_ms <= 200:
            raise ValueError("period_ms 必须在20—200ms")
        if not 1 <= self.max_chord_mm <= 10:
            raise ValueError("max_chord_mm 必须在1—10mm")
        if not 0 <= self.deadband_mm < self.max_chord_mm:
            raise ValueError("deadband_mm 必须非负且小于空间间距上限")
        if not 15 <= self.stop_turn_deg <= 120:
            raise ValueError("stop_turn_deg 必须在15—120°")
        if self.window_points != 3:
            raise ValueError("当前审查器只允许最近3点前瞻")


def _distance(a, b):
    return math.dist(a, b)


def _turn_deg(a, b, c):
    first = tuple(b[i] - a[i] for i in range(3))
    second = tuple(c[i] - b[i] for i in range(3))
    an, bn = math.hypot(*first), math.hypot(*second)
    if an < 1e-9 or bn < 1e-9:
        return 0.0
    cosine = max(-1.0, min(1.0, sum(x*y for x, y in zip(first, second)) / (an*bn)))
    return math.degrees(math.acos(cosine))


def time_sample_latest(points, period_ms):
    """固定时刻仅取当时已经收到的最新观测，不向未来取点或外推。"""
    if len(points) < 2:
        raise ValueError("至少需要2个观测点")
    period_ns = int(period_ms * 1_000_000)
    sampled, index = [], 0
    first, last = points[0][0], points[-1][0]
    for stamp in range(first, last + 1, period_ns):
        while index + 1 < len(points) and points[index + 1][0] <= stamp:
            index += 1
        sampled.append((stamp, points[index][1], points[index][0]))
    return sampled


def bounded_candidates(points, settings=ShadowSettings()):
    """生成带空间上限的候选点。

    两个时间采样点相距过大时，在它们之间补几何检查点。补点时间设为后一个
    已观测时刻，因此它们只能用于事后/影子检查，不能冒充更早收到的实时输入。
    """
    sampled = time_sample_latest(points, settings.period_ms)
    result = []
    for sample_stamp, xyz, observed_stamp in sampled:
        xyz = tuple(float(v) for v in xyz)
        if not result:
            result.append((sample_stamp, xyz, observed_stamp, False))
            continue
        previous = result[-1][1]
        distance = _distance(previous, xyz)
        if distance < settings.deadband_mm:
            continue
        pieces = max(1, math.ceil(distance / settings.max_chord_mm))
        for index in range(1, pieces + 1):
            point = tuple(previous[i] + (xyz[i] - previous[i]) * index / pieces for i in range(3))
            result.append((sample_stamp, point, observed_stamp, pieces > 1))
    if len(result) < 3:
        raise ValueError("去死区后不足3个候选点")
    return result


def diagnose_windows(candidates, settings=ShadowSettings()):
    """逐个最近三点窗口分类；窗口不会累积成无限待执行队列。"""
    windows = []
    for index in range(2, len(candidates)):
        triple = candidates[index-2:index+1]
        turn = _turn_deg(*(item[1] for item in triple))
        windows.append({
            "latest_index": index,
            "point_count": settings.window_points,
            "turn_deg": round(turn, 3),
            "transition": "exact_stop" if turn >= settings.stop_turn_deg else "blend_candidate",
            "max_input_age_ms": round(max(item[0] - item[2] for item in triple) / 1e6, 3),
            "contains_spatial_subdivision": any(item[3] for item in triple),
        })
    chords = [_distance(a[1], b[1]) for a, b in zip(candidates, candidates[1:])]
    return {
        "candidate_points": len(candidates),
        "window_count": len(windows),
        "max_chord_mm": round(max(chords), 6),
        "candidate_polyline_mm": round(sum(chords), 3),
        "blend_candidate_windows": sum(w["transition"] == "blend_candidate" for w in windows),
        "exact_stop_windows": sum(w["transition"] == "exact_stop" for w in windows),
        "max_turn_deg": round(max(w["turn_deg"] for w in windows), 3),
        "max_input_age_ms": round(max(w["max_input_age_ms"] for w in windows), 3),
        "windows": windows,
    }
