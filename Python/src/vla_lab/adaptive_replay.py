"""已录制手柄轨迹的离线采样对照：不导入或连接任何机器人 SDK。

重放能够确定几何合并与初始检查点数。新点没有对应的真实关节解，因此这里
只给出无风险细分情况下的调用量估计；不能把估计写成现场已通过或速度提升。
"""

from dataclasses import asdict
import math

from .adaptive_sampling import AdaptiveSampling, select_keypoint
from .trajectory_reference import PoseSample, angle_deg, ordered_chord_fits
from .trajectory_horizon import interpolate_pose


def compare_sampling(events, *, policy=AdaptiveSampling(), window_s=0.12):
    """每次最多看120ms原始轨迹，在相同几何误差下比较固定/动态检查工作量。

    对照中的2mm/0.2°与5mm/1°都作用于同一组选定线段，避免把轨迹简化收益
    错算成逆解算法收益。实际还会增加风险细分和超时/权限轮询的成本。
    """
    windows = []
    current = []
    epoch = None
    actual_calls = 0
    for event in events:
        if event.get("state") == "trajectory_readonly_finished":
            actual_calls += (
                event.get("sdk_call_timing", {}).get("kine_inverse", {}).get("count", 0)
            )
        if event.get("state") != "trajectory_raw_sample":
            continue
        if event["epoch"] != epoch:
            if current:
                windows.append(current)
            current = []
            epoch = event["epoch"]
        current.append(PoseSample(event["t"], tuple(event["xyz"]), tuple(event["q"])))
    if current:
        windows.append(current)
    segments = fixed_checks = adaptive_checks = merged = 0
    residuals = 0
    # 弧长减小允许来自预算内的小抖动，但不能把整圈首尾重合当作静止。
    raw_arc = retained_arc = 0.0
    geometry_failures = []
    for points in windows:
        raw_arc += sum(angle_deg(a.q, b.q) for a, b in zip(points, points[1:]))
        anchor = points[0]
        pending = points[1:]
        while pending:
            visible = [p for p in pending if p.t - anchor.t <= window_s]
            if not visible:
                visible = [pending[0]]
            target = select_keypoint(
                anchor,
                visible,
                max_translation_mm=80.0,
                max_rotation_deg=4.0,
                policy=policy,
            )
            source = target
            intermediate = [p for p in pending if p.t < source.t]
            if not ordered_chord_fits(
                intermediate,
                anchor,
                source,
                policy.position_error_mm,
                policy.orientation_error_deg,
            ):
                geometry_failures.append({"start_s": anchor.t, "end_s": source.t})
            length = math.dist(anchor.xyz, target.xyz)
            rotation = angle_deg(anchor.q, target.q)
            fraction = min(1.0, 80 / max(length, 1e-12), 4 / max(rotation, 1e-12))
            target = interpolate_pose(anchor, target, fraction)
            length = math.dist(anchor.xyz, target.xyz)
            rotation = angle_deg(anchor.q, target.q)
            fixed_checks += max(1, math.ceil(length / 2), math.ceil(rotation / 0.2))
            adaptive_checks += 2 * max(
                1,
                math.ceil(length / (2 * policy.max_check_mm)),
                math.ceil(rotation / (2 * policy.max_check_deg)),
            )
            segments += 1
            merged += len(intermediate)
            retained_arc += rotation
            pending = [p for p in pending if p.t > source.t]
            if target.t < source.t:
                pending.insert(0, source)
                residuals += 1
            anchor = target
    return {
        "schema": "jaka_adaptive_sampling_geometry_comparison.v1",
        "policy": asdict(policy),
        "lookahead_window_s": window_s,
        "raw_samples": sum(map(len, windows)),
        "windows": len(windows),
        "selected_segments": segments,
        "merged_original_samples": merged,
        "retained_residuals": residuals,
        "geometry_failures": geometry_failures,
        "raw_rotation_arc_deg": raw_arc,
        "retained_rotation_arc_deg": retained_arc,
        "same_segments_fixed_check_count": fixed_checks,
        "same_segments_adaptive_initial_check_count": adaptive_checks,
        "estimated_initial_check_reduction_fraction": 1
        - adaptive_checks / max(1, fixed_checks),
        "original_session_actual_ik_calls": actual_calls,
        "sdk_called": False,
        "movement_commands_sent": 0,
        "limitations": [
            "初始检查数不含风险细分、SDK波动、执行队列等待",
            "新的几何点需要厂商SDK只读复核；日志不能补出未求过的真实关节解",
            "没有模拟控制柜实际圆滑，也没有证明物理速度或加速度连续",
        ],
    }
