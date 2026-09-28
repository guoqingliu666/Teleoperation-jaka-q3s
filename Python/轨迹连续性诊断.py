"""直接在 PyCharm 运行：离线回放最新遥操作日志，不连接机械臂。

输出 C2 参考连续性、轨迹压缩率、旧队列拒绝/停顿证据；不把它当作真机验收。
可指定 --replay 某个.jsonl。旧日志只有已限幅 TCP，不能恢复被45°限制截掉的
原始转圈意图，因此完整360°另外由自动化单元测试验证。
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Python" / "src"))
from vla_lab.trajectory_reference import (
    PoseSample,
    RotationHistory,
    C2Reference,
    StreamingReference,
    adaptive_knots,
    angle_deg,
    slerp,
)
from vla_lab.adaptive_replay import compare_sampling


def analyze_raw_capture(events, path):
    """复现现场原始窗口；只读日志中的原始时间与四元数，不重新编造输入。"""
    stream = StreamingReference()
    epoch = None
    spans = []
    raw_count = windows = reductions = 0
    errors = []
    continuity = {
        "velocity_mm_s": 0.0,
        "acceleration_mm_s2": 0.0,
        "angular_velocity_rad_s": 0.0,
        "angular_acceleration_rad_s2": 0.0,
    }
    previous = None

    def accept(span):
        nonlocal previous
        if span is None:
            return
        spans.append(span)
        if previous is not None:
            a, b = previous.evaluate(previous.right.t), span.evaluate(span.left.t)
            for key, x, y in (
                ("velocity_mm_s", a.velocity, b.velocity),
                ("acceleration_mm_s2", a.acceleration, b.acceleration),
                ("angular_velocity_rad_s", a.angular_velocity, b.angular_velocity),
                (
                    "angular_acceleration_rad_s2",
                    a.angular_acceleration,
                    b.angular_acceleration,
                ),
            ):
                continuity[key] = max(continuity[key], math.dist(x, y))
        previous = span

    def finish():
        nonlocal reductions, previous
        if not stream.finished:
            accept(stream.finish())  # 仅离线验证末端；松Grip时真机绝不补追这段。
        reductions += stream.tangent_reductions
        stream.reset()
        previous = None

    for e in events:
        if e.get("state") != "trajectory_raw_sample":
            continue
        if e["epoch"] != epoch:
            finish()
            epoch = e["epoch"]
            windows += 1
        raw_count += 1
        try:
            accept(stream.push(PoseSample(e["t"], tuple(e["xyz"]), tuple(e["q"]))))
        except ValueError as error:
            errors.append({"epoch": epoch, "t": e["t"], "error": str(error)})
            stream.finished = True  # 同一窗口不能失败后偷偷重捕获。
    finish()
    bounds = [s.bounds() for s in spans]
    return {
        "schema": "jaka_raw_stream_replay.v1",
        "source": str(path.resolve()),
        "adaptive_sampling_comparison": compare_sampling(events),
        "raw_samples": raw_count,
        "windows": windows,
        "reference_spans": len(spans),
        "shared_tangent_reductions": reductions,
        "errors": errors,
        "analytic_knot_continuity_errors": continuity,
        "conservative_reference_bounds": (
            {k: max(b[k] for b in bounds) for k in bounds[0]} if bounds else {}
        ),
        "robot_connected": False,
        "movement_commands_sent": 0,
        "physical_C2_verified": False,
        "limitations": [
            "只验证捕获到的原始片段；原检查阻断后的手柄轨迹未被记录",
            "没有调用逆解或厂商运动规划；没有证明实机关节速度/加速度",
            "导数缩放保持几何预算，不代表全程恒速或无减速",
        ],
    }


def analyze(path):
    """按日志顺序分开 Grip/断流窗口；绝不排序拼接互不连续的运动。"""
    events = []
    with path.open(encoding="utf-8-sig") as file:
        for number, line in enumerate(file, 1):
            if line.strip():
                try:
                    event = json.loads(line)
                except ValueError as error:
                    raise ValueError(f"日志第{number}行无效：{error}") from error
                if not isinstance(event, dict):
                    raise ValueError(f"日志第{number}行不是事件对象")
                events.append(event)
    if any(e.get("state") == "trajectory_raw_sample" for e in events):
        return analyze_raw_capture(events, path)
    counts = Counter(e.get("state", "unknown") for e in events)
    strokes, current = [], []
    history = RotationHistory()
    invalid_boundaries = duplicate_timestamps = 0
    t0 = next((e["time_ns"] for e in events if "time_ns" in e), 0)

    def finish():
        nonlocal current
        if len(current) >= 2:
            strokes.append(current)
        current = []
        history.reset()

    for event in events:
        if event.get("state") in (
            "paused",
            "stopping",
            "finished",
            "fault",
            "pose_blocked",
        ):
            finish()
        if event.get("state") == "display_binding":
            finish()
        if event.get("state") != "target" or event.get("target") is None:
            continue
        p = PoseSample.from_tcp((event["time_ns"] - t0) / 1e9, event["target"])
        if history.previous and p.t == history.previous.t:
            # 历史 time.time_ns 的分辨率会使两帧共用时间戳；无法推断速度，分开处理。
            duplicate_timestamps += 1
            finish()
        try:
            p = history.push(p)
        except ValueError:
            invalid_boundaries += 1
            finish()
            p = history.push(p)
        current.append(p)
    finish()
    largest_errors = {}
    raw_count = knot_count = span_count = 0
    sampled_max_speed = sampled_max_acceleration = 0.0
    max_chord_deviation = max_chord_rotation = 0.0
    tangent_reductions = 0
    reference_time_scale = 1.0
    conservative_bounds = {}
    for stroke in strokes:
        raw_count += len(stroke)
        knots = adaptive_knots(stroke)
        knot_count += len(knots)
        curve = C2Reference(knots)
        tangent_reductions += curve.tangent_reductions
        reference_time_scale = max(
            reference_time_scale,
            curve.required_time_scale(
                speed_mm_s=200.0,
                acceleration_mm_s2=500.0,
                angular_speed_deg_s=45.0,
                angular_acceleration_deg_s2=180.0,
            ),
        )
        for key, value in curve.derivative_bounds().items():
            conservative_bounds[key] = max(conservative_bounds.get(key, 0.0), value)
        span_count += len(curve.spans)
        for key, value in curve.continuity_errors().items():
            largest_errors[key] = max(largest_errors.get(key, 0.0), value)
        # 仅采样诊断，不把有限采样当作全轨迹包络的数学证明。
        for span in curve.spans:
            for i in range(21):
                u = i / 20
                s = span.evaluate(span.left.t + span.h * u)
                chord = tuple(
                    a + (b - a) * u for a, b in zip(span.left.xyz, span.right.xyz)
                )
                max_chord_deviation = max(
                    max_chord_deviation, math.dist(s.pose.xyz, chord)
                )
                max_chord_rotation = max(
                    max_chord_rotation,
                    angle_deg(s.pose.q, slerp(span.left.q, span.right.q, u)),
                )
                sampled_max_speed = max(sampled_max_speed, math.hypot(*s.velocity))
                sampled_max_acceleration = max(
                    sampled_max_acceleration, math.hypot(*s.acceleration)
                )
    commands = [e for e in events if e.get("state") == "rolling_pose_command"]
    segments = sorted(
        float(e["translation_mm"]) for e in commands if "translation_mm" in e
    )
    final = next((e for e in reversed(events) if e.get("state") == "finished"), {})
    return {
        "schema": "jaka_reference_replay.v1",
        "source": str(path.resolve()),
        "robot_connected": False,
        "movement_commands_sent": 0,
        "reference_only": True,
        "physical_C2_verified": False,
        "continuous_windows": len(strokes),
        "raw_samples": raw_count,
        "adaptive_knots": knot_count,
        "reference_spans": span_count,
        "retained_fraction": knot_count / max(1, raw_count),
        "invalid_input_boundaries": invalid_boundaries,
        "duplicate_timestamps": duplicate_timestamps,
        "analytic_knot_continuity_errors": largest_errors,
        "conservative_reference_bounds": conservative_bounds,
        "shared_tangent_reductions": tangent_reductions,
        "finite_window_uniform_time_scale": reference_time_scale,
        "time_scale_applied_to_robot": False,
        "sampled_reference_max_speed_mm_s": sampled_max_speed,
        "sampled_reference_max_acceleration_mm_s2": sampled_max_acceleration,
        "sampled_same_time_chord_error_mm": max_chord_deviation,
        "sampled_same_time_chord_error_deg": max_chord_rotation,
        "old_command_count": len(commands),
        "old_median_segment_mm": segments[len(segments) // 2] if segments else None,
        "old_rejection_reasons": dict(
            Counter(
                e.get("message", "")
                for e in events
                if e.get("state") == "rolling_pose_target_rejected"
            )
        ),
        "old_logged_starvation": counts["rolling_pose_queue_starved"],
        "old_logged_drained": counts["rolling_pose_queue_drained"],
        "old_final_failure": final.get("failure"),
        "limitations": [
            "参考曲线C2不代表控制柜实际曲线C2；没有测得实机关节速度/加速度",
            "原始手柄时间尺度可能超出机器人速度/加速度；该报告不输出运动指令",
            "旧target日志已被姿态限幅，不能用它验证完整转圈，也不能证明奇异点绕行",
            "旧queue_starved不是全部断供次数；未记录的耗尽不能当作0",
            "same_time_chord_error包含时间进度差；几何偏差看conservative_reference_bounds",
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    source = args.replay
    if source is None:
        files = [
            p
            for p in (ROOT / "Validation" / "continuous_sampled_follow").glob(
                "session_*.jsonl"
            )
            if not p.name.endswith(".sdk.jsonl")
        ]
        if not files:
            parser.error("没有可回放日志；请使用 --replay 指定会话JSONL")
        source = max(files, key=lambda p: p.stat().st_mtime_ns)
    output = (
        args.output
        or ROOT
        / "Validation"
        / "trajectory_reference"
        / datetime.now().strftime("replay_%Y%m%d_%H%M%S_%f.json")
    )
    if output.resolve().drive.upper() != "D:":
        parser.error("报告必须写入D盘")
    if source.resolve() == output.resolve():
        parser.error("报告不能覆盖输入日志")
    report = analyze(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    # 不覆盖既有报告，保留每次检查证据。
    with output.open("x", encoding="utf-8") as file:
        json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    print(f"报告：{output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
