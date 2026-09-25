"""只读采样 JAKA ``get_motion_status`` 的 Python 返回结构。

用途是确认当前 SDK 的 ``queue``、``active_queue``、``inpos`` 等字段怎样暴露给
Python，为下一阶段厂商轨迹圆滑验收提供依据。默认不连接；只有显式传入
``--live-readonly`` 才登录控制器。脚本不接收手柄，也不调用任何运动、上电、
使能、清报警、停止或 IO 写接口。
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
REPORT_ROOT = ROOT / "Validation" / "motion_status_readonly"
sys.path.insert(0, str(ROOT / "Python" / "src"))

from vla_lab.jaka_telemetry import SDK_DIRECTORY, ReadOnlySession, load_sdk  # noqa: E402
from vla_lab.motion_status import MotionStatus  # noqa: E402


def json_safe(value):
    """保留 SDK 原始层级；未知扩展对象只记类型和 repr，不猜字段含义。"""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    return {"python_type": type(value).__name__, "repr": repr(value)}


def timing_summary(values: list[float]) -> dict:
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "median_ms": statistics.median(ordered),
        "max_ms": ordered[-1],
    }


def collect(session: ReadOnlySession, samples: int, interval_s: float) -> dict:
    """只通过 ReadOnlySession 白名单读取状态；返回值不用于生成运动命令。"""
    raw_samples = []
    durations_ms = []
    for index in range(samples):
        started = time.perf_counter_ns()
        result = session.call("get_motion_status")
        durations_ms.append((time.perf_counter_ns() - started) / 1e6)
        # 当前 Python SDK 把 MotionStatus 放在返回元组的第 2 项。
        if len(result) != 2:
            raise RuntimeError(f"get_motion_status 返回项数异常：{result!r}")
        raw_samples.append(json_safe(result[1]))
        if index + 1 < samples:
            time.sleep(interval_s)
    return {
        "samples": raw_samples,
        "parsed_samples": [MotionStatus.parse(item).as_dict() for item in raw_samples],
        "timing": timing_summary(durations_ms),
        "distinct_sample_count": len({json.dumps(item, sort_keys=True, ensure_ascii=False) for item in raw_samples}),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-readonly", action="store_true")
    parser.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", "127.0.0.1"))
    parser.add_argument("--sdk-dir", type=Path, default=SDK_DIRECTORY)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--interval-ms", type=float, default=50.0)
    args = parser.parse_args(argv)
    if not args.live_readonly:
        print("默认不连接。现场静止只读检查请添加 --live-readonly。")
        return 0
    if not 1 <= args.samples <= 200:
        parser.error("--samples 必须为 1–200")
    if not 10.0 <= args.interval_ms <= 1000.0:
        parser.error("--interval-ms 必须为 10–1000")

    sdk = load_sdk(args.sdk_dir)
    if not hasattr(sdk.RC, "get_motion_status"):
        parser.error("当前 jkrc.RC 没有 get_motion_status")
    session = ReadOnlySession(sdk, args.host)
    logged_in = False
    try:
        session.connect()
        logged_in = True
        result = collect(session, args.samples, args.interval_ms / 1000.0)
    finally:
        if logged_in:
            session.close()

    report = {
        "schema": "jaka.motion_status.readonly.v1",
        "created_at": datetime.now().astimezone().isoformat(),
        "sample_count": args.samples,
        "interval_ms": args.interval_ms,
        **result,
        "movement_commands_sent": 0,
        "interpretation": "只读返回结构核查；不证明轨迹圆滑、队列语义或真机运动安全",
    }
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    report_path = REPORT_ROOT / (datetime.now().strftime("motion_status_%Y%m%d_%H%M%S") + ".json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告：{report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
