"""读取既有目标日志，生成最近3点有限前瞻报告；不导入SDK、不连接机器人。"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Python" / "src"))
from vla_lab.lookahead_shadow import ShadowSettings, bounded_candidates, diagnose_windows


def load_points(path):
    points = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        event = json.loads(line)
        if event.get("state") != "target":
            continue
        stamp, target = event.get("time_ns"), event.get("target")
        if not isinstance(stamp, int) or not isinstance(target, list) or len(target) != 6:
            raise ValueError(f"第{number}行目标格式错误")
        xyz = tuple(float(value) for value in target[:3])
        if points and stamp <= points[-1][0]:
            raise ValueError(f"第{number}行时间未严格递增")
        points.append((stamp, xyz))
    return points


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--period-ms", type=int, default=50)
    parser.add_argument("--max-chord-mm", type=float, default=8)
    parser.add_argument("--stop-turn-deg", type=float, default=60)
    parser.add_argument("--details", action="store_true", help="同时输出全部最近3点窗口")
    args = parser.parse_args(argv)
    settings = ShadowSettings(args.period_ms, args.max_chord_mm, .5, args.stop_turn_deg, 3)
    candidates = bounded_candidates(load_points(args.input), settings)
    diagnosis = diagnose_windows(candidates, settings)
    print(json.dumps({
        "schema": "quest_finite_lookahead_shadow.v1",
        "input": str(args.input),
        "settings": asdict(settings),
        "movement_commands_sent": 0,
        "sdk_loaded": False,
        "diagnosis": {key: value for key, value in diagnosis.items() if key != "windows"},
        **({"windows": diagnosis["windows"]} if args.details else {}),
        "interpretation": "离线影子候选；尚未检查厂商逆解、碰撞、关节速度或控制器融合语义",
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
