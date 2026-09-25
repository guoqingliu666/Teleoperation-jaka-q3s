"""JAKA 厂商规划器的固定两段圆滑验收入口。

默认不连接。``--live-plan-only`` 只读取状态并用厂商逆解检查固定路径；
``--live-two-segment`` 还需精确确认短语，才会执行所选固定档。程序不读取 Quest，
不调用 servo_j/servo_p，也不负责上电、使能、清报警或设置 Tool。
"""
from __future__ import annotations

import argparse
import configparser
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
REPORT_ROOT = ROOT / "Validation" / "blend_acceptance"
sys.path.insert(0, str(ROOT / "Python" / "src"))

from vla_lab.blend_acceptance import (  # noqa: E402
    PROFILES, build_fixed_corner_plan, execute_fixed_corner, scan_fixed_corner_envelope,
)
from vla_lab.jaka_telemetry import SDK_DIRECTORY, load_sdk, validate_host  # noqa: E402
from vla_lab.sdk_owner_lease import SdkOwnerLease, reject_legacy_bridge  # noqa: E402

def load_limits(path: Path):
    raw = path.read_bytes()
    config = configparser.ConfigParser(strict=False)
    config.read_string(raw.decode("utf-8-sig"))
    limits = [
        (float(config[f"JOINT_{index}"]["JOINT_MIN_LIMIT"]),
         float(config[f"JOINT_{index}"]["JOINT_MAX_LIMIT"]))
        for index in range(6)
    ]
    if any(not math.isfinite(low + high) or low >= high for low, high in limits):
        raise ValueError("关节限位配置无效")
    return limits, hashlib.sha256(raw).hexdigest()


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    mode = result.add_mutually_exclusive_group()
    mode.add_argument("--live-plan-only", action="store_true")
    mode.add_argument("--live-envelope-scan", action="store_true",
                      help="只读扫描固定 +Z→+X 路径的候选段长；不发送运动")
    mode.add_argument("--live-two-segment", action="store_true")
    result.add_argument("--profile", choices=tuple(PROFILES), default="micro5")
    result.add_argument("--confirm", default="")
    result.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", "127.0.0.1"))
    result.add_argument("--sdk-dir", type=Path, default=SDK_DIRECTORY)
    result.add_argument("--limits-file", type=Path, default=Path(os.environ.get(
        "QUEST_JAKA_LIMITS_FILE", "__SET_QUEST_JAKA_LIMITS_FILE_IN_LOCAL_CONFIG__")))
    return result


def joint_path_metrics(plan, limits):
    """只汇总厂商逆解结果，不修改、不平滑也不重算任何关节目标。"""
    points = [plan.start_joints, *plan.first_solutions, *plan.second_solutions]
    max_adjacent = max(
        abs(math.degrees(current[index] - previous[index]))
        for previous, current in zip(points, points[1:])
        for index in range(6)
    )
    max_from_start = max(
        abs(math.degrees(point[index] - plan.start_joints[index]))
        for point in points[1:]
        for index in range(6)
    )
    minimum_margin = min(
        min(math.degrees(point[index]) - limits[index][0],
            limits[index][1] - math.degrees(point[index]))
        for point in points
        for index in range(6)
    )
    return {
        "sample_count": len(points),
        "max_adjacent_joint_step_deg": max_adjacent,
        "max_joint_change_from_start_deg": max_from_start,
        "minimum_configured_joint_margin_deg": minimum_margin,
    }


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if not (args.live_plan_only or args.live_envelope_scan or args.live_two_segment):
        print("默认不连接。先运行 --live-plan-only；现场确认后才运行固定两段真机验收。")
        return 0
    profile = PROFILES[args.profile]
    if args.live_two_segment and args.confirm != profile.confirmation:
        raise SystemExit(f"真机验收必须添加 --confirm \"{profile.confirmation}\"")

    host = validate_host(args.host)
    limits, limits_sha256 = load_limits(args.limits_file)
    reject_legacy_bridge()
    lease = SdkOwnerLease()
    robot = None
    logged_in = False
    commands_sent = 0
    execution = None
    try:
        sdk = load_sdk(args.sdk_dir)
        if not hasattr(sdk.RC, "get_motion_status"):
            raise RuntimeError("当前 SDK 缺少 get_motion_status")
        robot = sdk.RC(host)
        login_result = robot.login()
        if not isinstance(login_result, tuple) or not login_result or login_result[0] != 0:
            raise RuntimeError(f"JAKA login 失败：{login_result!r}")
        logged_in = True
        if args.live_envelope_scan:
            report = {
                "schema": "jaka.vendor_blend_envelope_scan.v1",
                "created_at": datetime.now().astimezone().isoformat(),
                "mode": "live-envelope-scan",
                "scan": scan_fixed_corner_envelope(robot, limits),
                "limits_sha256": limits_sha256,
                "movement_commands_sent": 0,
                "interpretation": "当前姿态下固定路径的厂商逆解只读扫描；不是运动或工作空间放行",
            }
        else:
            plan = build_fixed_corner_plan(robot, limits, profile)
            if args.live_two_segment:
                execution = execute_fixed_corner(robot, plan)
                commands_sent = int(execution["commands_sent"])
            report = {
            "schema": "jaka.vendor_blend_acceptance.v1",
            "created_at": datetime.now().astimezone().isoformat(),
            "mode": "live-two-segment" if args.live_two_segment else "live-plan-only",
            "profile": profile.key,
            "path": {
                "start_tcp": plan.start_tcp,
                "corner_tcp": plan.corner_tcp,
                "final_tcp": plan.final_tcp,
                "segment_mm": profile.segment_mm,
                "first_tolerance_mm": profile.blend_tolerance_mm,
                "final_tolerance_mm": 0.0,
                "speed_mm_s": profile.speed_mm_s,
                "acceleration_mm_s2": profile.acceleration_mm_s2,
            },
            "vendor_ik_path_metrics": joint_path_metrics(plan, limits),
            "limits_sha256": limits_sha256,
            "movement_commands_sent": commands_sent,
            "execution": execution,
                "interpretation": "固定小范围厂商圆滑验收，不是连续手柄遥操作放行",
            }
    finally:
        if logged_in and robot is not None:
            try:
                robot.logout()
            except Exception as error:
                print(f"警告：logout 未确认：{error}", file=sys.stderr)
        lease.close()

    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    path = REPORT_ROOT / (datetime.now().strftime("blend_%Y%m%d_%H%M%S") + ".json")
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告：{path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
