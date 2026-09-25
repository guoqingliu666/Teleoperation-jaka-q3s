"""JAKA 固定三段圆滑验收入口。

默认不连接。``--live-plan-only`` 只读实机并使用厂商逆解检查固定路径；
``--live-three-segment`` 必须提供精确确认短语。它不会读取 Quest、不会调用
``servo_j``/``servo_p``，也不会上电、使能、清报警或修改 Tool。
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Python" / "src"))
sys.path.insert(0, str(ROOT / "Python"))

from vla_lab.multisegment_acceptance import (  # noqa: E402
    TRI20, build_three_segment_plan, execute_three_segment, path_metrics,
)
from vla_lab.jaka_telemetry import SDK_DIRECTORY, load_sdk, validate_host  # noqa: E402
from vla_lab.sdk_owner_lease import SdkOwnerLease, reject_legacy_bridge  # noqa: E402
from 真机验证_两段圆滑 import load_limits  # noqa: E402

REPORT_ROOT = ROOT / "Validation" / "multisegment_acceptance"


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    mode = result.add_mutually_exclusive_group()
    mode.add_argument("--live-plan-only", action="store_true")
    mode.add_argument("--live-three-segment", action="store_true")
    result.add_argument("--confirm", default="")
    result.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", "127.0.0.1"))
    result.add_argument("--sdk-dir", type=Path, default=SDK_DIRECTORY)
    result.add_argument("--limits-file", type=Path, default=Path(os.environ.get(
        "QUEST_JAKA_LIMITS_FILE", "__SET_QUEST_JAKA_LIMITS_FILE_IN_LOCAL_CONFIG__")))
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if not (args.live_plan_only or args.live_three_segment):
        print("默认不连接。先运行 --live-plan-only；现场确认后才运行固定三段真机验收。")
        return 0
    if args.live_three_segment and args.confirm != TRI20.confirmation:
        raise SystemExit(f"真机验收必须添加 --confirm \"{TRI20.confirmation}\"")

    host = validate_host(args.host)
    limits, limits_sha256 = load_limits(args.limits_file)
    reject_legacy_bridge()
    lease = SdkOwnerLease()
    robot = None
    logged_in = False
    report = None
    failure = None
    plan = None
    try:
        sdk = load_sdk(args.sdk_dir)
        if not hasattr(sdk.RC, "get_motion_status"):
            raise RuntimeError("当前 SDK 缺少 get_motion_status")
        robot = sdk.RC(host)
        if robot.login()[0] != 0:
            raise RuntimeError("JAKA login 失败")
        logged_in = True
        plan = build_three_segment_plan(robot, limits)
        execution = execute_three_segment(robot, plan) if args.live_three_segment else None
        report = {
            "schema": "jaka.vendor_three_segment_acceptance.v1",
            "created_at": datetime.now().astimezone().isoformat(),
            "mode": "live-three-segment" if args.live_three_segment else "live-plan-only",
            "profile": TRI20.key,
            "path": {"start_tcp": plan.start_tcp, "targets": plan.targets,
                     "leg_mm": TRI20.leg_mm, "speed_mm_s": TRI20.speed_mm_s,
                     "acceleration_mm_s2": TRI20.acceleration_mm_s2,
                     "blend_tolerance_mm": TRI20.blend_tolerance_mm},
            "vendor_ik_path_metrics": path_metrics(plan, limits),
            "limits_sha256": limits_sha256,
            "movement_commands_sent": 0 if execution is None else execution["commands_sent"],
            "execution": execution,
            "interpretation": "固定三段厂商圆滑验收，不是连续手柄遥操作放行",
        }
    except Exception as error:
        # 运动核心已在“首段可能开始”后承担 motion_abort + 停止确认义务；入口
        # 不吞掉失败，而是把当前已知计划、原始错误文字写入报告，便于审计 SDK 返回。
        failure = str(error)
        report = {
            "schema": "jaka.vendor_three_segment_acceptance.v1",
            "created_at": datetime.now().astimezone().isoformat(),
            "mode": "failed-live-three-segment" if args.live_three_segment else "failed-live-plan-only",
            "profile": TRI20.key,
            "path": None if plan is None else {"start_tcp": plan.start_tcp, "targets": plan.targets},
            "limits_sha256": limits_sha256,
            "movement_commands_sent": "unknown_after_failure",
            "execution": None,
            "failure": failure,
            "interpretation": "失败已记录；不是运动成功或安全放行",
        }
    finally:
        if logged_in and robot is not None:
            try:
                robot.logout()
            except Exception as error:
                print(f"警告：logout 未确认：{error}", file=sys.stderr)
        lease.close()

    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    path = REPORT_ROOT / (datetime.now().strftime("three_segment_%Y%m%d_%H%M%S") + ".json")
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"报告：{path}")
    return 2 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
