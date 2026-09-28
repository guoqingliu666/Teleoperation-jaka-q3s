"""把遥操作会话切换到 Tool 1 / 用户坐标系 0；不发送任何运动命令。"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Python" / "src"))

from vla_lab.jaka_telemetry import load_sdk, validate_host
from vla_lab.sampled_follow import checked, flag
from vla_lab.sdk_owner_lease import (
    SdkOwnerLease,
    reject_legacy_bridge,
    request_readonly_handoff,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true",
                        help="必须显式提供；仅设置坐标系，不上电、不使能、不运动")
    parser.add_argument("--host", default=os.environ.get("QUEST_JAKA_HOST", ""))
    parser.add_argument("--sdk-dir", type=Path, default=Path(os.environ.get(
        "QUEST_JAKA_SDK_DIR", "__NOT_CONFIGURED__")))
    args = parser.parse_args(argv)
    if not args.apply:
        print("默认不连接；添加 --apply 才设置 Tool 1 / 用户坐标系 0。")
        return 0

    host = validate_host(args.host)
    lease = robot = None
    logged = False
    try:
        if not request_readonly_handoff(timeout_s=5.0):
            reject_legacy_bridge()
        else:
            time.sleep(1.0)
        lease = SdkOwnerLease()
        robot = load_sdk(args.sdk_dir).RC(host)
        checked(robot.login(), "login")
        logged = True

        # 写坐标系前要求机械臂静止且控制器没有安全故障；本脚本无运动接口调用。
        status = checked(robot.get_robot_status_simple(), "get_robot_status_simple")
        if len(status) != 4 or int(status[0]) != 0 or not bool(status[2]) or not bool(status[3]):
            raise RuntimeError(f"控制器未处于无报警、上电使能状态：{status!r}")
        if not flag(robot.is_in_pos(), "is_in_pos"):
            raise RuntimeError("机械臂尚未静止到位")
        if flag(robot.is_in_estop(), "is_in_estop") or flag(
                robot.is_in_collision(), "is_in_collision") or flag(
                robot.is_on_limit(), "is_on_limit") or flag(
                robot.is_in_servomove(), "is_in_servomove"):
            raise RuntimeError("控制器安全状态不允许切换遥操作坐标系")

        before = {
            "tool_id": int(checked(robot.get_tool_id(), "get_tool_id")),
            "user_frame_id": int(checked(robot.get_user_frame_id(), "get_user_frame_id")),
        }
        checked(robot.set_tool_id(1), "set_tool_id(1)")
        checked(robot.set_user_frame_id(0), "set_user_frame_id(0)")
        after = {
            "tool_id": int(checked(robot.get_tool_id(), "get_tool_id(readback)")),
            "user_frame_id": int(checked(
                robot.get_user_frame_id(), "get_user_frame_id(readback)")),
        }
        if after != {"tool_id": 1, "user_frame_id": 0}:
            raise RuntimeError(f"坐标系读回不一致：{after!r}")
        print(json.dumps({"before": before, "after": after,
                          "movement_commands_sent": 0}, ensure_ascii=False))
        return 0
    finally:
        if logged:
            checked(robot.logout(), "logout")
        if lease is not None:
            lease.close()


if __name__ == "__main__":
    raise SystemExit(main())
