"""新②动态影子脚本的离线测试；假 SDK 不具备任何运动方法。"""

from __future__ import annotations

import ast
import importlib.util
import io
import json
import socket
import threading
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "只读检查新②_动态影子.py"
spec = importlib.util.spec_from_file_location("vendor_ik_dynamic_shadow", SCRIPT)
assert spec is not None and spec.loader is not None
shadow = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shadow)

from vla_lab.quest_vr_input import QuestUdpReceiver


class ReadOnlyFakeRobot:
    """故意没有伺服、上电或使能函数；误调用会直接失败。"""

    def __init__(self) -> None:
        self.calls: list[str] = []

    def get_actual_tcp_position(self):
        self.calls.append("get_actual_tcp_position")
        return (0, (400.0, 100.0, 400.0, 0.0, 0.0, 0.0))

    def get_actual_joint_position(self):
        self.calls.append("get_actual_joint_position")
        return (0, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def get_tool_id(self):
        self.calls.append("get_tool_id")
        return (0, 1)

    def kine_inverse(self, joints, target):
        self.calls.append("kine_inverse")
        assert len(joints) == len(target) == 6
        return (0, joints)


def packet(*, tracked=True, grip=1.0, x=0.0):
    return {
        "head": {"connected": True, "pose_valid": True, "rotation_xyzw": [0, 0, 0, 1]},
        "right": {
            "connected": True,
            "tracked": tracked,
            "pose_valid": tracked,
            "position_m": [x, 0.8, 0.2],
            "rotation_xyzw": [0, 0, 0, 1],
            "grip": grip,
            "trigger": 0.0,
        },
    }


def frame(*, tracked=True, grip=1.0, x=0.0):
    return QuestUdpReceiver._frame(packet(tracked=tracked, grip=grip, x=x), "127.0.0.1:5005")


class DynamicShadowTests(unittest.TestCase):
    def setUp(self):
        self.robot = ReadOnlyFakeRobot()
        self.observer = shadow.DynamicShadow(self.robot, {
            "forward": "X+", "backward": "X-", "left": "Y+", "right": "Y-", "up": "Z+", "down": "Z-"
        })

    def test_default_does_not_connect(self):
        with redirect_stdout(io.StringIO()) as output:
            self.assertEqual(shadow.main([]), 0)
        self.assertIn("默认不连接机器人", output.getvalue())

    def test_grip_generates_only_inverse_diagnostic(self):
        first = self.observer.process(frame(), time.perf_counter())
        self.assertEqual(first["event"], "仅生成厂商逆解；未发送")
        second = self.observer.process(frame(x=0.01), time.perf_counter())
        self.assertIn("solution_joints_rad", second)
        self.assertIn("largest_solution_step_deg", second)
        self.assertEqual(self.robot.calls.count("kine_inverse"), 2)

    def test_tracking_loss_clears_anchor_and_stops_inverse(self):
        self.observer.process(frame(), time.perf_counter())
        before = self.robot.calls.count("kine_inverse")
        lost = self.observer.process(frame(tracked=False), time.perf_counter())
        self.assertIn("锚点已清除", lost["event"])
        self.assertIsNone(self.observer.anchor_tcp)
        self.assertEqual(self.robot.calls.count("kine_inverse"), before)

    def test_position_and_rotation_modes_freeze_other_part(self):
        position = shadow.DynamicShadow(self.robot, {
            "forward": "X+", "backward": "X-", "left": "Y+", "right": "Y-", "up": "Z+", "down": "Z-"
        }, mode="position-only")
        first = position.process(frame(), time.perf_counter())
        moved = position.process(frame(x=0.01), time.perf_counter())
        self.assertNotEqual(first["target_tcp"][:3], moved["target_tcp"][:3])
        self.assertEqual(first["target_tcp"][3:], moved["target_tcp"][3:])

        rotation = shadow.DynamicShadow(self.robot, {
            "forward": "X+", "backward": "X-", "left": "Y+", "right": "Y-", "up": "Z+", "down": "Z-"
        }, mode="rotation-only")
        first = rotation.process(frame(), time.perf_counter())
        moved = rotation.process(frame(x=0.01), time.perf_counter())
        self.assertEqual(first["target_tcp"][:3], moved["target_tcp"][:3])

    def test_source_has_only_read_and_inverse_sdk_calls(self):
        allowed = {
            "login", "logout", "get_tool_id", "is_in_pos", "is_in_servomove",
            "get_actual_joint_position", "get_actual_tcp_position", "kine_inverse",
        }
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        calls = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            owner = node.func.value
            if isinstance(owner, ast.Name) and owner.id == "robot":
                calls.add(node.func.attr)
            if (isinstance(owner, ast.Attribute) and isinstance(owner.value, ast.Name)
                    and owner.value.id == "self" and owner.attr == "robot"):
                calls.add(node.func.attr)
        self.assertTrue(calls)
        self.assertEqual(calls - allowed, set())

    def test_local_udp_run_with_fake_robot(self):
        # 使用本机 UDP 真正穿过接收循环；假 SDK 仍然没有任何运动方法。
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]

        def send():
            time.sleep(0.03)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                for index in range(30):
                    payload = packet(x=index * 0.0001)
                    sender.sendto(json.dumps(payload).encode("utf-8"), ("127.0.0.1", port))
                    time.sleep(0.005)

        thread = threading.Thread(target=send)
        thread.start()
        with redirect_stdout(io.StringIO()):
            summary, events = shadow.run(self.robot, seconds=0.4, port=port)
        thread.join()
        self.assertGreaterEqual(summary["solved_frames"], 20)
        self.assertEqual(summary["movement_commands_sent"], 0)
        self.assertEqual(summary["safety_acceptance"], "not_assessed")
        self.assertEqual(summary["diagnostic_errors"], 0)
        self.assertTrue(events)


if __name__ == "__main__":
    unittest.main()
