"""手柄选择目标点的纯离线测试；不导入 jkrc，不绑定 UDP 5005。"""

from __future__ import annotations

import importlib.util
import io
import json
import socket
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from vla_lab.quest_vr_input import QuestUdpReceiver


SCRIPT = Path(__file__).resolve().parents[1] / "真机验证新②_手柄选择目标点.py"
spec = importlib.util.spec_from_file_location("quest_selected_target", SCRIPT)
assert spec and spec.loader
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


MAPPING = {
    "forward": "X+", "backward": "X-", "left": "Y+",
    "right": "Y-", "up": "Z+", "down": "Z-",
}


def frame(*, y=1.0, grip=0.0, a=False):
    payload = {
        "head": {"connected": True, "pose_valid": True, "rotation_xyzw": [0, 0, 0, 1]},
        "left": {},
        "right": {
            "connected": True, "tracked": True, "pose_valid": True,
            "position_m": [0.0, y, 0.0], "rotation_xyzw": [0, 0, 0, 1],
            "grip": grip, "trigger": 0.0, "primary_button": a,
        },
    }
    return QuestUdpReceiver._frame(payload, "127.0.0.1:5005")


class FakeRobot:
    def __init__(self): self.joint_read_count = 0
    def get_robot_status_simple(self): return (0, (0, 0, 1, 1))
    def is_in_pos(self): return (0, 1)
    def is_in_estop(self): return (0, 0)
    def is_in_collision(self): return (0, 0)
    def is_on_limit(self): return (0, 0)
    def is_in_servomove(self): return (0, 0)
    def get_tool_id(self): return (0, 1)
    def get_actual_joint_position(self):
        self.joint_read_count += 1
        return (0, (0.0,) * 6)
    def get_actual_tcp_position(self): return (0, (400.0, 100.0, 300.0, 0.1, 0.2, 0.3))


class QuestSelectedTargetTests(unittest.TestCase):
    def test_default_never_connects(self):
        with redirect_stdout(io.StringIO()) as output:
            self.assertEqual(gate.main([]), 0)
        self.assertIn("默认不连接", output.getvalue())

    def test_receiver_diagnostics_are_named_for_timeout_reports(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn('report["ignored_untracked_before_lock"]', source)
        self.assertIn('report["quest_receiver_error"]', source)

    def test_grip_gesture_freezes_one_position_only_target(self):
        selector, robot = gate.EndpointSelector(MAPPING), FakeRobot()
        selector.process(frame(grip=0.0), robot)
        selector.process(frame(grip=1.0), robot)
        selector.process(frame(y=1.03, grip=1.0), robot)
        selector.process(frame(y=1.03, grip=0.0), robot)
        joints, anchor, target = selector.frozen_target()
        self.assertEqual(joints, (0.0,) * 6)
        self.assertEqual(anchor[3:], target[3:])
        self.assertAlmostEqual(target[2] - anchor[2], 30.0, places=6)

    def test_target_is_clamped_to_50mm(self):
        selector, robot = gate.EndpointSelector(MAPPING), FakeRobot()
        selector.process(frame(), robot)
        selector.process(frame(grip=1.0), robot)
        selector.process(frame(y=1.2, grip=1.0), robot)
        selector.process(frame(y=1.2, grip=0.0), robot)
        _joints, anchor, target = selector.frozen_target()
        self.assertAlmostEqual(gate.math.dist(anchor[:3], target[:3]), 50.0, places=6)

    def test_tiny_gesture_is_rejected(self):
        selector, robot = gate.EndpointSelector(MAPPING), FakeRobot()
        selector.process(frame(), robot)
        selector.process(frame(grip=1.0), robot)
        selector.process(frame(y=1.001, grip=1.0), robot)
        selector.process(frame(y=1.001, grip=0.0), robot)
        with self.assertRaisesRegex(RuntimeError, "小于"):
            selector.frozen_target()

    def test_cycle_reset_discards_old_robot_anchor_and_target(self):
        selector, robot = gate.EndpointSelector(MAPPING), FakeRobot()
        selector.process(frame(), robot)
        selector.process(frame(grip=1.0), robot)
        selector.process(frame(y=1.03, grip=1.0), robot)
        selector.process(frame(y=1.03, grip=0.0), robot)
        self.assertIsNotNone(selector.frozen_target())
        selector.reset_cycle()
        self.assertIsNone(selector.frozen_target())
        self.assertIsNone(selector.anchor_tcp)
        self.assertIsNone(selector.anchor_joints)
        self.assertIsNone(selector.candidate)

    def test_source_never_uses_high_frequency_servo_commands(self):
        source = SCRIPT.read_text(encoding="utf-8")
        for forbidden in (".servo_j(", ".servo_p(", ".servo_move_enable("):
            self.assertNotIn(forbidden, source)

    def test_continuous_session_waits_for_session_budget_not_short_capture_timeout(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("selector, receiver, robot, remaining", source)
        self.assertIn("本轮执行确认已取消", source)

    def test_motion_feedback_uses_measured_joints_and_never_commands_robot(self):
        class FakeBroadcaster:
            def __init__(self): self.frames = []
            def publish(self, snapshot, **kwargs): self.frames.append((snapshot, kwargs))

        robot, broadcaster = FakeRobot(), FakeBroadcaster()
        target = (410.0, 100.0, 300.0, 0.1, 0.2, 0.3)
        callback = gate.make_motion_feedback_callback(robot, broadcaster, target)
        callback()
        self.assertEqual(robot.joint_read_count, 1)
        self.assertEqual(len(broadcaster.frames), 1)
        snapshot, kwargs = broadcaster.frames[0]
        self.assertEqual(snapshot.joints_rad, (0.0,) * 6)
        self.assertIsNone(snapshot.tcp_pose)
        self.assertEqual(kwargs["target_tcp_mm_rad"], target)

    def test_motion_feedback_udp_is_labeled_for_unity_priority(self):
        robot = FakeRobot()
        target = (410.0, 100.0, 300.0, 0.1, 0.2, 0.3)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1.0)
            broadcaster = gate.RobotVrBroadcaster(
                port=receiver.getsockname()[1], max_hz=20.0
            )
            try:
                gate.make_motion_feedback_callback(robot, broadcaster, target)()
                packet = json.loads(receiver.recv(65535))
            finally:
                broadcaster.close()
        self.assertEqual(packet["feedback_source"], "motion_session")
        self.assertEqual(packet["joints_rad"], [0.0] * 6)
        self.assertEqual(packet["target_tcp_mm_rad"], list(target))


if __name__ == "__main__":
    unittest.main()
