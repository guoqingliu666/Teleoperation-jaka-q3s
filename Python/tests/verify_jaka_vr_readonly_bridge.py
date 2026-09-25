"""数字孪生只读反馈桥的离线门禁；绝不导入 jkrc 或启动 Unity。"""

from __future__ import annotations

import io
import json
import socket
import unittest
from unittest.mock import patch
from contextlib import redirect_stdout
from pathlib import Path

from vla_lab import jaka_vr_readonly_bridge as bridge


class FakeSession:
    def __init__(self):
        self.calls = []
        self.values = {
            "get_robot_status_simple": (0, (0, "", 1, 1)),
            "get_actual_joint_position": (0, [0.1] * 6),
            "get_actual_tcp_position": (0, [400, 100, 300, 0.1, 0.2, 0.3]),
            "get_tool_id": (0, 1),
        }

    def call(self, name):
        self.calls.append(name)
        return self.values[name]


class BridgeTests(unittest.TestCase):
    def test_default_does_not_start_anything(self):
        with redirect_stdout(io.StringIO()) as output:
            self.assertEqual(bridge.main([]), 0)
        self.assertIn("默认不启动", output.getvalue())

    def test_read_snapshot_uses_measured_feedback(self):
        state = bridge.read_snapshot(FakeSession())
        self.assertTrue(state.connected)
        self.assertTrue(state.powered_on)
        self.assertTrue(state.enabled)
        self.assertEqual(state.tool_id, 1)
        self.assertEqual(state.joints_rad, (0.1,) * 6)
        self.assertEqual(state.tcp_pose[:3], (400.0, 100.0, 300.0))

    def test_source_contains_no_robot_write_calls(self):
        source = Path(bridge.__file__).read_text(encoding="utf-8")
        for forbidden in (
            ".linear_move(", ".joint_move(", ".servo_j(", ".servo_p(",
            ".power_on(", ".enable_robot(", ".motion_abort(", ".kine_inverse(",
        ):
            self.assertNotIn(forbidden, source)

    def test_fast_reader_reads_joints_each_frame_and_decimates_status(self):
        session = FakeSession()
        with patch.object(bridge.time, "perf_counter", return_value=10.0):
            reader = bridge.FastMeasuredReader()
            state = reader.read(session)
        first = len(session.calls)
        with patch.object(bridge.time, "perf_counter", return_value=10.016):
            state = reader.read(session)
        self.assertEqual(session.calls[first:], ["get_actual_joint_position"])
        self.assertEqual(state.feedback_source, "readonly_bridge")
        self.assertGreater(state.sample_time_ns, 0)

    def test_slow_joint_read_is_rejected_not_retimestamped_as_fresh(self):
        session = FakeSession()
        with patch.object(bridge.time, "perf_counter", return_value=10.0):
            reader = bridge.FastMeasuredReader()
            reader.read(session)
        with patch.object(bridge.time, "perf_counter", side_effect=[10.01,10.01,10.3]):
            with self.assertRaisesRegex(RuntimeError, "丢弃迟到"):
                reader.read(session)

    def test_target_overlay_is_display_only_and_clearable(self):
        receiver = bridge.TargetOverlayReceiver(0)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            port = receiver._socket.getsockname()[1]
            packet = {
                "schema": "quest_jaka_target_overlay.v1",
                "target_tcp_mm_rad": [400, 100, 300, 0.1, 0.2, 0.3],
                "hold_s": 10,
            }
            sender.sendto(json.dumps(packet).encode("utf-8"), ("127.0.0.1", port))
            self.assertEqual(receiver.latest(), (400.0, 100.0, 300.0, 0.1, 0.2, 0.3))
            packet["target_tcp_mm_rad"] = None
            sender.sendto(json.dumps(packet).encode("utf-8"), ("127.0.0.1", port))
            self.assertIsNone(receiver.latest())
        finally:
            sender.close()
            receiver.close()


if __name__ == "__main__":
    unittest.main()
