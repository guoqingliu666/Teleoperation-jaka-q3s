"""运动队列状态工具的离线验收；不导入厂商 SDK，不连接机器人。"""
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest

PYTHON_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PYTHON_ROOT / "src"))
SCRIPT = PYTHON_ROOT / "只读检查运动队列状态.py"
SPEC = importlib.util.spec_from_file_location("motion_status_readonly", SCRIPT)
module = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(module)
from vla_lab.motion_status import MotionStatus, MotionStatusParseError  # noqa: E402


class FakeReadOnlySession:
    def __init__(self):
        self.calls = []

    def call(self, name):
        self.calls.append(name)
        return (0, (12, 0, True, 0, 0, 0, False, False, False, False, False))


class MotionStatusReadonlyTests(unittest.TestCase):
    def test_default_does_not_connect(self):
        self.assertEqual(module.main([]), 0)

    def test_collect_only_calls_motion_status(self):
        session = FakeReadOnlySession()
        report = module.collect(session, samples=3, interval_s=0.0)
        self.assertEqual(session.calls, ["get_motion_status"] * 3)
        self.assertEqual(report["timing"]["count"], 3)
        self.assertEqual(report["distinct_sample_count"], 1)
        self.assertTrue(report["parsed_samples"][0]["inpos"])
        self.assertEqual(report["parsed_samples"][0]["queue"], 0)

    def test_exact_measured_shape_maps_to_named_fields(self):
        state = MotionStatus.parse([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        state.require_idle_safe()
        self.assertTrue(state.inpos)
        self.assertFalse(state.in_collision)

    def test_unknown_or_occupied_state_is_rejected(self):
        with self.assertRaises(ValueError):
            MotionStatus.parse([0] * 10)
        with self.assertRaises(ValueError):
            MotionStatus.parse([0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0])
        with self.assertRaises(RuntimeError):
            MotionStatus.parse([0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]).require_idle_safe()

    def test_negative_queue_preserves_raw_vendor_response_for_audit(self):
        raw = [3, 3, 0, 0, -1, 1, 0, 0, 0, 0, 0]
        with self.assertRaises(MotionStatusParseError) as caught:
            MotionStatus.parse(raw)
        self.assertEqual(caught.exception.raw, tuple(raw))
        self.assertIn("原始返回", str(caught.exception))

    def test_source_contains_no_motion_command_call(self):
        source = SCRIPT.read_text(encoding="utf-8")
        for forbidden in (
            ".linear_move(", ".linear_move_extend(", ".joint_move(",
            ".servo_j(", ".servo_p(", ".motion_abort(",
            ".power_on(", ".enable_robot(",
        ):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
