"""目标点运动门槛的纯离线测试；不会导入 jkrc，也不会连接机器人。"""

from __future__ import annotations

import importlib.util
import io
import unittest
from contextlib import redirect_stdout
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "真机验证新②_目标点运动.py"
spec = importlib.util.spec_from_file_location("target_point_gate", SCRIPT)
assert spec and spec.loader
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


class FakeClock:
    def __init__(self):
        self.now = 1.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeSdk:
    def __init__(self):
        self.calls = []
        self.in_pos_values = iter([0, 0, 1])

    def kine_inverse(self, joints, target):
        self.calls.append(("kine_inverse", (joints, target)))
        solved = list(joints)
        solved[1] += 0.02
        return (0, tuple(solved))

    def linear_move(self, *args):
        self.calls.append(("linear_move", args))
        return (0,)

    def is_in_pos(self):
        return (0, next(self.in_pos_values, 1))

    def is_in_estop(self):
        return (0, 0)

    def is_in_collision(self):
        return (0, 0)

    def is_on_limit(self):
        return (0, 0)

    def motion_abort(self):
        self.calls.append(("motion_abort", ()))
        return (0,)


class TargetPointTests(unittest.TestCase):
    def test_default_never_connects(self):
        with redirect_stdout(io.StringIO()) as output:
            self.assertEqual(gate.main([]), 0)
        self.assertIn("默认不连接", output.getvalue())

    def test_build_target_calls_vendor_inverse_once(self):
        sdk = FakeSdk()
        joints = (0.0,) * 6
        tcp = (500.0, 200.0, 300.0, 0.1, 0.2, 0.3)
        target, solved, delta = gate.build_target(sdk, joints, tcp, gate.PROFILES["z-plus-20mm"])
        self.assertEqual(target, (500.0, 200.0, 320.0, 0.1, 0.2, 0.3))
        self.assertEqual(len([call for call in sdk.calls if call[0] == "kine_inverse"]), 1)
        self.assertEqual(solved[1], 0.02)
        self.assertGreater(delta[1], 1.0)

    def test_50mm_profiles_are_locked_to_single_z_axis_and_10mm_per_second(self):
        plus = gate.PROFILES["z-plus-50mm"]
        minus = gate.PROFILES["z-minus-50mm"]
        self.assertEqual(plus["delta_xyz_mm"], (0.0, 0.0, 50.0))
        self.assertEqual(minus["delta_xyz_mm"], (0.0, 0.0, -50.0))
        self.assertEqual(plus["delta_rpy_deg"], (0.0, 0.0, 0.0))
        self.assertEqual(minus["delta_rpy_deg"], (0.0, 0.0, 0.0))
        self.assertEqual(plus["speed_mm_s"], 10.0)
        self.assertEqual(minus["speed_mm_s"], 10.0)
        self.assertEqual(plus["max_joint_delta_deg"], 12.0)
        self.assertEqual(minus["max_joint_delta_deg"], 12.0)

    def test_execute_sends_one_controller_planned_command(self):
        sdk, clock = FakeSdk(), FakeClock()
        result = gate.execute_target(
            sdk,
            (500.0, 200.0, 320.0, 0.1, 0.2, 0.3),
            speed_mm_s=10.0,
            expected_distance_mm=20.0,
            clock=clock,
            sleeper=clock.sleep,
        )
        moves = [call for call in sdk.calls if call[0] == "linear_move"]
        self.assertEqual(len(moves), 1)
        self.assertEqual(moves[0][1][1:], (0, False, 10.0))
        self.assertEqual(result["movement_commands_sent"], 1)
        self.assertFalse(result["motion_abort_sent"])

    def test_execute_reports_display_feedback_without_adding_motion_commands(self):
        sdk, clock = FakeSdk(), FakeClock()
        samples = []
        result = gate.execute_target(
            sdk,
            (500.0, 200.0, 320.0, 0.1, 0.2, 0.3),
            speed_mm_s=10.0,
            expected_distance_mm=20.0,
            clock=clock,
            sleeper=clock.sleep,
            progress_callback=lambda: samples.append(clock()),
        )
        self.assertEqual(len(samples), 3)
        self.assertEqual(result["display_feedback_samples"], 3)
        self.assertEqual(result["display_feedback_errors"], 0)
        self.assertEqual(len([call for call in sdk.calls if call[0] == "linear_move"]), 1)

    def test_display_feedback_failure_does_not_abort_vendor_motion(self):
        sdk, clock = FakeSdk(), FakeClock()

        def broken_display_callback():
            raise OSError("UDP display unavailable")

        result = gate.execute_target(
            sdk,
            (500.0, 200.0, 320.0, 0.1, 0.2, 0.3),
            speed_mm_s=10.0,
            expected_distance_mm=20.0,
            clock=clock,
            sleeper=clock.sleep,
            progress_callback=broken_display_callback,
        )
        self.assertEqual(result["display_feedback_samples"], 0)
        self.assertEqual(result["display_feedback_errors"], 3)
        self.assertNotIn(("motion_abort", ()), sdk.calls)

    def test_fault_requests_vendor_motion_abort(self):
        class CollisionSdk(FakeSdk):
            def is_in_collision(self):
                return (0, 1)

        sdk, clock = CollisionSdk(), FakeClock()
        with self.assertRaisesRegex(RuntimeError, "碰撞保护"):
            gate.execute_target(
                sdk,
                (500.0, 200.0, 320.0, 0.1, 0.2, 0.3),
                speed_mm_s=10.0,
                expected_distance_mm=20.0,
                clock=clock,
                sleeper=clock.sleep,
            )
        self.assertIn(("motion_abort", ()), sdk.calls)

    def test_speed_above_first_stage_limit_is_rejected_before_motion(self):
        sdk = FakeSdk()
        with self.assertRaisesRegex(ValueError, "1—30"):
            gate.execute_target(sdk, (0.0,) * 6, speed_mm_s=31.0, expected_distance_mm=20.0)
        self.assertFalse(any(name == "linear_move" for name, _ in sdk.calls))


if __name__ == "__main__":
    unittest.main()
