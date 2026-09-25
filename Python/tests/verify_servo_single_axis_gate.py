"""单轴往返真机门槛的纯离线测试；不会导入 jkrc。"""

from __future__ import annotations

import importlib.util
import io
import unittest
from contextlib import redirect_stdout
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "真机验证新②_单轴往返.py"
spec = importlib.util.spec_from_file_location("servo_single_axis_gate", SCRIPT)
assert spec and spec.loader
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, seconds)


class FakeSdk:
    def __init__(self, clock=None):
        self.clock = clock
        self.calls = []
        self.servo = False
        self.fail_servo_at = None
        self.inverse_jump = False

    def kine_inverse(self, reference, pose):
        self.calls.append(("kine_inverse", (reference, pose)))
        if self.clock:
            self.clock.now += 0.001
        offset_mm = pose[2] - 300.0
        solution = list(reference)
        solution[1] += offset_mm * 0.001
        if self.inverse_jump:
            solution[3] += 0.2
        return (0, tuple(solution))

    def servo_move_use_joint_LPF(self, value):
        self.calls.append(("servo_move_use_joint_LPF", (value,)))
        return (0,)

    def servo_move_enable(self, enabled, blocking):
        self.servo = bool(enabled)
        self.calls.append(("servo_move_enable", (enabled, blocking)))
        return (0,)

    def is_in_servomove(self):
        self.calls.append(("is_in_servomove", ()))
        return (0, int(self.servo))

    def servo_j(self, *args):
        self.calls.append(("servo_j", args))
        if self.clock:
            self.clock.now += 0.007
        servo_count = sum(name == "servo_j" for name, _ in self.calls)
        return (-1,) if self.fail_servo_at == servo_count else (0,)


class SequencedTimedFakeSdk(FakeSdk):
    def __init__(self, clock, call_seconds):
        super().__init__(clock)
        self.call_seconds = iter(call_seconds)

    def servo_j(self, *args):
        self.calls.append(("servo_j", args))
        self.clock.now += next(self.call_seconds, 0.007)
        return (0,)


class QueueFakeSdk(FakeSdk):
    def __init__(self, clock, queue_depths):
        super().__init__(clock)
        self.queue_depths = iter(queue_depths)

    def servo_j(self, *args):
        self.calls.append(("servo_j", args))
        self.clock.now += 0.007
        return (0, next(self.queue_depths, 1))


class SingleAxisGateTests(unittest.TestCase):
    def test_default_never_connects(self):
        with redirect_stdout(io.StringIO()) as output:
            self.assertEqual(gate.main([]), 0)
        self.assertIn("默认不连接", output.getvalue())

    def test_plan_is_z_only_roundtrip_with_unchanged_orientation(self):
        sdk = FakeSdk()
        joints = (0.0,) * 6
        tcp = (500.0, 200.0, 300.0, 0.1, 0.2, 0.3)
        plan, report = gate.build_plan(sdk, joints, tcp)
        targets = [args[1] for name, args in sdk.calls if name == "kine_inverse"]
        self.assertEqual(len(plan), 376)
        self.assertEqual(report["motion_points"], 251)
        self.assertEqual(report["settle_points"], 125)
        self.assertEqual(report["plan_points"], 376)
        self.assertEqual(targets[0], tcp)
        self.assertEqual(targets[-1], tcp)
        self.assertAlmostEqual(max(target[2] for target in targets), 301.0)
        self.assertTrue(all(target[:2] == tcp[:2] and target[3:] == tcp[3:] for target in targets))

    def test_vendor_inverse_results_are_sent_unchanged_and_in_order(self):
        clock = FakeClock()
        sdk = FakeSdk(clock)
        plan = [(index * 0.00001,) * 6 for index in range(8)]
        result = gate.execute_plan(sdk, plan, lpf_cutoff=0.8, clock=clock, sleeper=clock.sleep)
        sent = [args[0] for name, args in sdk.calls if name == "servo_j"]
        self.assertEqual(sent, plan)
        self.assertEqual(result["send_count"], len(plan))
        self.assertIsNone(result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_locked_5mm_profile_is_smooth_roundtrip(self):
        sdk = FakeSdk()
        joints = (0.0,) * 6
        tcp = (500.0, 200.0, 300.0, 0.1, 0.2, 0.3)
        plan, report = gate.build_plan(
            sdk,
            joints,
            tcp,
            distance_mm=5.0,
            duration_s=3.0,
            max_solution_excursion_deg=2.0,
        )
        targets = [args[1] for name, args in sdk.calls if name == "kine_inverse"]
        self.assertEqual(len(plan), 501)
        self.assertEqual(report["motion_points"], 376)
        self.assertEqual(report["settle_points"], 125)
        self.assertAlmostEqual(max(target[2] for target in targets), 305.0, delta=0.001)
        self.assertEqual(targets[0], tcp)
        self.assertEqual(targets[-1], tcp)
        self.assertEqual(report["distance_mm"], 5.0)
        self.assertEqual(report["orientation_change"], 0.0)

    def test_inverse_branch_jump_is_rejected_before_servo(self):
        sdk = FakeSdk()
        sdk.inverse_jump = True
        with self.assertRaisesRegex(RuntimeError, "厂商逆解相邻跳变"):
            gate.build_plan(sdk, (0.0,) * 6, (500.0, 200.0, 300.0, 0.1, 0.2, 0.3))
        self.assertFalse(any(name == "servo_j" for name, _ in sdk.calls))

    def test_servo_failure_disables_and_latches_failure(self):
        clock = FakeClock()
        sdk = FakeSdk(clock)
        sdk.fail_servo_at = 3
        plan = [(index * 0.00001,) * 6 for index in range(8)]
        result = gate.execute_plan(sdk, plan, lpf_cutoff=0.8, clock=clock, sleeper=clock.sleep)
        self.assertEqual(result["send_count"], 2)
        self.assertIn("servo_j", result["failure"])
        self.assertTrue(result["stop_confirmed"])
        self.assertIn(("servo_move_enable", (False, True)), sdk.calls)

    def test_integer_servo_readback_is_supported(self):
        self.assertTrue(gate.checked_bool((0, 1), "整数真值"))
        self.assertFalse(gate.checked_bool((0, 0), "整数假值"))

    def test_two_cycle_5mm_profile_uses_16ms_vendor_steps(self):
        sdk = FakeSdk()
        joints = (0.0,) * 6
        tcp = (500.0, 200.0, 300.0, 0.1, 0.2, 0.3)
        plan, report = gate.build_plan(
            sdk,
            joints,
            tcp,
            distance_mm=5.0,
            duration_s=3.0,
            max_solution_excursion_deg=2.0,
            step_num=2,
        )
        self.assertEqual(report["servo_period_ms"], 16.0)
        self.assertEqual(report["motion_points"], 189)
        self.assertEqual(report["settle_points"], 62)
        self.assertEqual(len(plan), 251)

    def test_two_cycle_horizon_with_8ms_updates_keeps_full_path_resolution(self):
        sdk = FakeSdk()
        joints = (0.0,) * 6
        tcp = (500.0, 200.0, 300.0, 0.1, 0.2, 0.3)
        plan, report = gate.build_plan(
            sdk,
            joints,
            tcp,
            distance_mm=1.0,
            duration_s=2.0,
            step_num=2,
            command_period_s=0.008,
        )
        self.assertEqual(report["interpolation_horizon_ms"], 16.0)
        self.assertEqual(report["command_period_ms"], 8.0)
        self.assertEqual(report["motion_points"], 251)
        self.assertEqual(report["settle_points"], 125)
        self.assertEqual(len(plan), 376)

    def test_split_schedule_skips_stale_point_but_still_sends_final_target(self):
        clock = FakeClock()
        sdk = SequencedTimedFakeSdk(clock, [0.015, 0.007, 0.007, 0.007, 0.007])
        plan = [(index * 0.00001,) * 6 for index in range(8)]
        result = gate.execute_plan(
            sdk,
            plan,
            lpf_cutoff=0.8,
            step_num=2,
            command_period_s=0.008,
            clock=clock,
            sleeper=clock.sleep,
        )
        sent = [args[0] for name, args in sdk.calls if name == "servo_j"]
        self.assertGreaterEqual(result["skipped_plan_points"], 1)
        self.assertEqual(sent[-1], plan[-1])
        self.assertIsNone(result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_three_cycle_horizon_with_8ms_updates_retains_full_plan(self):
        sdk = FakeSdk()
        plan, report = gate.build_plan(
            sdk,
            (0.0,) * 6,
            (500.0, 200.0, 300.0, 0.1, 0.2, 0.3),
            step_num=3,
            command_period_s=0.008,
        )
        self.assertEqual(report["interpolation_horizon_ms"], 24.0)
        self.assertEqual(report["command_period_ms"], 8.0)
        self.assertEqual(len(plan), 376)

    def test_vendor_queue_depth_is_recorded_and_limited(self):
        clock = FakeClock()
        sdk = QueueFakeSdk(clock, [1, 2, 11])
        plan = [(index * 0.00001,) * 6 for index in range(8)]
        result = gate.execute_plan(
            sdk, plan, lpf_cutoff=0.8, clock=clock, sleeper=clock.sleep
        )
        self.assertEqual(result["queue_depth"]["max"], 11)
        self.assertIn("队列长度 11", result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_step_one_recovers_one_expired_point_after_13ms_call(self):
        clock = FakeClock()
        sdk = SequencedTimedFakeSdk(clock, [0.0135, 0.007, 0.007, 0.007, 0.007])
        plan = [(index * 0.00001,) * 6 for index in range(12)]
        result = gate.execute_plan(
            sdk,
            plan,
            lpf_cutoff=0.8,
            step_num=1,
            command_period_s=0.008,
            clock=clock,
            sleeper=clock.sleep,
        )
        sent = [args[0] for name, args in sdk.calls if name == "servo_j"]
        self.assertGreaterEqual(result["skipped_plan_points"], 1)
        self.assertLessEqual(result["max_consecutive_skipped_points"], 1)
        self.assertEqual(sent[-1], plan[-1])
        self.assertIsNone(result["failure"])
        self.assertTrue(result["stop_confirmed"])


if __name__ == "__main__":
    unittest.main()
