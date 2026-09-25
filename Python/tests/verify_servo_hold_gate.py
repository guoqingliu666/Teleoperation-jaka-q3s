"""零位保持真机门槛的纯离线测试；不会导入 jkrc。"""

from __future__ import annotations

import importlib.util
import io
import unittest
from contextlib import redirect_stdout
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "真机验证新②_零位保持.py"
spec = importlib.util.spec_from_file_location("servo_hold_gate", SCRIPT)
assert spec and spec.loader
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


class FakeClock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, seconds)


class FakeSdk:
    def __init__(self):
        self.calls = []
        self.servo = False
        self.fail_servo_j = False

    def _ok(self, name, *args):
        self.calls.append((name, args))
        return (0,)

    def servo_move_use_joint_LPF(self, value):
        return self._ok("servo_move_use_joint_LPF", value)

    def servo_move_enable(self, enabled, blocking):
        self.servo = enabled
        return self._ok("servo_move_enable", enabled, blocking)

    def is_in_servomove(self):
        self.calls.append(("is_in_servomove", ()))
        # 现场 JAKA SDK 返回整数 1/0，不是 Python bool。
        return (0, int(self.servo))

    def servo_j(self, *args):
        self.calls.append(("servo_j", args))
        return (-1,) if self.fail_servo_j else (0,)


class TimedFakeSdk(FakeSdk):
    """让假 SDK 调用占用指定时间，用于复现现场阻塞耗时。"""

    def __init__(self, clock, call_seconds):
        super().__init__()
        self.clock = clock
        self.call_seconds = call_seconds

    def servo_j(self, *args):
        self.clock.now += self.call_seconds
        return super().servo_j(*args)


class SequencedTimedFakeSdk(FakeSdk):
    """按序模拟正常调用、单次抖动、随后恢复。"""

    def __init__(self, clock, call_seconds):
        super().__init__()
        self.clock = clock
        self.call_seconds = iter(call_seconds)

    def servo_j(self, *args):
        self.clock.now += next(self.call_seconds, 0.007)
        return super().servo_j(*args)


class QueueFakeSdk(FakeSdk):
    def __init__(self, queue_depths):
        super().__init__()
        self.queue_depths = iter(queue_depths)

    def servo_j(self, *args):
        self.calls.append(("servo_j", args))
        return (0, next(self.queue_depths, 0))


class ServoHoldTests(unittest.TestCase):
    def test_sdk_integer_boolean_readback_is_supported(self):
        self.assertTrue(gate.checked_bool((0, 1), "整数真值"))
        self.assertFalse(gate.checked_bool((0, 0), "整数假值"))
        with self.assertRaises(RuntimeError):
            gate.checked_bool((0, 2), "非法布尔值")

    def test_default_never_connects(self):
        with redirect_stdout(io.StringIO()) as output:
            self.assertEqual(gate.main([]), 0)
        self.assertIn("默认不连接", output.getvalue())

    def test_constant_current_joint_target_and_confirmed_stop(self):
        sdk, clock = FakeSdk(), FakeClock()
        target = (0.1,) * 6
        result = gate.servo_hold(
            sdk, target, lpf_cutoff=0.5, seconds=0.032,
            clock=clock, sleeper=clock.sleep,
        )
        servo_calls = [args for name, args in sdk.calls if name == "servo_j"]
        self.assertGreaterEqual(len(servo_calls), 3)
        self.assertTrue(all(args == (target, 0, 1) for args in servo_calls))
        self.assertTrue(result["stop_confirmed"])
        self.assertIsNone(result["failure"])
        self.assertEqual(sdk.calls[0], ("servo_move_use_joint_LPF", (0.5,)))
        self.assertEqual(sdk.calls[-1], ("is_in_servomove", ()))

    def test_servo_error_disables_and_reports_failure(self):
        sdk, clock = FakeSdk(), FakeClock()
        sdk.fail_servo_j = True
        result = gate.servo_hold(
            sdk, (0.0,) * 6, lpf_cutoff=0.5, seconds=0.032,
            clock=clock, sleeper=clock.sleep,
        )
        self.assertIn("servo_j 失败", result["failure"])
        self.assertTrue(result["stop_confirmed"])
        self.assertIn(("servo_move_enable", (False, True)), sdk.calls)

    def test_real_observed_6_839_ms_call_is_inside_8_ms_period(self):
        clock = FakeClock()
        sdk = TimedFakeSdk(clock, 0.006839)
        result = gate.servo_hold(
            sdk, (0.0,) * 6, lpf_cutoff=0.8, seconds=0.032,
            clock=clock, sleeper=clock.sleep,
        )
        self.assertGreaterEqual(result["send_count"], 3)
        self.assertIsNone(result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_single_8_676_ms_return_is_reported_but_scheduler_can_recover(self):
        clock = FakeClock()
        sdk = SequencedTimedFakeSdk(clock, [0.007, 0.008676, 0.007, 0.007])
        result = gate.servo_hold(
            sdk, (0.0,) * 6, lpf_cutoff=0.8, seconds=0.032,
            clock=clock, sleeper=clock.sleep,
        )
        self.assertGreaterEqual(result["send_count"], 3)
        self.assertEqual(result["servo_call_over_period_count"], 1)
        self.assertIsNone(result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_call_over_period_plus_lateness_stops_without_second_send(self):
        clock = FakeClock()
        sdk = TimedFakeSdk(clock, 0.012001)
        result = gate.servo_hold(
            sdk, (0.0,) * 6, lpf_cutoff=0.8, seconds=0.032,
            clock=clock, sleeper=clock.sleep,
        )
        self.assertEqual(result["send_count"], 1)
        self.assertIn("servo_j 调用耗时", result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_repeated_11_ms_calls_accumulate_lateness_and_stop(self):
        clock = FakeClock()
        sdk = TimedFakeSdk(clock, 0.011)
        result = gate.servo_hold(
            sdk, (0.0,) * 6, lpf_cutoff=0.8, seconds=0.050,
            clock=clock, sleeper=clock.sleep,
        )
        self.assertEqual(result["send_count"], 2)
        self.assertIn("伺服调度迟到", result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_vendor_two_cycle_mode_accepts_observed_13_193_ms_call(self):
        clock = FakeClock()
        sdk = TimedFakeSdk(clock, 0.013193)
        result = gate.servo_hold(
            sdk,
            (0.0,) * 6,
            lpf_cutoff=0.8,
            seconds=0.064,
            step_num=2,
            clock=clock,
            sleeper=clock.sleep,
        )
        servo_calls = [args for name, args in sdk.calls if name == "servo_j"]
        self.assertGreaterEqual(result["send_count"], 4)
        self.assertTrue(all(args[2] == 2 for args in servo_calls))
        self.assertEqual(result["servo_period_ms"], 16.0)
        self.assertIsNone(result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_two_cycle_horizon_can_receive_targets_every_8ms(self):
        clock = FakeClock()
        sdk = TimedFakeSdk(clock, 0.007)
        result = gate.servo_hold(
            sdk,
            (0.0,) * 6,
            lpf_cutoff=0.8,
            seconds=0.040,
            step_num=2,
            command_period_s=0.008,
            clock=clock,
            sleeper=clock.sleep,
        )
        self.assertEqual(result["interpolation_horizon_ms"], 16.0)
        self.assertEqual(result["command_period_ms"], 8.0)
        self.assertGreaterEqual(result["send_count"], 5)
        self.assertIsNone(result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_two_cycle_horizon_skips_expired_slot_after_one_slow_call(self):
        clock = FakeClock()
        sdk = SequencedTimedFakeSdk(clock, [0.015, 0.007, 0.007, 0.007])
        result = gate.servo_hold(
            sdk,
            (0.0,) * 6,
            lpf_cutoff=0.8,
            seconds=0.048,
            step_num=2,
            command_period_s=0.008,
            clock=clock,
            sleeper=clock.sleep,
        )
        self.assertGreaterEqual(result["skipped_update_slots"], 1)
        self.assertIsNone(result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_three_cycle_horizon_covers_observed_22ms_stall(self):
        clock = FakeClock()
        sdk = SequencedTimedFakeSdk(clock, [0.022, 0.007, 0.007, 0.007])
        result = gate.servo_hold(
            sdk,
            (0.0,) * 6,
            lpf_cutoff=0.8,
            seconds=0.056,
            step_num=3,
            command_period_s=0.008,
            clock=clock,
            sleeper=clock.sleep,
        )
        self.assertEqual(result["interpolation_horizon_ms"], 24.0)
        self.assertGreaterEqual(result["skipped_update_slots"], 2)
        self.assertIsNone(result["failure"])
        self.assertTrue(result["stop_confirmed"])

    def test_vendor_queue_depth_is_recorded(self):
        sdk, clock = QueueFakeSdk([1, 2, 3, 4]), FakeClock()
        result = gate.servo_hold(
            sdk,
            (0.0,) * 6,
            lpf_cutoff=0.8,
            seconds=0.032,
            clock=clock,
            sleeper=clock.sleep,
        )
        self.assertGreaterEqual(result["queue_depth"]["count"], 3)
        self.assertGreaterEqual(result["queue_depth"]["max"], 3)
        self.assertEqual(result["queue_depth"]["over_10"], 0)

    def test_no_filter_default(self):
        with self.assertRaisesRegex(ValueError, "截止频率"):
            gate.servo_hold(FakeSdk(), (0.0,) * 6, lpf_cutoff=0.0, seconds=1.0)


if __name__ == "__main__":
    unittest.main()
