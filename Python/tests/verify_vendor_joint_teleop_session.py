"""新②伺服会话的离线协议、节拍和故障注入测试。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from vla_lab.vendor_joint_teleop_candidate import JointLpfCutoff, RejectLimits, Sample  # noqa: E402
from vla_lab.vendor_joint_teleop_session import (  # noqa: E402
    OfflineJointServoSession, SessionTiming,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now


class FakeSdk:
    __vla_offline_test_double__ = True

    def __init__(self) -> None:
        self.calls = []
        self.fail_on = None
        self.joints = (0.01, 0.02, 0.03, 0.04, 0.05, 0.06)
        self.servo_enabled = False
        self.force_readback = None

    def _call(self, name, *args):
        self.calls.append((name, args))
        return (-1,) if name == self.fail_on else (0,)

    def kine_inverse(self, reference, pose):
        self.calls.append(("kine_inverse", (reference, pose)))
        return (0, self.joints)

    def servo_move_use_joint_LPF(self, *args):
        return self._call("servo_move_use_joint_LPF", *args)

    def servo_move_enable(self, *args):
        result = self._call("servo_move_enable", *args)
        if result[0] == 0:
            self.servo_enabled = args[0]
        return result

    def is_in_servomove(self):
        state = self.servo_enabled if self.force_readback is None else self.force_readback
        self.calls.append(("is_in_servomove", ()))
        return (0, state)

    def servo_j(self, *args):
        return self._call("servo_j", *args)


def sample(now=10.0, **changes):
    values = dict(
        target_pose=(101.0, 200.0, 300.0, 0.0, 0.0, 0.0),
        actual_tcp_pose=(100.0, 200.0, 300.0, 0.0, 0.0, 0.0),
        actual_joints=(0.0,) * 6,
        input_time_s=now, feedback_time_s=now,
        tracked=True, grip_held=True, powered=True, enabled=True,
        estop=False, collision=False, on_limit=False, tool_id=1,
    )
    values.update(changes)
    return Sample(**values)


def session(sdk=None, clock=None, *, max_input_age_s=0.1):
    sdk = sdk or FakeSdk()
    clock = clock or FakeClock()
    return OfflineJointServoSession(
        sdk,
        reject_limits=RejectLimits(max_input_age_s, 0.1, 5.0, 5.0, 0.2, 0.007, 50.0, 20.0),
        # 0.5 只用于假 SDK 协议测试，不是本机真机推荐值。
        lpf_cutoff=JointLpfCutoff(0.5),
        timing=SessionTiming(0.003, 0.004, 0.09, 1.0),
        clock=clock,
    )


class SessionTests(unittest.TestCase):
    def test_requires_explicit_offline_double(self):
        with self.assertRaisesRegex(RuntimeError, "只能用于离线"):
            session(sdk=object())

    def test_sdk_call_order_and_exact_ik_to_servo_j(self):
        sdk, clock = FakeSdk(), FakeClock()
        worker = session(sdk, clock)
        self.assertTrue(worker.start(sample()))
        self.assertEqual([name for name, _ in sdk.calls], [
            "kine_inverse", "servo_move_use_joint_LPF", "servo_move_enable", "is_in_servomove",
        ])
        self.assertTrue(worker.tick())
        self.assertEqual(sdk.calls[-1], ("servo_j", (sdk.joints, 0, 1)))
        clock.now += 0.008
        self.assertTrue(worker.tick())
        self.assertEqual(worker.sent_count, 2)
        self.assertNotIn("servo_p", [name for name, _ in sdk.calls])
        self.assertNotIn("servo_speed_foresight", [name for name, _ in sdk.calls])

    def test_stale_feedback_stops_before_next_servo_command(self):
        sdk, clock = FakeSdk(), FakeClock()
        worker = session(sdk, clock)
        self.assertTrue(worker.start(sample()))
        self.assertTrue(worker.tick())
        clock.now = 10.08
        self.assertTrue(worker.update_target(sample(
            now=clock.now, feedback_time_s=10.0,
        )))
        clock.now = 10.101
        self.assertFalse(worker.tick())
        self.assertEqual(worker.sent_count, 1)
        self.assertIn("过期", worker.stop_reason)
        self.assertIn(("servo_move_enable", (False, True)), sdk.calls)
        self.assertEqual(sdk.calls[-1], ("is_in_servomove", ()))
        self.assertTrue(worker.stop_confirmed)

    def test_late_8ms_tick_stops_without_burst_catch_up(self):
        sdk, clock = FakeSdk(), FakeClock()
        worker = session(sdk, clock)
        self.assertTrue(worker.start(sample()))
        self.assertTrue(worker.tick())
        clock.now += 0.015
        self.assertFalse(worker.tick())
        self.assertIn("节拍迟到", worker.stop_reason)
        self.assertEqual(worker.sent_count, 1)

    def test_sdk_servo_error_disables_and_latches(self):
        sdk = FakeSdk()
        worker = session(sdk)
        self.assertTrue(worker.start(sample()))
        sdk.fail_on = "servo_j"
        self.assertFalse(worker.tick())
        self.assertTrue(worker.stopped)
        self.assertTrue(worker.stop_confirmed)
        self.assertIn(("servo_move_enable", (False, True)), sdk.calls)
        self.assertEqual(sdk.calls[-1], ("is_in_servomove", ()))
        with self.assertRaisesRegex(RuntimeError, "不允许重用"):
            worker.start(sample())

    def test_failed_stop_ack_is_not_misreported_as_safe(self):
        sdk = FakeSdk()
        worker = session(sdk)
        self.assertTrue(worker.start(sample()))
        sdk.fail_on = "servo_move_enable"
        worker.stop()
        self.assertTrue(worker.stopped)
        self.assertFalse(worker.stop_confirmed)
        self.assertIn("停止未确认", worker.stop_reason)

    def test_filter_failure_cannot_enable_or_send_motion(self):
        sdk = FakeSdk()
        sdk.fail_on = "servo_move_use_joint_LPF"
        worker = session(sdk)
        self.assertFalse(worker.start(sample()))
        self.assertNotIn(("servo_move_enable", (True, True)), sdk.calls)
        self.assertFalse(any(name == "servo_j" for name, _ in sdk.calls))

    def test_bad_target_refuses_before_filter_or_enable(self):
        sdk = FakeSdk()
        worker = session(sdk)
        self.assertFalse(worker.start(sample(tracked=False)))
        self.assertEqual(sdk.calls, [])

    def test_start_requires_servo_mode_readback(self):
        sdk = FakeSdk()
        sdk.force_readback = False
        worker = session(sdk)
        self.assertFalse(worker.start(sample()))
        self.assertFalse(worker.active)
        self.assertTrue(worker.stop_confirmed)
        self.assertIn(("servo_move_enable", (False, True)), sdk.calls)
        self.assertFalse(any(name == "servo_j" for name, _ in sdk.calls))

    def test_stop_requires_servo_mode_readback(self):
        sdk = FakeSdk()
        worker = session(sdk)
        self.assertTrue(worker.start(sample()))
        sdk.force_readback = True
        worker.stop()
        self.assertFalse(worker.stop_confirmed)
        self.assertIn("停止未确认", worker.stop_reason)

    def test_new_frame_is_resolved_from_measured_joints(self):
        sdk, clock = FakeSdk(), FakeClock()
        worker = session(sdk, clock)
        self.assertTrue(worker.start(sample()))
        self.assertTrue(worker.tick())
        clock.now += 0.008
        actual = (0.02,) * 6
        sdk.joints = (0.03,) * 6
        self.assertTrue(worker.update_target(sample(now=clock.now, actual_joints=actual)))
        self.assertEqual(sdk.calls[-1], ("kine_inverse", (actual, sample().target_pose)))
        self.assertTrue(worker.tick())
        self.assertEqual(sdk.calls[-1], ("servo_j", (sdk.joints, 0, 1)))

    def test_25hz_targets_and_125hz_sender_in_fake_clock(self):
        """理想时钟回归，不代表 Windows/真实 SDK 达到该节拍。"""
        sdk, clock = FakeSdk(), FakeClock()
        worker = session(sdk, clock)
        self.assertTrue(worker.start(sample()))
        for index in range(100):
            clock.now = 10.0 + index * 0.008
            if index and index % 5 == 0:
                self.assertTrue(worker.update_target(sample(now=clock.now)))
            self.assertTrue(worker.tick())
        self.assertEqual(worker.sent_count, 100)
        self.assertEqual(worker.send_attempt_count, 100)
        self.assertTrue(worker.active)

    def test_slow_servo_call_is_counted_then_stopped(self):
        sdk, clock = FakeSdk(), FakeClock()

        def delayed_servo_j(*args):
            clock.now += 0.005
            return sdk._call("servo_j", *args)

        sdk.servo_j = delayed_servo_j
        worker = session(sdk, clock)
        self.assertTrue(worker.start(sample()))
        self.assertFalse(worker.tick())
        self.assertEqual(worker.send_attempt_count, 1)
        self.assertEqual(worker.sent_count, 1)
        self.assertIn("耗时超限", worker.stop_reason)


if __name__ == "__main__":
    unittest.main()
