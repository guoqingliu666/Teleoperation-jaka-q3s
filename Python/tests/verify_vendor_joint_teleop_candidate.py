"""新②候选的纯离线回归；SDK 只用假对象，绝不连接真机。"""

from __future__ import annotations

import math
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from vla_lab.vendor_joint_teleop_candidate import (  # noqa: E402
    JointLpfCutoff, JointNlfLimits, OfflineVendorIkCandidate, RejectLimits, Sample,
)


class FakeSdk:
    def __init__(self, response=(0, (0.01, 0.02, 0.03, 0.04, 0.05, 0.06))):
        self.response = response
        self.calls = []

    def kine_inverse(self, reference, pose):
        self.calls.append(("kine_inverse", reference, pose))
        return self.response


def frame(**changes):
    data = dict(
        target_pose=(101.0, 200.0, 300.0, 0.0, 0.0, 0.0),
        actual_tcp_pose=(100.0, 200.0, 300.0, 0.0, 0.0, 0.0),
        actual_joints=(0.0,) * 6,
        input_time_s=9.99, feedback_time_s=9.99,
        tracked=True, grip_held=True, powered=True, enabled=True,
        estop=False, collision=False, on_limit=False, tool_id=1,
    )
    data.update(changes)
    return Sample(**data)


def candidate():
    return OfflineVendorIkCandidate(
        RejectLimits(0.1, 0.1, 5.0, 5.0, 0.2, 0.007, 1000.0, 180.0),
        clock=lambda: 10.0,
    )


class OfflineCandidateTests(unittest.TestCase):
    def test_exact_vendor_result_becomes_joint_servo_plan(self):
        sdk = FakeSdk()
        decision = candidate().evaluate(sdk, frame())
        self.assertTrue(decision.accepted)
        self.assertEqual(sdk.calls[0][1], (0.0,) * 6)
        self.assertEqual(decision.servo_call, ("servo_j", (sdk.response[1], 0, 1)))
        self.assertEqual(len(sdk.calls), 1)

    def test_filter_is_vendor_joint_nlf_with_explicit_parameters(self):
        self.assertEqual(JointNlfLimits(10, 20, 30).sdk_call(),
                         ("servo_move_use_joint_NLF", (10, 20, 30)))
        with self.assertRaises(ValueError):
            JointNlfLimits(10, 20, math.nan).sdk_call()

    def test_lpf_requires_explicit_finite_positive_cutoff(self):
        self.assertEqual(JointLpfCutoff(0.5).sdk_call(),
                         ("servo_move_use_joint_LPF", (0.5,)))
        for value in (0, -1, math.nan):
            with self.subTest(value=value), self.assertRaises(ValueError):
                JointLpfCutoff(value).sdk_call()

    def test_orientation_jump_latches_without_calling_vendor(self):
        sdk = FakeSdk()
        evaluator = candidate()
        bad = frame(target_pose=(101, 200, 300, 0, 0, math.radians(30)))
        self.assertFalse(evaluator.evaluate(sdk, bad).accepted)
        self.assertIn("姿态跳变", evaluator.latched_reason)
        self.assertFalse(evaluator.evaluate(sdk, frame()).accepted)
        self.assertEqual(sdk.calls, [])

    def test_stale_and_untracked_samples_are_rejected(self):
        for bad in (frame(input_time_s=9.0), frame(tracked=False), frame(grip_held=False)):
            with self.subTest(bad=bad):
                sdk = FakeSdk()
                self.assertFalse(candidate().evaluate(sdk, bad).accepted)
                self.assertEqual(sdk.calls, [])

    def test_robot_health_and_tool_are_required(self):
        for changes in ({"enabled": False}, {"estop": True}, {"collision": True},
                        {"on_limit": True}, {"tool_id": 0}):
            with self.subTest(changes=changes):
                sdk = FakeSdk()
                self.assertFalse(candidate().evaluate(sdk, frame(**changes)).accepted)
                self.assertEqual(sdk.calls, [])

    def test_sdk_error_and_invalid_solution_never_make_servo_plan(self):
        for response in ((-4, None), (0, (0.0,) * 5), (0, (math.nan,) * 6),
                         (0, (0.3,) * 6)):
            with self.subTest(response=response):
                result = candidate().evaluate(FakeSdk(response), frame())
                self.assertFalse(result.accepted)
                self.assertIsNone(result.servo_call)

    def test_second_frame_large_position_jump_is_not_clipped(self):
        sdk = FakeSdk()
        evaluator = candidate()
        self.assertTrue(evaluator.evaluate(sdk, frame()).accepted)
        result = evaluator.evaluate(sdk, frame(target_pose=(120, 200, 300, 0, 0, 0)))
        self.assertFalse(result.accepted)
        self.assertEqual(len(sdk.calls), 1)

    def test_slow_inverse_is_rejected_even_if_answer_is_valid(self):
        ticks = [10.0]

        class SlowFakeSdk(FakeSdk):
            def kine_inverse(self, reference, pose):
                ticks[0] += 0.009  # 模拟一次逆解耗时超过本测试的 7 ms 预算。
                return super().kine_inverse(reference, pose)

        evaluator = OfflineVendorIkCandidate(
            RejectLimits(0.1, 0.1, 5.0, 5.0, 0.2, 0.007, 1000.0, 180.0),
            clock=lambda: ticks[0],
        )
        result = evaluator.evaluate(SlowFakeSdk(), frame())
        self.assertFalse(result.accepted)
        self.assertIsNone(result.servo_call)
        self.assertIn("耗时", result.reason)

    def test_wraparound_euler_representation_is_not_a_large_physical_rotation(self):
        sdk = FakeSdk()
        evaluator = candidate()
        actual = (100.0, 200.0, 300.0, 0.0, 0.0, math.pi - 0.01)
        target = (101.0, 200.0, 300.0, 0.0, 0.0, -math.pi + 0.01)
        self.assertTrue(evaluator.evaluate(
            sdk, frame(actual_tcp_pose=actual, target_pose=target)
        ).accepted)

    def test_feedback_expiring_during_inverse_is_rejected(self):
        ticks = [10.0]

        class DelayedFakeSdk(FakeSdk):
            def kine_inverse(self, reference, pose):
                ticks[0] += 0.006
                return super().kine_inverse(reference, pose)

        evaluator = OfflineVendorIkCandidate(
            RejectLimits(0.1, 0.1, 5.0, 5.0, 0.2, 0.007, 1000.0, 180.0),
            clock=lambda: ticks[0],
        )
        result = evaluator.evaluate(
            DelayedFakeSdk(), frame(input_time_s=9.99, feedback_time_s=9.903)
        )
        self.assertFalse(result.accepted)
        self.assertIsNone(result.servo_call)
        self.assertIn("已经过期", result.reason)

    def test_incident_log_stale_feedback_latches_before_another_plan(self):
        """只回放历史 JSONL；假逆解恒返回实测关节，不读取或连接机器人。"""
        history = (
            Path(__file__).resolve().parents[2]
            / "Validation/live_engineering_teleop_sessions"
            / "level_a_20260919_214948_3af0bd8e/events.jsonl"
        )
        if not history.exists():
            self.skipTest("本机事故原始记录不随公开仓库分发")
        events = [json.loads(line) for line in history.read_text(encoding="utf-8").splitlines()]
        targets = [event for event in events if event.get("event") == "target"]
        self.assertEqual(len(targets), 158)
        ticks = [0.0]

        class RecordedJointFakeSdk:
            def kine_inverse(self, reference, pose):
                return (0, reference)

        evaluator = OfflineVendorIkCandidate(
            RejectLimits(0.1, 0.1, 1000.0, 180.0, 1.0, 0.007, 1000.0, 180.0),
            clock=lambda: ticks[0],
        )
        first_rejection = None
        for event in targets:
            ticks[0] = event["time_ns"] / 1e9
            sample = frame(
                target_pose=tuple(event["target"]),
                actual_tcp_pose=tuple(event["actual_tcp"]),
                actual_joints=tuple(event["actual_joints_rad"]),
                input_time_s=ticks[0],
                feedback_time_s=event["actual_timestamp_ns"] / 1e9,
            )
            decision = evaluator.evaluate(RecordedJointFakeSdk(), sample)
            if not decision.accepted:
                first_rejection = (event, decision)
                break
        self.assertIsNotNone(first_rejection)
        event, decision = first_rejection
        self.assertTrue(event["sent"])  # 旧事故记录此时仍在送笛卡尔目标。
        self.assertGreater(event["actual_age_ms"], 100)
        self.assertIn("机器人反馈时间戳", decision.reason)
        self.assertIsNone(decision.servo_call)

    def test_many_small_steps_cannot_escape_session_envelope(self):
        sdk = FakeSdk()
        evaluator = OfflineVendorIkCandidate(
            RejectLimits(0.1, 0.1, 10.0, 10.0, 0.2, 0.007, 10.0, 20.0),
            clock=lambda: 10.0,
        )
        self.assertTrue(evaluator.evaluate(sdk, frame()).accepted)
        self.assertTrue(evaluator.evaluate(sdk, frame(
            target_pose=(108.0, 200.0, 300.0, 0.0, 0.0, 0.0)
        )).accepted)
        result = evaluator.evaluate(sdk, frame(
            target_pose=(116.0, 200.0, 300.0, 0.0, 0.0, 0.0)
        ))
        self.assertFalse(result.accepted)
        self.assertIn("会话起点", result.reason)
        self.assertEqual(len(sdk.calls), 2)

    def test_solution_branch_jump_is_rejected_even_if_close_to_actual(self):
        sdk = FakeSdk()
        evaluator = candidate()
        self.assertTrue(evaluator.evaluate(sdk, frame()).accepted)
        sdk.response = (0, (0.25,) * 6)
        result = evaluator.evaluate(sdk, frame(actual_joints=(0.25,) * 6))
        self.assertFalse(result.accepted)
        self.assertIn("上一解", result.reason)


if __name__ == "__main__":
    unittest.main()
