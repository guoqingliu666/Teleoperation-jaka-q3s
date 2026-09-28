"""动态采样的反例与故障注入；假 SDK 仅验证逻辑，不代表真实 JAKA 动力学。"""

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab.adaptive_sampling import AdaptiveSampling, select_keypoint
from vla_lab.adaptive_inverse import AdaptiveInverse
from vla_lab.trajectory_horizon import TimeHorizon, IncrementalInverse
from vla_lab.trajectory_reference import PoseSample, rpy_quaternion, angle_deg
from vla_lab.sampled_follow import RejectedPath
from vla_lab.adaptive_replay import compare_sampling

LIMITS = ((-360.0, 360.0),) * 6


def point(t, x=0.0, y=0.0, deg=0.0):
    return PoseSample(t, (x, y, 0.0), rpy_quaternion((0.0, 0.0, math.radians(deg))))


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class Inverse:
    def __init__(self):
        self.calls = []

    def __call__(self, seed, tcp):
        self.calls.append((tuple(seed), tuple(tcp)))
        return (0, (tcp[0] * 0.001, tcp[1] * 0.001, 0.0, 0.0, 0.0, tcp[5]))


def finish(job):
    for _ in range(1000):
        result = job.advance()
        if result is not None:
            return result
    raise AssertionError("逆解任务没有在次数上限内结束")


class AdaptiveTests(unittest.TestCase):
    def test_constant_high_gain_terminates_at_joint_resolution(self):
        # 平稳但放大比高，二分前后斜率不变；无需一直逼近最小空间间隔。
        def linear(seed, tcp):
            return (0, (math.radians(.4 * tcp[0]), 0, 0, 0, 0, 0))

        args = (linear, point(0), point(.1, 10), (0.,) * 6, LIMITS)
        old = finish(AdaptiveInverse(*args, epoch=1,
                     policy=replace(AdaptiveSampling(), gain_resolved_step_deg=1e-9)))
        new = finish(AdaptiveInverse(*args, epoch=1))
        self.assertLess(new.inverse_calls, old.inverse_calls)
        self.assertLess(len(new.solutions), len(old.solutions))
        self.assertEqual(new.command_tcp, old.command_tcp)
        self.assertEqual(new.solutions[-1], old.solutions[-1])
        self.assertLessEqual(new.largest_sample_step_deg, .25 + 1e-10)
        self.assertLessEqual(max(b-a for a,b in zip((0.,)+new.fractions, new.fractions))*10, 5)

    def test_gain_resolution_cannot_exceed_hard_joint_step(self):
        with self.assertRaises(ValueError):
            replace(AdaptiveSampling(), gain_resolved_step_deg=3.)

    def test_high_gain_finishes_within_unchanged_budget_when_calls_cost_2ms(self):
        class WholeSegment(AdaptiveInverse):
            def _budget_prefix_ready(self):
                return False
        def run(policy):
            clock = Clock()
            def inverse(seed, tcp):
                clock.t += .002
                return (0, (math.radians(.4 * tcp[0]), 0, 0, 0, 0, 0))
            return finish(WholeSegment(inverse, point(0), point(.1, 10),
                          (0.,)*6, LIMITS, epoch=1, clock=clock, policy=policy))
        with self.assertRaisesRegex(RuntimeError, "迟到|时间预算"):
            run(replace(AdaptiveSampling(), gain_resolved_step_deg=1e-9))
        plan = run(AdaptiveSampling())
        self.assertLess(plan.preparation_s, .12)
        self.assertEqual(plan.target.xyz, (10., 0., 0.))

    def test_budget_slice_returns_only_completed_interval_and_retains_remainder(self):
        clock = Clock()
        def inverse(seed, tcp):
            clock.t += .004
            return (0, (math.radians(.4*tcp[0]), 0, 0, 0, 0, 0))
        h = TimeHorizon(inverse, LIMITS, clock=clock, adaptive=True)
        h.reset(point(0), (0.,)*6)
        clock.t = .02
        h.push(point(.02, 10), .02)
        clock.t = .101
        while h.ready is None:
            # 新鲜握持输入不能刷新旧目标的年龄。
            h.push(point(clock.t, 10), clock.t)
            h.advance(permit=True)
        result = h.take_when_slot_open(expected_epoch=h.epoch)
        self.assertEqual(result.completion_reason, "planning_slice")
        self.assertTrue(result.shortened)
        self.assertLess(result.target.xyz[0], 10)
        self.assertEqual(h.samples[0][0].xyz[0], 10)
        self.assertEqual(h.samples[0][1], .02)
        self.assertEqual(result.command_tcp[0], result.target.xyz[0])
        self.assertAlmostEqual(result.sample_times()[-1], result.target.t)

    def test_soft_budget_does_not_accept_late_sdk_result(self):
        clock = Clock()
        calls = []
        def inverse(seed, tcp):
            calls.append(tcp)
            clock.t += .002 if len(calls) <= 2 else .2
            return Inverse()(seed, tcp)
        job = AdaptiveInverse(inverse, point(0), point(.1, 20), (0.,)*6,
                              LIMITS, epoch=1, clock=clock)
        job.advance(); job.advance()
        self.assertTrue(job.solutions)
        with self.assertRaisesRegex(RuntimeError, "迟到"):
            job.advance()

    def test_backlog_cannot_reissue_expired_completed_prefix(self):
        clock = Clock()
        def inverse(seed, tcp):
            clock.t += .01
            return (0, (math.radians(.4 * tcp[0]), 0, 0, 0, 0, 0))
        h = TimeHorizon(inverse, LIMITS, clock=clock, adaptive=True)
        h.reset(point(0), (0.,)*6)
        clock.t = .02
        h.push(point(.02, 10), .02)
        clock.t = .101
        # 先形成至少一个完整检查区间，但不等软预算主动交付。
        h.job = AdaptiveInverse(inverse, point(0), point(.1, 10), (0.,)*6,
                                LIMITS, epoch=h.epoch, clock=clock)
        while not h.job._at_checked_boundary:
            h.job.advance()
        clock.t = .15
        h.push(point(.15, 10), .15)
        clock.t = .20
        h.push(point(.20, 10), .20)
        clock.t = .32
        h.push(point(.32, 10), .32)  # 新鲜心跳不改变最老待消费点的年龄。
        with self.assertRaisesRegex(RuntimeError, "时间预算"):
            h.advance(permit=True)
        self.assertIsNone(h.ready)

    def test_backlog_without_complete_prefix_still_pauses(self):
        clock = Clock()
        h = TimeHorizon(Inverse(), LIMITS, clock=clock, adaptive=True)
        h.reset(point(0), (0.,)*6)
        clock.t = .02
        h.push(point(.02, 10), .02)
        h.job = AdaptiveInverse(h.inverse, point(0), point(.1, 10), (0.,)*6,
                                LIMITS, epoch=h.epoch, clock=clock)
        clock.t = .15
        h.push(point(.15, 10), .15)
        clock.t = .20
        h.push(point(.20, 10), .20)
        clock.t = .32
        h.push(point(.32, 10), .32)
        with self.assertRaisesRegex(RuntimeError, "时间预算"):
            h.advance(permit=True)
        self.assertIsNone(h.ready)

    def test_early_refinement_saves_calls_without_changing_accepted_path(self):
        class PreviousOrder(AdaptiveInverse):
            # 复现旧顺序：先算右端点，再用完整区间判定是否细分。
            def _midpoint_requires_refinement(self, pose, solved):
                return False, False

        def amplified(seed, tcp):
            return (0, (math.radians(tcp[0]), 0, 0, 0, 0, 0))

        args = (amplified, point(0), point(.1, 10), (0.,) * 6, LIMITS)
        before = finish(PreviousOrder(*args, epoch=1))
        after = finish(AdaptiveInverse(*args, epoch=1))
        self.assertLess(after.inverse_calls, before.inverse_calls)
        self.assertEqual(after.fractions, before.fractions)
        self.assertEqual(after.solutions, before.solutions)
        self.assertEqual(after.command_tcp, before.command_tcp)
        self.assertEqual(after.subdivisions, before.subdivisions)
        self.assertEqual(before.inverse_calls - after.inverse_calls, after.avoided_endpoint_calls)

    def test_restart_after_idle_collects_a_window_instead_of_single_frame(self):
        clock, ik = Clock(), Inverse()
        h = TimeHorizon(ik, LIMITS, clock=clock, adaptive=True)
        h.reset(point(0), (0.0,) * 6)
        # 实际接收心跳保持新鲜，旧锚点已经静止了一秒。
        for i in range(1, 11):
            clock.t = i * 0.1
            h.push(point(clock.t), clock.t)
            h.advance(permit=True)
        for t, x in ((1.02, 1), (1.04, 2), (1.06, 3)):
            clock.t = t
            h.push(point(t, x), t)
            h.advance(permit=True)
        self.assertEqual(ik.calls, [])
        clock.t = 1.101
        h.advance(permit=True)
        self.assertEqual(len(ik.calls), 1)
        self.assertEqual(h.job.target.xyz[0], 3)
        h.advance(permit=True)
        plan = h.take_when_slot_open(expected_epoch=h.epoch)
        self.assertIsNotNone(plan)
        self.assertEqual(plan.target.t, 1.06)

    def test_visible_corner_prepares_before_merge_timer_without_cutting_corner(self):
        clock, ik = Clock(), Inverse()
        h = TimeHorizon(ik, LIMITS, clock=clock, adaptive=True)
        h.reset(point(0), (0.0,) * 6)
        for p in (point(.02, 5), point(.04, 10), point(.06, 10, 5)):
            clock.t = p.t
            h.push(p, p.t)
        h.advance(permit=True)
        self.assertEqual(h.job.target.xyz, (10, 0, 0))
        h.advance(permit=True)
        h.take_when_slot_open(expected_epoch=h.epoch)
        self.assertEqual(h.samples[0][0].xyz, (10, 5, 0))
        self.assertEqual(h.samples[0][1], .06)

    def test_span_limit_prepares_early_and_small_stationary_tail_flushes(self):
        for x, expected_calls in ((80., 1), (.1, 0)):
            clock, ik = Clock(), Inverse()
            h = TimeHorizon(ik, LIMITS, clock=clock, adaptive=True)
            h.reset(point(0), (0.0,) * 6)
            clock.t = .02
            h.push(point(.02, x), .02)
            h.advance(permit=True)
            self.assertEqual(len(ik.calls), expected_calls)
            if x == .1:
                clock.t = .101
                h.advance(permit=True)
                h.advance(permit=True)
                self.assertEqual(h.ready.target.xyz[0], .1)
                self.assertEqual(h.ready.target.t, .02)

    def test_smooth_rotation_uses_fewer_vendor_calls(self):
        fixed, adaptive = Inverse(), Inverse()
        args = (point(0), point(0.08, deg=4), (0.0,) * 6, LIMITS)
        old = finish(IncrementalInverse(fixed, *args, epoch=1))
        new = finish(AdaptiveInverse(adaptive, *args, epoch=1))
        self.assertEqual(len(fixed.calls), 20)
        self.assertEqual(len(adaptive.calls), 4)
        self.assertEqual(old.solutions[-1], new.solutions[-1])
        self.assertEqual(new.subdivisions, 0)
        self.assertEqual(new.sample_times(), (0.02, 0.04, 0.06, 0.08))

    def test_each_tick_calls_at_most_one_inverse_and_uses_previous_seed(self):
        ik = Inverse()
        job = AdaptiveInverse(ik, point(0), point(0.1, 20), (0.0,) * 6, LIMITS, epoch=1)
        while not job.done:
            before = len(ik.calls)
            job.advance()
            self.assertLessEqual(len(ik.calls) - before, 1)
        for (seed, tcp), (next_seed, _) in zip(ik.calls, ik.calls[1:]):
            self.assertAlmostEqual(next_seed[0], tcp[0] * 0.001)

    def test_local_joint_curvature_triggers_subdivision(self):
        calls = []

        def curved(seed, tcp):
            calls.append((seed, tcp))
            return (0, (math.radians(0.06 * tcp[0] ** 2), 0.0, 0.0, 0.0, 0.0, 0.0))

        plan = finish(
            AdaptiveInverse(
                curved, point(0), point(0.1, 10), (0.0,) * 6, LIMITS, epoch=1
            )
        )
        self.assertGreater(plan.subdivisions, 0)
        self.assertGreater(len(calls), 2)
        self.assertAlmostEqual(plan.solutions[-1][0], math.radians(6))
        self.assertEqual(sorted(set(plan.fractions)), list(plan.fractions))

    def test_endpoint_recomputed_after_new_predecessor(self):
        history = []

        def curved(seed, tcp):
            history.append((seed[0], tcp[0]))
            return (0, (math.radians(0.06 * tcp[0] ** 2), 0.0, 0.0, 0.0, 0.0, 0.0))

        finish(
            AdaptiveInverse(
                curved, point(0), point(0.1, 10), (0.0,) * 6, LIMITS, epoch=1,
                # 本例专测需要右端点才能确定的曲率，不由中点放大提前细分。
                policy=replace(AdaptiveSampling(), joint_gain_deg=100.),
            )
        )
        seeds = [seed for seed, x in history if abs(x - 10) < 1e-10]
        self.assertGreater(len(seeds), 1)
        self.assertNotEqual(seeds[0], seeds[-1])

    def test_discontinuous_branch_is_never_wrapped_or_accepted(self):
        def jump(seed, tcp):
            return (0, (0.0 if tcp[0] < 5 else 2 * math.pi, 0, 0, 0, 0, 0))

        with self.assertRaises(RejectedPath):
            finish(
                AdaptiveInverse(
                    jump, point(0), point(0.1, 10), (0.0,) * 6, LIMITS, epoch=1
                )
            )

    def test_midpoint_limit_violation_rejected_even_when_endpoints_clear(self):
        def bump(seed, tcp):
            return (
                0,
                (math.radians(359 * math.sin(math.pi * tcp[0] / 10)), 0, 0, 0, 0, 0),
            )

        with self.assertRaises(RejectedPath):
            finish(
                AdaptiveInverse(
                    bump, point(0), point(0.1, 10), (0.0,) * 6, LIMITS, epoch=1
                )
            )

    def test_calls_and_latency_bounded(self):
        ik = Inverse()
        job = AdaptiveInverse(
            ik,
            point(0),
            point(0.1, 20),
            (0.0,) * 6,
            LIMITS,
            epoch=1,
            policy=replace(AdaptiveSampling(), max_calls=1),
        )
        with self.assertRaises(RejectedPath):
            finish(job)
        self.assertEqual(len(ik.calls), 1)
        clock = Clock()

        def slow(seed, tcp):
            clock.t += 0.2
            return Inverse()(seed, tcp)

        with self.assertRaisesRegex(RuntimeError, "迟到"):
            finish(
                AdaptiveInverse(
                    slow,
                    point(0),
                    point(0.1, 2),
                    (0.0,) * 6,
                    LIMITS,
                    epoch=1,
                    clock=clock,
                )
            )

    def test_sdk_errors_keep_original_classification(self):
        for code, kind in ((-4, RejectedPath), (-3, RuntimeError)):
            with self.assertRaises(kind):
                finish(
                    AdaptiveInverse(
                        lambda *_: (code,),
                        point(0),
                        point(0.1, 1),
                        (0.0,) * 6,
                        LIMITS,
                        epoch=1,
                    )
                )

    def test_sampling_preserves_corner_reverse_and_full_turn(self):
        args = dict(max_translation_mm=80, max_rotation_deg=4)
        straight = [point(i * 0.02, i) for i in range(1, 8)]
        self.assertEqual(select_keypoint(point(0), straight, **args), straight[-1])
        reverse = [point(0.02, 4), point(0.04, 8), point(0.06, 4), point(0.08, 0)]
        self.assertEqual(select_keypoint(point(0), reverse, **args), reverse[1])
        corner = [
            point(0.02, 5),
            point(0.04, 10),
            point(0.06, 10, 5),
            point(0.08, 10, 10),
        ]
        self.assertEqual(select_keypoint(point(0), corner, **args), corner[1])
        raw = [point(i * 0.02, deg=i * 2) for i in range(181)]
        anchor = raw.pop(0)
        arc = 0.0
        while raw:
            target = select_keypoint(anchor, raw, **args)
            arc += angle_deg(anchor.q, target.q)
            raw = [p for p in raw if p.t > target.t]
            anchor = target
        self.assertAlmostEqual(arc, 360, places=6)

    def test_replay_preserves_turn_without_calling_sdk(self):
        raw = [point(i * 0.02, deg=i * 2) for i in range(181)]
        events = [
            dict(state="trajectory_raw_sample", t=p.t, xyz=p.xyz, q=p.q, epoch=1)
            for p in raw
        ]
        report = compare_sampling(events)
        self.assertEqual(report["geometry_failures"], [])
        self.assertFalse(report["sdk_called"])
        self.assertAlmostEqual(report["retained_rotation_arc_deg"], 360, places=6)
        self.assertLess(
            report["same_segments_adaptive_initial_check_count"],
            report["same_segments_fixed_check_count"],
        )

    def test_backlog_pauses_before_hard_expiry_without_refreshing_time(self):
        clock = Clock()
        ik = Inverse()
        h = TimeHorizon(ik, LIMITS, clock=clock, adaptive=True)
        h.reset(point(0), (0.0,) * 6)
        for t, x in ((0.02, 1), (0.1, 2), (0.2, 3), (0.3, 4)):
            clock.t = t
            h.push(point(t, x), t)
        clock.t = 0.325
        with self.assertRaisesRegex(RejectedPath, "提前暂停"):
            h.advance(permit=True)
        self.assertTrue(h.blocked)
        self.assertGreater(h.last_failure_context["oldest_received_age_s"], 0.3)
        self.assertLess(h.last_failure_context["oldest_received_age_s"], 0.4)
        self.assertEqual(ik.calls, [])

    def test_partial_path_uses_nonuniform_time_and_preserves_remainder(self):
        ik = lambda seed, tcp: (0, (math.radians(tcp[0]), 0, 0, 0, 0, 0))
        plan = finish(
            AdaptiveInverse(ik, point(0), point(0.1, 20), (0.0,) * 6, LIMITS, epoch=1)
        )
        self.assertTrue(plan.shortened)
        self.assertLessEqual(math.degrees(plan.solutions[-1][0]), 12)
        self.assertAlmostEqual(plan.sample_times()[-1], plan.target.t)
        self.assertLess(plan.target.t, plan.source_target.t)


if __name__ == "__main__":
    unittest.main()
