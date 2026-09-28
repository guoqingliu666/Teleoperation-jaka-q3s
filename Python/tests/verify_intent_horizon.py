"""有界意图调度的压力与故障测试；没有硬件连接和运动命令。"""
from dataclasses import replace
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from vla_lab.intent_horizon import IntentHorizon
from vla_lab.adaptive_inverse import AdaptiveInverse
from vla_lab.trajectory_horizon import PreviewProfile
from vla_lab.trajectory_reference import PoseSample, angle_deg, rpy_quaternion
from verify_adaptive_sampling import Clock, Inverse, point, LIMITS, finish


class IntentTests(unittest.TestCase):
    def test_full_orientation_accepts_ordered_turn_across_half_and_full_circle(self):
        clock, h = self.new(rotation_radius_deg=180.)
        total = 0.
        # 模拟消费端及时完成，仅验证输入接纳几何，不冒充真实IK/关节可达验证。
        for degree in range(1, 361):
            clock.t += .05
            previous = h.admitted
            h.push(PoseSample(clock.t, (0., 0., 0.),
                rpy_quaternion((0., 0., math.radians(degree)))), clock.t)
            total += angle_deg(previous.q, h.admitted.q)
            self.assertAlmostEqual(angle_deg(h.admitted.q, h.raw_previous.q), 0., places=5)
            h.anchor = h.admitted
            h.samples.clear()
        self.assertAlmostEqual(total, 360., places=5)

    def test_processed_gap_has_specific_failure_even_when_latest_packet_is_fresh(self):
        clock, h = self.new()
        clock.t = .16
        with self.assertRaisesRegex(RuntimeError, "已处理样本时间间隔"):
            h.push(point(.16, 1.), .16)
        self.assertEqual(h.last_failure_context['input_failure']['receive_age_s'], 0.)
        self.assertTrue(h.blocked)

    def new(self, **kwargs):
        clock = Clock()
        h = IntentHorizon(Inverse(), LIMITS, clock=clock, adaptive=True, **kwargs)
        h.reset(point(0), (0.,)*6)
        return clock, h

    def feed(self, clock, h, x, dt=.01):
        clock.t += dt
        h.push(point(clock.t, x), clock.t)
        h.advance(permit=True)

    def test_ten_seconds_of_fast_input_with_closed_slot_is_bounded(self):
        clock, h = self.new()
        for i in range(1000):
            self.feed(clock, h, i*2.)
            self.assertLessEqual(len(h.samples), 128)
            self.assertLessEqual(h.pending_seconds(), h.capacity_s+1e-8)
        self.assertIsNotNone(h.ready)
        self.assertFalse(h.blocked)
        self.assertGreater(h.limited_frames, 0)
        self.assertGreater(h.discarded_mm, 0)
        # 候选正常等队列十秒，不借改时间戳续命；新鲜 Grip 和固定起点仍有效。
        self.assertLess(h.ready.created_s, 1.)
        self.assertIsNotNone(h.take_when_slot_open(expected_epoch=h.epoch))

    def test_compute_yields_do_not_split_one_motion_segment(self):
        clock = Clock()
        def inverse(seed, tcp):
            clock.t += .002
            return (0, (math.radians(.4*tcp[0]), 0.,0.,0.,0.,0.))
        job = AdaptiveInverse(inverse, point(0), point(.1, 10), (0.,)*6,
            LIMITS, epoch=1, clock=clock, cooperative=True)
        result = None
        while result is None:
            result = job.advance()
            clock.t += .02  # 模拟状态轮询和调度间隙，而非 SDK 超时。
        self.assertGreater(result.preparation_s, .12)
        self.assertEqual(result.completion_reason, "complete")
        self.assertFalse(result.shortened)
        self.assertEqual(result.command_tcp[0], 10.)

    def test_single_late_sdk_call_still_invalidates(self):
        clock = Clock()
        def inverse(seed, tcp):
            clock.t += .121
            return Inverse()(seed, tcp)
        job = AdaptiveInverse(inverse, point(0), point(.1, 10), (0.,)*6,
            LIMITS, epoch=1, clock=clock, cooperative=True)
        with self.assertRaisesRegex(RuntimeError, "迟到"):
            job.advance()
        self.assertTrue(job.done)
        with self.assertRaisesRegex(RuntimeError, "结束"):
            job.completed_prefix("retry")

    def test_input_timeout_still_discards_ready(self):
        clock, h = self.new()
        for _ in range(20):
            self.feed(clock, h, 5.)
        self.assertIsNotNone(h.ready)
        clock.t += .151
        with self.assertRaisesRegex(RuntimeError, "断流"):
            h.advance(permit=True)
        self.assertIsNone(h.ready)
        self.assertTrue(h.blocked)

    def test_superseded_grip_cannot_deliver(self):
        clock, h = self.new()
        for _ in range(20):
            self.feed(clock, h, 5.)
        epoch = h.epoch
        h.reset(point(clock.t, 3), (0.,)*6)
        self.assertIsNone(h.take_when_slot_open(expected_epoch=epoch))

    def test_admitted_corner_and_reverse_are_not_shortcut(self):
        clock, h = self.new()
        for t, x, y in ((.02,5,0),(.04,10,0),(.06,10,5)):
            clock.t = t
            h.push(point(t,x,y),t)
        h.advance(permit=True)
        self.assertEqual(h.job.target.xyz, (10.,0.,0.))
        while h.ready is None:
            h.advance(permit=True)
        h.take_when_slot_open(expected_epoch=h.epoch)
        self.assertEqual(h.samples[0][0].xyz, (10.,5.,0.))

    def test_saturation_does_not_catch_up_discarded_motion_after_stopping(self):
        clock, h = self.new()
        for i in range(100):
            self.feed(clock,h,i*2.)
        admitted = h.admitted.xyz
        plans = []
        for _ in range(500):
            self.feed(clock,h,198.)
            plan = h.take_when_slot_open(expected_epoch=h.epoch)
            if plan:
                plans.append(plan)
        self.assertEqual(h.admitted.xyz, admitted)
        self.assertAlmostEqual(h.anchor.xyz[0], admitted[0])
        self.assertLess(h.anchor.xyz[0],198.)
        self.assertEqual(len(h.samples),0)
        self.assertTrue(all(p.completion_reason not in ("planning_slice","backlog_slice")
                            for p in plans))

    def test_limited_reference_stays_inside_global_range(self):
        clock, h = self.new(position_radius_mm=15.,rotation_radius_deg=10.)
        for i in range(1,100):
            clock.t += .01
            h.push(PoseSample.from_tcp(clock.t,(i,0,0,0,0,math.radians(i))),clock.t)
            h.advance(permit=True)
            h.take_when_slot_open(expected_epoch=h.epoch)
            self.assertLessEqual(math.dist(h.origin.xyz,h.admitted.xyz),15.+1e-8)
            self.assertLessEqual(angle_deg(h.origin.q,h.admitted.q),10.+1e-8)

    def test_regrip_does_not_move_workspace_center(self):
        clock,h=self.new(position_radius_mm=15.)
        h.workspace_origin=point(0)
        clock.t=.1
        h.reset(point(.1,14.),(.014,0.,0.,0.,0.,0.))
        for i in range(20):
            self.feed(clock,h,14.+i)
            h.take_when_slot_open(expected_epoch=h.epoch)
        self.assertEqual(h.origin.xyz,(0.,0.,0.))
        self.assertLessEqual(h.admitted.xyz[0],15.+1e-8)

if __name__ == "__main__":
    unittest.main()
