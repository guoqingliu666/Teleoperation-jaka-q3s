"""新轨迹外围离线回归：不导入 jkrc、不连接网络、不发送运动。"""
from pathlib import Path
import math
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from vla_lab.trajectory_reference import (
    PoseSample, RotationHistory, StreamingReference, C2Reference, adaptive_knots,
    angle_deg, rpy_quaternion,
)
from vla_lab.trajectory_horizon import (
    PreviewProfile, IncrementalInverse, TimeHorizon, corner_speed_preview,
)
from vla_lab.sampled_follow import RejectedPath


LIMITS = ((-360., 360.),)*6


def sample(t, x=0., y=0., z=0., deg=0.):
    return PoseSample(t, (x, y, z), rpy_quaternion((0., 0., math.radians(deg))))


class Clock:
    def __init__(self): self.t = 0.
    def __call__(self): return self.t


class InverseOnly:
    """仅供逻辑测试的假厂商回调，不是机器人逆解，也不提供运动方法。"""
    def __init__(self, clock=None, delay=0., gain=.001):
        self.calls = 0
        self.clock, self.delay, self.gain = clock, delay, gain
        self.return_code = 0

    def __call__(self, reference, tcp):
        self.calls += 1
        if self.clock:
            self.clock.t += self.delay
        if self.return_code:
            return (self.return_code,)
        return (0, (tcp[0]*self.gain, tcp[1]*self.gain, tcp[2]*self.gain, 0., 0., tcp[5]))


class ReferenceTests(unittest.TestCase):
    def test_nan_and_zero_quaternion_rejected(self):
        for p in [(0., (float("nan"),0.,0.), (0,0,0,1)), (0., (0,0,0), (0,0,0,0))]:
            with self.assertRaises(ValueError): PoseSample(*p)

    def test_full_turn_survives_quaternion_sign_flips(self):
        h = RotationHistory()
        for i in range(73):
            p = sample(i*.02, deg=i*5.)
            if i % 2:
                p = replace(p, q=tuple(-x for x in p.q))
            h.push(p)
        self.assertAlmostEqual(h.twist_deg, 360., places=8)
        self.assertAlmostEqual(h.arc_deg, 360., places=8)
        self.assertLess(angle_deg(h.previous.q, sample(0).q), 1e-8)

    def test_reverse_twist_and_three_grips(self):
        h = RotationHistory()
        for _ in range(3):
            h.reset()
            for i in range(73): h.push(sample(i*.02, deg=-i*5))
            self.assertAlmostEqual(h.twist_deg, -360., places=8)

    def test_jump_latches_instead_of_adopting_bad_baseline(self):
        h = RotationHistory()
        h.push(sample(0))
        with self.assertRaises(ValueError): h.push(sample(.02, deg=90))
        with self.assertRaises(ValueError): h.push(sample(.04, deg=91))
        h.reset()
        h.push(sample(.04, deg=91))
        self.assertEqual(h.arc_deg, 0.)

    def test_out_of_order_duplicate_and_gap_rejected(self):
        for t in [0., -.1, .2]:
            h = RotationHistory()
            h.push(sample(0))
            with self.assertRaises(ValueError): h.push(sample(t))

    def test_sampling_reduces_straight_line_and_keeps_corner(self):
        raw = [sample(i*.01, min(i,20), max(0,i-20)) for i in range(41)]
        knots = adaptive_knots(raw, position_error_mm=.1)
        self.assertLess(len(knots), len(raw)/2)
        self.assertTrue(any(p.xyz == (20.,0.,0.) for p in knots))

    def test_sampling_does_not_collapse_turn_to_equal_endpoints(self):
        raw = [sample(i*.01, deg=5*i) for i in range(73)]
        knots = adaptive_knots(raw, max_arc_deg=15.)
        self.assertGreaterEqual(len(knots), 25)
        self.assertAlmostEqual(sum(angle_deg(a.q,b.q) for a,b in zip(knots,knots[1:])),360.)

    def test_c2_analytic_continuity_at_all_knots(self):
        points = [sample(i*.1, i*3., math.sin(i)*2., deg=i*5) for i in range(8)]
        curve = C2Reference(points)
        for name, error in curve.continuity_errors().items():
            self.assertLess(error, 1e-7, name)
        # 内部结点不是每段零速起步。
        self.assertGreater(math.hypot(*curve.spans[2].evaluate(points[2].t).velocity),1.)
        self.assertLess(math.hypot(*curve.spans[0].evaluate(points[0].t).velocity),1e-8)

    def test_nonuniform_time_and_pi_crossing(self):
        points = [sample(t, i, deg=170+i*5) for i,t in enumerate((0.,.03,.08,.1,.19,.25))]
        for error in C2Reference(points).continuity_errors().values():
            self.assertLess(error, 1e-6)

    def test_streaming_preserves_derivatives_not_zero_at_every_window(self):
        stream, spans = StreamingReference(), []
        for i in range(80):
            span = stream.push(sample(i*.02, i*.5, deg=i*5))
            if span: spans.append(span)
            self.assertLessEqual(len(stream.pending), 2)
        spans.append(stream.finish())
        for a,b in zip(spans, spans[1:]):
            x,y = a.evaluate(a.right.t), b.evaluate(b.left.t)
            self.assertLess(math.dist(x.velocity,y.velocity),1e-7)
            self.assertLess(math.dist(x.acceleration,y.acceleration),1e-6)
            self.assertLess(math.dist(x.angular_velocity,y.angular_velocity),1e-7)
            self.assertLess(math.dist(x.angular_acceleration,y.angular_acceleration),1e-6)
            self.assertGreater(math.hypot(*x.velocity), 1.)

    def test_stream_reset_discards_old_grip(self):
        stream = StreamingReference()
        stream.push(sample(0))
        stream.push(sample(.02,1))
        stream.reset()
        self.assertIsNone(stream.push(sample(10,100)))
        self.assertEqual(stream.history.arc_deg,0)

    def test_no_extrapolation(self):
        span = C2Reference([sample(0),sample(.1,1)]).spans[0]
        with self.assertRaises(ValueError): span.evaluate(.2)

    def test_curve_control_hull_respects_deviation_budgets(self):
        points=[sample(i*.04, min(i,2)*20, max(0,i-2)*20, deg=i*10) for i in range(6)]
        curve=C2Reference(points,position_deviation_mm=.5,orientation_deviation_deg=.2)
        bounds=curve.derivative_bounds()
        self.assertLessEqual(bounds["position_deviation_mm"],.5)
        self.assertLessEqual(bounds["orientation_deviation_deg"],.2)
        for value in curve.continuity_errors().values(): self.assertLess(value,1e-6)

    def test_time_scaling_bounds_both_translation_and_orientation(self):
        curve=C2Reference([sample(i*.02,i*10,deg=i*5) for i in range(8)])
        scale=curve.required_time_scale(speed_mm_s=200,acceleration_mm_s2=500,
                                       angular_speed_deg_s=45,angular_acceleration_deg_s2=180)
        bounds=curve.derivative_bounds()
        self.assertGreater(scale,1.)
        self.assertLessEqual(bounds["speed_mm_s"]/scale,200+1e-8)
        self.assertLessEqual(bounds["acceleration_mm_s2"]/scale**2,500+1e-8)
        self.assertLessEqual(bounds["angular_speed_deg_s"]/scale,45+1e-8)
        self.assertLessEqual(bounds["angular_acceleration_deg_s2"]/scale**2,180+1e-8)


class PlanningTests(unittest.TestCase):
    def finish(self, job):
        for _ in range(1000):
            result = job.advance()
            if result: return result
        self.fail("planning failed to terminate")

    def test_translation_rotation_share_fraction(self):
        ik = InverseOnly()
        plan = self.finish(IncrementalInverse(ik, sample(0), sample(.1,80,deg=8),
                                             (0.,)*6,LIMITS,epoch=1))
        self.assertAlmostEqual(plan.target.xyz[0],40.)
        self.assertAlmostEqual(angle_deg(plan.start.q,plan.target.q),4.)
        self.assertEqual(ik.calls,20)

    def test_one_ik_per_tick_and_no_motion_capability(self):
        ik = InverseOnly()
        job = IncrementalInverse(ik,sample(0),sample(.1,20),(0.,)*6,LIMITS,epoch=1)
        for i in range(10):
            job.advance()
            self.assertEqual(ik.calls,i+1)
        self.assertFalse(hasattr(job,"linear_move_extend_ori"))

    def test_cumulative_limit_shortens_without_retrying_whole_path(self):
        ik = InverseOnly(gain=.01)
        plan = self.finish(IncrementalInverse(ik,sample(0),sample(.1,40),(0.,)*6,LIMITS,epoch=1))
        self.assertTrue(plan.shortened)
        self.assertAlmostEqual(plan.target.xyz[0],20.)
        self.assertEqual(ik.calls,11)

    def test_shortened_chord_does_not_backtrack_to_old_nonuniform_samples(self):
        clock=Clock()
        h=TimeHorizon(InverseOnly(clock,.001,gain=.01),LIMITS,clock=clock)
        h.reset(sample(0),(0.,)*6)
        for t,x in ((.045,5),(.051,6),(.09,40)):
            clock.t=t; h.push(sample(t,x),t)
        for _ in range(11): h.advance(permit=True)
        plan=h.take_preview(0.,permit=True,expected_epoch=h.epoch)
        self.assertTrue(plan.shortened)
        self.assertAlmostEqual(plan.target.xyz[0],20.)
        self.assertEqual(len(h.samples),1)
        self.assertEqual(h.samples[0][0].xyz[0],40.)
        # 禁止给残余目标刷新接收时间来绕过积压保护。
        self.assertEqual(h.samples[0][1],.045)
        clock.t=.14
        h.advance(permit=True)
        self.assertGreater(h.job.target.xyz[0],h.anchor.xyz[0])

    def test_expired_residual_stays_expired_and_keeps_failure_context(self):
        clock=Clock()
        h=TimeHorizon(InverseOnly(clock,.001,gain=.01),LIMITS,clock=clock)
        h.reset(sample(0),(0.,)*6)
        clock.t=.09; h.push(sample(.09,40),.09)
        for _ in range(11): h.advance(permit=True)
        h.take_preview(0.,permit=True,expected_epoch=h.epoch)
        for t in (.2,.3,.4):
            clock.t=t; h.push(sample(t,40),t)
        clock.t=.5
        with self.assertRaisesRegex(RuntimeError,"过期"):
            h.advance(permit=True)
        self.assertGreater(h.last_failure_context["oldest_received_age_s"],.4)
        self.assertEqual(len(h.samples),0)

    def test_neighbor_jump_never_returns_prefix_as_accepted(self):
        with self.assertRaises(RejectedPath):
            self.finish(IncrementalInverse(InverseOnly(gain=.1),sample(0),sample(.1,10),
                                           (0.,)*6,LIMITS,epoch=1))

    def test_sdk_fault_is_not_projected_as_unreachable(self):
        for code, error in [(-4,RejectedPath),(-3,RuntimeError)]:
            ik=InverseOnly(); ik.return_code=code
            job=IncrementalInverse(ik,sample(0),sample(.1,1),(0.,)*6,LIMITS,epoch=1)
            with self.assertRaises(error): job.advance()
            self.assertTrue(job.done)
            self.assertEqual(ik.calls,1)

    def test_no_joint_wrapping_to_hide_branch_flip(self):
        def inverse(ref, tcp): return (0,(2*math.pi,0,0,0,0,0))
        with self.assertRaises(RejectedPath):
            self.finish(IncrementalInverse(inverse,sample(0),sample(.1,1),(0.,)*6,LIMITS,epoch=1))

    def test_late_inverse_result_is_discarded(self):
        clock=Clock()
        job=IncrementalInverse(InverseOnly(clock,.2),sample(0),sample(.1,1),(0.,)*6,LIMITS,
                               epoch=1,clock=clock)
        with self.assertRaisesRegex(RuntimeError,"迟到"): job.advance()

    def test_joint_limit_margin_is_not_removed(self):
        def inverse(ref,tcp): return (0,(math.radians(359),0,0,0,0,0))
        joints=(math.radians(356),0,0,0,0,0)
        with self.assertRaises(RejectedPath):
            self.finish(IncrementalInverse(inverse,sample(0),sample(.1,1),joints,LIMITS,epoch=1))

    def new_horizon(self):
        clock=Clock(); ik=InverseOnly(clock,.001)
        h=TimeHorizon(ik,LIMITS,clock=clock)
        h.reset(sample(0),(0.,)*6)
        return h,clock,ik

    def test_small_tail_flushes_without_20mm_or_2deg_threshold(self):
        h,clock,ik=self.new_horizon()
        clock.t=.02; h.push(sample(.02,1),clock.t)
        h.advance(permit=True)
        self.assertEqual(ik.calls,0)
        clock.t=.11; h.advance(permit=True)
        plan=h.take_preview(0.,permit=True,expected_epoch=h.epoch)
        self.assertIsNotNone(plan)
        self.assertAlmostEqual(plan.target.xyz[0],1.)

    def test_prepare_while_execution_has_time_then_take_when_low(self):
        h,clock,ik=self.new_horizon()
        clock.t=.09; h.push(sample(.09,6),clock.t)
        for _ in range(3): h.advance(permit=True)
        self.assertEqual(ik.calls,3)
        self.assertIsNone(h.take_preview(.8,permit=True,expected_epoch=h.epoch))
        self.assertIsNotNone(h.take_preview(.02,permit=True,expected_epoch=h.epoch))

    def test_unknown_remaining_time_does_not_assume_safe_to_send(self):
        h,clock,ik=self.new_horizon()
        clock.t=.09; h.push(sample(.09,1),clock.t); h.advance(permit=True)
        self.assertIsNone(h.take_preview(None,permit=True,expected_epoch=h.epoch))

    def test_old_epoch_cannot_replay_on_regrip(self):
        h,clock,ik=self.new_horizon()
        old=h.epoch
        clock.t=.09; h.push(sample(.09,1),clock.t); h.advance(permit=True)
        h.reset(sample(.1,1),(.001,0,0,0,0,0))
        self.assertIsNone(h.take_preview(0.,permit=True,expected_epoch=old))
        self.assertTrue(h.blocked)

    def test_permission_loss_clears_all_prepared_work(self):
        h,clock,ik=self.new_horizon()
        clock.t=.09; h.push(sample(.09,1),clock.t); h.advance(permit=True)
        h.advance(permit=False)
        self.assertIsNone(h.ready)
        self.assertEqual(len(h.samples),0)

    def test_stale_input_rejected_before_ik(self):
        h,clock,ik=self.new_horizon()
        clock.t=.5
        with self.assertRaises(RuntimeError): h.push(sample(.01,1),.01)
        self.assertEqual(ik.calls,0)

    def test_future_reference_with_fresh_envelope_is_rejected(self):
        h,clock,ik=self.new_horizon()
        clock.t=.1
        with self.assertRaisesRegex(RuntimeError,"时间不一致"):
            h.push(sample(5.,1),clock.t)
        self.assertEqual(ik.calls,0)

    def test_old_plan_not_refreshed_by_new_heartbeat(self):
        h,clock,ik=self.new_horizon()
        clock.t=.09; h.push(sample(.09,1),clock.t); h.advance(permit=True)
        clock.t=.2; h.push(sample(.2,1),clock.t)
        clock.t=.3; h.push(sample(.3,1),clock.t)
        self.assertIsNone(h.take_preview(0.,permit=True,expected_epoch=h.epoch))

    def test_backlog_does_not_silently_drop_middle_of_turn(self):
        h,clock,ik=self.new_horizon()
        with self.assertRaisesRegex(RuntimeError,"积压"):
            for i in range(20):
                clock.t=.05*(i+1); h.push(sample(clock.t,deg=10*i),clock.t)
        self.assertTrue(h.blocked)

    def test_curvature_slows_but_does_not_claim_vendor_c2(self):
        p=corner_speed_preview(sample(0),sample(.1,20),sample(.2,20,20))
        self.assertGreater(p["speed_mm_s"],0)
        self.assertLess(p["speed_mm_s"],200)
        self.assertIn("未核实",p["reason"])

    def test_reversal_requires_stop_and_pure_rotation_is_unknown(self):
        p=corner_speed_preview(sample(0),sample(.1,20),sample(.2,0))
        self.assertEqual(p["speed_mm_s"],0)
        p=corner_speed_preview(sample(0),sample(.1,deg=1),sample(.2,deg=2))
        self.assertIsNone(p["radius_mm"])


if __name__ == "__main__":
    unittest.main()
