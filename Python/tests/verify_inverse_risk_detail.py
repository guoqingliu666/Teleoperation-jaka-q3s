"""路径拒绝诊断的离线测试：只使用假逆解，不连接机器人。"""
import math
import unittest

from verify_adaptive_sampling import AdaptiveInverse, Clock, LIMITS, point, finish
from vla_lab.sampled_follow import RejectedPath
from vla_lab.intent_horizon import IntentHorizon


class RiskDetailTests(unittest.TestCase):
    def test_known_margin_violation_rejects_without_repeated_subdivision(self):
        def inverse(seed, tcp):
            return (0, (0.,0.,0.,math.radians(358),0.,0.))
        job = AdaptiveInverse(inverse, point(0), point(.1,1),
                              (0.,)*6, LIMITS, epoch=1, clock=Clock())
        with self.assertRaisesRegex(RejectedPath, 'J4余量'):
            job.advance()
        self.assertEqual(job.calls, 1)
        self.assertEqual(job.solutions, [])
        self.assertTrue(job.risk_detail['early_margin_rejection'])

    def test_discontinuous_solution_identifies_axis_without_releasing_path(self):
        def inverse(seed, tcp):
            return (0, (0., 0., 0., math.radians(20), 0., 0.))
        job = AdaptiveInverse(inverse, point(0), point(.1, 1),
                              (0.,)*6, LIMITS, epoch=1, clock=Clock())
        with self.assertRaises(RejectedPath):
            finish(job)
        self.assertIn('joint_step', job.risk_detail['reasons'])
        self.assertEqual(job.risk_detail['largest_step_axis'], 4)
        self.assertIsNone(job.risk_detail['joint_curvature_deg'])
        self.assertTrue(job.done)

    def test_margin_and_curvature_have_separate_axis_values(self):
        job = AdaptiveInverse(lambda seed,tcp:(0,seed), point(0), point(.1,1),
                              (0.,)*6, LIMITS, epoch=1, clock=Clock())
        near_limit = (0., 0., 0., 0., math.radians(359), 0.)
        job._record_risk(((0.,)*6, near_limit), None, 'midpoint')
        self.assertIn('joint_margin', job.risk_detail['reasons'])
        self.assertEqual(job.risk_detail['smallest_margin_axis'], 5)
        job._record_risk(((0.,)*6, (0.,)*6), [0.,0.,.2,0.,0.,0.], 'interval')
        self.assertEqual(job.risk_detail['reasons'], ['joint_curvature'])

    def test_failure_snapshot_keeps_evidence_after_job_is_cleared(self):
        clock = Clock()
        horizon = IntentHorizon(lambda seed,tcp:(0,(0.,0.,0.,1.,0.,0.)),
                                LIMITS, clock=clock, adaptive=True)
        horizon.reset(point(0), (0.,)*6)
        clock.t = .05
        horizon.push(point(.05, 4), .05)
        clock.t = .18
        horizon.push(point(.18, 4), .18)
        clock.t = .22
        with self.assertRaises(RejectedPath):
            for _ in range(30):
                horizon.advance(permit=True)
        self.assertIsNone(horizon.job)
        self.assertIn('joint_step', horizon.last_failure_context['inverse_risk']['reasons'])


if __name__ == '__main__':
    unittest.main()
