"""几何采样/关节时序诊断回归；无SDK、无机器人连接。"""
from pathlib import Path
from dataclasses import replace
import importlib.util
import math
import random
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from vla_lab.trajectory_reference import (
    PoseSample,rpy_quaternion,slerp,angle_deg,chord_progress_interval,
    ordered_chord_fits,adaptive_knots,StreamingReference,RotationHistory,
)
from vla_lab.trajectory_dynamics import JointBudgets,JointTimingAudit,inspect_exported_joint_settings
from vla_lab.trajectory_horizon import TimeHorizon

spec=importlib.util.spec_from_file_location("geometry_readonly_entry",
    Path(__file__).resolve().parents[1]/"只读检查新轨迹.py")
entry=importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)
replay_spec=importlib.util.spec_from_file_location("geometry_replay_entry",
    Path(__file__).resolve().parents[1]/"轨迹连续性诊断.py")
replay=importlib.util.module_from_spec(replay_spec)
replay_spec.loader.exec_module(replay)


def point(t,x=0.,deg=0.,y=0.):
    return PoseSample(t,(x,y,0.),rpy_quaternion((0.,0.,math.radians(deg))))


class GeometryTests(unittest.TestCase):
    def test_rejected_rotation_keeps_exact_reason_and_original_reference(self):
        for sample, reason in ((point(.03,deg=25),"姿态单帧变化超限"),
                               (point(.2,deg=1),"输入采样间隔超限"),
                               (point(0,deg=1),"时间倒退或重复")):
            history=RotationHistory()
            anchor=point(0)
            history.push(anchor)
            with self.assertRaisesRegex(ValueError,reason):
                history.push(sample)
            self.assertEqual(history.previous,anchor)
            self.assertEqual(history.last_failure["reason"],reason)
            self.assertEqual(history.last_failure["rejected_t"],sample.t)
            self.assertTrue(history.blocked)
            history.reset()
            self.assertIsNone(history.last_failure)

    def test_accelerating_straight_line_not_mistaken_for_corner(self):
        points=[point(i*.01,100*(i/10)**3,10*(i/10)**3) for i in range(11)]
        self.assertEqual(len(adaptive_knots(points)),2)

    def test_position_rotation_different_progress_is_not_merged(self):
        self.assertFalse(ordered_chord_fits([point(.05,50,9)],point(0),point(.1,100,10)))

    def test_corner_and_backtrack_retained(self):
        self.assertFalse(ordered_chord_fits([point(.03,20,y=10)],point(0),point(.1,40)))
        self.assertFalse(ordered_chord_fits([point(.03,30),point(.06,10)],point(0),point(.1,40)))

    def test_pure_rotation_variable_speed_and_sign(self):
        p=point(.01,deg=12)
        self.assertTrue(ordered_chord_fits([replace(p,q=tuple(-x for x in p.q))],
                                          point(0),point(.1,deg=15)))

    def test_pure_rotation_reversal_and_turn_not_collapsed(self):
        self.assertFalse(ordered_chord_fits([point(.03,deg=12),point(.06,deg=2)],
                                            point(0),point(.1,deg=15)))
        self.assertFalse(ordered_chord_fits([point(.03,deg=90),point(.06,deg=180)],
                                            point(0),point(.1,deg=360)))

    def test_degenerate_stationary_chord(self):
        self.assertIsNotNone(chord_progress_interval(point(.05,.1,.01),point(0),point(.1)))
        self.assertIsNone(chord_progress_interval(point(.05,2),point(0),point(.1)))
        self.assertIsNone(chord_progress_interval(point(.05,deg=2),point(0),point(.1)))

    def test_large_chord_requires_intermediate_samples(self):
        self.assertFalse(ordered_chord_fits([point(.03,deg=60)],point(0),point(.1,deg=120)))

    def test_invalid_budgets(self):
        for pos,rot in [(0,.1),(1,0),(float("nan"),1),(1,180)]:
            with self.assertRaises(ValueError): chord_progress_interval(point(0),point(0),point(1),pos,rot)

    def test_returned_intervals_respect_both_budgets_randomized(self):
        rng=random.Random(573)
        accepted=0
        for _ in range(600):
            a=PoseSample(0,tuple(rng.uniform(-100,100) for _ in range(3)),
                         rpy_quaternion(tuple(rng.uniform(-2,2) for _ in range(3))))
            b=PoseSample(.1,tuple(v+rng.uniform(-30,30) for v in a.xyz),
                         rpy_quaternion(tuple(v for v in a.tcp()[3:])))
            # 包括不同旋转轴，而不只测试Z转动。
            from vla_lab.trajectory_reference import qmul
            b=replace(b,q=qmul(a.q,rpy_quaternion(tuple(rng.uniform(-.2,.2) for _ in range(3)))))
            u=rng.random()
            p=PoseSample(.05,tuple(x+(y-x)*u+rng.uniform(-.6,.6) for x,y in zip(a.xyz,b.xyz)),
                         qmul(slerp(a.q,b.q,u),rpy_quaternion(tuple(rng.uniform(-.002,.002) for _ in range(3)))))
            interval=chord_progress_interval(p,a,b)
            if interval is None: continue
            accepted+=1
            for v in (interval[0],sum(interval)/2,interval[1]):
                self.assertLessEqual(math.dist(p.xyz,tuple(x+(y-x)*v for x,y in zip(a.xyz,b.xyz))),1.+1e-7)
                self.assertLessEqual(angle_deg(p.q,slerp(a.q,b.q,v)),.25+1e-7)
        self.assertGreater(accepted,300)

    def test_horizon_combines_variable_speed_without_losing_time(self):
        now=[0.]
        h=TimeHorizon(lambda *_:(0,(0.,)*6),((-360.,360.),)*6,clock=lambda:now[0])
        h.reset(point(0),(0.,)*6)
        for i in range(1,9):
            now[0]=i*.01
            h.push(point(now[0],20*(i/8)**3,2*(i/8)**3),now[0])
        selected=h._select_target()
        self.assertAlmostEqual(selected.t,.08)
        self.assertAlmostEqual(selected.xyz[0],20.)

    def test_streaming_freezes_tangent_only_after_checking_both_sides(self):
        stream=StreamingReference()
        spans=[]
        t=0.
        for i in range(80):
            t += (.016,.031,.032)[i%3]
            p=PoseSample(t,(i*3.,math.sin(i*.6)*4,math.cos(i*.8)*2),
                         rpy_quaternion((math.sin(i*.9)*.05,math.cos(i*.4)*.02,i*.012)))
            span=stream.push(p)
            if span: spans.append(span)
        spans.append(stream.finish())
        self.assertGreater(stream.tangent_reductions,0)
        for span in spans:
            b=span.bounds()
            self.assertLessEqual(b["position_deviation_mm"],1.)
            self.assertLessEqual(b["orientation_deviation_deg"],.25)
        for a,b in zip(spans,spans[1:]):
            l,r=a.evaluate(a.right.t),b.evaluate(b.left.t)
            self.assertLess(math.dist(l.velocity,r.velocity),1e-8)
            self.assertLess(math.dist(l.acceleration,r.acceleration),1e-6)
            self.assertLess(math.dist(l.angular_velocity,r.angular_velocity),1e-8)
            self.assertLess(math.dist(l.angular_acceleration,r.angular_acceleration),1e-6)

    def test_raw_replay_never_joins_different_grip_windows(self):
        events=[]
        for epoch in (1,3,5):
            for i in range(4):
                p=point(epoch+i*.03,x=epoch*100+i,deg=i*.1)
                events.append(dict(state="trajectory_raw_sample",epoch=epoch,t=p.t,xyz=p.xyz,q=p.q))
        report=replay.analyze_raw_capture(events,Path("readonly.jsonl"))
        self.assertEqual(report["windows"],3)
        self.assertEqual(report["reference_spans"],9)
        self.assertEqual(report["errors"],[])
        self.assertFalse(report["robot_connected"])

    def test_raw_replay_time_jump_is_not_silently_reanchored(self):
        events=[dict(state="trajectory_raw_sample",epoch=1,t=t,xyz=(0.,0.,0.),q=(0.,0.,0.,1.))
                for t in (0.,.03,.01,.04)]
        report=replay.analyze_raw_capture(events,Path("readonly.jsonl"))
        self.assertEqual(len(report["errors"]),2)
        self.assertEqual(report["reference_spans"],0)


class TimingTests(unittest.TestCase):
    def joints(self,degree): return (math.radians(degree),0.,0.,0.,0.,0.)

    def test_nonuniform_times_constant_velocity(self):
        a=JointTimingAudit(time_basis="test")
        for t in (0.,.02,.07,.12): a.push(t,self.joints(20*t))
        self.assertAlmostEqual(a.report()["max_interval_average_speed_deg_s"][0],20)
        self.assertLess(a.report()["max_difference_acceleration_deg_s2"][0],1e-10)

    def test_acceleration_and_cross_segment_reversal(self):
        a=JointTimingAudit(time_basis="test")
        for t,q in [(0,0),(.1,1),(.2,0)]: a.push(t,self.joints(q))
        self.assertAlmostEqual(a.report()["max_difference_acceleration_deg_s2"][0],200.)

    def test_small_angle_can_require_large_acceleration(self):
        a=JointTimingAudit(time_basis="test",budgets=JointBudgets((5.,)*6,(20.,)*6,"test_only"))
        for t,q in [(0,0),(.01,.1),(.02,0)]: a.push(t,self.joints(q))
        self.assertAlmostEqual(a.report()["sampled_uniform_time_scale"],10.)
        b=JointTimingAudit(time_basis="test",budgets=a.budgets)
        for t,q in [(0,0),(.1,.1),(.2,0)]: b.push(t,self.joints(q))
        self.assertAlmostEqual(b.report()["sampled_uniform_time_scale"],1.)
        self.assertFalse(b.report()["continuous_peak_bounds_proven"])

    def test_no_modulo_hides_full_joint_turn(self):
        a=JointTimingAudit(time_basis="test")
        a.push(0,self.joints(179)); a.push(.1,self.joints(-179))
        self.assertAlmostEqual(a.report()["max_interval_average_speed_deg_s"][0],3580.)

    def test_reset_does_not_differentiate_across_grip(self):
        a=JointTimingAudit(time_basis="test")
        a.push(0,self.joints(0)); a.push(.1,self.joints(1))
        a.reset(); a.push(.01,self.joints(100))
        self.assertEqual(a.report()["max_interval_average_speed_deg_s"],[0.]*6)
        self.assertIsNone(a.report()["sampled_uniform_time_scale"])

    def test_bad_time_and_budgets_rejected(self):
        for t in (0.,-.1,float("nan"),1e-10):
            a=JointTimingAudit(time_basis="test"); a.push(0,self.joints(0))
            with self.assertRaises(ValueError): a.push(t,self.joints(1))
        with self.assertRaises(ValueError): JointBudgets((1.,)*5,(2.,)*6,"test")
        with self.assertRaises(ValueError): JointBudgets((1.,)*6,(float("nan"),)*6,"test")


class ConfigTests(unittest.TestCase):
    def temporary(self): return tempfile.TemporaryDirectory(dir="D:/ChatGPT/Temp")

    def test_local_cmd_is_data_not_executed_and_only_whitelist_read(self):
        with self.temporary() as temp:
            file=Path(temp)/"local.cmd"
            file.write_text('@echo off\nset "QUEST_JAKA_HOST=example"\nset "UNKNOWN=ignored"\nexit 100\n',encoding="utf-8")
            self.assertEqual(entry.read_local_settings(file),{"QUEST_JAKA_HOST":"example"})

    def test_ambiguous_local_values_fail(self):
        with self.temporary() as temp:
            file=Path(temp)/"local.cmd"
            for content in ('set "QUEST_JAKA_HOST=%SOME_HOST%"',
                            'set "QUEST_JAKA_HOST=a"\nset "QUEST_JAKA_HOST=b"'):
                file.write_text(content,encoding="utf-8")
                with self.assertRaises(ValueError): entry.read_local_settings(file)

    def test_export_snapshot_does_not_claim_units_or_active_limits(self):
        with self.temporary() as temp:
            file=Path(temp)/"settings.ini"
            file.write_text("\n".join(f"[JOINT_{i}]\nJOINT_MIN_LIMIT=-100\nJOINT_MAX_LIMIT=100\nJOINT_VEL_LIMIT=10\nJOINT_ACC_LIMIT=20" for i in range(6)),encoding="utf-8")
            report=inspect_exported_joint_settings(file)
            self.assertEqual(len(report["sha256"]),64)
            self.assertFalse(report["units_confirmed"])
            self.assertFalse(report["applied_to_robot"])
            file.write_text("[JOINT_0]\nJOINT_MIN_LIMIT=-100",encoding="utf-8")
            with self.assertRaises(KeyError): inspect_exported_joint_settings(file)


if __name__=="__main__": unittest.main()
