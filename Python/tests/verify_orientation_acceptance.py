"""一次≤1°姿态真机验收核心的假SDK测试；不会连接设备。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from vla_lab.orientation_acceptance import OrientationAcceptance
from verify_sampled_follow import Clock, LIMITS, MAPPING, Robot, frame


def q_y(degrees):
    half=math.radians(degrees)/2
    return (0.0,math.sin(half),0.0,math.cos(half))


class OrientationRobot(Robot):
    def __init__(self):
        super().__init__()
        self.orientation_moves=[]
    def linear_move_extend_ori(self,target,*args):
        self.orientation_moves.append((target,args))
        self.in_pos=False
        return (0,)
    def complete_orientation(self):
        self.tcp=self.orientation_moves[-1][0]
        self.in_pos=True


class Tests(unittest.TestCase):
    def setup_candidate(self,degrees=.6, **profile):
        robot,clock,events=OrientationRobot(),Clock(),[]
        accept=OrientationAcceptance(robot,MAPPING,LIMITS,emit=lambda **e:events.append(e),
                                     clock=clock, **profile)
        accept.initialize()
        released=frame(clock,grip=0)
        accept.ready_heading(released)
        accept.tick(released)
        clock.now+=.01
        accept.tick(replace(frame(clock,grip=1),rotation_xyzw=q_y(0)))
        clock.now+=.01
        current=replace(frame(clock,grip=1),rotation_xyzw=q_y(degrees))
        accept.tick(current)
        return robot,clock,events,accept,current

    def test_one_command_uses_explicit_vendor_orientation_limits(self):
        robot,clock,events,accept,current=self.setup_candidate()
        self.assertIsNotNone(accept.pending)
        accept.dispatch(current,permit=True)
        self.assertEqual(accept.commands,1)
        self.assertEqual(len(robot.orientation_moves),1)
        target,args=robot.orientation_moves[0]
        self.assertEqual(target[:3],robot.tcp[:3])
        self.assertEqual(args[:5],(0,False,5.0,10.0,0.0))
        self.assertAlmostEqual(math.degrees(args[5]),1.0)
        self.assertAlmostEqual(math.degrees(args[6]),2.0)
        # 第二个候选永远不能产生第二条命令。
        accept.pending=(target,accept.path_solutions,current.received_s,current.rotation_xyzw,robot.tcp,.6)
        accept.dispatch(current,permit=True)
        self.assertEqual(len(robot.orientation_moves),1)

    def test_measured_completion_requires_position_and_orientation(self):
        robot,clock,events,accept,current=self.setup_candidate()
        accept.dispatch(current,permit=True)
        robot.complete_orientation()
        clock.now+=.03
        accept.tick(current,allow_plan=False)
        self.assertEqual(accept.completed_segments,1)
        self.assertFalse(accept.active)

    def test_release_requests_abort(self):
        robot,clock,events,accept,current=self.setup_candidate()
        accept.dispatch(current,permit=True)
        clock.now+=.01
        accept.tick(frame(clock,grip=0),allow_plan=False)
        self.assertEqual(robot.abort_count,1)
        self.assertFalse(accept.active)

    def test_large_hand_frame_is_blocked_without_command(self):
        robot,clock,events,accept,current=self.setup_candidate(.1)
        clock.now+=.01
        accept.tick(replace(frame(clock,grip=1),rotation_xyzw=q_y(5)))
        self.assertTrue(any(e.get("state")=="orientation_acceptance_blocked" for e in events))
        self.assertEqual(robot.orientation_moves,[])

    def test_three_segment_profile_tracks_latest_relative_target_without_xyz_drift(self):
        profile=dict(command_limit=3,max_total_deg=3.0,
                     orientation_speed_deg_s=2.0,
                     orientation_acceleration_deg_s2=4.0)
        robot,clock,events,accept,current=self.setup_candidate(.6,**profile)
        for degrees in (.6,1.6,2.6):
            current=replace(frame(clock,grip=1),rotation_xyzw=q_y(degrees))
            accept.tick(current)
            self.assertIsNotNone(accept.pending)
            accept.dispatch(current,permit=True)
            robot.complete_orientation()
            clock.now+=.03
            current=replace(frame(clock,grip=1),rotation_xyzw=q_y(degrees))
            accept.tick(current,allow_plan=False)
            clock.now+=.01
        self.assertEqual(accept.commands,3)
        self.assertEqual(accept.completed_segments,3)
        self.assertLessEqual(accept.commanded_orientation_deg,3.0)
        self.assertEqual(len(robot.orientation_moves),3)
        for target,args in robot.orientation_moves:
            self.assertEqual(target[:3],(400.0,100.0,300.0))
            self.assertAlmostEqual(math.degrees(args[5]),2.0)
            self.assertAlmostEqual(math.degrees(args[6]),4.0)

    def test_ten_degree_profile_caps_every_command_at_one_degree(self):
        profile=dict(command_limit=10,max_total_deg=10.0,
                     orientation_speed_deg_s=5.0,
                     orientation_acceleration_deg_s2=10.0)
        robot,clock,events,accept,current=self.setup_candidate(9.6,**profile)
        # 第一个大跨度样本只用于把输入基准追到最新值；稳定后的下一帧才允许规划。
        clock.now+=.01
        current=replace(frame(clock,grip=1),rotation_xyzw=q_y(9.6))
        accept.tick(current)
        self.assertIsNotNone(accept.pending)
        for _ in range(10):
            accept.dispatch(current,permit=True)
            robot.complete_orientation()
            clock.now+=.03
            current=replace(frame(clock,grip=1),rotation_xyzw=q_y(9.6))
            accept.tick(current,allow_plan=False)
            if accept.commands < 10:
                clock.now+=.01
                accept.tick(current)
                self.assertIsNotNone(accept.pending)
        self.assertEqual(accept.commands,10)
        self.assertEqual(accept.completed_segments,10)
        self.assertLessEqual(accept.commanded_orientation_deg,10.0)
        for target,args in robot.orientation_moves:
            self.assertEqual(target[:3],(400.0,100.0,300.0))
            self.assertAlmostEqual(math.degrees(args[5]),5.0)
            self.assertAlmostEqual(math.degrees(args[6]),10.0)

    def test_large_sample_updates_baseline_instead_of_permanently_blocking(self):
        robot,clock,events,accept,current=self.setup_candidate(.1)
        clock.now+=.01
        jumped=replace(frame(clock,grip=1),rotation_xyzw=q_y(8))
        accept.tick(jumped)
        self.assertIsNone(accept.pending)
        clock.now+=.01
        stable=replace(frame(clock,grip=1),rotation_xyzw=q_y(8.1))
        accept.tick(stable)
        self.assertIsNotNone(accept.pending)


if __name__=="__main__": unittest.main()
