"""受限连续六维跟随的假SDK测试；不会连接或移动真机。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))

from vla_lab.bounded_pose_follow import (
    BoundedPoseFollower, plan_pose_segment, rpy_matrix,
)
from vla_lab.sampled_follow import Settings
from verify_sampled_follow import Clock, LIMITS, MAPPING, Robot, frame


def q_y(degrees):
    half=math.radians(degrees)/2
    return (0.0,math.sin(half),0.0,math.cos(half))


class PoseRobot(Robot):
    def __init__(self):
        super().__init__()
        self.orientation_moves=[]
    def linear_move_extend_ori(self,target,*args):
        self.orientation_moves.append((target,args))
        self.in_pos=False
        return (0,)
    def complete_pose(self):
        self.tcp=self.orientation_moves[-1][0]
        self.in_pos=True


class Tests(unittest.TestCase):
    def test_full_orientation_mapping_crosses_180_without_rpy_wrap(self):
        robot = PoseRobot()
        follow = BoundedPoseFollower(robot, MAPPING,
            Settings(radius_mm=1000,speed_mm_s=150,acceleration_mm_s2=400,segment_mm=60),
            LIMITS, rotation_radius_deg=180,orientation_speed_deg_s=30,
            orientation_acceleration_deg_s2=120,max_orientation_step_deg=6)
        follow.initialize()
        follow.anchor = tuple(robot.tcp[:3]) + (0.,0.,0.)
        follow.center_rotation = rpy_matrix((0.,0.,0.))
        for degree in range(0, 361):
            follow._update_target(None,(0.,0.,0.),rpy_matrix((0.,0.,math.radians(degree))))
            self.assertAlmostEqual(follow.desired[5],math.radians(degree),places=6)
        self.assertEqual(robot.orientation_moves, [])

    def setup_follow(self):
        robot,clock,events=PoseRobot(),Clock(),[]
        follow=BoundedPoseFollower(robot,MAPPING,Settings(radius_mm=200,speed_mm_s=30,
                                   acceleration_mm_s2=60),LIMITS,
                                   clock=clock,emit=lambda **e:events.append(e))
        follow.initialize()
        released=frame(clock,grip=0)
        follow.ready_heading(released); follow.tick(released)
        clock.now+=.02
        follow.tick(replace(frame(clock,grip=1),rotation_xyzw=q_y(0)))
        clock.now+=.06
        current=replace(frame(clock,grip=1,y=1.03),rotation_xyzw=q_y(5))
        follow.tick(current)
        return robot,clock,events,follow,current

    def test_combined_command_has_hard_translation_and_rotation_steps(self):
        robot,clock,events,follow,current=self.setup_follow()
        self.assertIsNotNone(follow.pending)
        follow.dispatch(current,permit=True)
        self.assertEqual(len(robot.orientation_moves),1)
        target,args=robot.orientation_moves[0]
        self.assertLessEqual(math.dist(target[:3],(400,100,300)),10.000001)
        self.assertLessEqual(math.degrees(abs(target[4])),1.000001)
        self.assertEqual(args[:5],(0,False,30,60,0.0))
        self.assertAlmostEqual(math.degrees(args[5]),5)
        self.assertAlmostEqual(math.degrees(args[6]),10)

    def test_measured_completion_and_latest_target_continue(self):
        robot,clock,events,follow,current=self.setup_follow()
        follow.dispatch(current,permit=True)
        robot.complete_pose(); clock.now+=.03
        follow.tick(current)
        self.assertEqual(follow.completed_segments,1)
        self.assertFalse(follow.active)

    def test_release_aborts_active_command(self):
        robot,clock,events,follow,current=self.setup_follow()
        follow.dispatch(current,permit=True)
        clock.now+=.06
        follow.tick(frame(clock,grip=0))
        self.assertEqual(robot.abort_count,1)

    def test_stale_combined_intent_is_not_dispatched(self):
        robot,clock,events,follow,current=self.setup_follow()
        changed=replace(frame(clock,grip=1,y=1.08),rotation_xyzw=q_y(12))
        follow.dispatch(changed,permit=True)
        self.assertEqual(robot.orientation_moves,[])

    def test_planner_uses_complete_pose_and_vendor_ik(self):
        robot=PoseRobot()
        target,solutions=plan_pose_segment(
            robot,robot.joints,robot.tcp,(430,100,300,0,math.radians(5),0),
            Settings(radius_mm=200,speed_mm_s=30,acceleration_mm_s2=60),LIMITS)
        self.assertLessEqual(math.dist(target[:3],robot.tcp[:3]),10.000001)
        self.assertLessEqual(math.degrees(abs(target[4])),1.000001)
        self.assertGreaterEqual(len(solutions),5)

    def test_expanded_profile_is_still_bounded_per_command(self):
        robot,clock=PoseRobot(),Clock()
        settings=Settings(radius_mm=500,speed_mm_s=50,acceleration_mm_s2=100,
                          segment_mm=20,deadband_mm=3)
        follow=BoundedPoseFollower(
            robot,MAPPING,settings,LIMITS,clock=clock,
            rotation_radius_deg=30,orientation_speed_deg_s=10,
            orientation_acceleration_deg_s2=20,max_orientation_step_deg=2)
        follow.initialize()
        released=frame(clock,grip=0); follow.ready_heading(released); follow.tick(released)
        clock.now+=.02
        follow.tick(replace(frame(clock,grip=1),rotation_xyzw=q_y(0)))
        clock.now+=.06
        current=replace(frame(clock,grip=1,y=1.08),rotation_xyzw=q_y(25))
        follow.tick(current)
        # 大姿态跨样本仅丢弃一帧并更新基准，下一稳定帧进入规划。
        clock.now+=.06
        current=replace(frame(clock,grip=1,y=1.08),rotation_xyzw=q_y(25))
        follow.tick(current)
        follow.dispatch(current,permit=True)
        target,args=robot.orientation_moves[0]
        self.assertLessEqual(math.dist(target[:3],(400,100,300)),20.000001)
        self.assertLessEqual(math.degrees(abs(target[4])),2.000001)
        self.assertEqual(args[:5],(0,False,50,100,0.0))
        self.assertAlmostEqual(math.degrees(args[5]),10)
        self.assertAlmostEqual(math.degrees(args[6]),20)

    def test_production_profile_expands_only_session_envelope(self):
        settings=Settings(radius_mm=1000,speed_mm_s=45,acceleration_mm_s2=90,
                          segment_mm=20,deadband_mm=3)
        follow=BoundedPoseFollower(
            PoseRobot(),MAPPING,settings,LIMITS,
            rotation_radius_deg=25,orientation_speed_deg_s=8,
            orientation_acceleration_deg_s2=32,max_orientation_step_deg=2)
        self.assertEqual(follow.settings.radius_mm,1000)
        self.assertEqual(follow.max_orientation_step_deg,2)

    def test_development_profile_allows_longer_endpoint_segment_but_keeps_vendor_ik(self):
        robot=PoseRobot()
        settings=Settings(radius_mm=1000,speed_mm_s=150,acceleration_mm_s2=400,
                          segment_mm=60,deadband_mm=3)
        follow=BoundedPoseFollower(
            robot,MAPPING,settings,LIMITS,
            rotation_radius_deg=30,orientation_speed_deg_s=30,
            orientation_acceleration_deg_s2=120,max_orientation_step_deg=6)
        target,solutions=plan_pose_segment(
            robot,robot.joints,robot.tcp,(500,100,300,0,math.radians(12),0),
            settings,LIMITS,max_orientation_step_deg=6)
        self.assertLessEqual(math.dist(target[:3],robot.tcp[:3]),60.000001)
        self.assertLessEqual(math.degrees(abs(target[4])),6.000001)
        # 60mm / 6°仍按2mm / 0.2°逐点调用厂商逆解，不是跳过中间检查。
        self.assertGreaterEqual(len(solutions),30)
        self.assertEqual(follow.settings.segment_mm,60)

    def test_pose_priority_keeps_full_orientation_but_clamps_xyz_to_10cm(self):
        robot = PoseRobot()
        settings = Settings(radius_mm=1000,speed_mm_s=150,
                            acceleration_mm_s2=400,segment_mm=60,deadband_mm=3)
        follow = BoundedPoseFollower(
            robot,MAPPING,settings,LIMITS,
            rotation_radius_deg=30,orientation_speed_deg_s=30,
            orientation_acceleration_deg_s2=120,max_orientation_step_deg=3,
            position_radius_mm=100)
        follow.initialize()
        follow.anchor = robot.tcp
        follow._update_target(
            None,(500.0,0.0,0.0),rpy_matrix((0.0,math.radians(30.0),0.0)))
        self.assertAlmostEqual(math.dist(follow.desired[:3],robot.tcp[:3]),100.0)
        self.assertAlmostEqual(math.degrees(follow.desired[4]),30.0,places=5)
        self.assertEqual(follow.position_radius_mm,100.0)

    def test_expanded_pose_priority_clamps_xyz_to_30cm_and_orientation_to_45deg(self):
        robot = PoseRobot()
        settings = Settings(radius_mm=1000,speed_mm_s=200,
                            acceleration_mm_s2=500,segment_mm=80,deadband_mm=3)
        follow = BoundedPoseFollower(
            robot,MAPPING,settings,LIMITS,
            rotation_radius_deg=45,orientation_speed_deg_s=45,
            orientation_acceleration_deg_s2=180,max_orientation_step_deg=4,
            position_radius_mm=300)
        follow.initialize()
        follow.anchor = robot.tcp
        follow._update_target(
            None,(600.0,0.0,0.0),rpy_matrix((0.0,math.radians(60.0),0.0)))
        self.assertAlmostEqual(math.dist(follow.desired[:3],robot.tcp[:3]),300.0)
        self.assertAlmostEqual(math.degrees(follow.desired[4]),45.0,places=5)
        self.assertEqual(follow.position_radius_mm,300.0)


if __name__=="__main__": unittest.main()
