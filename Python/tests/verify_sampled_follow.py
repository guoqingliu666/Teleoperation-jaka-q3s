"""连续跟随故障注入测试：假SDK、假时钟，无硬件连接。"""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab.sampled_follow import SampledFollower, Settings, plan_segment, RejectedPath
from vla_lab.quest_vr_input import QuestUdpReceiver

MAPPING = {"forward":"X+", "backward":"X-", "left":"Y+", "right":"Y-", "up":"Z+", "down":"Z-"}
LIMITS = [(-360,360),(-85,265),(-175,175),(-85,265),(-360,360),(-360,360)]


class Clock:
    now = 10.0
    def __call__(self): return self.now


class Robot:
    def __init__(self):
        self.tcp = (400.,100.,300.,0.,0.,0.)
        self.joints = (0.,)*6
        self.in_pos = True
        self.moves, self.abort_count = [], 0
        self.ik_error = 0
        self.collision = False
    def get_robot_status_simple(self): return (0,(0,0,1,1))
    def is_in_pos(self): return (0,self.in_pos)
    def is_in_estop(self): return (0,False)
    def is_in_collision(self): return (0,self.collision)
    def is_on_limit(self): return (0,False)
    def is_in_servomove(self): return (0,False)
    def get_tool_id(self): return (0,1)
    def get_user_frame_id(self): return (0,0)
    def get_actual_joint_position(self): return (0,self.joints)
    def get_actual_tcp_position(self): return (0,self.tcp)
    def kine_inverse(self, reference, pose):
        if self.ik_error: return (self.ik_error,)
        return (0,tuple((pose[i]-self.tcp[i])*.001+self.joints[i] for i in range(3))+(0.,)*3)
    def linear_move_extend(self, target, *args):
        self.moves.append((target,args)); self.in_pos=False
        return (0,)
    def motion_abort(self): self.abort_count+=1; self.in_pos=True; return (0,)
    def complete(self):
        target=self.moves[-1][0]
        self.joints=self.kine_inverse(self.joints,target)[1]
        self.tcp=target; self.in_pos=True


def frame(clock, grip=0, y=1.0, **overrides):
    value=QuestUdpReceiver._frame({"head":{"connected":True,"pose_valid":True,"rotation_xyzw":[0,0,0,1]},
        "right":{"connected":True,"tracked":True,"pose_valid":True,"position_m":[0,y,0],
                 "rotation_xyzw":[0,0,0,1],"grip":grip,"trigger":0}},"test:1")
    return replace(value, received_s=clock(), **overrides)


class Tests(unittest.TestCase):
    def setup_follow(self, *, command_limit=None):
        robot, clock=Robot(),Clock()
        follower=SampledFollower(robot,MAPPING,Settings(),LIMITS,clock=clock,
                                 command_limit=command_limit)
        follower.initialize()
        f=frame(clock); follower.ready_heading(f); follower.tick(f)
        clock.now+=.02; follower.tick(frame(clock,1))
        clock.now+=.06; f=frame(clock,1,1.03); follower.tick(f)
        return robot,clock,follower,f

    def test_grip_follows_without_a_and_single_segment(self):
        robot,clock,follow,f=self.setup_follow()
        follow.dispatch(f,permit=True)
        self.assertEqual(len(robot.moves),1)
        self.assertAlmostEqual(robot.moves[0][0][2],310)
        self.assertEqual(robot.moves[0][1],(0,False,10.,50.,0.))
        clock.now+=.03; follow.tick(frame(clock,1,1.04)); follow.dispatch(frame(clock,1,1.04),permit=True)
        self.assertEqual(len(robot.moves),1,"运行期间不排队")

    def test_one_segment_wait_cannot_plan_next_target(self):
        robot,clock,follow,f=self.setup_follow()
        follow.dispatch(f,permit=True)
        robot.complete()
        clock.now += .03
        follow.tick(frame(clock,1,1.06), allow_plan=False)
        self.assertEqual(follow.completed_segments,1)
        self.assertFalse(follow.active)
        self.assertIsNone(getattr(follow,"pending",None))

    def test_command_limit_independently_blocks_second_dispatch(self):
        robot,clock,follow,f=self.setup_follow(command_limit=1)
        follow.dispatch(f,permit=True)
        robot.complete()
        clock.now += .03
        follow.tick(frame(clock,1,1.06))
        self.assertEqual(follow.completed_segments,1)
        self.assertIsNone(getattr(follow,"pending",None))
        follow.pending=(robot.tcp,[robot.joints],clock.now,(0,1.06,0),robot.tcp)
        follow.dispatch(frame(clock,1,1.06),permit=True)
        self.assertEqual(len(robot.moves),1)

    def test_release_aborts_once_and_does_not_auto_resume(self):
        robot,clock,follow,f=self.setup_follow(); follow.dispatch(f,permit=True)
        clock.now+=.02; follow.tick(frame(clock,0,1.03))
        self.assertEqual(robot.abort_count,1)
        clock.now+=.02; follow.tick(frame(clock,1,1.03)); follow.dispatch(frame(clock,1,1.03),permit=True)
        self.assertEqual(len(robot.moves),1)
        self.assertIsNone(follow.anchor)

    def test_stale_input_after_inverse_never_sends(self):
        robot,clock,follow,f=self.setup_follow(); clock.now+=.16
        follow.dispatch(f,permit=True); self.assertEqual(robot.moves,[])

    def test_release_during_inverse_never_sends(self):
        robot,clock,follow,f=self.setup_follow()
        follow.dispatch(frame(clock,0,1.03),permit=True); self.assertEqual(robot.moves,[])

    def test_ui_heartbeat_lost_never_sends(self):
        robot,clock,follow,f=self.setup_follow(); follow.dispatch(f,permit=False)
        self.assertEqual(robot.moves,[])

    def test_lost_tracking_aborts(self):
        robot,clock,follow,f=self.setup_follow(); follow.dispatch(f,permit=True)
        clock.now+=.02; follow.tick(frame(clock,1,1.03,tracked=False))
        self.assertEqual(robot.abort_count,1)

    def test_source_change_aborts(self):
        robot,clock,follow,f=self.setup_follow(); follow.dispatch(f,permit=True)
        clock.now+=.02; follow.tick(frame(clock,1,1.03,udp_source="test:2"))
        self.assertEqual(robot.abort_count,1)

    def test_large_change_during_inverse_discards_pending(self):
        robot,clock,follow,f=self.setup_follow()
        follow.dispatch(frame(clock,1,1.08),permit=True)
        self.assertEqual(robot.moves,[])

    def test_newest_target_replaces_history(self):
        robot,clock,follow,f=self.setup_follow(); follow.dispatch(f,permit=True)
        clock.now+=.06; follow.tick(frame(clock,1,1.08))
        clock.now+=.06; follow.tick(frame(clock,1,.97))
        # 11cm突变会停止而不是反向补跑。
        self.assertEqual(robot.abort_count,1)
        self.assertIsNone(follow.desired)

    def test_workspace_clamps_at_100mm_without_recentering(self):
        robot,clock,follow,f=self.setup_follow()
        center=follow.center
        clock.now+=.06; follow.tick(frame(clock,1,1.10))
        clock.now+=.06; follow.tick(frame(clock,1,1.15))
        self.assertAlmostEqual(math.dist(follow.desired[:3],center[:3]),100)
        clock.now+=.02; follow.tick(frame(clock,0,1.15))
        robot.tcp=(400,100,320,0,0,0)
        clock.now+=.02; follow.tick(frame(clock,1,1.15))
        self.assertEqual(follow.center,center)

    def test_internal_ik_failure_blocks_even_when_endpoint_is_reachable(self):
        robot=Robot(); original=robot.kine_inverse
        robot.kine_inverse=lambda ref,pose: (-4,) if 303<pose[2]<307 else original(ref,pose)
        with self.assertRaises(RejectedPath):
            plan_segment(robot,robot.joints,robot.tcp,(400,100,310,0,0,0),Settings(),LIMITS)

    def test_non_ik_sdk_failure_is_not_projected(self):
        robot=Robot(); robot.ik_error=-1
        with self.assertRaises(RuntimeError) as error:
            plan_segment(robot,robot.joints,robot.tcp,(400,100,310,0,0,0),Settings(),LIMITS)
        self.assertNotIsInstance(error.exception,RejectedPath)

    def test_wrapped_branch_jump_is_rejected(self):
        robot=Robot(); robot.kine_inverse=lambda ref,pose:(0,(0,0,0,math.tau,0,0))
        with self.assertRaises(RejectedPath):
            plan_segment(robot,robot.joints,robot.tcp,(400,100,310,0,0,0),Settings(),LIMITS)

    def test_stale_in_pos_does_not_complete_far_from_target(self):
        robot,clock,follow,f=self.setup_follow(); follow.dispatch(f,permit=True)
        robot.in_pos=True
        clock.now+=.02; follow.tick(frame(clock,1,1.03))
        self.assertTrue(follow.active)

    def test_nan_parameters_rejected(self):
        with self.assertRaises(ValueError): Settings(radius_mm=float('nan'))

    def test_settings_supports_staged_workspace_up_to_one_meter(self):
        self.assertEqual(Settings(radius_mm=200).radius_mm, 200)
        self.assertEqual(Settings(radius_mm=1000).radius_mm, 1000)
        with self.assertRaises(ValueError):
            Settings(radius_mm=1000.1)

    def test_robot_moved_during_planning_rejected(self):
        robot,clock,follow,f=self.setup_follow()
        robot.tcp=(401.,100.,300.,0.,0.,0.)
        with self.assertRaises(RuntimeError): follow.dispatch(f,permit=True)
        self.assertEqual(robot.moves,[])

    def test_release_during_final_status_queries_rejected(self):
        robot,clock,follow,f=self.setup_follow()
        follow.dispatch(f,permit=True,refresh=lambda:(frame(clock,0,1.03),True))
        self.assertEqual(robot.moves,[])

    def test_ambiguous_move_error_keeps_abort_obligation(self):
        robot,clock,follow,f=self.setup_follow()
        def ambiguous(*args):
            robot.in_pos=False
            raise RuntimeError("模拟SDK超时，但命令可能已到控制柜")
        robot.linear_move_extend=ambiguous
        with self.assertRaises(RuntimeError): follow.dispatch(f,permit=True)
        self.assertTrue(follow.active)
        self.assertTrue(follow.shutdown())
        self.assertEqual(robot.abort_count,1)

    def test_paused_health_poll_is_rate_limited(self):
        robot, clock = Robot(), Clock()
        calls = 0
        original = robot.get_robot_status_simple
        def counted():
            nonlocal calls
            calls += 1
            return original()
        robot.get_robot_status_simple = counted
        follow = SampledFollower(robot, MAPPING, Settings(), LIMITS, clock=clock)
        follow.initialize()
        for _ in range(20):
            follow.tick(frame(clock, 0))
            clock.now += .01
        self.assertEqual(calls, 2, "暂停时不得以主循环频率重复查询整组状态")

    def test_active_motion_poll_is_capped_at_fifty_hz(self):
        robot,clock,follow,f=self.setup_follow()
        follow.dispatch(f,permit=True)
        reads = 0
        original = robot.get_actual_joint_position
        def counted():
            nonlocal reads
            reads += 1
            return original()
        robot.get_actual_joint_position = counted
        for _ in range(10):
            clock.now += .001
            follow.tick(frame(clock,1,1.03))
        self.assertEqual(reads, 1, "在途反馈不得按10ms主循环无限重复读取")


if __name__ == '__main__': unittest.main()
