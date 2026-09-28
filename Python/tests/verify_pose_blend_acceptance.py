"""两段六维厂商队列圆滑的离线故障注入测试。"""
from __future__ import annotations

from pathlib import Path
import math
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab.pose_blend_acceptance import (  # noqa: E402
    build_fixed_pose_blend_plan, build_requested_pose_blend_plan,
    execute_fixed_pose_blend, RollingPoseQueue,
)
from vla_lab.bounded_pose_follow import RejectedPath  # noqa: E402
from vla_lab.sampled_follow import Settings  # noqa: E402


LIMITS = [(-360, 360), (-85, 265), (-175, 175), (-85, 265), (-360, 360), (-360, 360)]


class Clock:
    def __init__(self): self.now = 10.0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class Robot:
    def __init__(self, *, never_busy=False):
        self.tcp = (400.0, 100.0, 300.0, 0.0, 0.0, 0.0)
        self.joints = (0.0,) * 6
        self.moves = []
        self.abort_count = 0
        self.aborted = False
        self.status_reads = 0
        self.never_busy = never_busy
        self.last_solution = self.joints

    def get_robot_status_simple(self): return (0, (0, 0, 1, 1))
    def is_in_estop(self): return (0, False)
    def is_in_collision(self): return (0, False)
    def is_on_limit(self): return (0, False)
    def is_in_servomove(self): return (0, False)
    def get_tool_id(self): return (0, 1)
    def get_user_frame_id(self): return (0, 0)
    def get_actual_joint_position(self): return (0, self.joints)
    def get_actual_tcp_position(self): return (0, self.tcp)

    def kine_inverse(self, reference, pose):
        # 小且连续的伪厂商解：前三轴跟随平移，末轴跟随 Rz。
        self.last_solution = (
            (pose[0] - 400.0) * .001,
            (pose[1] - 100.0) * .001,
            (pose[2] - 300.0) * .001,
            0.0, 0.0, pose[5],
        )
        return (0, self.last_solution)

    def get_motion_status(self):
        self.status_reads += 1
        if self.aborted:
            return (0, [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        if not self.moves or self.never_busy:
            return (0, [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        if len(self.moves) == 1:
            return (0, [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])
        if self.status_reads < 7:
            return (0, [2, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0])
        self.tcp = self.moves[-1][0]
        self.joints = self.last_solution
        return (0, [2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])

    def linear_move_extend_ori(self, target, *args):
        self.moves.append((tuple(target), args))
        return (0,)

    def motion_abort(self):
        self.abort_count += 1
        self.aborted = True
        return (0,)


class RollingRobot(Robot):
    def __init__(self):
        super().__init__()
        self.active_target = None
        self.queued_target = None
        self.phase = 0

    def kine_inverse(self, reference, pose):
        return (0, ((pose[0]-400.0)*.001, (pose[1]-100.0)*.001,
                    (pose[2]-300.0)*.001, 0.0, 0.0, pose[5]))

    def linear_move_extend_ori(self, target, *args):
        self.moves.append((tuple(target), args))
        if self.active_target is None:
            self.active_target = tuple(target)
            self.phase = 0
        elif self.queued_target is None:
            self.queued_target = tuple(target)
        else:
            return (-99,)
        return (0,)

    def _arrive(self, target):
        self.tcp = tuple(target)
        self.joints = self.kine_inverse(self.joints, target)[1]

    def get_motion_status(self):
        if self.aborted:
            return (0, [0,0,1,0,0,0,0,0,0,0,0])
        if self.active_target is None:
            return (0, [0,0,1,0,0,0,0,0,0,0,0])
        self.phase += 1
        if self.queued_target is not None:
            if self.phase >= 3:
                self._arrive(self.active_target)
                self.active_target = self.queued_target
                self.queued_target = None
                self.phase = 0
                return (0, [1,0,0,0,1,1,0,0,0,0,0])
            return (0, [2,0,0,0,2,1,0,0,0,0,0])
        if self.phase >= 3:
            self._arrive(self.active_target)
            self.active_target = None
            self.phase = 0
            return (0, [0,0,1,0,0,0,0,0,0,0,0])
        return (0, [1,0,0,0,1,1,0,0,0,0,0])

    def motion_abort(self):
        self.active_target = self.queued_target = None
        return super().motion_abort()


class DelayedBusyRollingRobot(RollingRobot):
    """模拟命令已接收，但运动状态始终没有出现忙态。"""
    def get_motion_status(self):
        if self.aborted:
            return (0, [0,0,1,0,0,0,0,0,0,0,0])
        return (0, [0,0,1,0,0,0,0,0,0,0,0])


class HiddenSecondQueueRobot(RollingRobot):
    """模拟第二条调用返回成功，但控制柜始终未报告 queue=2。"""
    def get_motion_status(self):
        if self.aborted:
            return (0, [0,0,1,0,0,0,0,0,0,0,0])
        if self.active_target is None:
            return (0, [0,0,1,0,0,0,0,0,0,0,0])
        return (0, [1,0,0,0,1,1,0,0,0,0,0])


class OverflowRollingRobot(RollingRobot):
    def get_motion_status(self):
        if self.active_target is None:
            return super().get_motion_status()
        return (0, [3,0,0,0,3,1,0,0,0,0,0])


class PoseBlendAcceptanceTests(unittest.TestCase):
    def test_stop_failure_preserves_original_reason_and_unconfirmed_state(self):
        robot, clock, events = RollingRobot(), Clock(), []
        settings = Settings(radius_mm=1000, speed_mm_s=150,
                            acceleration_mm_s2=400, segment_mm=60, deadband_mm=3)
        queue = RollingPoseQueue(robot, settings, LIMITS,clock=clock,sleep=clock.sleep,
                                 emit=lambda **event: events.append(event))
        queue.active = True
        queue.stop_confirmed = True  # 上次成功不能被误当成本次的停止确认。
        def broken_abort():
            raise RuntimeError("模拟停止通信故障")
        robot.motion_abort = broken_abort
        with self.assertRaisesRegex(RuntimeError,"模拟停止通信故障"):
            queue._abort("原始原因：输入失追踪")
        self.assertFalse(queue.stop_confirmed)
        self.assertEqual(events[-1]['state'], 'rolling_pose_stop_requested')
        self.assertEqual(events[-1]['message'], '原始原因：输入失追踪')

    def test_release_during_inverse_prevents_first_send(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000, speed_mm_s=150,
                            acceleration_mm_s2=400, segment_mm=60, deadband_mm=3)
        queue = RollingPoseQueue(robot, settings, LIMITS,clock=clock,sleep=clock.sleep)
        queue.initialize()
        queue.tick((424.,100.,300.,0.,0.,0.),permit=True,refresh_permit=lambda:False)
        self.assertEqual(robot.moves,[])
        self.assertEqual(robot.abort_count,0)

    def test_release_during_prequeue_inverse_aborts_existing_motion(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000, speed_mm_s=150,
                            acceleration_mm_s2=400, segment_mm=60, deadband_mm=3)
        queue = RollingPoseQueue(robot, settings, LIMITS,clock=clock,sleep=clock.sleep)
        queue.initialize()
        queue.tick((424.,100.,300.,0.,0.,0.),permit=True)
        queue.tick((448.,100.,300.,0.,0.,0.),permit=True,refresh_permit=lambda:False)
        self.assertEqual(len(robot.moves),1)
        self.assertEqual(robot.abort_count,1)

    def test_long_inverse_never_sends_late_target(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000, speed_mm_s=150,
                            acceleration_mm_s2=400, segment_mm=60, deadband_mm=3)
        queue = RollingPoseQueue(robot,settings,LIMITS,clock=clock,sleep=clock.sleep)
        queue.initialize()
        inverse=robot.kine_inverse
        def slow(reference,pose):
            clock.sleep(.02)
            return inverse(reference,pose)
        robot.kine_inverse=slow
        with self.assertRaisesRegex(RuntimeError,"迟到"):
            queue.tick((424.,100.,300.,0.,0.,0.),permit=True,refresh_permit=lambda:True)
        self.assertEqual(robot.moves,[])

    def test_fixed_plan_is_two_30mm_and_two_3degree_steps(self):
        robot = Robot()
        plan = build_fixed_pose_blend_plan(robot, LIMITS)
        self.assertAlmostEqual(plan.corner_tcp[2] - plan.start_tcp[2], 30.0)
        self.assertAlmostEqual(plan.final_tcp[0] - plan.corner_tcp[0], 30.0)
        self.assertAlmostEqual(math.degrees(plan.corner_tcp[5]), 3.0, places=6)
        self.assertAlmostEqual(math.degrees(plan.final_tcp[5]), 6.0, places=6)
        self.assertEqual(robot.moves, [])

    def test_first_command_blends_and_second_stops_exactly(self):
        robot, clock = Robot(), Clock()
        plan = build_fixed_pose_blend_plan(robot, LIMITS)
        result = execute_fixed_pose_blend(robot, plan, clock=clock, sleep=clock.sleep)
        self.assertEqual(result["commands_sent"], 2)
        self.assertEqual(len(robot.moves), 2)
        self.assertEqual(robot.moves[0][1][:5], (0, False, 150.0, 400.0, 5.0))
        self.assertEqual(robot.moves[1][1][:5], (0, False, 150.0, 400.0, 0.0))
        self.assertAlmostEqual(math.degrees(robot.moves[0][1][5]), 30.0)
        self.assertAlmostEqual(math.degrees(robot.moves[0][1][6]), 120.0)
        self.assertEqual(robot.abort_count, 0)
        self.assertGreaterEqual(result["max_queue"], 1)
        self.assertGreaterEqual(result["max_active_queue"], 1)

    def test_missing_busy_state_aborts_before_second_command(self):
        robot, clock = Robot(never_busy=True), Clock()
        plan = build_fixed_pose_blend_plan(robot, LIMITS)
        with self.assertRaisesRegex(RuntimeError, "未观察到队列忙态"):
            execute_fixed_pose_blend(robot, plan, clock=clock, sleep=clock.sleep)
        self.assertEqual(len(robot.moves), 1)
        self.assertEqual(robot.abort_count, 1)

    def test_permission_loss_after_first_command_aborts(self):
        robot, clock = Robot(), Clock()
        plan = build_fixed_pose_blend_plan(robot, LIMITS)
        calls = 0

        def permission():
            nonlocal calls
            calls += 1
            return calls < 3

        with self.assertRaisesRegex(RuntimeError, "许可失效"):
            execute_fixed_pose_blend(
                robot, plan, permission=permission, clock=clock, sleep=clock.sleep)
        self.assertEqual(len(robot.moves), 1)
        self.assertEqual(robot.abort_count, 1)

    def test_hand_samples_are_clamped_to_60mm_and_6degrees_per_segment(self):
        robot = Robot()
        first = (500.0, 100.0, 300.0, 0.0, 0.0, math.radians(10.0))
        second = (600.0, 100.0, 300.0, 0.0, 0.0, math.radians(20.0))
        plan = build_requested_pose_blend_plan(robot, LIMITS, first, second)
        self.assertLessEqual(math.dist(plan.start_tcp[:3], plan.corner_tcp[:3]), 60.000001)
        self.assertLessEqual(math.dist(plan.corner_tcp[:3], plan.final_tcp[:3]), 60.000001)
        self.assertLessEqual(math.degrees(plan.corner_tcp[5]), 6.000001)
        self.assertLessEqual(math.degrees(plan.final_tcp[5]-plan.corner_tcp[5]), 6.000001)

    def test_nearly_duplicate_second_hand_sample_is_rejected_without_motion(self):
        robot = Robot()
        first = (420.0, 100.0, 300.0, 0.0, 0.0, 0.0)
        second = (421.0, 100.0, 300.0, 0.0, 0.0, math.radians(.1))
        with self.assertRaisesRegex(ValueError, "过近"):
            build_requested_pose_blend_plan(robot, LIMITS, first, second)
        self.assertEqual(robot.moves, [])

    def test_rolling_queue_uses_latest_targets_with_depth_two_and_exact_final(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(robot, settings, LIMITS,
                                 clock=clock, sleep=clock.sleep)
        queue.initialize()
        for _ in range(200):
            desired = (400.0 + 24.0*(queue.commands+1), 100.0, 300.0,
                       0.0, 0.0, 0.0)
            queue.tick(desired, permit=True)
            clock.sleep(.02)
            if queue.done:
                break
        self.assertTrue(queue.done)
        self.assertEqual(queue.commands, 10)
        self.assertEqual(queue.completed_segments, 10)
        self.assertEqual([move[1][4] for move in robot.moves[:-1]], [5.0]*9)
        self.assertEqual(robot.moves[-1][1][4], 0.0)
        self.assertGreaterEqual(queue.max_queue, 2)
        self.assertGreaterEqual(queue.max_active_queue, 1)
        self.assertGreaterEqual(queue.queue_visible_count, 9)
        self.assertGreaterEqual(queue.handoff_count, 8)
        self.assertEqual(robot.abort_count, 0)

    def test_rolling_queue_release_aborts_and_confirms_stop(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(robot, settings, LIMITS,
                                 clock=clock, sleep=clock.sleep)
        queue.initialize()
        queue.tick((420.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        queue.tick(None, permit=False)
        self.assertEqual(robot.abort_count, 1)
        self.assertFalse(queue.active)
        self.assertTrue(queue.stop_confirmed)

    def test_rolling_queue_discards_intermediate_target_while_prequeue_is_full(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(robot, settings, LIMITS,
                                 clock=clock, sleep=clock.sleep)
        queue.initialize()
        queue.tick((420.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        clock.sleep(.02)
        queue.tick((430.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        clock.sleep(.02)
        queue.tick((440.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        self.assertEqual(len(robot.moves),2, "预排槽占用时不得堆积中间目标")
        clock.sleep(.02)
        queue.tick((470.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        self.assertEqual(len(robot.moves),3)
        self.assertEqual(queue.late_handoff_count, 1)
        self.assertAlmostEqual(robot.moves[-1][0][0],470.0)
        self.assertNotAlmostEqual(robot.moves[-1][0][0],440.0)

    def test_rolling_queue_never_treats_pre_busy_idle_as_arrival(self):
        robot, clock = DelayedBusyRollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(robot, settings, LIMITS,
                                 clock=clock, sleep=clock.sleep)
        queue.initialize()
        queue.tick((420.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        clock.sleep(4.1)
        with self.assertRaisesRegex(RuntimeError, "4秒无进展"):
            queue.tick((440.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        self.assertEqual(len(robot.moves),1)
        self.assertEqual(robot.abort_count,1)

    def test_rolling_queue_hidden_second_command_times_out_as_no_progress(self):
        robot, clock = HiddenSecondQueueRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(robot, settings, LIMITS,
                                 clock=clock, sleep=clock.sleep)
        queue.initialize()
        queue.tick((420.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        clock.sleep(.02)
        queue.tick((440.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        self.assertEqual(len(robot.moves),2)
        clock.sleep(4.1)
        with self.assertRaisesRegex(RuntimeError, "4秒无进展"):
            queue.tick((460.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        self.assertEqual(robot.abort_count,1)

    def test_rolling_queue_coalesces_sub_20mm_and_sub_2degree_targets(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(robot, settings, LIMITS,
                                 clock=clock, sleep=clock.sleep)
        queue.initialize()
        queue.tick((419.0,100.0,300.0,0.0,0.0,math.radians(1.9)), permit=True)
        self.assertEqual(robot.moves, [])
        queue.tick((420.0,100.0,300.0,0.0,0.0,math.radians(1.9)), permit=True)
        self.assertEqual(len(robot.moves), 1)

    def test_rolling_queue_extended_profile_reaches_fifty_commands(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(robot, settings, LIMITS, command_limit=50,
                                 minimum_handoffs=8,
                                 clock=clock, sleep=clock.sleep)
        queue.initialize()
        for _ in range(1000):
            desired = (400.0 + 24.0*(queue.commands+1), 100.0, 300.0,
                       0.0, 0.0, 0.0)
            queue.tick(desired, permit=True)
            clock.sleep(.02)
            if queue.done:
                break
        self.assertTrue(queue.done)
        self.assertEqual(queue.commands, 50)
        self.assertEqual(queue.completed_segments, 50)
        self.assertGreaterEqual(queue.handoff_count, 8)
        self.assertEqual(robot.moves[-1][1][4], 0.0)
        self.assertEqual(robot.abort_count, 0)

    def test_continuous_queue_has_no_fifty_command_auto_stop(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(
            robot, settings, LIMITS, command_limit=None, minimum_handoffs=0,
            clock=clock, sleep=clock.sleep)
        queue.initialize()
        for _ in range(2000):
            desired = (400.0 + 24.0*(queue.commands+1), 100.0, 300.0,
                       0.0, 0.0, 0.0)
            queue.tick(desired, permit=True)
            clock.sleep(.02)
            if queue.commands >= 55:
                break
        self.assertGreaterEqual(queue.commands, 55)
        self.assertFalse(queue.done)
        self.assertTrue(all(move[1][4] == 5.0 for move in robot.moves))
        self.assertTrue(queue.shutdown())

    def test_continuous_queue_can_resume_after_grip_release(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(
            robot, settings, LIMITS, command_limit=None, minimum_handoffs=0,
            clock=clock, sleep=clock.sleep)
        queue.initialize()
        queue.tick((424.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        queue.tick(None, permit=False)
        self.assertEqual(queue.commands, 1)
        self.assertFalse(queue.active)
        self.assertTrue(queue.stop_confirmed)
        # 假 SDK 用 aborted 标记模拟停止后的 idle；重新握持前恢复正常空闲态。
        robot.aborted = False
        clock.sleep(.25)
        queue.tick((448.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        self.assertEqual(queue.commands, 2)
        self.assertTrue(queue.active)

    def test_expanded_pose_priority_profile_uses_200mm_s_vendor_queue(self):
        robot, clock = RollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=200.0,
                            acceleration_mm_s2=500.0, segment_mm=80.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(
            robot, settings, LIMITS, command_limit=None, minimum_handoffs=0,
            orientation_speed_deg_s=45.0,
            orientation_acceleration_deg_s2=180.0,
            max_orientation_step_deg=4.0,
            clock=clock, sleep=clock.sleep)
        queue.initialize()
        queue.tick((480.0,100.0,300.0,0.0,0.0,math.radians(4.0)), permit=True)
        self.assertEqual(len(robot.moves),1)
        target,args = robot.moves[0]
        self.assertLessEqual(math.dist(target[:3],robot.tcp[:3]),80.000001)
        self.assertLessEqual(math.degrees(abs(target[5])),4.000001)
        self.assertEqual(args[:5],(0,False,200.0,500.0,5.0))
        self.assertAlmostEqual(math.degrees(args[5]),45.0)
        self.assertAlmostEqual(math.degrees(args[6]),180.0)

    def test_rejected_unsent_target_is_skipped_without_ending_session(self):
        robot, clock, events = RollingRobot(), Clock(), []
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(
            robot, settings, LIMITS, command_limit=None, minimum_handoffs=0,
            emit=lambda **event: events.append(event),
            clock=clock, sleep=clock.sleep)
        queue.initialize()
        with patch("vla_lab.pose_blend_acceptance.plan_pose_segment",
                   side_effect=RejectedPath("测试候选关节变化过大")):
            queue.tick((424.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        self.assertEqual(queue.commands, 0)
        self.assertFalse(queue.active)
        self.assertFalse(queue.done)
        self.assertEqual(events[-1]["state"], "rolling_pose_target_rejected")
        clock.sleep(.21)
        queue.tick((424.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        self.assertEqual(queue.commands, 1)
        self.assertTrue(queue.active)

    def test_rolling_queue_rejects_controller_depth_above_two(self):
        robot, clock = OverflowRollingRobot(), Clock()
        settings = Settings(radius_mm=1000.0, speed_mm_s=150.0,
                            acceleration_mm_s2=400.0, segment_mm=60.0,
                            deadband_mm=3.0)
        queue = RollingPoseQueue(robot, settings, LIMITS,
                                 clock=clock, sleep=clock.sleep)
        queue.initialize()
        queue.tick((420.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        with self.assertRaisesRegex(RuntimeError, "超过本模式深度2边界"):
            queue.tick((440.0,100.0,300.0,0.0,0.0,0.0), permit=True)
        self.assertEqual(robot.abort_count,1)


if __name__ == "__main__":
    unittest.main()
