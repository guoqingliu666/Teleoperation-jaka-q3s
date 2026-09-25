"""固定三段厂商圆滑验收的离线故障注入测试。"""
from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab.multisegment_acceptance import (  # noqa: E402
    TRI20, build_three_segment_plan, execute_three_segment, path_metrics,
)

LIMITS = [(-360, 360), (-85, 265), (-175, 175), (-85, 265), (-360, 360), (-360, 360)]


class Clock:
    def __init__(self): self.now = 10.0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class Robot:
    def __init__(self, *, collision_on_handoff=False):
        self.tcp = (400.0, 100.0, 300.0, 0.0, 0.0, 0.0)
        self.joints = (0.0,) * 6
        self.moves = []
        self.aborted = False
        self.abort_count = 0
        self.handoff_reads = 0
        self.final_reads = 0
        self.collision_on_handoff = collision_on_handoff
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
        return (0, tuple((pose[i] - (400.0, 100.0, 300.0)[i]) * .001 for i in range(3)) + (0.0,) * 3)
    def linear_move_extend(self, target, *args):
        self.moves.append((target, args))
        return (0,)
    def motion_abort(self):
        self.aborted = True
        self.abort_count += 1
        return (0,)
    def get_motion_status(self):
        if self.aborted or not self.moves:
            return (0, [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        if len(self.moves) == 1:
            return (0, [1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0])
        if len(self.moves) == 2:
            self.handoff_reads += 1
            if self.collision_on_handoff:
                return (0, [1, 2, 0, 0, 1, 1, 0, 0, 0, 0, 1])
            return (0, [2, 2, 0, 0, 1, 1, 0, 0, 0, 0, 0])
        self.final_reads += 1
        if self.final_reads < 3:
            return (0, [3, 3, 0, 0, 1, 1, 0, 0, 0, 0, 0])
        self.tcp = self.moves[-1][0]
        return (0, [3, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0])


class ThreeSegmentTests(unittest.TestCase):
    def test_plan_has_fixed_z_x_negative_z_path(self):
        plan = build_three_segment_plan(Robot(), LIMITS)
        self.assertEqual(plan.targets[0][:3], (400.0, 100.0, 320.0))
        self.assertEqual(plan.targets[1][:3], (420.0, 100.0, 320.0))
        self.assertEqual(plan.targets[2][:3], (420.0, 100.0, 300.0))
        metrics = path_metrics(plan, LIMITS)
        self.assertEqual(metrics["sample_count"], 31)
        self.assertLess(metrics["max_adjacent_joint_step_deg"], 3.0)

    def test_execution_keeps_at_most_two_queued_and_fine_stops_last_leg(self):
        robot, clock = Robot(), Clock()
        plan = build_three_segment_plan(robot, LIMITS)
        result = execute_three_segment(robot, plan, clock=clock, sleep=clock.sleep)
        self.assertEqual(result["commands_sent"], 3)
        self.assertEqual([move[1][-1] for move in robot.moves], [3.0, 3.0, 0.0])
        self.assertEqual(result["max_queue"], 1)
        self.assertEqual(result["max_active_queue"], 1)
        self.assertEqual(robot.abort_count, 0)

    def test_fault_before_third_segment_requests_abort(self):
        robot, clock = Robot(collision_on_handoff=True), Clock()
        plan = build_three_segment_plan(robot, LIMITS)
        with self.assertRaisesRegex(RuntimeError, "状态禁止继续"):
            execute_three_segment(robot, plan, clock=clock, sleep=clock.sleep)
        self.assertEqual(len(robot.moves), 2)
        self.assertEqual(robot.abort_count, 1)

    def test_profile_is_explicit_low_speed_gate(self):
        self.assertEqual((TRI20.leg_mm, TRI20.speed_mm_s, TRI20.acceleration_mm_s2, TRI20.blend_tolerance_mm),
                         (20.0, 10.0, 20.0, 3.0))
        self.assertEqual(TRI20.confirmation, "执行固定三段20MM圆滑")


if __name__ == "__main__":
    unittest.main()
