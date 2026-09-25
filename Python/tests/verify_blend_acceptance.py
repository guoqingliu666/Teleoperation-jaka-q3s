"""固定两段厂商圆滑验收核心的离线故障注入。"""
from __future__ import annotations

from pathlib import Path
import importlib.util
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab.blend_acceptance import (  # noqa: E402
    VISIBLE20, VISIBLE24, VISIBLE30, VISIBLE50, build_fixed_corner_plan, execute_fixed_corner,
    scan_fixed_corner_envelope,
)

ENTRY = Path(__file__).resolve().parents[1] / "真机验证_两段圆滑.py"
SPEC = importlib.util.spec_from_file_location("blend_acceptance_entry", ENTRY)
entry = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(entry)

LIMITS = [(-360, 360), (-85, 265), (-175, 175), (-85, 265), (-360, 360), (-360, 360)]


class Clock:
    def __init__(self): self.now = 10.0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class Robot:
    def __init__(self, *, never_busy=False, collision_after_first=False):
        self.tcp = (400.0, 100.0, 300.0, 0.0, 0.0, 0.0)
        self.joints = (0.0,) * 6
        self.moves = []
        self.abort_count = 0
        self.aborted = False
        self.status_reads = 0
        self.never_busy = never_busy
        self.collision_after_first = collision_after_first
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
        return (0, tuple((pose[i] - 400.0 if i == 0 else pose[i] - 100.0 if i == 1 else pose[i] - 300.0) * .001 for i in range(3)) + (0.0,) * 3)
    def get_motion_status(self):
        self.status_reads += 1
        if self.aborted:
            return (0, [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        if not self.moves or self.never_busy:
            return (0, [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        if self.collision_after_first:
            return (0, [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1])
        if len(self.moves) == 1:
            return (0, [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])
        if self.status_reads < 7:
            return (0, [2, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0])
        self.tcp = self.moves[-1][0]
        return (0, [2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
    def linear_move_extend(self, target, *args):
        self.moves.append((target, args))
        return (0,)
    def motion_abort(self):
        self.abort_count += 1
        self.aborted = True
        return (0,)


class BlendAcceptanceTests(unittest.TestCase):
    def test_entry_default_is_no_connection_and_live_requires_phrase(self):
        self.assertEqual(entry.main([]), 0)
        with self.assertRaisesRegex(SystemExit, "必须添加"):
            entry.main(["--live-two-segment"])

    def test_two_vendor_segments_use_blend_then_fine_endpoint(self):
        robot, clock = Robot(), Clock()
        plan = build_fixed_corner_plan(robot, LIMITS)
        result = execute_fixed_corner(robot, plan, clock=clock, sleep=clock.sleep)
        self.assertEqual(result["commands_sent"], 2)
        self.assertEqual([move[1][-1] for move in robot.moves], [1.0, 0.0])
        self.assertEqual(robot.abort_count, 0)
        self.assertEqual(robot.tcp, plan.final_tcp)
        self.assertGreaterEqual(result["max_queue"], 1)
        self.assertGreaterEqual(result["max_active_queue"], 1)

    def test_joint_metrics_are_finite_and_small_for_fixture(self):
        robot = Robot()
        plan = build_fixed_corner_plan(robot, LIMITS)
        metrics = entry.joint_path_metrics(plan, LIMITS)
        self.assertEqual(metrics["sample_count"], 7)
        self.assertLess(metrics["max_adjacent_joint_step_deg"], 3.0)
        self.assertGreater(metrics["minimum_configured_joint_margin_deg"], 3.0)

    def test_visible_profile_is_fixed_20mm_and_uses_3mm_blend(self):
        robot, clock = Robot(), Clock()
        plan = build_fixed_corner_plan(robot, LIMITS, VISIBLE20)
        result = execute_fixed_corner(robot, plan, clock=clock, sleep=clock.sleep)
        self.assertEqual(plan.corner_tcp[2] - plan.start_tcp[2], 20.0)
        self.assertEqual(plan.final_tcp[0] - plan.corner_tcp[0], 20.0)
        self.assertEqual(robot.moves[0][1], (0, False, 10.0, 20.0, 3.0))
        self.assertEqual(robot.moves[1][1], (0, False, 10.0, 20.0, 0.0))
        self.assertTrue(result["stop_confirmed"])

    def test_next_profile_is_fixed_50mm_and_requires_its_own_confirmation(self):
        robot = Robot()
        plan = build_fixed_corner_plan(robot, LIMITS, VISIBLE50)
        self.assertEqual(plan.corner_tcp[2] - plan.start_tcp[2], 50.0)
        self.assertEqual(plan.final_tcp[0] - plan.corner_tcp[0], 50.0)
        self.assertEqual(plan.profile.speed_mm_s, 15.0)
        self.assertEqual(plan.profile.acceleration_mm_s2, 30.0)
        self.assertEqual(plan.profile.blend_tolerance_mm, 5.0)
        self.assertEqual(plan.profile.confirmation, "执行固定两段50MM圆滑")

    def test_intermediate_profile_is_fixed_30mm(self):
        robot = Robot()
        plan = build_fixed_corner_plan(robot, LIMITS, VISIBLE30)
        self.assertEqual(plan.corner_tcp[2] - plan.start_tcp[2], 30.0)
        self.assertEqual(plan.final_tcp[0] - plan.corner_tcp[0], 30.0)
        self.assertEqual((plan.profile.speed_mm_s, plan.profile.acceleration_mm_s2, plan.profile.blend_tolerance_mm),
                         (12.0, 24.0, 4.0))

    def test_next_live_profile_preserves_20mm_speed_and_has_24mm_segments(self):
        robot = Robot()
        plan = build_fixed_corner_plan(robot, LIMITS, VISIBLE24)
        self.assertEqual(plan.corner_tcp[2] - plan.start_tcp[2], 24.0)
        self.assertEqual(plan.final_tcp[0] - plan.corner_tcp[0], 24.0)
        self.assertEqual((plan.profile.speed_mm_s, plan.profile.acceleration_mm_s2, plan.profile.blend_tolerance_mm),
                         (10.0, 20.0, 3.0))

    def test_envelope_scan_is_read_only_and_reports_largest_candidate(self):
        robot = Robot()
        result = scan_fixed_corner_envelope(robot, LIMITS, minimum_mm=5, maximum_mm=8)
        self.assertEqual(result["accepted_segment_mm"], [5, 6, 7, 8])
        self.assertEqual(result["largest_accepted_segment_mm"], 8)
        self.assertEqual(result["rejected_segment_mm"], {})
        self.assertEqual(robot.moves, [])

    def test_never_observing_busy_state_blocks_second_segment(self):
        robot, clock = Robot(never_busy=True), Clock()
        plan = build_fixed_corner_plan(robot, LIMITS)
        with self.assertRaisesRegex(RuntimeError, "未观察到队列忙态"):
            execute_fixed_corner(robot, plan, clock=clock, sleep=clock.sleep)
        self.assertEqual(len(robot.moves), 1)
        self.assertEqual(robot.abort_count, 1)

    def test_motion_fault_after_first_segment_aborts(self):
        robot, clock = Robot(collision_after_first=True), Clock()
        plan = build_fixed_corner_plan(robot, LIMITS)
        with self.assertRaisesRegex(RuntimeError, "状态禁止继续"):
            execute_fixed_corner(robot, plan, clock=clock, sleep=clock.sleep)
        self.assertEqual(len(robot.moves), 1)
        self.assertEqual(robot.abort_count, 1)


if __name__ == "__main__":
    unittest.main()
