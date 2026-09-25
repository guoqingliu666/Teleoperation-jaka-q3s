"""姿态只读影子的离线验收：只读反馈 + 厂商逆解，绝不调用运动接口。"""
from dataclasses import replace
import ast
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from vla_lab.rotation_shadow import RotationShadow
from verify_sampled_follow import MAPPING, Robot, frame, Clock


def q_y(degrees):
    half = math.radians(degrees) / 2
    return (0.0, math.sin(half), 0.0, math.cos(half))


class Tests(unittest.TestCase):
    def test_small_rotation_updates_preview_and_vendor_ik_without_motion(self):
        robot, clock, events = Robot(), Clock(), []
        shadow = RotationShadow(robot, MAPPING, lambda **item: events.append(item))
        released = frame(clock, grip=0)
        shadow.ready_heading(released, clock.now)
        clock.now += .01
        held = replace(frame(clock, grip=1), rotation_xyzw=q_y(0), rotation_valid=True)
        shadow.process(held, clock.now)
        clock.now += .01
        rotated = replace(frame(clock, grip=1), rotation_xyzw=q_y(1), rotation_valid=True)
        shadow.process(rotated, clock.now)

        solved = [item for item in events if item.get("state") == "rotation_shadow"]
        self.assertEqual(len(solved), 2)  # Grip锚点恒等姿态 + 1°姿态帧。
        self.assertEqual(tuple(solved[-1]["target"][:3]), robot.tcp[:3])
        self.assertGreater(abs(solved[-1]["target"][3]) + abs(solved[-1]["target"][4])
                           + abs(solved[-1]["target"][5]), 0)
        self.assertEqual(robot.moves, [])

    def test_large_single_frame_rotation_is_blocked(self):
        robot, clock, events = Robot(), Clock(), []
        shadow = RotationShadow(robot, MAPPING, lambda **item: events.append(item))
        released = frame(clock, grip=0)
        shadow.ready_heading(released, clock.now)
        clock.now += .01
        shadow.process(replace(frame(clock, grip=1), rotation_xyzw=q_y(0)), clock.now)
        clock.now += .01
        shadow.process(replace(frame(clock, grip=1), rotation_xyzw=q_y(8)), clock.now)
        self.assertTrue(any(item.get("state") == "rotation_shadow_blocked" for item in events))
        self.assertEqual(shadow.solved_frames, 1)  # Grip锚点帧通过，8°突变帧拒绝。
        self.assertEqual(robot.moves, [])

    def test_source_contains_no_robot_motion_method_calls(self):
        source = Path(__file__).resolve().parents[1] / "src" / "vla_lab" / "rotation_shadow.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        called = {node.func.attr for node in ast.walk(tree)
                  if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
        forbidden = {"linear_move", "linear_move_extend", "joint_move", "servo_j",
                     "servo_move_enable", "motion_abort", "power_on", "enable_robot"}
        self.assertFalse(called & forbidden, called & forbidden)


if __name__ == "__main__":
    unittest.main()
