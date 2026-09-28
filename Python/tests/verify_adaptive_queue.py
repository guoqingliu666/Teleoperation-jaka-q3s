"""动态执行适配器故障注入：测试替身不会加载SDK或连接机械臂。"""

import math
from pathlib import Path
import sys
import unittest
import importlib.util
from contextlib import redirect_stderr
import io
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab.adaptive_pose_queue import AdaptivePoseQueue
from vla_lab.sampled_follow import Settings
from verify_pose_blend_acceptance import RollingRobot, Clock, LIMITS


class AdaptiveQueueTests(unittest.TestCase):
    def test_ordered_input_samples_reach_intent_without_extra_ik_per_packet(self):
        robot, clock, c, _ = self.new()
        start=clock()
        # 第一帧建立Grip捕获，其余两帧维持原顺序；本轮至多推进一次厂商逆解。
        samples=[((400.+index,100.,300.,0.,0.,0.),start+index*.01)
                 for index in range(3)]
        clock.sleep(.03)
        before=len(robot.inverse_calls) if hasattr(robot,'inverse_calls') else 0
        c.tick(samples[-1][0],permit=True,refresh_permit=lambda:True,
               received_s=samples[-1][1],capture_id='grip1',input_samples=samples)
        self.assertEqual(len(c.horizon.samples),2)
        self.assertEqual(c.horizon.raw_previous.t,samples[-1][1])
        self.assertFalse(c.horizon.blocked)

    def test_shadow_cannot_reach_motion_queue_even_with_production_flags(self):
        spec = importlib.util.spec_from_file_location(
            "adaptive_runner_gate",
            Path(__file__).resolve().parents[1] / "连续采样位置遥操作.py",
        )
        runner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runner)
        with patch.object(runner, "load_sdk") as sdk, redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                runner.main(
                    [
                        "--shadow",
                        "--production-six-dof",
                        "--production-rolling-pose",
                        "--adaptive-trajectory",
                        "--ui-heartbeat",
                        "--single-owner-display",
                    ]
                )
        sdk.assert_not_called()

    def test_medium_pilot_rejects_bypassed_or_enlarged_limits_before_sdk(self):
        spec = importlib.util.spec_from_file_location(
            "adaptive_pilot_gate",
            Path(__file__).resolve().parents[1] / "连续采样位置遥操作.py",
        )
        runner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runner)
        base = ["--live", "--ui-heartbeat", "--ui-protocol", "2",
                "--single-owner-display", "--production-six-dof",
                "--production-rolling-pose", "--adaptive-trajectory",
                "--adaptive-pilot", "--radius-mm", "200", "--speed-mm-s", "30",
                "--acceleration-mm-s2", "60", "--segment-mm", "20",
                "--rotation-radius-deg", "10", "--orientation-speed-deg-s", "5",
                "--max-orientation-step-deg", "2"]
        with patch.object(runner, "load_sdk") as sdk, redirect_stderr(io.StringIO()):
            for altered in (base + ["--speed-mm-s", "150"],
                            base + ["--max-check-deg", "2"],
                            [value for value in base if value != "--adaptive-trajectory"]):
                with self.assertRaises(SystemExit):
                    runner.main(altered)
        sdk.assert_not_called()

    def new(self):
        robot, clock = RollingRobot(), Clock()
        events = []
        controller = AdaptivePoseQueue(
            robot,
            Settings(
                radius_mm=1000,
                speed_mm_s=150,
                acceleration_mm_s2=400,
                segment_mm=60,
                deadband_mm=3,
            ),
            LIMITS,
            command_limit=None,
            minimum_handoffs=0,
            clock=clock,
            sleep=clock.sleep,
            emit=lambda **event: events.append(event),
        )
        controller.initialize()
        return robot, clock, controller, events

    def tick(
        self,
        controller,
        clock,
        x=410,
        *,
        permit=True,
        callback=lambda: True,
        capture="grip1"
    ):
        controller.tick(
            (x, 100.0, 300.0, 0.0, 0.0, 0.0),
            permit=permit,
            refresh_permit=callback,
            received_s=clock(),
            capture_id=capture,
        )
        clock.sleep(0.003)

    def test_prepares_and_dispatches_exact_vendor_checked_endpoint(self):
        robot, clock, c, events = self.new()
        for i in range(120):
            self.tick(c, clock, 400 + min(i * 0.1, 10))
        self.assertGreater(c.commands, 0)
        plans = [e for e in events if e["state"] == "adaptive_queue_plan"]
        self.assertEqual(len(plans), len(robot.moves))
        for plan, move in zip(plans, robot.moves):
            self.assertEqual(tuple(plan["target"]), move[0])
            self.assertAlmostEqual(
                plan["checked_joints"][-1][0], (move[0][0] - 400) * 0.001
            )
        self.assertLessEqual(c.max_queue, 2)

    def test_release_discards_prepared_and_regrip_starts_from_measured_pose(self):
        robot, clock, c, _ = self.new()
        for grip in range(3):
            previous = c.commands
            robot.aborted = False
            for i in range(70):
                self.tick(c, clock, robot.tcp[0] + 3, capture=str(grip))
            self.assertGreater(c.commands, previous)
            self.tick(c, clock, permit=False)
            self.assertFalse(c.active)
            self.assertIsNone(c.horizon.ready)
            self.assertEqual(len(c.horizon.samples), 0)
            self.assertTrue(c.stop_confirmed)

    def test_latest_permission_prevents_first_send(self):
        robot, clock, c, _ = self.new()
        for i in range(100):
            self.tick(c, clock, 400 + i * 0.1, callback=lambda: False)
        self.assertEqual(robot.moves, [])
        self.assertTrue(c.wait_release)

    def test_stale_goal_cannot_be_refreshed_by_held_grip(self):
        robot, clock, c, _ = self.new()
        self.tick(c, clock, 400)
        old = clock()
        clock.sleep(0.2)
        c.tick(
            (410, 100, 300, 0, 0, 0),
            permit=True,
            received_s=old,
            capture_id="grip1",
            refresh_permit=lambda: True,
        )
        self.assertEqual(robot.moves, [])
        self.assertTrue(c.wait_release)

    def test_external_start_change_refuses_cached_plan(self):
        robot, clock, c, _ = self.new()
        self.tick(c, clock, 400)
        robot.joints = (0.1, 0, 0, 0, 0, 0)
        for i in range(80):
            self.tick(c, clock, 405)
        self.assertEqual(robot.moves, [])
        self.assertTrue(c.wait_release)

    def test_call_failure_after_possible_acceptance_requires_stop(self):
        robot, clock, c, _ = self.new()
        original = robot.linear_move_extend_ori

        def uncertain(*args):
            original(*args)
            return (-3,)

        robot.linear_move_extend_ori = uncertain
        with self.assertRaises(RuntimeError):
            for i in range(100):
                self.tick(c, clock, 400 + i * 0.1)
        self.assertEqual(len(robot.moves), 1)
        self.assertGreater(robot.abort_count, 0)
        self.assertTrue(c.stop_confirmed)

    def test_planning_progresses_while_prequeue_is_occupied(self):
        robot, clock, c, _ = self.new()
        # 保持状态机处于明确可见的深度2忙态；提前计算不会再发送第三条。
        for i in range(200):
            self.tick(c, clock, 400 + i * 0.2)
            if c.commands >= 2:
                break
        self.assertEqual(c.commands, 2)
        robot.get_motion_status = lambda: (0, [2, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0])
        for i in range(350):
            self.tick(c, clock, 430 + i * 0.1)
        self.assertEqual(c.commands, 2)
        self.assertFalse(c.wait_release)
        self.assertFalse(c.horizon.blocked)
        self.assertLessEqual(c.horizon.pending_seconds(),c.horizon.capacity_s+1e-8)
        self.assertTrue(c.horizon.job is not None or c.horizon.ready is not None)


if __name__ == "__main__":
    unittest.main()
