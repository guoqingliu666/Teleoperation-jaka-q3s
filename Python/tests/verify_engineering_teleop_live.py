"""事故前控制逻辑的离线回归检查；不连接真实机器人。

2026-09-21 恢复了事故前控制代码，但真机入口仍被硬锁定。本文件中标为
expectedFailure 的项目是旧版已知安全缺口，不能把其它测试通过当作上机许可。
"""

from __future__ import annotations

import math
import json
import socket
import sys
import time
import tkinter as tk
import unittest
from unittest.mock import patch
from pathlib import Path

PYTHON_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PYTHON_ROOT / "src"))

from vla_lab.engineering_teleop_live import LevelALiveWindow, LevelASettings, compose_target, main  # noqa: E402
from vla_lab.jaka_jog_controller import (  # noqa: E402
    ENGINEERING_LIVE_LOCKOUT_REASON,
    ENGINEERING_MAX_ANGULAR_SPEED_DEG_S, ENGINEERING_MAX_JOINT_SPEED_DEG_S,
    ENGINEERING_MAX_LINEAR_SPEED_MM_S, ENGINEERING_MAX_RELATIVE_MM,
    ENGINEERING_MAX_ROTATION_DEG, ENGINEERING_REQUIRED_TOOL_ID,
    ENGINEERING_TARGET_WATCHDOG_S,
    JakaJogController, RobotSnapshot,
    _snapshot_dict,
    guard_engineering_joint_rate,
    limit_engineering_target, project_engineering_target,
    project_engineering_target_with_orientation_priority,
)
from vla_lab.quest_vr_input import QuestUdpReceiver, axis_map, matvec, rpy_matrix  # noqa: E402
from vla_lab.vr_robot_visualization import RobotVrBroadcaster  # noqa: E402


SAFE_JOINTS = tuple(math.radians(value) for value in (0.0, 0.0, -60.0, 0.0, 30.0, 0.0))


def settings(**updates) -> LevelASettings:
    values = dict(
        radius_mm=50.0, translation_scale=1.0, linear_speed_mm_s=20.0,
        rotation_enabled=True, rotation_scale=1.0, rotation_deg=2.0,
        angular_speed_deg_s=3.0, joint_speed_deg_s=3.0,
        xyz_axes=(True, True, True), rpy_axes=(True, True, True),
    )
    values.update(updates)
    return LevelASettings(**values)


class LevelALiveTests(unittest.TestCase):
    def test_incident_lockout_rejects_live_before_gui_or_robot_connection(self) -> None:
        """事故版本控制链未重新验收时，真机入口必须在创建窗口和连接硬件前失败。"""
        with patch("vla_lab.engineering_teleop_live.tk.Tk") as create_window:
            self.assertEqual(main(["--live", "--host", "192.0.2.10"]), 90)
            create_window.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, "涉险事件后已锁定"):
            LevelALiveWindow(object(), live=True, host="192.0.2.10")
        self.assertIn("禁止再次 ARM", ENGINEERING_LIVE_LOCKOUT_REASON)

    def test_installed_dexterous_hand_requires_tool_one(self) -> None:
        self.assertEqual(ENGINEERING_REQUIRED_TOOL_ID, 1)

    def test_settings_accept_level_a_maximum(self) -> None:
        maximum = settings(
            radius_mm=ENGINEERING_MAX_RELATIVE_MM,
            linear_speed_mm_s=ENGINEERING_MAX_LINEAR_SPEED_MM_S,
            rotation_deg=ENGINEERING_MAX_ROTATION_DEG,
            angular_speed_deg_s=ENGINEERING_MAX_ANGULAR_SPEED_DEG_S,
            joint_speed_deg_s=ENGINEERING_MAX_JOINT_SPEED_DEG_S,
        )
        self.assertIsNone(maximum.validate())

    def test_calibrated_hand_directions_match_reported_physical_motion(self) -> None:
        """现场报告的左右、前后反向必须由配置修正，并防止以后回归。"""

        config = json.loads((PYTHON_ROOT / "config" / "jaka_jog.json").read_text(encoding="utf-8"))
        mapping = axis_map(config["quest_vr"]["direction_mapping"])
        self.assertEqual(matvec(mapping, (0.0, 0.0, 1.0)), (1.0, 0.0, 0.0))   # 手向前 → JAKA X+
        self.assertEqual(matvec(mapping, (-1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))  # 手向左 → JAKA Y+

    def test_settings_reject_beyond_one_meter_budget(self) -> None:
        with self.assertRaises(ValueError):
            settings(radius_mm=1000.1).validate()

    def test_compose_target_freezes_unchecked_axes(self) -> None:
        reference = (400.0, 100.0, 500.0, 0.1, 0.2, 0.3)
        target = compose_target(reference, (20.0, 30.0, 40.0), rpy_matrix((0.02, 0.03, 0.04)),
                                settings(xyz_axes=(True, False, False), rpy_axes=(True, False, False)))
        self.assertAlmostEqual(target[0], 420.0)
        self.assertEqual(target[1:3], reference[1:3])
        self.assertLess(math.degrees(abs(target[4] - reference[4])), 0.1)
        self.assertLess(math.degrees(abs(target[5] - reference[5])), 0.1)

    def test_free_6d_composes_all_translation_and_rotation_axes(self) -> None:
        reference = (400.0, 100.0, 500.0, 0.1, 0.2, 0.3)
        target = compose_target(
            reference,
            (10.0, 20.0, 30.0),
            rpy_matrix((0.03, -0.04, 0.05)),
            settings(),
        )
        self.assertEqual(target[:3], (410.0, 120.0, 530.0))
        self.assertTrue(all(abs(target[index] - reference[index]) > 1e-4 for index in range(3, 6)))

    def test_worker_limits_translation_and_rotation_rate(self) -> None:
        reference = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        requested = (50.0, 0.0, 0.0, math.radians(2), 0.0, 0.0)
        result = limit_engineering_target(
            reference, reference, requested, 0.1,
            max_relative_mm=100, max_linear_speed_mm_s=20,
            max_rotation_deg=5, max_angular_speed_deg_s=3,
        )
        self.assertLessEqual(math.dist(result[:3], reference[:3]), 2.0 + 1e-9)
        self.assertLessEqual(math.degrees(abs(result[3])), 0.3 + 1e-6)

    def test_worker_saturates_boundary_noise_and_rejects_unsafe_setting(self) -> None:
        reference = (0.0,) * 6
        result = limit_engineering_target(
            reference, reference, (50.008, 0, 0, 0, 0, 0), 1.0,
            max_relative_mm=50, max_linear_speed_mm_s=150,
            max_rotation_deg=5, max_angular_speed_deg_s=3,
        )
        self.assertLessEqual(math.dist(result[:3], reference[:3]), 50.0 + 1e-9)
        with self.assertRaises(ValueError):
            limit_engineering_target(
                reference, reference, reference, 0.02,
                max_relative_mm=ENGINEERING_MAX_RELATIVE_MM + 1, max_linear_speed_mm_s=20,
                max_rotation_deg=5, max_angular_speed_deg_s=3,
            )

    def test_unreachable_ik_projects_to_last_reachable_point(self) -> None:
        previous = (0.0,) * 6
        candidate = (10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        joints = SAFE_JOINTS

        def solve(_reference_joints, pose):
            if pose[0] > 4.0:
                raise RuntimeError("unreachable")
            result = list(SAFE_JOINTS)
            result[0] = pose[0] * 0.0001
            return tuple(result)

        pose, _joints, _rate, limited, notice = project_engineering_target(
            previous, candidate, joints, 0.1, 30.0, solve,
        )
        self.assertTrue(limited)
        self.assertTrue(notice)
        self.assertGreater(pose[0], 3.5)
        self.assertLessEqual(pose[0], 4.0)

    def test_joint_rate_excess_is_scaled_not_aborted(self) -> None:
        previous_pose = (0.0,) * 6
        candidate_pose = (10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        previous_joints = SAFE_JOINTS

        def solve(_reference_joints, pose):
            result = list(SAFE_JOINTS)
            result[0] = pose[0] * 0.001
            return tuple(result)

        pose, joints, requested_rate, limited = guard_engineering_joint_rate(
            previous_pose, candidate_pose, previous_joints, 0.1, 3.0, solve,
        )
        self.assertTrue(limited)
        self.assertGreater(requested_rate, 3.0)
        self.assertGreater(pose[0], 0.0)
        self.assertLess(pose[0], candidate_pose[0])
        self.assertLessEqual(math.degrees(joints[0]) / 0.1, 3.0 * 1.02 + 1e-6)

    def test_unreachable_translation_does_not_swallow_reachable_orientation(self) -> None:
        """工作空间边缘的平移失败时，仍应尽量响应手柄转腕。"""

        previous = (0.0,) * 6
        candidate = (10.0, 0.0, 0.0, 0.0, 0.0, math.radians(8.0))
        joints = SAFE_JOINTS

        def solve(_reference_joints, pose):
            # 模拟“向 X 再走一步不可达，但原地转腕可达”的真机边界。
            if pose[0] > 0.0:
                raise RuntimeError("translation unreachable")
            result = list(SAFE_JOINTS)
            result[5] = pose[5] * 0.1
            return tuple(result)

        pose, _joints, _rate, limited, notice = (
            project_engineering_target_with_orientation_priority(
                previous, candidate, joints, 1.0, 30.0, solve,
            )
        )
        self.assertTrue(limited)
        self.assertEqual(pose[:3], previous[:3])
        self.assertGreater(math.degrees(abs(pose[5])), 1.0)
        self.assertIn("优先追踪姿态", notice)

    @unittest.expectedFailure
    def test_non_ik_sdk_fault_is_not_masked_as_workspace_boundary(self) -> None:
        """旧版已知缺口：非逆解 SDK 故障可能被误当成工作空间边界。"""

        def solve(_reference_joints, _pose):
            raise RuntimeError("SDK transport timeout")

        with self.assertRaisesRegex(RuntimeError, "transport timeout"):
            project_engineering_target(
                (0.0,) * 6, (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                SAFE_JOINTS, 0.1, 60.0, solve,
            )

    def test_command_ack_does_not_clear_measured_state(self) -> None:
        controller = object.__new__(JakaJogController)
        controller._snapshot = RobotSnapshot(
            connected=True, powered_on=True, enabled=True, tool_id=1,
            timestamp_ns=123456, tcp_pose=(1.0,) * 6, joints_rad=(2.0,) * 6,
        )
        controller._apply_locked(("snapshot", _snapshot_dict(
            connected=True, powered_on=True, enabled=True, tool_id=1,
            message="command accepted",
        )))
        self.assertEqual(controller._snapshot.timestamp_ns, 123456)
        self.assertEqual(controller._snapshot.tcp_pose, (1.0,) * 6)
        self.assertEqual(controller._snapshot.joints_rad, (2.0,) * 6)

    def test_demo_engineering_watchdog(self) -> None:
        controller = JakaJogController(demo=True, poll_hz=30)
        try:
            controller.login()
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and not controller.get_snapshot().connected:
                time.sleep(0.02)
            controller.set_tool_id(ENGINEERING_REQUIRED_TOOL_ID)
            time.sleep(0.05)
            controller.start_engineering_cartesian_servo()
            start_deadline = time.monotonic() + 1
            while time.monotonic() < start_deadline and not controller.get_snapshot().engineering_servo_active:
                time.sleep(0.01)
            self.assertTrue(controller.get_snapshot().engineering_servo_active)
            time.sleep(ENGINEERING_TARGET_WATCHDOG_S + 0.20)
            snap = controller.get_snapshot()
            self.assertTrue(any("engineering" in item and "看门狗" in item for item in snap.log))
            self.assertFalse(snap.engineering_servo_active)
        finally:
            controller.shutdown()

    def test_window_opens_in_demo(self) -> None:
        root = tk.Tk(); root.withdraw()
        app = LevelALiveWindow(root, live=False, host="192.0.2.10", quest_port=0)
        try:
            root.update_idletasks(); root.update()
            self.assertIn("模拟", root.title())
            self.assertEqual(app.radius_cm.get(), 100.0)
            self.assertEqual(app.linear_speed.get(), 50.0)
            self.assertEqual(app.angular_speed.get(), 15.0)
            self.assertEqual(app.joint_speed.get(), 15.0)
            self.assertTrue(app.rotation_enabled.get())
            self.assertTrue(all(axis.get() for axis in app.xyz_axes + app.rpy_axes))
            self.assertFalse(any(check.get() for check in app.checks))
            self.assertEqual(app.confirm.get(), "")
        finally:
            app.close()

    def test_pause_keeps_arm_and_allows_in_process_resume(self) -> None:
        root = tk.Tk(); root.withdraw()
        app = LevelALiveWindow(root, live=False, host="192.0.2.10", quest_port=0)
        try:
            app.armed = True
            app.active = True
            app.reference = (0.0,) * 6
            app.pause("test_pause", require_release=True)
            self.assertTrue(app.armed)
            self.assertFalse(app.active)
            self.assertTrue(app.paused)
            self.assertTrue(app.must_release_before_resume)
            self.assertIsNone(app.reference)
        finally:
            app.close()

    def test_gui_waits_for_worker_ack_before_marking_motion_active(self) -> None:
        root = tk.Tk(); root.withdraw()
        app = LevelALiveWindow(root, live=False, host="192.0.2.10", quest_port=0)
        try:
            packet = json.loads((PYTHON_ROOT.parent / "Validation" / "unity_v2_fixture.json").read_text())
            frame = QuestUdpReceiver._frame(packet)
            app.settings = settings()
            snap = RobotSnapshot(tcp_pose=(400.0, 100.0, 500.0, 0.0, 0.0, 0.0))
            with patch.object(app.controller, "start_engineering_cartesian_servo") as start:
                app._begin(frame, snap)
            start.assert_called_once()
            self.assertTrue(app.starting)
            self.assertFalse(app.active, "GUI must not claim motion before worker servo=True acknowledgement")
        finally:
            app.close()

    def test_ui_has_no_power_or_enable_controls(self) -> None:
        source = (PYTHON_ROOT / "src" / "vla_lab" / "engineering_teleop_live.py").read_text(encoding="utf-8")
        self.assertNotIn(".power_on(", source)
        self.assertNotIn(".enable(", source)
        self.assertNotIn("gripper_", source)
        self.assertIn("POSE5", source)
        self.assertIn("self.checks", source)

    def test_vr_broadcast_uses_measured_feedback(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1)
            port = receiver.getsockname()[1]
            broadcaster = RobotVrBroadcaster(port=port, max_hz=120)
            try:
                measured_joints = tuple(0.1 * index for index in range(6))
                snap = RobotSnapshot(
                    connected=True, powered_on=True, enabled=True, tool_id=1,
                    joints_rad=measured_joints, tcp_pose=(400.0, 100.0, 500.0, 0.0, 0.1, 0.2),
                    engineering_servo_active=True,
                )
                target = (410.0, 120.0, 530.0, 0.2, 0.3, 0.4)
                broadcaster.publish(snap, armed=True, starting=False, active=True,
                                    target_tcp_mm_rad=target)
                packet = json.loads(receiver.recvfrom(65535)[0].decode("utf-8"))
                self.assertEqual(packet["schema"], "quest_jaka_robot_state.v1")
                self.assertEqual(packet["joints_rad"], list(measured_joints))
                self.assertEqual(packet["target_tcp_mm_rad"], list(target))
                self.assertTrue(packet["servo_active"])
            finally:
                broadcaster.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
