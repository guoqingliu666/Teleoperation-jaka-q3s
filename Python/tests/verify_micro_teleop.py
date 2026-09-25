"""微动调试安全验收；只运行模拟/纯函数，不连接真实机器人。"""

from __future__ import annotations

import json
import sys
import time
import tkinter as tk
import unittest
from pathlib import Path


PYTHON_ROOT = Path(__file__).resolve().parents[1]
ROOT = PYTHON_ROOT.parent
sys.path.insert(0, str(PYTHON_ROOT / "src"))

from vla_lab.jaka_jog_controller import (  # noqa: E402
    COMMISSION_MAX_RELATIVE_MM,
    COMMISSION_MAX_SPEED_MM_S,
    COMMISSION_TARGET_WATCHDOG_S,
    JakaJogController,
    SHOWCASE_MAX_RELATIVE_MM,
    SHOWCASE_MAX_SESSION_S,
    SHOWCASE_MAX_SPEED_MM_S,
    SHOWCASE_TARGET_WATCHDOG_S,
    limit_commissioning_target,
    limit_showcase_target,
)
from vla_lab.micro_teleop_gui import (  # noqa: E402
    DEADMAN_ON,
    MicroMotionMapper,
    MicroTeleopWindow,
    SHOWCASE_PROFILE,
    SHOWCASE_UI_MAX_RELATIVE_MM,
    UI_MAX_RELATIVE_MM,
    build_parser,
    clamp_vector,
    dual_deadman_pressed,
    project_single_axis,
    translation_only_target,
)
from vla_lab.quest_vr_input import QuestUdpReceiver  # noqa: E402


class MicroTeleopSafetyTests(unittest.TestCase):
    def setUp(self) -> None:
        fixture = ROOT / "Validation" / "unity_v2_fixture.json"
        self.packet = json.loads(fixture.read_text(encoding="utf-8"))
        self.mapping = {
            "forward": "X-", "backward": "X+",
            "left": "Y-", "right": "Y+",
            "up": "Z+", "down": "Z-",
        }

    def test_default_is_demo(self) -> None:
        args = build_parser().parse_args([])
        self.assertFalse(args.live)
        self.assertFalse(args.showcase)

    def test_showcase_flag_selects_independent_profile(self) -> None:
        args = build_parser().parse_args(["--live", "--showcase"])
        self.assertTrue(args.live)
        self.assertTrue(args.showcase)
        self.assertEqual(SHOWCASE_PROFILE.confirmation, "5CM")
        self.assertEqual(SHOWCASE_PROFILE.session_s, SHOWCASE_MAX_SESSION_S)
        self.assertLess(SHOWCASE_UI_MAX_RELATIVE_MM, SHOWCASE_MAX_RELATIVE_MM)
        self.assertEqual(SHOWCASE_PROFILE.hand_scale, 1.0)

    def test_axis_acceptance_projects_other_axes_to_zero(self) -> None:
        self.assertEqual(project_single_axis((1.0, 2.0, 3.0), "x"), (1.0, 0.0, 0.0))
        self.assertEqual(project_single_axis((1.0, 2.0, 3.0), "y"), (0.0, 2.0, 0.0))
        self.assertEqual(project_single_axis((1.0, 2.0, 3.0), "z"), (0.0, 0.0, 3.0))
        self.assertEqual(project_single_axis((1.0, 2.0, 3.0), "free"), (1.0, 2.0, 3.0))

    def test_ui_vector_is_spherical_and_inside_worker_envelope(self) -> None:
        limited = clamp_vector((4.0, 4.0, 4.0), UI_MAX_RELATIVE_MM)
        self.assertAlmostEqual(sum(value * value for value in limited) ** 0.5, 4.0)
        self.assertLess(UI_MAX_RELATIVE_MM, COMMISSION_MAX_RELATIVE_MM)

    def test_target_is_translation_only(self) -> None:
        reference = (100.0, 200.0, 300.0, 0.1, -0.2, 2.5)
        target = translation_only_target(reference, (20.0, 0.0, 0.0))
        self.assertEqual(target[3:], reference[3:])
        self.assertAlmostEqual(math_dist(target[:3], reference[:3]), UI_MAX_RELATIVE_MM)

    def test_worker_freezes_rotation_and_speed_limits(self) -> None:
        reference = (100.0, 200.0, 300.0, 0.1, -0.2, 2.5)
        requested = (104.0, 200.0, 300.0, 9.0, 9.0, 9.0)
        limited = limit_commissioning_target(reference, reference, requested, 0.1)
        self.assertEqual(limited[3:], reference[3:])
        self.assertLessEqual(math_dist(limited[:3], reference[:3]), COMMISSION_MAX_SPEED_MM_S * 0.1 + 1e-9)

    def test_worker_rejects_outside_hard_envelope(self) -> None:
        reference = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            limit_commissioning_target(reference, reference, (5.01, 0.0, 0.0, 0.0, 0.0, 0.0), 0.02)

    def test_showcase_ui_target_is_translation_only_and_below_hard_envelope(self) -> None:
        reference = (100.0, 200.0, 300.0, 0.1, -0.2, 2.5)
        target = translation_only_target(
            reference,
            (100.0, 0.0, 0.0),
            SHOWCASE_UI_MAX_RELATIVE_MM,
        )
        self.assertEqual(target[3:], reference[3:])
        self.assertAlmostEqual(
            math_dist(target[:3], reference[:3]),
            SHOWCASE_UI_MAX_RELATIVE_MM,
        )

    def test_showcase_worker_freezes_rotation_and_speed_limits(self) -> None:
        reference = (100.0, 200.0, 300.0, 0.1, -0.2, 2.5)
        requested = (148.0, 200.0, 300.0, 9.0, 9.0, 9.0)
        limited = limit_showcase_target(reference, reference, requested, 0.1)
        self.assertEqual(limited[3:], reference[3:])
        self.assertLessEqual(
            math_dist(limited[:3], reference[:3]),
            SHOWCASE_MAX_SPEED_MM_S * 0.1 + 1e-9,
        )

    def test_showcase_worker_rejects_outside_hard_envelope(self) -> None:
        reference = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            limit_showcase_target(
                reference,
                reference,
                (SHOWCASE_MAX_RELATIVE_MM + 0.01, 0.0, 0.0, 0.0, 0.0, 0.0),
                0.02,
            )

    def test_showcase_worker_rejects_unsafe_dynamic_settings(self) -> None:
        reference = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            limit_showcase_target(
                reference, reference, reference, 0.02,
                max_relative_mm=SHOWCASE_MAX_RELATIVE_MM + 1.0,
            )
        with self.assertRaises(ValueError):
            limit_showcase_target(
                reference, reference, reference, 0.02,
                max_speed_mm_s=SHOWCASE_MAX_SPEED_MM_S + 1.0,
            )

    def test_both_buttons_are_required(self) -> None:
        self.packet["right"]["grip"] = DEADMAN_ON
        self.packet["right"]["trigger"] = 0.0
        self.assertFalse(dual_deadman_pressed(QuestUdpReceiver._frame(self.packet), already_active=False))
        self.packet["right"]["trigger"] = DEADMAN_ON
        self.assertTrue(dual_deadman_pressed(QuestUdpReceiver._frame(self.packet), already_active=False))

    def test_mapper_clamps_large_hand_motion(self) -> None:
        mapper = MicroMotionMapper(self.mapping)
        mapper.lock_heading((0.0, 0.0, 0.0, 1.0))
        mapper.begin((0.0, 0.0, 0.0))
        delta = mapper.delta_mm((1.0, 1.0, 1.0))
        self.assertLessEqual(math_dist(delta, (0.0, 0.0, 0.0)), UI_MAX_RELATIVE_MM + 1e-9)

    def test_demo_worker_watchdog_stops_without_refresh(self) -> None:
        controller = JakaJogController(demo=True, poll_hz=30.0)
        try:
            controller.login()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not controller.get_snapshot().connected:
                time.sleep(0.02)
            controller.start_commissioning_cartesian_servo(0)
            time.sleep(COMMISSION_TARGET_WATCHDOG_S + 0.20)
            snapshot = controller.get_snapshot()
            self.assertTrue(any("看门狗" in item for item in snapshot.log))
        finally:
            controller.shutdown()

    def test_demo_showcase_watchdog_stops_without_refresh(self) -> None:
        controller = JakaJogController(demo=True, poll_hz=30.0)
        try:
            controller.login()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not controller.get_snapshot().connected:
                time.sleep(0.02)
            controller.start_showcase_cartesian_servo(0)
            time.sleep(SHOWCASE_TARGET_WATCHDOG_S + 0.20)
            snapshot = controller.get_snapshot()
            self.assertTrue(any("showcase" in item and "看门狗" in item for item in snapshot.log))
        finally:
            controller.shutdown()

    def test_real_tk_window_opens_in_demo_without_hardware(self) -> None:
        root = tk.Tk()
        root.withdraw()
        app = MicroTeleopWindow(root, live=False, host="192.0.2.10", quest_port=0)
        try:
            root.update_idletasks()
            root.update()
            self.assertIn("模拟模式", root.title())
            self.assertFalse(app.live)
        finally:
            app.close()

    def test_showcase_tk_window_opens_in_demo_without_hardware(self) -> None:
        root = tk.Tk()
        root.withdraw()
        app = MicroTeleopWindow(
            root,
            live=False,
            host="192.0.2.10",
            quest_port=0,
            profile=SHOWCASE_PROFILE,
        )
        try:
            root.update_idletasks()
            root.update()
            self.assertIn("约 5 cm 展示遥操作", root.title())
            self.assertEqual(app.profile.confirmation, "5CM")
        finally:
            app.close()


def math_dist(a, b) -> float:
    return sum((float(x) - float(y)) ** 2 for x, y in zip(a, b, strict=True)) ** 0.5


if __name__ == "__main__":
    unittest.main(verbosity=2)
