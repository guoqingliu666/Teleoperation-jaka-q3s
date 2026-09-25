"""1:1 参数化预览器离线验收；不连接真实机器人。"""

from __future__ import annotations

import sys
import tkinter as tk
import unittest
from pathlib import Path


PYTHON_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PYTHON_ROOT / "src"))

from vla_lab.engineering_teleop_preview import (  # noqa: E402
    EngineeringTeleopPreview,
    PreviewSettings,
    rate_limit_vector,
)


class PreviewTests(unittest.TestCase):
    def test_settings_accept_requested_maximum(self) -> None:
        settings = PreviewSettings(100.0, 1.0, 200.0, True, 1.0, 90.0, 90.0, 30.0)
        self.assertIs(settings.validate(), settings)

    def test_settings_reject_beyond_preview_boundaries(self) -> None:
        with self.assertRaises(ValueError):
            PreviewSettings(100.1, 1.0, 100.0, True, 1.0, 30.0, 30.0, 10.0).validate()
        with self.assertRaises(ValueError):
            PreviewSettings(100.0, 1.01, 100.0, True, 1.0, 30.0, 30.0, 10.0).validate()

    def test_rate_limit_is_euclidean(self) -> None:
        result = rate_limit_vector((0.0, 0.0, 0.0), (30.0, 40.0, 0.0), 10.0, 1.0)
        self.assertAlmostEqual(sum(v * v for v in result) ** 0.5, 10.0)

    def test_source_has_no_robot_controller_or_motion_api(self) -> None:
        source = (PYTHON_ROOT / "src" / "vla_lab" / "engineering_teleop_preview.py").read_text(encoding="utf-8")
        for forbidden in (
            "JakaJogController", "jkrc", ".servo_p(", ".servo_j(",
            ".linear_move(", ".joint_move(", ".power_on(", ".enable_robot(",
        ):
            self.assertNotIn(forbidden, source)
        # 其他测试可能先导入控制器；这里只验证预览器自己的源码边界。
        self.assertNotIn("jaka_jog_controller", source)

    def test_real_tk_window_opens_without_hardware(self) -> None:
        root = tk.Tk(); root.withdraw()
        app = EngineeringTeleopPreview(root, quest_port=0)
        try:
            root.update_idletasks(); root.update()
            self.assertIn("零机器人命令", root.title())
            self.assertEqual(app.radius_cm.get(), 100.0)
            self.assertEqual(app.translation_scale.get(), 1.0)
        finally:
            app.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
