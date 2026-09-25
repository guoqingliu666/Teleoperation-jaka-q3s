"""打开真实 Tk GUI，发送模拟 UDP，验证屏幕状态，再自动关闭。仅 --demo。"""
from pathlib import Path
import json
import socket
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab import jaka_jog_gui as gui


def run():
    root_path = Path(__file__).resolve().parents[2]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        test_port = probe.getsockname()[1]
    config = json.loads((Path(__file__).resolve().parents[1] / "config" / "jaka_jog.json").read_text(encoding="utf-8"))
    config["quest_vr"]["udp_port"] = test_port
    # 即使配置误开启夹爪，DEMO 仍不得创建真实夹爪控制器。
    config["gripper"]["enabled"] = True
    # 与旧 Unity/实际采集端口隔离，模拟数据绝不送入其他控制程序。
    config_path = root_path / "Validation" / "gui_test_config.json"
    config_path.write_text(json.dumps(config), encoding="utf-8")
    packet = json.loads((root_path / "Validation" / "unity_v2_fixture.json").read_text())
    original = gui.JogApplication
    failures = []

    class VerifiedApplication(original):
        def __init__(self, root, **kwargs):
            assert kwargs["demo"] is True
            super().__init__(root, **kwargs)
            root.attributes("-topmost", True)
            root.after(700, self.inject)
            root.after(1300, self.verify)

        def inject(self):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(json.dumps(packet).encode(), ("127.0.0.1", test_port))

        def verify(self):
            try:
                assert self.controller is not None and self.controller.demo, "demo controller missing"
                assert self._quest_latest_frame is not None and self._quest_latest_frame.valid, "frame missing: " + self._quest_status.get()
                assert "X=+0.300" in self._quest_detail.get(), "detail: " + self._quest_detail.get()
                assert self.demonstration is None, "unexpected recorder"
                assert self.gripper_controller is None, "unexpected gripper"
                assert self._hik_preview.process is None, "camera must not auto-connect"
                assert self._hik_preview.discover_button.winfo_ismapped(), "camera controls not visible"
                assert "超时" in self._quest_status.get(), "stopped sender should show timeout"
                from PIL import ImageGrab
                self.root.update_idletasks()
                ImageGrab.grab(bbox=(self.root.winfo_rootx(), self.root.winfo_rooty(),
                    self.root.winfo_rootx()+self.root.winfo_width(), self.root.winfo_rooty()+self.root.winfo_height())).save(
                    root_path / "Validation" / "gui_camera_panel.png")
                (root_path / "Validation" / "gui_validation.txt").write_text(
                    "PASS: actual Tk window opened in demo mode; Unity v2 synthetic UDP parsed and displayed.\n"
                    + self._quest_detail.get(), encoding="utf-8")
                print("GUI_DEMO_UDP_PASS", self._quest_detail.get())
            except Exception as exc:
                failures.append(repr(exc))
            finally:
                self._on_close()

    gui.JogApplication = VerifiedApplication
    gui.main(["--demo", "--config", str(config_path)])
    if failures:
        raise AssertionError(failures)


if __name__ == "__main__":
    run()
