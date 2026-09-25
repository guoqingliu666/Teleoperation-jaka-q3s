"""无真机测试：精确 SDK 白名单、坏数据、子进程，以及真实 Tk 窗口。"""
import ast
import json
import math
from pathlib import Path
import sys
import time
import tkinter as tk
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab import jaka_telemetry as telemetry
from vla_lab.jaka_readonly_gui import ROOT, ReadOnlyWindow, display_sample, write_json


class FakeRobot:
    def __init__(self, host):
        self.calls = []
        self.tool_ids = [0, 0]
        self.frames = [0, 0]
        self.fail = None
        self.joints = (0.1, 1.0, -1.5, 0.2, 0.3, 0.4)
        self.status = (0, "", 0, 0)

    def __getattr__(self, name):
        if name not in telemetry.READ_OPERATIONS:
            raise AssertionError("Unexpected robot operation: " + name)
        def call(*args):
            self.calls.append(name)
            if self.fail == name:
                return (-3,)
            if name == "get_tool_id":
                return (0, self.tool_ids.pop(0))
            if name == "get_user_frame_id":
                return (0, self.frames.pop(0))
            if name == "get_tool_data":
                return (0, args[0], (0, 0, 0, 0, 0, 0))
            if name == "get_actual_joint_position":
                return (0, self.joints)
            if name in ("get_tcp_position", "get_actual_tcp_position"):
                return (0, (424.952, 106.558, 539.919, 0.0, 0.0, math.pi / 4))
            if name == "get_robot_status_simple":
                return (0, self.status)
            return (0,)
        return call


def fake_session():
    return telemetry.ReadOnlySession(types.SimpleNamespace(RC=FakeRobot), "192.168.1.10")


class SafetyTests(unittest.TestCase):
    def test_ip_requires_explicit_private_ipv4(self):
        for bad in ("", "<ROBOT_IP>", "6502", "localhost", "127.0.0.1", "8.8.8.8", "192.168.1.2:6502", "::1"):
            with self.assertRaises(ValueError):
                telemetry.validate_host(bad)
        self.assertEqual(telemetry.validate_host(" 192.168.110.11 "), "192.168.110.11")

    def test_api_whitelist_and_units(self):
        session = fake_session()
        session.connect()
        sample = session.sample()
        session.close()
        self.assertEqual(sample["tcp_mm_rad"][0], 424.952)
        self.assertAlmostEqual(sample["tcp_mm_rad"][5], math.pi / 4)
        self.assertIn("45.000", display_sample(sample))
        self.assertIn("法兰中心", display_sample(sample))
        self.assertIsNone(sample["hand_feedback"])
        self.assertFalse(sample["hardware_motion_commanded"])
        self.assertEqual(session.robot.calls[0], "login")
        self.assertEqual(session.robot.calls[-1], "logout")
        for name in ("power_on", "enable_robot", "set_tool_id", "servo_j", "set_digital_output", "motion_abort"):
            with self.assertRaises(PermissionError):
                session.call(name)
        self.assertTrue(set(session.robot.calls) <= telemetry.READ_OPERATIONS)

    def test_required_read_failure_never_zero_fills(self):
        for name in ("get_tcp_position", "get_tool_id", "get_actual_joint_position", "login"):
            session = fake_session()
            session.robot.fail = name
            with self.assertRaises(RuntimeError):
                session.connect() if name == "login" else session.sample()

    def test_bad_numbers_rejected(self):
        for values in ((1, 2, 3), (0, 0, 0, 0, 0, float("nan")), (0, 0, 0, 0, 0, float("inf")), (False, 0, 0, 0, 0, 0)):
            with self.assertRaises(ValueError):
                telemetry.six_numbers(values)

    def test_changing_coordinate_frame_rejected(self):
        for field in ("tool_ids", "frames"):
            session = fake_session()
            setattr(session.robot, field, [0, 1])
            with self.assertRaises(ValueError):
                session.sample()

    def test_unknown_optional_state_is_explicit(self):
        session = fake_session()
        session.robot.fail = "get_robot_status_simple"
        result = session.sample()
        self.assertIsNone(result["status"]["enabled"])
        self.assertTrue(result["warnings"])
        session = fake_session()
        session.robot.status = (0, "")
        self.assertIsNone(session.sample()["status"]["powered_on"])

    def test_query_age_stale_and_future_rejected(self):
        result = fake_session().sample()
        self.assertTrue(telemetry.fresh(result))
        result["host_query_started_monotonic_ns"] -= 3_000_000_000
        self.assertFalse(telemetry.fresh(result))
        result["host_query_started_monotonic_ns"] = time.monotonic_ns() + 1_000_000_000
        self.assertFalse(telemetry.fresh(result))

    def test_worker_failure_logs_out_without_reconnect(self):
        fake = FakeRobot("unused")
        fake.fail = "get_tcp_position"
        sdk = types.SimpleNamespace(RC=lambda host: fake)
        pipe = types.SimpleNamespace(messages=[], close=lambda: None)
        pipe.send = pipe.messages.append
        stop = types.SimpleNamespace(is_set=lambda: False, wait=lambda _: None)
        with patch.object(telemetry, "preflight", return_value={"fake": True}), patch.dict(sys.modules, {"jkrc": sdk}):
            telemetry.worker(pipe, stop, "read", "192.168.1.10", "fake")
        self.assertEqual(fake.calls.count("login"), 1)
        self.assertEqual(fake.calls[-1], "logout")
        self.assertIn("error", [p[0] for p in pipe.messages])
        self.assertNotIn("sample", [p[0] for p in pipe.messages])

    def test_preflight_does_not_construct_robot(self):
        sdk = types.SimpleNamespace(RC=lambda host: self.fail("RC must not be constructed"), __file__="fake/jkrc.pyd")
        with patch.object(telemetry, "load_sdk", return_value=sdk):
            result = telemetry.preflight("fake")
        self.assertFalse(result["robot_constructed"])
        self.assertFalse(result["robot_connected"])

    def test_child_timeout_exits_without_restart(self):
        backend = telemetry.TelemetryProcess()
        class Process:
            terminated = False
            def join(self, _): pass
            def is_alive(self): return True
            def terminate(self): self.terminated = True
            def close(self): pass
        process = Process()
        backend.process = process
        backend.pipe = types.SimpleNamespace(poll=lambda: False, close=lambda: None)
        backend.stop_event = types.SimpleNamespace(set=lambda: None)
        backend.last_message = time.monotonic() - 11
        self.assertEqual(backend.poll()[0][0], "error")
        self.assertTrue(process.terminated)
        self.assertIsNone(backend.process)
        self.assertEqual(backend.poll(), [])

    def test_source_has_no_controller_or_io_imports(self):
        tree = ast.parse(Path(telemetry.__file__).read_text(encoding="utf-8"))
        calls = [n.args[0].value for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "call" and n.args and isinstance(n.args[0], ast.Constant)]
        self.assertTrue(set(calls) <= telemetry.READ_OPERATIONS)
        # 全套测试共享 sys.modules，不能把“其他测试已导入”误判为只读模块导入。
        imported_names = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        }
        imported_names.update(
            node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
        )
        for name in ("vla_lab.jaka_jog_controller", "vla_lab.misumi_gripper_controller", "vla_lab.hik_camera_preview"):
            self.assertNotIn(name, imported_names)


def check_tk():
    from PIL import ImageGrab
    root = tk.Tk()
    window = ReadOnlyWindow(root)
    try:
        root.attributes("-topmost", True)
        root.update()
        assert window.backend.process is None  # 启动不会加载或连接 SDK。
        assert not window.host.get()
        assert str(window.save_button["state"]) == "disabled"
        sample = fake_session().sample()
        window.latest = sample
        window.active_read = True
        window.show("自动验收合成数据 / SYNTHETIC — 不是实际机器人反馈\n\n" + display_sample(sample))
        root.title("自动验收 / SYNTHETIC — 只读窗口显示检查，无机器人连接")
        root.after_cancel(window.timer)
        window.tick()
        root.update()
        assert str(window.save_button["state"]) == "normal"
        sample["host_query_started_monotonic_ns"] -= 3_000_000_000
        root.after_cancel(window.timer)
        window.tick()
        root.update()
        assert "过期" in window.status.get()
        assert str(window.save_button["state"]) == "disabled"
        path = ROOT / "Validation" / "jaka_readonly_synthetic.png"
        ImageGrab.grab(bbox=(root.winfo_rootx(), root.winfo_rooty(), root.winfo_rootx()+root.winfo_width(), root.winfo_rooty()+root.winfo_height())).save(path)
        window.disconnect()
        assert window.latest is None
        assert "已断开" in window.text.get("1.0", "end")
        write_json(ROOT / "Validation" / "jaka_readonly_gui_test.json", {"synthetic_only": True, "auto_connection": False, "fresh_save_enabled": True, "stale_save_blocked": True, "disconnect_clears_display": True})
    finally:
        window.close()


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SafetyTests))
    if not result.wasSuccessful():
        raise SystemExit(1)
    check_tk()
    write_json(ROOT / "Validation" / "jaka_readonly_test_results.json", {"tests_run": result.testsRun, "passed": result.wasSuccessful(), "robot_connected": False, "tk_synthetic_display_checked": True})
    print("PASS: read-only SDK safety tests and real Tk synthetic display. No hardware connection.")
