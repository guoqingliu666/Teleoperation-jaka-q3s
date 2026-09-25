"""合成数据验证真实观察包的对齐门、来源声明和界面默认安全状态。"""
from pathlib import Path
import json
import sys
import time
import tkinter as tk
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab import hardware_observation_gui as app


# 测试数据跟随仓库目录，克隆到其他 D: 路径也能运行。
VALIDATION_ROOT = Path(__file__).resolve().parents[2] / "Validation"
TEST_ROOT = VALIDATION_ROOT / "hardware_observation_synthetic"


class FakeCamera:
    def __init__(self):
        self.requests = []

    def request_full_frame(self, token, path):
        self.requests.append((token, Path(path)))

    def capture(self, stamp):
        token, path = self.requests[-1]
        path.write_bytes(b"SYNTHETIC_FULL_FRAME")
        return {"token": token, "path": str(path), "host_time_ns": stamp,
                "frame_id": len(self.requests), "width": 5120, "height": 5120,
                "camera": {"serial": "FAKE", "name": "synthetic"}}


def robot(timestamp_ns):
    return {
        "schema": "jaka.readonly.telemetry.v1", "host_time_unix_ns": timestamp_ns,
        "host_query_started_monotonic_ns": time.monotonic_ns(),
        "active_tool_id": 0, "joints_rad": [0.0] * 6, "tcp_mm_rad": [0.0] * 6,
        "status": {"powered_on": False, "enabled": False, "error_code": 0},
        "hardware_motion_commanded": False,
    }


def quest(timestamp_ns, valid=True):
    return SimpleNamespace(received_time_ns=timestamp_ns, received_s=time.monotonic(),
        connected=True, tracked=valid, valid=valid, udp_source="127.0.0.1:12345",
        raw_packet={"version": 2, "sequence": 42, "head": {}, "left": {}, "right": {}})


class RecordingTests(unittest.TestCase):
    def make_pending(self, name):
        path = TEST_ROOT / ("." + name + ".pending")
        path.mkdir(parents=True, exist_ok=False)
        (path / "global_rgb.jpg").write_bytes(b"SYNTHETIC_NOT_A_REAL_IMAGE")
        return name, path, {"schema": "quest_jaka_hik_observation.v1", "instruction": "synthetic", "safety": {"read_only": True}}

    def test_good_alignment_accepted_and_declared_not_training_episode(self):
        stamp = time.time_ns()
        pending = self.make_pending("good_" + str(stamp))
        capture = {"token": pending[0], "path": str(pending[1] / "global_rgb.jpg"), "host_time_ns": stamp,
                   "frame_id": 5, "width": 5120, "height": 5120, "camera": {"serial": "FAKE"}}
        with patch.object(app, "OUTPUT_ROOT", TEST_ROOT):
            safe_robot = robot(stamp + 50_000_000)
            safe_robot["status"] = {"powered_on": False, "enabled": False}
            path, payload = app.finish_capture(pending, capture, [safe_robot], [quest(stamp + 5_000_000)])
        self.assertTrue(path.name.startswith("accepted_"))
        self.assertTrue(payload["quality_gate_passed"])
        self.assertFalse(payload["limitations"]["usable_for_training"])
        self.assertFalse(payload["safety"].get("robot_command_sent", False))

    def test_bad_alignment_or_tracking_isolated(self):
        stamp = time.time_ns()
        pending = self.make_pending("bad_" + str(stamp))
        capture = {"token": pending[0], "path": str(pending[1] / "global_rgb.jpg"), "host_time_ns": stamp,
                   "frame_id": 6, "width": 5120, "height": 5120, "camera": {"serial": "FAKE"}}
        with patch.object(app, "OUTPUT_ROOT", TEST_ROOT):
            unsafe_robot = robot(stamp - 900_000_000)
            unsafe_robot["status"] = {"powered_on": True, "enabled": True}
            path, payload = app.finish_capture(pending, capture, [unsafe_robot], [quest(stamp, False)])
        self.assertTrue(path.name.startswith("rejected_"))
        self.assertFalse(payload["quality_gate_passed"])
        self.assertGreaterEqual(len(payload["rejection_reasons"]), 2)

    def test_unknown_token_and_outside_path_rejected(self):
        with self.assertRaises(ValueError):
            app.finish_capture(("a", TEST_ROOT / "none", {}), {"token": "b", "host_time_ns": 1}, [], [])
        from vla_lab.hik_camera_preview import _capture_path
        with self.assertRaises(ValueError):
            _capture_path(r"D:\ChatGPT\outside.jpg")

    def test_shadow_target_and_trace_are_translation_only_zero_command(self):
        reference = (428.0, 106.0, 535.0, -0.005, 0.020, 2.668)
        target = app.shadow_target_tcp(reference, (-10.0, 20.0, 30.0))
        self.assertEqual(target, (418.0, 126.0, 565.0, -0.005, 0.020, 2.668))
        frame = quest(time.time_ns())
        trace = app.ShadowTrace()
        shadow_root = TEST_ROOT / "shadow"
        with patch.object(app, "SHADOW_ROOT", shadow_root):
            trace.start({"test": True, "rotation_enabled": False})
            trace.append(frame, robot(time.time_ns()), (-10.0, 20.0, 30.0), target, True)
            path = trace.finish("synthetic_test")
        metadata = __import__("json").loads((path / "metadata.json").read_text(encoding="utf-8"))
        row = __import__("json").loads((path / "trace.jsonl").read_text(encoding="utf-8"))
        self.assertFalse(metadata["safety"]["robot_command_sent"])
        self.assertFalse(row["robot_command_sent"])
        self.assertEqual(row["shadow_target_tcp_mm_rad"], list(target))

    def test_hardware_observation_source_has_no_robot_motion_calls(self):
        source = Path(app.__file__).read_text(encoding="utf-8")
        for forbidden in ("power_on(", "enable_robot(", "joint_move(", "linear_move(", "servo_j(", "servo_p("):
            self.assertNotIn(forbidden, source)

    def test_bounded_continuous_observation_accepts_and_remains_non_training(self):
        camera = FakeCamera()
        episode = app.ContinuousObservationEpisode()
        episode_root = TEST_ROOT / "episodes"
        with patch.object(app, "OUTPUT_ROOT", episode_root):
            episode.start("synthetic continuous", camera, duration_s=5, max_frames=20)
            stamp = time.time_ns()
            self.assertIsNone(episode.handle_capture(camera.capture(stamp), [robot(stamp)], [quest(stamp)], camera))
            episode.deadline_s = 0.0
            final_path = episode.handle_capture(camera.capture(stamp + 10_000_000),
                [robot(stamp + 10_000_000)], [quest(stamp + 10_000_000)], camera)
        metadata = json.loads((final_path / "metadata.json").read_text(encoding="utf-8"))
        records = [json.loads(row) for row in (final_path / "frames.jsonl").read_text(encoding="utf-8").splitlines()]
        self.assertTrue(final_path.name.startswith("accepted_episode_"))
        self.assertEqual(metadata["accepted_frames"], 2)
        self.assertFalse(metadata["limitations"]["usable_for_training"])
        self.assertTrue(all(not row["robot_command_sent"] for row in records))


def check_window_defaults():
    from PIL import ImageGrab
    root = tk.Tk(); root.attributes("-topmost", True)
    with patch.object(app, "QuestUdpReceiver") as Receiver:
        Receiver.return_value.latest.return_value = None
        Receiver.return_value.close.return_value = None
        Receiver.return_value.error = None
        Receiver.return_value.ignored_other_source_packets = 0
        Receiver.return_value.ignored_untracked_before_lock = 0
        window = app.HardwareObservationWindow(root)
        try:
            root.update()
            assert window.backend.process is None
            assert window.camera.process is None
            assert not window.robot_connected
            assert str(window.capture_button["state"]) == "disabled"
            ImageGrab.grab(bbox=(root.winfo_rootx(), root.winfo_rooty(), root.winfo_rootx()+root.winfo_width(), root.winfo_rooty()+root.winfo_height())).save(
                VALIDATION_ROOT / "hardware_observation_gui_safe_default.png")
        finally:
            window.close()


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RecordingTests))
    if not result.wasSuccessful(): raise SystemExit(1)
    check_window_defaults()
    print("PASS: hardware observation alignment/provenance and safe GUI defaults; synthetic only.")
