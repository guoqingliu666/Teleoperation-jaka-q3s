"""只读验收测试：使用 Unity 导出的模拟数据，不导入或连接 JAKA SDK。"""
from pathlib import Path
import copy
import json
import socket
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab.quest_vr_input import QuestUdpReceiver, RelativeQuestTracker, quaternion
from vla_lab.demo_pose_recording import DemoPoseRecorder
import tempfile
from vla_lab.jaka_jog_gui import build_parser


class ReproTests(unittest.TestCase):
    def setUp(self):
        self.packet = json.loads((Path(__file__).resolve().parents[2] / "Validation" / "unity_v2_fixture.json").read_text())

    def test_safe_default(self):
        self.assertTrue(build_parser().parse_args([]).demo)
        self.assertTrue(build_parser().parse_args(["--demo"]).demo)

    def test_unity_schema(self):
        frame = QuestUdpReceiver._frame(self.packet)
        self.assertTrue(frame.valid and frame.rotation_valid and frame.head_rotation_valid)
        self.assertAlmostEqual(frame.position_m[0], .3, places=5)

    def test_nonfinite_input_rejected(self):
        for number in (float("nan"), float("inf"), -float("inf")):
            for field in ("grip", "trigger", "position_m"):
                packet = copy.deepcopy(self.packet)
                if field == "position_m":
                    packet["right"][field]["x"] = number
                else:
                    packet["right"][field] = number
                with self.assertRaises(ValueError):
                    QuestUdpReceiver._frame(packet)
            with self.assertRaises(ValueError):
                quaternion([number, 0, 0, 1])

    def test_zero_rotation_is_not_valid(self):
        self.packet["right"]["rotation_xyzw"] = [0, 0, 0, 0]
        self.assertFalse(QuestUdpReceiver._frame(self.packet).rotation_valid)

    def test_demo_episode_lifecycle(self):
        output = Path(__file__).resolve().parents[2] / "Validation"
        with tempfile.TemporaryDirectory(dir=output, prefix="recording_test_") as folder:
            recorder = DemoPoseRecorder(Path(folder))
            recorder.start("synthetic test", {"test": True})
            with self.assertRaises(RuntimeError):
                recorder.start("duplicate", {})
            recorder.append(QuestUdpReceiver._frame(self.packet), None, None, False, False)
            recorder.mark("test marker")
            path = recorder.finish("discarded")
            metadata = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["frames"], 1)
            self.assertEqual(metadata["outcome"], "discarded")
            self.assertEqual(metadata["robot_source"], "SIMULATED")
            self.assertTrue((path / "frames.jsonl").exists())
            self.assertFalse(recorder.active)
            with self.assertRaises(RuntimeError):
                recorder.finish("success")

    def test_udp_wire(self):
        # 自动选空闲测试端口，不占用正式 GUI 的 5005。
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        receiver = QuestUdpReceiver("127.0.0.1", port)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                result = None
                deadline = time.monotonic() + 2
                while result is None and time.monotonic() < deadline:
                    sender.sendto(json.dumps(self.packet).encode(), ("127.0.0.1", port))
                    time.sleep(.02)
                    result = receiver.latest()
                self.assertIsNotNone(result)
                self.assertTrue(result.valid)
                self.assertIsNone(receiver.error)
                self.assertIsNotNone(receiver.source_endpoint)
        finally:
            receiver.close()

    def test_udp_locks_valid_sender_and_ignores_second_player(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0)); port = probe.getsockname()[1]
        receiver = QuestUdpReceiver("127.0.0.1", port)
        invalid = copy.deepcopy(self.packet)
        invalid["sequence"] = 13000
        for side in ("left", "right"):
            invalid[side]["connected"] = invalid[side]["tracked"] = invalid[side]["pose_valid"] = False
        valid = copy.deepcopy(self.packet); valid["sequence"] = 9000
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bad, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as good:
                deadline = time.monotonic() + 2; result = None
                while time.monotonic() < deadline:
                    bad.sendto(json.dumps(invalid).encode(), ("127.0.0.1", port))
                    good.sendto(json.dumps(valid).encode(), ("127.0.0.1", port))
                    time.sleep(.02)
                    result = receiver.latest() or result
                    if result is not None and receiver.ignored_other_source_packets:
                        break
                self.assertIsNotNone(result)
                self.assertTrue(result.valid)
                self.assertEqual(result.raw_packet["sequence"], 9000)
                self.assertEqual(receiver.source_endpoint, f"127.0.0.1:{good.getsockname()[1]}")
                self.assertGreater(receiver.ignored_untracked_before_lock + receiver.ignored_other_source_packets, 0)
        finally:
            receiver.close()

    def test_udp_switches_away_from_an_untracked_old_player(self):
        """旧 Player 即使继续发无效包，也不能永久挡住新 Player 的有效追踪。"""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0)); port = probe.getsockname()[1]
        receiver = QuestUdpReceiver("127.0.0.1", port)
        first_valid = copy.deepcopy(self.packet); first_valid["sequence"] = 111
        first_invalid = copy.deepcopy(first_valid); first_invalid["sequence"] = 112
        first_invalid["right"]["tracked"] = first_invalid["right"]["pose_valid"] = False
        second_valid = copy.deepcopy(self.packet); second_valid["sequence"] = 222
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as old, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as new:
                old.sendto(json.dumps(first_valid).encode(), ("127.0.0.1", port))
                time.sleep(.05); receiver.latest()
                deadline = time.monotonic() + 2.2
                switched = None
                while time.monotonic() < deadline:
                    # 旧实例仍在发送，但已经丢失追踪；新实例追踪有效。
                    old.sendto(json.dumps(first_invalid).encode(), ("127.0.0.1", port))
                    new.sendto(json.dumps(second_valid).encode(), ("127.0.0.1", port))
                    time.sleep(.03)
                    frame = receiver.latest()
                    if frame is not None and frame.raw_packet["sequence"] == 222:
                        switched = frame
                        break
                self.assertIsNotNone(switched)
                self.assertEqual(receiver.source_endpoint, f"127.0.0.1:{new.getsockname()[1]}")
        finally:
            receiver.close()

    def test_mapping_and_tracking_loss(self):
        tracker = RelativeQuestTracker(
            {"forward":"Y+", "backward":"Y-", "left":"X-", "right":"X+", "up":"Z+", "down":"Z-"},
            grip_on=.7, grip_off=.5, trigger_on=.7, trigger_off=.5)
        self.packet["right"]["grip"] = 1
        tracker.update(QuestUdpReceiver._frame(self.packet), 1)
        moved = copy.deepcopy(self.packet)
        moved["right"]["position_m"]["x"] += .01
        events = tracker.update(QuestUdpReceiver._frame(moved), 1)
        delta = next(value for name, value in events if name == "pose_delta")
        self.assertAlmostEqual(delta[0][0], 10, places=5)
        moved["right"]["tracked"] = False
        self.assertIn(("grip_stop", None), tracker.update(QuestUdpReceiver._frame(moved), 1))


if __name__ == "__main__":
    unittest.main(verbosity=2)
