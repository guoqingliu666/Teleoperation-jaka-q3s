"""单连接数字孪生的离线测试；不导入或连接厂商 SDK。"""
import json
import socket
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from vla_lab.sdk_owner_lease import SdkOwnerLease, reject_legacy_bridge, request_readonly_handoff
from vla_lab.single_owner_feedback import SingleOwnerFeedback
from vla_lab.vr_robot_visualization import RobotVrBroadcaster
from vla_lab import jaka_vr_readonly_bridge as bridge


class FakeRobot:
    def __init__(self):
        self.calls = []

    def get_robot_status_simple(self):
        self.calls.append("status")
        return 0, (0, 0, 1, 1)

    def get_tool_id(self):
        self.calls.append("tool")
        return 0, 1

    def get_actual_joint_position(self):
        self.calls.append("joints")
        return 0, [0.1] * 6

    def get_actual_tcp_position(self):
        self.calls.append("tcp")
        return 0, [400., 100., 500., 0., 0., 0.]


class Capture:
    def __init__(self):
        self.sent = []
        self.closed = False

    def publish(self, state, **kwargs):
        self.sent.append((state, kwargs))

    def close(self):
        self.closed = True


class SingleOwnerTests(unittest.TestCase):
    def test_feedback_uses_measured_not_target(self):
        times = iter([0., 0., .001, .004, .01, .01, .02, .03])
        capture = Capture()
        reader = SingleOwnerFeedback(FakeRobot(), broadcaster=capture,
                                     clock=lambda: next(times), wall_clock_ns=lambda: 123)
        self.assertTrue(reader.tick())
        state, options = capture.sent[0]
        self.assertEqual(state.feedback_source, "single_owner")
        self.assertEqual(state.sample_time_ns, 123)
        self.assertEqual(state.joints_rad, (.1,) * 6)
        self.assertIsNone(options["target_tcp_mm_rad"])
        reader.close()
        self.assertTrue(capture.closed)

    def test_second_sdk_owner_refused(self):
        # 操作系统分配空闲本机端口；不与在运行的项目争用 5011。
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        first = SdkOwnerLease(port=port)
        try:
            with self.assertRaisesRegex(RuntimeError, "拒绝双连接"):
                SdkOwnerLease(port=port)
        finally:
            first.close()
        second = SdkOwnerLease(port=port)
        second.close()

    def test_real_udp_packet_contains_measured_joints(self):
        # 真正经过本机 UDP 编解码，但机器人仍然是假对象。
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1)
            sender = RobotVrBroadcaster(port=receiver.getsockname()[1], max_hz=120)
            feedback = SingleOwnerFeedback(FakeRobot(), broadcaster=sender)
            try:
                self.assertTrue(feedback.tick())
                packet = json.loads(receiver.recvfrom(8192)[0].decode("utf-8"))
            finally:
                feedback.close()
        self.assertEqual(packet["feedback_source"], "single_owner")
        self.assertEqual(packet["joints_rad"], [0.1] * 6)
        self.assertEqual(packet["tcp_pose_mm_rad"][:3], [400., 100., 500.])
        self.assertIsNone(packet["target_tcp_mm_rad"])

    def test_old_bridge_udp_port_blocks_new_owner(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as old_bridge:
            old_bridge.bind(("127.0.0.1", 0))
            with self.assertRaisesRegex(RuntimeError, "旧①"):
                reject_legacy_bridge(port=old_bridge.getsockname()[1])

    def test_new_readonly_bridge_releases_owner_before_ack(self):
        # 仅用本机临时端口与假连接验证交接；不导入厂商SDK、不发送运动。
        with socket.socket() as free_port:
            free_port.bind(("127.0.0.1", 0))
            lease_port = free_port.getsockname()[1]
        old_owner = SdkOwnerLease(port=lease_port)
        server = bridge.ReadonlyHandoffServer(0)
        handoff_port = server._socket.getsockname()[1]
        errors = []

        def serve():
            try:
                while True:
                    requests = list(server.requests())
                    if requests:
                        address, nonce = requests[0]
                        old_owner.close()
                        server.released(address, nonce)
                        return
                    threading.Event().wait(.005)
            except Exception as error:
                errors.append(error)

        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        try:
            self.assertTrue(request_readonly_handoff(port=handoff_port, timeout_s=1))
            thread.join(timeout=1)
            self.assertFalse(errors)
            second = SdkOwnerLease(port=lease_port)
            second.close()
        finally:
            old_owner.close()
            server.close()

    def test_missing_handoff_server_is_not_permission(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as free_port:
            free_port.bind(("127.0.0.1", 0))
            port = free_port.getsockname()[1]
        self.assertFalse(request_readonly_handoff(port=port, timeout_s=.02))

    def test_vr_only_does_not_load_sdk(self):
        with patch.object(bridge, "run_vr_only", return_value=0) as launch, \
             patch.object(bridge, "run", side_effect=AssertionError("must not connect")):
            result = bridge.main(["--vr-only", "--player-exe", "D:/dummy.exe",
                                  "--player-log", "D:/dummy.log"])
        self.assertEqual(result, 0)
        launch.assert_called_once()


if __name__ == "__main__":
    unittest.main()
