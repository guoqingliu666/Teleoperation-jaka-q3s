"""真实 Tk + 原 DEMO 子进程的端到端测试。只向独立测试端口发送合成 UDP。"""
from pathlib import Path
import json
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab import jaka_jog_gui as gui
from vla_lab.demo_pose_recording import DemoPoseRecorder


def run():
    root_path = Path(__file__).resolve().parents[2]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    config = json.loads((Path(__file__).resolve().parents[1] / "config" / "jaka_jog.json").read_text(encoding="utf-8"))
    config["quest_vr"]["udp_port"] = port
    config["quest_vr"]["translation_scale"] = 1
    config["quest_vr"]["rotation_enabled"] = False
    config["quest_vr"]["packet_timeout_s"] = .3
    config["gripper"]["enabled"] = True  # DEMO 仍必须忽略此项
    config_path = root_path / "Validation" / "mapping_test_config.json"
    config_path.write_text(json.dumps(config), encoding="utf-8")
    packet = json.loads((root_path / "Validation" / "unity_v2_fixture.json").read_text())
    original = gui.JogApplication
    failures = []
    results = []

    class VerifiedApplication(original):
        def __init__(self, root, **kwargs):
            assert kwargs["demo"]
            super().__init__(root, **kwargs)
            self._demo_recorder = DemoPoseRecorder(root_path / "Validation" / "demo_sessions")
            self.transmit = True
            self.commands = []
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.start_time = time.monotonic()
            self.send()
            self.later(900, self.ready)

        def later(self, delay, function):
            def guarded():
                try:
                    function()
                except Exception as error:
                    import traceback
                    failures.append(traceback.format_exc())
                    self.finish_test()
            self.root.after(delay, guarded)

        def send(self):
            if self.transmit:
                self.sock.sendto(json.dumps(packet).encode(), ("127.0.0.1", port))
            self.root.after(20, self.send)

        def ready(self):
            snapshot = self._latest_robot_snapshot
            if snapshot is None or not snapshot.connected:
                assert time.monotonic() - self.start_time < 5, "demo startup timeout"
                self.later(200, self.ready)
                return
            assert self.controller.demo and self.gripper_controller is None
            old_send = self.controller._send
            def spy(command):
                self.commands.append(command)
                old_send(command)
            self.controller._send = spy
            self._quest_confirm.set(True)
            self._quest_arm()
            assert not self._quest_armed, "must lock heading before ARM"
            self._quest_lock_heading()
            assert self._quest_heading_locked
            self._quest_arm()
            assert self._quest_armed, self._quest_status.get()
            self.reference = tuple(snapshot.tcp_pose)
            self._record_instruction_var.set("SYNTHETIC END-TO-END TEST: forward 1cm, release, tracking loss, timeout")
            self._record_start()
            assert self._demo_recorder.active
            packet["right"]["grip"] = 1
            self.later(200, self.move)

        def move(self):
            assert self._quest_servo_active
            packet["right"]["position_m"]["z"] += .01
            self.later(400, self.check_move)

        def check_move(self):
            target = self._quest_last_sent_target
            # 现场校正配置：手向前(+Z) → 模拟机器人 X+。1 cm 对应 10 mm。
            assert abs(target[0] - self.reference[0] - 10) < .02, (target, self.reference)
            assert abs(self._latest_robot_snapshot.tcp_pose[0] - target[0]) < .02
            results.append("PASS: VR forward 1cm -> simulated X +10mm, feedback follows")
            self._record_mark("映射通过")
            packet["right"]["grip"] = 0
            self.later(150, self.check_release)

        def check_release(self):
            assert not self._quest_servo_active and self._quest_armed
            assert any(command[0] == "cartesian_servo_stop" for command in self.commands)
            results.append("PASS: Grip release stops servo, ARM retained")
            packet["right"]["grip"] = 1
            self.later(150, self.lose_tracking)

        def lose_tracking(self):
            assert self._quest_servo_active
            packet["right"]["tracked"] = False
            packet["right"]["pose_valid"] = False
            self.later(150, self.check_loss)

        def check_loss(self):
            assert not self._quest_servo_active and not self._quest_armed
            packet["right"]["tracked"] = packet["right"]["pose_valid"] = True
            self.later(150, self.check_recovery)

        def check_recovery(self):
            assert not self._quest_servo_active and not self._quest_armed
            self._quest_arm()
            assert not self._quest_armed, "held Grip must not rearm after reconnect"
            results.append("PASS: tracking loss disarms; recovery with held Grip does not restart")
            packet["right"]["grip"] = 0
            self.later(150, self.rearm)

        def rearm(self):
            self._quest_arm()
            assert self._quest_armed
            packet["right"]["grip"] = 1
            self.later(150, self.stop_udp)

        def stop_udp(self):
            assert self._quest_servo_active
            self.transmit = False
            self.later(550, self.check_timeout)

        def check_timeout(self):
            assert not self._quest_servo_active and not self._quest_armed
            assert "超时" in self._quest_status.get(), self._quest_status.get()
            self._quest_arm()
            assert not self._quest_armed
            results.append("PASS: UDP timeout stops/disarms; stale input cannot ARM")
            self._record_finish(True)
            path = self._demo_recorder.path
            meta = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
            rows = [json.loads(line) for line in (path / "frames.jsonl").read_text(encoding="utf-8").splitlines()]
            assert meta["robot_source"] == "SIMULATED" and not meta["contains_images"]
            assert meta["outcome"] == "success" and meta["frames"] == len(rows) and len(rows) > 10
            assert any(not row["vr"]["valid"] for row in rows)
            assert any(row["last_command_tcp_mm_rad"] is not None for row in rows)
            results.append(f"PASS: saved/read back {len(rows)} demo frames, provenance and invalid frames included")
            results.append("Episode: " + str(path))
            self.finish_test()

        def finish_test(self):
            self.transmit = False
            self.sock.close()
            self._on_close()

    gui.JogApplication = VerifiedApplication
    gui.main(["--demo", "--config", str(config_path)])
    report = "\n".join(results + failures)
    (root_path / "Validation" / "mapping_recording_validation.txt").write_text(report, encoding="utf-8")
    print(report)
    if failures:
        raise AssertionError("End-to-end verification failed")


if __name__ == "__main__":
    run()
