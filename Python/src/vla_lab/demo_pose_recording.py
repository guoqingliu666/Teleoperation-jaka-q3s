"""DEMO 专用：真实/测试 VR 帧 + 模拟机器人反馈，不冒充硬件 episode。

采样频率是 GUI 消费 UDP 的实际频率，不是固定帧率；逐帧保存单调时间。
输入、命令、反馈明确分开，反馈是最近一次模拟器快照，并非同步实测。
"""
from dataclasses import asdict
from datetime import datetime
import json
import os
from pathlib import Path
import time
import uuid


class DemoPoseRecorder:
    def __init__(self, output_root: Path):
        self.output_root = Path(output_root)
        self.stream = None
        self.path = None
        self.frames = 0
        self.events = 0
        self.meta = {}

    @property
    def active(self):
        return self.stream is not None

    def start(self, instruction, settings):
        if self.active:
            raise RuntimeError("已有录制，请先保存或放弃")
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.path = self.output_root / (datetime.now().strftime("demo_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
        self.path.mkdir()
        self.frames = self.events = 0
        self.meta = {"schema": "quest_demo_pose_mapping_v1", "robot_source": "SIMULATED",
                     "vr_source": "received_udp_not_authenticated", "contains_images": False,
                     "contains_real_robot": False, "phase": "recording", "instruction": instruction,
                     "started_unix_ns": time.time_ns(), "settings_at_start": settings,
                     "sampling": "one row per GUI-consumed UDP frame; variable rate; no resampling",
                     "units": {"vr_position": "m", "tcp_xyz": "mm", "tcp_rpy": "rad"}}
        self._save_metadata()
        self.stream = (self.path / "frames.jsonl").open("x", encoding="utf-8")

    def _save_metadata(self):
        temporary = self.path / "metadata.json.tmp"
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(self.meta, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.path / "metadata.json")

    def append(self, frame, snapshot, target, armed, active):
        if not self.active:
            return
        row = {"row": self.frames, "consumed_monotonic_s": time.monotonic(),
               "vr": asdict(frame), "armed": bool(armed), "grip_servo_active": bool(active),
               "last_command_tcp_mm_rad": target,
               "latest_simulated_feedback_tcp_mm_rad": getattr(snapshot, "tcp_pose", None),
               "feedback_timestamp_ns": getattr(snapshot, "timestamp_ns", None)}
        self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        self.frames += 1
        if self.frames % 30 == 0:
            self.stream.flush()

    def mark(self, label):
        if self.active:
            with (self.path / "events.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"monotonic_s": time.monotonic(), "label": label}, ensure_ascii=False) + "\n")
            self.events += 1

    def finish(self, outcome):
        if not self.active:
            raise RuntimeError("当前没有录制")
        self.stream.flush()
        os.fsync(self.stream.fileno())
        self.stream.close()
        self.stream = None
        self.meta.update(phase="finished", outcome=outcome, frames=self.frames, events=self.events,
                         finished_unix_ns=time.time_ns())
        self._save_metadata()
        return self.path
