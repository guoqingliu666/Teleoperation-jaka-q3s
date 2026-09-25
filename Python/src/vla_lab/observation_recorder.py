"""Persist one complete, inspectable VLA observation from a monitor snapshot."""

from __future__ import annotations

import json
import shutil
import time
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

from .contracts import Observation, RobotState
from .monitor_types import MonitorSnapshot


class IncompleteObservationError(RuntimeError):
    """Raised when a snapshot is unsuitable for a training record."""


class ObservationRecorder:
    """Write RGB, metric depth and robot state without controlling hardware."""

    def __init__(self, output_directory: str | Path, max_gap_ms: float) -> None:
        self.output_directory = Path(output_directory).resolve()
        self.max_gap_ms = float(max_gap_ms)

    def record(self, snapshot: MonitorSnapshot, instruction: str) -> Path:
        instruction = instruction.strip()
        if not instruction:
            raise IncompleteObservationError("请先输入本条样本对应的自然语言指令")
        camera, robot = snapshot.camera, snapshot.robot
        global_camera = snapshot.global_camera
        if camera is None:
            raise IncompleteObservationError("尚未收到ZED画面")
        if robot is None:
            raise IncompleteObservationError("尚未收到JAKA状态")
        if camera.depth_mm is None:
            raise IncompleteObservationError("当前ZED帧不含深度；请启用深度模式")
        if robot.joint_positions_rad is None:
            raise IncompleteObservationError("尚未读到J1-J6关节角")
        if robot.enabled is None:
            raise IncompleteObservationError("尚未读到机器人使能状态")
        gap_ms = snapshot.software_time_delta_ms
        if gap_ms is None or gap_ms > self.max_gap_ms:
            raise IncompleteObservationError(
                f"相机与机器人快照相差{gap_ms or 0:.1f} ms，"
                f"超过{self.max_gap_ms:.1f} ms"
            )

        sample_id = time.strftime("sample_%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
        final_dir = self.output_directory / sample_id
        temp_dir = self.output_directory / f".{sample_id}.tmp"
        temp_dir.mkdir(parents=True, exist_ok=False)
        try:
            rgb_path = temp_dir / "rgb.png"
            depth_path = temp_dir / "depth_mm.npy"
            global_rgb_path = temp_dir / "global_rgb.png"
            if not cv2.imwrite(str(rgb_path), camera.image_bgr):
                raise RuntimeError("OpenCV保存RGB图像失败")
            if global_camera is not None and not cv2.imwrite(
                str(global_rgb_path), global_camera.image_bgr
            ):
                raise RuntimeError("OpenCV failed to save global RGB image")
            with depth_path.open("wb") as file:
                np.save(file, np.asarray(camera.depth_mm, dtype=np.float32))

            observation = Observation(
                timestamp_ns=camera.timestamp_ns,
                instruction=instruction,
                rgb_path="rgb.png",
                depth_path="depth_mm.npy",
                robot=RobotState(
                    joint_positions_rad=robot.joint_positions_rad,
                    tcp_pose_base_mm_deg=robot.tcp_pose_base_mm_deg,
                    enabled=robot.enabled,
                    in_motion=robot.in_motion,
                    gripper_opening_mm=robot.gripper_opening_mm,
                ),
                camera_name=camera.camera_name,
                frame_id=camera.frame_id,
                global_rgb_path=(
                    "global_rgb.png" if global_camera is not None else None
                ),
                global_camera_name=(
                    global_camera.camera_name if global_camera is not None else None
                ),
                global_camera_timestamp_ns=(
                    global_camera.timestamp_ns if global_camera is not None else None
                ),
                global_frame_id=(
                    global_camera.frame_id if global_camera is not None else None
                ),
            )
            depth = np.asarray(camera.depth_mm, dtype=np.float32)
            finite = np.isfinite(depth) & (depth > 0)
            payload = {
                "schema_version": "vla_lab.observation.v1",
                "observation": asdict(observation),
                "capture": {
                    "camera_serial_number": camera.serial_number,
                    "robot_timestamp_ns": robot.timestamp_ns,
                    "joint_timestamp_ns": robot.joint_timestamp_ns,
                    "software_time_delta_ms": gap_ms,
                    "global_camera_delta_ms": snapshot.global_camera_delta_ms,
                    "global_camera_serial_number": (
                        global_camera.serial_number if global_camera is not None else None
                    ),
                    "gripper_timestamp_ns": robot.gripper_timestamp_ns,
                    "gripper_source": robot.gripper_source,
                    "gripper_error": robot.gripper_error,
                    "depth_unit": "millimetres",
                    "depth_shape": list(depth.shape),
                    "valid_depth_fraction": float(finite.mean()),
                },
                "safety": {
                    "read_only": True,
                    "robot_command_sent": False,
                    "gripper_command_sent": False,
                },
            }
            (temp_dir / "observation.json").write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temp_dir.replace(final_dir)
            return final_dir
        except Exception:
            shutil.rmtree(temp_dir, ignore_errors=True)
            raise
