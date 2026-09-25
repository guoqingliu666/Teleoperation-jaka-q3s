"""Record a read-only human demonstration as Observation -> Action steps."""

from __future__ import annotations

import json
import math
import shutil
import statistics
import time
import uuid
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

from .contracts import (
    Action,
    ActionSource,
    Episode,
    EpisodeStep,
    GripperCommand,
    Observation,
    RobotState,
)
from .monitor_types import MonitorSnapshot
from .observation_recorder import IncompleteObservationError
from .storage import load_episode, save_episode


def _shortest_angle_delta_deg(previous: float, current: float) -> float:
    """Return a wrapped orientation delta in [-180, 180)."""

    return (current - previous + 180.0) % 360.0 - 180.0


class DemonstrationEpisodeRecorder:
    """Build one portable episode without exposing any hardware command API.

    A step is completed only when the following valid snapshot arrives.  The
    action is the measured TCP change between the two snapshots, never a
    command sent to JAKA.
    """

    def __init__(
        self,
        output_directory: str | Path,
        max_gap_ms: float,
        *,
        require_gripper: bool = False,
        require_global_camera: bool = False,
        max_global_camera_gap_ms: float | None = None,
        alignment_delay_ms: float = 0.0,
        global_camera_watermark_present: bool = False,
        gripper_change_threshold_mm: float = 0.5,
        human_control_commands: bool = False,
        sample_hz: float | None = None,
        record_depth: bool = True,
    ) -> None:
        self.output_directory = Path(output_directory).resolve()
        self.max_gap_ms = float(max_gap_ms)
        self.require_gripper = bool(require_gripper)
        self.require_global_camera = bool(require_global_camera)
        self.max_global_camera_gap_ms = (
            float(max_global_camera_gap_ms)
            if max_global_camera_gap_ms is not None
            else None
        )
        self.alignment_delay_ms = float(alignment_delay_ms)
        self.global_camera_watermark_present = bool(
            global_camera_watermark_present
        )
        self.gripper_change_threshold_mm = float(gripper_change_threshold_mm)
        self.human_control_commands = bool(human_control_commands)
        self.sample_hz = float(sample_hz) if sample_hz is not None else None
        self.record_depth = bool(record_depth)
        if self.gripper_change_threshold_mm < 0:
            raise ValueError("gripper_change_threshold_mm must be non-negative")
        if (
            self.max_global_camera_gap_ms is not None
            and self.max_global_camera_gap_ms <= 0
        ):
            raise ValueError("max_global_camera_gap_ms must be positive")
        if self.alignment_delay_ms < 0:
            raise ValueError("alignment_delay_ms must be non-negative")
        if self.sample_hz is not None and not 5.0 <= self.sample_hz <= 50.0:
            raise ValueError("sample_hz must be within 5--50 Hz")
        self._episode_id: str | None = None
        self._instruction: str | None = None
        self._temp_dir: Path | None = None
        self._final_dir: Path | None = None
        self._pending: MonitorSnapshot | None = None
        self._steps: list[EpisodeStep] = []
        self._skipped_samples = 0
        self._accepted_robot_gaps_ms: list[float] = []
        self._accepted_global_gaps_ms: list[float] = []
        self._stage_markers: list[dict[str, object]] = []

    @property
    def active(self) -> bool:
        return self._temp_dir is not None

    @property
    def step_count(self) -> int:
        return len(self._steps)

    @property
    def skipped_samples(self) -> int:
        return self._skipped_samples

    def start(self, instruction: str) -> str:
        if self.active:
            raise RuntimeError("a demonstration episode is already active")
        instruction = instruction.strip()
        if not instruction:
            raise IncompleteObservationError("示教任务指令不能为空")
        stamp = time.strftime("episode_%Y%m%d_%H%M%S")
        episode_id = f"{stamp}_{uuid.uuid4().hex[:8]}"
        final_dir = self.output_directory / episode_id
        temp_dir = self.output_directory / f".{episode_id}.tmp"
        temp_dir.mkdir(parents=True, exist_ok=False)
        self._episode_id = episode_id
        self._instruction = instruction
        self._temp_dir = temp_dir
        self._final_dir = final_dir
        self._pending = None
        self._steps = []
        self._skipped_samples = 0
        self._accepted_robot_gaps_ms = []
        self._accepted_global_gaps_ms = []
        self._stage_markers = []
        return episode_id

    def mark_stage(self, label: str) -> None:
        """Attach a human review marker without changing any ACT action."""

        if not self.active:
            raise RuntimeError("no demonstration episode is active")
        label = label.strip()
        if not label:
            raise ValueError("stage label must not be empty")
        self._stage_markers.append(
            {
                "label": label,
                "timestamp_ns": time.time_ns(),
                "completed_step_count": self.step_count,
            }
        )

    def append(self, snapshot: MonitorSnapshot) -> int:
        if not self.active:
            raise RuntimeError("start the demonstration before appending samples")
        self._validate_snapshot(snapshot)
        self._note_accepted_sync(snapshot)
        if self._pending is None:
            self._pending = snapshot
            return self.step_count

        previous = self._pending
        if snapshot.camera.timestamp_ns <= previous.camera.timestamp_ns:
            raise IncompleteObservationError("相机帧时间戳没有递增")

        index = len(self._steps)
        observation = self._write_observation(previous, index)
        duration_ms = max(
            1,
            round((snapshot.camera.timestamp_ns - previous.camera.timestamp_ns) / 1e6),
        )
        current_pose = snapshot.robot.tcp_pose_base_mm_deg
        previous_pose = previous.robot.tcp_pose_base_mm_deg
        previous_opening = previous.robot.gripper_opening_mm
        current_opening = snapshot.robot.gripper_opening_mm
        gripper_command = GripperCommand.KEEP
        if previous_opening is not None and current_opening is not None:
            opening_delta = current_opening - previous_opening
            if opening_delta > self.gripper_change_threshold_mm:
                gripper_command = GripperCommand.OPEN
            elif opening_delta < -self.gripper_change_threshold_mm:
                gripper_command = GripperCommand.CLOSE
        action = Action(
            delta_xyz_mm=tuple(
                current_pose[index] - previous_pose[index] for index in range(3)
            ),
            delta_rpy_deg=tuple(
                _shortest_angle_delta_deg(previous_pose[index], current_pose[index])
                for index in range(3, 6)
            ),
            gripper=gripper_command,
            duration_ms=duration_ms,
            source=ActionSource.HUMAN,
        )
        self._steps.append(EpisodeStep(observation=observation, action=action))
        self._pending = snapshot
        return self.step_count

    def finish(self, *, success: bool | None = None) -> Path:
        if not self.active:
            raise RuntimeError("no demonstration episode is active")
        if not self._steps:
            self.cancel()
            raise IncompleteObservationError(
                "示教至少需要两个有效快照，当前没有完整 Observation→Action 步骤"
            )
        assert self._temp_dir is not None
        assert self._final_dir is not None
        assert self._episode_id is not None
        assert self._instruction is not None
        assert self._pending is not None
        terminal_observation = self._write_observation(
            self._pending, len(self._steps)
        )
        terminal_path = self._temp_dir / "terminal_observation.json"
        terminal_path.write_text(
            json.dumps(asdict(terminal_observation), ensure_ascii=False, indent=2)
            + "\n",
            encoding="utf-8",
        )
        sampling_quality = self._sampling_quality()
        quality_issues = self._sampling_quality_issues(sampling_quality)
        if quality_issues:
            raise IncompleteObservationError(
                "Episode 采样质量不合格：" + "; ".join(quality_issues)
            )
        episode = Episode(
            episode_id=self._episode_id,
            instruction=self._instruction,
            steps=tuple(self._steps),
            success=success,
            metadata={
                "capture_mode": "read_only_human_demonstration",
                "action_semantics": "measured_tcp_delta_to_next_observation",
                "gripper_semantics": "measured_opening_to_next_observation",
                "gripper_required": self.require_gripper,
                "depth_recorded": self.record_depth,
                "global_camera_required": self.require_global_camera,
                "global_camera_sync_enforced": self._global_sync_enforced,
                "global_camera_capture": "nearest_timestamp_frame",
                "global_camera_watermark_present": (
                    self.global_camera_watermark_present
                ),
                "synchronization": {
                    "method": "buffered_nearest_host_timestamp_to_zed",
                    "hardware_triggered": False,
                    "reference_delay_ms": self.alignment_delay_ms,
                    "max_zed_jaka_gap_ms": self.max_gap_ms,
                    "max_zed_global_gap_ms": self.max_global_camera_gap_ms,
                    "accepted_snapshot_count": len(self._accepted_robot_gaps_ms),
                    "accepted_zed_jaka_gap_ms": self._gap_summary(
                        self._accepted_robot_gaps_ms
                    ),
                    "accepted_zed_global_gap_ms": self._gap_summary(
                        self._accepted_global_gaps_ms
                    ),
                },
                "terminal_observation_path": terminal_path.name,
                "skipped_samples": self._skipped_samples,
                "sampling_quality": sampling_quality,
                "stage_markers": self._stage_markers,
                "safety": {
                    # These fields describe the recorder itself.  The GUI may
                    # be used by a human to demonstrate motion, but the
                    # recorder never emits a robot/COM command.
                    "read_only": True,
                    "robot_command_sent": False,
                    "gripper_command_sent": False,
                    "recorder_sent_no_commands": True,
                    "operator_control_surface": (
                        "jaka_jog_gui" if self.human_control_commands else None
                    ),
                    "operator_robot_command_sent": self.human_control_commands,
                    "operator_gripper_command_sent": self.human_control_commands,
                },
            },
        )
        manifest = save_episode(episode, self._temp_dir / "episode.json")
        # Round-trip before publishing the directory so corrupt/incomplete
        # episodes never become visible as finished training data.
        if load_episode(manifest) != episode:
            raise RuntimeError("episode round-trip validation failed")
        self._temp_dir.replace(self._final_dir)
        result = self._final_dir
        self._reset()
        return result

    def _sampling_quality(self) -> dict[str, float | int | None]:
        """Summarise fixed-rate sampling so later training audits can inspect it."""

        durations = [step.action.duration_ms for step in self._steps]
        return {
            "requested_hz": self.sample_hz,
            "nominal_period_ms": (1000.0 / self.sample_hz) if self.sample_hz else None,
            "step_count": len(self._steps),
            "skipped_samples": self._skipped_samples,
            "median_period_ms": float(statistics.median(durations)) if durations else None,
            "max_period_ms": float(max(durations)) if durations else None,
        }

    def _sampling_quality_issues(
        self, quality: dict[str, float | int | None]
    ) -> list[str]:
        """Reject short, stalled or heavily skipped Episodes before publishing."""

        if self.sample_hz is None:
            return []
        step_count = int(quality["step_count"] or 0)
        minimum_steps = max(10, math.ceil(self.sample_hz))
        issues: list[str] = []
        if step_count < minimum_steps:
            issues.append(f"有效 steps 仅 {step_count}，至少需要 {minimum_steps}")
        skipped = int(quality["skipped_samples"] or 0)
        attempted = step_count + skipped
        if attempted and skipped / attempted > 0.15:
            issues.append(f"跳帧 {skipped}/{attempted} 超过 15%")
        nominal = float(quality["nominal_period_ms"] or 0.0)
        median = float(quality["median_period_ms"] or 0.0)
        maximum = float(quality["max_period_ms"] or 0.0)
        if nominal > 0 and (median > nominal * 1.8 or maximum > nominal * 3.0):
            issues.append(
                f"采样间隔异常（中位 {median:.0f} ms，最大 {maximum:.0f} ms，目标 {nominal:.0f} ms）"
            )
        return issues

    def cancel(self) -> None:
        if self._temp_dir is not None:
            shutil.rmtree(self._temp_dir, ignore_errors=True)
        self._reset()

    def note_skipped_sample(self) -> None:
        self._skipped_samples += 1

    def _reset(self) -> None:
        self._episode_id = None
        self._instruction = None
        self._temp_dir = None
        self._final_dir = None
        self._pending = None
        self._steps = []
        self._skipped_samples = 0
        self._accepted_robot_gaps_ms = []
        self._accepted_global_gaps_ms = []
        self._stage_markers = []

    @property
    def _global_sync_enforced(self) -> bool:
        return bool(
            self.require_global_camera
            and self.max_global_camera_gap_ms is not None
        )

    @staticmethod
    def _gap_summary(gaps_ms: list[float]) -> dict[str, float | None]:
        if not gaps_ms:
            return {"mean": None, "max": None}
        return {
            "mean": round(sum(gaps_ms) / len(gaps_ms), 3),
            "max": round(max(gaps_ms), 3),
        }

    def _note_accepted_sync(self, snapshot: MonitorSnapshot) -> None:
        robot_gap = snapshot.software_time_delta_ms
        if robot_gap is not None:
            self._accepted_robot_gaps_ms.append(float(robot_gap))
        global_gap = snapshot.global_camera_delta_ms
        if global_gap is not None:
            self._accepted_global_gaps_ms.append(float(global_gap))

    def _validate_snapshot(self, snapshot: MonitorSnapshot) -> None:
        camera, robot = snapshot.camera, snapshot.robot
        if camera is None or robot is None:
            raise IncompleteObservationError("尚未收到完整的 ZED 与 JAKA 快照")
        if self.record_depth and camera.depth_mm is None:
            raise IncompleteObservationError("当前 ZED 帧没有深度")
        if self.require_global_camera and snapshot.global_camera is None:
            raise IncompleteObservationError(
                "current snapshot is missing the global RGB camera frame"
            )
        global_gap_ms = snapshot.global_camera_delta_ms
        if self._global_sync_enforced and (
            global_gap_ms is None
            or global_gap_ms > self.max_global_camera_gap_ms
        ):
            raise IncompleteObservationError(
                "ZED and global camera snapshots differ by "
                f"{global_gap_ms or 0:.1f} ms, exceeding "
                f"{self.max_global_camera_gap_ms:.1f} ms"
            )
        if snapshot.global_camera is not None:
            global_image = np.asarray(snapshot.global_camera.image_bgr)
            if (
                global_image.ndim != 3
                or global_image.shape[2] != 3
                or global_image.size == 0
            ):
                raise IncompleteObservationError(
                    "global RGB camera frame must be a non-empty HxWx3 image"
                )
        if robot.joint_positions_rad is None or robot.enabled is None:
            raise IncompleteObservationError("当前 JAKA 快照缺少实时 J1-J6 或使能状态")
        gap_ms = snapshot.software_time_delta_ms
        if gap_ms is None or gap_ms > self.max_gap_ms:
            raise IncompleteObservationError(
                f"相机与机器人快照相差 {gap_ms or 0:.1f} ms，"
                f"超过 {self.max_gap_ms:.1f} ms"
            )
        for name, values in (
            ("J1-J6", robot.joint_positions_rad),
            ("TCP", robot.tcp_pose_base_mm_deg),
        ):
            if not all(math.isfinite(float(value)) for value in values):
                raise IncompleteObservationError(f"{name} 包含非有限数值")
        if self.require_gripper and robot.gripper_opening_mm is None:
            raise IncompleteObservationError("当前快照缺少夹爪开度")
        if robot.gripper_opening_mm is not None and (
            not math.isfinite(float(robot.gripper_opening_mm))
            or robot.gripper_opening_mm < 0
        ):
            raise IncompleteObservationError("夹爪开度包含无效数值")

    def _write_observation(self, snapshot: MonitorSnapshot, index: int) -> Observation:
        assert self._temp_dir is not None
        assert self._instruction is not None
        camera, robot = snapshot.camera, snapshot.robot
        frame_dir = self._temp_dir / "frames" / f"step_{index:06d}"
        frame_dir.mkdir(parents=True, exist_ok=False)
        rgb_path = frame_dir / "rgb.png"
        depth_path = frame_dir / "depth_mm.npy" if self.record_depth else None
        global_rgb_path = (
            frame_dir / "global_rgb.png"
            if snapshot.global_camera is not None
            else None
        )
        if not cv2.imwrite(str(rgb_path), camera.image_bgr):
            raise RuntimeError("OpenCV 保存示教 RGB 图像失败")
        if depth_path is not None:
            with depth_path.open("wb") as handle:
                np.save(handle, np.asarray(camera.depth_mm, dtype=np.float32))
        if global_rgb_path is not None and not cv2.imwrite(
            str(global_rgb_path), snapshot.global_camera.image_bgr
        ):
            raise RuntimeError("OpenCV failed to save the global RGB image")
        capture = {
            "camera_timestamp_ns": camera.timestamp_ns,
            "depth_recorded": self.record_depth,
            "robot_timestamp_ns": robot.timestamp_ns,
            "joint_timestamp_ns": robot.joint_timestamp_ns,
            "software_time_delta_ms": snapshot.software_time_delta_ms,
            "camera_serial_number": camera.serial_number,
            "global_camera_timestamp_ns": (
                snapshot.global_camera.timestamp_ns
                if snapshot.global_camera is not None
                else None
            ),
            "global_camera_frame_id": (
                snapshot.global_camera.frame_id
                if snapshot.global_camera is not None
                else None
            ),
            "global_camera_name": (
                snapshot.global_camera.camera_name
                if snapshot.global_camera is not None
                else None
            ),
            "global_camera_delta_ms": snapshot.global_camera_delta_ms,
            "global_camera_sync_enforced": self._global_sync_enforced,
            "global_camera_capture": "nearest_timestamp_frame",
            "gripper_timestamp_ns": robot.gripper_timestamp_ns,
            "gripper_source": robot.gripper_source,
            "gripper_error": robot.gripper_error,
            "gripper_status_code": robot.gripper_status_code,
            "gripper_fault_code": robot.gripper_fault_code,
            "gripper_position_units": robot.gripper_position_units,
            "gripper_enable_status_code": robot.gripper_enable_status_code,
        }
        (frame_dir / "capture.json").write_text(
            json.dumps(capture, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return Observation(
            timestamp_ns=camera.timestamp_ns,
            instruction=self._instruction,
            rgb_path=rgb_path.relative_to(self._temp_dir).as_posix(),
            depth_path=(
                depth_path.relative_to(self._temp_dir).as_posix()
                if depth_path is not None
                else None
            ),
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
                global_rgb_path.relative_to(self._temp_dir).as_posix()
                if global_rgb_path is not None
                else None
            ),
            global_camera_name=(
                snapshot.global_camera.camera_name
                if snapshot.global_camera is not None
                else None
            ),
            global_camera_timestamp_ns=(
                snapshot.global_camera.timestamp_ns
                if snapshot.global_camera is not None
                else None
            ),
            global_frame_id=(
                snapshot.global_camera.frame_id
                if snapshot.global_camera is not None
                else None
            ),
        )
