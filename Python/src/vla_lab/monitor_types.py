"""Small immutable values exchanged by the read-only monitor.

These monitor values deliberately do not contain any robot command.  Joint
angles are optional because the RGB stream must remain usable while the JAKA
SDK is reconnecting. The recorder always requires all six joints and can be
configured for RGB-only or RGB-D episodes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CameraFrame:
    """One RGB or RGB-D camera frame stamped immediately after retrieval."""

    timestamp_ns: int
    frame_id: int
    image_bgr: Any
    depth_mm: Any | None
    camera_name: str
    serial_number: int | None


@dataclass(frozen=True)
class RobotTelemetry:
    """One read-only JAKA controller snapshot in the base coordinate frame."""

    timestamp_ns: int
    tcp_pose_base_mm_deg: tuple[float, float, float, float, float, float]
    tool_id: int | None
    in_motion: bool
    joint_positions_rad: tuple[float, float, float, float, float, float] | None = None
    enabled: bool | None = None
    joint_error: str | None = None
    joint_timestamp_ns: int | None = None
    controller_reachable: bool = True
    gripper_opening_mm: float | None = None
    gripper_timestamp_ns: int | None = None
    gripper_in_motion: bool | None = None
    gripper_source: str | None = None
    gripper_error: str | None = None
    gripper_status_code: int | None = None
    gripper_fault_code: int | None = None
    gripper_position_units: int | None = None
    gripper_enable_status_code: int | None = None


@dataclass(frozen=True)
class MonitorSnapshot:
    """Latest independently sampled camera and robot values."""

    camera: CameraFrame | None = None
    robot: RobotTelemetry | None = None
    camera_error: str | None = None
    robot_error: str | None = None
    camera_fps: float = 0.0
    robot_poll_hz: float = 0.0
    global_camera: CameraFrame | None = None
    global_camera_error: str | None = None
    global_camera_fps: float = 0.0

    @property
    def software_time_delta_ms(self) -> float | None:
        """Absolute timestamp gap; this is not hardware synchronization."""

        if self.camera is None or self.robot is None:
            return None
        return abs(self.camera.timestamp_ns - self.robot.timestamp_ns) / 1_000_000.0

    @property
    def global_camera_delta_ms(self) -> float | None:
        """Absolute ZED/global-camera timestamp gap in milliseconds."""

        if self.camera is None or self.global_camera is None:
            return None
        return abs(
            self.camera.timestamp_ns - self.global_camera.timestamp_ns
        ) / 1_000_000.0
