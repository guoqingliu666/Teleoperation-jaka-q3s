"""Configuration loading for the read-only observation monitor."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping


@dataclass(frozen=True)
class ZedMonitorConfig:
    serial_number: int | None = None
    resolution: str = "HD720"
    fps: int = 30
    retry_interval_s: float = 2.0
    depth_mode: str = "NEURAL"
    # RGB-only episodes neither retrieve nor write ZED depth.  Keep the
    # dataclass default compatible with older read-only RGB-D monitor users;
    # the jog GUI config explicitly selects false.
    capture_depth: bool = True


@dataclass(frozen=True)
class GlobalCameraConfig:
    enabled: bool = False
    device_name: str = "DroidCam Video"
    watermark_present: bool = False
    width: int = 640
    height: int = 480
    fps: int = 30
    retry_interval_s: float = 2.0


@dataclass(frozen=True)
class JakaMonitorConfig:
    host: str = "10.5.5.100"
    port: int = 10001
    timeout_s: float = 0.8
    poll_hz: float = 5.0
    tool_refresh_s: float = 10.0
    sdk_directory: str = "third_party/jaka_sdk"


@dataclass(frozen=True)
class MisumiGripperConfig:
    """Read-only USB/RS-485 settings for the MISUMI E-ELGPS(D) gripper."""

    enabled: bool = False
    port: str = "COM6"
    slave_id: int = 9
    baudrate: int = 115200
    timeout_s: float = 0.12
    read_function_code: int = 3
    fully_closed_position_units: int = 5000
    position_unit_mm: float = 0.01


@dataclass(frozen=True)
class RecorderConfig:
    output_directory: str = "datasets/observations"
    max_snapshot_gap_ms: float = 150.0
    max_global_camera_gap_ms: float = 100.0
    alignment_delay_ms: float = 120.0


@dataclass(frozen=True)
class DemonstrationConfig:
    output_directory: str = "datasets/episodes"
    sample_hz: float = 5.0
    default_duration_s: float = 6.0


@dataclass(frozen=True)
class WindowConfig:
    title: str = "JAKA S5 + ZED Mini 只读观测监视器"
    display_width: int = 960
    display_height: int = 540
    display_fps: float = 15.0
    refresh_ms: int = 50
    stale_after_s: float = 2.0


@dataclass(frozen=True)
class ReadOnlyMonitorConfig:
    zed: ZedMonitorConfig = ZedMonitorConfig()
    global_camera: GlobalCameraConfig = GlobalCameraConfig()
    jaka: JakaMonitorConfig = JakaMonitorConfig()
    misumi_gripper: MisumiGripperConfig = MisumiGripperConfig()
    monitor: WindowConfig = WindowConfig()
    recorder: RecorderConfig = RecorderConfig()
    demonstration: DemonstrationConfig = DemonstrationConfig()


def _section(payload: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = payload.get(name, {})
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a JSON object")
    return value


def load_monitor_config(path: str | Path) -> ReadOnlyMonitorConfig:
    """Load and validate JSON without silently accepting unsafe nonsense."""

    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("monitor config root must be a JSON object")
    zed = ZedMonitorConfig(**_section(payload, "zed"))
    global_camera = GlobalCameraConfig(**_section(payload, "global_camera"))
    jaka = JakaMonitorConfig(**_section(payload, "jaka"))
    misumi_gripper = MisumiGripperConfig(**_section(payload, "misumi_gripper"))
    window = WindowConfig(**_section(payload, "monitor"))
    recorder = RecorderConfig(**_section(payload, "recorder"))
    demonstration = DemonstrationConfig(**_section(payload, "demonstration"))
    if zed.fps <= 0 or jaka.poll_hz <= 0:
        raise ValueError("camera fps and robot poll_hz must be positive")
    if not isinstance(zed.capture_depth, bool):
        raise ValueError("zed.capture_depth must be boolean")
    if (
        not misumi_gripper.port.strip()
        or not 1 <= misumi_gripper.slave_id <= 247
        or misumi_gripper.baudrate <= 0
        or misumi_gripper.timeout_s <= 0
        or misumi_gripper.read_function_code not in (3, 4)
        or misumi_gripper.fully_closed_position_units <= 0
        or misumi_gripper.position_unit_mm <= 0
    ):
        raise ValueError("invalid MISUMI read-only gripper configuration")
    if (
        global_camera.width <= 0
        or global_camera.height <= 0
        or global_camera.fps <= 0
        or global_camera.retry_interval_s <= 0
    ):
        raise ValueError("global camera dimensions, fps and retry must be positive")
    if not global_camera.device_name.strip():
        raise ValueError("global camera device_name must not be empty")
    if window.refresh_ms < 10:
        raise ValueError("refresh_ms must be at least 10")
    if window.display_fps <= 0:
        raise ValueError("display_fps must be positive")
    if (
        recorder.max_snapshot_gap_ms <= 0
        or recorder.max_global_camera_gap_ms <= 0
        or recorder.alignment_delay_ms < 0
    ):
        raise ValueError("recorder synchronization thresholds must be positive")
    if demonstration.sample_hz <= 0 or demonstration.default_duration_s <= 0:
        raise ValueError("demonstration sample rate and duration must be positive")
    return ReadOnlyMonitorConfig(
        zed=zed,
        global_camera=global_camera,
        jaka=jaka,
        misumi_gripper=misumi_gripper,
        monitor=window,
        recorder=recorder,
        demonstration=demonstration,
    )


def default_monitor_config_path() -> Path:
    return Path(__file__).resolve().parents[2] / "config" / "readonly_monitor.json"
