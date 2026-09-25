"""The smallest stable data contract shared by VLA components.

The future observation monitor, demonstration recorder, ACT policy and
SmolVLA adapter must communicate through these structures. Keeping the
contract independent from the JAKA and ZED SDKs makes recorded data replayable
even when no hardware is connected.

Units are deliberately explicit:

* robot joints: radians
* TCP position: millimetres in the JAKA base frame
* TCP orientation: degrees using JAKA RX/RY/RZ convention
* action translation: millimetres relative to the current TCP
* action rotation: degrees relative to the current TCP
* time: Unix nanoseconds
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = "vla_lab.episode.v1"


def _fixed_floats( values: Sequence[float],  *, length: int, name: str,) -> tuple[float, ...]:
    """Convert a numeric sequence and reject malformed robot vectors early."""

    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    return result


class GripperCommand(str, Enum):
    """Logical gripper request; no serial protocol is exposed to a policy."""

    KEEP = "keep"
    OPEN = "open"
    CLOSE = "close"


class ActionSource(str, Enum):
    """Who proposed an action, retained for later evaluation and auditing."""

    HUMAN = "human"
    RULE = "rule"
    ACT = "act"
    SMOLVLA = "smolvla"


@dataclass(frozen=True)
class RobotState:
    """One synchronised read-only snapshot of JAKA and the gripper."""

    joint_positions_rad: tuple[float, ...]
    tcp_pose_base_mm_deg: tuple[float, ...]
    enabled: bool
    in_motion: bool
    gripper_opening_mm: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "joint_positions_rad",
            _fixed_floats(
                self.joint_positions_rad,
                length=6,
                name="joint_positions_rad",
            ),
        )
        object.__setattr__(
            self,
            "tcp_pose_base_mm_deg",
            _fixed_floats(
                self.tcp_pose_base_mm_deg,
                length=6,
                name="tcp_pose_base_mm_deg",
            ),
        )
        if self.gripper_opening_mm is not None and self.gripper_opening_mm < 0:
            raise ValueError("gripper_opening_mm must be >= 0")

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "RobotState":
        return cls(
            joint_positions_rad=payload["joint_positions_rad"],
            tcp_pose_base_mm_deg=payload["tcp_pose_base_mm_deg"],
            enabled=bool(payload["enabled"]),
            in_motion=bool(payload["in_motion"]),
            gripper_opening_mm=(
                float(payload["gripper_opening_mm"])
                if payload.get("gripper_opening_mm") is not None
                else None
            ),
        )


@dataclass(frozen=True)
class Observation:
    """What the robot knew at one instant before choosing an action.

    Image arrays are stored as files rather than embedded in JSON. Paths are
    relative to an episode directory so a dataset can be moved to another PC.
    """

    timestamp_ns: int
    instruction: str
    rgb_path: str
    depth_path: str | None
    robot: RobotState
    camera_name: str = "zed_mini_left"
    frame_id: int = 0
    global_rgb_path: str | None = None
    global_camera_name: str | None = None
    global_camera_timestamp_ns: int | None = None
    global_frame_id: int | None = None

    def __post_init__(self) -> None:
        if self.timestamp_ns <= 0:
            raise ValueError("timestamp_ns must be positive")
        if not self.instruction.strip():
            raise ValueError("instruction must not be empty")
        if not self.rgb_path.strip():
            raise ValueError("rgb_path must not be empty")
        if self.frame_id < 0:
            raise ValueError("frame_id must be >= 0")
        global_values = (
            self.global_camera_name,
            self.global_camera_timestamp_ns,
            self.global_frame_id,
        )
        if self.global_rgb_path is None:
            if any(value is not None for value in global_values):
                raise ValueError(
                    "global camera metadata requires global_rgb_path"
                )
        else:
            if not self.global_rgb_path.strip():
                raise ValueError("global_rgb_path must not be empty")
            if not self.global_camera_name or not self.global_camera_name.strip():
                raise ValueError(
                    "global_camera_name is required with global_rgb_path"
                )
            if (
                self.global_camera_timestamp_ns is None
                or self.global_camera_timestamp_ns <= 0
            ):
                raise ValueError(
                    "global_camera_timestamp_ns must be positive"
                )
            if self.global_frame_id is None or self.global_frame_id < 0:
                raise ValueError("global_frame_id must be >= 0")

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "Observation":
        return cls(
            timestamp_ns=int(payload["timestamp_ns"]),
            instruction=str(payload["instruction"]),
            rgb_path=str(payload["rgb_path"]),
            depth_path=(
                str(payload["depth_path"])
                if payload.get("depth_path") is not None
                else None
            ),
            robot=RobotState.from_dict(payload["robot"]),
            camera_name=str(payload.get("camera_name", "zed_mini_left")),
            frame_id=int(payload.get("frame_id", 0)),
            global_rgb_path=(
                str(payload["global_rgb_path"])
                if payload.get("global_rgb_path") is not None
                else None
            ),
            global_camera_name=(
                str(payload["global_camera_name"])
                if payload.get("global_camera_name") is not None
                else None
            ),
            global_camera_timestamp_ns=(
                int(payload["global_camera_timestamp_ns"])
                if payload.get("global_camera_timestamp_ns") is not None
                else None
            ),
            global_frame_id=(
                int(payload["global_frame_id"])
                if payload.get("global_frame_id") is not None
                else None
            ),
        )


@dataclass(frozen=True)
class Action:
    """One proposed short-horizon Cartesian action.

    This is data, not permission to move. A later safety filter must approve it
    before a JAKA adapter may send anything to the robot.
    """

    delta_xyz_mm: tuple[float, ...]
    delta_rpy_deg: tuple[float, ...]
    gripper: GripperCommand = GripperCommand.KEEP
    duration_ms: int = 200
    source: ActionSource = ActionSource.HUMAN
    confidence: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "delta_xyz_mm",
            _fixed_floats(self.delta_xyz_mm, length=3, name="delta_xyz_mm"),
        )
        object.__setattr__(
            self,
            "delta_rpy_deg",
            _fixed_floats(self.delta_rpy_deg, length=3, name="delta_rpy_deg"),
        )
        object.__setattr__(self, "gripper", GripperCommand(self.gripper))
        object.__setattr__(self, "source", ActionSource(self.source))
        if self.duration_ms <= 0:
            raise ValueError("duration_ms must be positive")
        if self.confidence is not None and not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["gripper"] = self.gripper.value
        payload["source"] = self.source.value
        return payload

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "Action":
        return cls(
            delta_xyz_mm=payload["delta_xyz_mm"],
            delta_rpy_deg=payload["delta_rpy_deg"],
            gripper=GripperCommand(payload.get("gripper", "keep")),
            duration_ms=int(payload.get("duration_ms", 200)),
            source=ActionSource(payload.get("source", "human")),
            confidence=(
                float(payload["confidence"])
                if payload.get("confidence") is not None
                else None
            ),
        )


@dataclass(frozen=True)
class EpisodeStep:
    """A training pair: current observation followed by the chosen action."""

    observation: Observation
    action: Action

    def to_dict(self) -> dict[str, Any]:
        observation = asdict(self.observation)
        return {"observation": observation, "action": self.action.to_dict()}

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "EpisodeStep":
        return cls(
            observation=Observation.from_dict(payload["observation"]),
            action=Action.from_dict(payload["action"]),
        )


@dataclass(frozen=True)
class Episode:
    """A complete demonstration or rollout for one language instruction."""

    episode_id: str
    instruction: str
    steps: tuple[EpisodeStep, ...]
    success: bool | None = None
    schema_version: str = SCHEMA_VERSION
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.episode_id.strip():
            raise ValueError("episode_id must not be empty")
        if not self.instruction.strip():
            raise ValueError("instruction must not be empty")
        if not self.steps:
            raise ValueError("episode must contain at least one step")
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"unsupported schema_version: {self.schema_version}")
        timestamps = [step.observation.timestamp_ns for step in self.steps]
        if timestamps != sorted(timestamps) or len(timestamps) != len(set(timestamps)):
            raise ValueError("observation timestamps must be strictly increasing")
        for step in self.steps:
            if step.observation.instruction != self.instruction:
                raise ValueError("step instruction must match episode instruction")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "episode_id": self.episode_id,
            "instruction": self.instruction,
            "success": self.success,
            "metadata": dict(self.metadata),
            "steps": [step.to_dict() for step in self.steps],
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "Episode":
        return cls(
            schema_version=str(payload["schema_version"]),
            episode_id=str(payload["episode_id"]),
            instruction=str(payload["instruction"]),
            success=payload.get("success"),
            metadata=dict(payload.get("metadata", {})),
            steps=tuple(EpisodeStep.from_dict(step) for step in payload["steps"]),
        )
