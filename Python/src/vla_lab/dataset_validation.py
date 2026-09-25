"""Offline structural and numerical validation for recorded VLA episodes."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .contracts import Observation
from .storage import load_episode


@dataclass(frozen=True)
class EpisodeValidationReport:
    episode_directory: Path
    step_count: int
    issues: tuple[str, ...]

    @property
    def valid(self) -> bool:
        return not self.issues


def _resolve_member(root: Path, relative: str) -> Path:
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root)
    except ValueError as error:
        raise ValueError(f"path escapes episode directory: {relative}") from error
    return candidate


def _read_image_unicode_safe(path: Path) -> np.ndarray | None:
    """Decode an image when its Windows path contains non-ASCII characters."""

    # ``cv2.imread(str(path))`` is not Unicode-safe on some Windows OpenCV
    # builds.  Episode relation directories intentionally contain Chinese
    # names, so use Python to open the path and OpenCV only to decode bytes.
    try:
        encoded = np.fromfile(path, dtype=np.uint8)
    except OSError:
        return None
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def validate_episode_directory(path: str | Path) -> EpisodeValidationReport:
    root = Path(path).resolve()
    issues: list[str] = []
    try:
        episode = load_episode(root / "episode.json")
    except Exception as error:
        return EpisodeValidationReport(root, 0, (f"invalid episode.json: {error}",))

    terminal_relative = episode.metadata.get("terminal_observation_path")
    terminal: Observation | None = None
    if not isinstance(terminal_relative, str):
        issues.append("metadata.terminal_observation_path is missing")
    else:
        try:
            terminal_payload = json.loads(
                _resolve_member(root, terminal_relative).read_text(encoding="utf-8")
            )
            terminal = Observation.from_dict(terminal_payload)
        except Exception as error:
            issues.append(f"invalid terminal observation: {error}")

    observations = [step.observation for step in episode.steps]
    depth_recorded = bool(episode.metadata.get("depth_recorded", True))
    if terminal is not None:
        observations.append(terminal)
    global_camera_flags: list[bool] = []
    for index, observation in enumerate(observations):
        global_camera_flags.append(observation.global_rgb_path is not None)
        try:
            rgb_path = _resolve_member(root, observation.rgb_path)
            rgb = _read_image_unicode_safe(rgb_path)
            if rgb is None:
                raise ValueError(f"RGB cannot be decoded: {rgb_path}")
            if depth_recorded:
                if observation.depth_path is None:
                    raise ValueError("depth path is missing from an RGB-D episode")
                depth_path = _resolve_member(root, observation.depth_path)
                depth = np.load(depth_path, allow_pickle=False)
                if depth.ndim != 2:
                    raise ValueError(f"depth must be 2-D, got {depth.shape}")
                if rgb.shape[:2] != depth.shape:
                    raise ValueError(
                        f"RGB/depth shape mismatch: {rgb.shape[:2]} vs {depth.shape}"
                    )
                if depth.dtype != np.float32:
                    raise ValueError(f"depth dtype must be float32, got {depth.dtype}")
                if not np.any(np.isfinite(depth) & (depth > 0)):
                    raise ValueError("depth contains no positive finite measurement")
            elif observation.depth_path is not None:
                raise ValueError("RGB-only episode unexpectedly contains a depth path")
            if observation.global_rgb_path is not None:
                global_rgb_path = _resolve_member(
                    root, observation.global_rgb_path
                )
                global_rgb = _read_image_unicode_safe(global_rgb_path)
                if global_rgb is None:
                    raise ValueError(
                        f"global RGB cannot be decoded: {global_rgb_path}"
                    )
        except Exception as error:
            issues.append(f"observation {index}: {error}")
    if any(global_camera_flags) and not all(global_camera_flags):
        issues.append(
            "global RGB camera is present in only part of the episode"
        )

    if terminal is not None:
        for index, step in enumerate(episode.steps):
            following = observations[index + 1]
            current_pose = step.observation.robot.tcp_pose_base_mm_deg
            next_pose = following.robot.tcp_pose_base_mm_deg
            expected_translation = tuple(
                next_pose[axis] - current_pose[axis] for axis in range(3)
            )
            expected_rotation = tuple(
                (next_pose[axis] - current_pose[axis] + 180.0) % 360.0 - 180.0
                for axis in range(3, 6)
            )
            if any(
                not math.isclose(actual, expected, abs_tol=1e-6)
                for actual, expected in zip(
                    step.action.delta_xyz_mm, expected_translation
                )
            ):
                issues.append(f"step {index}: translation action does not reach next pose")
            if any(
                not math.isclose(actual, expected, abs_tol=1e-6)
                for actual, expected in zip(
                    step.action.delta_rpy_deg, expected_rotation
                )
            ):
                issues.append(f"step {index}: rotation action does not reach next pose")
            expected_duration = max(
                1,
                round(
                    (following.timestamp_ns - step.observation.timestamp_ns) / 1e6
                ),
            )
            if step.action.duration_ms != expected_duration:
                issues.append(f"step {index}: duration does not match timestamps")

    safety = episode.metadata.get("safety")
    if not isinstance(safety, dict) or safety.get("read_only") is not True:
        issues.append("episode is not marked read_only")
    elif safety.get("robot_command_sent") is not False or safety.get(
        "gripper_command_sent"
    ) is not False:
        issues.append("episode safety metadata reports a hardware command")
    return EpisodeValidationReport(root, len(episode.steps), tuple(issues))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_directory", type=Path)
    args = parser.parse_args(argv)
    report = validate_episode_directory(args.episode_directory)
    if report.valid:
        print(f"[PASS] valid episode with {report.step_count} steps")
        return 0
    print("[FAIL] invalid episode")
    for issue in report.issues:
        print(f"- {issue}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
