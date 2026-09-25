"""Independent camera and robot polling for the read-only monitor.

The two devices are deliberately sampled on separate threads.  A slow Wi-Fi
status request must never freeze the ZED image, and a camera reconnect must not
erase the last known robot pose.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import replace
from typing import Protocol

from .monitor_types import CameraFrame, MonitorSnapshot, RobotTelemetry


class CameraSource(Protocol):
    def open(self) -> None: ...
    def read_frame(self) -> CameraFrame: ...
    def close(self) -> None: ...


class RobotSource(Protocol):
    def read_telemetry(self) -> RobotTelemetry: ...


class _RateMeter:
    def __init__(self, window: int = 30) -> None:
        self._timestamps: deque[float] = deque(maxlen=window)

    def tick(self) -> float:
        self._timestamps.append(time.monotonic())
        if len(self._timestamps) < 2:
            return 0.0
        duration = self._timestamps[-1] - self._timestamps[0]
        return (len(self._timestamps) - 1) / duration if duration > 0 else 0.0


class ReadOnlyMonitorService:
    """Own device threads and publish the latest immutable snapshot."""

    def __init__(
        self,
        camera: CameraSource,
        robot: RobotSource,
        *,
        global_camera: CameraSource | None = None,
        robot_poll_hz: float = 2.0,
        camera_retry_interval_s: float = 2.0,
        global_camera_retry_interval_s: float = 2.0,
        history_size: int = 120,
    ) -> None:
        if robot_poll_hz <= 0:
            raise ValueError("robot_poll_hz must be positive")
        if history_size < 2:
            raise ValueError("history_size must be at least 2")
        self.camera = camera
        self.robot = robot
        self.global_camera = global_camera
        self.robot_poll_hz = float(robot_poll_hz)
        self.camera_retry_interval_s = float(camera_retry_interval_s)
        self.global_camera_retry_interval_s = float(global_camera_retry_interval_s)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._snapshot = MonitorSnapshot()
        # The UI needs only the newest sample.  Recording instead selects the
        # closest sample from these short histories, using the ZED timestamp as
        # the reference clock.  This is software alignment, never a claim of
        # hardware triggering.
        self._camera_history: deque[CameraFrame] = deque(maxlen=history_size)
        self._robot_history: deque[RobotTelemetry] = deque(maxlen=history_size)
        self._global_camera_history: deque[CameraFrame] = deque(maxlen=history_size)
        self._threads: list[threading.Thread] = []
        self._robot_sample_count = 0
        self._robot_max_gap_ms = 0.0
        self._robot_last_received_ns: int | None = None

    def start(self) -> None:
        if self._threads:
            return
        self._stop.clear()
        self._threads = [
            threading.Thread(
                target=self._camera_loop,
                name="readonly-zed",
                daemon=True,
            ),
            threading.Thread(
                target=self._robot_loop,
                name="readonly-jaka",
                daemon=True,
            ),
        ]
        if self.global_camera is not None:
            self._threads.append(
                threading.Thread(
                    target=self._global_camera_loop,
                    name="readonly-global-camera",
                    daemon=True,
                )
            )
        for thread in self._threads:
            thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop.set()
        deadline = time.monotonic() + timeout_s
        for thread in self._threads:
            thread.join(max(0.0, deadline - time.monotonic()))
        # Normally the camera thread closes its own SDK handle.  This final
        # close is only a fallback for a source that did not stop in time.
        try:
            self.camera.close()
        except Exception:
            pass
        if self.global_camera is not None:
            try:
                self.global_camera.close()
            except Exception:
                pass
        close_robot = getattr(self.robot, "close", None)
        if close_robot is not None:
            try:
                close_robot()
            except Exception:
                pass
        self._threads.clear()

    def snapshot(self) -> MonitorSnapshot:
        with self._lock:
            return self._snapshot

    def set_robot_source(self, source: RobotSource) -> None:
        """Switch endpoints without mixing old and new host history."""
        with self._lock:
            self.robot = source
            self._robot_history.clear()
            self._robot_sample_count = 0
            self._robot_max_gap_ms = 0.0
            self._robot_last_received_ns = None
            self._snapshot = replace(self._snapshot, robot=None, robot_poll_hz=0.0, robot_error="等待独立只读采集")

    def aligned_snapshot(
        self,
        *,
        require_global_camera: bool = False,
        reference_delay_ms: float = 0.0,
        camera_target_timestamp_ns: int | None = None,
        max_camera_target_gap_ms: float | None = None,
    ) -> MonitorSnapshot:
        """Return the closest robot/global samples for a buffered ZED frame.

        The returned timestamps retain their original acquisition values so
        callers can enforce task-specific tolerance gates and audit them. A
        recorder may provide a fixed camera target to keep its sampling grid
        stable while waiting for newer robot telemetry.
        """

        if reference_delay_ms < 0:
            raise ValueError("reference_delay_ms must be non-negative")
        if max_camera_target_gap_ms is not None and max_camera_target_gap_ms <= 0:
            raise ValueError("max_camera_target_gap_ms must be positive")

        with self._lock:
            snapshot = self._snapshot
            newest_camera = snapshot.camera
            if newest_camera is None:
                return snapshot
            if camera_target_timestamp_ns is not None:
                camera = self._nearest(
                    self._camera_history, int(camera_target_timestamp_ns)
                )
                if (
                    camera is not None
                    and max_camera_target_gap_ms is not None
                    and abs(camera.timestamp_ns - int(camera_target_timestamp_ns))
                    > max_camera_target_gap_ms * 1_000_000
                ):
                    camera = None
            else:
                cutoff_ns = newest_camera.timestamp_ns - round(
                    reference_delay_ms * 1_000_000
                )
                camera = next(
                    (
                        frame
                        for frame in reversed(self._camera_history)
                        if frame.timestamp_ns <= cutoff_ns
                    ),
                    None,
                )
            if camera is None:
                # During startup there may not yet be enough buffered ZED
                # frames.  Returning an incomplete snapshot makes a recorder
                # wait instead of pairing a frame with a one-sided history.
                return replace(snapshot, camera=None, robot=None, global_camera=None)
            robot = self._nearest(self._robot_history, camera.timestamp_ns)
            global_camera = self._nearest(
                self._global_camera_history, camera.timestamp_ns
            )
            if robot is None:
                robot = snapshot.robot
            if global_camera is None and not require_global_camera:
                global_camera = snapshot.global_camera
            return replace(
                snapshot, camera=camera, robot=robot, global_camera=global_camera
            )

    @staticmethod
    def _nearest(values: deque[object], reference_timestamp_ns: int) -> object | None:
        if not values:
            return None
        return min(
            values,
            key=lambda value: abs(value.timestamp_ns - reference_timestamp_ns),
        )

    def _update(self, **changes: object) -> None:
        with self._lock:
            self._snapshot = replace(self._snapshot, **changes)

    def _publish_camera(self, frame: CameraFrame, *, fps: float) -> None:
        with self._lock:
            self._camera_history.append(frame)
            self._snapshot = replace(
                self._snapshot, camera=frame, camera_error=None, camera_fps=fps
            )

    def _publish_robot(self, telemetry: RobotTelemetry, *, poll_hz: float, source: RobotSource | None = None) -> None:
        with self._lock:
            if source is not None and source is not self.robot:
                return
            if self._robot_history:
                delta = telemetry.timestamp_ns - self._robot_history[-1].timestamp_ns
                if delta <= 0:
                    return
                self._robot_max_gap_ms = max(self._robot_max_gap_ms, delta / 1e6)
            self._robot_history.append(telemetry)
            self._robot_sample_count += 1
            self._robot_last_received_ns = time.time_ns()
            # Batched arrival frequency is not acquisition frequency.
            # Compute the displayed rate from original sample timestamps.
            recent = list(self._robot_history)[-30:]
            if len(recent) >= 2:
                duration_s = (recent[-1].timestamp_ns - recent[0].timestamp_ns) / 1e9
                poll_hz = (len(recent) - 1) / duration_s if duration_s > 0 else 0.0
            self._snapshot = replace(
                self._snapshot,
                robot=telemetry,
                robot_error=None,
                robot_poll_hz=poll_hz,
            )

    def timing_diagnostics(self) -> dict:
        with self._lock:
            now = time.time_ns()
            return {
                "robot_unique_samples_since_service_start": self._robot_sample_count,
                "robot_max_sample_gap_ms_since_service_start": self._robot_max_gap_ms,
                "robot_last_received_ns": self._robot_last_received_ns,
                "robot_error": self._snapshot.robot_error,
                "robot_poll_hz": self._snapshot.robot_poll_hz,
                "robot_history_timestamp_ns": [value.timestamp_ns for value in self._robot_history],
                "camera_history_timestamp_ns": [value.timestamp_ns for value in self._camera_history],
                "latest_robot_age_ms": None if self._snapshot.robot is None else (now - self._snapshot.robot.timestamp_ns) / 1e6,
            }

    def _publish_global_camera(self, frame: CameraFrame, *, fps: float) -> None:
        with self._lock:
            self._global_camera_history.append(frame)
            self._snapshot = replace(
                self._snapshot,
                global_camera=frame,
                global_camera_error=None,
                global_camera_fps=fps,
            )

    def _camera_loop(self) -> None:
        meter = _RateMeter()
        while not self._stop.is_set():
            try:
                self.camera.open()
                self._update(camera_error=None)
                while not self._stop.is_set():
                    frame = self.camera.read_frame()
                    self._publish_camera(frame, fps=meter.tick())
            except Exception as error:
                if not self._stop.is_set():
                    self._update(camera_error=str(error))
            finally:
                try:
                    self.camera.close()
                except Exception:
                    pass
            self._stop.wait(self.camera_retry_interval_s)

    def _robot_loop(self) -> None:
        meter = _RateMeter()
        period_s = 1.0 / self.robot_poll_hz
        while not self._stop.is_set():
            started = time.monotonic()
            source = self.robot
            try:
                read_batch = getattr(source, "read_telemetry_batch", None)
                batch = read_batch() if callable(read_batch) else [source.read_telemetry()]
                for telemetry in batch:
                    self._publish_robot(telemetry, poll_hz=meter.tick(), source=source)
            except Exception as error:
                with self._lock:
                    if source is self.robot:
                        self._snapshot = replace(self._snapshot, robot_error=str(error))
            elapsed = time.monotonic() - started
            self._stop.wait(max(0.0, period_s - elapsed))

    def _global_camera_loop(self) -> None:
        assert self.global_camera is not None
        meter = _RateMeter()
        while not self._stop.is_set():
            try:
                self.global_camera.open()
                self._update(global_camera_error=None)
                while not self._stop.is_set():
                    frame = self.global_camera.read_frame()
                    self._publish_global_camera(frame, fps=meter.tick())
            except Exception as error:
                if not self._stop.is_set():
                    self._update(global_camera_error=str(error))
            finally:
                try:
                    self.global_camera.close()
                except Exception:
                    pass
            self._stop.wait(self.global_camera_retry_interval_s)
