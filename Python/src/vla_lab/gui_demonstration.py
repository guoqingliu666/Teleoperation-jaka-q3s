"""Non-blocking Episode recording for the JAKA jog GUI.

The GUI remains the only owner of JAKA SDK and MISUMI COM commands.  This
module only mirrors its latest measured state into the existing read-only
camera/synchronisation pipeline and writes Episode files in one background
thread.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from .demonstration_recorder import DemonstrationEpisodeRecorder
from .dataset_validation import validate_episode_directory
from .droidcam_readonly import DroidCamReadOnlyCamera
from .monitor_config import ReadOnlyMonitorConfig, load_monitor_config
from .monitor_service import ReadOnlyMonitorService
from .monitor_types import CameraFrame, MonitorSnapshot, RobotTelemetry
from .observation_recorder import IncompleteObservationError
from .zed_readonly import ZedLeftCamera


# 2026-09-07 keyboard recording audit: at a nominal 30 Hz, saved unique
# robot states were ~19 Hz and global camera frames ~21 Hz. Cap this GUI
# pipeline at 20 Hz; 15 Hz remains the preferred routine collection rate.
# This is an operational estimate, not a guarantee of fresh data each slot.
GUI_RECORD_MIN_HZ = 5.0
GUI_RECORD_MAX_HZ = 50.0


@dataclass(frozen=True)
class GuiRecordingStatus:
    phase: str = "idle"  # idle, starting, recording, finishing, finished, cancelled, error
    episode_id: str | None = None
    step_count: int = 0
    skipped_samples: int = 0
    message: str = "采集源启动中"
    result_path: Path | None = None
    error: str | None = None


class GuiRobotStateBridge:
    """Thread-safe measured-state adapter; it never calls JAKA or COM itself."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._telemetry: RobotTelemetry | None = None
        self._pending: deque[RobotTelemetry] = deque(maxlen=512)
        self.dropped_samples = 0

    def publish(
        self,
        jaka: Any | None,
        gripper: Any | None,
        *,
        selected_tool_id: int | None = None,
    ) -> None:
        if jaka is None or not jaka.connected or jaka.tcp_pose is None:
            return
        pose = tuple(float(value) for value in jaka.tcp_pose)
        if len(pose) != 6:
            return
        joints = (
            tuple(float(value) for value in jaka.joints_rad)
            if jaka.joints_rad is not None and len(jaka.joints_rad) == 6
            else None
        )
        if jaka.timestamp_ns is None:
            return
        timestamp_ns = int(jaka.timestamp_ns)
        gripper_timestamp_ns = getattr(gripper, "timestamp_ns", None) if gripper is not None and gripper.connected else None
        # The JAKA SDK's get_tool_id call may be unavailable on some firmware
        # revisions.  The jog GUI explicitly sends the selected TCP ID on
        # connect/selection, so retain it as a labelled fallback rather than
        # recording an otherwise avoidable null tool ID in every Episode.
        active_tool_id = (
            int(jaka.tool_id)
            if getattr(jaka, "tool_id", None) is not None
            else selected_tool_id
        )
        telemetry = RobotTelemetry(
            timestamp_ns=timestamp_ns,
            tcp_pose_base_mm_deg=(
                pose[0], pose[1], pose[2],
                math.degrees(pose[3]), math.degrees(pose[4]), math.degrees(pose[5]),
            ),
            tool_id=active_tool_id,
            in_motion=bool(jaka.moving),
            joint_positions_rad=joints,
            enabled=bool(jaka.enabled),
            joint_timestamp_ns=timestamp_ns if joints is not None else None,
            controller_reachable=True,
            gripper_opening_mm=(
                float(gripper.opening_mm)
                if gripper is not None and gripper.connected and gripper.opening_mm is not None
                else None
            ),
            gripper_timestamp_ns=gripper_timestamp_ns,
            gripper_in_motion=(bool(gripper.in_motion) if gripper is not None and gripper.connected else None),
            gripper_source=(
                f"misumi_gui_com6_{gripper.opening_source or 'unknown'}"
                if gripper is not None and gripper.connected
                else None
            ),
            gripper_error=(gripper.error if gripper is not None else None),
            gripper_status_code=(gripper.holding_status if gripper is not None else None),
            gripper_fault_code=(gripper.fault if gripper is not None else None),
            gripper_position_units=(gripper.closing_units if gripper is not None else None),
            gripper_enable_status_code=(gripper.enable_status if gripper is not None else None),
        )
        with self._lock:
            if self._telemetry is not None and timestamp_ns <= self._telemetry.timestamp_ns:
                return
            self._telemetry = telemetry
            if len(self._pending) == self._pending.maxlen:
                self.dropped_samples += 1
            self._pending.append(telemetry)

    def read_telemetry_batch(self) -> list[RobotTelemetry]:
        """Transfer measured samples once; repeated polling creates no data."""
        with self._lock:
            values = list(self._pending)
            self._pending.clear()
            return values

    def read_telemetry(self) -> RobotTelemetry:
        with self._lock:
            telemetry = self._telemetry
        if telemetry is None:
            raise RuntimeError("等待 JAKA Jog GUI 的实时状态")
        return telemetry


def _freeze_frame(frame: CameraFrame | None, *, copy_depth: bool = True) -> CameraFrame | None:
    if frame is None:
        return None
    return CameraFrame(
        timestamp_ns=frame.timestamp_ns,
        frame_id=frame.frame_id,
        image_bgr=np.asarray(frame.image_bgr).copy(),
        depth_mm=(
            np.asarray(frame.depth_mm).copy()
            if copy_depth and frame.depth_mm is not None
            else None
        ),
        camera_name=frame.camera_name,
        serial_number=frame.serial_number,
    )


def _freeze_snapshot(snapshot: MonitorSnapshot, *, copy_depth: bool = True) -> MonitorSnapshot:
    """Copy image arrays before passing them from Tk to the writer thread."""

    return MonitorSnapshot(
        camera=_freeze_frame(snapshot.camera, copy_depth=copy_depth),
        robot=snapshot.robot,
        camera_error=snapshot.camera_error,
        robot_error=snapshot.robot_error,
        camera_fps=snapshot.camera_fps,
        robot_poll_hz=snapshot.robot_poll_hz,
        global_camera=_freeze_frame(snapshot.global_camera, copy_depth=False),
        global_camera_error=snapshot.global_camera_error,
        global_camera_fps=snapshot.global_camera_fps,
    )


class GuiDemonstrationSession:
    """Camera synchronisation plus a background writer for one GUI Episode."""

    def __init__(self, config_path: str | Path) -> None:
        self.config_path = Path(config_path).resolve()
        self.config: ReadOnlyMonitorConfig = load_monitor_config(self.config_path)
        self.bridge = GuiRobotStateBridge()
        global_camera = (
            DroidCamReadOnlyCamera(self.config.global_camera)
            if self.config.global_camera.enabled
            else None
        )
        self.service = ReadOnlyMonitorService(
            ZedLeftCamera(self.config.zed),
            self.bridge,
            global_camera=global_camera,
            robot_poll_hz=max(5.0, self.config.jaka.poll_hz),
            camera_retry_interval_s=self.config.zed.retry_interval_s,
            global_camera_retry_interval_s=self.config.global_camera.retry_interval_s,
        )
        self._status = GuiRecordingStatus()
        self._status_lock = threading.Lock()
        self._commands: queue.Queue[tuple[str, object]] = queue.Queue(maxsize=12)
        self._events: queue.Queue[GuiRecordingStatus] = queue.Queue()
        self._writer: threading.Thread | None = None
        self._sampler: threading.Thread | None = None
        self._sample_stop = threading.Event()
        self._next_camera_timestamp_ns: int | None = None
        self._sources_started = False
        self._sampling_diagnostics_lock = threading.Lock()
        self._sampling_skip_reasons: Counter[str] = Counter()
        self._sampling_overrun_ms: list[float] = []
        self._last_sampling_detail = ""
        self._last_sync_failure: dict | None = None
        self._sampling_rate_advisory = ""

    @property
    def status(self) -> GuiRecordingStatus:
        with self._status_lock:
            return self._status

    @property
    def sample_hz(self) -> float:
        return float(self.config.demonstration.sample_hz)

    @property
    def capture_depth(self) -> bool:
        """RGB-D is retained only when explicitly configured for this session."""
        return bool(getattr(getattr(self.config, "zed", None), "capture_depth", True))

    @property
    def sampling_rate_advisory(self) -> str:
        return self._sampling_rate_advisory

    def sampling_diagnostics(self) -> str:
        """Return compact recorder-side causes without querying hardware."""

        with self._sampling_diagnostics_lock:
            parts = [
                f"{reason} ×{count}"
                for reason, count in self._sampling_skip_reasons.most_common(3)
            ]
            if self._sampling_overrun_ms:
                parts.append(f"采样线程超时最大 {max(self._sampling_overrun_ms):.0f} ms")
            if self._last_sampling_detail:
                parts.append(self._last_sampling_detail)
        return "；".join(parts) if parts else "未记录到具体跳帧原因"

    def _note_sampling_issue(
        self,
        reason: str,
        *,
        elapsed_ms: float | None = None,
        detail: str | None = None,
    ) -> None:
        with self._sampling_diagnostics_lock:
            self._sampling_skip_reasons[reason] += 1
            if elapsed_ms is not None:
                self._sampling_overrun_ms.append(float(elapsed_ms))
            if detail:
                self._last_sampling_detail = str(detail)

    def set_sample_hz(self, sample_hz: float, *, persist: bool = True) -> float:
        """Configure the next Episode rate without touching robot hardware.

        Rates above a configured source rate are permitted only to measure the
        recorder's failure boundary. They are explicitly labelled as a stress
        test; skipped/invalid samples still make the Episode fail validation.
        """

        if self.status.phase in {"starting", "recording", "finishing"}:
            raise RuntimeError("正在录制，不能修改采样频率")
        rate = float(sample_hz)
        if not GUI_RECORD_MIN_HZ <= rate <= GUI_RECORD_MAX_HZ:
            raise ValueError(
                f"采样频率必须在 {GUI_RECORD_MIN_HZ:g} 到 {GUI_RECORD_MAX_HZ:g} Hz；"
                "正式示教建议 15 Hz；高频仅用于压力测试"
            )
        source_rates = {
            "ZED": float(self.config.zed.fps),
            "JAKA": float(self.config.jaka.poll_hz),
        }
        if self.config.global_camera.enabled:
            source_rates["全局相机"] = float(self.config.global_camera.fps)
        available = min(source_rates.values())
        if rate > available:
            limiting = "/".join(
                name for name, value in source_rates.items() if value == available
            )
            self._sampling_rate_advisory = (
                f"压力测试：{rate:g} Hz 高于 {limiting} 的 {available:g} Hz；"
                "保存质量校验可能拒绝本条 Episode"
            )
        else:
            self._sampling_rate_advisory = ""
        self.config = replace(
            self.config,
            demonstration=replace(self.config.demonstration, sample_hz=rate),
        )
        if persist:
            payload = json.loads(self.config_path.read_text(encoding="utf-8"))
            demonstration = dict(payload.get("demonstration", {}))
            demonstration["sample_hz"] = rate
            payload["demonstration"] = demonstration
            self.config_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        return rate

    def start_sources(self) -> None:
        if not self._sources_started:
            self.service.start()
            self._sources_started = True

    def stop_sources(self, *, force: bool = False) -> None:
        """Release both camera devices while keeping the GUI/JAKA bridge alive."""

        if not force and self.status.phase in {"starting", "recording", "finishing"}:
            raise RuntimeError("正在录制，不能关闭相机")
        if self._sources_started:
            self.service.stop()
            self._sources_started = False

    def stop(self) -> None:
        self.cancel()
        self._stop_sampler()
        self.stop_sources(force=True)

    def publish_gui_state(
        self,
        jaka: Any | None,
        gripper: Any | None,
        *,
        selected_tool_id: int | None = None,
    ) -> None:
        self.bridge.publish(jaka, gripper, selected_tool_id=selected_tool_id)

    def reset_robot_bridge(self) -> None:
        """Clear recording history without opening ANY robot connection.

        The motion worker is the only JAKA session owner. A second JSON
        connection caused SDK communication failures on this controller.
        """
        self.bridge = GuiRobotStateBridge()
        self.service.set_robot_source(self.bridge)

    def preview_snapshot(self) -> MonitorSnapshot:
        """Return the newest two camera frames for the GUI preview.

        This is intentionally the live service snapshot, not an aligned copy:
        preview latency matters more than recording-time pairing, and the
        camera loops publish immutable frame arrays.
        """

        return self.service.snapshot()

    def readiness(
        self,
        *,
        camera_target_timestamp_ns: int | None = None,
    ) -> tuple[bool, str, MonitorSnapshot]:
        snapshot = self.service.aligned_snapshot(
            require_global_camera=self.config.global_camera.enabled,
            reference_delay_ms=self.config.recorder.alignment_delay_ms,
            camera_target_timestamp_ns=camera_target_timestamp_ns,
            max_camera_target_gap_ms=(
                min(80.0, 400.0 / self.config.demonstration.sample_hz)
                if camera_target_timestamp_ns is not None
                else None
            ),
        )
        if snapshot.camera is None:
            return False, "等待 ZED RGB", snapshot
        if self.capture_depth:
            # ZED can occasionally publish a depth buffer that exists but is
            # entirely invalid (NaN/Inf/0). It is only required in RGB-D mode.
            depth_mm = snapshot.camera.depth_mm
            if depth_mm is None:
                return False, "等待 ZED 深度", snapshot
            if not np.any(np.isfinite(depth_mm) & (depth_mm > 0)):
                return False, "等待有效 ZED 深度", snapshot
        if snapshot.robot is None or snapshot.robot.joint_positions_rad is None:
            return False, "等待 JAKA 关节状态", snapshot
        robot_gap_ms = snapshot.software_time_delta_ms
        if robot_gap_ms is None or robot_gap_ms > self.config.recorder.max_snapshot_gap_ms:
            # Selected frames are historical by design. Their age alone is
            # not evidence that live telemetry has stopped updating.
            latest_robot = self.service.snapshot().robot
            now_ns = time.time_ns()
            latest_age = "—" if latest_robot is None else f"{max(0, now_ns - latest_robot.timestamp_ns) / 1e6:.0f} ms"
            camera_age = f"{max(0, now_ns - snapshot.camera.timestamp_ns) / 1e6:.0f} ms"
            robot_age_ms = (
                max(0.0, (time.time_ns() - snapshot.robot.timestamp_ns) / 1_000_000.0)
                if snapshot.robot is not None
                else None
            )
            gap_text = "—" if robot_gap_ms is None else f"{robot_gap_ms:.0f} ms"
            age_text = "—" if robot_age_ms is None else f"{robot_age_ms:.0f} ms"
            return (
                False,
                f"等待 ZED 与 JAKA 同步（时间差 {gap_text}，配对JAKA年龄 {age_text}，相机帧年龄 {camera_age}，最新JAKA年龄 {latest_age}）",
                snapshot,
            )
        if self.config.global_camera.enabled:
            if snapshot.global_camera is None:
                return False, "等待 DroidCam 全局画面", snapshot
            if (
                snapshot.global_camera_delta_ms is None
                or snapshot.global_camera_delta_ms > self.config.recorder.max_global_camera_gap_ms
            ):
                return False, "等待 ZED 与 DroidCam 同步", snapshot
        if snapshot.robot.gripper_opening_mm is None:
            return False, "等待 MISUMI 夹爪开口状态", snapshot
        return True, "采集源已就绪", snapshot

    def start_recording(self, instruction: str) -> None:
        ready, message, _snapshot = self.readiness()
        if not ready:
            raise IncompleteObservationError(message)
        if self.status.phase not in {"idle", "finished", "cancelled", "error"}:
            raise RuntimeError("已有示教录制正在进行")
        self._stop_sampler()
        self._sample_stop = threading.Event()
        self._next_camera_timestamp_ns = None
        with self._sampling_diagnostics_lock:
            self._sampling_skip_reasons.clear()
            self._sampling_overrun_ms.clear()
            self._last_sampling_detail = ""
            self._last_sync_failure = None
        self._commands = queue.Queue(maxsize=12)
        self._writer = threading.Thread(target=self._writer_main, name="gui-episode-writer", daemon=True)
        self._writer.start()
        self._set_status(GuiRecordingStatus(phase="starting", message="正在创建 Episode"))
        self._commands.put(("start", instruction.strip()))
        self._sampler = threading.Thread(
            target=self._sampler_main,
            name="gui-episode-sampler",
            daemon=True,
        )
        self._sampler.start()

    def _sampler_main(self) -> None:
        """Capture at a fixed rate independently of Tk preview rendering."""

        period_s = 1.0 / self.config.demonstration.sample_hz
        retry_window_s = min(0.08, period_s * 0.4)
        next_tick_s: float | None = None
        while not self._sample_stop.is_set():
            phase = self.status.phase
            if phase == "starting":
                self._sample_stop.wait(0.01)
                continue
            if phase != "recording":
                return
            sample_started_s = time.monotonic()
            if next_tick_s is None:
                next_tick_s = sample_started_s
            # Bounded catch-up uses real buffered samples. If a long stall
            # exceeds this budget, advance BOTH clocks and count lost slots.
            # Otherwise the camera grid falls farther behind on every retry.
            lag_s = sample_started_s - next_tick_s
            if lag_s > max(0.5, 3 * period_s):
                lost_slots = int(lag_s / period_s)
                if self._next_camera_timestamp_ns is not None:
                    self._next_camera_timestamp_ns += lost_slots * round(period_s * 1e9)
                next_tick_s += lost_slots * period_s
                for _ in range(lost_slots):
                    self._note_sampling_issue("采样调度落后，跳过过期时隙")
                    self._send_nonblocking("skip", None)
            self.sample(retry_window_s=retry_window_s)
            elapsed_s = time.monotonic() - sample_started_s
            if elapsed_s > period_s:
                self._note_sampling_issue(
                    "采样线程处理超时",
                    elapsed_ms=elapsed_s * 1000.0,
                )
            next_tick_s += period_s
            self._sample_stop.wait(max(0.0, next_tick_s - time.monotonic()))

    def _stop_sampler(self) -> None:
        self._sample_stop.set()
        sampler = self._sampler
        if sampler is not None and sampler.is_alive() and sampler is not threading.current_thread():
            sampler.join(0.5)
        self._sampler = None

    def sample(self, *, retry_window_s: float = 0.0) -> None:
        """Queue one aligned observation, briefly awaiting a fresh source.

        The synchronisation limits are never relaxed.  Retrying only avoids a
        phase race where the 5 Hz sampler checks a few milliseconds before the
        next 15 Hz JAKA telemetry message arrives.
        """

        deadline_s = time.monotonic() + max(0.0, float(retry_window_s))
        camera_target_timestamp_ns = self._next_camera_timestamp_ns
        period_ns = round(1_000_000_000 / self.config.demonstration.sample_hz)
        while self.status.phase == "recording" and not self._sample_stop.is_set():
            ready, _message, snapshot = self.readiness(
                camera_target_timestamp_ns=camera_target_timestamp_ns,
            )
            # Freeze the first selected ZED frame for this slot. A retry may
            # receive fresher JAKA telemetry, but it must not drift to the next
            # camera frame and create a 133/267 ms pair.
            if camera_target_timestamp_ns is None and snapshot.camera is not None:
                camera_target_timestamp_ns = snapshot.camera.timestamp_ns
            # Apply the same completeness/synchronisation gate for every
            # sample as for recording start.  Never write stale state.
            if ready and snapshot.camera is not None:
                self._send_nonblocking(
                    "append",
                    _freeze_snapshot(
                        snapshot, copy_depth=self.capture_depth
                    ),
                )
                self._next_camera_timestamp_ns = (
                    int(camera_target_timestamp_ns) + period_ns
                )
                return
            remaining_s = deadline_s - time.monotonic()
            if _message.startswith("等待 ZED 与 JAKA 同步") and snapshot.camera is not None:
                latest_robot = self.service.snapshot().robot
                # If robot history has already passed this frame, newer
                # state cannot repair a historical hole. Avoid retrying it.
                if latest_robot is not None and latest_robot.timestamp_ns > snapshot.camera.timestamp_ns + round(self.config.recorder.max_snapshot_gap_ms * 1e6):
                    remaining_s = 0.0
            if remaining_s <= 0:
                if _message.startswith("等待 ZED 与 JAKA 同步"):
                    self._last_sync_failure = {
                        "recorded_at_ns": time.time_ns(),
                        "selected_camera_ns": snapshot.camera.timestamp_ns if snapshot.camera else None,
                        "selected_robot_ns": snapshot.robot.timestamp_ns if snapshot.robot else None,
                        **self.service.timing_diagnostics(),
                    }
                    self._note_sampling_issue(
                        "等待 ZED 与 JAKA 同步",
                        detail=_message,
                    )
                else:
                    self._note_sampling_issue(_message)
                self._send_nonblocking("skip", None)
                if camera_target_timestamp_ns is not None:
                    self._next_camera_timestamp_ns = (
                        int(camera_target_timestamp_ns) + period_ns
                    )
                return
            self._sample_stop.wait(min(0.01, remaining_s))

    def mark_stage(self, label: str) -> None:
        if self.status.phase == "recording":
            self._send_nonblocking("marker", label)

    def finish(self, success: bool) -> None:
        if self.status.phase != "recording":
            return
        self._set_status(GuiRecordingStatus(**{**self.status.__dict__, "phase": "finishing", "message": "正在写入并校验 Episode"}))
        self._stop_sampler()
        # The GUI can have several image-writing requests queued when the user
        # presses Save.  Do not let those stale sampling requests delay or
        # block the finish signal on the Tk thread.  They are deliberately
        # dropped: an Episode remains valid with its already-written frames.
        retained: list[tuple[str, object]] = []
        while True:
            try:
                name, value = self._commands.get_nowait()
            except queue.Empty:
                break
            if name not in {"append", "skip"}:
                retained.append((name, value))
        for command in retained:
            self._commands.put_nowait(command)
        self._commands.put_nowait(("finish", bool(success)))

    def cancel(self) -> None:
        self._stop_sampler()
        if self._writer is not None and self._writer.is_alive():
            try:
                self._commands.put_nowait(("cancel", None))
            except queue.Full:
                pass
            self._writer.join(1.0)
        self._writer = None
        if self.status.phase in {"starting", "recording", "finishing"}:
            self._set_status(GuiRecordingStatus(phase="cancelled", message="本次 Episode 已放弃"))

    def drain_events(self) -> GuiRecordingStatus:
        while True:
            try:
                incoming = self._events.get_nowait()
            except queue.Empty:
                return self.status
            current = self.status
            # Writer events are asynchronous.  An older "recording" event
            # may arrive after the GUI has already requested finish; never
            # allow it to resurrect sampling or overwrite a terminal result.
            if (
                current.phase in {"finishing", "finished", "cancelled", "error"}
                and incoming.phase == "recording"
            ):
                continue
            self._set_status(incoming)

    def _send_nonblocking(self, name: str, value: object) -> None:
        try:
            self._commands.put_nowait((name, value))
        except queue.Full:
            self._note_sampling_issue("写盘队列忙")
            current = self.status
            self._set_status(GuiRecordingStatus(**{**current.__dict__, "skipped_samples": current.skipped_samples + 1, "message": "写盘队列忙，已跳过一帧"}))

    def _set_status(self, status: GuiRecordingStatus) -> None:
        with self._status_lock:
            self._status = status

    def _emit(self, status: GuiRecordingStatus) -> None:
        self._events.put(status)

    def _writer_main(self) -> None:
        output = Path(self.config.demonstration.output_directory)
        if not output.is_absolute():
            output = self.config_path.parents[1] / output
        recorder = DemonstrationEpisodeRecorder(
            output,
            self.config.recorder.max_snapshot_gap_ms,
            require_gripper=True,
            require_global_camera=self.config.global_camera.enabled,
            max_global_camera_gap_ms=self.config.recorder.max_global_camera_gap_ms,
            alignment_delay_ms=self.config.recorder.alignment_delay_ms,
            global_camera_watermark_present=self.config.global_camera.watermark_present,
            human_control_commands=True,
            sample_hz=self.config.demonstration.sample_hz,
            record_depth=self.capture_depth,
        )
        skipped = 0
        try:
            while True:
                name, value = self._commands.get()
                if name == "start":
                    episode_id = recorder.start(str(value))
                    self._emit(GuiRecordingStatus(phase="recording", episode_id=episode_id, message="正在录制：人工控制机械臂完成整段任务"))
                elif name == "append":
                    try:
                        count = recorder.append(value)  # type: ignore[arg-type]
                        self._emit(GuiRecordingStatus(phase="recording", episode_id=recorder._episode_id, step_count=count, skipped_samples=skipped, message="正在录制"))
                    except IncompleteObservationError as error:
                        self._note_sampling_issue(f"写盘前校验：{error}")
                        recorder.note_skipped_sample()
                        skipped += 1
                        self._emit(GuiRecordingStatus(phase="recording", episode_id=recorder._episode_id, step_count=recorder.step_count, skipped_samples=skipped, message=f"跳过无效采样：{error}"))
                elif name == "skip":
                    recorder.note_skipped_sample()
                    skipped += 1
                elif name == "marker":
                    recorder.mark_stage(str(value))
                    self._emit(GuiRecordingStatus(phase="recording", episode_id=recorder._episode_id, step_count=recorder.step_count, skipped_samples=skipped, message=f"已标记：{value}"))
                elif name == "finish":
                    step_count = recorder.step_count
                    try:
                        result = recorder.finish(success=bool(value))
                    except IncompleteObservationError as error:
                        raise RuntimeError(
                            f"{error}；诊断：{self.sampling_diagnostics()}"
                        ) from error
                    report = validate_episode_directory(result)
                    if not report.valid:
                        raise RuntimeError("Episode 文件校验失败：" + "; ".join(report.issues))
                    self._emit(GuiRecordingStatus(phase="finished", episode_id=result.name, step_count=step_count, skipped_samples=skipped, result_path=result, message="Episode 已保存；可进入 ACT 审计"))
                    return
                elif name == "cancel":
                    recorder.cancel()
                    self._emit(GuiRecordingStatus(phase="cancelled", skipped_samples=skipped, message="本次 Episode 已放弃"))
                    return
        except Exception as error:
            # Preserve diagnostics even when the incomplete Episode is
            # cancelled; an unsuccessful save must not erase the evidence.
            try:
                log_dir = self.config_path.parents[1] / "logs"
                log_dir.mkdir(parents=True, exist_ok=True)
                report = {
                    "error": str(error),
                    "sample_hz": self.sample_hz,
                    "robot_source": "sdk_bridge",
                    "episode_id": recorder._episode_id,
                    "diagnostics": self.sampling_diagnostics(),
                    "skip_reasons": dict(self._sampling_skip_reasons),
                    "sampling_quality": recorder._sampling_quality(),
                    "stream_timing": self.service.timing_diagnostics(),
                    "bridge_overflow_samples": self.bridge.dropped_samples,
                    "controller_overflow_samples": getattr(self, "controller_telemetry_dropped", 0),
                    "last_sync_failure": self._last_sync_failure,
                }
                (log_dir / f"recording_failure_{time.time_ns()}.json").write_text(
                    json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
                )
            except Exception:
                pass
            recorder.cancel()
            self._emit(GuiRecordingStatus(phase="error", skipped_samples=skipped, error=str(error), message=f"录制失败：{error}"))
        finally:
            self._sample_stop.set()
