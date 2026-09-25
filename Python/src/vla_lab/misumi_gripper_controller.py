"""Manually-triggered MISUMI USB gripper controller for the JAKA jog GUI.

The worker is the sole COM-port owner. It exposes only manually triggered
enable/open/close/soft-stop operations. Open/close use one shared speed and
force setting, with all requested values range-checked before they are sent.
"""

from __future__ import annotations

import multiprocessing as mp
import math
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path
from typing import Any


def _load_pyserial() -> Any:
    """Load pyserial without exposing the ACT process to the legacy runtime.

    The ACT virtual environment intentionally contains only training
    dependencies.  The existing hi-2 hardware runtime already owns the
    verified pyserial 3.5 installation, so only the dedicated MISUMI child
    process receives that site-packages fallback.
    """

    try:
        import serial  # type: ignore[import-not-found]

        return serial
    except ModuleNotFoundError:
        legacy_site = (
            Path(__file__).resolve().parents[2]
            / ".."
            / "hi-2"
            / "work"
            / "runtime"
            / "Lib"
            / "site-packages"
        ).resolve()
        if not (legacy_site / "serial" / "__init__.py").is_file():
            raise RuntimeError(
                "pyserial is unavailable in both .venv-act and the hi-2 runtime"
            )
        sys.path.insert(0, str(legacy_site))
        import serial  # type: ignore[import-not-found,no-redef]

        return serial

from .gripper_readonly import modbus_rtu_crc


STATUS_ADDRESS = 0x1194
STATUS_COUNT = 6


def measured_opening_mm(closing_units: int, config: dict[str, Any]) -> tuple[float, str]:
    """Convert feedback, accepting only a bounded closed-end zero offset.

    The observed peak was -0.51 mm. Use a 0.60 mm software tolerance,
    NOT a manufacturer accuracy claim. Preserve closing_units unchanged;
    out-of-tolerance readings remain invalid for recording validation.
    """
    unit = float(config["position_unit_mm"])
    tolerance = float(config.get("closed_zero_tolerance_mm", 0.60))
    if not math.isfinite(unit) or unit <= 0:
        raise ValueError("position_unit_mm must be positive and finite")
    if not math.isfinite(tolerance) or not 0 <= tolerance <= 0.60:
        raise ValueError("closed_zero_tolerance_mm must be within [0, 0.60]")
    opening = (int(config["fully_closed_position_units"]) - closing_units) * unit
    if -tolerance <= opening < 0:
        return 0.0, "measured_closed_zero_tolerance"
    return opening, "measured"


@dataclass
class MisumiGripperSnapshot:
    timestamp_ns: int | None = None
    connected: bool = False
    enable_status: int | None = None
    fault: int | None = None
    holding_status: int | None = None
    closing_units: int | None = None
    opening_mm: float | None = None
    # ``measured`` comes from 0x1194 feedback.  Some USB adapters acknowledge
    # motion writes while intermittently dropping that read response; in that
    # case a successfully acknowledged open/close request still provides a
    # useful, explicitly labelled binary demonstration state.
    opening_source: str | None = None
    in_motion: bool = False
    error: str = ""
    log: list[str] = field(default_factory=list)


def _frame(slave_id: int, function: int, address: int, value: int) -> bytes:
    payload = bytes((slave_id, function)) + address.to_bytes(2, "big") + value.to_bytes(2, "big")
    return payload + modbus_rtu_crc(payload)


def _read_status(serial_port: object, slave_id: int) -> tuple[int, int, int, int, int, int]:
    request = _frame(slave_id, 3, STATUS_ADDRESS, STATUS_COUNT)
    serial_port.reset_input_buffer()
    if serial_port.write(request) != len(request):
        raise RuntimeError("MISUMI status request was not fully written")
    serial_port.flush()
    head = serial_port.read(3)
    if len(head) != 3:
        raise TimeoutError("MISUMI status response timed out")
    if head[0] != slave_id or head[1] != 3 or head[2] != STATUS_COUNT * 2:
        raise RuntimeError("MISUMI malformed status response")
    tail = serial_port.read(head[2] + 2)
    response = head + tail
    if len(tail) != head[2] + 2 or response[-2:] != modbus_rtu_crc(response[:-2]):
        raise RuntimeError("MISUMI status response CRC/integrity failure")
    return tuple(int.from_bytes(response[index : index + 2], "big") for index in range(3, 15, 2))


def _write_immediate_motion(
    serial_port: object, *, slave_id: int, position_units: int, speed_percent: int,
    force_percent: int, enable_value: int,
) -> tuple[bytes, bytes, bool]:
    """Use the six-register instantaneous-motion frame observed from Studio.

    This is intentionally not the dormant pre-set selector at ``0x0FA1``.
    The installed Studio has demonstrated that it writes 0x0FA0..0x0FA5:
    enable, mode=0, position, speed, force, trigger=1.
    """
    values = (enable_value, 0, position_units, speed_percent, force_percent, 1)
    payload = (
        bytes((slave_id, 0x10)) + (0x0FA0).to_bytes(2, "big")
        + len(values).to_bytes(2, "big") + bytes((len(values) * 2,))
        + b"".join(int(value).to_bytes(2, "big") for value in values)
    )
    request = payload + modbus_rtu_crc(payload)
    serial_port.reset_input_buffer()
    if serial_port.write(request) != len(request):
        raise RuntimeError("MISUMI command was not fully written")
    serial_port.flush()
    response = serial_port.read(8)
    acknowledgement = bytes((slave_id, 0x10)) + (0x0FA0).to_bytes(2, "big") + len(values).to_bytes(2, "big")
    acknowledgement += modbus_rtu_crc(acknowledgement)
    acknowledged = len(response) == 8 and response == acknowledgement
    return request, response, acknowledged


def _write_single_register(
    serial_port: object, *, slave_id: int, address: int, value: int,
) -> tuple[bytes, bytes]:
    request = _frame(slave_id, 6, address, value)
    serial_port.reset_input_buffer()
    if serial_port.write(request) != len(request):
        raise RuntimeError("MISUMI command was not fully written")
    serial_port.flush()
    response = serial_port.read(8)
    if len(response) != 8 or response != request:
        raise RuntimeError("MISUMI command acknowledgement failed")
    return request, response


def _worker_main(config: dict[str, Any], command_conn: Any, event_conn: Any) -> None:
    serial_port: Any = None
    last_poll = 0.0
    poll_interval = 1.0 / max(1.0, float(config.get("poll_hz", 10.0)))
    log: list[str] = []
    log_directory = Path(__file__).resolve().parents[2] / "logs"
    log_directory.mkdir(parents=True, exist_ok=True)
    log_path = log_directory / f"misumi_gripper_{datetime.now():%Y%m%d_%H%M%S}.log"
    last_status: tuple[int, int, int, int] | None = None
    consecutive_status_errors = 0

    def add_log(message: str) -> None:
        stamped = f"{datetime.now():%Y-%m-%d %H:%M:%S.%f}"[:-3] + f" {message}"
        log.append(stamped)
        with log_path.open("a", encoding="utf-8") as handle:
            handle.write(stamped + "\n")

    def emit(**values: Any) -> None:
        if "opening_mm" in values:
            values.setdefault("timestamp_ns", time.time_ns())
        values["log"] = list(log[-12:])
        event_conn.send(("snapshot", values))

    def close_port() -> None:
        nonlocal serial_port
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        serial_port = None

    def status() -> None:
        nonlocal last_status
        if serial_port is None:
            return
        enabled, fault, holding, closed, _speed, _torque = _read_status(serial_port, int(config["slave_id"]))
        measured_ns = time.time_ns()
        raw_opening = (int(config["fully_closed_position_units"]) - closed) * float(config["position_unit_mm"])
        opening, opening_source = measured_opening_mm(closed, config)
        # The physical feedback dithers by one count at rest.  Do not turn
        # that harmless ±0.01 mm noise into hundreds of log lines.
        current_status = (enabled, fault, holding, closed // 5)
        if current_status != last_status:
            add_log(
                f"STATUS enable={enabled} fault={fault} holding={holding} "
                f"closing_units={closed} opening_mm={opening:.2f} "
                f"raw_opening_mm={raw_opening:.2f} source={opening_source}"
            )
            last_status = current_status
        emit(
            connected=True,
            enable_status=enabled,
            fault=fault,
            holding_status=holding,
            closing_units=closed,
            opening_mm=opening,
            opening_source=opening_source,
            timestamp_ns=measured_ns,
            in_motion=holding == 1,
            error="",
        )

    def poll_status() -> float:
        """Try one feedback read and return the next safe poll delay.

        Some MISUMI USB adapters acknowledge write commands but do not answer
        the optional 0x1194 read at all.  Repeating a timeout-length read at
        the nominal rate floods the half-duplex link and can then make a real
        open/close acknowledgement miss its short reply window.  Back off
        aggressively after failures; a successful read immediately restores
        the configured telemetry rate.
        """

        nonlocal consecutive_status_errors
        try:
            status()
        except Exception as error:
            consecutive_status_errors += 1
            # Keep the existing last-good/command-acknowledged opening.  The
            # error is diagnostic only and must not erase it.
            if consecutive_status_errors == 1 or consecutive_status_errors % 10 == 0:
                add_log(
                    f"ERROR telemetry failures={consecutive_status_errors} detail={error}"
                )
            emit(connected=True, error=f"telemetry: {error}")
            return min(2.0, 0.25 * (2 ** min(consecutive_status_errors - 1, 3)))
        consecutive_status_errors = 0
        return poll_interval

    try:
        while True:
            now = time.monotonic()
            if command_conn.poll():
                command = command_conn.recv()
                name = command[0]
                if name == "shutdown":
                    return
                try:
                    if name == "connect":
                        if serial_port is None:
                            serial = _load_pyserial()

                            serial_port = serial.Serial(
                                port=str(config["port"]), baudrate=int(config["baudrate"]),
                                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE, timeout=float(config["timeout_s"]),
                                write_timeout=float(config["timeout_s"]),
                            )
                        add_log(f"CONNECTED port={config['port']} baud={config['baudrate']}")
                        last_poll = time.monotonic() + poll_status()
                    elif name == "disconnect":
                        close_port()
                        add_log("DISCONNECTED")
                        emit(connected=False, error="")
                    elif name in ("enable", "open", "close", "move", "soft_stop"):
                        if serial_port is None:
                            raise RuntimeError("夹爪未连接；先点击“连接夹爪”")
                        if name == "enable":
                            request, response = _write_single_register(
                                serial_port, slave_id=int(config["slave_id"]), address=0x0FA0,
                                value=int(config["enable_value"]),
                            )
                            label = "夹爪使能请求"
                        elif name in ("open", "close", "move"):
                            if len(command) != 4:
                                raise ValueError("motion needs position, speed, and force")
                            position_units = int(command[1])
                            speed_percent = int(command[2])
                            force_percent = int(command[3])
                            max_position = int(config["fully_closed_position_units"])
                            if not 0 <= position_units <= max_position:
                                raise ValueError(f"closing position outside [0, {max_position}]")
                            if not 1 <= speed_percent <= 100:
                                raise ValueError("speed must be within 1--100%")
                            if not 20 <= force_percent <= 100:
                                raise ValueError("force must be within 20--100%")
                            request, response, acknowledged = _write_immediate_motion(
                                serial_port, slave_id=int(config["slave_id"]), position_units=position_units,
                                speed_percent=speed_percent, force_percent=force_percent,
                                enable_value=int(config["enable_value"]),
                            )
                            closing_mm = position_units * float(config["position_unit_mm"])
                            action_label = {
                                "open": "打开",
                                "close": "闭合",
                                "move": "移动至目标开口",
                            }[name]
                            label = (
                                f"夹爪{action_label}"
                                f"（闭合量 {closing_mm:.2f} mm，速度 {speed_percent}%，力 {force_percent}%）"
                            )
                        else:
                            request, response = _write_single_register(
                                serial_port, slave_id=int(config["slave_id"]), address=0x0FBE, value=1,
                            )
                            label = "夹爪软停止（非安全急停）"
                        if name in ("open", "close", "move"):
                            reply_label = (
                                f"ACK={response.hex(' ')}"
                                if acknowledged
                                else f"NO_ACK response={response.hex(' ') or '--'}"
                            )
                        else:
                            reply_label = f"ACK={response.hex(' ')}"
                        add_log(f"COMMAND {label} TX={request.hex(' ')} {reply_label}")
                        if name in ("open", "close", "move"):
                            # The complete command frame was written.  Preserve
                            # its requested state even when this USB adapter
                            # drops both the ACK and optional feedback frame.
                            # The source label remains explicit so downstream
                            # audits never confuse it with measured telemetry.
                            opening = (
                                int(config["fully_closed_position_units"]) - position_units
                            ) * float(config["position_unit_mm"])
                            emit(
                                connected=True,
                                closing_units=position_units,
                                opening_mm=opening,
                                opening_source=(
                                    "command_acknowledged"
                                    if acknowledged
                                    else "command_sent_unacknowledged"
                                ),
                                in_motion=True,
                                error=(
                                    ""
                                    if acknowledged
                                    else "command sent; MISUMI ACK unavailable"
                                ),
                            )
                        last_poll = time.monotonic() + poll_status()
                    else:
                        raise RuntimeError(f"unsupported gripper command: {name}")
                except Exception as error:
                    add_log(f"ERROR command={name} detail={error}")
                    emit(connected=serial_port is not None, error=f"{name}: {error}")
                continue
            if serial_port is not None and now >= last_poll:
                last_poll = now + poll_status()
            time.sleep(0.005)
    except (EOFError, BrokenPipeError, OSError):
        pass
    finally:
        close_port()
        try:
            event_conn.close()
        except Exception:
            pass


class MisumiGripperController:
    """Parent-side, manually-triggered handle for the dedicated COM6 worker."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = dict(config)
        self._snapshot = MisumiGripperSnapshot()
        self._snapshot_lock = threading.Lock()
        context = mp.get_context("spawn")
        command_recv, command_send = context.Pipe(duplex=False)
        event_recv, event_send = context.Pipe(duplex=False)
        self._process = context.Process(
            target=_worker_main, args=(self.config, command_recv, event_send),
            name="misumi-gripper-worker", daemon=True,
        )
        self._process.start()
        command_recv.close()
        event_send.close()
        self._command, self._event = command_send, event_recv

    def _send(self, name: str, *values: int) -> None:
        try:
            self._command.send((name, *values))
        except (BrokenPipeError, EOFError, OSError):
            with self._snapshot_lock:
                self._snapshot.error = "MISUMI worker is not running"

    def connect(self) -> None: self._send("connect")
    def disconnect(self) -> None: self._send("disconnect")
    def enable(self) -> None: self._send("enable")
    def open(self, *, speed_percent: int | None = None, force_percent: int | None = None) -> None:
        self._send(
            "open",
            int(self.config.get("open_position_units", 0)),
            int(self.config.get("speed_percent", 50) if speed_percent is None else speed_percent),
            int(self.config.get("force_percent", 80) if force_percent is None else force_percent),
        )

    def close(self, *, closing_units: int | None = None, speed_percent: int | None = None, force_percent: int | None = None) -> None:
        self._send(
            "close",
            int(self.config.get("close_position_units", 5000) if closing_units is None else closing_units),
            int(self.config.get("speed_percent", 50) if speed_percent is None else speed_percent),
            int(self.config.get("force_percent", 80) if force_percent is None else force_percent),
        )

    def move_to_opening_mm(
        self,
        opening_mm: float,
        *,
        speed_percent: int | None = None,
        force_percent: int | None = None,
    ) -> None:
        unit_mm = float(self.config.get("position_unit_mm", 0.01))
        fully_closed = int(self.config.get("fully_closed_position_units", 5000))
        max_opening_mm = fully_closed * unit_mm
        target_opening = min(max(float(opening_mm), 0.0), max_opening_mm)
        closing_units = round(fully_closed - target_opening / unit_mm)
        self._send(
            "move",
            int(closing_units),
            int(self.config.get("speed_percent", 50) if speed_percent is None else speed_percent),
            int(self.config.get("force_percent", 80) if force_percent is None else force_percent),
        )

    def soft_stop(self) -> None: self._send("soft_stop")

    def get_snapshot(self) -> MisumiGripperSnapshot:
        with self._snapshot_lock:
            if not hasattr(self, "_feedback_history"):
                self._feedback_history: deque[MisumiGripperSnapshot] = deque(maxlen=512)
            try:
                while self._event.poll():
                    event = self._event.recv()
                    if event and event[0] == "snapshot":
                        for key, value in event[1].items():
                            if hasattr(self._snapshot, key):
                                setattr(self._snapshot, key, value)
                        if event[1].get("timestamp_ns") is not None:
                            self._feedback_history.append(replace(self._snapshot, log=[]))
                        elif event[1].get("connected") is False:
                            self._feedback_history.clear()
            except (BrokenPipeError, EOFError, OSError):
                self._snapshot.connected = False
                self._snapshot.error = "MISUMI worker stopped"
            return MisumiGripperSnapshot(
                timestamp_ns=self._snapshot.timestamp_ns,
                connected=self._snapshot.connected,
                enable_status=self._snapshot.enable_status,
                fault=self._snapshot.fault,
                holding_status=self._snapshot.holding_status,
                closing_units=self._snapshot.closing_units,
                opening_mm=self._snapshot.opening_mm,
                opening_source=self._snapshot.opening_source,
                in_motion=self._snapshot.in_motion,
                error=self._snapshot.error,
                log=list(self._snapshot.log),
            )

    def snapshot_at(self, timestamp_ns: int) -> MisumiGripperSnapshot | None:
        """Last known feedback at a robot sample; never borrow a future close."""
        with self._snapshot_lock:
            return next((value for value in reversed(getattr(self, "_feedback_history", ()))
                         if value.timestamp_ns is not None and value.timestamp_ns <= timestamp_ns), None)

    def shutdown(self) -> None:
        self._send("shutdown")
        self._process.join(1.0)
        if self._process.is_alive():
            self._process.terminate()
        for connection in (self._command, self._event):
            try:
                connection.close()
            except Exception:
                pass
