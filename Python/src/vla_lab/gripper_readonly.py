"""Read-only gripper telemetry composition and a hardware-free simulator.

This module intentionally exposes no serial, socket, or actuator command.  A
real gripper driver can later implement ``read_telemetry`` while motion commands
remain in a separate, explicitly safety-gated adapter.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace
from typing import Protocol

from .monitor_types import RobotTelemetry


@dataclass(frozen=True)
class GripperTelemetry:
    timestamp_ns: int
    opening_mm: float
    in_motion: bool
    source: str
    status_code: int | None = None
    fault_code: int | None = None
    position_units: int | None = None
    enable_status_code: int | None = None

    def __post_init__(self) -> None:
        if self.timestamp_ns <= 0:
            raise ValueError("gripper timestamp must be positive")
        if not math.isfinite(self.opening_mm) or self.opening_mm < 0:
            raise ValueError("gripper opening must be finite and non-negative")


class GripperTelemetrySource(Protocol):
    def read_telemetry(self) -> GripperTelemetry: ...


def modbus_rtu_crc(payload: bytes) -> bytes:
    """Return the Modbus RTU CRC in the protocol's low-byte-first order."""

    value = 0xFFFF
    for byte in payload:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xA001 if value & 1 else value >> 1
    return value.to_bytes(2, byteorder="little")


class MisumiModbusReadOnlyGripper:
    """Read MISUMI E-ELGPS(D) feedback through a USB RS-485 adapter.

    This adapter deliberately implements only Modbus read function codes.  It
    has no enable, position, force, velocity, trigger, or stop methods.
    """

    _STATUS_ADDRESS = 0x1194
    _STATUS_COUNT = 6

    def __init__(
        self,
        *,
        port: str,
        slave_id: int = 9,
        baudrate: int = 115200,
        timeout_s: float = 0.12,
        read_function_code: int = 3,
        fully_closed_position_units: int = 5000,
        position_unit_mm: float = 0.01,
    ) -> None:
        if not port.strip() or not 1 <= slave_id <= 247:
            raise ValueError("invalid MISUMI serial port or slave id")
        if baudrate <= 0 or timeout_s <= 0 or read_function_code not in (3, 4):
            raise ValueError("invalid MISUMI read-only serial settings")
        if fully_closed_position_units <= 0 or position_unit_mm <= 0:
            raise ValueError("invalid MISUMI position scaling")
        self.port = port
        self.slave_id = int(slave_id)
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.read_function_code = int(read_function_code)
        self.fully_closed_position_units = int(fully_closed_position_units)
        self.position_unit_mm = float(position_unit_mm)

    def _request(self) -> bytes:
        payload = bytes((self.slave_id, self.read_function_code)) + self._STATUS_ADDRESS.to_bytes(2, "big") + self._STATUS_COUNT.to_bytes(2, "big")
        return payload + modbus_rtu_crc(payload)

    def _read_response(self, serial_port: object) -> bytes:
        serial_port.reset_input_buffer()
        written = serial_port.write(self._request())
        if written != len(self._request()):
            raise RuntimeError("MISUMI status request was not fully written")
        serial_port.flush()
        head = serial_port.read(3)
        if len(head) != 3:
            raise TimeoutError("MISUMI status response timed out")
        if head[0] != self.slave_id:
            raise RuntimeError(f"MISUMI unexpected slave response: {head[0]}")
        if head[1] == self.read_function_code | 0x80:
            code = serial_port.read(1)
            serial_port.read(2)
            raise RuntimeError(f"MISUMI Modbus exception {code[0] if code else 'unknown'}")
        if head[1] != self.read_function_code or head[2] != self._STATUS_COUNT * 2:
            raise RuntimeError("MISUMI malformed status response header")
        tail = serial_port.read(head[2] + 2)
        response = head + tail
        if len(tail) != head[2] + 2:
            raise TimeoutError("MISUMI status response was incomplete")
        if response[-2:] != modbus_rtu_crc(response[:-2]):
            raise RuntimeError("MISUMI status response CRC mismatch")
        return response

    def read_telemetry(self) -> GripperTelemetry:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as error:
            raise RuntimeError(
                "pyserial is required for MISUMI USB read-only telemetry; run setup_misumi_gripper_telemetry.cmd"
            ) from error
        with serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout_s,
            write_timeout=self.timeout_s,
        ) as serial_port:
            response = self._read_response(serial_port)
        values = tuple(int.from_bytes(response[index : index + 2], "big") for index in range(3, 15, 2))
        enabled, fault, holding, closed_units, _speed, _torque = values
        if closed_units > self.fully_closed_position_units:
            raise RuntimeError(
                f"MISUMI position {closed_units} exceeds configured closed travel {self.fully_closed_position_units}"
            )
        opening_mm = (self.fully_closed_position_units - closed_units) * self.position_unit_mm
        return GripperTelemetry(
            timestamp_ns=time.time_ns(),
            opening_mm=opening_mm,
            in_motion=holding == 1,
            source=f"misumi_modbus_rtu:{self.port}",
            status_code=holding,
            fault_code=fault,
            position_units=closed_units,
            enable_status_code=enabled,
        )


class MockReadOnlyGripper:
    """Deterministic open/close telemetry for tests; never touches hardware."""

    def __init__(
        self,
        *,
        minimum_mm: float = 0.0,
        maximum_mm: float = 70.0,
        cycle_s: float = 6.0,
    ) -> None:
        if minimum_mm < 0 or maximum_mm <= minimum_mm:
            raise ValueError("invalid mock gripper opening range")
        if cycle_s <= 0:
            raise ValueError("cycle_s must be positive")
        self.minimum_mm = float(minimum_mm)
        self.maximum_mm = float(maximum_mm)
        self.cycle_s = float(cycle_s)
        self._started_s = time.monotonic()

    def read_telemetry(self) -> GripperTelemetry:
        phase = ((time.monotonic() - self._started_s) / self.cycle_s) % 1.0
        # Triangular wave: fully open -> closed -> fully open.
        fraction = abs(2.0 * phase - 1.0)
        opening = self.minimum_mm + fraction * (self.maximum_mm - self.minimum_mm)
        return GripperTelemetry(
            timestamp_ns=time.time_ns(),
            opening_mm=opening,
            in_motion=not (phase < 0.01 or abs(phase - 0.5) < 0.01),
            source="mock_read_only",
        )


class RobotWithReadOnlyGripper:
    """Merge independent robot and gripper readings into one monitor sample."""

    def __init__(self, robot: object, gripper: GripperTelemetrySource) -> None:
        self.robot = robot
        self.gripper = gripper

    def read_telemetry(self) -> RobotTelemetry:
        robot = self.robot.read_telemetry()
        try:
            gripper = self.gripper.read_telemetry()
        except Exception as error:
            return replace(robot, gripper_error=str(error))
        return replace(
            robot,
            gripper_opening_mm=gripper.opening_mm,
            gripper_timestamp_ns=gripper.timestamp_ns,
            gripper_in_motion=gripper.in_motion,
            gripper_source=gripper.source,
            gripper_error=None,
            gripper_status_code=gripper.status_code,
            gripper_fault_code=gripper.fault_code,
            gripper_position_units=gripper.position_units,
            gripper_enable_status_code=gripper.enable_status_code,
        )

    def close(self) -> None:
        for source in (self.gripper, self.robot):
            close = getattr(source, "close", None)
            if close is not None:
                close()
