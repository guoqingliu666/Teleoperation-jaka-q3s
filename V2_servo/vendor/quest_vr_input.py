"""Quest 3S 的纯输入层：只收 UDP，不拥有 JAKA、相机或夹爪。

Unity 把头显/左右手柄 JSON 发到本机 5005。本模块负责三件事：校验数据、从多个
Unity Player 中锁定一个有效来源、把 Quest 坐标变化转换为可配置的 JAKA XYZ 方向。
它不导入厂商 SDK，因此 VR 数据异常不会绕过遥操作界面的 ARM 和安全状态机。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import queue
import socket
import threading
import time
from typing import Any


HAND_DIRECTIONS = {
    "forward": (0.0, 0.0, 1.0),
    "backward": (0.0, 0.0, -1.0),
    "left": (-1.0, 0.0, 0.0),
    "right": (1.0, 0.0, 0.0),
    "up": (0.0, 1.0, 0.0),
    "down": (0.0, -1.0, 0.0),
}
JAKA_DIRECTIONS = ("X+", "X-", "Y+", "Y-", "Z+", "Z-")


def axis_map(mapping: dict[str, str]) -> tuple[tuple[float, float, float], ...]:
    """生成 Unity XYZ → JAKA XYZ 的 3×3 方向矩阵，并拒绝重复/不成对映射。"""

    if set(mapping) != set(HAND_DIRECTIONS) or set(mapping.values()) != set(JAKA_DIRECTIONS):
        raise ValueError("Quest direction mapping must use every hand/JAKA direction exactly once")
    for positive, negative in (("forward", "backward"), ("right", "left"), ("up", "down")):
        output = mapping[positive]
        inverse = output[0] + ("-" if output[1] == "+" else "+")
        if mapping[negative] != inverse:
            raise ValueError(f"{positive}/{negative} must map to opposite JAKA directions")
    rows = [[0.0, 0.0, 0.0] for _ in range(3)]
    for hand_name, hand_vector in HAND_DIRECTIONS.items():
        output = mapping[hand_name]
        axis = "XYZ".index(output[0])
        sign = 1.0 if output[1] == "+" else -1.0
        for column, value in enumerate(hand_vector):
            rows[axis][column] += sign * value
    # Each physical axis is described twice (for example, `right` and
    # `left`).  Averaging the two signed descriptions keeps a 1 cm hand move
    # equal to a 1 cm mapped target move, rather than accidentally doubling it.
    return tuple(tuple(value * 0.5 for value in row) for row in rows)


def matvec(matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(sum(row[index] * vector[index] for index in range(3)) for row in matrix)  # type: ignore[return-value]


def matmul(a: tuple[tuple[float, float, float], ...], b: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def transpose(a: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def quaternion(value: Any) -> tuple[float, float, float, float]:
    if isinstance(value, dict):
        raw = (value.get("x"), value.get("y"), value.get("z"), value.get("w"))
    elif isinstance(value, (list, tuple)) and len(value) == 4:
        raw = tuple(value)
    else:
        raise ValueError("right.rotation_xyzw must contain XYZW")
    q = tuple(float(item) for item in raw)
    if not all(math.isfinite(item) for item in q):
        raise ValueError("rotation_xyzw contains NaN or infinity")
    norm = math.hypot(*q)
    if norm < 1e-8:
        raise ValueError("right.rotation_xyzw is zero")
    return tuple(item / norm for item in q)  # type: ignore[return-value]


def quaternion_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return -q[0], -q[1], -q[2], q[3]


def quaternion_multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return quaternion((aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz))


def quaternion_matrix(q: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = q
    return ((1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)), (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)), (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)))


def rpy_matrix(rpy: tuple[float, float, float]) -> tuple[tuple[float, float, float], ...]:
    rx, ry, rz = rpy
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    return ((cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx), (sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx), (-sy, cy*sx, cy*cx))


def matrix_rpy(m: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float]:
    ry = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(math.cos(ry)) > 1e-7:
        return math.atan2(m[2][1], m[2][2]), ry, math.atan2(m[1][0], m[0][0])
    return math.atan2(-m[1][2], m[1][1]), ry, 0.0


def scaled_rotation(m: tuple[tuple[float, float, float], ...], scale: float) -> tuple[tuple[float, float, float], ...]:
    """Scale a bounded rotation through axis-angle for opt-in hand rotation."""
    angle = math.acos(max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0)))
    if angle < 1e-8:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    denom = 2.0 * math.sin(angle)
    if math.pi - angle < 1e-6:
        # At 180 degrees the antisymmetric terms vanish. Recover the axis
        # from the largest diagonal instead of dividing by sin(pi).
        index = max(range(3), key=lambda i: m[i][i])
        axis = [0.0, 0.0, 0.0]
        axis[index] = math.sqrt(max(0.0, (m[index][index] + 1.0) / 2.0))
        for j in range(3):
            if j != index:
                axis[j] = (m[index][j] + m[j][index]) / (4.0 * axis[index])
        norm = math.sqrt(sum(v*v for v in axis))
        x, y, z = (v / norm for v in axis)
    else:
        x, y, z = ((m[2][1] - m[1][2]) / denom, (m[0][2] - m[2][0]) / denom, (m[1][0] - m[0][1]) / denom)
    a, c, s, t = angle * float(scale), math.cos(angle * float(scale)), math.sin(angle * float(scale)), 1.0 - math.cos(angle * float(scale))
    return ((t*x*x+c, t*x*y-s*z, t*x*z+s*y), (t*x*y+s*z, t*y*y+c, t*y*z-s*x), (t*x*z-s*y, t*y*z+s*x, t*z*z+c))


def rotation_angle_rad(m: tuple[tuple[float, float, float], ...]) -> float:
    """Return the principal angle of a 3x3 rotation matrix in radians."""

    return math.acos(max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0)))


class ContinuousRotation:
    """Scale consecutive hand rotations, not a wrapped grip-relative angle."""

    def __init__(self) -> None:
        self.previous = rpy_matrix((0.0, 0.0, 0.0))
        self.target = self.previous

    def update(self, relative, sensitivity: float):
        step = matmul(relative, transpose(self.previous))
        self.previous = relative
        self.target = matmul(scaled_rotation(step, sensitivity), self.target)
        return self.target


def limit_rotation_step(
    current: tuple[tuple[float, float, float], ...],
    requested: tuple[tuple[float, float, float], ...],
    max_step_rad: float,
) -> tuple[tuple[float, float, float], ...]:
    """Move from ``current`` toward ``requested`` by at most one angular step.

    This is the rotational counterpart of a Cartesian linear speed limiter:
    it never drops a fast hand-pose frame.  Instead it advances the target by
    the allowed fraction, preventing a later frame from becoming a large jump.
    """

    relative = matmul(requested, transpose(current))
    angle = rotation_angle_rad(relative)
    if angle <= 1e-9 or max_step_rad >= angle:
        return requested
    if max_step_rad <= 0.0:
        return current
    return matmul(scaled_rotation(relative, max_step_rad / angle), current)


@dataclass(frozen=True)
class QuestFrame:
    connected: bool
    tracked: bool
    valid: bool
    position_m: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]
    rotation_valid: bool
    head_rotation_xyzw: tuple[float, float, float, float]
    head_rotation_valid: bool
    grip: float
    trigger: float
    thumbstick_click: bool
    button_a: bool
    button_b: bool
    button_x: bool
    button_y: bool
    received_time_ns: int
    received_s: float
    raw_packet: dict[str, Any]
    udp_source: str | None


class QuestUdpReceiver:
    """只保留最新帧的 UDP 接收器；与机器人软件完全解耦。

    可能同时残留编辑器和多个 Player。只有右手 ``connected+tracked+valid`` 的来源
    才能取得锁；已锁来源失追踪超过 1 秒后，新的有效来源可接管。
    """

    def __init__(self, host: str, port: int) -> None:
        self._queue: queue.Queue[QuestFrame] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self.error: str | None = None
        self.source_endpoint: str | None = None
        self.ignored_other_source_packets = 0
        self.ignored_untracked_before_lock = 0
        self._thread = threading.Thread(target=self._run, args=(host, int(port)), daemon=True, name="quest-udp-input")
        self._thread.start()

    @staticmethod
    def _frame(payload: dict[str, Any], udp_source: str | None = None) -> QuestFrame:
        """把 Unity JSON 校验成一帧手柄数据；缺失/非有限值不进入控制链。"""
        right = payload.get("right")
        if not isinstance(right, dict):
            raise ValueError("UDP JSON has no right-controller object")
        left = payload.get("left")
        if not isinstance(left, dict):
            left = {}
        raw_position = right.get("position_m")
        if isinstance(raw_position, dict):
            position = tuple(float(raw_position[axis]) for axis in ("x", "y", "z"))
        elif isinstance(raw_position, (list, tuple)) and len(raw_position) == 3:
            position = tuple(float(value) for value in raw_position)
        else:
            raise ValueError("right.position_m must contain XYZ")
        if not all(math.isfinite(value) for value in position):
            raise ValueError("right.position_m contains NaN or infinity")
        grip = float(right.get("grip", 0.0))
        trigger = float(right.get("trigger", 0.0))
        if not all(math.isfinite(value) and 0 <= value <= 1 for value in (grip, trigger)):
            raise ValueError("Grip/Trigger must be finite values in [0, 1]")
        try:
            rotation = quaternion(right.get("rotation_xyzw"))
            rotation_valid = True
        except (TypeError, ValueError):
            # 有些 Unity/OpenXR 配置在姿态追踪尚未就绪时会发送全零四元数。
            # 此时只标记姿态无效；当前 XYZ 位置跟随不因此被误判成姿态可用。
            rotation = (0.0, 0.0, 0.0, 1.0)
            rotation_valid = False
        head = payload.get("head")
        try:
            if not isinstance(head, dict) or not bool(head.get("connected", False)) or not bool(head.get("pose_valid", False)):
                raise ValueError("head pose unavailable")
            head_rotation = quaternion(head.get("rotation_xyzw"))
            head_rotation_valid = True
        except (TypeError, ValueError):
            # 兼容旧版 Unity 数据：缺少头显姿态时仍可解析手柄，
            # 但不允许把默认四元数当作已经锁定的头显朝向。
            head_rotation = (0.0, 0.0, 0.0, 1.0)
            head_rotation_valid = False
        return QuestFrame(
            connected=bool(right.get("connected", False)),
            tracked=bool(right.get("tracked", False)),
            valid=bool(right.get("pose_valid", False)),
            position_m=position,  # type: ignore[arg-type]
            rotation_xyzw=rotation,
            rotation_valid=rotation_valid,
            head_rotation_xyzw=head_rotation,
            head_rotation_valid=head_rotation_valid,
            grip=grip,
            trigger=trigger,
            thumbstick_click=bool(right.get("thumbstick_click", False)),
            # Quest 按键约定：A/B 在右手，X/Y 在左手；
            # 旧 UDP 包没有这些字段时按“未按下”处理。
            button_a=bool(right.get("primary_button", False)),
            button_b=bool(right.get("secondary_button", False)),
            button_x=bool(left.get("primary_button", False)),
            button_y=bool(left.get("secondary_button", False)),
            received_time_ns=time.time_ns(),
            received_s=time.monotonic(),
            # 保留 Unity 的头显、左右手、速度、摇杆和序列号，供只读数据采集。
            # json.loads 每次生成新对象；后续代码不修改它。
            raw_packet=payload,
            udp_source=udp_source,
        )

    def _run(self, host: str, port: int) -> None:
        """后台收包，只接受当前锁定来源；队列满时丢旧帧、保留最新帧。"""
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.settimeout(0.20)
        locked_address = None
        locked_last_s = float("-inf")
        try:
            receiver.bind((host, port))
            while not self._stop.is_set():
                try:
                    data, address = receiver.recvfrom(65535)
                    now_s = time.monotonic()
                    source = f"{address[0]}:{address[1]}"
                    frame = self._frame(json.loads(data.decode("utf-8")), source)
                    if locked_address is None:
                        # 多个 Unity/Player 可同时发往 5005。只由有效右手流取得锁，
                        # 避免先到的无 XR 桌面实例污染整条记录。
                        if not (frame.connected and frame.tracked and frame.valid):
                            self.ignored_untracked_before_lock += 1
                            continue
                        locked_address = address
                        locked_last_s = now_s
                        self.source_endpoint = source
                    elif address != locked_address:
                        if now_s - locked_last_s > 1.0 and frame.connected and frame.tracked and frame.valid:
                            locked_address = address
                            locked_last_s = now_s
                            self.source_endpoint = source
                        else:
                            self.ignored_other_source_packets += 1
                            continue
                    else:
                        # 只把“仍然有效的追踪帧”当作来源存活证明。旧实现即使某个
                        # Player 已丢失头显/手柄，仍会用无效 UDP 永久续租来源锁，
                        # 从而挡住后来重新打开、且追踪正常的 Player。这正是有时
                        # 明明 VR 内看得到手柄，遥操作窗口却一直 tracked=False 的原因。
                        if frame.connected and frame.tracked and frame.valid:
                            locked_last_s = now_s
                    try:
                        self._queue.put_nowait(frame)
                    except queue.Full:
                        try:
                            self._queue.get_nowait()
                        except queue.Empty:
                            pass
                        self._queue.put_nowait(frame)
                except socket.timeout:
                    continue
                except Exception as error:
                    self.error = str(error)
        except Exception as error:
            self.error = str(error)
        finally:
            self.source_endpoint = None
            receiver.close()

    def latest(self) -> QuestFrame | None:
        """取出当前最新帧并清空旧帧，避免机器人追赶积压的历史轨迹。"""
        latest: QuestFrame | None = None
        while True:
            try:
                latest = self._queue.get_nowait()
            except queue.Empty:
                return latest

    def close(self) -> None:
        """结束 UDP 接收线程，不涉及 JAKA SDK。"""
        self._stop.set()
        self._thread.join(0.5)


class RelativeQuestTracker:
    """把双按钮离合、相对位置和相对旋转转换为与硬件无关的事件流。"""

    def __init__(self, mapping: dict[str, str], *, grip_on: float, grip_off: float, trigger_on: float, trigger_off: float) -> None:
        self._axis_map = axis_map(mapping)
        self.grip_on, self.grip_off = float(grip_on), float(grip_off)
        self.trigger_on, self.trigger_off = float(trigger_on), float(trigger_off)
        self.reference: tuple[float, float, float] | None = None
        self.reference_rotation: tuple[float, float, float, float] | None = None
        self._heading_frame: tuple[tuple[float, float, float], ...] = (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
        self.trigger_active = False
        self.stick_active = False
        self._button_active = {"a": False, "b": False, "x": False, "y": False}

    def reset_grip(self) -> None:
        self.reference = None
        self.reference_rotation = None

    def lock_heading(self, head_rotation_xyzw: tuple[float, float, float, float]) -> float:
        """Make the HMD's current horizontal forward direction local +Z.

        Pitch and roll are intentionally discarded, so looking up/down does
        not tilt the robot's translation axes.  Returns the captured yaw in
        degrees for diagnostics.
        """

        rotation = quaternion_matrix(head_rotation_xyzw)
        forward_x, forward_z = rotation[0][2], rotation[2][2]
        length = math.hypot(forward_x, forward_z)
        if length < 1e-6:
            raise ValueError("head forward direction is vertical")
        forward = (forward_x / length, 0.0, forward_z / length)
        right = (forward[2], 0.0, -forward[0])
        self._heading_frame = (right, (0.0, 1.0, 0.0), forward)
        self.reset_grip()
        return math.degrees(math.atan2(forward[0], forward[2]))

    def update(self, frame: QuestFrame, sensitivity: float) -> list[tuple[str, Any]]:
        if not (frame.connected and frame.tracked and frame.valid):
            was_active = self.reference is not None
            self.reference = None
            self.reference_rotation = None
            self.trigger_active = False
            self.stick_active = False
            self._button_active = {"a": False, "b": False, "x": False, "y": False}
            return [("grip_stop", None)] if was_active else []
        events: list[tuple[str, Any]] = []
        for name, pressed, event_name in (
            ("a", frame.button_a, "record_start"),
            ("x", frame.button_x, "record_success"),
            ("y", frame.button_y, "record_cancel"),
            ("b", frame.button_b, "restore_saved_position"),
        ):
            if pressed and not self._button_active[name]:
                self._button_active[name] = True
                events.append((event_name, None))
            elif not pressed:
                self._button_active[name] = False
        if frame.thumbstick_click and not self.stick_active:
            self.stick_active = True
            events.append(("stick_click", None))
        elif not frame.thumbstick_click:
            self.stick_active = False
        if not self.trigger_active and frame.trigger >= self.trigger_on:
            self.trigger_active = True
            events.append(("gripper_close", None))
        elif self.trigger_active and frame.trigger <= self.trigger_off:
            self.trigger_active = False
            events.append(("gripper_open", None))
        if self.reference is None and frame.grip >= self.grip_on:
            self.reference = frame.position_m
            self.reference_rotation = frame.rotation_xyzw
            events.append(("grip_start", None))
        elif self.reference is not None and frame.grip <= self.grip_off:
            self.reference = None
            self.reference_rotation = None
            events.append(("grip_stop", None))
        if self.reference is not None:
            delta = tuple(frame.position_m[index] - self.reference[index] for index in range(3))
            heading_delta = matvec(self._heading_frame, delta)
            mapped_mm = tuple(value * 1000.0 * float(sensitivity) for value in matvec(self._axis_map, heading_delta))
            mapped_rotation = None
            if frame.rotation_valid and self.reference_rotation is not None:
                delta_rotation = quaternion_matrix(quaternion_multiply(frame.rotation_xyzw, quaternion_conjugate(self.reference_rotation)))
                delta_rotation = matmul(matmul(self._heading_frame, delta_rotation), transpose(self._heading_frame))
                mapped_rotation = matmul(matmul(self._axis_map, delta_rotation), transpose(self._axis_map))
            # One controller frame must become exactly one complete TCP
            # target.  Sending translation and rotation separately makes the
            # RPY alternate between the reference and rotated orientations.
            events.append(("pose_delta", (mapped_mm, mapped_rotation)))
        return events
