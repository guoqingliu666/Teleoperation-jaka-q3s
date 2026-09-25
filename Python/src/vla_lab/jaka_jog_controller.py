"""用独立子进程隔离的 JAKA S5 SDK 控制器。

官方 Windows ``jkrc`` 是编译扩展：个别同步调用会阻塞，某些版本等待控制器时还会
占住 Python GIL。仅换成工作线程不能保证 Tk 界面继续刷新，所以本模块让一个 spawn
子进程独占 JAKA 会话，GUI 只通过 Pipe 发送小命令、接收遥测。

关键安全原则：界面是“请求方”，本子进程才是最终执行方。100 cm 位移预算、速度、
Tool 1、逆解、关节速度、目标看门狗和 30 秒连续段都在这里再次检查；即使 GUI 出错或
参数文件被改大，子进程也不会越过编译在代码中的硬上限。
"""

from __future__ import annotations

import multiprocessing as mp
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from .quest_vr_input import (
    limit_rotation_step,
    matmul,
    matrix_rpy,
    rotation_angle_rad,
    rpy_matrix,
    transpose,
)


# The compiled extension does not expose the C enums; these values are taken
# from the official C++ headers (jktypes.h / JAKAZuRobot.h).
COORD_BASE = 0
COORD_JOINT = 1
COORD_TOOL = 2

MODE_ABS = 0
MODE_INCR = 1
MODE_CONTINUE = 2

# Final, non-configurable guardrails for manually approved ACT steps.  The
# policy/safety layer can be stricter, but no GUI setting can make this worker
# accept a larger model-generated command.
APPROVAL_MAX_TRANSLATION_MM = 6.0
APPROVAL_MAX_ROTATION_DEG = 2.0
APPROVAL_MAX_SPEED_MM_S = 30.0

# Joint-target ACT is deliberately executed through JAKA's servo interface,
# never through a sequence of Cartesian ``linear_move`` calls.  The model is
# sampled at 5 Hz; the worker fills each 200 ms segment with 8 ms ``servo_j``
# points, which is the controller interpolation cycle documented by JAKA.
SERVO_PERIOD_S = 0.008
SERVO_MAX_TARGET_DELTA_RAD = 0.16

# Quest teleoperation uses the same SDK worker as jogging and recording.  The
# limits are deliberately local to this worker (rather than a second bridge
# process), so only one process can own JAKA's servo session.
CARTESIAN_SERVO_MAX_RELATIVE_MM = 500.0
CARTESIAN_SERVO_MAX_SPEED_MM_S = 500.0

# First-live-motion commissioning envelope.  These values are deliberately
# compiled into the SDK worker: neither a JSON file nor the GUI can enlarge
# them.  The commissioning path is translation-only and expires quickly.
COMMISSION_MAX_RELATIVE_MM = 5.0
COMMISSION_MAX_SPEED_MM_S = 5.0
COMMISSION_TARGET_WATCHDOG_S = 0.25
COMMISSION_MAX_SESSION_S = 10.0
COMMISSION_REQUIRED_TOOL_ID = 0

# Second-stage visible demonstration envelope.  It remains a guarded,
# translation-only profile and deliberately lives beside (not in place of)
# the already-validated 5 mm commissioning profile.
SHOWCASE_MAX_RELATIVE_MM = 50.0
SHOWCASE_MAX_SPEED_MM_S = 30.0
SHOWCASE_TARGET_WATCHDOG_S = 0.25
SHOWCASE_MAX_SESSION_S = 15.0
SHOWCASE_REQUIRED_TOOL_ID = 0

# 事故前版本的工程限幅。100 cm 只是相对目标预算，不等于完整球体均可达；
# 这些数值本身也不能证明当前设备可安全运行。
ENGINEERING_MAX_RELATIVE_MM = 1000.0
ENGINEERING_MAX_LINEAR_SPEED_MM_S = 150.0
ENGINEERING_MAX_ROTATION_DEG = 45.0
ENGINEERING_MAX_ANGULAR_SPEED_DEG_S = 30.0
ENGINEERING_MAX_JOINT_SPEED_DEG_S = 30.0
ENGINEERING_TARGET_WATCHDOG_S = 0.25
ENGINEERING_MAX_SESSION_S = 30.0
# 2026-09-19 真机出现剧烈抖动、J4 伺服欠压和保护性停止。机械臂后来已由
# 厂商上位机检查，但这不能验证事故版本的逐周期 servo_p 控制链；该旧链路仍锁定。
# 离线单元测试不能证明继续发 servo_p 安全；在 SDK 工作进程硬拒绝启动。
# 这不是可以由 GUI 勾选框或命令行参数绕过的普通软件门槛。
ENGINEERING_LIVE_LOCKOUT_REASON = (
    "2026-09-19 涉险事件后已锁定真机六维遥操作：J4 欠压/保护性停止，"
    "事故版本的逐周期 servo_p 控制链未经重新验收；禁止再次 ARM 或发送运动命令"
)
# 当前灵巧手 TCP 是控制器 Tool 1。控制器侧已经保存相对 Tool 0 的轴向 +146 mm，
# 所以反馈、逆解和 servo_p 全部保持 Tool 1，Python 不能再重复叠加 146 mm。
ENGINEERING_REQUIRED_TOOL_ID = 1


# A full JAKA telemetry bundle used to synchronously query eight SDK methods
# for every frame.  On Windows this can make the pose timestamp hundreds of
# milliseconds old before it reaches the recorder.  TCP/joints are the only
# data needed at the recording cadence; status/health information is cached
# below and refreshed separately.
HEALTH_POLL_INTERVAL_S = 0.50
TOOL_ID_POLL_INTERVAL_S = 2.00

SDK_DIRECTORY_DEFAULT = "third_party/jaka_sdk"

# Public command names are the SDK method names for power, but the SDK uses
# ``enable_robot``/``disable_robot`` for the enable/disable toggle.
_SDK_METHODS = {
    "power_on": "power_on",
    "power_off": "power_off",
    "enable": "enable_robot",
    "disable": "disable_robot",
}

_POWER_MESSAGES = {
    "power_on": "已上电",
    "power_off": "已下电",
    "enable": "已使能",
    "disable": "已去使能",
}


def _load_jkrc(sdk_directory: str) -> Any:
    """Import ``jkrc`` inside the SDK worker process."""

    sdk_dir = Path(sdk_directory).resolve()
    if not sdk_dir.is_dir():
        raise RuntimeError(f"JAKA SDK directory not found: {sdk_dir}")
    if str(sdk_dir) not in sys.path:
        sys.path.insert(0, str(sdk_dir))
    if os.name == "nt":
        os.add_dll_directory(str(sdk_dir))
    import jkrc

    return jkrc


def _check(result: Any, operation: str) -> Any:
    if not isinstance(result, tuple) or not result:
        raise RuntimeError(f"unexpected SDK response for {operation}: {result!r}")
    if int(result[0]) != 0:
        raise RuntimeError(f"SDK {operation} failed: {result!r}")
    return result


def _logout(robot: Any) -> None:
    if robot is None:
        return
    try:
        robot.logout()
    except Exception:
        pass


def _read_bool(robot: Any, method: str) -> bool:
    try:
        return bool(_check(getattr(robot, method)(), method)[1])
    except Exception:
        return False


def _read_power_enable(robot: Any) -> tuple[bool, bool]:
    simple = _check(robot.get_robot_status_simple(), "get_robot_status_simple")[1]
    powered_on = bool(int(simple[2])) if len(simple) > 2 else False
    enabled = bool(int(simple[3])) if len(simple) > 3 else False
    return powered_on, enabled


def _require_enabled(robot: Any) -> None:
    powered_on, enabled = _read_power_enable(robot)
    if not powered_on:
        raise RuntimeError("机械臂尚未上电，请先点击“上电”")
    if not enabled:
        raise RuntimeError("机械臂尚未使能，请先点击“使能”")


def _read_bool_strict(robot: Any, method: str) -> bool:
    """Read a safety bit and fail closed when the SDK cannot answer."""

    return bool(_check(getattr(robot, method)(), method)[1])


def _require_commissioning_ready(robot: Any, required_tool_id: int) -> tuple[float, ...]:
    """Perform fresh, fail-closed checks immediately before enabling servo."""

    _require_enabled(robot)
    if not _read_bool_strict(robot, "is_in_pos"):
        raise RuntimeError("机械臂仍在运动，拒绝进入微动调试")
    for method, label in (
        ("is_in_estop", "急停"),
        ("is_in_collision", "碰撞"),
        ("is_on_limit", "限位"),
    ):
        if _read_bool_strict(robot, method):
            raise RuntimeError(f"机械臂处于{label}状态，拒绝进入微动调试")
    active_tool_id = int(_check(robot.get_tool_id(), "get_tool_id(commissioning)")[1])
    if active_tool_id != int(required_tool_id):
        raise RuntimeError(
            f"微动调试要求 Tool ID {int(required_tool_id)}，当前为 {active_tool_id}"
        )
    current = tuple(
        float(value)
        for value in _check(
            robot.get_actual_tcp_position(),
            "get_actual_tcp_position(commissioning)",
        )[1]
    )
    if len(current) != 6 or not all(math.isfinite(value) for value in current):
        raise RuntimeError("微动调试未取得六维有限 TCP 位姿")
    return current


def limit_commissioning_target(
    reference: tuple[float, ...],
    previous: tuple[float, ...],
    requested: tuple[float, ...],
    elapsed_s: float,
) -> tuple[float, ...]:
    """Validate and speed-limit one translation-only commissioning target."""

    return _limit_guarded_cartesian_target(
        reference,
        previous,
        requested,
        elapsed_s,
        max_relative_mm=COMMISSION_MAX_RELATIVE_MM,
        max_speed_mm_s=COMMISSION_MAX_SPEED_MM_S,
        watchdog_s=COMMISSION_TARGET_WATCHDOG_S,
        profile_name="commissioning",
    )


def limit_showcase_target(
    reference: tuple[float, ...],
    previous: tuple[float, ...],
    requested: tuple[float, ...],
    elapsed_s: float,
    *,
    max_relative_mm: float = SHOWCASE_MAX_RELATIVE_MM,
    max_speed_mm_s: float = SHOWCASE_MAX_SPEED_MM_S,
) -> tuple[float, ...]:
    """Validate and speed-limit one translation-only 5 cm showcase target."""

    if not 0.0 < float(max_relative_mm) <= SHOWCASE_MAX_RELATIVE_MM:
        raise ValueError(
            f"showcase radius must be within (0, {SHOWCASE_MAX_RELATIVE_MM:.1f}] mm"
        )
    if not 0.0 < float(max_speed_mm_s) <= SHOWCASE_MAX_SPEED_MM_S:
        raise ValueError(
            f"showcase speed must be within (0, {SHOWCASE_MAX_SPEED_MM_S:.1f}] mm/s"
        )

    return _limit_guarded_cartesian_target(
        reference,
        previous,
        requested,
        elapsed_s,
        max_relative_mm=float(max_relative_mm),
        max_speed_mm_s=float(max_speed_mm_s),
        watchdog_s=SHOWCASE_TARGET_WATCHDOG_S,
        profile_name="showcase",
    )


def _unwrap_rpy_near(values: tuple[float, float, float], reference: tuple[float, float, float]) -> tuple[float, float, float]:
    """Choose equivalent Euler values closest to the previous command."""

    result = []
    for value, near in zip(values, reference, strict=True):
        result.append(value + round((near - value) / (2.0 * math.pi)) * 2.0 * math.pi)
    return tuple(result)  # type: ignore[return-value]


def limit_engineering_target(
    reference: tuple[float, ...],
    previous: tuple[float, ...],
    requested: tuple[float, ...],
    elapsed_s: float,
    *,
    max_relative_mm: float,
    max_linear_speed_mm_s: float,
    max_rotation_deg: float,
    max_angular_speed_deg_s: float,
) -> tuple[float, ...]:
    """Saturating 6-DoF limiter for engineering teleoperation.

    Finite requests outside the selected translation/rotation envelope are
    projected onto that envelope instead of turning an ordinary boundary
    touch (including floating-point noise such as 50.008 mm) into a stopped
    servo session.  Invalid settings and non-finite data remain hard errors.
    """

    if any(len(values) != 6 for values in (reference, previous, requested)):
        raise ValueError("engineering TCP poses must contain six values")
    if not all(math.isfinite(value) for values in (reference, previous, requested) for value in values):
        raise ValueError("engineering TCP poses must be finite")
    bounds = (
        (max_relative_mm, ENGINEERING_MAX_RELATIVE_MM, "radius mm"),
        (max_linear_speed_mm_s, ENGINEERING_MAX_LINEAR_SPEED_MM_S, "linear speed mm/s"),
        (max_rotation_deg, ENGINEERING_MAX_ROTATION_DEG, "rotation deg"),
        (max_angular_speed_deg_s, ENGINEERING_MAX_ANGULAR_SPEED_DEG_S, "angular speed deg/s"),
    )
    for value, ceiling, label in bounds:
        if not 0.0 < float(value) <= ceiling:
            raise ValueError(f"engineering {label} must be within (0, {ceiling}]")

    relative_xyz = tuple(
        target - origin
        for target, origin in zip(requested[:3], reference[:3], strict=True)
    )
    distance = math.dist(relative_xyz, (0.0, 0.0, 0.0))
    if distance > max_relative_mm:
        ratio = max_relative_mm / distance
        requested_xyz = tuple(
            origin + ratio * delta
            for origin, delta in zip(reference[:3], relative_xyz, strict=True)
        )
    else:
        requested_xyz = requested[:3]
    safe_dt = max(SERVO_PERIOD_S, min(float(elapsed_s), ENGINEERING_TARGET_WATCHDOG_S))
    allowed_mm = max_linear_speed_mm_s * safe_dt
    step_mm = math.dist(requested_xyz, previous[:3])
    if step_mm <= allowed_mm or step_mm <= 1e-12:
        xyz = requested_xyz
    else:
        ratio = allowed_mm / step_mm
        xyz = tuple(a + ratio * (b - a) for a, b in zip(previous[:3], requested_xyz, strict=True))

    reference_rotation = rpy_matrix(tuple(reference[3:]))
    requested_rotation = rpy_matrix(tuple(requested[3:]))
    relative = matmul(requested_rotation, transpose(reference_rotation))
    angle = rotation_angle_rad(relative)
    max_angle = math.radians(max_rotation_deg)
    if angle > max_angle:
        requested_rotation = limit_rotation_step(reference_rotation, requested_rotation, max_angle)
    previous_rotation = rpy_matrix(tuple(previous[3:]))
    limited_rotation = limit_rotation_step(
        previous_rotation,
        requested_rotation,
        math.radians(max_angular_speed_deg_s) * safe_dt,
    )
    limited_rpy = _unwrap_rpy_near(matrix_rpy(limited_rotation), tuple(previous[3:]))
    return (*xyz, *limited_rpy)


def project_engineering_target(
    previous_pose: tuple[float, ...],
    candidate_pose: tuple[float, ...],
    previous_joints: tuple[float, ...],
    elapsed_s: float,
    max_joint_speed_deg_s: float,
    solve_ik,
    *,
    iterations: int = 10,
) -> tuple[tuple[float, ...], tuple[float, ...], float, bool, str]:
    """Accept a target or project it to the nearest tested reachable point.

    The first attempt preserves the normal joint-rate limiter.  If IK rejects
    the requested pose, a bounded line search walks from the last accepted TCP
    pose toward the request.  Holding the last accepted pose is therefore a
    recoverable workspace boundary, not a reason to tear down the GUI/UDP
    pipeline.  Servo transport and robot-health errors are still handled by
    the worker's independent hard-stop paths.
    """

    try:
        pose, joints, rate, limited = guard_engineering_joint_rate(
            previous_pose,
            candidate_pose,
            previous_joints,
            elapsed_s,
            max_joint_speed_deg_s,
            solve_ik,
        )
        return pose, joints, rate, limited, ""
    except Exception as first_error:
        low, high = 0.0, 1.0
        best_pose = previous_pose
        best_joints = previous_joints
        best_rate = 0.0
        found = False
        for _ in range(max(1, int(iterations))):
            ratio = (low + high) * 0.5
            probe = _interpolate_cartesian_pose(previous_pose, candidate_pose, ratio)
            try:
                pose, joints, rate, _limited = guard_engineering_joint_rate(
                    previous_pose,
                    probe,
                    previous_joints,
                    elapsed_s,
                    max_joint_speed_deg_s,
                    solve_ik,
                )
            except Exception:
                high = ratio
            else:
                found = True
                low = ratio
                best_pose, best_joints, best_rate = pose, joints, rate
        reason = f"目标接近不可达边界，已投影/保持：{first_error}"
        return best_pose, best_joints, best_rate, True, reason


def project_engineering_target_with_orientation_priority(
    previous_pose: tuple[float, ...],
    candidate_pose: tuple[float, ...],
    previous_joints: tuple[float, ...],
    elapsed_s: float,
    max_joint_speed_deg_s: float,
    solve_ik,
) -> tuple[tuple[float, ...], tuple[float, ...], float, bool, str]:
    """组合 6D 目标受阻时，避免越界平移把姿态一起冻结。

    完整 XYZ+RPY 目标始终是第一选择。只有完整目标被投影时，才额外尝试
    “保持当前 XYZ、追踪新姿态”；若它确实带来更大的姿态进展，就采用该结果。
    这不能把不可达位姿变成可达位姿，但能改善工作空间边缘的手柄转腕体验。
    """

    full = project_engineering_target(
        previous_pose, candidate_pose, previous_joints, elapsed_s,
        max_joint_speed_deg_s, solve_ik,
    )
    if not full[4]:
        return full
    previous_rotation = rpy_matrix(tuple(previous_pose[3:]))
    requested_rotation = rpy_matrix(tuple(candidate_pose[3:]))
    if rotation_angle_rad(matmul(requested_rotation, transpose(previous_rotation))) < 1e-7:
        return full
    orientation_only = previous_pose[:3] + candidate_pose[3:]
    rotated = project_engineering_target(
        previous_pose, orientation_only, previous_joints, elapsed_s,
        max_joint_speed_deg_s, solve_ik,
    )
    full_progress = rotation_angle_rad(
        matmul(rpy_matrix(tuple(full[0][3:])), transpose(previous_rotation))
    )
    rotated_progress = rotation_angle_rad(
        matmul(rpy_matrix(tuple(rotated[0][3:])), transpose(previous_rotation))
    )
    if rotated_progress > full_progress + math.radians(0.01):
        notice = "组合目标接近不可达边界，已保持 XYZ 并优先追踪姿态"
        if rotated[4]:
            notice += "（姿态也已投影）"
        return rotated[0], rotated[1], rotated[2], True, notice
    return full


def _interpolate_cartesian_pose(
    start: tuple[float, ...],
    target: tuple[float, ...],
    ratio: float,
) -> tuple[float, ...]:
    """Interpolate one already-safe TCP step without Euler-angle artifacts."""

    ratio = max(0.0, min(1.0, float(ratio)))
    xyz = tuple(a + ratio * (b - a) for a, b in zip(start[:3], target[:3], strict=True))
    start_rotation = rpy_matrix(tuple(start[3:]))
    target_rotation = rpy_matrix(tuple(target[3:]))
    full_angle = rotation_angle_rad(matmul(target_rotation, transpose(start_rotation)))
    rotation = limit_rotation_step(start_rotation, target_rotation, full_angle * ratio)
    rpy = _unwrap_rpy_near(matrix_rpy(rotation), tuple(start[3:]))
    return (*xyz, *rpy)


def guard_engineering_joint_rate(
    previous_pose: tuple[float, ...],
    candidate_pose: tuple[float, ...],
    previous_joints: tuple[float, ...],
    elapsed_s: float,
    max_joint_speed_deg_s: float,
    solve_ik,
) -> tuple[tuple[float, ...], tuple[float, ...], float, bool]:
    """Scale a TCP step to its joint-rate budget instead of aborting ARM.

    Returns ``(accepted_pose, accepted_joints, original_rate, limited)``.  If
    the scaled IK still indicates a discontinuity, the last safe pose is held.
    """

    if not 0.0 < float(max_joint_speed_deg_s) <= ENGINEERING_MAX_JOINT_SPEED_DEG_S:
        raise ValueError("engineering joint speed exceeds hard limit")
    safe_dt = max(SERVO_PERIOD_S, min(float(elapsed_s), ENGINEERING_TARGET_WATCHDOG_S))

    def checked_solution(pose: tuple[float, ...], reference_joints: tuple[float, ...]) -> tuple[float, ...]:
        values = tuple(float(value) for value in solve_ik(reference_joints, pose))
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise RuntimeError("Level A 逆解返回无效关节目标")
        return values

    candidate_joints = checked_solution(candidate_pose, previous_joints)

    def rate(joints: tuple[float, ...]) -> float:
        return max(
            math.degrees(abs(a - b)) / safe_dt
            for a, b in zip(joints, previous_joints, strict=True)
        )

    requested_rate = rate(candidate_joints)
    if requested_rate <= max_joint_speed_deg_s + 1e-6:
        return candidate_pose, candidate_joints, requested_rate, False

    # Leave a small numerical margin: IK is nonlinear, so an exact ratio can
    # still land a fraction above the configured limit.
    ratio = 0.95 * max_joint_speed_deg_s / requested_rate
    scaled_pose = _interpolate_cartesian_pose(previous_pose, candidate_pose, ratio)
    scaled_joints = checked_solution(scaled_pose, previous_joints)
    if rate(scaled_joints) <= max_joint_speed_deg_s * 1.02 + 1e-6:
        return scaled_pose, scaled_joints, requested_rate, True

    # A branch jump or singular solution is not a reason to chase the target.
    # Holding the last safe command is fail-closed and keeps deadman control
    # usable; an actual IK failure still raises and stops the servo.
    return previous_pose, previous_joints, requested_rate, True


def _limit_guarded_cartesian_target(
    reference: tuple[float, ...],
    previous: tuple[float, ...],
    requested: tuple[float, ...],
    elapsed_s: float,
    *,
    max_relative_mm: float,
    max_speed_mm_s: float,
    watchdog_s: float,
    profile_name: str,
) -> tuple[float, ...]:
    """Shared fail-closed limiter for fixed-envelope Cartesian profiles."""

    if any(len(values) != 6 for values in (reference, previous, requested)):
        raise ValueError(f"{profile_name} TCP poses must contain six values")
    if not all(math.isfinite(value) for values in (reference, previous, requested) for value in values):
        raise ValueError(f"{profile_name} TCP poses must be finite")
    distance = math.dist(requested[:3], reference[:3])
    if distance > float(max_relative_mm) + 1e-9:
        raise ValueError(
            f"{profile_name} target {distance:.3f} mm exceeds "
            f"{float(max_relative_mm):.1f} mm envelope"
        )
    allowed = float(max_speed_mm_s) * max(
        SERVO_PERIOD_S,
        min(float(elapsed_s), float(watchdog_s)),
    )
    delta = math.dist(requested[:3], previous[:3])
    if delta <= allowed or delta <= 1e-12:
        xyz = requested[:3]
    else:
        ratio = allowed / delta
        xyz = tuple(
            current + ratio * (target - current)
            for current, target in zip(previous[:3], requested[:3], strict=True)
        )
    return (*xyz, *reference[3:])


@dataclass
class RobotSnapshot:
    """Latest telemetry and connection state, read by the GUI thread."""

    connected: bool = False
    powered_on: bool = False
    enabled: bool = False
    moving: bool = False
    estop: bool = False
    collision: bool = False
    on_limit: bool = False
    tool_id: int | None = None
    tool_profiles: list[dict[str, Any]] = field(default_factory=list)
    timestamp_ns: int | None = None
    tcp_pose: tuple[float, ...] | None = None
    joints_rad: tuple[float, ...] | None = None
    # 这是底层 SDK 工作进程的“真实伺服已开启”确认，不是界面自行猜测。
    # GUI 只有看到它变成 True，才允许开始连续发送手柄目标。
    engineering_servo_active: bool = False
    error: str = ""
    log: list[str] = field(default_factory=list)


def _snapshot_dict(
    *,
    connected: bool,
    powered_on: bool = False,
    enabled: bool = False,
    moving: bool = False,
    estop: bool = False,
    collision: bool = False,
    on_limit: bool = False,
    tool_id: int | None = None,
    timestamp_ns: int | None = None,
    tcp_pose: tuple[float, ...] | None = None,
    joints_rad: tuple[float, ...] | None = None,
    engineering_servo_active: bool | None = None,
    error: str = "",
    message: str = "",
) -> dict[str, Any]:
    return {
        "connected": connected,
        "powered_on": powered_on,
        "enabled": enabled,
        "moving": moving,
        "estop": estop,
        "collision": collision,
        "on_limit": on_limit,
        "tool_id": tool_id,
        "timestamp_ns": timestamp_ns,
        "tcp_pose": tcp_pose,
        "joints_rad": joints_rad,
        # None 表示这条普通遥测不改变现有伺服状态；命令确认必须显式给 True/False。
        "engineering_servo_active": engineering_servo_active,
        "error": error,
        "message": message,
    }


def _poll_sdk(robot: Any, evt_conn: Any, health: dict[str, Any]) -> None:
    """Publish the fast recording telemetry and refresh slow read-only health.

    This function deliberately contains *only SDK getter calls*.  Its pose
    timestamp is captured around TCP/joint reads, not after unrelated health
    checks, so the timestamp represents the actual observation much better.
    """
    try:
        now = time.monotonic()
        # These three are the high-frequency data path used by Episode
        # recording.  Do not insert tool/limit/estop getters in this path.
        pose_start_ns = time.time_ns()
        pose = tuple(float(v) for v in _check(robot.get_actual_tcp_position(), "get_actual_tcp_position")[1])
        joints = tuple(float(v) for v in _check(robot.get_actual_joint_position(), "get_actual_joint_position")[1])
        pose_end_ns = time.time_ns()
        simple = _check(robot.get_robot_status_simple(), "get_robot_status_simple")[1]
        powered_on = bool(int(simple[2])) if len(simple) > 2 else False
        enabled = bool(int(simple[3])) if len(simple) > 3 else False
        health["powered_on"] = powered_on
        health["enabled"] = enabled
        evt_conn.send(
            (
                "snapshot",
                _snapshot_dict(
                    connected=True,
                    powered_on=powered_on,
                    enabled=enabled,
                    moving=bool(health.get("moving", False)),
                    estop=bool(health.get("estop", False)),
                    collision=bool(health.get("collision", False)),
                    on_limit=bool(health.get("on_limit", False)),
                    tool_id=health.get("tool_id"),
                    # Approximate the time at which this TCP/joint pair was
                    # acquired.  The old code timestamped it after 5 more
                    # synchronous SDK queries, creating a false camera/JAKA
                    # synchronization failure.
                    timestamp_ns=(pose_start_ns + pose_end_ns) // 2,
                    tcp_pose=pose,
                    joints_rad=joints,
                ),
            )
        )
        # The following calls are useful for the GUI, but they do not change
        # sample content.  They intentionally occur *after* dispatching the
        # fresh pose event.  A slow health getter can therefore neither alter
        # the pose timestamp nor hold the current recording frame hostage.
        if now - float(health.get("last_health_s", 0.0)) >= HEALTH_POLL_INTERVAL_S:
            health["moving"] = not _read_bool(robot, "is_in_pos")
            health["estop"] = _read_bool(robot, "is_in_estop")
            health["collision"] = _read_bool(robot, "is_in_collision")
            health["on_limit"] = _read_bool(robot, "is_on_limit")
            health["last_health_s"] = now
        if now - float(health.get("last_tool_s", 0.0)) >= TOOL_ID_POLL_INTERVAL_S:
            try:
                health["tool_id"] = int(_check(robot.get_tool_id(), "get_tool_id")[1])
            except Exception:
                pass
            health["last_tool_s"] = now
    except Exception as error:
        evt_conn.send(("snapshot", _snapshot_dict(connected=True, error=f"telemetry: {error}")))


def _read_tool_profiles(robot: Any) -> list[dict[str, Any]]:
    """Read every controller TCP slot without changing the active tool."""

    # The Windows SDK exposes get_tool_data(id), but no portable tool-list
    # method. Probe the controller's 0--31 TCP slots without changing tools.
    profiles: list[dict[str, Any]] = []
    for tool_id in range(32):
        try:
            result = _check(robot.get_tool_data(tool_id), f"get_tool_data({tool_id})")
            # SDK response: (error_code, name, [x, y, z, rx, ry, rz]).
            tcp = tuple(float(value) for value in result[2])
            if len(tcp) != 6:
                raise ValueError(f"unexpected TCP length: {len(tcp)}")
            raw_name = str(result[1]).strip() if len(result) > 1 and result[1] else ""
            # Some SDK builds return the numeric ID in result[1], not a name.
            name = raw_name if raw_name and not raw_name.lstrip("+-").isdigit() else f"Tool {tool_id}"
            profiles.append({"id": tool_id, "name": name, "tcp": tcp})
        except Exception:
            # Empty slots are rejected by some controller firmware versions.
            continue
    return profiles


def _plan_flange_leveling(
    robot: Any,
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], float]:
    """Port the safety checks of hi-2's general leveling path to Tool 0.

    hi-2 uses the same IK -> sampled forward-kinematics strategy.  Here the
    requested frame is explicitly the flange, so RX=RY=0 keeps its plane
    parallel to the base XOY plane while preserving yaw.
    """

    current_pose = tuple(float(value) for value in _check(robot.get_actual_tcp_position(), "get_actual_tcp_position(flange)")[1])
    current_joints = tuple(float(value) for value in _check(robot.get_actual_joint_position(), "get_actual_joint_position")[1])
    # A plane parallel to XOY has two possible normals.  RX=RY=0 points the
    # flange normal upward, whereas RX=+/-pi points it downward.  The previous
    # implementation always chose the upward form, making a normally downward
    # gripper flip by about 180 degrees.  Resolve all equivalent horizontal
    # candidates and retain the IK solution nearest the current joints.
    candidates = (
        (*current_pose[:3], 0.0, 0.0, current_pose[5]),
        (*current_pose[:3], math.pi, 0.0, current_pose[5]),
        (*current_pose[:3], -math.pi, 0.0, current_pose[5]),
    )
    solutions: list[tuple[float, tuple[float, ...], tuple[float, ...]]] = []
    inverse_errors: list[str] = []
    for target_pose in candidates:
        try:
            result = _check(
                robot.kine_inverse(current_joints, target_pose),
                "kine_inverse(level flange)",
            )
            target_joints = tuple(float(value) for value in result[1])
            if len(target_joints) != 6:
                raise RuntimeError("inverse kinematics returned invalid joints")
            largest_delta = max(
                abs(math.degrees(target - current))
                for current, target in zip(current_joints, target_joints, strict=True)
            )
            solutions.append((largest_delta, target_pose, target_joints))
        except Exception as error:
            inverse_errors.append(str(error))
    if not solutions:
        raise RuntimeError("no horizontal flange IK solution: " + "; ".join(inverse_errors))
    largest_joint_delta_deg, target_pose, target_joints = min(solutions, key=lambda item: item[0])
    if largest_joint_delta_deg > 60.0:
        raise RuntimeError(f"flange leveling needs {largest_joint_delta_deg:.1f} deg at one joint; manually move closer first")

    # Measured for the current workcell: table surface is base Z=-124.729 mm.
    table_z_mm = -124.729
    min_clearance_mm = 100.0
    max_deviation_mm = 80.0
    for index in range(41):
        ratio = index / 40.0
        joints = tuple(current + ratio * (target - current) for current, target in zip(current_joints, target_joints, strict=True))
        pose = tuple(float(value) for value in _check(robot.kine_forward(joints), "kine_forward(level flange path)")[1])
        clearance = pose[2] - table_z_mm
        deviation = math.sqrt(sum((pose[axis] - current_pose[axis]) ** 2 for axis in range(3)))
        if clearance < min_clearance_mm:
            raise RuntimeError(f"flange leveling path clearance {clearance:.1f} mm is below {min_clearance_mm:.1f} mm")
        if deviation > max_deviation_mm:
            raise RuntimeError(f"flange leveling path deviation {deviation:.1f} mm exceeds {max_deviation_mm:.1f} mm")
    return current_pose, target_pose, target_joints, largest_joint_delta_deg


def _sdk_worker_main(host: str, sdk_directory: str, cmd_conn: Any, evt_conn: Any, poll_interval: float) -> None:
    robot: Any = None
    last_poll = 0.0
    log_directory = Path(__file__).resolve().parents[2] / "logs"
    log_directory.mkdir(parents=True, exist_ok=True)
    log_path = log_directory / f"jaka_jog_{time.strftime('%Y%m%d_%H%M%S')}.log"
    servo_active = False
    servo_start: tuple[float, ...] | None = None
    servo_target: tuple[float, ...] | None = None
    servo_segment_started = 0.0
    servo_segment_duration = 0.20
    next_servo_tick = 0.0
    cartesian_servo_active = False
    cartesian_reference: tuple[float, ...] | None = None
    cartesian_target: tuple[float, ...] | None = None
    cartesian_last_target_s = 0.0
    next_cartesian_tick = 0.0
    cartesian_profile: str | None = None
    commissioning_started_s = 0.0
    showcase_radius_mm = SHOWCASE_MAX_RELATIVE_MM
    showcase_speed_mm_s = 20.0
    engineering_radius_mm = 50.0
    engineering_linear_speed_mm_s = 20.0
    engineering_rotation_deg = 2.0
    engineering_angular_speed_deg_s = 3.0
    engineering_joint_speed_deg_s = 3.0
    engineering_joint_target: tuple[float, ...] | None = None
    engineering_last_rate_notice_s = 0.0
    engineering_last_projection_notice_s = 0.0
    # GUI may produce targets faster than one inverse-kinematics call can
    # finish.  Keep at most one non-target command aside and collapse queued
    # pose targets to the newest one, otherwise an old-target backlog creates
    # visible lag and can keep moving after the operator has already stopped.
    pending_command: tuple[Any, ...] | None = None
    # A non-blocking joint_move can remain owned by the controller even after
    # the target has been reached.  This prevents JAKA App tool drag.  Track
    # saved-pose restores and explicitly retire that motion once the measured
    # joints have settled at the requested position.
    restore_target: tuple[float, ...] | None = None
    restore_started = 0.0
    next_restore_check = 0.0
    telemetry_health: dict[str, Any] = {
        "powered_on": False,
        "enabled": False,
        "moving": False,
        "estop": False,
        "collision": False,
        "on_limit": False,
        "tool_id": None,
        "last_health_s": 0.0,
        "last_tool_s": 0.0,
    }

    def log(message: str) -> None:
        with log_path.open("a", encoding="utf-8") as handle:
            handle.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}\n")

    def stop_commissioning(message: str, *, error: str = "") -> None:
        """Disable either fixed-envelope, translation-only servo session."""

        nonlocal cartesian_servo_active, cartesian_reference, cartesian_target
        nonlocal cartesian_profile, commissioning_started_s, engineering_joint_target
        if cartesian_profile not in ("commissioning", "showcase", "engineering"):
            return
        stopped_profile = cartesian_profile
        try:
            if robot is not None and cartesian_servo_active:
                _check(
                    robot.servo_move_enable(False, True),
                    f"servo_move_enable(False, {stopped_profile})",
                )
        except Exception as stop_error:
            log(f"ERROR {stopped_profile} stop: {stop_error!r}")
            error = error or f"{stopped_profile} stop: {stop_error}"
        cartesian_servo_active = False
        cartesian_reference = None
        cartesian_target = None
        cartesian_profile = None
        engineering_joint_target = None
        commissioning_started_s = 0.0
        evt_conn.send(
            (
                "snapshot",
                _snapshot_dict(
                    connected=robot is not None,
                    powered_on=bool(telemetry_health.get("powered_on", False)),
                    enabled=bool(telemetry_health.get("enabled", False)),
                    tool_id=telemetry_health.get("tool_id"),
                    engineering_servo_active=False if stopped_profile == "engineering" else None,
                    error=error,
                    message=message,
                ),
            )
        )

    try:
        while True:
            now = time.monotonic()
            if cartesian_servo_active and cartesian_profile in ("commissioning", "showcase", "engineering"):
                if cartesian_profile == "engineering":
                    watchdog_s = ENGINEERING_TARGET_WATCHDOG_S
                    session_s = ENGINEERING_MAX_SESSION_S
                    profile_label = "Level A 六维遥操作"
                elif cartesian_profile == "showcase":
                    watchdog_s = SHOWCASE_TARGET_WATCHDOG_S
                    session_s = SHOWCASE_MAX_SESSION_S
                    profile_label = "展示遥操作"
                else:
                    watchdog_s = COMMISSION_TARGET_WATCHDOG_S
                    session_s = COMMISSION_MAX_SESSION_S
                    profile_label = "微动"
                stale = now - cartesian_last_target_s > watchdog_s
                expired = now - commissioning_started_s > session_s
                unsafe_health = (
                    not bool(telemetry_health.get("powered_on", False))
                    or not bool(telemetry_health.get("enabled", False))
                    or bool(telemetry_health.get("estop", False))
                    or bool(telemetry_health.get("collision", False))
                    or bool(telemetry_health.get("on_limit", False))
                )
                if stale:
                    stop_commissioning(f"{profile_label}伺服已自动停止：目标数据超过 0.25 秒未刷新")
                elif expired:
                    stop_commissioning(f"{profile_label}伺服已自动停止：{session_s:.0f} 秒会话上限已到")
                elif unsafe_health:
                    stop_commissioning(f"{profile_label}伺服已自动停止：机器人安全状态不再满足")
            # Keep the servo packet cadence independent from the UI / camera
            # polling cadence.  New targets merely replace the end of the
            # current short interpolation segment below.
            if robot is not None and servo_active and now >= next_servo_tick:
                assert servo_start is not None and servo_target is not None
                ratio = min(1.0, max(0.0, (now - servo_segment_started) / servo_segment_duration))
                point = tuple(
                    start + ratio * (target - start)
                    for start, target in zip(servo_start, servo_target, strict=True)
                )
                try:
                    _check(robot.servo_j(point, MODE_ABS, 1), "servo_j")
                    next_servo_tick = now + SERVO_PERIOD_S
                except Exception as error:
                    log(f"ERROR servo_j: {error!r}")
                    try:
                        robot.servo_move_enable(False, True)
                    except Exception:
                        pass
                    servo_active = False
                    evt_conn.send(("snapshot", _snapshot_dict(connected=True, error=f"servo_j: {error}", message=f"⚠ 关节伺服已停止：{error}")))
            if robot is not None and cartesian_servo_active and now >= next_cartesian_tick:
                assert cartesian_target is not None
                try:
                    _check(robot.servo_p(cartesian_target, MODE_ABS, 1), "servo_p")
                    next_cartesian_tick = now + SERVO_PERIOD_S
                except Exception as error:
                    log(f"ERROR servo_p: {error!r}")
                    try:
                        robot.servo_move_enable(False, True)
                    except Exception:
                        pass
                    cartesian_servo_active = False
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=True, error=f"servo_p: {error}",
                        message=f"⚠ Quest 笛卡尔伺服已停止：{error}",
                    )))
            if robot is not None and restore_target is not None and now >= next_restore_check:
                next_restore_check = now + max(0.05, poll_interval)
                # Avoid treating the pre-move in-position state as completion.
                if now - restore_started >= 0.25:
                    try:
                        actual = tuple(
                            float(value)
                            for value in _check(
                                robot.get_actual_joint_position(),
                                "get_actual_joint_position(restore settle)",
                            )[1]
                        )
                        settled = (
                            len(actual) == 6
                            and _read_bool(robot, "is_in_pos")
                            and max(
                                abs(current - target)
                                for current, target in zip(actual, restore_target, strict=True)
                            ) <= 0.003
                        )
                        if settled:
                            _check(robot.motion_abort(), "motion_abort(restore settled)")
                            restore_target = None
                            evt_conn.send(
                                (
                                    "snapshot",
                                    _snapshot_dict(
                                        connected=True,
                                        message="已复原保存位置并结束运动会话；可在 JAKA App 直接开启工具拖拽",
                                    ),
                                )
                            )
                    except Exception as error:
                        log(f"ERROR restore settle check: {error!r}")
            if pending_command is not None or cmd_conn.poll():
                if pending_command is not None:
                    command = pending_command
                    pending_command = None
                else:
                    command = cmd_conn.recv()
                dropped_targets = 0
                if command[0] == "engineering_servo_target":
                    while cmd_conn.poll():
                        newer_command = cmd_conn.recv()
                        if newer_command[0] == "engineering_servo_target":
                            command = newer_command
                            dropped_targets += 1
                        else:
                            pending_command = newer_command
                            break
                name = command[0]
                log(f"COMMAND {command!r}")
                if dropped_targets:
                    log(f"TARGET_COALESCED dropped={dropped_targets}")
                command_started_s = time.perf_counter()
                if name == "shutdown":
                    if robot is not None:
                        try:
                            if servo_active or cartesian_servo_active:
                                robot.servo_move_enable(False, True)
                            _check(robot.motion_abort(), "motion_abort")
                        except Exception:
                            pass
                    return
                try:
                    if name == "login":
                        # 双击按钮或界面卡顿时可能重复触发“连接”。重复 login 会在部分
                        # 控制器固件上留下两个会话，使下一次伺服看似 ARM、实际未接管。
                        if robot is not None:
                            evt_conn.send(("snapshot", _snapshot_dict(
                                connected=True,
                                powered_on=bool(telemetry_health.get("powered_on", False)),
                                enabled=bool(telemetry_health.get("enabled", False)),
                                tool_id=telemetry_health.get("tool_id"),
                                message=f"已经连接 {host}；已忽略重复连接请求",
                            )))
                        else:
                            jkrc = _load_jkrc(sdk_directory)
                            candidate = jkrc.RC(host)
                            _check(candidate.login(), "login")
                            robot = candidate
                            evt_conn.send(("snapshot", _snapshot_dict(connected=True, message=f"connected to {host}")))
                            evt_conn.send(("tool_profiles", _read_tool_profiles(robot)))
                    elif name == "clear_error":
                        evt_conn.send(("snapshot", _snapshot_dict(
                            connected=robot is not None,
                            powered_on=bool(telemetry_health.get("powered_on", False)),
                            enabled=bool(telemetry_health.get("enabled", False)),
                            estop=bool(telemetry_health.get("estop", False)),
                            collision=bool(telemetry_health.get("collision", False)),
                            on_limit=bool(telemetry_health.get("on_limit", False)),
                            tool_id=telemetry_health.get("tool_id"),
                            message="已清除软件故障提示；请重新检查实时状态",
                        )))
                    elif name == "logout":
                        if robot is not None and (servo_active or cartesian_servo_active):
                            try:
                                robot.servo_move_enable(False, True)
                            except Exception:
                                pass
                        servo_active = cartesian_servo_active = False
                        cartesian_reference = cartesian_target = None
                        _logout(robot)
                        robot = None
                        evt_conn.send(("snapshot", _snapshot_dict(connected=False, message="disconnected")))
                    elif name in _SDK_METHODS:
                        if robot is None:
                            evt_conn.send(("snapshot", _snapshot_dict(connected=False, error="未连接，请先点击“连接”", message="⚠ 未连接")))
                        else:
                            _check(getattr(robot, _SDK_METHODS[name])(), _SDK_METHODS[name])
                            evt_conn.send(("snapshot", _snapshot_dict(connected=True, message=_POWER_MESSAGES[name])))
                    elif name == "set_tool_id":
                        _, tool_id = command
                        if robot is None:
                            evt_conn.send(("snapshot", _snapshot_dict(connected=False, error="未连接", message="⚠ 未连接")))
                        else:
                            _check(robot.set_tool_id(int(tool_id)), "set_tool_id")
                            evt_conn.send(("snapshot", _snapshot_dict(connected=True, message=f"已切换工具 ID {int(tool_id)}")))
                    elif name == "read_tool_profiles":
                        if robot is None:
                            evt_conn.send(("snapshot", _snapshot_dict(connected=False, error="not connected")))
                        else:
                            evt_conn.send(("tool_profiles", _read_tool_profiles(robot)))
                    elif name == "level_flange":
                        _, joint_speed = command
                        if robot is None:
                            raise RuntimeError("not connected")
                        if cartesian_servo_active:
                            _check(robot.servo_move_enable(False, True), "servo_move_enable(False, Quest->level)")
                            cartesian_servo_active = False
                            cartesian_reference = cartesian_target = None
                        _require_enabled(robot)
                        original_tool = int(_check(robot.get_tool_id(), "get_tool_id")[1])
                        try:
                            _check(robot.set_tool_id(0), "set_tool_id(flange)")
                            flange_pose, target_pose, target_joints, largest_delta = _plan_flange_leveling(robot)
                            log(
                                "LEVEL_PLAN "
                                f"current_pose={flange_pose!r} target_pose={target_pose!r} "
                                f"max_joint_delta_deg={largest_delta:.3f}"
                            )
                            # Unlike hi-2's CLI, this is a live GUI.  Do not
                            # block the SDK worker: Stop must remain available
                            # if the operator intervenes while it is moving.
                            _check(robot.joint_move(target_joints, 0, False, float(joint_speed)), "joint_move(level flange)")
                        finally:
                            _check(robot.set_tool_id(original_tool), "set_tool_id(restore)")
                        evt_conn.send(("snapshot", _snapshot_dict(connected=True, message="已发送法兰找平路径；可随时 STOP")))
                    elif name == "restore_joints":
                        _, joints, joint_speed = command
                        if robot is None:
                            raise RuntimeError("not connected")
                        if cartesian_servo_active:
                            _check(robot.servo_move_enable(False, True), "servo_move_enable(False, Quest->restore)")
                            cartesian_servo_active = False
                            cartesian_reference = cartesian_target = None
                        _require_enabled(robot)
                        values = tuple(float(value) for value in joints)
                        if len(values) != 6:
                            raise ValueError("saved joint position must contain J1-J6")
                        _check(robot.joint_move(values, 0, False, float(joint_speed)), "joint_move(restore saved position)")
                        restore_target = values
                        restore_started = time.monotonic()
                        next_restore_check = restore_started + 0.25
                        evt_conn.send(("snapshot", _snapshot_dict(connected=True, message="已发送复原保存位置路径；可随时 STOP")))
                    elif name == "linear_relative":
                        _, delta_xyz, delta_rpy_deg, speed_mm_s, required_tool_id = command
                        if robot is None:
                            raise RuntimeError("not connected")
                        if cartesian_servo_active:
                            _check(robot.servo_move_enable(False, True), "servo_move_enable(False, Quest->linear)")
                            cartesian_servo_active = False
                            cartesian_reference = cartesian_target = None
                        _require_enabled(robot)
                        if not _read_bool(robot, "is_in_pos"):
                            raise RuntimeError("机械臂仍在运动，拒绝新的 ACT 单步")
                        active_tool_id = int(_check(robot.get_tool_id(), "get_tool_id")[1])
                        if active_tool_id != int(required_tool_id):
                            raise RuntimeError(
                                f"ACT 单步要求 Tool ID {int(required_tool_id)}，当前为 {active_tool_id}"
                            )
                        xyz = tuple(float(value) for value in delta_xyz)
                        rpy_deg = tuple(float(value) for value in delta_rpy_deg)
                        if len(xyz) != 3 or len(rpy_deg) != 3:
                            raise ValueError("ACT relative move requires XYZ and RPY triples")
                        translation = math.dist(xyz, (0.0, 0.0, 0.0))
                        rotation = math.dist(rpy_deg, (0.0, 0.0, 0.0))
                        speed = float(speed_mm_s)
                        if translation > APPROVAL_MAX_TRANSLATION_MM + 1e-9:
                            raise ValueError(
                                f"ACT translation {translation:.3f} mm exceeds hard limit "
                                f"{APPROVAL_MAX_TRANSLATION_MM:.1f} mm"
                            )
                        if rotation > APPROVAL_MAX_ROTATION_DEG + 1e-9:
                            raise ValueError(
                                f"ACT rotation {rotation:.3f} deg exceeds hard limit "
                                f"{APPROVAL_MAX_ROTATION_DEG:.1f} deg"
                            )
                        if not 0.1 <= speed <= APPROVAL_MAX_SPEED_MM_S:
                            raise ValueError(
                                f"ACT speed must be within [0.1, {APPROVAL_MAX_SPEED_MM_S:.1f}] mm/s"
                            )
                        current = tuple(
                            float(value)
                            for value in _check(
                                robot.get_actual_tcp_position(),
                                "get_actual_tcp_position(ACT approval)",
                            )[1]
                        )
                        target = (
                            current[0] + xyz[0],
                            current[1] + xyz[1],
                            current[2] + xyz[2],
                            current[3] + math.radians(rpy_deg[0]),
                            current[4] + math.radians(rpy_deg[1]),
                            current[5] + math.radians(rpy_deg[2]),
                        )
                        _check(
                            robot.linear_move(target, MODE_ABS, False, speed),
                            "linear_move(ACT approved step)",
                        )
                        evt_conn.send(
                            (
                                "snapshot",
                                _snapshot_dict(
                                    connected=True,
                                    message=(
                                        "已发送人工确认 ACT 单步："
                                        f"XYZ={xyz!r} mm RPY={rpy_deg!r} deg"
                                    ),
                                ),
                            )
                        )
                    elif name == "servo_start":
                        if robot is None:
                            raise RuntimeError("not connected")
                        _require_enabled(robot)
                        current = tuple(float(v) for v in _check(robot.get_actual_joint_position(), "get_actual_joint_position(servo start)")[1])
                        if len(current) != 6:
                            raise RuntimeError(f"expected six actual joints, got {len(current)}")
                        if cartesian_servo_active:
                            _check(robot.servo_move_enable(False, True), "servo_move_enable(False, cartesian->joint)")
                            cartesian_servo_active = False
                        _check(robot.servo_move_enable(True, True), "servo_move_enable(True)")
                        servo_active = True
                        restore_target = None
                        servo_start = current
                        servo_target = current
                        servo_segment_started = time.monotonic()
                        next_servo_tick = servo_segment_started
                        evt_conn.send(("snapshot", _snapshot_dict(connected=True, message="ACT 关节伺服已就绪；等待模型目标")))
                    elif name == "servo_target":
                        _, values, duration_s = command
                        if robot is None or not servo_active or servo_target is None:
                            raise RuntimeError("joint servo is not active")
                        target = tuple(float(value) for value in values)
                        if len(target) != 6:
                            raise ValueError("servo joint target must contain J1-J6")
                        maximum_delta = max(abs(target_value - current_value) for target_value, current_value in zip(target, servo_target, strict=True))
                        if maximum_delta > SERVO_MAX_TARGET_DELTA_RAD:
                            raise ValueError(f"servo target joint delta {maximum_delta:.4f} rad exceeds {SERVO_MAX_TARGET_DELTA_RAD:.3f} rad")
                        # Start the next segment from the presently planned
                        # point, avoiding a discontinuity when model inference
                        # finishes a few milliseconds early or late.
                        elapsed = time.monotonic() - servo_segment_started
                        ratio = min(1.0, max(0.0, elapsed / servo_segment_duration))
                        servo_start = tuple(
                            start + ratio * (end - start)
                            for start, end in zip(servo_start, servo_target, strict=True)
                        )
                        servo_target = target
                        servo_segment_started = time.monotonic()
                        servo_segment_duration = min(1.0, max(0.04, float(duration_s)))
                    elif name == "servo_stop":
                        if robot is not None and servo_active:
                            _check(robot.servo_move_enable(False, True), "servo_move_enable(False)")
                        servo_active = False
                        evt_conn.send(("snapshot", _snapshot_dict(connected=robot is not None, message="ACT 关节伺服已停止")))
                    elif name == "cartesian_servo_start":
                        if robot is None:
                            raise RuntimeError("not connected")
                        _require_enabled(robot)
                        if servo_active:
                            _check(robot.servo_move_enable(False, True), "servo_move_enable(False, joint->cartesian)")
                            servo_active = False
                        current = tuple(float(value) for value in _check(robot.get_actual_tcp_position(), "get_actual_tcp_position(Quest servo start)")[1])
                        if len(current) != 6:
                            raise RuntimeError(f"expected six TCP values, got {len(current)}")
                        _check(robot.servo_move_enable(True, True), "servo_move_enable(True, Quest)")
                        cartesian_reference = current
                        cartesian_target = current
                        cartesian_last_target_s = time.monotonic()
                        next_cartesian_tick = cartesian_last_target_s
                        cartesian_servo_active = True
                        cartesian_profile = "general"
                        restore_target = None
                        evt_conn.send(("snapshot", _snapshot_dict(connected=True, message="Quest 笛卡尔伺服已就绪；按住 Grip 才会更新目标")))
                    elif name == "cartesian_servo_target":
                        _, values = command
                        if robot is None or not cartesian_servo_active or cartesian_reference is None or cartesian_target is None:
                            raise RuntimeError("Quest cartesian servo is not active")
                        target = tuple(float(value) for value in values)
                        if len(target) != 6 or not all(math.isfinite(value) for value in target):
                            raise ValueError("Quest TCP target must contain six finite values")
                        if math.dist(target[:3], cartesian_reference[:3]) > CARTESIAN_SERVO_MAX_RELATIVE_MM:
                            raise ValueError(f"Quest target exceeds {CARTESIAN_SERVO_MAX_RELATIVE_MM:.0f} mm reference range")
                        now_target = time.monotonic()
                        allowed = CARTESIAN_SERVO_MAX_SPEED_MM_S * max(SERVO_PERIOD_S, now_target - cartesian_last_target_s)
                        delta = math.dist(target[:3], cartesian_target[:3])
                        if delta > allowed > 0.0:
                            ratio = allowed / delta
                            target = tuple(
                                current + ratio * (requested - current)
                                if index < 3 else requested
                                for index, (current, requested) in enumerate(zip(cartesian_target, target, strict=True))
                            )
                        cartesian_target = target
                        cartesian_last_target_s = now_target
                    elif name == "cartesian_servo_stop":
                        if robot is not None and cartesian_servo_active:
                            _check(robot.servo_move_enable(False, True), "servo_move_enable(False, Quest)")
                        cartesian_servo_active = False
                        cartesian_reference = None
                        cartesian_target = None
                        cartesian_profile = None
                        evt_conn.send(("snapshot", _snapshot_dict(connected=robot is not None, message="Quest 笛卡尔伺服已停止")))
                    elif name == "commissioning_servo_start":
                        _, required_tool_id = command
                        if robot is None:
                            raise RuntimeError("not connected")
                        if int(required_tool_id) != COMMISSION_REQUIRED_TOOL_ID:
                            raise ValueError(
                                f"commissioning requires Tool ID {COMMISSION_REQUIRED_TOOL_ID}"
                            )
                        if servo_active:
                            _check(
                                robot.servo_move_enable(False, True),
                                "servo_move_enable(False, joint->commissioning)",
                            )
                            servo_active = False
                        if cartesian_servo_active:
                            _check(
                                robot.servo_move_enable(False, True),
                                "servo_move_enable(False, cartesian->commissioning)",
                            )
                            cartesian_servo_active = False
                        current = _require_commissioning_ready(robot, required_tool_id)
                        _check(
                            robot.servo_move_enable(True, True),
                            "servo_move_enable(True, commissioning)",
                        )
                        started = time.monotonic()
                        cartesian_reference = current
                        cartesian_target = current
                        cartesian_last_target_s = started
                        next_cartesian_tick = started
                        commissioning_started_s = started
                        cartesian_profile = "commissioning"
                        cartesian_servo_active = True
                        restore_target = None
                        telemetry_health.update(
                            powered_on=True,
                            enabled=True,
                            estop=False,
                            collision=False,
                            on_limit=False,
                            tool_id=int(required_tool_id),
                        )
                        evt_conn.send(
                            (
                                "snapshot",
                                _snapshot_dict(
                                    connected=True,
                                    powered_on=True,
                                    enabled=True,
                                    tool_id=int(required_tool_id),
                                    tcp_pose=current,
                                    message="5 mm / 5 mm/s 微动伺服已启动；必须持续按住 Grip+Trigger",
                                ),
                            )
                        )
                    elif name == "commissioning_servo_target":
                        _, values = command
                        if (
                            robot is None
                            or not cartesian_servo_active
                            or cartesian_profile != "commissioning"
                            or cartesian_reference is None
                            or cartesian_target is None
                        ):
                            raise RuntimeError("commissioning cartesian servo is not active")
                        requested = tuple(float(value) for value in values)
                        now_target = time.monotonic()
                        cartesian_target = limit_commissioning_target(
                            cartesian_reference,
                            cartesian_target,
                            requested,
                            now_target - cartesian_last_target_s,
                        )
                        cartesian_last_target_s = now_target
                    elif name == "commissioning_servo_stop":
                        stop_commissioning("微动伺服已停止；再次运动必须重新 ARM")
                    elif name == "showcase_servo_start":
                        _, required_tool_id, requested_radius_mm, requested_speed_mm_s = command
                        if robot is None:
                            raise RuntimeError("not connected")
                        if int(required_tool_id) != SHOWCASE_REQUIRED_TOOL_ID:
                            raise ValueError(
                                f"showcase requires Tool ID {SHOWCASE_REQUIRED_TOOL_ID}"
                            )
                        requested_radius_mm = float(requested_radius_mm)
                        requested_speed_mm_s = float(requested_speed_mm_s)
                        if not 0.0 < requested_radius_mm <= SHOWCASE_MAX_RELATIVE_MM:
                            raise ValueError(
                                f"showcase radius must be <= {SHOWCASE_MAX_RELATIVE_MM:.0f} mm"
                            )
                        if not 0.0 < requested_speed_mm_s <= SHOWCASE_MAX_SPEED_MM_S:
                            raise ValueError(
                                f"showcase speed must be <= {SHOWCASE_MAX_SPEED_MM_S:.0f} mm/s"
                            )
                        if servo_active:
                            _check(
                                robot.servo_move_enable(False, True),
                                "servo_move_enable(False, joint->showcase)",
                            )
                            servo_active = False
                        if cartesian_servo_active:
                            _check(
                                robot.servo_move_enable(False, True),
                                "servo_move_enable(False, cartesian->showcase)",
                            )
                            cartesian_servo_active = False
                        current = _require_commissioning_ready(robot, required_tool_id)
                        _check(
                            robot.servo_move_enable(True, True),
                            "servo_move_enable(True, showcase)",
                        )
                        started = time.monotonic()
                        cartesian_reference = current
                        cartesian_target = current
                        cartesian_last_target_s = started
                        next_cartesian_tick = started
                        commissioning_started_s = started
                        showcase_radius_mm = requested_radius_mm
                        showcase_speed_mm_s = requested_speed_mm_s
                        cartesian_profile = "showcase"
                        cartesian_servo_active = True
                        restore_target = None
                        telemetry_health.update(
                            powered_on=True,
                            enabled=True,
                            estop=False,
                            collision=False,
                            on_limit=False,
                            tool_id=int(required_tool_id),
                        )
                        evt_conn.send(
                            (
                                "snapshot",
                                _snapshot_dict(
                                    connected=True,
                                    powered_on=True,
                                    enabled=True,
                                    tool_id=int(required_tool_id),
                                    tcp_pose=current,
                                    message=(f"{showcase_radius_mm:.0f} mm / "
                                             f"{showcase_speed_mm_s:.0f} mm/s 展示遥操作已启动；"
                                             "必须持续按住 Grip+Trigger"),
                                ),
                            )
                        )
                    elif name == "showcase_servo_target":
                        _, values = command
                        if (
                            robot is None
                            or not cartesian_servo_active
                            or cartesian_profile != "showcase"
                            or cartesian_reference is None
                            or cartesian_target is None
                        ):
                            raise RuntimeError("showcase cartesian servo is not active")
                        requested = tuple(float(value) for value in values)
                        now_target = time.monotonic()
                        cartesian_target = limit_showcase_target(
                            cartesian_reference,
                            cartesian_target,
                            requested,
                            now_target - cartesian_last_target_s,
                            max_relative_mm=showcase_radius_mm,
                            max_speed_mm_s=showcase_speed_mm_s,
                        )
                        cartesian_last_target_s = now_target
                    elif name == "showcase_servo_stop":
                        stop_commissioning("展示遥操作伺服已停止；再次运动必须重新 ARM")
                    elif name == "engineering_servo_start":
                        raise RuntimeError(ENGINEERING_LIVE_LOCKOUT_REASON)
                        (
                            _, required_tool_id, requested_radius_mm,
                            requested_linear_speed_mm_s, requested_rotation_deg,
                            requested_angular_speed_deg_s, requested_joint_speed_deg_s,
                        ) = command
                        if robot is None:
                            raise RuntimeError("not connected")
                        if int(required_tool_id) != ENGINEERING_REQUIRED_TOOL_ID:
                            raise ValueError(f"engineering requires Tool ID {ENGINEERING_REQUIRED_TOOL_ID}")
                        requested_radius_mm = float(requested_radius_mm)
                        requested_linear_speed_mm_s = float(requested_linear_speed_mm_s)
                        requested_rotation_deg = float(requested_rotation_deg)
                        requested_angular_speed_deg_s = float(requested_angular_speed_deg_s)
                        requested_joint_speed_deg_s = float(requested_joint_speed_deg_s)
                        requested_limits = (
                            (requested_radius_mm, ENGINEERING_MAX_RELATIVE_MM, "radius"),
                            (requested_linear_speed_mm_s, ENGINEERING_MAX_LINEAR_SPEED_MM_S, "linear speed"),
                            (requested_rotation_deg, ENGINEERING_MAX_ROTATION_DEG, "rotation"),
                            (requested_angular_speed_deg_s, ENGINEERING_MAX_ANGULAR_SPEED_DEG_S, "angular speed"),
                            (requested_joint_speed_deg_s, ENGINEERING_MAX_JOINT_SPEED_DEG_S, "joint speed"),
                        )
                        for value, ceiling, label in requested_limits:
                            if not 0.0 < value <= ceiling:
                                raise ValueError(f"engineering {label} exceeds hard limit {ceiling}")
                        if servo_active:
                            _check(robot.servo_move_enable(False, True), "servo_move_enable(False, joint->engineering)")
                            servo_active = False
                        if cartesian_servo_active:
                            _check(robot.servo_move_enable(False, True), "servo_move_enable(False, cartesian->engineering)")
                            cartesian_servo_active = False
                        current = _require_commissioning_ready(robot, required_tool_id)
                        joints_result = _check(robot.get_actual_joint_position(), "get_actual_joint_position(engineering)")
                        current_joints = tuple(float(value) for value in joints_result[1])
                        if len(current_joints) != 6 or not all(math.isfinite(value) for value in current_joints):
                            raise RuntimeError("Level A 未取得六维有限关节角")
                        _check(robot.servo_move_enable(True, True), "servo_move_enable(True, engineering)")
                        started = time.monotonic()
                        cartesian_reference = current
                        cartesian_target = current
                        cartesian_last_target_s = started
                        next_cartesian_tick = started
                        commissioning_started_s = started
                        engineering_radius_mm = requested_radius_mm
                        engineering_linear_speed_mm_s = requested_linear_speed_mm_s
                        engineering_rotation_deg = requested_rotation_deg
                        engineering_angular_speed_deg_s = requested_angular_speed_deg_s
                        engineering_joint_speed_deg_s = requested_joint_speed_deg_s
                        engineering_joint_target = current_joints
                        cartesian_profile = "engineering"
                        cartesian_servo_active = True
                        restore_target = None
                        telemetry_health.update(powered_on=True, enabled=True, estop=False,
                                                collision=False, on_limit=False, tool_id=int(required_tool_id))
                        evt_conn.send(("snapshot", _snapshot_dict(
                            connected=True, powered_on=True, enabled=True,
                            tool_id=int(required_tool_id), tcp_pose=current,
                            engineering_servo_active=True,
                            message=(f"Level A 已启动：{engineering_radius_mm:.0f} mm / "
                                     f"{engineering_rotation_deg:.1f} deg；持续按 Grip"))))
                    elif name == "engineering_servo_target":
                        _, values = command
                        if (robot is None or not cartesian_servo_active
                                or cartesian_profile != "engineering"
                                or cartesian_reference is None or cartesian_target is None
                                or engineering_joint_target is None):
                            raise RuntimeError("engineering cartesian servo is not active")
                        requested = tuple(float(value) for value in values)
                        now_target = time.monotonic()
                        elapsed = now_target - cartesian_last_target_s
                        candidate = limit_engineering_target(
                            cartesian_reference, cartesian_target, requested, elapsed,
                            max_relative_mm=engineering_radius_mm,
                            max_linear_speed_mm_s=engineering_linear_speed_mm_s,
                            max_rotation_deg=engineering_rotation_deg,
                            max_angular_speed_deg_s=engineering_angular_speed_deg_s,
                        )
                        def solve_engineering_ik(reference_joints, pose):
                            return _check(
                                robot.kine_inverse(reference_joints, pose),
                                "kine_inverse(engineering)",
                            )[1]

                        (
                            accepted_pose,
                            accepted_joints,
                            requested_joint_rate,
                            rate_limited,
                            projection_notice,
                        ) = project_engineering_target_with_orientation_priority(
                            cartesian_target,
                            candidate,
                            engineering_joint_target,
                            elapsed,
                            engineering_joint_speed_deg_s,
                            solve_engineering_ik,
                        )
                        cartesian_target = accepted_pose
                        engineering_joint_target = accepted_joints
                        cartesian_last_target_s = now_target
                        if rate_limited and now_target - engineering_last_rate_notice_s >= 0.5:
                            engineering_last_rate_notice_s = now_target
                            evt_conn.send(("snapshot", _snapshot_dict(
                                connected=True,
                                powered_on=True,
                                enabled=True,
                                tool_id=ENGINEERING_REQUIRED_TOOL_ID,
                                message=(f"关节限速介入：请求 {requested_joint_rate:.2f} deg/s，"
                                         f"限制为 {engineering_joint_speed_deg_s:.2f} deg/s；ARM 保持"),
                            )))
                        if projection_notice and now_target - engineering_last_projection_notice_s >= 0.5:
                            engineering_last_projection_notice_s = now_target
                            evt_conn.send(("snapshot", _snapshot_dict(
                                connected=True,
                                powered_on=True,
                                enabled=True,
                                tool_id=ENGINEERING_REQUIRED_TOOL_ID,
                                message=projection_notice + "；ARM 保持",
                            )))
                    elif name == "engineering_servo_stop":
                        stop_commissioning("Level A 运动已暂停；ARM 可保持并重新抓取")
                    elif name in ("gripper_open", "gripper_close"):
                        action = "张开" if name == "gripper_open" else "闭合"
                        evt_conn.send(("snapshot", _snapshot_dict(connected=robot is not None, message=f"夹爪{action}：接口已预留，尚未接入硬件")))
                    elif name == "jog_continue":
                        _, axis, direction, coord_type, speed = command
                        if robot is None:
                            evt_conn.send(("snapshot", _snapshot_dict(connected=False, error="未连接", message="⚠ 未连接")))
                        else:
                            if cartesian_servo_active:
                                _check(robot.servo_move_enable(False, True), "servo_move_enable(False, Quest->jog)")
                                cartesian_servo_active = False
                                cartesian_reference = cartesian_target = None
                            restore_target = None
                            _require_enabled(robot)
                            _check(robot.jog(int(axis), MODE_CONTINUE, int(coord_type), float(speed) * int(direction), 0.0), "jog")
                            evt_conn.send(("snapshot", _snapshot_dict(connected=True, message=f"jog axis {int(axis)} {'+' if int(direction) > 0 else '-'}")))
                    elif name == "jog_incr":
                        _, axis, direction, coord_type, speed, step = command
                        if robot is None:
                            evt_conn.send(("snapshot", _snapshot_dict(connected=False, error="未连接", message="⚠ 未连接")))
                        else:
                            if cartesian_servo_active:
                                _check(robot.servo_move_enable(False, True), "servo_move_enable(False, Quest->jog)")
                                cartesian_servo_active = False
                                cartesian_reference = cartesian_target = None
                            restore_target = None
                            _require_enabled(robot)
                            _check(robot.jog(int(axis), MODE_INCR, int(coord_type), float(speed), float(step) * int(direction)), "jog")
                            evt_conn.send(("snapshot", _snapshot_dict(connected=True, message=f"step axis {int(axis)} {'+' if int(direction) > 0 else '-'}")))
                    elif name == "stop":
                        if robot is not None:
                            restore_target = None
                            had_active_motion = servo_active or cartesian_servo_active
                            if had_active_motion:
                                _check(robot.servo_move_enable(False, True), "servo_move_enable(False)")
                                servo_active = False
                                cartesian_servo_active = False
                                cartesian_reference = cartesian_target = None
                                _check(robot.motion_abort(), "motion_abort")
                            evt_conn.send(("snapshot", _snapshot_dict(
                                connected=True,
                                engineering_servo_active=False,
                                message="已停止运动" if had_active_motion else "当前无活动运动；STOP 已确认",
                            )))
                except Exception as error:
                    log(f"ERROR {name}: {error!r}")
                    if name.startswith(("commissioning_", "showcase_", "engineering_")):
                        profile_label = (
                            "Level A 六维遥操作" if name.startswith("engineering_")
                            else "展示遥操作" if name.startswith("showcase_") else "微动"
                        )
                        stop_commissioning(
                            f"{profile_label}伺服已自动停止：{error}",
                            error=f"{name}: {error}",
                        )
                    evt_conn.send(
                        (
                            "snapshot",
                            _snapshot_dict(
                                connected=robot is not None,
                                engineering_servo_active=False if name.startswith("engineering_") else None,
                                error=f"{name}: {error}",
                                message=f"⚠ {name} 失败：{error}",
                            ),
                        )
                    )
                finally:
                    command_ms = (time.perf_counter() - command_started_s) * 1000
                    if command_ms >= 20:
                        log(f"COMMAND_TIMING name={name} elapsed_ms={command_ms:.3f}")
                # A stream of Quest targets must not starve measured state.
                # Process one command, then service telemetry if it is due.
            now = time.monotonic()
            if robot is not None and now - last_poll >= poll_interval:
                poll_started_s = time.perf_counter()
                _poll_sdk(robot, evt_conn, telemetry_health)
                poll_ms = (time.perf_counter() - poll_started_s) * 1000
                if poll_ms >= 50:
                    log(f"TELEMETRY_TIMING elapsed_ms={poll_ms:.3f}")
                last_poll = now
            time.sleep(0.005)
    except (EOFError, BrokenPipeError, OSError):
        pass
    finally:
        _logout(robot)
        try:
            evt_conn.close()
        except Exception:
            pass


def _demo_worker_main(cmd_conn: Any, evt_conn: Any, poll_interval: float) -> None:
    connected = False
    powered_on = False
    enabled = False
    tool_id = 0
    xyz = [0.43, 0.0, 0.35]
    joints = [0.0, 1.55, 0.25, 1.45, 0.0, 0.0]
    velocity = [0.0] * 6
    rpy = [0.0, 0.0, 0.0]
    last_tick = time.monotonic()
    last_poll = 0.0
    commissioning_active = False
    commissioning_reference: tuple[float, ...] | None = None
    commissioning_target: tuple[float, ...] | None = None
    commissioning_last_target_s = 0.0
    commissioning_started_s = 0.0
    demo_guarded_profile: str | None = None
    showcase_radius_mm = SHOWCASE_MAX_RELATIVE_MM
    showcase_speed_mm_s = 20.0
    engineering_radius_mm = 50.0
    engineering_linear_speed_mm_s = 20.0
    engineering_rotation_deg = 2.0
    engineering_angular_speed_deg_s = 3.0
    try:
        while True:
            now = time.monotonic()
            demo_watchdog_s = (
                ENGINEERING_TARGET_WATCHDOG_S if demo_guarded_profile == "engineering"
                else SHOWCASE_TARGET_WATCHDOG_S if demo_guarded_profile == "showcase"
                else COMMISSION_TARGET_WATCHDOG_S
            )
            demo_session_s = (
                ENGINEERING_MAX_SESSION_S if demo_guarded_profile == "engineering"
                else SHOWCASE_MAX_SESSION_S if demo_guarded_profile == "showcase"
                else COMMISSION_MAX_SESSION_S
            )
            if commissioning_active and (
                now - commissioning_last_target_s > demo_watchdog_s
                or now - commissioning_started_s > demo_session_s
            ):
                commissioning_active = False
                commissioning_reference = commissioning_target = None
                stopped_profile = demo_guarded_profile or "commissioning"
                demo_guarded_profile = None
                evt_conn.send(
                    (
                        "snapshot",
                        _snapshot_dict(
                            connected=connected,
                            powered_on=powered_on,
                            enabled=enabled,
                            tool_id=tool_id,
                            engineering_servo_active=False if stopped_profile == "engineering" else None,
                            message=f"{stopped_profile} 伺服已自动停止（演示看门狗）",
                        ),
                    )
                )
            if cmd_conn.poll():
                command = cmd_conn.recv()
                name = command[0]
                if name == "shutdown":
                    return
                if name == "login":
                    connected = powered_on = enabled = True
                    evt_conn.send(("snapshot", _snapshot_dict(connected=True, powered_on=True, enabled=True, tool_id=tool_id, message="connected (demo)")))
                elif name == "clear_error":
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=connected, powered_on=powered_on, enabled=enabled,
                        tool_id=tool_id, message="已清除软件故障提示；请重新检查实时状态（演示）",
                    )))
                elif name == "logout":
                    connected = powered_on = enabled = False
                    velocity = [0.0] * 6
                    commissioning_active = False
                    commissioning_reference = commissioning_target = None
                    demo_guarded_profile = None
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=False, engineering_servo_active=False,
                        message="disconnected (demo)",
                    )))
                elif name == "power_on":
                    powered_on = True
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=True, enabled=enabled, tool_id=tool_id, message="已上电")))
                elif name == "power_off":
                    powered_on = enabled = False
                    velocity = [0.0] * 6
                    commissioning_active = False
                    commissioning_reference = commissioning_target = None
                    demo_guarded_profile = None
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=connected, powered_on=False, enabled=False, tool_id=tool_id,
                        engineering_servo_active=False, message="已下电",
                    )))
                elif name == "enable":
                    enabled = True
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=powered_on, enabled=True, tool_id=tool_id, message="已使能")))
                elif name == "disable":
                    enabled = False
                    velocity = [0.0] * 6
                    commissioning_active = False
                    commissioning_reference = commissioning_target = None
                    demo_guarded_profile = None
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=connected, powered_on=powered_on, enabled=False, tool_id=tool_id,
                        engineering_servo_active=False, message="已去使能",
                    )))
                elif name == "set_tool_id":
                    _, new_id = command
                    tool_id = int(new_id)
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=powered_on, enabled=enabled, tool_id=tool_id, message=f"已切换工具 ID {tool_id}")))
                elif name == "level_flange":
                    rpy[0] = rpy[1] = 0.0
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=powered_on, enabled=enabled, tool_id=tool_id, tcp_pose=(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]), joints_rad=tuple(joints), message="法兰已找平（演示）")))
                elif name == "restore_joints":
                    _, target, _speed = command
                    if len(target) != 6:
                        raise ValueError("saved joint position must contain J1-J6")
                    joints = [float(value) for value in target]
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=powered_on, enabled=enabled, tool_id=tool_id, joints_rad=tuple(joints), message="已复原保存位置（演示）")))
                elif name == "linear_relative":
                    _, delta_xyz, delta_rpy_deg, _speed, required_tool_id = command
                    if tool_id != int(required_tool_id):
                        raise RuntimeError("wrong tool ID")
                    xyz_delta = tuple(float(value) for value in delta_xyz)
                    rpy_delta = tuple(float(value) for value in delta_rpy_deg)
                    if math.dist(xyz_delta, (0.0, 0.0, 0.0)) > APPROVAL_MAX_TRANSLATION_MM:
                        raise ValueError("translation exceeds approval hard limit")
                    if math.dist(rpy_delta, (0.0, 0.0, 0.0)) > APPROVAL_MAX_ROTATION_DEG:
                        raise ValueError("rotation exceeds approval hard limit")
                    for axis in range(3):
                        xyz[axis] += xyz_delta[axis]
                        rpy[axis] += math.radians(rpy_delta[axis])
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=powered_on, enabled=enabled, tool_id=tool_id, tcp_pose=(*xyz, *rpy), joints_rad=tuple(joints), message="ACT 单步已执行（演示）")))
                elif name == "servo_start":
                    if not (connected and enabled):
                        raise RuntimeError("demo robot is not enabled")
                    velocity = [0.0] * 6
                    evt_conn.send(("snapshot", _snapshot_dict(connected=True, powered_on=True, enabled=True, tool_id=tool_id, joints_rad=tuple(joints), message="ACT 关节伺服已就绪（演示）")))
                elif name == "servo_target":
                    _, target, _duration_s = command
                    if len(target) != 6:
                        raise ValueError("servo joint target must contain J1-J6")
                    joints = [float(value) for value in target]
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=powered_on, enabled=enabled, tool_id=tool_id, joints_rad=tuple(joints), message="ACT 关节伺服目标已更新（演示）")))
                elif name == "servo_stop":
                    velocity = [0.0] * 6
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=powered_on, enabled=enabled, tool_id=tool_id, message="ACT 关节伺服已停止（演示）")))
                elif name == "cartesian_servo_start":
                    if not (connected and enabled):
                        raise RuntimeError("demo robot is not enabled")
                    evt_conn.send(("snapshot", _snapshot_dict(connected=True, powered_on=True, enabled=True, tool_id=tool_id, tcp_pose=(*xyz, *rpy), message="Quest 笛卡尔伺服已就绪（演示）")))
                elif name == "cartesian_servo_target":
                    _, target = command
                    if len(target) != 6:
                        raise ValueError("Quest TCP target must contain six values")
                    xyz = [float(value) for value in target[:3]]
                    rpy = [float(value) for value in target[3:]]
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=powered_on, enabled=enabled, tool_id=tool_id, tcp_pose=(*xyz, *rpy), message="Quest TCP 目标已更新（演示）")))
                elif name == "cartesian_servo_stop":
                    velocity = [0.0] * 6
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, powered_on=powered_on, enabled=enabled, tool_id=tool_id, message="Quest 笛卡尔伺服已停止（演示）")))
                elif name == "commissioning_servo_start":
                    _, required_tool_id = command
                    if not (connected and powered_on and enabled):
                        raise RuntimeError("demo robot is not enabled")
                    if tool_id != int(required_tool_id) or tool_id != COMMISSION_REQUIRED_TOOL_ID:
                        raise RuntimeError("wrong commissioning tool ID")
                    current = (*xyz, *rpy)
                    commissioning_reference = current
                    commissioning_target = current
                    commissioning_last_target_s = commissioning_started_s = time.monotonic()
                    commissioning_active = True
                    demo_guarded_profile = "commissioning"
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=True, powered_on=True, enabled=True, tool_id=tool_id,
                        tcp_pose=current,
                        message="5 mm / 5 mm/s 微动伺服已启动（演示）")))
                elif name == "commissioning_servo_target":
                    _, target = command
                    if not commissioning_active or commissioning_reference is None or commissioning_target is None:
                        raise RuntimeError("commissioning cartesian servo is not active")
                    now_target = time.monotonic()
                    commissioning_target = limit_commissioning_target(
                        commissioning_reference,
                        commissioning_target,
                        tuple(float(value) for value in target),
                        now_target - commissioning_last_target_s,
                    )
                    commissioning_last_target_s = now_target
                    xyz = list(commissioning_target[:3])
                    rpy = list(commissioning_target[3:])
                elif name == "commissioning_servo_stop":
                    commissioning_active = False
                    commissioning_reference = commissioning_target = None
                    demo_guarded_profile = None
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=connected, powered_on=powered_on, enabled=enabled,
                        tool_id=tool_id,
                        message="微动伺服已停止（演示）；再次运动必须重新 ARM")))
                elif name == "showcase_servo_start":
                    _, required_tool_id, requested_radius_mm, requested_speed_mm_s = command
                    if not (connected and powered_on and enabled):
                        raise RuntimeError("demo robot is not enabled")
                    if tool_id != int(required_tool_id) or tool_id != SHOWCASE_REQUIRED_TOOL_ID:
                        raise RuntimeError("wrong showcase tool ID")
                    requested_radius_mm = float(requested_radius_mm)
                    requested_speed_mm_s = float(requested_speed_mm_s)
                    if not 0.0 < requested_radius_mm <= SHOWCASE_MAX_RELATIVE_MM:
                        raise ValueError("showcase radius exceeds hard limit")
                    if not 0.0 < requested_speed_mm_s <= SHOWCASE_MAX_SPEED_MM_S:
                        raise ValueError("showcase speed exceeds hard limit")
                    current = (*xyz, *rpy)
                    commissioning_reference = current
                    commissioning_target = current
                    commissioning_last_target_s = commissioning_started_s = time.monotonic()
                    commissioning_active = True
                    demo_guarded_profile = "showcase"
                    showcase_radius_mm = requested_radius_mm
                    showcase_speed_mm_s = requested_speed_mm_s
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=True, powered_on=True, enabled=True, tool_id=tool_id,
                        tcp_pose=current,
                        message=(f"{showcase_radius_mm:.0f} mm / {showcase_speed_mm_s:.0f} mm/s "
                                 "展示遥操作已启动（演示）"))))
                elif name == "showcase_servo_target":
                    _, target = command
                    if (not commissioning_active or demo_guarded_profile != "showcase"
                            or commissioning_reference is None or commissioning_target is None):
                        raise RuntimeError("showcase cartesian servo is not active")
                    now_target = time.monotonic()
                    commissioning_target = limit_showcase_target(
                        commissioning_reference,
                        commissioning_target,
                        tuple(float(value) for value in target),
                        now_target - commissioning_last_target_s,
                        max_relative_mm=showcase_radius_mm,
                        max_speed_mm_s=showcase_speed_mm_s,
                    )
                    commissioning_last_target_s = now_target
                    xyz = list(commissioning_target[:3])
                    rpy = list(commissioning_target[3:])
                elif name == "showcase_servo_stop":
                    commissioning_active = False
                    commissioning_reference = commissioning_target = None
                    demo_guarded_profile = None
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=connected, powered_on=powered_on, enabled=enabled,
                        tool_id=tool_id,
                        message="展示遥操作伺服已停止（演示）；再次运动必须重新 ARM")))
                elif name == "engineering_servo_start":
                    (
                        _, required_tool_id, requested_radius_mm,
                        requested_linear_speed_mm_s, requested_rotation_deg,
                        requested_angular_speed_deg_s, requested_joint_speed_deg_s,
                    ) = command
                    if not (connected and powered_on and enabled):
                        raise RuntimeError("demo robot is not enabled")
                    if tool_id != int(required_tool_id) or tool_id != ENGINEERING_REQUIRED_TOOL_ID:
                        raise RuntimeError("wrong engineering tool ID")
                    limits = (
                        (float(requested_radius_mm), ENGINEERING_MAX_RELATIVE_MM),
                        (float(requested_linear_speed_mm_s), ENGINEERING_MAX_LINEAR_SPEED_MM_S),
                        (float(requested_rotation_deg), ENGINEERING_MAX_ROTATION_DEG),
                        (float(requested_angular_speed_deg_s), ENGINEERING_MAX_ANGULAR_SPEED_DEG_S),
                        (float(requested_joint_speed_deg_s), ENGINEERING_MAX_JOINT_SPEED_DEG_S),
                    )
                    if any(not 0.0 < value <= ceiling for value, ceiling in limits):
                        raise ValueError("engineering setting exceeds hard limit")
                    current = (*xyz, *rpy)
                    commissioning_reference = commissioning_target = current
                    commissioning_last_target_s = commissioning_started_s = time.monotonic()
                    commissioning_active = True
                    demo_guarded_profile = "engineering"
                    engineering_radius_mm = float(requested_radius_mm)
                    engineering_linear_speed_mm_s = float(requested_linear_speed_mm_s)
                    engineering_rotation_deg = float(requested_rotation_deg)
                    engineering_angular_speed_deg_s = float(requested_angular_speed_deg_s)
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=True, powered_on=True, enabled=True, tool_id=tool_id,
                        tcp_pose=current, engineering_servo_active=True,
                        message="Level A 六维遥操作已启动（演示）")))
                elif name == "engineering_servo_target":
                    _, target = command
                    if (not commissioning_active or demo_guarded_profile != "engineering"
                            or commissioning_reference is None or commissioning_target is None):
                        raise RuntimeError("engineering cartesian servo is not active")
                    now_target = time.monotonic()
                    commissioning_target = limit_engineering_target(
                        commissioning_reference, commissioning_target,
                        tuple(float(value) for value in target),
                        now_target - commissioning_last_target_s,
                        max_relative_mm=engineering_radius_mm,
                        max_linear_speed_mm_s=engineering_linear_speed_mm_s,
                        max_rotation_deg=engineering_rotation_deg,
                        max_angular_speed_deg_s=engineering_angular_speed_deg_s,
                    )
                    commissioning_last_target_s = now_target
                    xyz = list(commissioning_target[:3])
                    rpy = list(commissioning_target[3:])
                elif name == "engineering_servo_stop":
                    commissioning_active = False
                    commissioning_reference = commissioning_target = None
                    demo_guarded_profile = None
                    evt_conn.send(("snapshot", _snapshot_dict(
                        connected=connected, powered_on=powered_on, enabled=enabled,
                        tool_id=tool_id, engineering_servo_active=False,
                        message="Level A 运动已暂停（演示）；ARM 可保持并重新抓取")))
                elif name in ("gripper_open", "gripper_close"):
                    action = "张开" if name == "gripper_open" else "闭合"
                    evt_conn.send(("snapshot", _snapshot_dict(connected=connected, message=f"夹爪{action}：接口已预留（演示）")))
                elif name == "jog_continue":
                    _, axis, direction, coord_type, speed = command
                    if 0 <= int(axis) <= 5:
                        velocity[int(axis)] = float(speed) * int(direction)
                elif name == "jog_incr":
                    _, axis, direction, coord_type, speed, step = command
                    if 0 <= int(axis) <= 2:
                        xyz[int(axis)] += float(step) * int(direction)
                    elif 3 <= int(axis) <= 5:
                        rpy[int(axis) - 3] += float(step) * int(direction) * 0.017453292519943295
                elif name == "stop":
                    velocity = [0.0] * 6
                continue

            dt = now - last_tick
            last_tick = now
            if connected and enabled:
                for axis in range(3):
                    xyz[axis] += velocity[axis] * dt
                joints[0] += velocity[0] * 0.7 * dt
                joints[2] += velocity[1] * 0.7 * dt
                joints[4] += velocity[2] * 0.7 * dt
                for axis in range(3, 6):
                    rpy[axis - 3] += velocity[axis] * 0.017453292519943295 * dt

            if now - last_poll >= poll_interval:
                moving = any(abs(v) > 1e-6 for v in velocity)
                evt_conn.send(
                    (
                        "snapshot",
                        _snapshot_dict(
                            connected=connected,
                            powered_on=powered_on,
                            enabled=enabled,
                            moving=moving,
                            tool_id=tool_id,
                            timestamp_ns=time.time_ns(),
                            tcp_pose=(xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]),
                            joints_rad=tuple(joints),
                        ),
                    )
                )
                last_poll = now
            time.sleep(0.005)
    except (EOFError, BrokenPipeError, OSError):
        pass
    finally:
        try:
            evt_conn.close()
        except Exception:
            pass


class JakaJogController:
    """Parent-side handle to a spawned JAKA SDK worker process."""

    def __init__(
        self,
        host: str = "10.5.5.100",
        sdk_directory: str | Path = SDK_DIRECTORY_DEFAULT,
        *,
        poll_hz: float = 15.0,
        demo: bool = False,
    ) -> None:
        self.host = host
        self.sdk_directory = str(sdk_directory)
        self.demo = demo
        self.poll_interval_s = 1.0 / max(1.0, float(poll_hz))
        self._snapshot = RobotSnapshot()
        self._telemetry_pending: deque[RobotSnapshot] = deque(maxlen=512)
        self.telemetry_dropped = 0
        # Tk rendering and Episode recording both consume telemetry.  Keep
        # pipe draining and snapshot copying safe when they run concurrently.
        self._drain_lock = threading.Lock()
        self._snapshot_lock = threading.RLock()
        self._proc: Any = None
        self._cmd: Any = None
        self._evt: Any = None
        self._start()

    def _start(self) -> None:
        context = mp.get_context("spawn")
        cmd_recv, cmd_send = context.Pipe(duplex=False)
        evt_recv, evt_send = context.Pipe(duplex=False)
        if self.demo:
            target = _demo_worker_main
            args: tuple[Any, ...] = (cmd_recv, evt_send, self.poll_interval_s)
        else:
            target = _sdk_worker_main
            args = (self.host, self.sdk_directory, cmd_recv, evt_send, self.poll_interval_s)
        process = context.Process(target=target, args=args, name="jaka-jog-worker", daemon=True)
        process.start()
        cmd_recv.close()
        evt_send.close()
        self._cmd = cmd_send
        self._evt = evt_recv
        self._proc = process

    def _send(self, command: tuple[Any, ...]) -> None:
        if self._cmd is None:
            return
        try:
            self._cmd.send(command)
        except (BrokenPipeError, OSError, EOFError):
            with self._snapshot_lock:
                self._snapshot.error = "JAKA worker is not running"

    # ------------------------------------------------------------------ API
    def login(self) -> None:
        self._send(("login",))

    def logout(self) -> None:
        self._send(("logout",))

    def clear_error(self) -> None:
        """Clear a latched software message without changing robot state."""
        self._send(("clear_error",))

    def power_on(self) -> None:
        self._send(("power_on",))

    def power_off(self) -> None:
        self._send(("power_off",))

    def enable(self) -> None:
        self._send(("enable",))

    def disable(self) -> None:
        self._send(("disable",))

    def set_tool_id(self, tool_id: int) -> None:
        self._send(("set_tool_id", int(tool_id)))

    def read_tool_profiles(self) -> None:
        self._send(("read_tool_profiles",))

    def level_flange(self, joint_speed_rad_s: float = 0.12) -> None:
        self._send(("level_flange", float(joint_speed_rad_s)))

    def restore_joints(self, joints_rad: tuple[float, ...], joint_speed_rad_s: float = 0.12) -> None:
        self._send(("restore_joints", tuple(float(value) for value in joints_rad), float(joint_speed_rad_s)))

    def linear_relative(
        self,
        delta_xyz_mm: tuple[float, float, float],
        delta_rpy_deg: tuple[float, float, float],
        *,
        speed_mm_s: float = 10.0,
        required_tool_id: int = 7,
    ) -> None:
        self._send(
            (
                "linear_relative",
                tuple(float(value) for value in delta_xyz_mm),
                tuple(float(value) for value in delta_rpy_deg),
                float(speed_mm_s),
                int(required_tool_id),
            )
        )

    def gripper_open(self) -> None:
        self._send(("gripper_open",))

    def gripper_close(self) -> None:
        self._send(("gripper_close",))

    def start_joint_servo(self) -> None:
        """Enter JAKA joint servo mode without moving the robot."""
        self._send(("servo_start",))

    def set_joint_servo_target(self, joints_rad: tuple[float, ...], duration_s: float = 0.20) -> None:
        """Replace the end target of the currently active 8 ms servo stream."""
        self._send(("servo_target", tuple(float(value) for value in joints_rad), float(duration_s)))

    def stop_joint_servo(self) -> None:
        self._send(("servo_stop",))

    def start_cartesian_servo(self) -> None:
        """Enter absolute TCP servo mode without issuing a target movement."""
        self._send(("cartesian_servo_start",))

    def set_cartesian_servo_target(self, tcp_pose_mm_rad: tuple[float, ...]) -> None:
        """Replace the absolute `[x, y, z, rx, ry, rz]` TCP target in worker state."""
        self._send(("cartesian_servo_target", tuple(float(value) for value in tcp_pose_mm_rad)))

    def stop_cartesian_servo(self) -> None:
        self._send(("cartesian_servo_stop",))

    def start_commissioning_cartesian_servo(self, required_tool_id: int = COMMISSION_REQUIRED_TOOL_ID) -> None:
        """Start the fixed 5 mm / 5 mm/s, translation-only commissioning path."""
        self._send(("commissioning_servo_start", int(required_tool_id)))

    def set_commissioning_cartesian_target(self, tcp_pose_mm_rad: tuple[float, ...]) -> None:
        """Refresh the commissioning target and its fail-closed watchdog."""
        self._send(("commissioning_servo_target", tuple(float(value) for value in tcp_pose_mm_rad)))

    def stop_commissioning_cartesian_servo(self) -> None:
        self._send(("commissioning_servo_stop",))

    def start_showcase_cartesian_servo(
        self,
        required_tool_id: int = SHOWCASE_REQUIRED_TOOL_ID,
        *,
        radius_mm: float = SHOWCASE_MAX_RELATIVE_MM,
        speed_mm_s: float = 20.0,
    ) -> None:
        """Start the configurable but hard-capped translation-only showcase path."""
        self._send(
            (
                "showcase_servo_start",
                int(required_tool_id),
                float(radius_mm),
                float(speed_mm_s),
            )
        )

    def set_showcase_cartesian_target(self, tcp_pose_mm_rad: tuple[float, ...]) -> None:
        """Refresh the visible-motion showcase target and watchdog."""
        self._send(("showcase_servo_target", tuple(float(value) for value in tcp_pose_mm_rad)))

    def stop_showcase_cartesian_servo(self) -> None:
        self._send(("showcase_servo_stop",))

    def start_engineering_cartesian_servo(
        self,
        required_tool_id: int = ENGINEERING_REQUIRED_TOOL_ID,
        *,
        radius_mm: float = 50.0,
        linear_speed_mm_s: float = 20.0,
        rotation_deg: float = 2.0,
        angular_speed_deg_s: float = 3.0,
        joint_speed_deg_s: float = 3.0,
    ) -> None:
        """Start the worker-capped 6-DoF Level-A teleoperation profile."""
        self._send((
            "engineering_servo_start", int(required_tool_id), float(radius_mm),
            float(linear_speed_mm_s), float(rotation_deg),
            float(angular_speed_deg_s), float(joint_speed_deg_s),
        ))

    def set_engineering_cartesian_target(self, tcp_pose_mm_rad: tuple[float, ...]) -> None:
        self._send(("engineering_servo_target", tuple(float(value) for value in tcp_pose_mm_rad)))

    def stop_engineering_cartesian_servo(self) -> None:
        self._send(("engineering_servo_stop",))

    def start_jog(self, axis: int, direction: int, coord_type: int, speed: float) -> None:
        self._send(("jog_continue", int(axis), int(direction), int(coord_type), float(speed)))

    def step_jog(self, axis: int, direction: int, coord_type: int, speed: float, step: float) -> None:
        self._send(("jog_incr", int(axis), int(direction), int(coord_type), float(speed), float(step)))

    def stop_jog(self) -> None:
        self._send(("stop",))

    def shutdown(self) -> None:
        self._send(("stop",))
        self._send(("shutdown",))
        process, self._proc = self._proc, None
        if process is not None:
            process.join(2.0)
            if process.is_alive():
                process.terminate()
                process.join(0.5)
        for connection in (self._cmd, self._evt):
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
        self._cmd = None
        self._evt = None

    def drain(self) -> None:
        if self._evt is None:
            return
        with self._drain_lock:
            try:
                while self._evt is not None and self._evt.poll():
                    event = self._evt.recv()
                    self._apply(event)
            except (EOFError, BrokenPipeError, OSError):
                with self._snapshot_lock:
                    self._snapshot.connected = False
                    self._snapshot.error = "JAKA worker stopped"

    def _apply(self, event: tuple[Any, ...]) -> None:
        with self._snapshot_lock:
            self._apply_locked(event)

    def _apply_locked(self, event: tuple[Any, ...]) -> None:
        if not event:
            return
        if event[0] == "tool_profiles":
            profiles = event[1] if len(event) > 1 else []
            if isinstance(profiles, list):
                self._snapshot.tool_profiles = [dict(profile) for profile in profiles if isinstance(profile, dict)]
            return
        if event[0] != "snapshot":
            return
        data = event[1]
        if not isinstance(data, dict):
            return
        snap = self._snapshot
        for key in ("connected", "powered_on", "enabled", "moving", "estop", "collision", "on_limit"):
            if key in data:
                setattr(snap, key, bool(data[key]))
        if data.get("engineering_servo_active") is not None:
            snap.engineering_servo_active = bool(data["engineering_servo_active"])
        if "tool_id" in data:
            # Some JAKA firmware accepts set_tool_id but cannot report the
            # active ID through get_tool_id. Keep the last successfully set
            # value instead of erasing it on every telemetry poll; otherwise
            # the approval GUI resends set_tool_id between every ACT step.
            if data["tool_id"] is not None:
                snap.tool_id = int(data["tool_id"])
            elif data.get("connected") is False:
                snap.tool_id = None
        if "timestamp_ns" in data:
            # Command acknowledgements deliberately have no measurement
            # timestamp.  Keep the latest measured timestamp instead of
            # making the GUI briefly conclude that robot telemetry vanished.
            if data["timestamp_ns"] is not None:
                snap.timestamp_ns = int(data["timestamp_ns"])
            elif data.get("connected") is False:
                snap.timestamp_ns = None
        if "tcp_pose" in data:
            if data["tcp_pose"] is not None:
                snap.tcp_pose = tuple(float(v) for v in data["tcp_pose"])
            elif data.get("connected") is False:
                snap.tcp_pose = None
        if "joints_rad" in data:
            if data["joints_rad"] is not None:
                snap.joints_rad = tuple(float(v) for v in data["joints_rad"])
            elif data.get("connected") is False:
                snap.joints_rad = None
        message = data.get("message")
        incoming_error = str(data.get("error") or "")
        if incoming_error:
            snap.error = incoming_error
        elif message:
            # A successful command clears the previous command error. Routine
            # telemetry carries an empty error field and must not erase it.
            snap.error = ""
        if message:
            snap.log.append(str(message))
            snap.log = snap.log[-200:]
        # Preserve every measured sample while draining a burst of IPC events.
        # Command acknowledgements are not measurements and must not acquire
        # an artificial timestamp or enter the recording history.
        if data.get("timestamp_ns") is not None and data.get("tcp_pose") is not None and data.get("joints_rad") is not None:
            if len(self._telemetry_pending) == self._telemetry_pending.maxlen:
                self.telemetry_dropped += 1
            self._telemetry_pending.append(replace(snap, log=[], tool_profiles=[]))
        elif data.get("connected") is False:
            snap.engineering_servo_active = False
            self._telemetry_pending.clear()

    def take_telemetry(self) -> list[RobotSnapshot]:
        """Drain all new measured samples for the GUI recorder, not just last."""
        self.drain()
        with self._snapshot_lock:
            result = list(self._telemetry_pending)
            self._telemetry_pending.clear()
            return result

    def get_snapshot(self) -> RobotSnapshot:
        self.drain()
        with self._snapshot_lock:
            return RobotSnapshot(
                connected=self._snapshot.connected,
                powered_on=self._snapshot.powered_on,
                enabled=self._snapshot.enabled,
                moving=self._snapshot.moving,
                estop=self._snapshot.estop,
                collision=self._snapshot.collision,
                on_limit=self._snapshot.on_limit,
                tool_id=self._snapshot.tool_id,
                tool_profiles=[dict(profile) for profile in self._snapshot.tool_profiles],
                timestamp_ns=self._snapshot.timestamp_ns,
                tcp_pose=self._snapshot.tcp_pose,
                joints_rad=self._snapshot.joints_rad,
                engineering_servo_active=self._snapshot.engineering_servo_active,
                error=self._snapshot.error,
                log=list(self._snapshot.log),
            )
