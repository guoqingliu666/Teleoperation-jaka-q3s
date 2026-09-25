"""新②的离线候选：只生成 JAKA SDK 调用建议，绝不发送运动命令。

为什么与原控制器分开：2026-09-19 真机曾出现剧烈抖动、J4 欠压和保护停机。
现有真机入口必须继续硬锁。这里仅用注入的 SDK 测试替身调用厂商
``kine_inverse``，生成与逆解结果逐元素相同的 ``servo_j`` 参数，供离线
回放核对。没有登录、上电、使能、伺服启动或实际发送代码。

本模块不实现自己的逆运动学、奇异点绕行或速度前瞻。跳变检查只会拒绝，
不会修改目标、投影到边界或自动追赶。通过离线测试不代表可以接入真机。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Protocol

from .quest_vr_input import matmul, rotation_angle_rad, rpy_matrix, transpose


Pose = tuple[float, float, float, float, float, float]  # XYZ 毫米、RPY 弧度
Joints = tuple[float, float, float, float, float, float]  # 六轴弧度


class InverseSolver(Protocol):
    """测试替身的唯一接口；候选模块不创建或连接真实 RC。"""

    def kine_inverse(self, reference: Joints, pose: Pose) -> object: ...


@dataclass(frozen=True)
class JointNlfLimits:
    """厂商滤波器参数；数值必须经现场工程师确认，故没有默认值。"""

    speed_deg_s: float
    acceleration_deg_s2: float
    jerk_deg_s3: float

    def sdk_call(self) -> tuple[str, tuple[float, float, float]]:
        for name, value in (
            ("speed_deg_s", self.speed_deg_s),
            ("acceleration_deg_s2", self.acceleration_deg_s2),
            ("jerk_deg_s3", self.jerk_deg_s3),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须为正的有限数")
        return (
            "servo_move_use_joint_NLF",
            (self.speed_deg_s, self.acceleration_deg_s2, self.jerk_deg_s3),
        )


@dataclass(frozen=True)
class JointLpfCutoff:
    """厂商关节一阶低通截止频率；必须由 JAKA 工程师确认，没有真机默认值。"""

    cutoff_frequency: float

    def sdk_call(self) -> tuple[str, tuple[float]]:
        value = float(self.cutoff_frequency)
        if not math.isfinite(value) or value <= 0:
            raise ValueError("cutoff_frequency 必须为正的有限数")
        return "servo_move_use_joint_LPF", (value,)


@dataclass(frozen=True)
class RejectLimits:
    """仅用于拒绝异常样本，不会裁剪或重规划目标。"""

    max_input_age_s: float
    max_feedback_age_s: float
    max_position_jump_mm: float
    max_orientation_jump_deg: float
    max_joint_solution_jump_rad: float
    max_inverse_elapsed_s: float
    max_session_translation_mm: float
    max_session_orientation_deg: float

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须为正的有限数")


@dataclass(frozen=True)
class Sample:
    """一帧完整输入；时间戳必须来自同一台主机的 perf_counter 时钟。"""

    target_pose: Pose
    actual_tcp_pose: Pose
    actual_joints: Joints
    input_time_s: float
    feedback_time_s: float
    tracked: bool
    grip_held: bool
    powered: bool
    enabled: bool
    estop: bool
    collision: bool
    on_limit: bool
    tool_id: int


@dataclass(frozen=True)
class Decision:
    """离线决策；servo_call 只是数据，绝不在这里执行。"""

    accepted: bool
    reason: str
    servo_call: tuple[str, tuple[object, ...]] | None = None
    inverse_elapsed_s: float | None = None


def _six_finite(values: object, label: str) -> tuple[float, ...]:
    if not isinstance(values, (tuple, list)) or len(values) != 6:
        raise ValueError(f"{label} 必须恰好有六个数")
    converted = tuple(float(item) for item in values)
    if not all(math.isfinite(item) for item in converted):
        raise ValueError(f"{label} 含 NaN 或无穷大")
    return converted


def _pose_jump(previous: Pose, current: Pose) -> tuple[float, float]:
    distance_mm = math.dist(previous[:3], current[:3])
    relative = matmul(transpose(rpy_matrix(previous[3:])), rpy_matrix(current[3:]))
    return distance_mm, math.degrees(rotation_angle_rad(relative))


class OfflineVendorIkCandidate:
    """单向离线评估器；一次拒绝后锁存，禁止下一帧自动恢复。"""

    def __init__(self, limits: RejectLimits, *, clock: Callable[[], float] = time.perf_counter):
        self._limits = limits
        self._clock = clock
        self._last_target: Pose | None = None
        self._anchor_pose: Pose | None = None
        self._last_solution: Joints | None = None
        self._latched_reason: str | None = None

    @property
    def latched_reason(self) -> str | None:
        return self._latched_reason

    def evaluate(self, sdk: InverseSolver, sample: Sample) -> Decision:
        """只求解，不发送；参考关节角必须是本帧实测角，而非上一目标。"""

        if self._latched_reason is not None:
            return Decision(False, f"已锁存：{self._latched_reason}")

        def reject(reason: str) -> Decision:
            self._latched_reason = reason
            return Decision(False, reason)

        now = self._clock()
        try:
            target = _six_finite(sample.target_pose, "目标位姿")
            actual_pose = _six_finite(sample.actual_tcp_pose, "实测 TCP")
            actual_joints = _six_finite(sample.actual_joints, "实测关节角")
            for label, stamp, max_age in (
                ("手柄", sample.input_time_s, self._limits.max_input_age_s),
                ("机器人反馈", sample.feedback_time_s, self._limits.max_feedback_age_s),
            ):
                if not math.isfinite(stamp) or not 0 <= now - stamp <= max_age:
                    return reject(f"{label}时间戳过期或超前")
            if not sample.tracked or not sample.grip_held:
                return reject("手柄跟踪或握持丢失")
            if not sample.powered or not sample.enabled or sample.estop or sample.collision or sample.on_limit:
                return reject("机器人状态异常")
            if sample.tool_id != 1:
                return reject("当前工具不是 Tool 1")
            # 首帧从实测 TCP 比较，后续帧从上一已接受目标比较。
            # 超阈值直接拒绝，不把大跳变拆成多个貌似安全的小命令。
            position_jump, orientation_jump = _pose_jump(self._last_target or actual_pose, target)
            if position_jump > self._limits.max_position_jump_mm:
                return reject("目标位置跳变超限")
            if orientation_jump > self._limits.max_orientation_jump_deg:
                return reject("目标姿态跳变超限")
            # 以本次会话的第一帧实测 TCP 为固定锚点，防止许多小步累计越界。
            anchor_distance, anchor_rotation = _pose_jump(self._anchor_pose or actual_pose, target)
            if anchor_distance > self._limits.max_session_translation_mm:
                return reject("相对会话起点的位移越界")
            if anchor_rotation > self._limits.max_session_orientation_deg:
                return reject("相对会话起点的姿态越界")
        except (TypeError, ValueError, OverflowError) as error:
            return reject(f"输入无效：{error}")

        started = self._clock()
        try:
            result = sdk.kine_inverse(actual_joints, target)
        except Exception as error:
            return reject(f"厂商逆解调用异常：{type(error).__name__}")
        finished = self._clock()
        elapsed = finished - started
        if not math.isfinite(elapsed) or elapsed < 0 or elapsed > self._limits.max_inverse_elapsed_s:
            return reject("厂商逆解耗时超过本帧预算")
        # SDK 返回后再查一次：不能用“求解前还是新鲜的反馈”生成过期命令。
        if (finished - sample.input_time_s > self._limits.max_input_age_s
                or finished - sample.feedback_time_s > self._limits.max_feedback_age_s):
            return reject("厂商逆解完成时输入或反馈已经过期")
        if not isinstance(result, tuple) or len(result) < 2 or result[0] != 0:
            return reject(f"厂商逆解未成功：{result!r}")
        try:
            solved = _six_finite(result[1], "厂商逆解关节角")
        except (TypeError, ValueError, OverflowError) as error:
            return reject(f"厂商逆解输出无效：{error}")
        if max(abs(goal - measured) for goal, measured in zip(solved, actual_joints, strict=True)) > self._limits.max_joint_solution_jump_rad:
            return reject("厂商逆解关节目标相对实测值跳变超限")
        if (self._last_solution is not None
                and max(abs(goal - previous) for goal, previous in zip(solved, self._last_solution, strict=True))
                > self._limits.max_joint_solution_jump_rad):
            return reject("厂商逆解关节目标相对上一解跳变超限")

        if self._anchor_pose is None:
            self._anchor_pose = actual_pose
        self._last_target = target  # 仅离线记账；没有发送，也不代表实际运动完成。
        self._last_solution = solved
        return Decision(
            True,
            "离线候选已生成，未发送",
            ("servo_j", (solved, 0, 1)),  # ABS=0；一步=8 ms；严格使用本次 SDK 逆解结果。
            elapsed,
        )
