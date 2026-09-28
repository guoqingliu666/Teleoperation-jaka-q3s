"""候选版时间前瞻和增量厂商逆解筛查（只生成计划，不发送运动）。

架构边界：采样/调度是本项目代码；每一个关节解只能来自注入的 JAKA
``kine_inverse``。这个模块故意没有 linear_move/servo/使能/清报警接口。
控制柜实际圆滑曲线、关节加速度和奇异点安全尚须独立验证，不能用这里的
名义时间或假 SDK 测试结果宣称已经证明实机 C2。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time

from .sampled_follow import checked, six, RejectedPath
from .trajectory_reference import PoseSample, angle_deg, slerp, ordered_chord_fits
from .adaptive_sampling import AdaptiveSampling, select_keypoint


def interpolate_pose(a, b, fraction):
    """平移/旋转使用相同进度，避免分别截断改变手柄六维路径。"""
    if not 0 <= fraction <= 1:
        raise ValueError("进度必须在0到1之间")
    return PoseSample(
        a.t + (b.t - a.t) * fraction,
        tuple(x + (y - x) * fraction for x, y in zip(a.xyz, b.xyz)),
        slerp(a.q, b.q, fraction),
    )


@dataclass(frozen=True)
class PreviewProfile:
    """沿用已用过的上限作离线比较，不代表新的真机放行参数。"""

    speed_mm_s: float = 200.0
    acceleration_mm_s2: float = 500.0
    angular_speed_deg_s: float = 45.0
    max_translation_mm: float = 80.0
    max_rotation_deg: float = 4.0
    max_deviation_mm: float = 1.0
    input_timeout_s: float = 0.15
    backlog_limit_s: float = 0.4
    planning_budget_s: float = 0.12
    flush_s: float = 0.08

    def __post_init__(self):
        for value in self.__dict__.values():
            if not math.isfinite(value) or value <= 0:
                raise ValueError("前瞻参数必须为有限正数")
        if self.max_rotation_deg >= 90 or self.backlog_limit_s > 1:
            raise ValueError("禁止大角度跨步或长时间积压")


@dataclass(frozen=True)
class PreparedLeg:
    start: PoseSample
    target: PoseSample
    start_joints: tuple
    solutions: tuple
    created_s: float
    epoch: int
    duration_lower_bound_s: float
    minimum_joint_margin_deg: float
    largest_sample_step_deg: float
    shortened: bool
    source_target: PoseSample
    preparation_s: float
    # 自适应点不是等间隔，诊断必须使用真实路径进度。
    fractions: tuple = ()
    inverse_calls: int = 0
    subdivisions: int = 0
    # 保留最后一次逆解使用的原始RPY表达，执行端不再换成另一组等价欧拉角。
    command_tcp: tuple = ()
    # 中点已判定需要细分时，省去的待作废右端点调用数；不含任何跳过的检查点。
    avoided_endpoint_calls: int = 0
    completion_reason: str = "complete"

    def sample_times(self):
        """返回检查点的原始参考时间，不把非均匀细分点均匀摊到整段上。"""
        progress = self.fractions or tuple(
            (i + 1) / len(self.solutions) for i in range(len(self.solutions))
        )
        end = progress[-1]
        return tuple(
            self.start.t + (self.target.t - self.start.t) * u / end for u in progress
        )


class IncrementalInverse:
    """每次 advance 至多调用一次厂商逆解，把轮询/许可检查机会还给主循环。

    不在满队列时闲等；上层可以一边执行当前段，一边分次准备后续段。
    单次原生 SDK 调用仍可能阻塞，此设计不能中断已阻塞的本地库。
    邻点3°/距限位3°门槛保留；12°累计门槛触发时截取已检查的安全前缀，
    不重做整段逆解，也不为避停顿跳过风险点。它不是主动避奇异点的证明。
    """

    def __init__(
        self,
        inverse,
        start,
        desired,
        joints,
        limits_deg,
        *,
        epoch,
        profile=PreviewProfile(),
        clock=time.monotonic,
    ):
        self.inverse, self.start, self.profile, self.clock = (
            inverse,
            start,
            profile,
            clock,
        )
        self.started = clock()
        if desired.t <= start.t:
            raise ValueError("逆解候选时间必须晚于起点")
        self.epoch = epoch
        self.source_target = desired
        self.start_joints = self.reference = six(joints)
        self.limits = tuple(tuple(pair) for pair in limits_deg)
        if len(self.limits) != 6 or any(
            len(p) != 2 or not all(math.isfinite(x) for x in p) or p[0] >= p[1]
            for p in self.limits
        ):
            raise ValueError("需要六轴有效配置限位")
        self.minimum_margin = min(
            min(math.degrees(q) - lo, hi - math.degrees(q))
            for q, (lo, hi) in zip(self.reference, self.limits)
        )
        if self.minimum_margin < 3:
            raise RejectedPath("起点关节余量不足3°")
        distance, rotation = math.dist(start.xyz, desired.xyz), angle_deg(
            start.q, desired.q
        )
        fraction = min(
            1.0,
            profile.max_translation_mm / max(distance, 1e-12),
            profile.max_rotation_deg / max(rotation, 1e-12),
        )
        self.target = interpolate_pose(start, desired, fraction)
        self.count = max(
            1,
            math.ceil(math.dist(start.xyz, self.target.xyz) / 2.0),
            math.ceil(angle_deg(start.q, self.target.q) / 0.2),
        )
        self.index, self.solutions = 0, []
        self.calls = 0
        self.last_pose, self.near_rpy = start, start.tcp()[3:]
        self.largest_step = 0.0
        self.done = False

    def _finish(self, shortened=False):
        self.done = True
        target = self.last_pose
        lower = max(
            math.dist(self.start.xyz, target.xyz) / self.profile.speed_mm_s,
            angle_deg(self.start.q, target.q) / self.profile.angular_speed_deg_s,
        )
        return PreparedLeg(
            self.start,
            target,
            self.start_joints,
            tuple(self.solutions),
            self.clock(),
            self.epoch,
            lower,
            self.minimum_margin,
            self.largest_step,
            shortened,
            self.source_target,
            self.clock() - self.started,
            inverse_calls=self.calls,
            command_tcp=tuple(target.xyz) + tuple(self.near_rpy),
        )

    def advance(self):
        if self.done:
            raise RuntimeError("已完成/失效的逆解任务不能复用")
        try:
            if self.clock() - self.started > self.profile.planning_budget_s:
                raise RuntimeError("逆解准备超出时间预算；旧计划作废")
            pose = interpolate_pose(
                self.start, self.target, (self.index + 1) / self.count
            )
            tcp = pose.tcp(self.near_rpy)
            result = self.inverse(self.reference, tcp)
            self.calls += 1
            if self.clock() - self.started > self.profile.planning_budget_s:
                raise RuntimeError("逆解迟到；禁止使用迟到结果")
            if isinstance(result, tuple) and result and result[0] == -4:
                raise RejectedPath("厂商逆解不可达；不得跨过此点")
            solved = six(checked(result, "前瞻 kine_inverse"))
            step = max(abs(math.degrees(a - b)) for a, b in zip(solved, self.reference))
            if step > 3:
                axis = max(range(6), key=lambda i: abs(solved[i] - self.reference[i]))
                raise RejectedPath(
                    f"相邻厂商逆解变化超过3°；J{axis+1}变化{step:.6f}°；不得跨分支"
                )
            margin = min(
                min(math.degrees(q) - lo, hi - math.degrees(q))
                for q, (lo, hi) in zip(solved, self.limits)
            )
            if margin < 3:
                raise RejectedPath("候选关节余量不足3°")
            if (
                max(abs(math.degrees(a - b)) for a, b in zip(solved, self.start_joints))
                > 12
            ):
                if self.solutions:
                    return self._finish(shortened=True)
                raise RejectedPath("候选累计关节变化超过12°")
            self.index += 1
            self.solutions.append(solved)
            self.last_pose, self.reference, self.near_rpy = pose, solved, tcp[3:]
            self.minimum_margin = min(self.minimum_margin, margin)
            self.largest_step = max(self.largest_step, step)
            if self.index == self.count:
                return self._finish()
            return None
        except Exception:
            self.done = True
            raise


class TimeHorizon:
    """按时间管理未承诺目标和提前计算结果，而不是固定20mm/2°开关。

    本类仅提供规划候选，不能作为真机发送器。输入应是连续映射的参考采样，
    epoch 在 Grip 释放、源切换、追踪丢失时递增；旧计划永远不能跨 epoch。
    超出缓冲时间时明确阻断，不 silently 丢掉一圈旋转后追向最终朝向。
    """

    def __init__(
        self,
        inverse,
        limits_deg,
        *,
        profile=PreviewProfile(),
        clock=time.monotonic,
        adaptive=False,
        policy=AdaptiveSampling(),
    ):
        self.inverse, self.limits, self.profile, self.clock = (
            inverse,
            limits_deg,
            profile,
            clock,
        )
        self.adaptive, self.policy = adaptive, policy
        if adaptive and policy.pause_backlog_s >= profile.backlog_limit_s:
            raise ValueError("提前暂停必须早于积压硬上限")
        self.epoch = 0
        self.samples = deque()
        self.ready = self.job = None
        self.anchor = self.joints = None
        self.latest_received_s = None
        self.costs = deque(maxlen=100)
        self.blocked = False
        self.last_sample = None
        self.last_failure_context = None
        self.pressure = "normal"
        self.max_backlog_s = 0.0

    def reset(self, anchor, joints):
        self.epoch += 1
        self.samples.clear()
        self.job = self.ready = None
        self.anchor, self.joints = anchor, six(joints)
        self.latest_received_s = self.last_sample = None
        self.blocked = False
        self.last_failure_context = None
        self.pressure = "normal"
        self.max_backlog_s = 0.0

    def invalidate(self):
        self.last_failure_context = self.snapshot()
        self.epoch += 1
        self.job = self.ready = None
        self.samples.clear()
        self.blocked = True

    def snapshot(self):
        now = self.clock()
        return {
            "epoch": self.epoch,
            "buffered_samples": len(self.samples),
            "oldest_received_age_s": now - self.samples[0][1] if self.samples else None,
            "latest_input_age_s": (
                now - self.latest_received_s
                if self.latest_received_s is not None
                else None
            ),
            "planning_elapsed_s": now - self.job.started if self.job else None,
            "solved_samples": self.job.index if self.job else None,
            "required_samples": self.job.count if self.job else None,
            "sampling_mode": "adaptive" if self.adaptive else "fixed",
            "pressure": self.pressure,
            "max_backlog_s": self.max_backlog_s,
            "inverse_calls": getattr(self.job, "calls", None),
            "avoided_endpoint_calls": getattr(self.job, "avoided_endpoint_calls", 0),
        }

    def push(self, sample, received_s):
        if self.blocked or self.anchor is None:
            raise RuntimeError("前瞻器需要重新捕获")
        now = self.clock()
        if (
            not math.isfinite(received_s)
            or not 0 <= now - received_s <= self.profile.input_timeout_s
        ):
            self.invalidate()
            raise RuntimeError("拒绝过期/未来手柄样本")
        if not 0 <= received_s - sample.t <= self.profile.input_timeout_s:
            self.invalidate()
            raise RuntimeError("参考样本时间与接收时间不一致；禁止未来目标或旧轨迹")
        if self.last_sample is None and sample.t <= self.anchor.t:
            self.invalidate()
            raise RuntimeError("参考轨迹必须从捕获锚点之后开始")
        if self.last_sample is not None and (
            sample.t <= self.last_sample.t
            or sample.t - self.last_sample.t > self.profile.input_timeout_s
            or angle_deg(self.last_sample.q, sample.q) > 20
        ):
            self.invalidate()
            raise RuntimeError("时间/姿态不连续，需重新捕获")
        self.latest_received_s, self.last_sample = received_s, sample
        # 只丢弃完全静止样本，不丢掉亚毫米残量或累计旋转。
        previous = self.samples[-1][0] if self.samples else self.anchor
        if (
            math.dist(previous.xyz, sample.xyz) < 1e-8
            and angle_deg(previous.q, sample.q) < 1e-7
        ):
            return
        self.samples.append((sample, received_s))
        if (
            now - self.samples[0][1] > self.profile.backlog_limit_s
            or len(self.samples) > 128
        ):
            self.invalidate()
            raise RuntimeError("轨迹积压超出允许延迟；禁止补追历史")

    def _select_target(self):
        """按时间/弦误差选点；停手后的细小残量也在 flush 时间内可见。"""
        if not self.samples:
            return None
        newest = self.samples[-1][0]
        if self.adaptive:
            # 合并窗口从本批第一个未消费输入开始计时。anchor.t 可能是数秒前
            # 停手的位置；用它计时会让重新移动后的第一帧立即变成很短的运动段。
            # 仍使用真实接收时间，已等待的残量不能在这里重新获得有效期。
            target = select_keypoint(
                self.anchor,
                [p for p, _ in self.samples],
                max_translation_mm=self.profile.max_translation_mm,
                max_rotation_deg=self.profile.max_rotation_deg,
                policy=self.policy,
            )
            window_elapsed = self.clock() - self.samples[0][1]
            at_span_limit = (
                math.dist(self.anchor.xyz, target.xyz) >= self.profile.max_translation_mm
                or angle_deg(self.anchor.q, target.q) >= self.profile.max_rotation_deg
            )
            # 已看到不能合并的转折/折返，或达到段长上限，就提前准备该段。
            # 平滑且很短的输入最多合并 flush_s，静止末端的小残量也会得到处理。
            if window_elapsed < self.profile.flush_s and target == newest and not at_span_limit:
                return None
            return target
        if (
            newest.t - self.anchor.t < self.profile.flush_s
            and self.clock() - self.samples[0][1] < self.profile.flush_s
        ):
            return None
        target = self.samples[0][0]
        for candidate, _ in self.samples:
            if candidate.t - self.anchor.t <= 0:
                continue
            if (
                math.dist(self.anchor.xyz, candidate.xyz)
                > self.profile.max_translation_mm
                or angle_deg(self.anchor.q, candidate.q) > self.profile.max_rotation_deg
            ):
                break
            intermediate = [p for p, _ in self.samples if p.t < candidate.t]
            if not ordered_chord_fits(
                intermediate,
                self.anchor,
                candidate,
                self.profile.max_deviation_mm,
                0.25,
            ):
                break
            target = candidate
            if candidate.t - self.anchor.t >= self.profile.flush_s:
                break
        return target

    def advance(self, *, permit):
        """执行端即使已有两段，本函数也能先准备下一段；每轮最多一次 IK。"""
        if not permit:
            self.invalidate()
            return
        if self.blocked:
            raise RuntimeError("前瞻已阻断，需重新捕获")
        now = self.clock()
        if self.latest_received_s is None:
            return
        if now - self.latest_received_s > self.profile.input_timeout_s:
            self.invalidate()
            raise RuntimeError("手柄输入断流")
        if self.samples and now - self.samples[0][1] > self.profile.backlog_limit_s:
            self.invalidate()
            raise RuntimeError("前瞻轨迹已过期")
        age = now - self.samples[0][1] if self.samples else 0.0
        self.max_backlog_s = max(self.max_backlog_s, age)
        self.pressure = (
            "compress"
            if self.adaptive and age >= self.policy.soft_backlog_s
            else "normal"
        )
        if self.adaptive and age >= self.policy.pause_backlog_s:
            # 先尝试交付一段已完整检查的前缀，消化输入而不丢掉折返、转圈或
            # 未检查尾段。若当前没有这样的边界，才保留原有的暂停策略。
            try:
                prefix = getattr(self.job, "completed_prefix", lambda _reason: None)(
                    "backlog_slice"
                )
            except Exception:
                self.invalidate()
                raise
            if prefix is not None:
                self.ready, self.job = prefix, None
                self.costs.append(prefix.preparation_s)
                self.pressure = "drain"
                return
            # 原始点已经无法及时消化，提前暂停并要求重捕获，不补追过期历史。
            self.pressure = "pause"
            self.invalidate()
            raise RejectedPath("动态轨迹积压达到提前暂停门槛；松Grip后重新捕获")
        if self.ready is not None:
            return
        try:
            if self.job is None:
                target = self._select_target()
                if target is None:
                    return
                from .adaptive_inverse import AdaptiveInverse

                factory = AdaptiveInverse if self.adaptive else IncrementalInverse
                options = {"policy": self.policy} if self.adaptive else {}
                self.job = factory(
                    self.inverse,
                    self.anchor,
                    target,
                    self.joints,
                    self.limits,
                    epoch=self.epoch,
                    profile=self.profile,
                    clock=self.clock,
                    **options,
                )
            result = self.job.advance()
            if result is not None:
                self.costs.append(self.clock() - self.job.started)
                self.ready, self.job = result, None
        except Exception:
            self.invalidate()
            raise

    def replenishment_threshold_s(self):
        """使用近期最坏准备耗时加余量；不是假设所有段都一样长。"""
        return min(
            self.profile.backlog_limit_s,
            max(self.profile.flush_s, 2 * max(self.costs, default=0.0) + 0.03),
        )

    def take_preview(self, remaining_execution_s, *, permit, expected_epoch):
        """交付离线/只读候选，不发送。remaining 必须由执行端可靠提供。

        不能用名义路程/速度冒充剩余执行时间；未知值(None)不放行。
        接收者必须继续验证实际队列、起点、轨迹误差和下发前许可。
        """
        if not permit or expected_epoch != self.epoch:
            self.invalidate()
            return None
        if self.blocked or self.ready is None or remaining_execution_s is None:
            return None
        if not math.isfinite(remaining_execution_s) or remaining_execution_s < 0:
            raise ValueError("剩余执行时间必须为有限非负数")
        now = self.clock()
        if (
            self.latest_received_s is None
            or now - self.latest_received_s > self.profile.input_timeout_s
            or now - self.ready.created_s > self.profile.input_timeout_s
            or (
                self.samples and now - self.samples[0][1] > self.profile.backlog_limit_s
            )
        ):
            self.invalidate()
            return None
        if remaining_execution_s > self.replenishment_threshold_s():
            return None
        result, self.ready = self.ready, None
        self.anchor, self.joints = result.target, result.solutions[-1]
        # 这些中间点已被几何检查批准用同一条弦替代。截短后应继续该弦的
        # 剩余部分，不能按线性插值时间又回头追原始非匀速中间点。
        # 剩余部分仍须重新调用厂商IK检查；不把尚未解算的部分视为已通过。
        oldest_received = self.samples[0][1] if self.samples else result.created_s
        while self.samples and self.samples[0][0].t <= result.source_target.t:
            self.samples.popleft()
        if result.target.t < result.source_target.t:
            self.samples.appendleft((result.source_target, oldest_received))
        return result

    def take_when_slot_open(self, *, expected_epoch):
        """执行适配器已读回预排槽空闲后交付候选。

        0只用来绕过本类的时间等待分支，不表示控制柜真实剩余时间为零。
        接口名称把这个条件与只读虚拟消费区分开，调用方必须先验证队列状态。
        """
        return self.take_preview(0.0, permit=True, expected_epoch=expected_epoch)


def corner_speed_preview(previous, corner, following, profile=PreviewProfile()):
    """几何建议而非 JAKA 圆滑保证；反向转折不能假装保持非零速度。

    以允许切角偏差约束相切圆弧，再由 sqrt(a*r) 估计法向速度上界。
    纯姿态/零长度没有毫米半径意义，必须由厂商核查其姿态圆滑语义。
    """
    a = tuple(y - x for x, y in zip(previous.xyz, corner.xyz))
    b = tuple(y - x for x, y in zip(corner.xyz, following.xyz))
    la, lb = math.hypot(*a), math.hypot(*b)
    if min(la, lb) < 1e-8:
        return {"radius_mm": None, "speed_mm_s": None, "reason": "纯姿态圆滑需厂商确认"}
    theta = math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)) / (la * lb))))
    if theta < 1e-6:
        return {"radius_mm": 0.0, "speed_mm_s": profile.speed_mm_s, "reason": "近直线"}
    if math.pi - theta < 1e-3:
        return {"radius_mm": 0.0, "speed_mm_s": 0.0, "reason": "反向必须连续减速"}
    radius = min(
        0.45 * min(la, lb) / math.tan(theta / 2),
        profile.max_deviation_mm / (1 / math.cos(theta / 2) - 1),
    )
    return {
        "radius_mm": radius,
        "speed_mm_s": min(
            profile.speed_mm_s, math.sqrt(profile.acceleration_mm_s2 * radius)
        ),
        "reason": "仅几何上界，未核实厂商执行曲线",
    }
