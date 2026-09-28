"""逐区间自适应检查：关节解全部来自注入的 JAKA kine_inverse。

本模块没有自编逆解或运动接口。少量点检查正常不等于连续路径安全；
输出仍须经过执行端的起点、许可、反馈与厂商运动规划检查。
"""

from collections import deque
from dataclasses import replace
import math

from .adaptive_sampling import AdaptiveSampling
from .sampled_follow import RejectedPath, checked, six
from .trajectory_horizon import IncrementalInverse, interpolate_pose
from .trajectory_reference import angle_deg


class AdaptiveInverse(IncrementalInverse):
    """每次 advance 最多一次 SDK 调用，把停止检查机会交还主循环。

    先依次求中点、右端点；关节变化不够平稳时二分当前区间。加入新前驱解后，
    下游旧解重新计算，不能拿不同参考种子的缓存直接拼成一条关节路径。
    """

    def __init__(self, *args, policy=AdaptiveSampling(), cooperative=False, **kwargs):
        super().__init__(*args, **kwargs)
        # 新调度把任务拆成多轮计算，但不因此拆成多个运动命令。旧诊断模式
        # 保留墙钟预算用于回归对照；cooperative 模式单独约束每次 SDK 调用。
        self.cooperative = cooperative
        self.sdk_elapsed_s = 0.0
        self.risk_detail = None
        self.policy = policy
        distance = math.dist(self.start.xyz, self.target.xyz)
        rotation = angle_deg(self.start.q, self.target.q)
        # 每个区间有中点检查，最终相邻已检查点的间距不超过配置值。
        chunks = max(
            1,
            math.ceil(distance / (2 * policy.max_check_mm)),
            math.ceil(rotation / (2 * policy.max_check_deg)),
        )
        self.pending_intervals = deque(((i + 1) / chunks, 0) for i in range(chunks))
        self.progress = 0.0
        self.fractions = []
        self.midpoint = None
        self.calls = self.subdivisions = 0
        self.avoided_endpoint_calls = 0
        self._at_checked_boundary = False
        self.completion_reason = "complete"
        self.count = chunks * 2
        if self.minimum_margin < policy.minimum_margin_deg:
            raise RejectedPath("动态逆解起点关节余量不足")

    def _finish(self, shortened=False):
        result = super()._finish(shortened)
        return replace(
            result,
            fractions=tuple(self.fractions),
            inverse_calls=self.calls,
            subdivisions=self.subdivisions,
            avoided_endpoint_calls=self.avoided_endpoint_calls,
            completion_reason=self.completion_reason,
        )

    def _budget_prefix_ready(self):
        """在完整区间通过后主动交付，给后续反馈与排队预留时间。

        采用原硬预算的一半作为软交付时刻；不延长120ms硬上限。只交付已完成
        中点、右端点和曲率检查的前缀，不能在发现风险的细分途中或迟到后放行。
        外层仍保留原始剩余目标及年龄，后续检查不能跨过剩余路径。
        """
        return (not self.cooperative and self._at_checked_boundary and bool(self.solutions)
                and self.clock() - self.started >= self.profile.planning_budget_s * .5)

    def completed_prefix(self, reason):
        """交付当前已完整检查的前缀，供积压恢复使用。

        不能从中点、细分途中或空结果交付；因此不会把“正在检查”的一半路径
        误标记为可执行。调用方仍需按既有规则检查候选年龄、许可和队列状态。
        """
        if self.done:
            raise RuntimeError("已结束的任务不能再次交付前缀")
        if self.clock() - self.started > self.profile.planning_budget_s:
            self.done = True
            raise RuntimeError("前缀任务超出时间预算；不得重新获得有效期")
        if not self._at_checked_boundary or not self.solutions:
            return None
        self.completion_reason = reason
        return self._finish(shortened=True)

    def _midpoint_requires_refinement(self, pose, solved):
        """中点已足以判定要细分时，省去必然作废的右端点逆解。

        只提前识别原有规则已经能确定的风险；中点正常仍须计算右端点，
        并检查两半步长与中点曲率。没有使用插值关节替代厂商逆解。
        """
        step = max(abs(math.degrees(b - a)) for a, b in zip(self.reference, solved))
        margin = min(min(math.degrees(q) - lo, hi - math.degrees(q))
                     for q, (lo, hi) in zip(solved, self.limits))
        left = interpolate_pose(self.start, self.target, self.progress)
        distance = math.dist(left.xyz, pose.xyz)
        rotation = angle_deg(left.q, pose.q)
        fine = (distance <= self.policy.fine_check_mm
                and rotation <= self.policy.fine_check_deg)
        normalized = max(distance / self.policy.max_check_mm,
                         rotation / self.policy.max_check_deg, 1e-9)
        hard = step > self.policy.joint_step_deg or margin < self.policy.minimum_margin_deg
        detail = ((step / normalized > self.policy.joint_gain_deg
                   and step > self.policy.gain_resolved_step_deg)
                  or (margin < self.policy.near_limit_deg
                      and (distance > 2.0 or rotation > 0.2)))
        self._record_risk((self.reference, solved), None, "midpoint")
        return hard or (detail and not fine), fine

    def _record_risk(self, vectors, curvature, phase):
        """记录原有检查的实测数值，不新增门槛，也不改变放行结果。

        轴号从1开始。中点阶段尚未计算右端点，曲率必须记为空，不能写成零。
        """
        steps = [max(abs(math.degrees(b[i]-a[i]))
                     for a,b in zip(vectors,vectors[1:])) for i in range(6)]
        margins = [min(min(math.degrees(q[i])-self.limits[i][0],
                           self.limits[i][1]-math.degrees(q[i]))
                       for q in vectors[1:]) for i in range(6)]
        reasons = []
        if max(steps) > self.policy.joint_step_deg:
            reasons.append("joint_step")
        if min(margins) < self.policy.minimum_margin_deg:
            reasons.append("joint_margin")
        if curvature is not None and max(curvature) > self.policy.joint_curve_error_deg:
            reasons.append("joint_curvature")
        self.risk_detail = dict(phase=phase, reasons=reasons,
            joint_steps_deg=steps, joint_margins_deg=margins,
            largest_step_axis=steps.index(max(steps))+1,
            smallest_margin_axis=margins.index(min(margins))+1,
            joint_curvature_deg=curvature,
            step_limit_deg=self.policy.joint_step_deg,
            minimum_margin_deg=self.policy.minimum_margin_deg,
            curvature_limit_deg=self.policy.joint_curve_error_deg)

    def _subdivide(self, right, middle, depth, fine):
        """替换待查区间，保持从左到右的种子顺序；不接纳任何未检查前缀。"""
        # 某个已求解点已经侵入关节余量时，缩小检查间距不会使这个结果
        # 合格。直接拒绝当前候选，避免沿同一风险路径继续花费多轮逆解。
        # 这不是绕障，也不把风险之前的前缀偷偷作为运动命令下发。
        if self.risk_detail and "joint_margin" in self.risk_detail["reasons"]:
            axis = self.risk_detail["smallest_margin_axis"]
            margin = self.risk_detail["joint_margins_deg"][axis-1]
            self.risk_detail.update(depth=depth, fine_resolution_reached=fine,
                left_fraction=self.progress, right_fraction=right,
                early_margin_rejection=True)
            raise RejectedPath(f"厂商逆解J{axis}余量{margin:.3f}°不足"
                               f"{self.policy.minimum_margin_deg:g}°；本段未放行")
        if fine or depth >= self.policy.max_depth:
            self.risk_detail.update(depth=depth, fine_resolution_reached=fine,
                                    left_fraction=self.progress, right_fraction=right)
            raise RejectedPath("自适应细分后关节跳变/余量/曲率风险仍存在；本段未放行")
        self.pending_intervals.popleft()
        self.pending_intervals.appendleft((right, depth + 1))
        self.pending_intervals.appendleft((middle, depth + 1))
        self.midpoint = None
        self.subdivisions += 1
        self.count = self.index + 2 * len(self.pending_intervals)

    def advance(self):
        if self.done:
            raise RuntimeError("已完成/失效的动态逆解任务不能复用")
        try:
            if not self.cooperative and self.clock() - self.started > self.profile.planning_budget_s:
                raise RuntimeError("动态逆解准备超出时间预算")
            if self._budget_prefix_ready():
                self.completion_reason = "planning_slice"
                return self._finish(shortened=True)
            if self.calls >= self.policy.max_calls:
                raise RejectedPath("动态逆解细分达到计算次数上限；本段未放行")
            right, depth = self.pending_intervals[0]
            middle = (self.progress + right) / 2
            u = middle if self.midpoint is None else right
            pose = interpolate_pose(self.start, self.target, u)
            seed = self.reference if self.midpoint is None else self.midpoint[1]
            near = self.near_rpy if self.midpoint is None else self.midpoint[2][3:]
            tcp = pose.tcp(near)
            self._at_checked_boundary = False
            call_started = self.clock()
            result = self.inverse(seed, tcp)
            call_elapsed = self.clock() - call_started
            self.sdk_elapsed_s += call_elapsed
            self.calls += 1
            if (call_elapsed > self.profile.planning_budget_s or
                    (not self.cooperative and
                     self.clock() - self.started > self.profile.planning_budget_s)):
                raise RuntimeError("动态逆解迟到；禁止使用返回结果")
            if isinstance(result, tuple) and result and result[0] == -4:
                raise RejectedPath("厂商逆解不可达；本段未放行")
            solved = six(checked(result, "动态 kine_inverse"))
            if self.midpoint is None:
                refine, fine = self._midpoint_requires_refinement(pose, solved)
                if refine:
                    self.avoided_endpoint_calls += 1
                    self._subdivide(right, middle, depth, fine)
                    return None
                self.midpoint = (pose, solved, tcp)
                return None

            mid_pose, mid_q, mid_tcp = self.midpoint
            steps = [
                max(abs(math.degrees(b - a)) for a, b in zip(left_q, right_q))
                for left_q, right_q in ((self.reference, mid_q), (mid_q, solved))
            ]
            margin = min(
                min(math.degrees(q) - lo, hi - math.degrees(q))
                for vector in (mid_q, solved)
                for q, (lo, hi) in zip(vector, self.limits)
            )
            curvature = max(
                abs(math.degrees(m - (a + b) / 2))
                for a, m, b in zip(self.reference, mid_q, solved)
            )
            left_pose = interpolate_pose(self.start, self.target, self.progress)
            distance = math.dist(left_pose.xyz, pose.xyz) / 2
            rotation = angle_deg(left_pose.q, pose.q) / 2
            fine = (
                distance <= self.policy.fine_check_mm
                and rotation <= self.policy.fine_check_deg
            )
            normalized_motion = max(
                distance / self.policy.max_check_mm,
                rotation / self.policy.max_check_deg,
                1e-9,
            )
            hard_risk = (
                max(steps) > self.policy.joint_step_deg
                or margin < self.policy.minimum_margin_deg
                or curvature > self.policy.joint_curve_error_deg
            )
            # 放大比是局部斜率，二分通常不能降低它。已把实际关节检查步长
            # 压到0.25°后，交给曲率、余量和跳变检查继续判断，不再只因斜率
            # 数值高而递归到底。这不等于控制柜的关节速度/加速度已经获验证。
            needs_detail = (max(steps) / normalized_motion > self.policy.joint_gain_deg
                            and max(steps) > self.policy.gain_resolved_step_deg) or (
                margin < self.policy.near_limit_deg
                and (distance > 2.0 or rotation > 0.2)
            )
            if hard_risk or (needs_detail and not fine):
                self._record_risk((self.reference, mid_q, solved),
                    [abs(math.degrees(m-(a+b)/2))
                     for a,m,b in zip(self.reference,mid_q,solved)], "interval")
                self._subdivide(right, middle, depth, fine)
                return None

            # 只有区间检查通过才接纳两点。总关节跨度超限时只能返回已检查前缀，
            # 外层保留原始目标及接收年龄，不通过刷新时间戳隐藏积压。
            for accepted_u, accepted_pose, q, accepted_tcp, step in (
                (middle, mid_pose, mid_q, mid_tcp, steps[0]),
                (right, pose, solved, tcp, steps[1]),
            ):
                if (
                    max(abs(math.degrees(b - a)) for a, b in zip(self.start_joints, q))
                    > self.policy.joint_segment_deg
                ):
                    if self.solutions:
                        self.completion_reason = "joint_span"
                        return self._finish(shortened=True)
                    raise RejectedPath("首个检查点已超过本段关节跨度")
                self.solutions.append(q)
                self.fractions.append(accepted_u)
                self.index += 1
                self.reference, self.last_pose, self.near_rpy = (
                    q,
                    accepted_pose,
                    accepted_tcp[3:],
                )
                self.largest_step = max(self.largest_step, step)
                self.minimum_margin = min(
                    self.minimum_margin,
                    min(
                        min(math.degrees(v) - lo, hi - math.degrees(v))
                        for v, (lo, hi) in zip(q, self.limits)
                    ),
                )
            self.progress = right
            self.pending_intervals.popleft()
            self.midpoint = None
            self._at_checked_boundary = True
            if not self.pending_intervals:
                return self._finish()
            return None
        except Exception:
            self.done = True
            raise
