"""新轨迹只读集成：真实手柄 → 连续旋转历史 → 时间前瞻 → 厂商IK。

只读影子把已解出的下一点当作“虚拟规划起点”，实机必须保持静止。
这允许检查完整旋转路径的逆解连续性，不会以静止的实测关节反复解所有远端
姿态，更不会把虚拟规划关节广播成数字孪生的实测关节。
"""

from __future__ import annotations

import math
import time

from .quest_vr_input import RelativeQuestTracker, matmul, matrix_rpy, rpy_matrix
from .sampled_follow import measured
from .trajectory_reference import PoseSample, StreamingReference, rpy_quaternion
from .trajectory_horizon import TimeHorizon, PreviewProfile
from .intent_horizon import IntentHorizon
from .trajectory_dynamics import JointTimingAudit
from .adaptive_sampling import AdaptiveSampling


class TrajectoryShadow:
    def __init__(
        self,
        robot,
        mapping,
        limits,
        *,
        emit,
        clock=time.monotonic,
        adaptive=True,
        policy=AdaptiveSampling(),
        profile=PreviewProfile(),
        rotation_radius_deg=180.,
        position_radius_mm=1000.,
    ):
        self.robot, self.emit, self.clock = robot, emit, clock
        self.tracker = RelativeQuestTracker(
            mapping, grip_on=0.75, grip_off=0.55, trigger_on=0.75, trigger_off=0.55
        )
        self.reference = StreamingReference()
        # 只把逆解方法交给前瞻器，不把拥有运动方法的RC对象传进去。
        self.horizon = (IntentHorizon if adaptive else TimeHorizon)(
            robot.kine_inverse, limits, clock=clock, adaptive=adaptive, policy=policy,
            profile=profile,
            **({"rotation_radius_deg": rotation_radius_deg,
                "position_radius_mm": position_radius_mm} if adaptive else {})
        )
        self.heading = self.release_seen = False
        self.anchor = self.source = self.initial_joints = self.last_processed = None
        self.last_position = None
        self.next_read = 0.0
        self.candidates = self.blocks = self.strokes = 0
        self.max_twist_deg = 0.0
        self.max_arc_deg = 0.0
        self.timing = JointTimingAudit(
            time_basis="hand_sample_time_not_controller_time"
        )
        self.sampling_summary = {
            "accepted_check_points": 0,
            "accepted_plan_ik_calls": 0,
            "accepted_plan_subdivisions": 0,
            "avoided_endpoint_calls": 0,
            "planning_slice_plans": 0,
            "backlog_slice_plans": 0,
            "max_backlog_s": 0.0,
            "max_preparation_s": 0.0,
            "limited_frames": 0,
            "max_pending_nominal_s": 0.0,
        }

    def record_backlog(self, snapshot):
        """保留整次会话的峰值；松Grip清空规划不会把诊断峰值也清零。"""
        age = (
            snapshot.get("max_backlog_s")
            or snapshot.get("oldest_received_age_s")
            or 0.0
        )
        self.sampling_summary["max_backlog_s"] = max(
            self.sampling_summary["max_backlog_s"], age
        )
        self.sampling_summary["max_pending_nominal_s"] = max(
            self.sampling_summary["max_pending_nominal_s"], snapshot.get("pending_nominal_s", 0.))
        # 逐窗口计数的累加在 release 前处理；这里保留实时值供 UI 使用。
        self.sampling_summary["current_limited_frames"] = snapshot.get("limited_frames", 0)

    def release(self, reason):
        if self.anchor is not None:
            self.sampling_summary["limited_frames"] += getattr(self.horizon, "limited_frames", 0)
        self.sampling_summary["current_limited_frames"] = 0
        if self.timing.count:
            self.emit(state="trajectory_joint_timing", **self.timing.report())
        self.timing.reset()
        self.horizon.invalidate()
        self.reference.reset()
        self.tracker.reset_grip()
        self.anchor = self.source = self.initial_joints = self.last_position = None
        self.release_seen = False
        self.emit(
            state="trajectory_preview_paused", message=reason, movement_commands_sent=0
        )

    def process(self, frame):
        now = self.clock()
        valid = bool(
            frame
            and frame.connected
            and frame.tracked
            and frame.valid
            and frame.rotation_valid
            and not frame.button_b
            and 0 <= now - frame.received_s <= 0.15
        )
        if not valid:
            if self.anchor is not None:
                self.release("追踪失效或输入过期；清空旧轨迹")
            self.release_seen = False
            return
        if frame.grip <= 0.55:
            if self.anchor is not None:
                self.release("Grip松开；旧规划不能跨Grip使用")
            self.release_seen = True
            if not self.heading and frame.head_rotation_valid:
                self.tracker.lock_heading(frame.head_rotation_xyzw)
                self.heading = True
            return
        if self.source is not None and frame.udp_source != self.source:
            self.release("输入来源改变；重新释放Grip")
            return
        if self.anchor is None:
            if not self.release_seen or not self.heading or frame.grip < 0.75:
                return
            joints, tcp = measured(self.robot)
            if self.clock() - frame.received_s > 0.15:
                self.release_seen = False
                return
            self.anchor, self.initial_joints = tcp, joints
            self.source = frame.udp_source
            self.reference.reset()
            self.tracker.reset_grip()
            self.horizon.reset(PoseSample.from_tcp(frame.received_s, tcp), joints)
            self.timing.reset()
            self.timing.push(frame.received_s, joints)
            self.last_processed = None
            self.release_seen = False
            self.strokes += 1
            self.emit(
                state="trajectory_preview_anchor",
                anchor_tcp=tcp,
                measured_joints=joints,
                epoch=self.horizon.epoch,
                reference_position_m=frame.position_m,
                reference_rotation_xyzw=frame.rotation_xyzw,
                mapping=self.tracker._axis_map,
                heading=self.tracker._heading_frame,
            )
        try:
            if now >= self.next_read:
                joints, _ = measured(self.robot)
                self.next_read = self.clock() + 0.1
                if (
                    max(
                        abs(math.degrees(a - b))
                        for a, b in zip(joints, self.initial_joints)
                    )
                    > 0.2
                ):
                    raise RuntimeError("只读影子期间实机发生运动，不能继续使用静止锚点")
            if frame.received_s != self.last_processed:
                if (
                    self.last_position is not None
                    and math.dist(frame.position_m, self.last_position) > 0.1
                ):
                    raise ValueError("手柄单帧位置跳变超过10cm")
                self.last_position = frame.position_m
                self.last_processed = frame.received_s
                for name, value in self.tracker.update(frame, 1.0):
                    if name != "pose_delta" or value[1] is None:
                        continue
                    delta, rotation = value
                    rotation = matmul(rotation, rpy_matrix(self.anchor[3:]))
                    p = PoseSample(
                        frame.received_s,
                        tuple(self.anchor[i] + delta[i] for i in range(3)),
                        rpy_quaternion(matrix_rpy(rotation)),
                    )
                    # V1.5 的厂商接口只接收关键位姿和运动参数，不执行五次曲线
                    # 系数。动态只读与动态真机共用原始位姿→关键点→厂商IK链路，
                    # 避免每帧拟合一条不会下发的曲线，并额外等待下一帧。
                    # 连续旋转历史仍逐帧检查；旧固定密度模式保留曲线研究对照。
                    span = None
                    if self.horizon.adaptive:
                        p = self.reference.history.push(p)
                    else:
                        span = self.reference.push(p)
                    self.max_twist_deg = max(
                        self.max_twist_deg, abs(self.reference.history.twist_deg)
                    )
                    self.max_arc_deg = max(
                        self.max_arc_deg, self.reference.history.arc_deg
                    )
                    # 保存未45°裁剪的数据。完整圈数不能从旧限幅target恢复。
                    self.emit(
                        state="trajectory_raw_sample",
                        t=p.t,
                        xyz=p.xyz,
                        q=p.q,
                        received_s=frame.received_s,
                        epoch=self.horizon.epoch,
                        twist_deg=self.reference.history.twist_deg,
                        arc_deg=self.reference.history.arc_deg,
                    )
                    if self.horizon.adaptive:
                        # 捕获帧仅建立起点，不把同时间戳再插入待规划队列。
                        if p.t > self.horizon.anchor.t:
                            self.horizon.push(p, frame.received_s)
                    elif span is not None:
                        bounds = span.bounds()
                        # 已承诺结点不能为修后段而反改其导数。无法满足偏差预算就
                        # 阻断本次影子，留日志供重采样策略改进；不偷偷跨段拼接。
                        if (
                            bounds["position_deviation_mm"] > 1.0
                            or bounds["orientation_deviation_deg"] > 0.25
                        ):
                            raise ValueError("在线曲线偏差预算不足；不得修改已承诺导数")
                        endpoint = span.evaluate(span.right.t).pose
                        self.horizon.push(endpoint, frame.received_s)
                        self.emit(
                            state="trajectory_reference_span",
                            bounds=bounds,
                            source_start_s=span.left.t,
                            source_end_s=span.right.t,
                        )
            self.horizon.advance(permit=True)
            self.record_backlog(self.horizon.snapshot())
            # 0仅表示只读的虚拟消费者不等待实机，不能用于评估真机队列是否断供。
            plan = self.horizon.take_preview(
                0.0, permit=True, expected_epoch=self.horizon.epoch
            )
            if plan is not None:
                self.candidates += 1
                self.sampling_summary["accepted_check_points"] += len(plan.solutions)
                self.sampling_summary["accepted_plan_ik_calls"] += plan.inverse_calls
                self.sampling_summary["accepted_plan_subdivisions"] += plan.subdivisions
                self.sampling_summary["avoided_endpoint_calls"] += plan.avoided_endpoint_calls
                self.sampling_summary["planning_slice_plans"] += int(plan.completion_reason == "planning_slice")
                self.sampling_summary["backlog_slice_plans"] += int(plan.completion_reason == "backlog_slice")
                self.sampling_summary["max_preparation_s"] = max(
                    self.sampling_summary["max_preparation_s"], plan.preparation_s
                )
                # 与增量IK相同的采样进度；截短前缀不能拉伸到原始整段时长。
                for t, joints in zip(plan.sample_times(), plan.solutions):
                    self.timing.push(t, joints)
                self.emit(
                    state="trajectory_vendor_preview",
                    target=plan.target.tcp(),
                    start_tcp=plan.start.tcp(),
                    source_start_s=plan.start.t,
                    source_end_s=plan.target.t,
                    source_target_s=plan.source_target.t,
                    preparation_s=plan.preparation_s,
                    planning_state=self.horizon.snapshot(),
                    solutions=plan.solutions,
                    minimum_joint_margin_deg=plan.minimum_joint_margin_deg,
                    largest_sample_step_deg=plan.largest_sample_step_deg,
                    shortened=plan.shortened,
                    completion_reason=plan.completion_reason,
                    epoch=plan.epoch,
                    sample_fractions=plan.fractions,
                    inverse_calls=plan.inverse_calls,
                    subdivisions=plan.subdivisions,
                    avoided_endpoint_calls=plan.avoided_endpoint_calls,
                    joint_timing=self.timing.report(),
                    simulated_consumer=True,
                    movement_commands_sent=0,
                )
        except (ValueError, RuntimeError) as error:
            self.blocks += 1
            self.record_backlog(
                self.horizon.last_failure_context or self.horizon.snapshot()
            )
            self.emit(
                state="trajectory_block_detail",
                message=str(error),
                input_discontinuity=self.reference.history.last_failure,
                planning_state=self.horizon.last_failure_context
                or self.horizon.snapshot(),
            )
            self.release(str(error))
            # SDK -3等异常不是普通路径拒绝；留给会话退出处理。
            from .sampled_follow import RejectedPath

            if isinstance(error, RuntimeError) and not isinstance(error, RejectedPath):
                raise
