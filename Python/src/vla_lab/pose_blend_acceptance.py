"""JAKA 厂商六维队列圆滑的隔离验收核心。

此模块只验证 ``linear_move_extend_ori`` 的厂商队列衔接，不实现自编伺服、
自编逆解或无限连续跟随。路径固定为基坐标 ``+Z -> +X`` 两段，每段 30 mm，
同时每段增加 3° 的基坐标 Z 轴姿态变化。第一段 ``tol=5 mm``，末段
``tol=0`` 精确停止；队列最多只有两条命令。

每段发送前仍按 2 mm / 0.2° 采样调用 JAKA ``kine_inverse``，沿用六维跟随
已经验证的相邻关节、累计关节变化和关节余量门槛。任何许可撤销、队列异常、
实测关节偏离已检查包络或超时都会调用厂商 ``motion_abort`` 并确认队列清空。

本模块还提供有限滚动队列：控制柜中最多一条执行段和一条预排段，预排槽
空出时只采用最新手柄目标，不补跑历史采样；首轮真机门槛最多十段。

以上固定采样描述适用于历史验收。动态采样入口位于 adaptive_pose_queue.py，
复用本模块的队列读回、停止确认与唯一厂商下发方法；新旧策略分开选择。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

from .blend_acceptance import abort_and_confirm, motion_status, require_no_motion_fault
from .bounded_pose_follow import RejectedPath, plan_pose_segment
from .orientation_acceptance import _pose_rotation_error_deg, _unwrap
from .quest_vr_input import matmul, matrix_rpy, rpy_matrix
from .sampled_follow import Settings, check_health, checked, measured


@dataclass(frozen=True)
class PoseBlendProfile:
    segment_mm: float = 30.0
    orientation_step_deg: float = 3.0
    speed_mm_s: float = 150.0
    acceleration_mm_s2: float = 400.0
    blend_tolerance_mm: float = 5.0
    orientation_speed_deg_s: float = 30.0
    orientation_acceleration_deg_s2: float = 120.0


# 手柄两端点验收沿用已通过的长段上限。它不是固定方向测试；每一段仍由
# ``plan_pose_segment`` 限制为 60 mm / 6°，且整个目标已由手柄跟随器限制在
# 启动 TCP 周围 1000 mm / 30°。
HAND_TWO_POINT = PoseBlendProfile(
    segment_mm=60.0,
    orientation_step_deg=6.0,
    speed_mm_s=150.0,
    acceleration_mm_s2=400.0,
    blend_tolerance_mm=5.0,
    orientation_speed_deg_s=30.0,
    orientation_acceleration_deg_s2=120.0,
)


@dataclass(frozen=True)
class PoseBlendPlan:
    profile: PoseBlendProfile
    start_joints: tuple[float, ...]
    start_tcp: tuple[float, ...]
    corner_tcp: tuple[float, ...]
    final_tcp: tuple[float, ...]
    first_solutions: tuple[tuple[float, ...], ...]
    second_solutions: tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class QueuedPoseSegment:
    """一条已经通过厂商逆解筛查、可进入控制柜队列的六维段。"""
    start_joints: tuple[float, ...]
    start_tcp: tuple[float, ...]
    target_tcp: tuple[float, ...]
    solutions: tuple[tuple[float, ...], ...]
    tolerance_mm: float


def _rotated_about_base_z(tcp, degrees: float):
    """在基坐标系 Z 轴上增加小角度，并把结果连续地表达为 RPY。"""
    delta = rpy_matrix((0.0, 0.0, math.radians(degrees)))
    rotation = matmul(delta, rpy_matrix(tuple(tcp[3:])))
    return _unwrap(matrix_rpy(rotation), tcp[3:])


def build_fixed_pose_blend_plan(robot, limits_deg,
                                profile: PoseBlendProfile = PoseBlendProfile()) -> PoseBlendPlan:
    """只读规划固定两段六维路径；本函数不发送运动。"""
    if profile != PoseBlendProfile():
        raise ValueError("首轮六维圆滑验收只允许代码中固定的 30mm/3° 档")
    check_health(robot)
    motion_status(robot).require_idle_safe()
    joints, tcp = measured(robot)
    settings = Settings(
        radius_mm=1000.0,
        speed_mm_s=profile.speed_mm_s,
        acceleration_mm_s2=profile.acceleration_mm_s2,
        segment_mm=profile.segment_mm,
        deadband_mm=3.0,
    )

    corner_rpy = _rotated_about_base_z(tcp, profile.orientation_step_deg)
    requested_corner = (tcp[0], tcp[1], tcp[2] + profile.segment_mm, *corner_rpy)
    corner, first = plan_pose_segment(
        robot, joints, tcp, requested_corner, settings, limits_deg,
        max_orientation_step_deg=profile.orientation_step_deg,
    )
    final_rpy = _rotated_about_base_z(corner, profile.orientation_step_deg)
    requested_final = (
        corner[0] + profile.segment_mm, corner[1], corner[2], *final_rpy)
    final, second = plan_pose_segment(
        robot, first[-1], corner, requested_final, settings, limits_deg,
        max_orientation_step_deg=profile.orientation_step_deg,
    )
    if math.dist(corner[:3], requested_corner[:3]) > 1e-6:
        raise RuntimeError("第一段被意外截短，拒绝使用非固定验收路径")
    if math.dist(final[:3], requested_final[:3]) > 1e-6:
        raise RuntimeError("第二段被意外截短，拒绝使用非固定验收路径")
    return PoseBlendPlan(
        profile, tuple(joints), tuple(tcp), tuple(corner), tuple(final),
        tuple(first), tuple(second),
    )


def build_requested_pose_blend_plan(robot, limits_deg, corner_request, final_request,
                                    profile: PoseBlendProfile = HAND_TWO_POINT) -> PoseBlendPlan:
    """把两个手柄采样端点转换为两条经过厂商逆解筛查的六维命令。

    本函数不会发送运动。第二段必须相对第一段具有可见变化，避免由输入噪声
    生成几乎重合的队列命令；超出单段上限的请求由 ``plan_pose_segment`` 截断，
    不会把一次快速手柄移动直接放大成大幅机器人动作。
    """
    if profile != HAND_TWO_POINT:
        raise ValueError("手柄两端点圆滑验收只允许固定的60mm/6°参数档")
    check_health(robot)
    motion_status(robot).require_idle_safe()
    joints, tcp = measured(robot)
    settings = Settings(
        radius_mm=1000.0,
        speed_mm_s=profile.speed_mm_s,
        acceleration_mm_s2=profile.acceleration_mm_s2,
        segment_mm=profile.segment_mm,
        deadband_mm=3.0,
    )
    corner, first = plan_pose_segment(
        robot, joints, tcp, tuple(corner_request), settings, limits_deg,
        max_orientation_step_deg=profile.orientation_step_deg,
    )
    final, second = plan_pose_segment(
        robot, first[-1], corner, tuple(final_request), settings, limits_deg,
        max_orientation_step_deg=profile.orientation_step_deg,
    )
    second_translation = math.dist(corner[:3], final[:3])
    second_rotation = _pose_rotation_error_deg(corner, final)
    if second_translation < 3.0 and second_rotation < 0.5:
        raise ValueError("第二个手柄端点距第一个端点过近，拒绝把输入噪声排入队列")
    return PoseBlendPlan(
        profile, tuple(joints), tuple(tcp), tuple(corner), tuple(final),
        tuple(first), tuple(second),
    )


def _same_start(plan: PoseBlendPlan, joints, tcp) -> bool:
    return (
        math.dist(tcp[:3], plan.start_tcp[:3]) <= 0.5
        and _pose_rotation_error_deg(tcp, plan.start_tcp) <= 0.1
        and max(math.degrees(abs(a - b)) for a, b in zip(
            joints, plan.start_joints, strict=True)) <= 0.2
    )


def _inside_checked_joint_envelope(joints, plan: PoseBlendPlan) -> bool:
    path = (plan.start_joints, *plan.first_solutions, *plan.second_solutions)
    margin = math.radians(3.0)
    return all(
        min(sample[index] for sample in path) - margin <= joints[index]
        <= max(sample[index] for sample in path) + margin
        for index in range(6)
    )


def execute_fixed_pose_blend(robot, plan: PoseBlendPlan, *, permission=None,
                             poll=None, on_command=None, on_stop_confirmed=None,
                             clock=time.monotonic, sleep=time.sleep, emit=None) -> dict:
    """下发固定两条六维厂商命令，并严格审计队列和停止状态。"""
    permission = permission or (lambda: True)
    poll = poll or (lambda: None)
    on_command = on_command or (lambda _count: None)
    on_stop_confirmed = on_stop_confirmed or (lambda _confirmed: None)
    emit = emit or (lambda **_: None)
    commands_sent = 0
    trace = []
    profile = plan.profile

    def allowed() -> bool:
        try:
            return bool(permission())
        except Exception:
            return False

    check_health(robot)
    motion_status(robot).require_idle_safe()
    joints, tcp = measured(robot)
    if not _same_start(plan, joints, tcp):
        raise RuntimeError("六维圆滑规划后机器人起点改变，拒绝执行旧路径")
    if not allowed():
        raise RuntimeError("六维圆滑下发前 Grip/界面许可失效")

    try:
        # 即使 SDK 调用抛异常，命令也可能已抵达控制柜，因此先承担停止义务。
        commands_sent = 1
        on_stop_confirmed(False)
        on_command(commands_sent)
        checked(robot.linear_move_extend_ori(
            plan.corner_tcp, 0, False,
            profile.speed_mm_s, profile.acceleration_mm_s2,
            profile.blend_tolerance_mm,
            math.radians(profile.orientation_speed_deg_s),
            math.radians(profile.orientation_acceleration_deg_s2)),
            "六维圆滑第一段 linear_move_extend_ori")
        emit(state="pose_blend_first_sent", target=plan.corner_tcp,
             tolerance_mm=profile.blend_tolerance_mm)

        busy_deadline = clock() + 0.25
        first_state = None
        while clock() < busy_deadline:
            poll()
            if not allowed():
                raise RuntimeError("第一段后 Grip/界面许可失效")
            check_health(robot)
            first_state = motion_status(robot)
            trace.append(first_state.as_dict())
            require_no_motion_fault(first_state)
            if not first_state.inpos or first_state.queue > 0 or first_state.active_queue > 0:
                break
            sleep(0.01)
        else:
            raise RuntimeError("第一段未观察到队列忙态，拒绝提交第二段")

        if not allowed():
            raise RuntimeError("第二段下发前 Grip/界面许可失效")
        commands_sent = 2
        on_command(commands_sent)
        checked(robot.linear_move_extend_ori(
            plan.final_tcp, 0, False,
            profile.speed_mm_s, profile.acceleration_mm_s2, 0.0,
            math.radians(profile.orientation_speed_deg_s),
            math.radians(profile.orientation_acceleration_deg_s2)),
            "六维圆滑末段 linear_move_extend_ori")
        emit(state="pose_blend_second_sent", target=plan.final_tcp, tolerance_mm=0.0)

        deadline = clock() + 8.0
        last_state = first_state
        while clock() < deadline:
            poll()
            if not allowed():
                raise RuntimeError("六维圆滑运动中 Grip/界面许可失效")
            check_health(robot)
            last_state = motion_status(robot)
            trace.append(last_state.as_dict())
            require_no_motion_fault(last_state)
            actual_joints, actual_tcp = measured(robot)
            if not _inside_checked_joint_envelope(actual_joints, plan):
                raise RuntimeError("六维圆滑实测关节偏离已检查路径包络")
            if (last_state.inpos and last_state.queue == 0 and last_state.active_queue == 0
                    and math.dist(actual_tcp[:3], plan.final_tcp[:3]) <= 1.0
                    and _pose_rotation_error_deg(actual_tcp, plan.final_tcp) <= 0.2):
                emit(state="pose_blend_complete", measured_joints=actual_joints,
                     measured_tcp=actual_tcp, movement_commands_sent=commands_sent)
                on_stop_confirmed(True)
                return {
                    "commands_sent": commands_sent,
                    "stop_confirmed": True,
                    "measured_joints": actual_joints,
                    "measured_tcp": actual_tcp,
                    "max_queue": max(item["queue"] for item in trace),
                    "max_active_queue": max(item["active_queue"] for item in trace),
                    "motion_status_trace": trace,
                }
            sleep(0.02)
        raise RuntimeError(f"两段六维圆滑到位超时：{last_state.as_dict() if last_state else None}")
    except Exception:
        if commands_sent:
            try:
                abort_and_confirm(robot, clock=clock, sleep=sleep)
                on_stop_confirmed(True)
            finally:
                emit(state="pose_blend_abort_requested",
                     movement_commands_sent=commands_sent)
        raise


class RollingPoseQueue:
    """最多保留一条执行段和一条预排段的有限滚动厂商队列。

    它只负责命令衔接。手柄映射和黄色目标仍由 ``BoundedPoseFollower`` 产生；
    每当控制柜预排槽空出时，本类只读取那个时刻的最新目标，不补发旧目标。
    验收允许十条基线或五十条扩展档；正式 V1.5 允许 ``command_limit=None``，
    由 Grip/界面许可结束。有限档末条 ``tol=0``，此前各条 ``tol=5mm``；
    连续档始终保持 ``tol=5mm``，停止时统一调用厂商 ``motion_abort``。
    """

    def __init__(self, robot, settings: Settings, limits_deg, *, emit=None,
                 command_limit=10, blend_tolerance_mm=5.0,
                 orientation_speed_deg_s=30.0,
                 orientation_acceleration_deg_s2=120.0,
                 max_orientation_step_deg=6.0,
                 minimum_translation_mm=20.0,
                 minimum_orientation_deg=2.0,
                 minimum_handoffs=8,
                 clock=time.monotonic, sleep=time.sleep):
        if command_limit not in (None, 10, 50):
            raise ValueError("滚动队列只允许连续档、10段基线或50段扩展档")
        profile = (float(settings.radius_mm), float(settings.segment_mm),
                   float(settings.speed_mm_s), float(settings.acceleration_mm_s2))
        if profile not in {
                (200.0, 20.0, 30.0, 60.0),
                (1000.0, 60.0, 150.0, 400.0),
                (1000.0, 80.0, 200.0, 500.0)}:
            raise ValueError("滚动队列只允许已定义的150或200mm/s固定验收参数")
        self.robot, self.settings, self.limits = robot, settings, limits_deg
        self.emit = emit or (lambda **_: None)
        self.command_limit = command_limit
        self.blend_tolerance_mm = float(blend_tolerance_mm)
        self.orientation_speed_deg_s = float(orientation_speed_deg_s)
        self.orientation_acceleration_deg_s2 = float(orientation_acceleration_deg_s2)
        self.max_orientation_step_deg = float(max_orientation_step_deg)
        self.minimum_translation_mm = float(minimum_translation_mm)
        self.minimum_orientation_deg = float(minimum_orientation_deg)
        if (self.minimum_translation_mm, self.minimum_orientation_deg) != (20.0, 2.0):
            raise ValueError("首轮滚动队列验收固定合并到20mm或2°再下发")
        self.minimum_handoffs = int(minimum_handoffs)
        if self.command_limit is None:
            if self.minimum_handoffs != 0:
                raise ValueError("连续滚动模式不使用结束时最少交接数")
        elif not 0 <= self.minimum_handoffs <= self.command_limit - 2:
            raise ValueError("最少连续交接数必须小于命令上限并且至少为1")
        self.clock, self.sleep = clock, sleep
        self.commands = self.completed_segments = 0
        self.active = self.done = self.stop_confirmed = False
        self.awaiting_busy = self.awaiting_queue_visibility = False
        self.queued_was_visible = False
        self.queue_visibility_started_s = 0.0
        self.segments: list[QueuedPoseSegment] = []
        self.tail_joints = self.tail_tcp = None
        self.last_status = None
        # 本 SDK 所有者读取的真实测量可给显示层复用，避免同一轮重复读关节/TCP。
        self.measured_sample = None
        self.last_progress_s = self.next_health_s = 0.0
        self.next_candidate_s = 0.0
        self.next_feedback_trace_s = 0.0
        self.max_queue = self.max_active_queue = 0
        self.queue_visible_count = self.handoff_count = 0
        self.late_handoff_count = self.starvation_count = 0

    def initialize(self):
        check_health(self.robot)
        motion_status(self.robot).require_idle_safe()
        self.tail_joints, self.tail_tcp = measured(self.robot)
        self.stop_confirmed = True
        self.emit(state="rolling_pose_ready",
                  message="滚动深度2队列已就绪；先松Grip，再持续握住移动")

    def _joint_envelope_ok(self, joints) -> bool:
        if not self.segments:
            return True
        path = []
        for segment in self.segments:
            path.append(segment.start_joints)
            path.extend(segment.solutions)
        margin = math.radians(3.0)
        return all(
            min(sample[index] for sample in path)-margin <= joints[index]
            <= max(sample[index] for sample in path)+margin
            for index in range(6))

    def _abort(self, reason):
        # 先记录触发原因，避免停止接口故障覆盖“为什么需要停止”。
        self.emit(state="rolling_pose_stop_requested", message=reason,
                  movement_commands_sent=self.commands)
        if self.active:
            self.stop_confirmed = False
            abort_and_confirm(self.robot, clock=self.clock, sleep=self.sleep)
        self.active = False
        self.stop_confirmed = True
        self.segments.clear()
        self.emit(state="rolling_pose_stopped", message=reason,
                  movement_commands_sent=self.commands)

    def _poll(self, permit: bool):
        now = self.clock()
        if now >= self.next_health_s:
            check_health(self.robot)
            self.next_health_s = now + .1
        state = motion_status(self.robot)
        self.last_status = state
        self.max_queue = max(self.max_queue, state.queue)
        self.max_active_queue = max(self.max_active_queue, state.active_queue)
        require_no_motion_fault(state)
        if self.active and (state.queue > 2 or state.active_queue > 1):
            raise RuntimeError(
                f"控制柜队列超过本模式深度2边界：{state.as_dict()}")
        if not permit:
            self._abort("Grip/追踪/界面许可撤销")
            return state
        if self.active:
            query_started_s = self.clock()
            joints, tcp = measured(self.robot)
            query_finished_s = self.clock()
            self.measured_sample = (joints, tcp, query_started_s, query_finished_s)
            if query_finished_s >= self.next_feedback_trace_s:
                self.next_feedback_trace_s = query_finished_s + .02
                # 记录实测，而不是用已发目标冒充机器人中间状态。PC查询时间窗
                # 不等于控制器采样时间；后处理必须保留这个测量不确定性。
                self.emit(state="rolling_pose_feedback", measured_joints=joints,
                          measured_tcp=tcp, query_started_s=query_started_s,
                          query_finished_s=query_finished_s,
                          queue=state.queue, active_queue=state.active_queue,
                          inpos=state.inpos, timestamp_source="pc_query_window")
            if not self._joint_envelope_ok(joints):
                raise RuntimeError("滚动队列实测关节偏离已检查路径包络")
            if self.awaiting_busy and (
                    not state.inpos or state.queue > 0 or state.active_queue > 0):
                self.awaiting_busy = False
                self.last_progress_s = now
            # 现场 SDK 的实际序列是：仅当前段时 queue=1/active_queue=1，
            # 下一段成功预排后 queue=2/active_queue=1，交接后又回到 1/1。
            # active_queue=1 不是“预排槽已占用”，不能据此阻止第二条命令。
            if self.awaiting_queue_visibility and state.queue >= 2:
                self.awaiting_queue_visibility = False
                self.queued_was_visible = True
                self.queue_visible_count += 1
                self.last_progress_s = now
                self.emit(state="rolling_pose_queue_visible",
                          message="已确认执行段与预排段同时在控制柜队列中",
                          queue=state.queue, active_queue=state.active_queue)
            elif (self.awaiting_queue_visibility and not state.inpos
                  and state.queue == 1 and state.active_queue > 0
                  and len(self.segments) >= 2
                  and math.dist(tcp[:3], self.segments[0].target_tcp[:3])
                  <= self.blend_tolerance_mm + 1.5
                  and _pose_rotation_error_deg(
                      tcp, self.segments[0].target_tcp) <= 1.0):
                # 极短前段可能恰好在“第二条命令返回成功”和下一次状态读取之间
                # 完成交接，因而看不到瞬时 queue=2。实测 TCP 已进入前段端点的
                # 圆滑容差、且控制柜仍明确处于 1/1 忙态，可确认第二段已成为当前段。
                self.segments.pop(0)
                self.completed_segments += 1
                self.handoff_count += 1
                self.late_handoff_count += 1
                self.awaiting_queue_visibility = False
                self.last_progress_s = now
                self.emit(
                    state="rolling_pose_late_handoff",
                    message="queue=2可见窗口被跨过；已由1/1忙态和实测前段端点确认交接",
                    completed_segments=self.completed_segments,
                    late_handoff_count=self.late_handoff_count)
            elif (self.queued_was_visible and not state.inpos
                  and state.active_queue > 0 and state.queue <= 1
                  and len(self.segments) >= 2):
                self.segments.pop(0)
                self.completed_segments += 1
                self.handoff_count += 1
                self.queued_was_visible = False
                self.last_progress_s = now
                self.emit(state="rolling_pose_handoff",
                          message="厂商队列已圆滑交接，预排槽重新开放",
                          completed_segments=self.completed_segments)
            # 命令刚发送时控制柜状态可能短暂仍是 idle。必须先观察到 busy/queue，
            # 才能把后续 idle 当作真实到位，避免把尚未显现的命令误判为已完成。
            if (not self.awaiting_busy and state.inpos
                    and state.queue == 0 and state.active_queue == 0):
                # 很短的运动可能在两次状态轮询之间已经执行完毕，此时 queue=2
                # 的可见窗口会被错过。只在实测 TCP 已到最后一个下发目标时把它
                # 认作“安全耗尽”；否则属于命令/状态不一致，不能继续排新目标。
                if self.segments:
                    final_target = self.segments[-1].target_tcp
                    position_error = math.dist(tcp[:3], final_target[:3])
                    rotation_error = _pose_rotation_error_deg(tcp, final_target)
                    if position_error > 1.0 or rotation_error > .2:
                        raise RuntimeError(
                            "控制柜队列已空闲但实测TCP未到最后目标："
                            f"位置误差{position_error:.3f}mm，姿态误差{rotation_error:.3f}°")
                if self.awaiting_queue_visibility:
                    self.starvation_count += 1
                    self.emit(
                        state="rolling_pose_queue_starved",
                        message="短段已在queue=2可见前执行完；已按实测到位点安全重建队列",
                        starvation_count=self.starvation_count)
                # 所有队列耗尽都记录，而不只记录错过 queue=2 的特例。
                # 只有这项与 Grip/候选拒绝的时间关联，才能分辨预期停手和断供。
                self.emit(state="rolling_pose_queue_drained",
                          message="控制柜已实测空闲；不等于段间保持了非零速度",
                          completed_pending_segments=len(self.segments),
                          movement_commands_sent=self.commands)
                self.completed_segments += len(self.segments)
                self.segments.clear()
                self.active = False
                self.awaiting_busy = self.awaiting_queue_visibility = False
                self.queued_was_visible = False
                self.stop_confirmed = True
                self.last_progress_s = now
                if (self.command_limit is not None
                        and self.commands >= self.command_limit):
                    if self.handoff_count < self.minimum_handoffs:
                        raise RuntimeError(
                            f"{self.command_limit}段已结束但只观察到"
                            f"{self.handoff_count}次连续队列交接，"
                            f"最低要求{self.minimum_handoffs}次")
                    self.done = True
                    self.emit(state="rolling_pose_complete",
                              message=f"{self.command_limit}段滚动六维队列全部实测结束",
                              movement_commands_sent=self.commands,
                              completed_segments=self.completed_segments,
                              queue_visible_count=self.queue_visible_count,
                              handoff_count=self.handoff_count,
                              late_handoff_count=self.late_handoff_count,
                              starvation_count=self.starvation_count,
                              max_queue=self.max_queue,
                              max_active_queue=self.max_active_queue)
            elif self.last_progress_s and now-self.last_progress_s > 4.0:
                raise RuntimeError("滚动六维队列4秒无进展")
        return state

    def tick(self, desired, *, permit: bool, refresh_permit=None):
        """轮询控制柜，并在唯一预排槽空闲时采用最新目标。"""
        if self.done:
            return
        try:
            state = self._poll(permit)
            if (not permit or self.done or desired is None
                    or (self.command_limit is not None
                        and self.commands >= self.command_limit)):
                return
            if self.active:
                if (self.awaiting_busy or self.awaiting_queue_visibility
                        or self.queued_was_visible or len(self.segments) != 1
                        or state.inpos or state.active_queue <= 0
                        or state.queue > 1):
                    return
                start_joints, start_tcp = self.tail_joints, self.tail_tcp
            else:
                state.require_idle_safe()
                start_joints, start_tcp = measured(self.robot)

            if self.clock() < self.next_candidate_s:
                return

            translation = math.dist(start_tcp[:3], desired[:3])
            rotation = _pose_rotation_error_deg(start_tcp, desired)
            # 太短的段只会把追踪噪声变成控制柜队列，且削弱圆滑效果。
            if (translation < self.minimum_translation_mm
                    and rotation < self.minimum_orientation_deg):
                return
            try:
                planning_started_s = self.clock()
                target, solutions = plan_pose_segment(
                    self.robot, start_joints, start_tcp, desired,
                    self.settings, self.limits,
                    max_orientation_step_deg=self.max_orientation_step_deg)
            except RejectedPath as error:
                # 目标尚未下发，厂商逆解筛查拒绝并不是运动故障。丢弃这一帧并
                # 继续等待新的手柄端点，不能因此终止整个会话。短暂退避避免在
                # 手柄停留于同一不可用姿态时反复占用 SDK。
                self.next_candidate_s = self.clock() + .2
                self.emit(state="rolling_pose_target_rejected",
                          message=str(error), target=desired)
                return
            # 原生 SDK 逆解循环期间，Grip/追踪/GUI 心跳可能已经改变。
            # 必须在副作用前再次检查；不能拿本轮开头的 bool 当作持续许可。
            if self.clock()-planning_started_s > self.settings.input_timeout_s:
                self._abort("逆解准备耗时超过输入有效期；未下发迟到目标")
                raise RuntimeError("滚动队列逆解准备迟到；释放Grip后重新检查")
            if refresh_permit is not None and not refresh_permit():
                self._abort("下发前Grip/追踪/界面许可撤销；未下发新目标")
                return
            tolerance = (0.0 if (self.command_limit is not None
                                  and self.commands+1 == self.command_limit)
                         else self.blend_tolerance_mm)
            self._dispatch_checked(start_joints,start_tcp,target,solutions,tolerance)
        except Exception:
            if self.active:
                self._abort("滚动队列异常，已请求厂商停止")
            raise

    def _dispatch_checked(self,start_joints,start_tcp,target,solutions,tolerance):
        """唯一的厂商下发位置；调用方先完成路径、队列槽和最新许可检查。

        固定采样和动态采样共用此处，检查过的目标原样传入 JAKA。SDK 调用失败
        也可能代表控制柜已经收到了命令，所以必须在调用前承担停止义务。
        """
        segment = QueuedPoseSegment(tuple(start_joints),tuple(start_tcp),tuple(target),
                                    tuple(solutions),tolerance)
        self.stop_confirmed = False
        self.active = True
        self.commands += 1
        self.segments.append(segment)
        self.tail_joints,self.tail_tcp = solutions[-1],target
        checked(self.robot.linear_move_extend_ori(
            target,0,False,self.settings.speed_mm_s,self.settings.acceleration_mm_s2,
            tolerance,math.radians(self.orientation_speed_deg_s),
            math.radians(self.orientation_acceleration_deg_s2)),
            "滚动六维 linear_move_extend_ori")
        self.last_progress_s = self.clock()
        if len(self.segments) == 1:
            self.awaiting_busy = True
        else:
            self.awaiting_queue_visibility = True
            self.queue_visibility_started_s = self.last_progress_s
        self.emit(state="rolling_pose_command",target=target,command_index=self.commands,
                  tolerance_mm=tolerance,
                  translation_mm=math.dist(start_tcp[:3],target[:3]),
                  orientation_deg=_pose_rotation_error_deg(start_tcp,target))

    def shutdown(self):
        if self.active:
            self._abort("滚动队列会话结束，已请求厂商停止")
        return self.stop_confirmed
