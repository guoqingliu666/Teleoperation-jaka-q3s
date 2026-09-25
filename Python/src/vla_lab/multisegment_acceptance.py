"""JAKA 厂商规划器的固定三段圆滑验收。

这是两段验收之后的下一层队列能力验证，不读取 Quest，也不含逐周期伺服控制。
它只使用厂商 ``kine_inverse`` 与 ``linear_move_extend``：每段先进行厂商逆解
筛查；实际执行时最多保持两条待执行命令，第三段仅在第二段已经被控制柜接管
且仍处于运动中时才提交。这样可以检验连续补段，而不猜测控制柜的队列容量。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

from .blend_acceptance import (
    abort_and_confirm, motion_status, plan_leg, require_no_motion_fault,
)
from .sampled_follow import check_health, checked, measured


@dataclass(frozen=True)
class ThreeSegmentProfile:
    key: str
    leg_mm: float
    speed_mm_s: float
    acceleration_mm_s2: float
    blend_tolerance_mm: float
    confirmation: str


# 仅在两段 20 mm/24 mm 已通过后使用的低速多段验证；第三段回落到起始高度，
# 不扩大 Z 方向的最高点。它不是手柄跟随入口。
TRI20 = ThreeSegmentProfile("tri20", 20.0, 10.0, 20.0, 3.0, "执行固定三段20MM圆滑")


@dataclass(frozen=True)
class ThreeSegmentPlan:
    profile: ThreeSegmentProfile
    start_joints: tuple[float, ...]
    start_tcp: tuple[float, ...]
    targets: tuple[tuple[float, ...], ...]
    leg_solutions: tuple[tuple[tuple[float, ...], ...], ...]


def build_three_segment_plan(robot, limits_deg, profile: ThreeSegmentProfile = TRI20) -> ThreeSegmentPlan:
    """只读读取实机状态，并在三个固定直线段上调用厂商逆解。"""
    check_health(robot)
    motion_status(robot).require_idle_safe()
    joints, start_tcp = measured(robot)
    p1 = (start_tcp[0], start_tcp[1], start_tcp[2] + profile.leg_mm, *start_tcp[3:])
    p2 = (p1[0] + profile.leg_mm, p1[1], p1[2], *p1[3:])
    p3 = (p2[0], p2[1], p2[2] - profile.leg_mm, *p2[3:])
    leg1 = plan_leg(robot, joints, start_tcp, p1, limits_deg)
    leg2 = plan_leg(robot, leg1[-1], p1, p2, limits_deg)
    leg3 = plan_leg(robot, leg2[-1], p2, p3, limits_deg)
    return ThreeSegmentPlan(profile, joints, start_tcp, (p1, p2, p3), (leg1, leg2, leg3))


def path_metrics(plan: ThreeSegmentPlan, limits_deg) -> dict:
    """汇总厂商逆解样本，便于只读报告审计。"""
    points = [plan.start_joints, *(point for leg in plan.leg_solutions for point in leg)]
    return {
        "sample_count": len(points),
        "max_adjacent_joint_step_deg": max(
            abs(math.degrees(current[index] - previous[index]))
            for previous, current in zip(points, points[1:]) for index in range(6)
        ),
        "max_joint_change_from_start_deg": max(
            abs(math.degrees(point[index] - plan.start_joints[index]))
            for point in points[1:] for index in range(6)
        ),
        "minimum_configured_joint_margin_deg": min(
            min(math.degrees(point[index]) - limits_deg[index][0],
                limits_deg[index][1] - math.degrees(point[index]))
            for point in points for index in range(6)
        ),
    }


def _same_start(plan: ThreeSegmentPlan, joints, tcp) -> bool:
    return (
        math.dist(tcp[:3], plan.start_tcp[:3]) <= 0.5
        and max(abs(math.degrees(a - b)) for a, b in zip(joints, plan.start_joints, strict=True)) <= 0.2
    )


def execute_three_segment(robot, plan: ThreeSegmentPlan, *, clock=time.monotonic, sleep=time.sleep, emit=None) -> dict:
    """流式提交固定三段，且队列中最多保有两条待执行命令。"""
    emit = emit or (lambda **_: None)
    check_health(robot)
    motion_status(robot).require_idle_safe()
    joints, tcp = measured(robot)
    if not _same_start(plan, joints, tcp):
        raise RuntimeError("规划后机器人起点改变，拒绝执行旧三段路径")

    profile = plan.profile
    trace: list[dict] = []
    first_may_have_started = False
    submitted = 0
    try:
        # 第一段和第二段形成已验证的两段圆滑队列。
        first_may_have_started = True
        checked(robot.linear_move_extend(plan.targets[0], 0, False, profile.speed_mm_s,
                                         profile.acceleration_mm_s2, profile.blend_tolerance_mm),
                "三段验收第一段 linear_move_extend")
        submitted = 1
        emit(state="first_sent", target=plan.targets[0])

        busy_deadline = clock() + 0.25
        while clock() < busy_deadline:
            state = motion_status(robot)
            trace.append(state.as_dict())
            require_no_motion_fault(state)
            if not state.inpos or state.queue > 0 or state.active_queue > 0:
                break
            sleep(0.01)
        else:
            raise RuntimeError("第一段未观察到队列忙态，拒绝提交后续段")

        checked(robot.linear_move_extend(plan.targets[1], 0, False, profile.speed_mm_s,
                                         profile.acceleration_mm_s2, profile.blend_tolerance_mm),
                "三段验收第二段 linear_move_extend")
        submitted = 2
        emit(state="second_sent", target=plan.targets[1])

        # 等第一段被消费，再补第三段；这避免未经确认地将队列深度堆到三条。
        # 低速 20 mm 档每段本身约需 2 s；加减速和控制柜状态回报会拉长交接时刻。
        # 旧的 4 s 窗口在现场没有观察到交接就提前退出，故采用明确的 10 s 上限；
        # 在此期间绝不提交第三段，任何故障状态仍会立即转入 abort_and_confirm。
        handoff_deadline = clock() + max(10.0, profile.leg_mm / profile.speed_mm_s * 4 + 2)
        last_handoff_state = None
        while clock() < handoff_deadline:
            state = motion_status(robot)
            last_handoff_state = state
            trace.append(state.as_dict())
            require_no_motion_fault(state)
            if not state.inpos and state.active_queue > 0 and state.queue <= 1:
                break
            sleep(0.02)
        else:
            raise RuntimeError(
                "第二段未观察到安全的队列交接，拒绝提交第三段："
                f"{last_handoff_state.as_dict() if last_handoff_state else None}"
            )

        checked(robot.linear_move_extend(plan.targets[2], 0, False, profile.speed_mm_s,
                                         profile.acceleration_mm_s2, 0.0),
                "三段验收末段 linear_move_extend")
        submitted = 3
        emit(state="third_sent", target=plan.targets[2])

        deadline = clock() + max(8.0, 3 * profile.leg_mm / profile.speed_mm_s * 3 + 2)
        while clock() < deadline:
            state = motion_status(robot)
            trace.append(state.as_dict())
            require_no_motion_fault(state)
            _, actual_tcp = measured(robot)
            if (state.inpos and state.queue == 0 and state.active_queue == 0
                    and math.dist(actual_tcp[:3], plan.targets[-1][:3]) <= 1.0):
                return {
                    "commands_sent": submitted,
                    "stop_confirmed": True,
                    "final_motion_status": state.as_dict(),
                    "measured_tcp": actual_tcp,
                    "max_queue": max(item["queue"] for item in trace),
                    "max_active_queue": max(item["active_queue"] for item in trace),
                    "motion_status_trace": trace,
                }
            sleep(0.02)
        raise RuntimeError(f"三段圆滑到位超时：{state.as_dict()}")
    except Exception:
        if first_may_have_started:
            abort_and_confirm(robot, clock=clock, sleep=sleep)
            emit(state="abort_requested", submitted=submitted)
        raise
