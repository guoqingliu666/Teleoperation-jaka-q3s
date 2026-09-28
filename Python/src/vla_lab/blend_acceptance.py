"""厂商 ``linear_move_extend`` 两段圆滑的固定小范围验收核心。

该模块不创建 SDK 连接，也不提供通用遥操作循环。调用者必须另外完成现场确认。
只提供代码中固定的 5 mm、20 mm、24 mm、30 mm 与 50 mm 五档：先沿基坐标 +Z，再沿 +X，第一段
使用固定 tol 圆滑半径，末段 tol=0。每 2 mm 调用厂商 ``kine_inverse`` 筛查。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time

from .motion_status import MotionStatus
from .sampled_follow import RejectedPath, check_health, checked, measured, six


LIMITS_MARGIN_DEG = 3.0


@dataclass(frozen=True)
class BlendProfile:
    key: str
    segment_mm: float
    speed_mm_s: float
    acceleration_mm_s2: float
    blend_tolerance_mm: float
    confirmation: str


MICRO5 = BlendProfile("micro5", 5.0, 5.0, 10.0, 1.0, "执行固定两段5MM圆滑")
VISIBLE20 = BlendProfile("visible20", 20.0, 10.0, 20.0, 3.0, "执行固定两段20MM圆滑")
# 24 mm 不贴近当前姿态只读扫描得到的 26 mm 可接受边界；保持 10 mm/s 与已经
# 真机通过的 20 mm 档一致，变化因素只剩路径长度与圆滑半径。
VISIBLE24 = BlendProfile("visible24", 24.0, 10.0, 20.0, 3.0, "执行固定两段24MM圆滑")
VISIBLE30 = BlendProfile("visible30", 30.0, 12.0, 24.0, 4.0, "执行固定两段30MM圆滑")
# 50 mm 档仅用于在 20 mm 真机圆滑验收通过后的下一道独立门槛。它仍须先完成
# ``--live-plan-only``，随后由操作者另行给出精确确认短语，绝不会随普通入口自动执行。
VISIBLE50 = BlendProfile("visible50", 50.0, 15.0, 30.0, 5.0, "执行固定两段50MM圆滑")
PROFILES = {profile.key: profile for profile in (MICRO5, VISIBLE20, VISIBLE24, VISIBLE30, VISIBLE50)}


@dataclass(frozen=True)
class BlendPlan:
    profile: BlendProfile
    start_joints: tuple[float, ...]
    start_tcp: tuple[float, ...]
    corner_tcp: tuple[float, ...]
    final_tcp: tuple[float, ...]
    first_solutions: tuple[tuple[float, ...], ...]
    second_solutions: tuple[tuple[float, ...], ...]


def motion_status(robot) -> MotionStatus:
    value = checked(robot.get_motion_status(), "get_motion_status")
    return MotionStatus.parse(value)


def require_no_motion_fault(state: MotionStatus) -> None:
    if state.queue_full or state.paused or state.on_limit or state.in_estop or state.in_collision:
        raise RuntimeError(f"运动队列状态禁止继续：{state.as_dict()}")


def plan_leg(robot, joints, tcp, target, limits_deg):
    """沿指定直线每 2 mm 检查厂商逆解；不生成自己的关节解。"""
    distance = math.dist(tcp[:3], target[:3])
    count = max(1, math.ceil(distance / 2.0))
    reference = joints
    solutions = []
    for index in range(1, count + 1):
        pose = tuple(tcp[axis] + (target[axis] - tcp[axis]) * index / count
                     for axis in range(3)) + tuple(tcp[3:])
        result = robot.kine_inverse(reference, pose)
        if isinstance(result, tuple) and result and result[0] == -4:
            raise RejectedPath("段内厂商逆解不可达")
        solved = six(checked(result, "kine_inverse"))
        if max(abs(math.degrees(a - b)) for a, b in zip(solved, reference, strict=True)) > 3.0:
            raise RejectedPath("相邻2mm样本的关节变化超过3°")
        if max(abs(math.degrees(a - b)) for a, b in zip(solved, joints, strict=True)) > 12.0:
            raise RejectedPath("单段累计关节变化超过12°")
        for value, (low, high) in zip(solved, limits_deg, strict=True):
            if not low + LIMITS_MARGIN_DEG <= math.degrees(value) <= high - LIMITS_MARGIN_DEG:
                raise RejectedPath("候选关节距配置限位不足3°")
        solutions.append(solved)
        reference = solved
    return tuple(solutions)


def build_fixed_corner_plan(robot, limits_deg, profile: BlendProfile = MICRO5) -> BlendPlan:
    """仅规划和逆解检查，不发送运动。"""
    check_health(robot)
    motion_status(robot).require_idle_safe()
    joints, tcp = measured(robot)
    if profile not in PROFILES.values():
        raise ValueError("只允许代码中固定的两段圆滑验收档")
    corner = (tcp[0], tcp[1], tcp[2] + profile.segment_mm, *tcp[3:])
    first = plan_leg(robot, joints, tcp, corner, limits_deg)
    final = (corner[0] + profile.segment_mm, corner[1], corner[2], *corner[3:])
    second = plan_leg(robot, first[-1], corner, final, limits_deg)
    return BlendPlan(profile, joints, tcp, corner, final, first, second)


def scan_fixed_corner_envelope(robot, limits_deg, *, minimum_mm: int = 5, maximum_mm: int = 50) -> dict:
    """只读扫描当前姿态下固定 ``+Z→+X`` 两段路径的可接受段长。

    每个候选段长仅调用厂商 ``kine_inverse``。不会调用任何运动、伺服或停止接口，
    也不会修改机器人状态。这里的“可接受”严格沿用本模块的相邻关节步长、单段
    累计关节变化与关节余量门槛；它不是工作空间或碰撞安全的完整证明。
    """
    if not (isinstance(minimum_mm, int) and isinstance(maximum_mm, int)
            and 1 <= minimum_mm <= maximum_mm <= 200):
        raise ValueError("只读扫描范围必须为 1 到 200 mm 的整数")
    check_health(robot)
    motion_status(robot).require_idle_safe()
    joints, tcp = measured(robot)
    accepted: list[int] = []
    rejected: dict[int, str] = {}
    for segment_mm in range(minimum_mm, maximum_mm + 1):
        corner = (tcp[0], tcp[1], tcp[2] + segment_mm, *tcp[3:])
        final = (corner[0] + segment_mm, corner[1], corner[2], *corner[3:])
        try:
            first = plan_leg(robot, joints, tcp, corner, limits_deg)
            plan_leg(robot, first[-1], corner, final, limits_deg)
        except RejectedPath as error:
            rejected[segment_mm] = str(error)
        else:
            accepted.append(segment_mm)
    return {
        "start_tcp": tcp,
        "minimum_mm": minimum_mm,
        "maximum_mm": maximum_mm,
        "accepted_segment_mm": accepted,
        "largest_accepted_segment_mm": max(accepted) if accepted else None,
        "rejected_segment_mm": rejected,
    }


def _same_start(plan: BlendPlan, joints, tcp) -> bool:
    return (
        math.dist(tcp[:3], plan.start_tcp[:3]) <= 0.5
        and max(abs(math.degrees(a - b)) for a, b in zip(joints, plan.start_joints, strict=True)) <= 0.2
    )


def abort_and_confirm(robot, *, clock=time.monotonic, sleep=time.sleep) -> None:
    """请求厂商停止并要求队列在 2 秒内回到空闲；不能把调用成功当成已停。"""
    checked(robot.motion_abort(), "两段圆滑异常 motion_abort")
    deadline = clock() + 2.0
    last_state = None
    while clock() < deadline:
        # 故障锁仍禁止普通查询和新运动，但必须留下专用的停稳读回通道。
        # 未包装的离线假SDK/原SDK继续使用原接口。
        stop_query = getattr(robot, "get_stop_motion_status", None)
        last_state = (MotionStatus.parse(checked(stop_query(), "停止确认状态"))
                      if callable(stop_query) else motion_status(robot))
        if clock() >= deadline:
            break  # 本地调用可能阻塞；超出确认窗口的结果不能当成及时确认。
        if last_state.inpos and last_state.queue == 0 and last_state.active_queue == 0:
            return
        sleep(0.02)
    raise RuntimeError(f"motion_abort 后停止未确认：{last_state.as_dict() if last_state else None}")


def execute_fixed_corner(
    robot,
    plan: BlendPlan,
    *,
    clock=time.monotonic,
    sleep=time.sleep,
    emit=None,
) -> dict:
    """发送两条厂商轨迹；队列未进入忙态就绝不提交第二段。"""
    emit = emit or (lambda **_: None)
    check_health(robot)
    motion_status(robot).require_idle_safe()
    joints, tcp = measured(robot)
    if not _same_start(plan, joints, tcp):
        raise RuntimeError("规划后机器人起点改变，拒绝执行旧圆滑路径")

    first_may_have_started = False
    abort_requested = False
    status_trace = []
    try:
        # 调用异常也可能表示命令已到控制柜，所以先承担停止义务。
        first_may_have_started = True
        profile = plan.profile
        checked(robot.linear_move_extend(
            plan.corner_tcp, 0, False, profile.speed_mm_s,
            profile.acceleration_mm_s2, profile.blend_tolerance_mm),
                "第一段 linear_move_extend")
        emit(state="first_sent", target=plan.corner_tcp,
             tolerance_mm=profile.blend_tolerance_mm)

        busy_deadline = clock() + 0.25
        first_state = None
        while clock() < busy_deadline:
            check_health(robot)
            first_state = motion_status(robot)
            status_trace.append(first_state.as_dict())
            require_no_motion_fault(first_state)
            if not first_state.inpos or first_state.queue > 0 or first_state.active_queue > 0:
                break
            sleep(0.01)
        else:
            raise RuntimeError("第一段未观察到队列忙态，拒绝提交第二段")

        checked(robot.linear_move_extend(
            plan.final_tcp, 0, False, profile.speed_mm_s,
            profile.acceleration_mm_s2, 0.0),
                "末段 linear_move_extend")
        emit(state="second_sent", target=plan.final_tcp, tolerance_mm=0.0)

        deadline = clock() + max(6.0, 2 * profile.segment_mm / profile.speed_mm_s * 3 + 2)
        last_state = first_state
        while clock() < deadline:
            check_health(robot)
            last_state = motion_status(robot)
            status_trace.append(last_state.as_dict())
            require_no_motion_fault(last_state)
            _, actual_tcp = measured(robot)
            if (last_state.inpos and last_state.queue == 0 and last_state.active_queue == 0
                    and math.dist(actual_tcp[:3], plan.final_tcp[:3]) <= 1.0):
                emit(state="complete", measured_tcp=actual_tcp)
                return {
                    "commands_sent": 2,
                    "stop_confirmed": True,
                    "final_motion_status": last_state.as_dict(),
                    "measured_tcp": actual_tcp,
                    "max_queue": max(item["queue"] for item in status_trace),
                    "max_active_queue": max(item["active_queue"] for item in status_trace),
                    "motion_status_trace": status_trace,
                }
            sleep(0.02)
        raise RuntimeError(f"两段圆滑到位超时：{last_state.as_dict() if last_state else None}")
    except Exception:
        if first_may_have_started:
            try:
                abort_requested = True
                abort_and_confirm(robot, clock=clock, sleep=sleep)
            finally:
                emit(state="abort_requested", requested=abort_requested)
        raise
