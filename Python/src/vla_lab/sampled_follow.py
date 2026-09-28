"""Grip 持续采样的位置跟随：JAKA 做逆解和直线规划，Python 管理短段生命周期。

只允许一段运动在途。最新手柄目标覆盖旧目标，不排队重放手柄历史。
段内每 2 mm 用厂商逆解检查关节连续性，这只是风险筛查，不是碰撞/奇异性证明。
使用厂商 linear_move_extend 明确速度、加速度及零过渡误差；不使用伺服接口。
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
import uuid

from .quest_vr_input import RelativeQuestTracker, matmul


class RejectedPath(RuntimeError):
    """路径不可达或候选关节变化过大；保持，等待新的手柄目标。"""


def checked(result, name):
    """统一解释厂商 SDK 的 `(错误码, 数据)` 返回值；非零码不得当作成功。"""
    if not isinstance(result, tuple) or not result or type(result[0]) is not int:
        raise RuntimeError(f"{name} 返回格式异常：{result!r}")
    if result[0] != 0:
        raise RuntimeError(f"{name} 失败：{result!r}")
    return result[1] if len(result) > 1 else None


def six(values):
    """把六个关节/TCP 数值转成有限浮点数，拒绝缺项与 NaN/Inf。"""
    if not isinstance(values, (tuple, list)) or len(values) != 6:
        raise RuntimeError("SDK 反馈不是六维数据")
    result = tuple(float(x) for x in values)
    if not all(math.isfinite(x) for x in result):
        raise RuntimeError("SDK 反馈含 NaN/Inf")
    return result


def flag(result, name):
    """读取只能是 0/1 的状态；未知返回值不能猜作“安全”。"""
    value = checked(result, name)
    if not isinstance(value, (int, bool)) or value not in (0, 1):
        raise RuntimeError(f"{name} 状态未知")
    return bool(value)


@dataclass(frozen=True)
class Settings:
    """单次会话的固定门槛；创建对象时就检查范围，运行途中不热更新。"""
    radius_mm: float = 100.0
    speed_mm_s: float = 10.0
    acceleration_mm_s2: float = 50.0
    segment_mm: float = 10.0
    input_timeout_s: float = 0.15
    sample_period_s: float = 0.05
    deadband_mm: float = 1.0

    def __post_init__(self):
        # 数据结构允许最终目标所需的1m工作球；现行真机入口仍只开放代码固定的
        # 20/30/60/100/200mm分阶段档位，不能通过命令行或滑杆绕过。
        for name, low, high in (("radius_mm", 20, 1000), ("speed_mm_s", 5, 300),
                                ("acceleration_mm_s2", 10, 800), ("segment_mm", 2, 100),
                                ("input_timeout_s", .05, .15), ("sample_period_s", .02, .1),
                                ("deadband_mm", .5, 3)):
            value = getattr(self, name)
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} 必须在 {low}—{high} 内")


def clamp_position(tcp, center, radius):
    """把位置目标限制在启动 TCP 周围的球内；保持原有三个姿态分量。"""
    delta = tuple(tcp[i] - center[i] for i in range(3))
    norm = math.hypot(*delta)
    scale = min(1.0, radius / norm) if norm else 1.0
    return tuple(center[i] + delta[i] * scale for i in range(3)) + tuple(tcp[3:])


def check_health(robot):
    """运行中状态检查；不写上电、使能、工具或报警参数。"""
    simple = checked(robot.get_robot_status_simple(), "状态")
    if not isinstance(simple, (list, tuple)) or len(simple) < 4:
        raise RuntimeError("简要状态格式异常")
    if simple[0] != 0 or simple[2] != 1 or simple[3] != 1:
        raise RuntimeError(f"状态不就绪：{simple!r}")
    for method in ("is_in_estop", "is_in_collision", "is_on_limit", "is_in_servomove"):
        if flag(getattr(robot, method)(), method):
            raise RuntimeError(f"状态禁止运动：{method}")
    if checked(robot.get_tool_id(), "工具") != 1:
        raise RuntimeError("Tool 已改变，要求 Tool 1")
    if checked(robot.get_user_frame_id(), "用户坐标系") != 0:
        raise RuntimeError("用户坐标系已改变，要求0")


def measured(robot):
    """读取关节与 TCP 的真实反馈，不使用刚发出的命令冒充到位。"""
    return (six(checked(robot.get_actual_joint_position(), "实测关节")),
            six(checked(robot.get_actual_tcp_position(), "实测TCP")))


def plan_segment(robot, joints, tcp, desired, settings, limits_deg):
    """筛查拟执行的直线段，而非仅检查远端目标。

    同一分支的相邻解作为下次种子，不枚举其它分支，不做角度取模掩盖跳变。
    逆解 -4 是不可达；其它厂商错误直接上抛终止会话。这里没有自编逆解算法。
    """
    target = clamp_position(desired, tcp, settings.segment_mm)
    distance = math.dist(tcp[:3], target[:3])
    count = max(1, math.ceil(distance / 2.0))
    reference = joints
    solutions = []
    for index in range(1, count + 1):
        pose = tuple(tcp[i] + (target[i] - tcp[i]) * index / count for i in range(3)) + tcp[3:]
        result = robot.kine_inverse(reference, pose)
        if isinstance(result, tuple) and result and result[0] == -4:
            raise RejectedPath("段内厂商逆解不可达")
        solved = six(checked(result, "kine_inverse"))
        if max(abs(math.degrees(a - b)) for a, b in zip(solved, reference)) > 3.0:
            raise RejectedPath("相邻2mm样本的关节变化超过3°，拒绝跨分支/高敏感路径")
        if max(abs(math.degrees(a - b)) for a, b in zip(solved, joints)) > 12.0:
            raise RejectedPath("短段累计关节变化超过12°")
        for q, (low, high) in zip(solved, limits_deg):
            if not low + 3.0 <= math.degrees(q) <= high - 3.0:
                raise RejectedPath("候选关节距配置限位不足3°")
        solutions.append(solved)
        reference = solved
    return target, solutions


class SampledFollower:
    """单线程 SDK 所有者；每次 tick 只前进一个状态，UI/UDP 不持有机器人。

    松 Grip/失追踪后请求 motion_abort 并等到位；再次完整松开后才能重捕获。
    总工作球固定于启动时的 TCP，反复松握不会把半径中心越移越远。
    """
    def __init__(self, robot, mapping, settings, limits_deg, *, clock=time.monotonic,
                 emit=None, command_limit=None, path_limit_mm=None):
        if command_limit is not None and (type(command_limit) is not int or command_limit < 1):
            raise ValueError("command_limit 必须为正整数")
        if path_limit_mm is not None and (not math.isfinite(path_limit_mm) or path_limit_mm <= 0):
            raise ValueError("path_limit_mm 必须为有限正数")
        self.robot, self.settings, self.limits = robot, settings, limits_deg
        self.command_limit = command_limit
        self.path_limit_mm = path_limit_mm
        self.clock, self.emit = clock, emit or (lambda **kw: None)
        self.tracker = RelativeQuestTracker(mapping, grip_on=.75, grip_off=.55,
                                            trigger_on=.75, trigger_off=.55)
        self.center = self.anchor = self.desired = self.active_target = None
        self.initial_joints = None
        self.last_completed_measurement = None
        self.measured_path_mm = 0.0
        self.commanded_path_mm = 0.0
        self.active_start_tcp = None
        self.path_solutions = []
        self.heading = False
        self.release_seen = False
        self.active = self.stopping = self.faulted = False
        self.last_frame = None
        self.source = None
        self.last_processed = None
        self.last_input_position = None
        self.next_plan = self.next_health = self.next_motion_poll = 0.0
        self.active_since = 0.0
        self.abort_requested = False
        self.commands = 0
        self.completed_segments = 0
        self.binding_id = None

    def initialize(self):
        """只有机器人就绪且静止时记录工作球中心与初始关节。"""
        check_health(self.robot)
        if not flag(self.robot.is_in_pos(), "到位"):
            raise RuntimeError("启动时机器人必须静止")
        self.initial_joints, self.center = measured(self.robot)
        self.emit(state="ready", message="按住 Grip 跟随，松开停止；无需 A 键", target=None)

    def stop(self, reason):
        """撤销目标绑定；若短段仍在途，请求 SDK 中止并等待实测停稳。"""
        self.emit(state="display_binding", binding=None)
        self.binding_id = None
        self.pending = None
        self.desired = self.anchor = None
        self.tracker.reset_grip()
        self.release_seen = False
        if self.active and not self.abort_requested:
            checked(self.robot.motion_abort(), "motion_abort")
            self.abort_requested = True
            self.stopping = True
            self.active_since = self.clock()
        self.emit(state="stopping" if self.active else "paused", message=reason, target=None)

    def tick(self, frame, *, permit=True, allow_plan=True):
        """处理一轮输入和反馈，最多准备一个短段，不在这里直接下发命令。

        `pending` 只是候选：运行器稍后还要重新读机器人、手柄与窗口许可，
        然后才允许 `dispatch`。这避免“检查过的路径”和“实际起点”不一致。
        """
        now = self.clock()
        if frame is not None:
            self.last_frame = frame
        frame = self.last_frame
        valid = bool(frame and frame.connected and frame.tracked and frame.valid
                     and 0 <= now - frame.received_s <= self.settings.input_timeout_s)
        if self.faulted:
            return
        if not permit or not valid or (frame and frame.button_b):
            if self.active or self.anchor is not None:
                self.stop("暂停：Grip/追踪/界面心跳许可已撤销")
            self.release_seen = False
        elif frame.grip <= .55:
            if self.active or self.anchor is not None:
                self.stop("Grip 已松开")
            if not self.active:
                self.release_seen = True
        if now >= self.next_health:
            check_health(self.robot)
            # 运动/握持准备时10Hz监控；完全暂停时2Hz。每次真正下发前仍会执行
            # 一次不缓存的完整check_health，不能用降频绕过安全状态核对。
            self.next_health = self.clock() + (.1 if self.active or self.anchor is not None else .5)
        if self.active and now >= self.next_motion_poll:
            # JAKA状态读取是同步网络调用。20ms轮询已足够判定短段到位/停止，
            # 避免主循环10ms节拍与30Hz数字孪生反馈叠加成数百次SDK调用每秒。
            self.next_motion_poll = now + .02
            joints, tcp = measured(self.robot)
            # 运行时核对实测关节是否还在已筛查的短段附近。不是按TCP目标伪造关节。
            if not self.stopping and self.path_solutions:
                for i in range(6):
                    lo = min(q[i] for q in self.path_solutions) - math.radians(3)
                    hi = max(q[i] for q in self.path_solutions) + math.radians(3)
                    if not lo <= joints[i] <= hi:
                        raise RuntimeError("实测关节偏离已检查路径；停止会话")
            in_pos = flag(self.robot.is_in_pos(), "到位")
            if self.stopping:
                if in_pos:
                    self.active = self.stopping = self.abort_requested = False
                    self.emit(state="paused", message="停止已确认；松开Grip再握持", target=None)
                elif now - self.active_since > 2.0:
                    raise RuntimeError("motion_abort 后2秒仍未确认停止")
                return
            if in_pos and math.dist(tcp[:3], self.active_target[:3]) <= 1.0:
                self.active = False
                self.completed_segments += 1
                self.measured_path_mm += math.dist(self.active_start_tcp[:3], tcp[:3])
                self.last_completed_measurement = (joints, tcp)
                self.emit(state="following", message="短段到位", target=self.desired,
                          measured_joints=joints, measured_tcp=tcp,
                          completed_segments=self.completed_segments,
                          measured_path_mm=self.measured_path_mm)
            elif now - self.active_since > max(3.0, self.settings.segment_mm / self.settings.speed_mm_s * 3 + 2):
                raise RuntimeError("短段到位超时")
        if (not allow_plan or (self.command_limit is not None and self.commands >= self.command_limit)
                or not permit or not valid or frame.button_b or frame.grip <= .55 or self.stopping):
            return
        if not self.heading:
            # 首次必须在松手时锁定；通过下面的 ready_heading 入口完成。
            return
        if self.source is not None and frame.udp_source != self.source:
            self.stop("Quest 来源改变，释放 Grip 后重捕获")
            self.source = None
            return
        if self.anchor is None:
            if not self.release_seen or frame.grip < .75:
                return
            _, tcp = measured(self.robot)
            self.anchor = tcp
            self.source = frame.udp_source
            self.tracker.reset_grip()
            self.release_seen = False
            self.last_input_position = None
        if self.last_processed != frame.received_s:
            self.last_processed = frame.received_s
            if self.last_input_position is not None and math.dist(frame.position_m, self.last_input_position) > .10:
                self.stop("手柄单帧位置跳变超过10cm，释放Grip重捕获")
                return
            self.last_input_position = frame.position_m
            for name, value in self.tracker.update(frame, 1.0):
                if name == "grip_start":
                    self.binding_id = uuid.uuid4().hex
                    matrix = matmul(self.tracker._axis_map, self.tracker._heading_frame)
                    # 仅显示契约：Unity按相同矩阵在本地逐帧画目标，不获得运动接口。
                    self.emit(state="display_binding", binding={
                        "binding_id": self.binding_id,
                        "anchor_tcp": self.anchor, "reference_m": frame.position_m,
                        "mapping": [x for row in matrix for x in row],
                        "center_tcp": self.center, "radius_mm": self.settings.radius_mm,
                        "position_enabled": True, "position_only": True})
                if name == "pose_delta":
                    delta, _ = value
                    requested = tuple(self.anchor[i] + delta[i] for i in range(3)) + self.anchor[3:]
                    self.desired = clamp_position(requested, self.center, self.settings.radius_mm)
                    self.emit(state="target", message="实时采样目标", target=self.desired)
        if self.active or self.desired is None or now < self.next_plan:
            return
        self.next_plan = now + self.settings.sample_period_s
        joints, tcp = measured(self.robot)
        if not flag(self.robot.is_in_pos(), "段间静止"):
            raise RuntimeError("上一段外出现运动，拒绝并发下发")
        if math.dist(tcp[:3], self.center[:3]) > self.settings.radius_mm + 1.0:
            raise RuntimeError("实测TCP超出本次工作球")
        if math.dist(tcp[:3], self.desired[:3]) < self.settings.deadband_mm:
            return
        try:
            target, solutions = plan_segment(self.robot, joints, tcp, self.desired, self.settings, self.limits)
        except RejectedPath as error:
            self.next_plan = now + .2
            self.emit(state="blocked", message=str(error), target=self.desired)
            return
        # 下发前由 runner 再次取最新输入、核对心跳；SDK规划耗时不能续命旧许可。
        self.pending = (target, [joints, *solutions], frame.received_s, frame.position_m, tcp)

    def ready_heading(self, frame):
        """手柄松开且追踪有效时锁定朝向；握持状态下不重置坐标基准。"""
        if (not self.heading and frame and frame.head_rotation_valid and frame.grip <= .55
                and frame.connected and frame.tracked and frame.valid
                and 0 <= self.clock() - frame.received_s <= self.settings.input_timeout_s):
            self.tracker.lock_heading(frame.head_rotation_xyzw)
            self.heading = True

    def dispatch(self, frame, *, permit, refresh=None):
        """提交候选短段前做最后一次实测与输入核对，防止过时命令。"""
        pending = getattr(self, "pending", None)
        self.pending = None
        if self.command_limit is not None and self.commands >= self.command_limit:
            return  # 独立于运行器循环的第二道闸门：绝不下发超过许可条数。
        if pending is None:
            return
        target, solutions, sample_time, sampled_position, planned_start = pending
        command_distance = math.dist(planned_start[:3], target[:3])
        if (self.path_limit_mm is not None
                and self.commanded_path_mm + command_distance > self.path_limit_mm + 1e-6):
            self.emit(state="path_limit", message="累计计划行程已达本次上限，不再下发",
                      commanded_path_mm=self.commanded_path_mm,
                      path_limit_mm=self.path_limit_mm)
            return
        # 求解后再次核对真实起点，防止平板拖动/其它控制端使检查结果过时。
        # SDK读取期间也可能松Grip，故生产入口必须在读取之后刷新许可和输入。
        check_health(self.robot)
        joints, tcp = measured(self.robot)
        if (not flag(self.robot.is_in_pos(), "下发前静止")
                or math.dist(tcp[:3], planned_start[:3]) > .5
                or max(abs(math.degrees(a-b)) for a,b in zip(joints, solutions[0])) > .2):
            raise RuntimeError("检查后机器人起点改变，拒绝执行旧路径")
        if refresh is not None:
            frame, permit = refresh()
        now = self.clock()
        if not (permit and frame and frame.connected and frame.tracked and frame.valid
                and frame.grip > .55 and not frame.button_b and frame.udp_source == self.source
                and 0 <= now-frame.received_s <= self.settings.input_timeout_s):
            self.stop("下发前许可已失效")
            return
        if now - sample_time > self.settings.input_timeout_s or math.dist(frame.position_m, sampled_position) > .01:
            # 求解期间手柄大幅改变意图：重新采样，不执行过时方向。
            return
        # 只提交一次短直线，容差0，不在控制器里排队或自动绕行未知路径。
        self.active = True  # 调用超时/异常时也要保守请求 abort，防止命令实际已被接受。
        self.abort_requested = False
        self.active_target, self.path_solutions = target, solutions
        self.active_start_tcp = tcp
        self.active_since = now
        self.next_motion_poll = 0.0
        checked(self.robot.linear_move_extend(target, 0, False, self.settings.speed_mm_s,
                                              self.settings.acceleration_mm_s2, 0.0), "linear_move_extend")
        self.commands += 1
        self.commanded_path_mm += command_distance
        self.emit(state="moving", message="JAKA 正在执行短段", target=self.desired)

    def shutdown(self):
        """会话清理时请求停止并等待确认；超时返回 False，由上层报警。"""
        if self.active:
            self.stop("会话关闭：请求停止")
            deadline = self.clock() + 2.0
            while self.clock() < deadline:
                if flag(self.robot.is_in_pos(), "关闭到位确认"):
                    self.active = False
                    return True
                time.sleep(.02)
            return False
        return True
