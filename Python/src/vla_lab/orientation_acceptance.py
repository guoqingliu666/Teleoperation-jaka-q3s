"""Tool 1 小角度、分段式姿态真机验收。

保持TCP XYZ不变；手柄只负责给出一个很小的相对姿态。路径仅用JAKA厂商
``kine_inverse``筛查，并用``linear_move_extend_ori``明确限制姿态速度/加速度。
当前只允许三组写死的验收档位：一次≤1°、三段累计≤3°，或十段累计≤10°；
每条机器人命令始终≤1°。
不进入伺服模式，也不把旧手柄轨迹排队重放。
"""
from __future__ import annotations

import math
import time
import uuid

from .quest_vr_input import (
    RelativeQuestTracker, matmul, matrix_rpy, quaternion_conjugate,
    quaternion_matrix, quaternion_multiply, rotation_angle_rad, rpy_matrix,
    scaled_rotation, transpose,
)
from .sampled_follow import checked, check_health, flag, measured, six


def _unwrap(values, near):
    return tuple(value + round((reference-value)/(2*math.pi))*2*math.pi
                 for value, reference in zip(values, near, strict=True))


def _pose_rotation_error_deg(a, b):
    relative = matmul(rpy_matrix(tuple(a[3:])), transpose(rpy_matrix(tuple(b[3:]))))
    return math.degrees(rotation_angle_rad(relative))


class OrientationAcceptance:
    """与位置跟随器同接口，硬限制为已批准的小角度纯姿态档位。"""

    def __init__(self, robot, mapping, limits_deg, *, emit=None, clock=time.monotonic,
                 command_limit=1, max_total_deg=1.0,
                 orientation_speed_deg_s=1.0, orientation_acceleration_deg_s2=2.0):
        if command_limit not in (1, 3, 10):
            raise ValueError("姿态验收只允许1段、3段或10段固定档位")
        if (command_limit, float(max_total_deg), float(orientation_speed_deg_s),
                float(orientation_acceleration_deg_s2)) not in (
                    (1, 1.0, 1.0, 2.0), (3, 3.0, 2.0, 4.0),
                    (10, 10.0, 5.0, 10.0)):
            raise ValueError("姿态验收参数必须使用代码内固定档位")
        self.robot, self.limits = robot, limits_deg
        self.emit, self.clock = emit or (lambda **_: None), clock
        self.tracker = RelativeQuestTracker(mapping, grip_on=.75, grip_off=.55,
                                            trigger_on=.75, trigger_off=.55)
        self.center = self.anchor = self.initial_joints = None
        self.active_target = self.last_completed_measurement = None
        self.path_solutions = []
        self.pending = None
        self.heading = self.release_seen = False
        self.source = self.last_frame = self.last_processed = None
        self.previous_rotation = None
        self.active = self.stopping = self.abort_requested = False
        self.active_since = self.next_health = self.next_motion_poll = 0.0
        self.commands = self.completed_segments = 0
        self.command_limit = command_limit
        self.max_total_deg = float(max_total_deg)
        self.orientation_speed_deg_s = float(orientation_speed_deg_s)
        self.orientation_acceleration_deg_s2 = float(orientation_acceleration_deg_s2)
        # 与通用验收日志字段兼容；姿态结果另记录orientation_deg。
        self.measured_path_mm = self.commanded_path_mm = 0.0
        self.commanded_orientation_deg = 0.0
        self.binding_id = None

    def initialize(self):
        check_health(self.robot)
        if not flag(self.robot.is_in_pos(), "姿态验收启动到位"):
            raise RuntimeError("姿态验收启动时机械臂必须静止")
        self.initial_joints, self.center = measured(self.robot)
        self.emit(state="orientation_acceptance_ready",
                  message=f"最多{self.command_limit}段、累计≤{self.max_total_deg:g}°姿态验收已就绪；先松Grip，再缓慢转动并持续握持")

    def ready_heading(self, frame):
        if (not self.heading and frame and frame.head_rotation_valid and frame.grip <= .55
                and frame.connected and frame.tracked and frame.valid
                and 0 <= self.clock()-frame.received_s <= .15):
            self.tracker.lock_heading(frame.head_rotation_xyzw)
            self.heading = True

    def _stop(self, reason):
        self.emit(state="display_binding", binding=None)
        self.pending = None
        self.anchor = None
        self.tracker.reset_grip()
        if self.active and not self.abort_requested:
            checked(self.robot.motion_abort(), "姿态验收motion_abort")
            self.abort_requested = self.stopping = True
            self.active_since = self.clock()
        self.emit(state="stopping" if self.active else "paused", message=reason)

    def _plan(self, mapped_rotation, frame):
        hand_total_deg = math.degrees(rotation_angle_rad(mapped_rotation))
        if hand_total_deg < .5:
            return
        session_rotation = scaled_rotation(
            mapped_rotation, min(1.0, self.max_total_deg/hand_total_deg))
        joints, tcp = measured(self.robot)
        if math.dist(tcp[:3], self.center[:3]) > 1.0:
            raise RuntimeError("姿态验收期间TCP位置漂移超过1mm")
        desired_rotation = matmul(session_rotation, rpy_matrix(tuple(self.anchor[3:])))
        current_rotation = rpy_matrix(tuple(tcp[3:]))
        remaining = matmul(desired_rotation, transpose(current_rotation))
        remaining_deg = math.degrees(rotation_angle_rad(remaining))
        if remaining_deg < .5:
            return
        command_rotation = scaled_rotation(remaining, min(1.0, 1.0/remaining_deg))
        command_deg = min(remaining_deg, 1.0)
        count = max(1, math.ceil(command_deg/.2))
        reference = joints
        solutions = []
        target = None
        for index in range(1, count+1):
            step = scaled_rotation(command_rotation, index/count)
            target = tcp[:3] + _unwrap(
                matrix_rpy(matmul(step, current_rotation)), tcp[3:])
            result = self.robot.kine_inverse(reference, target)
            if isinstance(result, tuple) and result and result[0] == -4:
                self.emit(state="orientation_acceptance_blocked", message="厂商逆解不可达；未运动")
                return
            solved = six(checked(result, "姿态验收kine_inverse"))
            if max(math.degrees(abs(a-b)) for a,b in zip(solved,reference,strict=True)) > .75:
                self.emit(state="orientation_acceptance_blocked",
                          message="0.2°样本的厂商逆解关节变化超过0.75°；未运动")
                return
            if max(math.degrees(abs(a-b)) for a,b in zip(solved,joints,strict=True)) > 2.0:
                self.emit(state="orientation_acceptance_blocked",
                          message="1°候选的累计关节变化超过2°；未运动")
                return
            for q,(low,high) in zip(solved,self.limits,strict=True):
                if not low+3 <= math.degrees(q) <= high-3:
                    self.emit(state="orientation_acceptance_blocked",
                              message="候选关节距配置限位不足3°；未运动")
                    return
            solutions.append(solved)
            reference = solved
        self.pending = (target, [joints,*solutions], frame.received_s,
                        frame.rotation_xyzw, tcp, command_deg)
        self.emit(state="orientation_candidate", message="单段≤1°厂商逆解路径已通过；等待下发前复核",
                  target=target, orientation_deg=command_deg,
                  hand_total_orientation_deg=min(hand_total_deg,self.max_total_deg))

    def tick(self, frame, *, permit=True, allow_plan=True):
        now = self.clock()
        if frame is not None:
            self.last_frame = frame
        frame = self.last_frame
        valid = bool(frame and frame.connected and frame.tracked and frame.valid
                     and frame.rotation_valid and 0 <= now-frame.received_s <= .15)
        if not permit or not valid or (frame and frame.button_b):
            if self.active or self.anchor is not None:
                self._stop("姿态验收许可/追踪已撤销")
            self.release_seen = False
        elif frame.grip <= .55:
            if self.active or self.anchor is not None:
                self._stop("Grip已松开")
            if not self.active:
                self.release_seen = True
        if now >= self.next_health:
            check_health(self.robot)
            self.next_health = self.clock()+(.1 if self.active or self.anchor else .5)
        if self.active and now >= self.next_motion_poll:
            self.next_motion_poll = now+.02
            joints,tcp = measured(self.robot)
            if not self.stopping and self.path_solutions:
                for i in range(6):
                    lo=min(q[i] for q in self.path_solutions)-math.radians(1)
                    hi=max(q[i] for q in self.path_solutions)+math.radians(1)
                    if not lo <= joints[i] <= hi:
                        raise RuntimeError("姿态实测关节偏离已检查路径")
            in_pos=flag(self.robot.is_in_pos(),"姿态验收到位")
            if self.stopping:
                if in_pos:
                    self.active=self.stopping=self.abort_requested=False
                elif now-self.active_since>2:
                    raise RuntimeError("姿态验收停止后2秒仍未确认")
                return
            if (in_pos and math.dist(tcp[:3],self.active_target[:3])<=1
                    and _pose_rotation_error_deg(tcp,self.active_target)<=.2):
                self.active=False
                self.completed_segments+=1
                self.last_completed_measurement=(joints,tcp)
                self.emit(state="orientation_following", message="≤1°姿态已实测到位",
                          measured_joints=joints, measured_tcp=tcp,
                          orientation_deg=self.commanded_orientation_deg)
            elif now-self.active_since>4:
                raise RuntimeError("姿态验收到位超时")
        if (not allow_plan or self.commands>=self.command_limit or self.active or not permit or not valid
                or not self.heading or frame.grip<=.55):
            return
        if self.source is not None and frame.udp_source != self.source:
            self._stop("Quest来源改变")
            self.source=None
            return
        if self.anchor is None:
            if not self.release_seen or frame.grip<.75:
                return
            self.initial_joints,self.anchor=measured(self.robot)
            self.source=frame.udp_source
            self.tracker.reset_grip()
            self.release_seen=False
            self.previous_rotation=None
        if self.last_processed == frame.received_s:
            return
        self.last_processed=frame.received_s
        for name,value in self.tracker.update(frame,1.0):
            if name=="grip_start":
                self.binding_id=uuid.uuid4().hex
                matrix=matmul(self.tracker._axis_map,self.tracker._heading_frame)
                self.emit(state="display_binding",binding={
                    "binding_id":self.binding_id,"anchor_tcp":self.anchor,
                    "reference_m":frame.position_m,
                    "reference_rotation_xyzw":frame.rotation_xyzw,
                    "mapping":[x for row in matrix for x in row],
                    "center_tcp":self.anchor,"radius_mm":20.0,
                    "position_only":False,"rotation_enabled":True,
                    "rotation_limit_deg":self.max_total_deg})
            elif name=="pose_delta" and value[1] is not None:
                rotation=value[1]
                if self.previous_rotation is not None:
                    jump=math.degrees(rotation_angle_rad(matmul(rotation,transpose(self.previous_rotation))))
                    if jump>3:
                        # 上一姿态可能停留在上一条机器人命令开始前；必须更新基准，
                        # 否则一次正常的持续转动会被永久误判为每帧都在跳变。
                        # 机器人侧仍由累计角度、单命令1°和厂商逆解路径三重限幅。
                        self.previous_rotation=rotation
                        self.emit(state="orientation_acceptance_blocked",
                                  message=f"新手柄样本相对上一处理点变化{jump:.2f}°；本帧未运动，已从最新姿态继续采样")
                        continue
                self.previous_rotation=rotation
                if self.pending is None:
                    self._plan(rotation,frame)

    def dispatch(self, frame, *, permit, refresh=None):
        pending,self.pending=self.pending,None
        if pending is None or self.commands>=self.command_limit:
            return
        target,solutions,sample_time,sampled_rotation,planned_start,total_deg=pending
        check_health(self.robot)
        joints,tcp=measured(self.robot)
        if (not flag(self.robot.is_in_pos(),"姿态下发前静止")
                or math.dist(tcp[:3],planned_start[:3])>.5
                or _pose_rotation_error_deg(tcp,planned_start)>.1
                or max(math.degrees(abs(a-b)) for a,b in zip(joints,solutions[0],strict=True))>.2):
            raise RuntimeError("姿态检查后机器人起点改变")
        if refresh is not None:
            frame,permit=refresh()
        now=self.clock()
        if not (permit and frame and frame.connected and frame.tracked and frame.valid
                and frame.rotation_valid and frame.grip>.55 and not frame.button_b
                and frame.udp_source==self.source and 0<=now-frame.received_s<=.15):
            return
        raw_delta=quaternion_matrix(quaternion_multiply(frame.rotation_xyzw,
                                                        quaternion_conjugate(sampled_rotation)))
        if now-sample_time>.15 or math.degrees(rotation_angle_rad(raw_delta))>1:
            return
        self.active=True
        self.active_target,self.path_solutions=target,solutions
        self.active_since=now
        self.next_motion_poll=0
        self.commanded_orientation_deg+=total_deg
        # 厂商接口显式限制姿态速度/加速度；XYZ保持不变，平移参数取最低验收档。
        checked(self.robot.linear_move_extend_ori(
            target,0,False,5.0,10.0,0.0,
            math.radians(self.orientation_speed_deg_s),
            math.radians(self.orientation_acceleration_deg_s2)),
            "linear_move_extend_ori")
        self.commands+=1
        self.emit(state="orientation_moving",message=f"JAKA执行第{self.commands}段≤1°姿态验收命令",
                  target=target,orientation_deg=total_deg)

    def shutdown(self):
        if self.active:
            self._stop("会话关闭：请求姿态停止")
            deadline=self.clock()+2
            while self.clock()<deadline:
                if flag(self.robot.is_in_pos(),"姿态关闭到位确认"):
                    self.active=False
                    return True
                time.sleep(.02)
            return False
        return True
