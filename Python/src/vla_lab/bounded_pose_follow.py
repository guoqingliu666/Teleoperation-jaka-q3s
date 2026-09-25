"""受限连续六维跟随：组合完整TCP目标，逐段调用JAKA厂商规划接口。

本模块不做自编逆解，也不使用servo接口。手柄只生成最新目标；标准档每次最多
平移10mm、旋转1°，扩展档每次最多平移20mm、旋转2°。两档均在段内按
2mm/0.2°采样并调用厂商kine_inverse筛查。任何时刻只有一条命令在途，
Grip松开或许可失效即请求停止。
"""
from __future__ import annotations

import math
import time
import uuid

from .orientation_acceptance import _pose_rotation_error_deg, _unwrap
from .quest_vr_input import (
    RelativeQuestTracker, matmul, matrix_rpy, quaternion_conjugate,
    quaternion_matrix, quaternion_multiply, rotation_angle_rad, rpy_matrix,
    scaled_rotation, transpose,
)
from .sampled_follow import (
    RejectedPath, Settings, checked, check_health, clamp_position, flag,
    measured, six,
)


def _clamp_rotation(rotation, center_rotation, maximum_deg):
    """把绝对姿态限制在会话初始姿态附近，避免反复松握扩大姿态许可。"""
    relative = matmul(rotation, transpose(center_rotation))
    angle_deg = math.degrees(rotation_angle_rad(relative))
    if angle_deg <= maximum_deg:
        return rotation
    return matmul(scaled_rotation(relative, maximum_deg / angle_deg), center_rotation)


def plan_pose_segment(robot, joints, tcp, desired, settings, limits_deg,
                      *, max_orientation_step_deg=1.0):
    """对一个短六维段逐点调用厂商逆解；返回完整目标和已检查关节路径。"""
    position_target = clamp_position(desired, tcp, settings.segment_mm)
    current_rotation = rpy_matrix(tuple(tcp[3:]))
    desired_rotation = rpy_matrix(tuple(desired[3:]))
    rotation_delta = matmul(desired_rotation, transpose(current_rotation))
    remaining_deg = math.degrees(rotation_angle_rad(rotation_delta))
    rotation_target = matmul(
        scaled_rotation(rotation_delta, min(1.0, max_orientation_step_deg / remaining_deg))
        if remaining_deg else rotation_delta,
        current_rotation,
    )
    target = tuple(position_target[:3]) + _unwrap(matrix_rpy(rotation_target), tcp[3:])
    distance = math.dist(tcp[:3], target[:3])
    command_angle_deg = _pose_rotation_error_deg(tcp, target)
    samples = max(1, math.ceil(distance / 2.0), math.ceil(command_angle_deg / .2))
    reference = joints
    solutions = []
    target_delta = matmul(rotation_target, transpose(current_rotation))
    for index in range(1, samples + 1):
        ratio = index / samples
        rotation = matmul(scaled_rotation(target_delta, ratio), current_rotation)
        pose = tuple(tcp[i] + (target[i] - tcp[i]) * ratio for i in range(3)) + _unwrap(
            matrix_rpy(rotation), tcp[3:])
        result = robot.kine_inverse(reference, pose)
        if isinstance(result, tuple) and result and result[0] == -4:
            raise RejectedPath("六维短段内厂商逆解不可达")
        solved = six(checked(result, "六维kine_inverse"))
        if max(math.degrees(abs(a-b)) for a,b in zip(solved, reference, strict=True)) > 3.0:
            raise RejectedPath("六维段内相邻逆解关节变化超过3°")
        if max(math.degrees(abs(a-b)) for a,b in zip(solved, joints, strict=True)) > 12.0:
            raise RejectedPath("六维短段累计关节变化超过12°")
        for q,(low,high) in zip(solved, limits_deg, strict=True):
            if not low + 3.0 <= math.degrees(q) <= high - 3.0:
                raise RejectedPath("六维候选关节距配置限位不足3°")
        solutions.append(solved)
        reference = solved
    return target, solutions


class BoundedPoseFollower:
    """单SDK所有者的受限连续六维状态机。"""

    def __init__(self, robot, mapping, settings: Settings, limits_deg, *,
                 emit=None, clock=time.monotonic, rotation_radius_deg=10.0,
                 orientation_speed_deg_s=5.0,
                 orientation_acceleration_deg_s2=10.0,
                 max_orientation_step_deg=1.0):
        self.robot, self.settings, self.limits = robot, settings, limits_deg
        self.emit, self.clock = emit or (lambda **_: None), clock
        self.rotation_radius_deg = float(rotation_radius_deg)
        self.orientation_speed_deg_s = float(orientation_speed_deg_s)
        self.orientation_acceleration_deg_s2 = float(orientation_acceleration_deg_s2)
        self.max_orientation_step_deg = float(max_orientation_step_deg)
        profile=(self.rotation_radius_deg,self.orientation_speed_deg_s,
                 self.orientation_acceleration_deg_s2,self.max_orientation_step_deg,
                 float(settings.radius_mm),float(settings.speed_mm_s),
                 float(settings.acceleration_mm_s2),float(settings.segment_mm))
        standard = profile == (10.0,5.0,10.0,1.0,200.0,30.0,60.0,10.0)
        expanded = profile == (30.0,10.0,20.0,2.0,500.0,50.0,100.0,20.0)
        production = (
            self.max_orientation_step_deg == 2.0
            and settings.segment_mm == 20.0
            and 200.0 <= settings.radius_mm <= 1000.0
            and 5.0 <= settings.speed_mm_s <= 100.0
            and 10.0 <= settings.acceleration_mm_s2 <= 400.0
            and 10.0 <= self.rotation_radius_deg <= 30.0
            and 1.0 <= self.orientation_speed_deg_s <= 20.0
            and self.orientation_acceleration_deg_s2 == min(
                80.0, 4.0*self.orientation_speed_deg_s))
        if not (standard or expanded or production):
            raise ValueError("六维真机参数必须使用已定义的标准档或扩展档")
        self.fresh_position_m=.04 if self.max_orientation_step_deg==2 else .02
        self.fresh_rotation_deg=10.0 if self.max_orientation_step_deg==2 else 5.0
        self.tracker = RelativeQuestTracker(mapping, grip_on=.75, grip_off=.55,
                                            trigger_on=.75, trigger_off=.55)
        self.center = self.center_rotation = self.initial_joints = None
        self.anchor = self.desired = self.active_target = None
        self.active_start_tcp = self.last_completed_measurement = None
        self.path_solutions = []
        self.pending = None
        self.heading = self.release_seen = False
        self.active = self.stopping = self.abort_requested = False
        self.last_frame = self.source = self.last_processed = None
        self.last_input_position = self.previous_mapped_rotation = None
        self.next_plan = self.next_health = self.next_motion_poll = 0.0
        self.active_since = 0.0
        self.commands = self.completed_segments = 0
        self.measured_path_mm = self.commanded_path_mm = 0.0
        self.commanded_orientation_deg = 0.0
        self.binding_id = None

    def initialize(self):
        check_health(self.robot)
        if not flag(self.robot.is_in_pos(), "六维启动到位"):
            raise RuntimeError("六维跟随启动时机械臂必须静止")
        self.initial_joints, self.center = measured(self.robot)
        self.center_rotation = rpy_matrix(tuple(self.center[3:]))
        self.emit(state="pose_ready",
                  message="受限连续六维已就绪；先松Grip，再握住跟随")

    def ready_heading(self, frame):
        if (not self.heading and frame and frame.head_rotation_valid and frame.grip <= .55
                and frame.connected and frame.tracked and frame.valid
                and 0 <= self.clock()-frame.received_s <= self.settings.input_timeout_s):
            self.tracker.lock_heading(frame.head_rotation_xyzw)
            self.heading = True

    def stop(self, reason):
        self.emit(state="display_binding", binding=None)
        self.pending = self.desired = self.anchor = None
        self.binding_id = None
        self.tracker.reset_grip()
        self.release_seen = False
        if self.active and not self.abort_requested:
            checked(self.robot.motion_abort(), "六维motion_abort")
            self.abort_requested = self.stopping = True
            self.active_since = self.clock()
        self.emit(state="stopping" if self.active else "paused", message=reason)

    def _update_target(self, frame, delta, mapped_rotation):
        requested_position = tuple(self.anchor[i] + delta[i] for i in range(3)) + self.anchor[3:]
        position = clamp_position(requested_position, self.center, self.settings.radius_mm)
        desired_rotation = rpy_matrix(tuple(self.anchor[3:]))
        if mapped_rotation is not None:
            desired_rotation = matmul(mapped_rotation, desired_rotation)
        desired_rotation = _clamp_rotation(
            desired_rotation, self.center_rotation, self.rotation_radius_deg)
        self.desired = tuple(position[:3]) + _unwrap(
            matrix_rpy(desired_rotation), self.center[3:])
        self.emit(state="target", message="实时六维采样目标", target=self.desired)

    def tick(self, frame, *, permit=True, allow_plan=True):
        now = self.clock()
        if frame is not None:
            self.last_frame = frame
        frame = self.last_frame
        valid = bool(frame and frame.connected and frame.tracked and frame.valid
                     and frame.rotation_valid
                     and 0 <= now-frame.received_s <= self.settings.input_timeout_s)
        if not permit or not valid or (frame and frame.button_b):
            if self.active or self.anchor is not None:
                self.stop("六维跟随许可/追踪已撤销")
            self.release_seen = False
        elif frame.grip <= .55:
            if self.active or self.anchor is not None:
                self.stop("Grip已松开")
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
                    lo=min(q[i] for q in self.path_solutions)-math.radians(3)
                    hi=max(q[i] for q in self.path_solutions)+math.radians(3)
                    if not lo <= joints[i] <= hi:
                        raise RuntimeError("六维实测关节偏离已检查路径")
            in_pos=flag(self.robot.is_in_pos(),"六维到位")
            if self.stopping:
                if in_pos:
                    self.active=self.stopping=self.abort_requested=False
                    self.emit(state="paused",message="停止已确认；松开Grip后可重新握持")
                elif now-self.active_since>2:
                    raise RuntimeError("六维motion_abort后2秒仍未确认停止")
                return
            if (in_pos and math.dist(tcp[:3],self.active_target[:3])<=1
                    and _pose_rotation_error_deg(tcp,self.active_target)<=.2):
                self.active=False
                self.completed_segments+=1
                self.measured_path_mm += math.dist(self.active_start_tcp[:3],tcp[:3])
                self.last_completed_measurement=(joints,tcp)
                self.emit(state="pose_following",message="六维短段实测到位",
                          measured_joints=joints,measured_tcp=tcp,
                          completed_segments=self.completed_segments)
            elif now-self.active_since>4:
                raise RuntimeError("六维短段到位超时")
        if (not allow_plan or self.active or self.stopping or not permit or not valid
                or not self.heading or frame.grip<=.55):
            return
        if self.source is not None and frame.udp_source != self.source:
            self.stop("Quest来源改变；释放Grip后重捕获")
            self.source=None
            return
        if self.anchor is None:
            if not self.release_seen or frame.grip<.75:
                return
            _,self.anchor=measured(self.robot)
            self.source=frame.udp_source
            self.tracker.reset_grip()
            self.release_seen=False
            self.last_input_position=self.previous_mapped_rotation=None
        if self.last_processed != frame.received_s:
            self.last_processed=frame.received_s
            if self.last_input_position is not None and math.dist(
                    frame.position_m,self.last_input_position)>.10:
                self.stop("手柄位置单帧跳变超过10cm；释放Grip重捕获")
                return
            self.last_input_position=frame.position_m
            for name,value in self.tracker.update(frame,1.0):
                if name=="grip_start":
                    self.binding_id=uuid.uuid4().hex
                    matrix=matmul(self.tracker._axis_map,self.tracker._heading_frame)
                    self.emit(state="display_binding",binding={
                        "binding_id":self.binding_id,"anchor_tcp":self.anchor,
                        "reference_m":frame.position_m,
                        "reference_rotation_xyzw":frame.rotation_xyzw,
                        "mapping":[x for row in matrix for x in row],
                        "center_tcp":self.center,"radius_mm":self.settings.radius_mm,
                        "position_only":False,"rotation_enabled":True,
                        "rotation_limit_deg":self.rotation_radius_deg})
                elif name=="pose_delta":
                    delta,rotation=value
                    if rotation is None:
                        continue
                    if self.previous_mapped_rotation is not None:
                        jump=math.degrees(rotation_angle_rad(matmul(
                            rotation,transpose(self.previous_mapped_rotation))))
                        if jump>20:
                            # 丢弃一个不可信样本但更新基准，避免永久卡死。
                            self.previous_mapped_rotation=rotation
                            self.emit(state="pose_blocked",
                                      message=f"手柄姿态跨样本变化{jump:.1f}°；本帧不运动")
                            continue
                    self.previous_mapped_rotation=rotation
                    self._update_target(frame,delta,rotation)
        if self.desired is None or now<self.next_plan:
            return
        self.next_plan=now+self.settings.sample_period_s
        joints,tcp=measured(self.robot)
        if not flag(self.robot.is_in_pos(),"六维段间静止"):
            raise RuntimeError("六维上一段外出现运动")
        if math.dist(tcp[:3],self.center[:3])>self.settings.radius_mm+1:
            raise RuntimeError("六维实测TCP超出工作球")
        position_error=math.dist(tcp[:3],self.desired[:3])
        rotation_error=_pose_rotation_error_deg(tcp,self.desired)
        if position_error<self.settings.deadband_mm and rotation_error<.5:
            return
        try:
            target,solutions=plan_pose_segment(
                self.robot,joints,tcp,self.desired,self.settings,self.limits,
                max_orientation_step_deg=self.max_orientation_step_deg)
        except RejectedPath as error:
            self.next_plan=now+.2
            self.emit(state="pose_blocked",message=str(error),target=self.desired)
            return
        self.pending=(target,[joints,*solutions],frame.received_s,
                      frame.position_m,frame.rotation_xyzw,tcp)

    def dispatch(self, frame, *, permit, refresh=None):
        pending,self.pending=self.pending,None
        if pending is None:
            return
        target,solutions,sample_time,sampled_position,sampled_rotation,planned_start=pending
        check_health(self.robot)
        joints,tcp=measured(self.robot)
        if (not flag(self.robot.is_in_pos(),"六维下发前静止")
                or math.dist(tcp[:3],planned_start[:3])>.5
                or _pose_rotation_error_deg(tcp,planned_start)>.1
                or max(math.degrees(abs(a-b)) for a,b in zip(
                    joints,solutions[0],strict=True))>.2):
            raise RuntimeError("六维检查后机器人起点改变")
        if refresh is not None:
            frame,permit=refresh()
        now=self.clock()
        if not (permit and frame and frame.connected and frame.tracked and frame.valid
                and frame.rotation_valid and frame.grip>.55 and not frame.button_b
                and frame.udp_source==self.source
                and 0<=now-frame.received_s<=self.settings.input_timeout_s):
            self.stop("六维下发前许可失效")
            return
        fresh_rotation_delta=quaternion_matrix(quaternion_multiply(
            frame.rotation_xyzw,quaternion_conjugate(sampled_rotation)))
        if (now-sample_time>self.settings.input_timeout_s
                or math.dist(frame.position_m,sampled_position)>self.fresh_position_m
                or math.degrees(rotation_angle_rad(fresh_rotation_delta))>self.fresh_rotation_deg):
            return
        self.active=True
        self.abort_requested=False
        self.active_target,self.path_solutions=target,solutions
        self.active_start_tcp=tcp
        self.active_since=now
        self.next_motion_poll=0
        distance=math.dist(tcp[:3],target[:3])
        angle=_pose_rotation_error_deg(tcp,target)
        checked(self.robot.linear_move_extend_ori(
            target,0,False,self.settings.speed_mm_s,
            self.settings.acceleration_mm_s2,0.0,
            math.radians(self.orientation_speed_deg_s),
            math.radians(self.orientation_acceleration_deg_s2)),
            "六维linear_move_extend_ori")
        self.commands+=1
        self.commanded_path_mm+=distance
        self.commanded_orientation_deg+=angle
        self.emit(state="pose_moving",message="JAKA执行受限六维短段",
                  target=target,translation_mm=distance,orientation_deg=angle)

    def shutdown(self):
        if self.active:
            self.stop("六维会话关闭：请求停止")
            deadline=self.clock()+2
            while self.clock()<deadline:
                if flag(self.robot.is_in_pos(),"六维关闭到位确认"):
                    self.active=False
                    return True
                time.sleep(.02)
            return False
        return True
