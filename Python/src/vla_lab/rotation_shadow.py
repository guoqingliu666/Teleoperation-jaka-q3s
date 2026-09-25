"""Tool 1 姿态只读影子：映射手柄旋转、调用厂商逆解，但绝不发送运动。"""
from __future__ import annotations

import math
import uuid

from .quest_vr_input import (
    RelativeQuestTracker, matmul, matrix_rpy, rotation_angle_rad,
    rpy_matrix, scaled_rotation, transpose,
)
from .sampled_follow import checked, six


def _unwrap(values, near):
    """把等价欧拉角放到锚点附近，仅用于连续显示和厂商逆解输入。"""
    return tuple(value + round((reference-value)/(2*math.pi))*2*math.pi
                 for value, reference in zip(values, near, strict=True))


class RotationShadow:
    """只读姿态状态机；对象只需要SDK读接口和 ``kine_inverse``。"""

    def __init__(self, robot, mapping, emit, *, angle_limit_deg=10.0,
                 frame_step_limit_deg=3.0, solution_step_limit_deg=3.0):
        self.robot, self.emit = robot, emit
        self.angle_limit_deg = float(angle_limit_deg)
        self.frame_step_limit_deg = float(frame_step_limit_deg)
        self.solution_step_limit_deg = float(solution_step_limit_deg)
        self.tracker = RelativeQuestTracker(mapping, grip_on=.75, grip_off=.55,
                                            trigger_on=.75, trigger_off=.55)
        self.heading = False
        self.anchor = self.previous_target = self.previous_solution = None
        self.source = None
        self.last_processed = None
        self.solved_frames = self.blocked_frames = 0

    def ready_heading(self, frame, now):
        if (not self.heading and frame and frame.head_rotation_valid
                and frame.connected and frame.tracked and frame.valid
                and frame.grip <= .55 and 0 <= now-frame.received_s <= .15):
            self.tracker.lock_heading(frame.head_rotation_xyzw)
            self.heading = True
            self.emit(state="rotation_shadow_ready",
                      message="姿态只读影子已就绪；按住Grip缓慢转动手柄，不会发送运动")

    def _release(self, message):
        self.emit(state="display_binding", binding=None)
        self.anchor = self.previous_target = self.previous_solution = None
        self.source = None
        self.tracker.reset_grip()
        self.emit(state="rotation_shadow_paused", message=message)

    def process(self, frame, now):
        valid = bool(frame and frame.connected and frame.tracked and frame.valid
                     and frame.rotation_valid and 0 <= now-frame.received_s <= .15)
        if not valid:
            if self.anchor is not None:
                self._release("追踪或手柄姿态失效；释放影子锚点")
            return
        if not self.heading:
            return
        if self.source is not None and frame.udp_source != self.source:
            self._release("Quest来源改变；释放Grip后重捕获")
            return
        if self.last_processed == frame.received_s:
            return
        self.last_processed = frame.received_s
        events = self.tracker.update(frame, 1.0)
        names = [name for name, _ in events]
        if "grip_stop" in names:
            self._release("Grip已松开；姿态影子暂停")
            return
        if "grip_start" in names:
            self.anchor = six(checked(self.robot.get_actual_tcp_position(), "姿态影子锚点TCP"))
            joints = six(checked(self.robot.get_actual_joint_position(), "姿态影子锚点关节"))
            self.previous_solution, self.previous_target = joints, self.anchor
            self.source = frame.udp_source
            matrix = matmul(self.tracker._axis_map, self.tracker._heading_frame)
            self.emit(state="display_binding", binding={
                "binding_id": uuid.uuid4().hex,
                "anchor_tcp": self.anchor,
                "reference_m": frame.position_m,
                "reference_rotation_xyzw": frame.rotation_xyzw,
                "mapping": [x for row in matrix for x in row],
                "center_tcp": self.anchor,
                "radius_mm": 20.0,
                "position_only": False,
                "rotation_enabled": True,
                "rotation_limit_deg": self.angle_limit_deg,
            })
        if self.anchor is None:
            return
        pose = next((value for name, value in events if name == "pose_delta"), None)
        if pose is None or pose[1] is None:
            return
        rotation = pose[1]
        total_deg = math.degrees(rotation_angle_rad(rotation))
        if total_deg > self.angle_limit_deg:
            rotation = scaled_rotation(rotation, self.angle_limit_deg/total_deg)
            total_deg = self.angle_limit_deg
        composed = matmul(rotation, rpy_matrix(tuple(self.anchor[3:])))
        target = self.anchor[:3] + _unwrap(matrix_rpy(composed), self.anchor[3:])
        if self.previous_target is not None:
            step_deg = math.degrees(rotation_angle_rad(
                matmul(rpy_matrix(tuple(target[3:])),
                       transpose(rpy_matrix(tuple(self.previous_target[3:]))))))
            if step_deg > self.frame_step_limit_deg:
                self.blocked_frames += 1
                self.emit(state="rotation_shadow_blocked",
                          message=f"单帧姿态变化{step_deg:.2f}°超过{self.frame_step_limit_deg:.1f}°；未采纳",
                          target=target)
                return
        joints = six(checked(self.robot.get_actual_joint_position(), "姿态影子实测关节"))
        solution = six(checked(self.robot.kine_inverse(joints, target), "姿态影子厂商逆解"))
        solution_step = max(math.degrees(abs(a-b))
                            for a,b in zip(solution, self.previous_solution, strict=True))
        if solution_step > self.solution_step_limit_deg:
            self.blocked_frames += 1
            self.emit(state="rotation_shadow_blocked",
                      message=f"厂商逆解相邻解变化{solution_step:.2f}°超过{self.solution_step_limit_deg:.1f}°；未采纳",
                      target=target)
            return
        self.previous_target, self.previous_solution = target, solution
        self.solved_frames += 1
        self.emit(state="rotation_shadow", message="仅厂商逆解与黄色姿态预览；零运动命令",
                  target=target, total_rotation_deg=total_deg,
                  largest_solution_step_deg=solution_step,
                  largest_solution_vs_actual_deg=max(
                      math.degrees(abs(a-b)) for a,b in zip(solution,joints,strict=True)))
