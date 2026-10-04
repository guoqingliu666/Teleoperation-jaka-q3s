"""真机姿态输入：累计手柄转幅饱和，追踪/单帧突跳/源锁/停止仍严格检查。

只读演练的30度幅度门保留在quest_rotation.py。真实输入不会因正常
累计转过30度被误报为坏包；机器人角域仍由原LPF/Ruckig与原始反馈守护。
"""
import numpy as np
from .quest_j1 import QuestJ1Mapping
from .quest import QuestInput
from vendor.quest_vr_input import quaternion,quaternion_conjugate,quaternion_multiply


class QuestLiveRotationMapping(QuestJ1Mapping):
    def __init__(self,basis):
        super().__init__(basis);self.reference_rotation=None;self.saturated_delta=None

    def decode(self,payload,address,stamp):
        # 父类仍拒绝重放、源切换、字段缺失、手柄单包位置/姿态突跳。
        frame=super().decode(payload,address,stamp)
        q=quaternion(payload['right']['rotation_xyzw'])
        if not frame['tracking_valid'] or not frame.get('deadman'):
            self.reference_rotation=None;self.saturated_delta=None;raw=np.zeros(3)
        else:
            if self.reference_rotation is None:self.reference_rotation=q
            # 相对四元数选短弧，避免180度附近的sin(theta)数值病态。
            relative=np.array(quaternion_multiply(q,quaternion_conjugate(self.reference_rotation)))
            if relative[3]<0:relative=-relative
            length=float(np.linalg.norm(relative[:3]))
            rv=relative[:3]*2 if length<1e-10 else relative[:3]*(2*np.arctan2(length,relative[3])/length)
            raw=np.rad2deg(np.linalg.det(self.basis)*self.basis@rv)
        norm=float(np.linalg.norm(raw))
        if norm>30. and self.saturated_delta is None:self.saturated_delta=raw*(30./norm)
        # 饱和后保持本次意图，直到松Grip；继续转过180/360度不会突然反向。
        saturated=self.saturated_delta is not None
        delta=self.saturated_delta if saturated else raw
        frame.update(input_mode='rotation_shadow',rotation_xyzw=list(q),rotation_delta_deg=delta.tolist(),
            rotation_intent_raw_deg=raw.tolist(),rotation_input_saturated=saturated,
            rotation_mapping='live_short_arc_radial_saturation_v1',hand_input_cap_deg=30.)
        return frame


class QuestLiveRotationReceiver(QuestInput):
    input_mode='rotation_shadow'
    def __init__(self,port,basis,audit=None):
        super().__init__(port,basis,mapper=QuestLiveRotationMapping(basis),audit=audit)
