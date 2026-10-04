"""H8复用联合影子规划器；只有当前静态官方模型可进入有限真机循环。"""
import numpy as np
from ..safety import SafetyFault
from .quest_pose_shadow import QuestPoseShadowPlanner, QuestPoseMapping
from .quest import QuestInput
from .official_local import rotation_vector, rpy_matrix


class QuestPoseLiveMapping(QuestPoseMapping):
    def decode(self,payload,address,stamp):
        frame=super().decode(payload,address,stamp)
        # 8ms循环的全量审计必须可直接JSON编码；不让日志线程的数组异常中断控制。
        frame={k:(v.tolist() if isinstance(v,(np.ndarray,np.generic)) else v) for k,v in frame.items()}
        frame['input_mode']='pose6dof_live';return frame


class QuestPoseLiveReceiver(QuestInput):
    input_mode = 'pose6dof_live'
    def __init__(self,port,basis,audit=None):
        super().__init__(port,basis,mapper=QuestPoseLiveMapping(basis),audit=audit)


class QuestPoseLivePlanner(QuestPoseShadowPlanner):
    def __init__(self, q, cfg, model, origin_tcp, profile):
        if model.get('scope') != 'official_local_XYZ_math_only_not_physical_or_collision_calibration':
            raise SafetyFault('H8 requires fresh official local XYZ model')
        if np.max(np.abs(np.asarray(model['q_origin_rad'])-q)) > np.deg2rad(.005):
            raise SafetyFault('H8 robot shifted during local model preparation')
        super().__init__(q,cfg,model,profile)
        self.origin_tcp=np.asarray(origin_tcp,dtype=float)
        self.raw_rotation_deg=np.zeros(3)

    def step(self, frame, now, finish=False, software_stop=False):
        motion=super().step(frame,now,finish,software_stop)
        if self.phase=='GRIP_6DOF_SHADOW':self.phase='GRIP_6DOF_LIVE'
        if self.stop_reason:self.stop_reason=self.stop_reason.replace('pose shadow','H8')
        return motion

    def check_tcp(self,tcp):
        super().check_tcp(tcp)
        self.raw_rotation_deg=np.rad2deg(rotation_vector(rpy_matrix(tcp)@rpy_matrix(self.origin_tcp).T))
