"""H5：X/Y逐轴验收，复用H4已运行的标量Grip/滤波/Ruckig与关节保护。

右手抬升仍是明确的单轴测试旋钮；不把它伪称为已标定的三维手柄映射。
轴只在退出伺服、机器人静止后的新会话中选定，不允许运动中切换。
"""
import numpy as np
from ..configuration import vector
from ..safety import SafetyFault
from .quest_tcp_z import QuestTCPZJog
from .official_local import rpy_matrix,rotation_vector


class QuestTCPAxisJog(QuestTCPZJog):
    def __init__(self,q,cfg,model,origin_tcp):
        self.axis=model['axis']
        if self.axis not in ('X','Y'):raise ValueError('H5 axis is X/Y; Z uses H4')
        self.axis_index='XYZ'.index(self.axis)
        projection=dict(model,joint_gain_rad_per_mm_Z=model['joint_gain_rad_per_mm_'+self.axis],
                        maximum_z_mm=model['maximum_axis_mm'])
        super().__init__(q,cfg,projection,origin_tcp)
        self.maximum_axis_mm=self.maximum_z_mm

    @property
    def axis_target_mm(self):return self.z_target_mm

    def check(self,candidate,actual,velocity):
        try:super().check(candidate,actual,velocity)
        except SafetyFault as exc:
            raise SafetyFault(str(exc).replace('H4','H5').replace('local Z','local '+self.axis)) from exc

    def check_tcp(self,tcp):
        delta=vector(tcp)-self.origin_tcp
        if (np.linalg.norm(np.delete(delta[:3],self.axis_index))>.25 or
            not -.15<=delta[self.axis_index]<=self.maximum_axis_mm+.15):
            raise SafetyFault('H5 raw Tool1 TCP '+self.axis+' translation envelope')
        angle=np.linalg.norm(rotation_vector(rpy_matrix(tcp)@rpy_matrix(self.origin_tcp).T))
        if angle>np.deg2rad(.1):raise SafetyFault('H5 raw Tool1 orientation envelope')
