"""H4：复用已验收Grip/滤波/标量Ruckig，经官方局部雅可比投影到六关节。

标量s单位mm；输出q/v/a/j仍是rad及对应导数，不是每周期完整IK。
仅Tool1基坐标+Z微动；正式全工作空间/6DoF另行验收。
"""
import numpy as np
from ..configuration import vector
from ..safety import SafetyFault
from ..trajectory import Motion
from .quest_j1 import QuestJ1Jog


class QuestTCPZJog:
    def __init__(self,q,cfg,model,origin_tcp):
        self.q0=vector(q);self.cfg=cfg;self.limits=cfg['limits'];self.origin_tcp=vector(origin_tcp)
        self.gain=vector(model['joint_gain_rad_per_mm_Z']);self.maximum_z_mm=float(model['maximum_z_mm'])
        self.jac=np.array(model['jacobian_m_rad']);self.state=Motion.rest(q)
        if not .05<=self.maximum_z_mm<=.5 or np.max(np.abs(self.gain))<1e-6:
            raise SafetyFault('H4 invalid reviewed local Z envelope/gain')
        # 已有标量Grip/滤波/Ruckig逻辑作为虚拟轨迹，再映射物理六关节。
        self.scalar=QuestJ1Jog(np.zeros(6),cfg)
        self.scalar.delta=self.maximum_z_mm
        gain_peak=float(np.max(np.abs(self.gain)))
        self.speed=min(.25,float(np.deg2rad(.05)/gain_peak))
        self.acceleration=min(.5,float(np.deg2rad(.2)/gain_peak))
        self.jerk=min(2.,float(np.deg2rad(1.)/gain_peak))
        self.scalar.speed=self.speed;self.scalar.acceleration=self.acceleration;self.scalar.jerk=self.jerk
        self.scalar.input.max_velocity=[self.speed]*6
        self.scalar.input.max_acceleration=[self.acceleration]*6
        self.scalar.input.max_jerk=[self.jerk]*6
        endpoints=np.array([self.q0,self.q0+self.gain*self.maximum_z_mm])
        if np.any(endpoints<self.limits['lower_rad']) or np.any(endpoints>self.limits['upper_rad']):
            raise SafetyFault('H4 local Z endpoint outside configured joint domain')

    @property
    def phase(self):return self.scalar.phase
    @property
    def finished(self):return self.scalar.finished
    @property
    def stop_reason(self):return self.scalar.stop_reason
    @property
    def release_stops(self):return self.scalar.release_stops
    @property
    def last_intent(self):return self.scalar.last_intent
    @property
    def released(self):return self.scalar.released
    @released.setter
    def released(self,v):self.scalar.released=v
    @property
    def z_target_mm(self):return float(self.scalar.state.q[0])

    def wait_ready(self,latest,now):return self.scalar.wait_ready(latest,now)
    def step(self,latest,now,finish=False,software_stop=False):
        m=self.scalar.step(latest,now,finish=finish,software_stop=software_stop)
        self.state=Motion(self.q0+self.gain*m.q[0],self.gain*m.v[0],
                          self.gain*m.a[0],self.gain*m.j[0])
        return self.state

    def check(self,candidate,actual,velocity):
        q=vector(candidate);actual=vector(actual);dq=vector(velocity)
        s=float(self.gain@(q-self.q0)/(self.gain@self.gain))
        if np.max(np.abs(q-(self.q0+self.gain*s)))>1e-10 or not -1e-6<=s<=self.maximum_z_mm+1e-6:
            raise SafetyFault('H4 command left official local Z line/envelope')
        if np.any(q<self.limits['lower_rad']) or np.any(q>self.limits['upper_rad']):
            raise SafetyFault('H4 configured joint limit')
        if np.max(np.abs(q-self.q0))>np.deg2rad(.1)+1e-9:
            raise SafetyFault('H4 target outside 0.1 degree per-joint trust region')
        if np.max(np.abs(actual-self.q0))>np.deg2rad(.12):
            raise SafetyFault('H4 raw measured joint envelope')
        if np.max(np.abs(q-actual))>np.deg2rad(.05):raise SafetyFault('H4 raw tracking >0.05 degree')
        if np.max(np.abs(dq))>np.deg2rad(.5):raise SafetyFault('H4 raw speed >0.5 degree/s')
        if (np.max(np.abs(self.state.v))>np.deg2rad(.05)+1e-9 or
            np.max(np.abs(self.state.a))>np.deg2rad(.2)+1e-9 or
            np.max(np.abs(self.state.j))>np.deg2rad(1.)+1e-7):
            raise SafetyFault('H4 physical joint derivative envelope')

    def check_tcp(self,tcp):
        delta=vector(tcp)-self.origin_tcp
        if np.linalg.norm(delta[:2])>.25 or not -.15<=delta[2]<=self.maximum_z_mm+.15:
            raise SafetyFault('H4 raw Tool1 TCP translation envelope')
        from .official_local import rpy_matrix,rotation_vector
        angle=np.linalg.norm(rotation_vector(rpy_matrix(tcp)@rpy_matrix(self.origin_tcp).T))
        if angle>np.deg2rad(.1):raise SafetyFault('H4 raw Tool1 orientation envelope')
