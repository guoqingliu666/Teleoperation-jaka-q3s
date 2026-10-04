"""连续速度输入 → 默认滤波 → 本地 jerk 有界在线轨迹 → 每周期绝对关节目标。"""
import numpy as np
from ruckig import Ruckig,InputParameter,OutputParameter,ControlInterface,Result,Synchronization
from ..configuration import vector,ROOT
from ..filters import CommandFilter
from ..trajectory import Motion
from ..kinematics import ArmKinematics

class VelocityServo:
    def __init__(self,q,cfg):
        self.cfg=cfg; self.dt=cfg['period_s']; self.limits=cfg['limits']
        self.otg=Ruckig(6,self.dt); self.input=InputParameter(6); self.output=OutputParameter(6)
        self.input.control_interface=ControlInterface.Velocity
        self.input.synchronization=Synchronization.No
        self.input.current_position=vector(q).tolist(); self.input.current_velocity=[0.]*6; self.input.current_acceleration=[0.]*6
        self.input.target_velocity=[0.]*6; self.input.target_acceleration=[0.]*6
        self.input.max_velocity=list(self.limits['velocity_rad_s'])
        self.input.max_acceleration=list(self.limits['acceleration_rad_s2'])
        self.input.max_jerk=list(self.limits['jerk_rad_s3'])
        self.filter=CommandFilter(cfg['filters'],self.dt,np.zeros(6)); self.state=Motion.rest(q)
        self.arm=ArmKinematics(ROOT/'models/assembly_reference.urdf'); self.sigma=None; self.boundary_braking=False
    def set_cutoff(self,hz): self.filter.tune_cutoff(hz)
    def joint_velocity(self,axes,space,speed):
        axes=vector(axes)
        if np.max(np.abs(axes))>1: raise ValueError('jog axes must be normalized')
        if not 0<speed<=1: raise ValueError('speed scaling outside (0,1]')
        if space=='joint': return axes*np.array(self.limits['velocity_rad_s'])*speed
        if space!='cartesian': raise ValueError('unknown coordinate space')
        _,jac,_=self.arm.forward(self.state.q); sigma=np.linalg.svd(jac,compute_uv=False)[-1]; self.sigma=float(sigma)
        if sigma<.02: raise ValueError('Cartesian singularity screening: switch to joint mode')
        twist=axes*np.array([.25,.25,.25,.5,.5,.5])*speed
        velocity=np.linalg.solve(jac.T@jac+np.eye(6)*.005**2,jac.T@twist)
        ratio=max(1.,float(np.max(np.abs(velocity)/np.array(self.limits['velocity_rad_s']))))
        return velocity/ratio
    def step(self,axes,space='joint',speed=.1,acceleration=.25,hold=False):
        if not .01<=acceleration<=1: raise ValueError('acceleration scale outside [.01,1]')
        # 先在输入速度空间低通，再算雅可比。笛卡尔时这是 twist 低通，
        # 不是把姿态/位移跳点送进逐周期完整 IK；只保留一个指令低通级。
        axes=vector(axes)
        if np.max(np.abs(axes))>1: raise ValueError('jog axes must be normalized')
        if not 0<speed<=1: raise ValueError('speed scaling outside (0,1]')
        filtered=self.filter.update(np.zeros(6) if hold else axes*speed)
        target=np.zeros(6) if hold else self.joint_velocity(filtered,space,1.)
        # 松手立即要求零速度；下游 OTG 平滑减速，避免低通尾巴继续追手。
        if hold: target=np.zeros(6)
        self.input.max_acceleration=(np.array(self.limits['acceleration_rad_s2'])*acceleration).tolist()
        # 可变速度上限通过目标缩放，OTG 的硬上限不突然下降到当前速度以下。
        q=self.state.q; v=self.state.v; amax=np.array(self.input.max_acceleration); jmax=np.array(self.input.max_jerk)
        # 保守制动余量用于提前降到零；不把目标位置裁剪成限位上的跳点。
        reserve=np.abs(v)*(np.abs(v)/amax+2*amax/jmax+2*self.dt)+np.abs(self.state.a)*(amax/jmax)**2+1e-4
        outward=((q-np.array(self.limits['lower_rad'])<reserve)&(target<0))|((np.array(self.limits['upper_rad'])-q<reserve)&(target>0))
        self.boundary_braking=bool(np.any(outward)); target[outward]=0
        self.input.target_velocity=target.tolist()
        result=self.otg.update(self.input,self.output)
        if result not in (Result.Working,Result.Finished): raise RuntimeError('Ruckig failed: '+str(result))
        nq=np.array(self.output.new_position); nv=np.array(self.output.new_velocity); na=np.array(self.output.new_acceleration)
        jerk=(na-self.state.a)/self.dt
        if np.any(nq<self.limits['lower_rad']) or np.any(nq>self.limits['upper_rad']): raise RuntimeError('joint stopping boundary exhausted')
        self.state=Motion(nq,nv,na,jerk); self.output.pass_to_input(self.input)
        return self.state
