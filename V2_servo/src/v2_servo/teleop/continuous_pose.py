"""连续Quest相对位姿→twist→低通→动态Jacobian→Ruckig。

复用已存在的输入审查、滤波和VelocityServo。此候选在影子运行器中调用；
没有SDK所有权/运动权限，也没有用CAD显示几何冒充碰撞几何。
"""
import copy
import numpy as np
from ..configuration import vector
from ..kinematics import rotate
from ..safety import SafetyFault
from .motion import VelocityServo
from .global_kinematics import rotation_log
from .quest_j1 import QuestJ1Mapping
from .quest import QuestInput
from vendor.quest_vr_input import quaternion,quaternion_matrix

def limited(v,maximum):
    n=np.linalg.norm(v)
    return v/max(1.,n/maximum)

class ContinuousMapping(QuestJ1Mapping):
    def decode(self,payload,address,stamp):
        f=super().decode(payload,address,stamp);right=payload['right'];p=right['position_m']
        f['position_m']=vector([p[k] for k in ('x','y','z')] if isinstance(p,dict) else p,3).tolist()
        f['rotation_xyzw']=list(quaternion(right['rotation_xyzw']))
        # 不再继承H9每次握持30°饱和；父类的源锁/单帧跳变/追踪检查保持。
        f['input_mode']='continuous_relative_pose'
        return {k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in f.items()}

class ContinuousReceiver(QuestInput):
    def __init__(self,port,basis,audit=None):
        super().__init__(port,basis,mapper=ContinuousMapping(basis),audit=audit)

class GlobalVelocityServo(VelocityServo):
    def __init__(self,q,cfg,model):
        super().__init__(q,cfg);self.arm=model;self.feedback_scale=1.
    def joint_velocity(self,axes,space,speed):
        _,jac,_=self.arm.forward(self.state.q);weight=np.diag([1,1,1,.2,.2,.2]);a=weight@jac
        self.sigma=float(np.linalg.svd(a,compute_uv=False)[-1])
        if self.sigma<.005:raise SafetyFault('Continuous global Jacobian near singularity')
        twist=vector(axes)*np.array([.25]*3+[.5]*3)*speed
        v=np.linalg.solve(a.T@a+np.eye(6)*.0001**2,a.T@(weight@twist))
        return v/max(1.,float(np.max(np.abs(v)/np.array(self.limits['velocity_rad_s']))))*self.feedback_scale

class ContinuousPoseServo:
    def __init__(self,q,cfg,model,profile,basis):
        if not cfg['filters']['enabled']:raise SafetyFault('Continuous filtering must stay enabled')
        self.p=dict(profile);self.cfg=copy.deepcopy(cfg);self.model=model;self.basis=np.asarray(basis,float)
        if self.basis.shape!=(3,3) or not np.allclose(self.basis.T@self.basis,np.eye(3)):raise ValueError('bad VR basis')
        for key in ('translation_gain','rotation_gain','linear_speed_m_s','angular_speed_rad_s',
                    'position_servo_gain_s_inv','rotation_servo_gain_s_inv','controlled_stop_budget_s'):
            if not np.isfinite(profile[key]) or profile[key]<=0:raise ValueError('invalid '+key)
        if profile['linear_speed_m_s']>.25 or profile['angular_speed_rad_s']>.5:raise ValueError('twist normalization exceeded')
        for key in ('position_deadzone_m','orientation_error_deadzone_rad'):
            if not np.isfinite(profile[key]) or profile[key]<0:raise ValueError('invalid deadzone')
        for key,d in (('velocity',1),('acceleration',2),('jerk',3)):
            value=profile[f'joint_{key}_deg_s'+str(d) if d>1 else f'joint_{key}_deg_s']
            if not np.isfinite(value) or value<=0:raise ValueError('joint dynamics')
            self.cfg['limits'][f'{key}_rad_s'+str(d) if d>1 else f'{key}_rad_s']=[float(np.deg2rad(value))]*6
        self.servo=GlobalVelocityServo(q,self.cfg,model);self.phase='WAIT_RELEASE';self.released=False;self.held=False
        self.locked=False;self.stop_reason=None;self.stop_started=None;self.last_seq=-1;self.last_stamp=-1.
        self.hand_reference=None;self.robot_reference=None;self.hand_rotation=None;self.goal=None
        self.events=[];self.pending_stop=None;self.stop_peak_q=None;self.last_frame=None

    @property
    def state(self):return self.servo.state
    def quiet(self):return bool(np.max(np.abs(self.state.v))<1e-5 and np.max(np.abs(self.state.a))<1e-4)
    def request_stop(self,reason,now,lock):
        if self.pending_stop is None and (self.held or not self.quiet()):
            self.pending_stop=dict(reason=reason,started_s=now,q_start=self.state.q.copy(),tcp_start=self.model.forward(self.state.q)[0][:3,3].copy())
        self.held=False;self.released=False;self.hand_reference=None;self.hand_rotation=None
        if lock and not self.locked:self.locked=True;self.stop_reason=reason;self.stop_started=now

    def step(self,frame,now,actual_q,feedback_stamp,hardware_fault=None):
        # 硬件/反馈失效：调用者必须立即保护停止，不能为了平滑继续输出目标。
        if hardware_fault:raise SafetyFault('protective stop: '+str(hardware_fault))
        if not 0<=now-feedback_stamp<=self.cfg['safety']['feedback_timeout_s']:raise SafetyFault('feedback invalid/stale')
        actual=vector(actual_q);gap=float(np.max(np.abs(self.state.q-actual)))
        if gap>np.deg2rad(.05):raise SafetyFault('raw tracking >0.05degree')
        self.servo.feedback_scale=float(np.clip((np.deg2rad(.025)-gap)/np.deg2rad(.01),0,1))
        valid=frame is not None and 0<=now-frame['stamp']<=self.cfg['safety']['input_timeout_s'] and frame.get('tracking_valid')
        if valid:
            sequence=frame['sequence'];stamp=frame['stamp']
            if sequence<self.last_seq or stamp<self.last_stamp or (sequence==self.last_seq and stamp!=self.last_stamp):raise SafetyFault('input reordered/retimestamped')
        new_frame=bool(valid and frame['sequence']!=self.last_seq)
        if not valid:self.request_stop('input timeout/tracking loss',now,True)
        elif frame.get('software_stop'):self.request_stop('software stop',now,True)
        elif not frame.get('deadman'):
            if self.held:self.request_stop('grip release',now,False)
            if frame.get('grip',1)<.3 and self.quiet():
                # 超时后必须看到新鲜、松开的输入且已停稳；软件停止则需显式重启。
                if self.stop_reason!='software stop':self.locked=False;self.stop_started=None
                self.released=not self.locked
        moving=bool(valid and frame.get('deadman') and self.released and not self.locked)
        current=self.model.forward(self.state.q)[0];twist=np.zeros(6)
        if moving:
            hand=vector(frame['position_m'],3);rotation=np.array(quaternion_matrix(frame['rotation_xyzw']))
            if not self.held:
                if not self.quiet():moving=False
                else:
                    self.held=True;self.hand_reference=hand.copy();self.robot_reference=current.copy()
                    self.hand_rotation=rotation.copy();self.goal=current.copy()
            if moving:
                d=self.basis@(hand-self.hand_reference);n=np.linalg.norm(d)
                d=d*max(0.,1-self.p['position_deadzone_m']/max(n,1e-12))
                self.goal[:3,3]=self.robot_reference[:3,3]+d*self.p['translation_gain']
                if new_frame:
                    rv=rotation_log(self.basis@(rotation@self.hand_rotation.T)@self.basis.T)*self.p['rotation_gain']
                    angle=np.linalg.norm(rv)
                    if angle>1e-12:self.goal[:3,:3]=rotate(rv/angle,angle)@self.goal[:3,:3]
                    self.hand_rotation=rotation.copy()
                twist[:3]=limited((self.goal[:3,3]-current[:3,3])*self.p['position_servo_gain_s_inv'],self.p['linear_speed_m_s'])
                error=rotation_log(self.goal[:3,:3]@current[:3,:3].T);n=np.linalg.norm(error)
                error*=max(0.,1-self.p['orientation_error_deadzone_rad']/max(n,1e-12))
                twist[3:]=limited(error*self.p['rotation_servo_gain_s_inv'],self.p['angular_speed_rad_s'])
        if valid:self.last_seq=frame['sequence'];self.last_stamp=frame['stamp'];self.last_frame=frame
        axes=twist/np.array([.25]*3+[.5]*3)
        motion=self.servo.step(axes,'cartesian',1.,1.,hold=not moving)
        self.phase='LOCKED_STOP' if self.locked else ('FOLLOW' if moving else 'RELEASE_BRAKE')
        if self.pending_stop is not None and self.quiet():
            s=self.pending_stop;tcp=self.model.forward(motion.q)[0][:3,3]
            self.events.append(dict(reason=s['reason'],stop_s=now-s['started_s'],
                command_stop_distance_mm=float(np.linalg.norm(tcp-s['tcp_start'])*1000),physical_stop_validated=False))
            self.pending_stop=None
        if self.locked and not self.quiet() and now-self.stop_started>self.p['controlled_stop_budget_s']:raise SafetyFault('controlled stop budget exceeded')
        return motion
