"""H3现场验证适配器：仅右Grip＋摇杆Y控制J1，不是6DoF映射。

复用Quest源校验与H2保护包络；范围0..+0.1°，松手jerk有界减速。
"""
import time
import numpy as np
from ruckig import ControlInterface,Result
from .quest import QuestVelocity,QuestInput
from .single_axis import J1Roundtrip
from ..filters import CommandFilter
from ..trajectory import Motion
from ..safety import SafetyFault
from ..configuration import vector


class QuestJ1Mapping(QuestVelocity):
    def decode(self,payload,address,stamp):
        result=super().decode(payload,address,stamp)
        right=payload['right']
        for name in ('grip_available','thumbstick_available','stick_click_available'):
            if right.get(name) is not True:raise ValueError('H3 missing tested control: '+name)
        stick=right.get('thumbstick',{})
        y=stick.get('y') if isinstance(stick,dict) else None
        if isinstance(y,bool) or not isinstance(y,(int,float)) or not np.isfinite(y) or abs(y)>1.001:
            raise ValueError('H3 invalid right stick Y')
        result['stick_y']=float(np.clip(y,-1,1));result['grip']=float(right['grip'])
        result['source_time_s']=payload['unity_time_s']
        result['tracking_valid']=bool(right['connected'] and right['tracked'] and right['pose_valid']
                                      and not result['state'].startswith('追踪无效'))
        result['stick_clicked']=right['thumbstick_click']
        return result


class QuestJ1PoseMapping(QuestJ1Mapping):
    """验证手柄位移链路：Grip重绑参考，右手Unity相对+Y只驱动J1测试轴。

    20mm相对抬手归一化为1；不是TCP坐标标定，也不发布六轴运动权限。
    """
    def __init__(self,basis):super().__init__(basis);self.reference_y=None
    def decode(self,payload,address,stamp):
        result=super().decode(payload,address,stamp)
        position=payload['right'].get('position_m')
        p=vector([position[k] for k in ('x','y','z')] if isinstance(position,dict) else position,3)
        if not result['tracking_valid'] or not result.get('deadman'):
            self.reference_y=None;delta=0.
        else:
            if self.reference_y is None:self.reference_y=float(p[1])
            delta=float(p[1]-self.reference_y)
        result.update(input_mode='pose',position_m=p.tolist(),reference_y_m=self.reference_y,
                      displacement_y_m=delta,axis_y=float(np.clip(delta/.020,-1,1)))
        return result


class QuestJ1Receiver(QuestInput):
    def __init__(self,port,audit=None,input_mode='joystick'):
        # 在接收线程运行前安装专用映射，避免启动竞争。
        if input_mode not in ('joystick','pose'):raise ValueError('unsupported commissioning input mode')
        self.input_mode=input_mode
        mapper=QuestJ1PoseMapping(np.eye(3)) if input_mode=='pose' else QuestJ1Mapping(np.eye(3))
        super().__init__(port,np.eye(3),mapper=mapper,audit=audit)


class QuestJ1Jog(J1Roundtrip):
    def __init__(self,q,cfg):
        super().__init__(q,cfg)
        self.speed=float(np.deg2rad(.05))
        self.input.control_interface=ControlInterface.Velocity
        self.input.max_velocity=[self.speed]*6
        self.input.target_velocity=[0.]*6
        self.filter=CommandFilter(cfg['filters'],self.dt,np.zeros(6))
        self.phase='WAIT_RELEASE';self.released=False;self.locked=False;self.finished=False
        self.last_frame=None;self.stop_reason=None;self.last_intent=None
        self.release_stops=0;self.was_gripped=False

    def wait_ready(self,latest,now):
        return ready_input(latest,now)

    def step(self,latest,now,finish=False,software_stop=False):
        valid=latest is not None and 0<=now-latest['stamp']<=.08
        # 正常Grip松开与超时分开：失效锁停不接受重新握持，须退出重启。
        if not valid:
            self.locked=True;self.stop_reason='Quest input lost/stale';self.phase='LOCKED_STOP'
        elif software_stop or latest.get('software_stop'):
            self.locked=True;self.stop_reason='right stick software stop';self.phase='LOCKED_STOP'
        elif latest.get('state','').startswith('追踪无效'):
            self.locked=True;self.stop_reason='Quest tracking invalid';self.phase='LOCKED_STOP'
        if finish:self.locked=True;self.stop_reason='session duration stop';self.phase='LOCKED_STOP'
        moving=False;axes=np.zeros(6)
        if valid:
            self.last_intent={k:latest.get(k) for k in ('sequence','stamp','source_time_s','grip','stick_y','axis_y',
                                                       'input_mode','position_m','reference_y_m','displacement_y_m','deadman','state')}
            if latest.get('grip',1)<.3:
                if self.was_gripped:self.release_stops+=1
                self.released=True;self.was_gripped=False
            moving=bool(not self.locked and self.released and latest.get('deadman'))
            if moving:
                self.was_gripped=True
                y=latest.get('axis_y',latest['stick_y']);axes[0]=0 if abs(y)<.15 else np.sign(y)*(abs(y)-.15)/.85
                self.phase=('GRIP_POSE_J1' if latest.get('input_mode')=='pose' else 'GRIP_J1') if axes[0] else 'GRIP_ZERO'
            elif not self.locked:self.phase='RELEASE_BRAKE'
        filtered=self.filter.update(axes)
        velocity=filtered*self.speed if moving and axes[0] else np.zeros(6)
        v=abs(self.state.v[0]);a=abs(self.state.a[0])
        reserve=v*(v/self.acceleration+2*self.acceleration/self.jerk+2*self.dt)+a*(self.acceleration/self.jerk)**2+np.deg2rad(.002)
        displacement=self.state.q[0]-self.q0[0]
        if velocity[0]>0 and self.delta-displacement<reserve:velocity[0]=0.;self.phase='UPPER_BRAKE'
        if velocity[0]<0 and displacement<reserve:velocity[0]=0.;self.phase='LOWER_BRAKE'
        self.input.target_velocity=velocity.tolist()
        result=self.otg.update(self.input,self.output)
        if result not in (Result.Working,Result.Finished):raise SafetyFault('H3 Ruckig failed: '+str(result))
        q=np.array(self.output.new_position);v=np.array(self.output.new_velocity);a=np.array(self.output.new_acceleration)
        self.state=Motion(q,v,a,(a-self.state.a)/self.dt);self.output.pass_to_input(self.input)
        # 锁停/限时结束在减速完毕后退出；原始故障仍由上层立即厂家停止。
        quiet=np.max(np.abs(v))<1e-6 and np.max(np.abs(a))<1e-5
        self.finished=bool(self.locked and quiet)
        return self.state


def ready_input(latest,now):
    """必须是新鲜、可追踪且Grip/摇杆均释放的真实输入。不会发机器人命令。"""
    return bool(latest is not None and 0<=now-latest['stamp']<=.08
                and latest.get('tracking_valid',not latest.get('state','').startswith('追踪无效'))
                and latest.get('grip',1)<.3 and abs(latest.get('axis_y',latest.get('stick_y',1)))<.15
                and not latest.get('stick_clicked') and not latest.get('software_stop'))


def input_readiness(latest,now,error=''):
    if latest is None:return 'WAIT_UDP_OR_FIELDS: '+(error or 'waiting Unity input')
    if not 0<=now-latest['stamp']<=.08:return 'WAIT_FRESH_INPUT'
    if not latest.get('tracking_valid',not latest.get('state','').startswith('追踪无效')):return 'WAIT_RIGHT_TRACKING: wear headset, keep right controller visible'
    if latest.get('stick_clicked') or latest.get('software_stop'):return 'RELEASE_RIGHT_STICK_CLICK'
    if latest.get('grip',1)>=.3:return 'RELEASE_RIGHT_GRIP'
    if abs(latest.get('axis_y',latest.get('stick_y',1)))>=.15:return 'CENTER_INPUT_AXIS'
    return 'QUEST_INPUT_READY_NO_ROBOT_COMMANDS'


class QuestInputRehearsal:
    """只读操作演练：先释放→Grip+正Y实际读回→再释放，绝不拥有SDK。"""
    def __init__(self):self.stage='RELEASE_INITIAL';self.count=0;self.sequence=None;self.passed=False
    def step(self,latest,now):
        if self.passed:return self.stage
        fresh=bool(latest is not None and 0<=now-latest['stamp']<=.08
                   and latest.get('tracking_valid',True) and not latest.get('stick_clicked'))
        if not fresh:self.count=0;return self.stage
        if latest['sequence']==self.sequence:return self.stage
        self.sequence=latest['sequence']
        if self.stage=='GRIP_STICK_UP':
            valid=bool(latest.get('deadman') and latest.get('grip',0)>.7 and latest.get('axis_y',latest.get('stick_y',0))>.3)
            required=3
        else:valid=ready_input(latest,now);required=10
        self.count=self.count+1 if valid else 0
        if self.count>=required:
            self.count=0
            if self.stage=='RELEASE_INITIAL':self.stage='GRIP_STICK_UP'
            elif self.stage=='GRIP_STICK_UP':self.stage='RELEASE_FINAL'
            else:self.stage='REHEARSAL_PASSED_NO_ROBOT_COMMANDS';self.passed=True
        return self.stage
