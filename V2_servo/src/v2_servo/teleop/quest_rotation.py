"""姿态阶段只读输入/影子规划；不导入SDK、不拥有机器人发送接口。"""
import numpy as np
from ruckig import Ruckig,InputParameter,OutputParameter,Trajectory,ControlInterface,Result
from ..configuration import vector
from ..filters import CommandFilter
from ..trajectory import Motion
from ..safety import SafetyFault
from .quest import QuestInput
from .quest_j1 import QuestJ1Mapping
from .official_local import rotation_vector
from vendor.quest_vr_input import quaternion,quaternion_matrix


class QuestRotationMapping(QuestJ1Mapping):
    def __init__(self,basis):super().__init__(basis);self.reference_rotation=None
    def decode(self,payload,address,stamp):
        frame=super().decode(payload,address,stamp)
        q=quaternion(payload['right']['rotation_xyzw']);rot=np.array(quaternion_matrix(q))
        if not frame['tracking_valid'] or not frame.get('deadman'):
            self.reference_rotation=None;delta=np.zeros(3)
        else:
            if self.reference_rotation is None:self.reference_rotation=rot.copy()
            delta=np.rad2deg(np.linalg.det(self.basis)*self.basis@rotation_vector(rot@self.reference_rotation.T))
            if np.linalg.norm(delta)>30:raise ValueError('rotation rehearsal >30 degrees; release Grip')
        frame.update(input_mode='rotation_shadow',rotation_xyzw=list(q),rotation_delta_deg=delta.tolist())
        return frame


class QuestRotationReceiver(QuestInput):
    input_mode='rotation_shadow'
    def __init__(self,port,basis,audit=None):super().__init__(port,basis,mapper=QuestRotationMapping(basis),audit=audit)


class RotationShadowPlanner:
    """首个RX候选：角度误差twist→低通→局部DLS增益→标量Ruckig→关节目标。

    标量Ruckig坐标单位明确为degree，映射后的关节状态为rad；从不下发。
    """
    def __init__(self,model,cfg):
        self.model=model;self.q0=vector(model['q_origin_rad']);self.gain=vector(model['gain_rad_per_deg']);self.dt=cfg['period_s']
        self.maximum=float(model['maximum_angle_deg']);self.index=('RX','RY','RZ').index(model['axis'])
        scale=float(np.max(np.abs(self.gain)))
        if not cfg['filters']['enabled'] or not 0<self.maximum<=.02 or scale<1e-10:raise SafetyFault('invalid rotation shadow model/filter')
        self.speed=min(.02,float(np.deg2rad(.05)/scale));self.accel=min(.2,float(np.deg2rad(.2)/scale));self.jerk=min(1.,float(np.deg2rad(1)/scale))
        self.otg=Ruckig(1,self.dt);self.input=InputParameter(1);self.output=OutputParameter(1)
        self.input.control_interface=ControlInterface.Velocity
        self.input.current_position=[0.];self.input.current_velocity=[0.];self.input.current_acceleration=[0.]
        self.input.target_velocity=[0.];self.input.target_acceleration=[0.]
        self.input.max_velocity=[self.speed];self.input.max_acceleration=[self.accel];self.input.max_jerk=[self.jerk]
        self.brake=Ruckig(1,self.dt);self.brake_input=InputParameter(1);self.brake_trajectory=Trajectory(1)
        self.brake_input.control_interface=ControlInterface.Velocity
        self.brake_input.target_velocity=[0.];self.brake_input.target_acceleration=[0.]
        self.brake_input.max_velocity=[self.speed];self.brake_input.max_acceleration=[self.accel];self.brake_input.max_jerk=[self.jerk]
        self.filter=CommandFilter(cfg['filters'],self.dt,np.zeros(6));self.state=Motion.rest(self.q0)
        self.angle=0.;self.goal=0.;self.reference=0.;self.released=False;self.gripped=False;self.locked=False
        self.releases=0;self.maximum_joint_error_deg=0.;self.stop_reason=None
    def step(self,frame,now):
        quiet=abs(self.input.current_velocity[0])<1e-7 and abs(self.input.current_acceleration[0])<1e-6
        valid=frame is not None and 0<=now-frame['stamp']<=.08 and frame.get('tracking_valid')
        if not valid or (frame and frame.get('software_stop')):
            self.locked=True;self.stop_reason='tracking/stale/software stop'
        active=False;axes=np.zeros(6)
        if valid:
            if self.gripped and not frame.get('deadman'):
                self.gripped=False;self.released=False;self.releases+=1
            if frame.get('grip',1)<.3:self.released=quiet
            active=bool(not self.locked and self.released and frame.get('deadman'))
            if active and not self.gripped:self.gripped=True;self.reference=self.angle
            if active:
                hand=float(frame['rotation_delta_deg'][self.index]);hand=np.sign(hand)*max(abs(hand)-1.,0.)
                self.goal=float(np.clip(self.reference+hand*.004,-.75*self.maximum,.75*self.maximum))
                axes[3]=np.clip((self.goal-self.angle)*2/self.speed,-1.,1.)
        filtered=self.filter.update(axes)
        self.input.target_velocity=[float(filtered[3]*self.speed) if active else 0.]
        result=self.otg.update(self.input,self.output)
        if result not in (Result.Working,Result.Finished):raise SafetyFault('rotation shadow Ruckig failure')
        def contained():
            b=self.brake_input;b.current_position=[0.];b.current_velocity=self.output.new_velocity;b.current_acceleration=self.output.new_acceleration
            code=self.brake.calculate(b,self.brake_trajectory)
            if code not in (Result.Working,Result.Finished):raise SafetyFault('rotation shadow brake failure')
            bounds=self.brake_trajectory.position_extrema[0];angle=self.output.new_position[0]
            return angle+bounds.min>=-self.maximum+1e-6 and angle+bounds.max<=self.maximum-1e-6
        if not contained():
            self.input.target_velocity=[0.];result=self.otg.update(self.input,self.output)
            if result not in (Result.Working,Result.Finished):raise SafetyFault('rotation shadow boundary Ruckig failure')
            if not contained():raise SafetyFault('rotation shadow cannot stay inside angular domain')
        self.angle=float(self.output.new_position[0]);v=self.gain*self.output.new_velocity[0];a=self.gain*self.output.new_acceleration[0]
        self.state=Motion(self.q0+self.gain*self.angle,v,a,(a-self.state.a)/self.dt)
        if np.max(np.abs(self.state.q-self.q0))>np.deg2rad(.1):raise SafetyFault('rotation shadow joint domain')
        if np.max(np.abs(v))>np.deg2rad(.05)+1e-9 or np.max(np.abs(a))>np.deg2rad(.2)+1e-9 or np.max(np.abs(self.state.j))>np.deg2rad(1)+1e-8:
            raise SafetyFault('rotation shadow q/v/a/j limits')
        self.output.pass_to_input(self.input);return self.state
