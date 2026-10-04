"""H2唯一轨迹：J1 +0.1°→静止→返回本次实测起点。不能配置成大范围运动。"""
import numpy as np
from ruckig import Ruckig,InputParameter,OutputParameter,Result

from ..configuration import vector
from ..filters import CommandFilter
from ..safety import SafetyFault
from ..trajectory import Motion


class J1Roundtrip:
    def __init__(self,q,cfg):
        self.q0=vector(q);self.dt=cfg['period_s'];self.limits=cfg['limits']
        self.delta=float(np.deg2rad(.1));self.speed=float(np.deg2rad(.1))
        self.acceleration=float(np.deg2rad(.2));self.jerk=float(np.deg2rad(1.))
        if self.q0[0]+self.delta>self.limits['upper_rad'][0]:
            raise SafetyFault('H2 J1 target exceeds configured joint range')
        self.otg=Ruckig(6,self.dt);self.input=InputParameter(6);self.output=OutputParameter(6)
        self.input.current_position=self.q0.tolist();self.input.current_velocity=[0.]*6
        self.input.current_acceleration=[0.]*6;self.input.target_velocity=[0.]*6
        self.input.target_acceleration=[0.]*6
        self.input.max_velocity=[self.speed]*6;self.input.max_acceleration=[self.acceleration]*6
        self.input.max_jerk=[self.jerk]*6
        self.filter=CommandFilter(cfg['filters'],self.dt,self.q0)
        self.state=Motion.rest(self.q0);self.phase='outbound';self.settled=0.;self.finished=False

    def step(self):
        goal=self.q0.copy()
        if self.phase in ('outbound','outbound_hold'):goal[0]+=self.delta
        self.input.target_position=self.filter.update(goal).tolist()
        result=self.otg.update(self.input,self.output)
        if result not in (Result.Working,Result.Finished):raise SafetyFault('H2 Ruckig failed: '+str(result))
        q=np.array(self.output.new_position);v=np.array(self.output.new_velocity);a=np.array(self.output.new_acceleration)
        self.state=Motion(q,v,a,(a-self.state.a)/self.dt)
        self.output.pass_to_input(self.input)
        near=np.max(np.abs(q-goal))<1e-7 and np.max(np.abs(v))<1e-6 and np.max(np.abs(a))<1e-5
        if near:
            self.settled+=self.dt
            if self.phase=='outbound':self.phase='outbound_hold';self.settled=0.
            elif self.phase=='outbound_hold' and self.settled>=.2:self.phase='return';self.settled=0.
            elif self.phase=='return':self.phase='return_hold';self.settled=0.
            elif self.phase=='return_hold' and self.settled>=.3:self.finished=True
        else:self.settled=0.
        return self.state

    def check(self,candidate,actual,velocity):
        q=vector(candidate);actual=vector(actual);dq=vector(velocity)
        if np.any(q<self.limits['lower_rad']) or np.any(q>self.limits['upper_rad']):
            raise SafetyFault('H2 configured joint boundary')
        if np.max(np.abs(q[1:]-self.q0[1:]))>1e-10 or not -1e-8<=q[0]-self.q0[0]<=self.delta+1e-8:
            raise SafetyFault('H2 command left the fixed J1 0.1 degree envelope')
        if np.max(np.abs(actual[1:]-self.q0[1:]))>np.deg2rad(.05):
            raise SafetyFault('H2 unexpected motion on other joints')
        if not -np.deg2rad(.02)<=actual[0]-self.q0[0]<=self.delta+np.deg2rad(.02):
            raise SafetyFault('H2 measured J1 outside supervised micro-motion envelope')
        if np.max(np.abs(q-actual))>np.deg2rad(.05):raise SafetyFault('H2 raw tracking deviation >0.05 degree')
        if np.max(np.abs(dq))>np.deg2rad(.5):raise SafetyFault('H2 raw speed >0.5 degree/s')
        if (np.max(np.abs(self.state.v))>self.speed+1e-9 or
            np.max(np.abs(self.state.a))>self.acceleration+1e-9 or
            np.max(np.abs(self.state.j))>self.jerk+1e-7):
            raise SafetyFault('H2 command derivative constraint')
