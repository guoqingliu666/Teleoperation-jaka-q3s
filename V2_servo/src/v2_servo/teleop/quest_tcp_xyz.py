"""H6：相对位移→位置误差twist→低通→官方局部DLS→关节Ruckig→servo_j。

制动预测仅调用同版本Ruckig，不调用SDK或每周期IK；状态保护不使用滤波值。
"""
import numpy as np
from ruckig import Ruckig,InputParameter,OutputParameter,Trajectory,ControlInterface,Synchronization,Result
from ..configuration import vector
from ..filters import CommandFilter
from ..safety import SafetyFault
from ..trajectory import Motion
from .quest_j1 import ready_input
from .official_local import rpy_matrix,rotation_vector


def validate_xyz_profile(profile):
    caps={'hand_deadzone_m':(.001,.01),'translation_gain':(.001,.01),'position_gain_s_inv':(.1,2.),
          'cartesian_speed_mm_s':(.001,.15),'joint_envelope_deg':(.1,.1),
          'joint_velocity_deg_s':(.001,.05),'joint_acceleration_deg_s2':(.001,.2),
          'joint_jerk_deg_s3':(.001,1.),'maximum_radius_mm':(.05,.5),'goal_radius_fraction':(.1,.75),
          'command_orientation_deg':(.001,.05),'raw_orientation_deg':(.001,.1),
          'controlled_stop_budget_s':(.5,1.),'brake_model_margin_mm':(.005,.02)}
    for key,(low,high) in caps.items():
        v=profile.get(key)
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not np.isfinite(v) or not low<=v<=high:
            raise SafetyFault('H6 unreviewed profile parameter '+key)
    if profile.get('brake_samples')!=17:raise SafetyFault('H6 brake prediction uses reviewed 17 samples')
    basis=np.asarray(profile.get('basis'),dtype=float)
    if basis.shape!=(3,3) or not np.isfinite(basis).all() or not np.allclose(basis.T@basis,np.eye(3)) or not np.isclose(abs(np.linalg.det(basis)),1):
        raise SafetyFault('H6 invalid command basis')
    return profile


class QuestTCPXYZJog:
    def __init__(self,q,cfg,model,origin_tcp,profile):
        validate_xyz_profile(profile)
        self.q0=vector(q);self.state=Motion.rest(q);self.cfg=cfg;self.p=profile;self.dt=cfg['period_s']
        self.origin_tcp=vector(origin_tcp);self.gain=np.array(model['gain_rad_per_mm_xyz']);self.jac=np.array(model['jacobian_m_rad'])
        self.radius=min(float(model['maximum_radius_mm']),profile['maximum_radius_mm'])
        self.speed=float(profile['cartesian_speed_mm_s']);self.maximum_z_mm=self.radius # 只供共用显示心跳读取。
        if self.gain.shape!=(6,3) or not np.isfinite(self.gain).all() or self.jac.shape!=(6,6) or not np.isfinite(self.jac).all() or not .05<=self.radius<=.5:
            raise SafetyFault('H6 invalid local XYZ model')
        if (not cfg['filters']['enabled'] or profile['joint_envelope_deg']!=.1 or
            not 0<self.speed<=.15 or not 0<profile['joint_velocity_deg_s']<=.05 or
            not 0<profile['joint_acceleration_deg_s2']<=.2 or not 0<profile['joint_jerk_deg_s3']<=1):
            raise SafetyFault('H6 candidate limit/filter profile is not reviewed')
        self.lower=np.maximum(vector(cfg['limits']['lower_rad']),self.q0-np.deg2rad(.1))
        self.upper=np.minimum(vector(cfg['limits']['upper_rad']),self.q0+np.deg2rad(.1))
        self.vmax=np.full(6,np.deg2rad(profile['joint_velocity_deg_s']))
        self.amax=np.full(6,np.deg2rad(profile['joint_acceleration_deg_s2']))
        self.jmax=np.full(6,np.deg2rad(profile['joint_jerk_deg_s3']))
        self.otg=Ruckig(6,self.dt);self.output=OutputParameter(6);self.input=InputParameter(6)
        self.brake_otg=Ruckig(6,self.dt);self.brake_input=InputParameter(6);self.brake_trajectory=Trajectory(6)
        for inp in (self.input,self.brake_input):
            inp.control_interface=ControlInterface.Velocity;inp.synchronization=Synchronization.No
            inp.current_position=self.q0.tolist();inp.current_velocity=[0.]*6;inp.current_acceleration=[0.]*6
            inp.target_velocity=[0.]*6;inp.target_acceleration=[0.]*6
            inp.max_velocity=self.vmax.tolist();inp.max_acceleration=self.amax.tolist();inp.max_jerk=self.jmax.tolist()
        self.filter=CommandFilter(cfg['filters'],self.dt,np.zeros(6))
        self.linear_lipschitz=float(1000*np.linalg.norm(self.jac[:3],2)*np.linalg.norm(self.vmax))
        self.angular_lipschitz=float(np.linalg.norm(self.jac[3:],2)*np.linalg.norm(self.vmax))
        self.phase='WAIT_RELEASE';self.released=False;self.locked=False;self.finished=False
        self.was_gripped=False;self.release_stops=0;self.stop_reason=None;self.stop_started=None
        self.last_intent=None;self.robot_reference=None;self.goal_mm=np.zeros(3);self.boundary_brakes=0;self.last_brake_bounds=None
        self.release_events=[];self.pending_release=None

    @property
    def xyz_target_mm(self):return 1000*self.jac[:3]@(self.state.q-self.q0)
    def wait_ready(self,latest,now):return ready_input(latest,now)
    def quiet(self):return np.max(np.abs(self.state.v))<1e-6 and np.max(np.abs(self.state.a))<1e-5

    def brake_bounds(self,q,v,a):
        # 在相对零位预测位移；Ruckig 0.12.2零时长/静止轴extrema含零，不能当绝对关节角。
        b=self.brake_input;b.current_position=[0.]*6;b.current_velocity=v.tolist();b.current_acceleration=a.tolist()
        code=self.brake_otg.calculate(b,self.brake_trajectory)
        if code not in (Result.Working,Result.Finished):raise SafetyFault('H6 Ruckig brake prediction failed '+str(code))
        duration=float(self.brake_trajectory.duration);count=int(self.p['brake_samples'])
        if count<17 or duration>self.p['controlled_stop_budget_s']:raise SafetyFault('H6 predicted stop budget exceeded')
        extrema=self.brake_trajectory.position_extrema
        joint_ok=all(q[i]+extrema[i].min>=self.lower[i]-1e-9 and q[i]+extrema[i].max<=self.upper[i]+1e-9 for i in range(6))
        samples=np.array([self.brake_trajectory.at_time(t)[0] for t in np.linspace(0,duration,count)])+q-self.q0
        cart=1000*samples@self.jac[:3].T;rot=samples@self.jac[3:].T
        half_gap=duration/(2*(count-1))
        # 固定Jacobian下的连续时间上界：采样最大值+速度Lipschitz×半采样间隔。
        radius_bound=float(np.max(np.linalg.norm(cart,axis=1))+self.linear_lipschitz*half_gap+self.p['brake_model_margin_mm'])
        angle_bound=float(np.max(np.linalg.norm(rot,axis=1))+self.angular_lipschitz*half_gap)
        safe=bool(joint_ok and radius_bound<=self.radius and angle_bound<=np.deg2rad(self.p['command_orientation_deg']))
        self.last_brake_bounds={'duration_s':duration,'linear_radius_upper_mm':radius_bound,
                                'angular_upper_deg':float(np.rad2deg(angle_bound)),'joint_extrema_inside':joint_ok,'safe':safe}
        return safe

    def step(self,latest,now,finish=False,software_stop=False):
        valid=latest is not None and 0<=now-latest['stamp']<=.08
        reason=None
        if not valid:reason='Quest input lost/stale'
        elif not latest.get('tracking_valid'):reason='Quest tracking invalid'
        elif software_stop or latest.get('software_stop'):reason='right stick software stop'
        if finish:reason='session duration stop'
        if reason:
            if not self.locked:self.stop_started=now;self.stop_reason=reason
            self.locked=True;self.phase='LOCKED_STOP'
        moving=False;twist=np.zeros(6)
        if valid:
            self.last_intent={k:latest.get(k) for k in ('sequence','stamp','source_time_s','grip','deadman','state',
                'input_mode','position_m','reference_position_m','hand_displacement_m','base_displacement_mm','axes_xyz')}
            if self.was_gripped and not latest.get('deadman'):
                if latest.get('tracking_valid') and latest.get('grip',1)<.7:
                    self.release_stops+=1
                    self.pending_release={'trigger_s':now,'command_q_start_rad':self.state.q.tolist(),
                        'command_xyz_start_mm':self.xyz_target_mm.tolist(),'command_v_start_rad_s':self.state.v.tolist(),
                        'moving_at_release':not self.quiet()}
                self.was_gripped=False;self.robot_reference=None;self.released=False
            if latest.get('grip',1)<.3:
                self.was_gripped=False;self.robot_reference=None
                self.released=self.quiet()
            moving=bool(not self.locked and self.released and latest.get('deadman'))
            if moving and not self.was_gripped:
                if not self.quiet():moving=False;self.released=False
                else:self.robot_reference=self.xyz_target_mm.copy();self.was_gripped=True
            if moving:
                hand=vector(latest['base_displacement_mm'],3)
                deadzone=self.p['hand_deadzone_m']*1000
                hand=np.sign(hand)*np.maximum(np.abs(hand)-deadzone,0.)
                goal=self.robot_reference+hand*self.p['translation_gain']
                maximum=self.radius*self.p['goal_radius_fraction'];norm=np.linalg.norm(goal)
                self.goal_mm=goal if norm<=maximum else goal*(maximum/norm)
                error=self.goal_mm-self.xyz_target_mm
                velocity=error*self.p['position_gain_s_inv'];norm=np.linalg.norm(velocity)
                if norm>self.speed:velocity*=self.speed/norm
                twist[:3]=velocity/self.speed;self.phase='GRIP_XYZ_POSITION'
            elif not self.locked:self.phase='RELEASE_BRAKE' if not self.was_gripped else 'GRIP_ZERO'
        filtered=self.filter.update(twist)
        target=self.gain@(filtered[:3]*self.speed) if moving else np.zeros(6)
        ratio=max(1.,float(np.max(np.abs(target)/self.vmax)));target/=ratio
        self.input.target_velocity=target.tolist()
        result=self.otg.update(self.input,self.output)
        if result not in (Result.Working,Result.Finished):raise SafetyFault('H6 Ruckig failed '+str(result))
        q=np.array(self.output.new_position);v=np.array(self.output.new_velocity);a=np.array(self.output.new_acceleration)
        if not self.brake_bounds(q,v,a):
            # 重算同一原始q/v/a的零速度目标；不能裁剪已经生成的下发位置。
            self.input.target_velocity=[0.]*6;result=self.otg.update(self.input,self.output)
            if result not in (Result.Working,Result.Finished):raise SafetyFault('H6 Ruckig boundary stop failed')
            q=np.array(self.output.new_position);v=np.array(self.output.new_velocity);a=np.array(self.output.new_acceleration)
            if not self.brake_bounds(q,v,a):raise SafetyFault('H6 controlled stop left predicted local domain')
            self.boundary_brakes+=1
            if not self.locked:self.phase='WORKSPACE_BRAKE'
        self.state=Motion(q,v,a,(a-self.state.a)/self.dt);self.output.pass_to_input(self.input)
        if self.pending_release is not None and self.quiet():
            self.release_events.append({**self.pending_release,'command_stop_s':now,
                'command_stop_time_ms':(now-self.pending_release['trigger_s'])*1000,
                'command_stop_distance_mm':float(np.linalg.norm(self.xyz_target_mm-self.pending_release['command_xyz_start_mm'])),
                'feedback_stop_validated':False})
            self.pending_release=None
        self.finished=bool(self.locked and self.quiet())
        if self.locked and not self.finished and now-self.stop_started>self.p['controlled_stop_budget_s']:
            raise SafetyFault('H6 controlled stop exceeded host budget')
        return self.state

    def check(self,candidate,actual,velocity):
        q=vector(candidate);actual=vector(actual);dq=vector(velocity)
        if np.any(q<self.lower-1e-9) or np.any(q>self.upper+1e-9):raise SafetyFault('H6 joint target trust domain')
        if np.linalg.norm(1000*self.jac[:3]@(q-self.q0))>self.radius+1e-6:raise SafetyFault('H6 command TCP sphere')
        if np.linalg.norm(self.jac[3:]@(q-self.q0))>np.deg2rad(self.p['command_orientation_deg']):raise SafetyFault('H6 command orientation')
        if np.max(np.abs(actual-self.q0))>np.deg2rad(.12):raise SafetyFault('H6 raw measured joint envelope')
        if np.max(np.abs(q-actual))>np.deg2rad(.05):raise SafetyFault('H6 raw tracking >0.05 degree')
        if np.max(np.abs(dq))>np.deg2rad(.5):raise SafetyFault('H6 raw speed >0.5 degree/s')
        if (np.any(np.abs(self.state.v)>self.vmax+1e-9) or np.any(np.abs(self.state.a)>self.amax+1e-9)
            or np.any(np.abs(self.state.j)>self.jmax+1e-7)):raise SafetyFault('H6 command joint v/a/j envelope')
    def check_tcp(self,tcp):
        if np.linalg.norm(vector(tcp)[:3]-self.origin_tcp[:3])>self.radius+.15:raise SafetyFault('H6 raw TCP local sphere')
        angle=np.linalg.norm(rotation_vector(rpy_matrix(tcp)@rpy_matrix(self.origin_tcp).T))
        if angle>np.deg2rad(self.p['raw_orientation_deg']):raise SafetyFault('H6 raw Tool1 orientation envelope')
