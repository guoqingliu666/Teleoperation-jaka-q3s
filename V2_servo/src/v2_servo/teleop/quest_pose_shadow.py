"""联合6DoF候选：复用XYZ制动/边界、指令低通和关节Ruckig；不拥有SDK。"""
import numpy as np
from ruckig import Result
from ..configuration import vector
from ..trajectory import Motion
from ..safety import SafetyFault
from .live_rotation_input import QuestLiveRotationMapping
from .quest import QuestInput
from .quest_tcp_xyz import QuestTCPXYZJog


class QuestPoseMapping(QuestLiveRotationMapping):
    def __init__(self,basis):super().__init__(basis);self.reference_position=None
    def decode(self,payload,address,stamp):
        frame=super().decode(payload,address,stamp);raw=payload['right']['position_m']
        p=vector([raw[k] for k in ('x','y','z')] if isinstance(raw,dict) else raw,3)
        if not frame['tracking_valid'] or not frame.get('deadman'):self.reference_position=None;d=np.zeros(3)
        else:
            if self.reference_position is None:self.reference_position=p.copy()
            d=1000*self.basis@(p-self.reference_position)
        frame.update(input_mode='pose6dof_shadow',position_m=p.tolist(),base_displacement_mm=d.tolist())
        return frame


class QuestPoseReceiver(QuestInput):
    input_mode='pose6dof_shadow'
    def __init__(self,port,basis,audit=None):super().__init__(port,basis,mapper=QuestPoseMapping(basis),audit=audit)


class QuestPoseShadowPlanner(QuestTCPXYZJog):
    def __init__(self,q,cfg,model,profile):
        jac=np.array(model['jacobian_m_rad']);w=np.diag([1,1,1,.2,.2,.2]);weighted=w@jac
        if np.linalg.svd(weighted,compute_uv=False)[-1]<.005:raise SafetyFault('pose shadow local singularity')
        self.dls=np.linalg.solve(weighted.T@weighted+np.eye(6)*.0001**2,weighted.T@w)
        synthetic=dict(model,gain_rad_per_mm_xyz=(self.dls[:,:3]/1000).tolist(),maximum_radius_mm=min(.5,float(model.get('maximum_radius_mm',.5))))
        p=dict(profile,command_orientation_deg=.02,raw_orientation_deg=.05)
        super().__init__(q,cfg,synthetic,model['tcp_origin_mm_rad'],p)
        self.maximum_rotation_deg=.02;self.angular_speed_deg_s=.02;self.rotation_reference=None
        self.goal_rotation_deg=np.zeros(3);self.stop_events=[]

    @property
    def rotation_target_deg(self):return np.rad2deg(self.jac[3:]@(self.state.q-self.q0))

    def step(self,frame,now,finish=False,software_stop=False):
        valid=frame is not None and 0<=now-frame['stamp']<=.08 and frame.get('tracking_valid')
        reason=None
        if not valid:reason='pose shadow input lost/stale/tracking invalid'
        elif software_stop or frame.get('software_stop'):reason='pose shadow software stop'
        elif finish:reason='pose shadow duration stop'
        if reason and not self.locked:self.locked=True;self.stop_started=now;self.stop_reason=reason
        moving=False;axes=np.zeros(6)
        if valid:
            self.last_intent=frame
            if self.was_gripped and not frame.get('deadman'):
                self.was_gripped=False;self.released=False;self.robot_reference=None;self.rotation_reference=None
                if frame.get('grip',1)<.7:
                    self.release_stops+=1;self.pending_release={'trigger_s':now,'moving_at_release':not self.quiet(),
                        'command_xyz_start_mm':self.xyz_target_mm.tolist(),'command_rotation_start_deg':self.rotation_target_deg.tolist()}
            if frame.get('grip',1)<.3:self.released=self.quiet()
            moving=bool(not self.locked and self.released and frame.get('deadman'))
            if moving and not self.was_gripped:
                if not self.quiet():moving=False;self.released=False
                else:
                    self.was_gripped=True;self.robot_reference=self.xyz_target_mm.copy();self.rotation_reference=self.rotation_target_deg.copy()
            if moving:
                hand=vector(frame['base_displacement_mm'],3)
                hand=np.sign(hand)*np.maximum(np.abs(hand)-self.p['hand_deadzone_m']*1000,0.)
                goal=self.robot_reference+hand*self.p['translation_gain'];cap=self.radius*.75;norm=np.linalg.norm(goal)
                self.goal_mm=goal if norm<=cap else goal*cap/norm
                angle=vector(frame['rotation_delta_deg'],3);angle=np.sign(angle)*np.maximum(np.abs(angle)-1.,0.)
                goal_r=self.rotation_reference+angle*getattr(self,'rotation_gain',.004);cap_r=self.maximum_rotation_deg*getattr(self,'rotation_goal_fraction',.6);norm=np.linalg.norm(goal_r)
                self.goal_rotation_deg=goal_r if norm<=cap_r else goal_r*cap_r/norm
                v=(self.goal_mm-self.xyz_target_mm)*2.;vr=(self.goal_rotation_deg-self.rotation_target_deg)*2.
                axes[:3]=v/max(self.speed,np.linalg.norm(v));axes[3:]=vr/max(self.angular_speed_deg_s,np.linalg.norm(vr))
        filtered=self.filter.update(axes)
        twist=np.r_[filtered[:3]*self.speed/1000,filtered[3:]*np.deg2rad(self.angular_speed_deg_s)]
        target=self.dls@twist if moving else np.zeros(6)
        target/=max(1.,float(np.max(np.abs(target)/self.vmax)))
        # H9按原始反馈提前限制目标速度；H8/影子没有此hook，既有行为不变。
        if hasattr(self,'shape_joint_velocity'):target=self.shape_joint_velocity(target)
        self.input.target_velocity=target.tolist();result=self.otg.update(self.input,self.output)
        if result not in (Result.Working,Result.Finished):raise SafetyFault('pose shadow Ruckig failed')
        q=np.array(self.output.new_position);v=np.array(self.output.new_velocity);a=np.array(self.output.new_acceleration)
        if not self.brake_bounds(q,v,a):
            self.input.target_velocity=[0.]*6;result=self.otg.update(self.input,self.output)
            if result not in (Result.Working,Result.Finished):raise SafetyFault('pose shadow boundary brake failed')
            q=np.array(self.output.new_position);v=np.array(self.output.new_velocity);a=np.array(self.output.new_acceleration)
            if not self.brake_bounds(q,v,a):raise SafetyFault('pose shadow predicted joint/TCP stop outside domain')
            self.boundary_brakes+=1
        self.state=Motion(q,v,a,(a-self.state.a)/self.dt);self.output.pass_to_input(self.input)
        self.phase='LOCKED_STOP' if self.locked else ('GRIP_6DOF_SHADOW' if moving else 'RELEASE_BRAKE')
        if self.pending_release is not None and self.quiet():
            self.release_events.append({**self.pending_release,'command_stop_s':now,'command_stop_time_ms':(now-self.pending_release['trigger_s'])*1000,
                'command_stop_distance_mm':float(np.linalg.norm(self.xyz_target_mm-self.pending_release['command_xyz_start_mm'])),
                'command_stop_angle_deg':float(np.linalg.norm(self.rotation_target_deg-self.pending_release['command_rotation_start_deg'])),
                'feedback_stop_validated':False});self.pending_release=None
        self.finished=bool(self.locked and self.quiet())
        if self.locked and not self.finished and now-self.stop_started>1.:raise SafetyFault('pose shadow controlled stop budget')
        return self.state
