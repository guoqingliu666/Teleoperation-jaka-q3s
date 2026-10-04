"""H7逐轴限时姿态：复用同一角度LPF/DLS/Ruckig规划，不拥有SDK。"""
import numpy as np
from ..configuration import vector
from ..safety import SafetyFault
from .quest_rotation import RotationShadowPlanner
from .quest_j1 import ready_input
from .official_local import rpy_matrix,rotation_vector


class QuestTCPRotationJog(RotationShadowPlanner):
    def __init__(self,q,cfg,model,origin_tcp):
        if model.get('axis') not in ('RX','RY','RZ'):raise SafetyFault('H7 requires reviewed rotation axis')
        super().__init__(model,cfg)
        if np.max(np.abs(vector(q)-self.q0))>np.deg2rad(.005):raise SafetyFault('H7 local origin shifted')
        self.cfg=cfg;self.origin_tcp=vector(origin_tcp);self.origin_rotation=rpy_matrix(self.origin_tcp)
        self.phase='WAIT_RELEASE';self.finished=False;self.stop_started=None;self.last_intent=None
        self.release_events=[];self.pending_release=None;self.raw_rotation_deg=np.zeros(3)

    @property
    def release_stops(self):return self.releases
    def wait_ready(self,frame,now):return ready_input(frame,now)
    def quiet(self):return np.max(np.abs(self.state.v))<1e-8 and np.max(np.abs(self.state.a))<1e-7

    def step(self,frame,now,finish=False,software_stop=False):
        valid=frame is not None and 0<=now-frame['stamp']<=.08 and frame.get('tracking_valid')
        reason=None
        if not valid:reason='Quest input lost/stale' if frame is None or not 0<=now-frame['stamp']<=.08 else 'Quest tracking invalid'
        elif software_stop or frame.get('software_stop'):reason='right stick software stop'
        elif finish:reason='session duration stop'
        if reason and not self.locked:self.locked=True;self.stop_started=now;self.stop_reason=reason
        saved_reason=self.stop_reason
        if frame is not None:
            self.last_intent={k:frame.get(k) for k in ('sequence','stamp','source_time_s','grip','deadman','state','tracking_valid','input_mode','rotation_xyzw','rotation_delta_deg')}
        if valid and self.gripped and not frame.get('deadman') and frame.get('grip',1)<.7:
            self.pending_release={'trigger_s':now,'command_angle_start_deg':self.angle,
                                  'moving_at_release':not self.quiet(),'command_q_start_rad':self.state.q.tolist()}
        motion=super().step(frame,now)
        if saved_reason:self.stop_reason=saved_reason
        if self.brake_trajectory.duration>1.:raise SafetyFault('H7 predicted controlled stop exceeds 1 second')
        if self.pending_release is not None and self.quiet():
            self.pending_release.update(command_stopped_s=now,command_stop_time_s=now-self.pending_release['trigger_s'],
                command_stop_distance_deg=abs(self.angle-self.pending_release['command_angle_start_deg']),physical_stop_validated=False)
            self.release_events.append(self.pending_release);self.pending_release=None
        self.phase='LOCKED_STOP' if self.locked else ('GRIP_'+self.model['axis'] if self.gripped else 'RELEASE_BRAKE')
        self.finished=bool(self.locked and self.quiet())
        if self.locked and self.stop_started is not None and not self.finished and now-self.stop_started>1.:
            raise SafetyFault('H7 controlled stop exceeds 1 second; protective stop required')
        return motion

    def check(self,candidate,actual,velocity):
        q=vector(candidate);actual=vector(actual);dq=vector(velocity)
        projection=float(self.gain@(q-self.q0)/(self.gain@self.gain))
        if np.max(np.abs(q-(self.q0+self.gain*projection)))>1e-10 or abs(projection)>self.maximum+1e-8:
            raise SafetyFault('H7 command outside '+self.model['axis']+' local angular domain')
        lower=vector(self.cfg['limits']['lower_rad']);upper=vector(self.cfg['limits']['upper_rad'])
        if np.any(q<lower) or np.any(q>upper) or np.any(actual<lower) or np.any(actual>upper):raise SafetyFault('H7 raw/configured joint limit')
        extrema=self.brake_trajectory.position_extrema[0]
        for offset in (extrema.min,extrema.max):
            brake_q=q+self.gain*offset
            if np.any(brake_q<lower) or np.any(brake_q>upper):raise SafetyFault('H7 predicted brake outside joint limits')
        if np.max(np.abs(actual-self.q0))>np.deg2rad(.12):raise SafetyFault('H7 raw measured joint envelope')
        if np.max(np.abs(q-actual))>np.deg2rad(.05):raise SafetyFault('H7 raw tracking >0.05 degree')
        if np.max(np.abs(dq))>np.deg2rad(.5):raise SafetyFault('H7 raw speed >0.5 degree/s')

    def check_tcp(self,tcp):
        tcp=vector(tcp)
        if np.linalg.norm(tcp[:3]-self.origin_tcp[:3])>.1:raise SafetyFault('H7 raw Tool1 translation exceeds 0.1mm')
        self.raw_rotation_deg=np.rad2deg(rotation_vector(rpy_matrix(tcp)@self.origin_rotation.T))
        if abs(self.raw_rotation_deg[self.index])>self.maximum+.03 or np.linalg.norm(np.delete(self.raw_rotation_deg,self.index))>.03:
            raise SafetyFault('H7 raw Tool1 orientation envelope')
