"""H9有限扩展：当前官方FK扫掠、组合DLS、低通/Ruckig、停止包络和原始保护。

沿用H8输入和step，不把数学采样当现场碰撞或物理停止验收。
"""
import itertools
import numpy as np
from ruckig import Ruckig,InputParameter,OutputParameter,Trajectory,ControlInterface,Synchronization
from ..configuration import vector
from ..trajectory import Motion
from ..filters import CommandFilter
from ..safety import SafetyFault
from ..jaka import checked
from .local_xyz import local_xyz_model
from .official_local import rpy_matrix,rotation_vector
from .quest_pose_shadow import QuestPoseShadowPlanner


EXPANSION_CAPS=dict(maximum_radius_mm=5.,maximum_rotation_deg=.2,joint_envelope_deg=2.,
    joint_velocity_deg_s=.5,joint_acceleration_deg_s2=2.,joint_jerk_deg_s3=8.,
    cartesian_speed_mm_s=1.,angular_speed_deg_s=.1,translation_gain=.1,rotation_gain=.04,
    maximum_FK_sample_translation_residual_mm=.5,maximum_FK_sample_rotation_residual_deg=.1,
    minimum_site_clearance_mm=100,controlled_stop_budget_s=1.,maximum_motion_window_s=30)


def validate_expansion(profile):
    # 只有已纳入本轮离线验证的档位；不能在线扩大范围或速度。
    if profile.get('scope')!='H9_supervised_finite_expansion_not_full_workspace_release':raise SafetyFault('H9 invalid profile scope')
    for k,v in EXPANSION_CAPS.items():
        if isinstance(profile.get(k),bool) or profile.get(k)!=v:raise SafetyFault('H9 unreviewed expansion parameter '+k)
    return profile


def expansion_model(rc,q,profile,limits):
    """仅静止阶段调用官方FK；逐周期无IK/FK。角点采样是诊断，不是连续域证明。"""
    validate_expansion(profile);q=vector(q);base=local_xyz_model(rc,q)
    jac=np.asarray(base['jacobian_m_rad']);w=np.diag([1.,1.,1.,.2,.2,.2]);a=w@jac
    dls=np.linalg.solve(a.T@a+np.eye(6)*1e-8,a.T@w)
    if np.linalg.norm(w@(jac@dls-np.eye(6)),ord=2)>1e-4:raise SafetyFault('H9 combined DLS residual')
    pose=np.array(base['tcp_origin_mm_rad']);rot=rpy_matrix(pose);attempts=[];accepted=None
    for envelope in (2.,1.,.5):
        margin=np.deg2rad(envelope)
        if np.any(q-margin<vector(limits['lower_rad'])) or np.any(q+margin>vector(limits['upper_rad'])):
            attempts.append({'joint_envelope_deg':envelope,'failure':'joint configured limit margin'});continue
        rotation_use=np.linalg.norm(dls[:,3:],axis=1)*np.deg2rad(profile['maximum_rotation_deg'])
        translation_gain=np.linalg.norm(dls[:,:3]/1000,axis=1)
        available=np.deg2rad(envelope*.7)-rotation_use
        radius=min(profile['maximum_radius_mm'],float(np.min(available/np.maximum(translation_gain,1e-12))))
        if radius<1.:
            attempts.append({'joint_envelope_deg':envelope,'failure':'combined joint travel budget below 1mm'});continue
        # 扫的是实际可达的组合球域，不能把会产生数十毫米/数度的任意关节立方角点
        # 当成这个3–5mm/0.2度域的误差。完整Jacobian满秩，组合目标映射唯一。
        dirs=[np.array(c,dtype=float) for c in itertools.product((-1,0,1),repeat=3)]
        dirs=[d/max(1.,np.linalg.norm(d)) for d in dirs]
        rows=[]
        for linear_direction,angular_direction in itertools.product(dirs,repeat=2):
            twist=np.r_[linear_direction*radius/1000,angular_direction*np.deg2rad(profile['maximum_rotation_deg'])]
            candidate=q+dls@twist
            if np.max(np.abs(candidate-q))>margin:raise SafetyFault('H9 combined sweep exceeds joint trust domain')
            p=vector(checked(rc.kine_forward(candidate.tolist()),'H9 official static FK sweep'))
            delta=candidate-q
            rows.append({'q_rad':candidate.tolist(),'pose_mm_rad':p.tolist(),
                'translation_residual_mm':float(np.linalg.norm(p[:3]-pose[:3]-1000*jac[:3]@delta)),
                'orientation_residual_deg':float(np.rad2deg(np.linalg.norm(rotation_vector(rpy_matrix(p)@rot.T)-jac[3:]@delta)))})
        linear=max(r['translation_residual_mm'] for r in rows);angular=max(r['orientation_residual_deg'] for r in rows)
        attempts.append({'joint_envelope_deg':envelope,'max_sample_translation_residual_mm':linear,
                         'max_sample_rotation_residual_deg':angular,'samples':rows})
        if linear>profile['maximum_FK_sample_translation_residual_mm'] or angular>profile['maximum_FK_sample_rotation_residual_deg']:continue
        accepted=(envelope,radius,linear,angular);break
    if accepted is None:
        fault=SafetyFault('H9 current official FK cannot support >=1mm expanded domain; no motion')
        fault.math_audit={'attempts':attempts,'jacobian_m_rad':jac.tolist(),'dls_rad_per_m_rad':dls.tolist()}
        raise fault
    envelope,radius,linear,angular=accepted
    return {'scope':'official_H9_expanded_FK_sampling_only_not_collision_clearance',
        'q_origin_rad':q.tolist(),'tcp_origin_mm_rad':pose.tolist(),'jacobian_m_rad':jac.tolist(),
        'dls_rad_per_m_rad':dls.tolist(),'maximum_radius_mm':radius,'joint_envelope_deg':envelope,
        'model_translation_margin_mm':max(.05,linear*1.25),'model_rotation_margin_deg':max(.005,angular*1.25),
        'attempts':attempts,'profile':profile,'source_local_derivative':{k:base[k] for k in ('weighted_singular_values','derivative_relative_error')},
        'continuous_nonlinear_error_bound_validated':False,'collision_clearance_validated':False,
        'physical_stop_validated':False,'robot_full_workspace_approved':False}


class QuestPoseExpansionPlanner(QuestPoseShadowPlanner):
    """复用H8 step/制动预测，仅由独立H9初始化指定经过静态核查的参数。"""
    model_scope='official_H9_expanded_FK_sampling_only_not_collision_clearance'
    profile_validator=staticmethod(validate_expansion)
    radius_cap=5.
    allowed_envelopes=(.5,1.,2.)
    def __init__(self,q,cfg,model,origin_tcp,profile):
        self.profile_validator(profile)
        if model.get('scope')!=self.model_scope or model.get('profile')!=profile:
            raise SafetyFault('H9 requires current expansion model/profile')
        q=vector(q)
        if np.max(np.abs(q-vector(model['q_origin_rad'])))>np.deg2rad(.005):raise SafetyFault('H9 origin changed')
        self.q0=q;self.state=Motion.rest(q);self.cfg=cfg;self.dt=cfg['period_s'];self.origin_tcp=vector(origin_tcp)
        self.jac=np.asarray(model['jacobian_m_rad']);self.dls=np.asarray(model['dls_rad_per_m_rad'])
        if not cfg['filters']['enabled'] or self.jac.shape!=(6,6) or not np.isfinite(self.dls).all():raise SafetyFault('H9 filters/model invalid')
        self.radius=float(model['maximum_radius_mm']);self.envelope=float(model['joint_envelope_deg'])
        if not 1<=self.radius<=self.radius_cap or self.envelope not in self.allowed_envelopes:raise SafetyFault('H9 model domain invalid')
        self.speed=profile['cartesian_speed_mm_s'];self.maximum_z_mm=self.radius
        self.maximum_rotation_deg=profile['maximum_rotation_deg'];self.angular_speed_deg_s=profile['angular_speed_deg_s']
        self.rotation_gain=profile['rotation_gain'];self.rotation_goal_fraction=.35
        self.angular_margin=float(model['model_rotation_margin_deg'])
        self.p={'hand_deadzone_m':.003,'translation_gain':profile['translation_gain'],
            'brake_samples':17,'controlled_stop_budget_s':1.,'brake_model_margin_mm':model['model_translation_margin_mm'],
            'command_orientation_deg':self.maximum_rotation_deg,'raw_orientation_deg':self.maximum_rotation_deg+.15}
        self.lower=np.maximum(vector(cfg['limits']['lower_rad']),q-np.deg2rad(self.envelope))
        self.upper=np.minimum(vector(cfg['limits']['upper_rad']),q+np.deg2rad(self.envelope))
        self.vmax=np.full(6,np.deg2rad(profile['joint_velocity_deg_s']));self.amax=np.full(6,np.deg2rad(profile['joint_acceleration_deg_s2']))
        self.jmax=np.full(6,np.deg2rad(profile['joint_jerk_deg_s3']))
        self.otg=Ruckig(6,self.dt);self.output=OutputParameter(6);self.input=InputParameter(6)
        self.brake_otg=Ruckig(6,self.dt);self.brake_input=InputParameter(6);self.brake_trajectory=Trajectory(6)
        for inp in (self.input,self.brake_input):
            inp.control_interface=ControlInterface.Velocity;inp.synchronization=Synchronization.No
            inp.current_position=q.tolist();inp.current_velocity=[0.]*6;inp.current_acceleration=[0.]*6
            inp.target_velocity=[0.]*6;inp.target_acceleration=[0.]*6
            inp.max_velocity=self.vmax.tolist();inp.max_acceleration=self.amax.tolist();inp.max_jerk=self.jmax.tolist()
        self.filter=CommandFilter(cfg['filters'],self.dt,np.zeros(6))
        self.linear_lipschitz=float(1000*np.linalg.norm(self.jac[:3],2)*np.linalg.norm(self.vmax))
        self.angular_lipschitz=float(np.linalg.norm(self.jac[3:],2)*np.linalg.norm(self.vmax))
        self.phase='WAIT_RELEASE';self.released=False;self.locked=False;self.finished=False;self.was_gripped=False
        self.release_stops=0;self.stop_reason=None;self.stop_started=None;self.last_intent=None
        self.robot_reference=None;self.rotation_reference=None;self.goal_mm=np.zeros(3);self.goal_rotation_deg=np.zeros(3)
        self.boundary_brakes=0;self.last_brake_bounds=None;self.release_events=[];self.pending_release=None
        self.raw_rotation_deg=np.zeros(3)
        self.feedback_q=None;self.feedback_dq=None;self.tracking_audit={}
        self.tracking_slow_count=0;self.tracking_reversal_wait_count=0
        self.feedback_stall_cycles=0
        self.last_tracking_check={}

    def observe_feedback(self,q,dq):
        # 不滤波、不修正/重打反馈时间戳。周期主循环每次用同一原始EDG样本调用。
        previous=self.feedback_q
        self.feedback_q=vector(q);self.feedback_dq=vector(dq)
        gap=np.abs(self.feedback_q-self.state.q);index=int(np.argmax(gap))
        self.last_tracking_check={'check_scope':'previous_sent_reference_vs_current_raw_feedback_before_planning',
            'reference_q_rad':self.state.q.tolist(),'raw_q_rad':self.feedback_q.tolist(),
            'raw_dq_rad_s':self.feedback_dq.tolist(),'largest_joint_1_based':index+1,
            'largest_gap_deg':float(np.rad2deg(gap[index])),'hard_limit_deg':.05}
        stalled=(previous is not None and np.max(np.abs(self.feedback_q-previous))<np.deg2rad(.0001)
                 and np.max(np.abs(self.feedback_q-self.state.q))>=np.deg2rad(.015)
                 and np.max(np.abs(self.feedback_dq))<np.deg2rad(.002))
        self.feedback_stall_cycles=self.feedback_stall_cycles+1 if stalled else 0
        if self.feedback_stall_cycles*self.dt>=1.:
            raise SafetyFault('H9 raw feedback position stalled with command lead for 1 second')
        if np.max(np.abs(self.feedback_q-self.state.q))>np.deg2rad(.05):
            raise SafetyFault('H9 raw tracking >0.05degree before planning')
        if np.max(np.abs(self.feedback_dq))>np.deg2rad(1.):raise SafetyFault('H9 raw speed >1degree/s')

    def shape_joint_velocity(self,target):
        if self.feedback_q is None:
            # 独立离线候选允许没有硬件；真机循环必须提供原始反馈。
            return target
        error=float(np.max(np.abs(self.state.q-self.feedback_q)))
        # 0.015度起降速、0.025度停止新增速度，留出Ruckig减速及反馈噪声余量。
        # 这是更严格的内部预防门，不提高原0.05度保护线，不做延迟前馈补偿。
        scale=float(np.clip((np.deg2rad(.025)-error)/np.deg2rad(.01),0.,1.))
        reversal=bool(np.any((target*self.feedback_dq<0)&(np.abs(target)>np.deg2rad(.02))&
                             (np.abs(self.feedback_dq)>np.deg2rad(.05))))
        if reversal:scale=0.;self.tracking_reversal_wait_count+=1
        if scale<1.:self.tracking_slow_count+=1
        self.tracking_audit={'raw_gap_before_step_deg':float(np.rad2deg(error)),
            'velocity_scale':scale,'reversal_wait':reversal,'hard_limit_deg':.05,
            'soft_slow_deg':.015,'soft_zero_deg':.025,'sample_age_validated':False}
        self.tracking_audit['feedback_stall_cycles']=self.feedback_stall_cycles
        return target*scale

    def brake_bounds(self,q,v,a):
        safe=super().brake_bounds(q,v,a)
        self.last_brake_bounds['angular_sample_margin_deg']=self.angular_margin
        safe=bool(safe and self.last_brake_bounds['angular_upper_deg']+self.angular_margin<=self.maximum_rotation_deg)
        safe=bool(safe and self.tracking_brake_safe(q))
        self.last_brake_bounds['safe']=safe;return safe

    def tracking_brake_safe(self,q):
        safe=True
        if self.feedback_q is not None:
            extrema=self.brake_trajectory.position_extrema
            # 当前原始反馈为参考检查完整指令制动extrema，不拿下一候选当真实状态。
            lead=max(max(abs(q[i]+x.min-self.feedback_q[i]),abs(q[i]+x.max-self.feedback_q[i]))
                     for i,x in enumerate(extrema))
            stopping=not np.any(np.abs(self.input.target_velocity)>1e-12)
            # 先以0.035度触发Ruckig零速度重算；重算仍超0.045度则原保护退出。
            cap=np.deg2rad(.045 if stopping else .035)
            safe=bool(safe and lead<=cap)
            self.last_brake_bounds.update(tracking_brake_lead_deg=float(np.rad2deg(lead)),
                tracking_brake_limit_deg=float(np.rad2deg(cap)),tracking_scope='command_stop_extrema_vs_current_raw_feedback_not_physical_stop_proof')
            self.tracking_audit.update(predicted_command_brake_lead_deg=float(np.rad2deg(lead)),
                tracking_slow_count=self.tracking_slow_count,reversal_wait_count=self.tracking_reversal_wait_count)
        return safe

    def model_delta(self,q):return self.jac@(q-self.q0)

    def step(self,frame,now,finish=False,software_stop=False):
        result=super().step(frame,now,finish,software_stop)
        if self.phase=='GRIP_6DOF_SHADOW':self.phase='GRIP_6DOF_EXPANDED'
        if self.stop_reason:self.stop_reason=self.stop_reason.replace('pose shadow','H9')
        return result

    def check(self,candidate,actual,velocity):
        q=vector(candidate);actual=vector(actual);dq=vector(velocity)
        gap=np.abs(q-actual);index=int(np.argmax(gap))
        self.last_tracking_check={'check_scope':'next_candidate_vs_pre_send_raw_feedback',
            'candidate_q_rad':q.tolist(),'raw_q_rad':actual.tolist(),
            'raw_dq_rad_s':dq.tolist(),'largest_joint_1_based':index+1,'largest_gap_deg':float(np.rad2deg(gap[index])),
            'hard_limit_deg':.05}
        if np.any(q<self.lower-1e-9) or np.any(q>self.upper+1e-9):raise SafetyFault('H9 command joint domain')
        if np.max(np.abs(actual-self.q0))>np.deg2rad(self.envelope+.05):raise SafetyFault('H9 raw joint envelope')
        if gap[index]>np.deg2rad(.05):raise SafetyFault(f'H9 raw tracking >0.05degree (J{index+1}, gap={np.rad2deg(gap[index]):.6f}deg)')
        if np.max(np.abs(dq))>np.deg2rad(1.):raise SafetyFault('H9 raw speed >1degree/s')
        delta=self.model_delta(q)
        if np.linalg.norm(1000*delta[:3])>self.radius+1e-6:raise SafetyFault('H9 command sphere')
        if np.linalg.norm(delta[3:])>np.deg2rad(self.maximum_rotation_deg):raise SafetyFault('H9 command rotation')
        if any(np.any(np.abs(value)>cap+1e-7) for value,cap in ((self.state.v,self.vmax),(self.state.a,self.amax),(self.state.j,self.jmax))):
            raise SafetyFault('H9 command v/a/j')

    def check_tcp(self,tcp):
        if np.linalg.norm(vector(tcp)[:3]-self.origin_tcp[:3])>self.radius+.15:raise SafetyFault('H9 raw TCP sphere')
        self.raw_rotation_deg=np.rad2deg(rotation_vector(rpy_matrix(tcp)@rpy_matrix(self.origin_tcp).T))
        if np.linalg.norm(self.raw_rotation_deg)>self.maximum_rotation_deg+.15:raise SafetyFault('H9 raw Tool1 angle')
