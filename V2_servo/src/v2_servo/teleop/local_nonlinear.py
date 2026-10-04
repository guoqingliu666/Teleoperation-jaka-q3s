"""H10：静止阶段官方FK三次局部近似；8ms内仅本地求导，不调用SDK IK/FK。

独立验证采样不是连续误差/碰撞证明，只供现场受监督有限实验。
"""
import itertools
import numpy as np
from ruckig import Result
from ..configuration import vector
from ..jaka import checked
from ..safety import SafetyFault
from .official_local import rpy_matrix,rotation_vector
from .local_xyz import local_xyz_model
from .pose_expansion import QuestPoseExpansionPlanner

SCOPE='official_H10_local_cubic_FK_math_only_not_physical_clearance'
CAPS=dict(maximum_radius_mm=10.,maximum_rotation_deg=.5,joint_envelope_deg=6.,joint_velocity_deg_s=.5,
    joint_acceleration_deg_s2=2.,joint_jerk_deg_s3=8.,cartesian_speed_mm_s=2.,angular_speed_deg_s=.15,
    translation_gain=.5,rotation_gain=.02,minimum_site_clearance_mm=200,controlled_stop_budget_s=1.,
    maximum_motion_window_s=30,seed=20261003)

def validate_workspace(p):
    if p.get('scope')!='H10_official_local_nonlinear_pilot_not_full_workspace':raise SafetyFault('H10 profile scope')
    for k,v in CAPS.items():
        if isinstance(p.get(k),bool) or p.get(k)!=v:raise SafetyFault('H10 unreviewed parameter '+k)
    return p

POWERS=np.array([a for a in itertools.product(range(4),repeat=6) if sum(a)<=3],dtype=int)

class CubicFK:
    def __init__(self,q,scale,coefficients):
        self.q=vector(q);self.scale=float(scale);self.c=np.asarray(coefficients)
        if not np.isfinite(self.scale) or self.scale<=0 or self.c.shape!=(len(POWERS),6) or not np.isfinite(self.c).all():
            raise SafetyFault('H10 invalid polynomial coefficients/scale')
    def evaluate(self,q):
        x=(np.asarray(q)-self.q)/self.scale
        if not np.isfinite(x).all() or np.max(np.abs(x))>1.+1e-7:raise SafetyFault('H10 nonlinear model outside trained joint domain')
        return np.prod(x[...,None,:]**POWERS,axis=-1)@self.c
    def jacobian(self,q):
        x=(vector(q)-self.q)/self.scale;out=np.zeros((6,6))
        if np.max(np.abs(x))>1.+1e-7:raise SafetyFault('H10 Jacobian outside trained domain')
        for j in range(6):
            mask=POWERS[:,j]>0;p=POWERS[mask].copy();p[:,j]-=1
            out[:,j]=(np.prod(x**p,axis=1)*POWERS[mask,j])@self.c[mask]/self.scale
        return out
    def derivative_upper(self):
        # |x|<=1的多项式区间上界，不是以角点采样代替连续时间速度上界。
        return np.stack([np.sum(np.abs(self.c)*POWERS[:,j,None],axis=0)/self.scale for j in range(6)],axis=1)

def workspace_model(rc,q,profile,limits,emit=lambda **row:None):
    validate_workspace(profile);q=vector(q);base=local_xyz_model(rc,q);pose=vector(base['tcp_origin_mm_rad']);rot=rpy_matrix(pose)
    rng=np.random.default_rng(profile['seed']);audit=[];calls=0
    def official(candidate):
        nonlocal calls
        p=vector(checked(rc.kine_forward(candidate.tolist()),'H10 static official FK'))
        calls+=1
        if calls%100==0:emit(event='H10_model_progress',official_FK_calls=calls,motion_commands=0)
        return np.r_[(p[:3]-pose[:3])/1000,rotation_vector(rpy_matrix(p)@rot.T)],p
    for envelope in (6.,4.,2.):
        scale=np.deg2rad(envelope)
        if np.any(q-scale<vector(limits['lower_rad'])) or np.any(q+scale>vector(limits['upper_rad'])):continue
        # 训练与独立随机验证使用不同样本，种子记录；厂商FK调用全部在静止阶段。
        x=np.r_[np.zeros((1,6)),np.array(list(itertools.product((-1.,1.),repeat=6))),rng.uniform(-1,1,(384,6))]
        features=np.prod(x[:,None,:]**POWERS,axis=-1)
        y=np.array([official(q+t*scale)[0] for t in x]);coeff,_,rank,_=np.linalg.lstsq(features,y,rcond=None)
        if rank!=len(POWERS):raise SafetyFault('H10 cubic fit rank')
        model=CubicFK(q,scale,coeff);zero=model.evaluate(q);coeff[0]-=zero  # POWERS[0]常数项，使当前起点严格为零。
        tests=[]
        for t in rng.uniform(-1,1,(192,6)):
            candidate=q+t*scale;expected,p=official(candidate);prediction=model.evaluate(candidate)
            tests.append(dict(q_rad=candidate.tolist(),official_pose_mm_rad=p.tolist(),
                translation_error_mm=float(np.linalg.norm(expected[:3]-prediction[:3])*1000),
                rotation_chart_error_deg=float(np.rad2deg(np.linalg.norm(expected[3:]-prediction[3:])))))
        le=max(t['translation_error_mm'] for t in tests);ae=max(t['rotation_chart_error_deg'] for t in tests)
        attempt=dict(joint_envelope_deg=envelope,holdout=tests,max_holdout_translation_mm=le,max_holdout_rotation_deg=ae)
        audit.append(attempt)
        if le>.2 or ae>.025:continue
        jac=model.jacobian(q);w=np.diag([1,1,1,.2,.2,.2])
        if np.linalg.svd(w@jac,compute_uv=False)[-1]<.005:continue
        # 对独立点数值求导复核本地Jacobian；全部在SDK静止阶段。
        derivatives=[]
        for t in rng.uniform(-.7,.7,(12,6)):
            candidate=q+t*scale;numeric=np.empty((6,6));h=1e-4
            for j in range(6):
                d=np.zeros(6);d[j]=h;numeric[:,j]=(official(candidate+d)[0]-official(candidate-d)[0])/(2*h)
            error=float(np.linalg.norm(w@(numeric-model.jacobian(candidate)))/np.linalg.norm(w@numeric));derivatives.append(error)
        attempt['derivative_relative_errors']=derivatives
        if max(derivatives)>.01:continue
        for radius in (10.,8.,5.):
            rows=[];good=True
            dirs=[np.asarray(c,dtype=float) for c in itertools.product((-1,0,1),repeat=3)]
            dirs=[d/max(1.,np.linalg.norm(d)) for d in dirs]
            for dl,dr in itertools.product(dirs,repeat=2):
                goal=np.r_[dl*radius/1000,dr*np.deg2rad(profile['maximum_rotation_deg'])];candidate=q.copy()
                for _ in range(8):
                    error=goal-model.evaluate(candidate)
                    if np.linalg.norm(w@error)<1e-8:break
                    candidate+=np.linalg.lstsq(w@model.jacobian(candidate),w@error,rcond=None)[0]
                    if np.max(np.abs(candidate-q))>scale*.7:good=False;break
                if not good:break
                if np.linalg.norm(w@(goal-model.evaluate(candidate)))>1e-7:good=False;break
                actual,p=official(candidate);prediction=model.evaluate(candidate)
                rows.append(dict(q_rad=candidate.tolist(),official_pose_mm_rad=p.tolist(),
                    translation_error_mm=float(np.linalg.norm(actual[:3]-prediction[:3])*1000),
                    rotation_chart_error_deg=float(np.rad2deg(np.linalg.norm(actual[3:]-prediction[3:])))))
            if not good or not rows:continue
            le=max(le,max(r['translation_error_mm'] for r in rows));ae=max(ae,max(r['rotation_chart_error_deg'] for r in rows))
            if le>.2 or ae>.025:continue
            attempt['workspace_sweep']=rows;attempt['accepted_radius_mm']=radius
            dls=np.linalg.solve((w@jac).T@(w@jac)+np.eye(6)*1e-8,(w@jac).T@w)
            return dict(scope=SCOPE,q_origin_rad=q.tolist(),tcp_origin_mm_rad=pose.tolist(),jacobian_m_rad=jac.tolist(),dls_rad_per_m_rad=dls.tolist(),
                maximum_radius_mm=radius,joint_envelope_deg=envelope,model_translation_margin_mm=max(.05,le*1.5),
                model_rotation_margin_deg=max(.005,ae*1.5),coefficients=coeff.tolist(),normalized_scale_rad=scale,
                polynomial_powers=POWERS.tolist(),profile=profile,attempts=audit,official_FK_calls=calls,
                continuous_error_bound_validated=False,collision_clearance_validated=False,physical_stop_validated=False,
                full_workspace_approved=False,source='static_official_FK_only_not_URDF_calibration')
    exc=SafetyFault('H10 official nonlinear model cannot support >=5mm; no motion');exc.math_audit=audit;raise exc

class QuestWorkspacePlanner(QuestPoseExpansionPlanner):
    model_scope=SCOPE;profile_validator=staticmethod(validate_workspace);radius_cap=10.;allowed_envelopes=(2.,4.,6.)
    def __init__(self,q,cfg,model,origin_tcp,profile):
        super().__init__(q,cfg,model,origin_tcp,profile)
        if model.get('polynomial_powers')!=POWERS.tolist():raise SafetyFault('H10 coefficient convention')
        self.nonlinear=CubicFK(model['q_origin_rad'],model['normalized_scale_rad'],model['coefficients'])
        self.rotation_goal_fraction=.6
        upper=self.nonlinear.derivative_upper()
        self.linear_lipschitz=1000*np.linalg.norm(upper[:3],2)*np.linalg.norm(self.vmax)
        self.angular_lipschitz=np.linalg.norm(upper[3:],2)*np.linalg.norm(self.vmax)
    def model_delta(self,q):return self.nonlinear.evaluate(q)
    @property
    def xyz_target_mm(self):return self.model_delta(self.state.q)[:3]*1000
    @property
    def rotation_target_deg(self):return np.rad2deg(self.model_delta(self.state.q)[3:])
    def step(self,frame,now,finish=False,software_stop=False):
        # 图表旋转向量导数，不把RPY导数当空间角速度；图表范数<0.5度。
        self.jac=self.nonlinear.jacobian(self.state.q);w=np.diag([1,1,1,.2,.2,.2]);a=w@self.jac
        if np.linalg.svd(a,compute_uv=False)[-1]<.005:raise SafetyFault('H10 local model singularity')
        self.dls=np.linalg.solve(a.T@a+np.eye(6)*1e-8,a.T@w)
        m=super().step(frame,now,finish,software_stop)
        if self.stop_reason:self.stop_reason=self.stop_reason.replace('H9','H10')
        self.phase=self.phase.replace('EXPANDED','WORKSPACE');return m
    def brake_bounds(self,q,v,a):
        b=self.brake_input;b.current_position=[0.]*6;b.current_velocity=v.tolist();b.current_acceleration=a.tolist()
        result=self.brake_otg.calculate(b,self.brake_trajectory);duration=float(self.brake_trajectory.duration)
        if result not in (Result.Working,Result.Finished) or duration>1:raise SafetyFault('H10 brake calculation/budget')
        ex=self.brake_trajectory.position_extrema
        joint_ok=all(q[i]+x.min>=self.lower[i]-1e-9 and q[i]+x.max<=self.upper[i]+1e-9 for i,x in enumerate(ex))
        if not joint_ok:return False
        qs=np.array([self.brake_trajectory.at_time(t)[0] for t in np.linspace(0,duration,17)])+q
        chart=self.nonlinear.evaluate(qs);half=duration/32
        linear=float(np.max(np.linalg.norm(chart[:,:3],axis=1))*1000+self.linear_lipschitz*half+self.p['brake_model_margin_mm'])
        angular=float(np.rad2deg(np.max(np.linalg.norm(chart[:,3:],axis=1))+self.angular_lipschitz*half)+self.angular_margin)
        self.last_brake_bounds=dict(duration_s=duration,linear_radius_upper_mm=linear,angular_upper_deg=angular,joint_extrema_inside=joint_ok,
            nonlinear_model=True,scope='local_polynomial_command_prediction_not_physical_stop_proof')
        safe=bool(linear<=self.radius and angular<=self.maximum_rotation_deg and self.tracking_brake_safe(q))
        self.last_brake_bounds['safe']=safe;return safe
    def check_model_feedback(self,q,tcp):
        predicted=self.model_delta(q);raw=vector(tcp)
        measured=np.r_[(raw[:3]-self.origin_tcp[:3])/1000,rotation_vector(rpy_matrix(raw)@rpy_matrix(self.origin_tcp).T)]
        if np.linalg.norm(predicted[:3]-measured[:3])*1000>.25 or np.rad2deg(np.linalg.norm(predicted[3:]-measured[3:]))>.05:
            raise SafetyFault('H10 raw feedback/model disagreement')
