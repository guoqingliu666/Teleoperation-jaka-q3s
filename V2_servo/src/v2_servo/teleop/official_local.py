"""当前Tool1的官方FK局部微分；只在静止阶段调用SDK，不做逐周期IK。"""
import time
import numpy as np
from ..configuration import vector
from ..jaka import checked
from ..kinematics import rotate
from ..safety import SafetyFault


def rpy_matrix(pose):
    p=vector(pose)
    return rotate([0,0,1],p[5])@rotate([0,1,0],p[4])@rotate([1,0,0],p[3])


def rotation_vector(matrix):
    m=np.asarray(matrix,float);angle=np.arccos(np.clip((np.trace(m)-1)/2,-1,1))
    skew=np.array([m[2,1]-m[1,2],m[0,2]-m[2,0],m[1,0]-m[0,1]])
    return skew*.5 if angle<1e-7 else skew*angle/(2*np.sin(angle))


def official_jacobian(rc,q,step_rad=1e-4):
    """中心差分：上3行m/rad，下3行基坐标旋转向量rad/rad；不是欧拉角速度。"""
    q=vector(q);jac=np.empty((6,6));samples=[]
    for axis in range(6):
        poses=[]
        for sign in (-1,1):
            candidate=q.copy();candidate[axis]+=sign*step_rad
            begin=time.perf_counter();p=vector(checked(rc.kine_forward(candidate.tolist()),'official local FK'))
            samples.append({'q_rad':candidate.tolist(),'pose_mm_rad':p.tolist(),
                            'call_ms':(time.perf_counter()-begin)*1000,'axis':axis,'sign':sign})
            poses.append(p)
        minus,plus=poses
        jac[:3,axis]=(plus[:3]-minus[:3])/1000/(2*step_rad)
        jac[3:,axis]=rotation_vector(rpy_matrix(plus)@rpy_matrix(minus).T)/(2*step_rad)
    if not np.isfinite(jac).all():raise SafetyFault('nonfinite official Jacobian')
    return jac,samples


def local_translation_model(rc,q,axis='Z'):
    """数值尺度/误差门槛仅用于候选数学检查，不是干细胞任务精度或安全标定。"""
    if axis not in ('X','Y','Z'):raise ValueError('official translation axis must be X/Y/Z')
    index='XYZ'.index(axis)
    q=vector(q);pose=vector(checked(rc.kine_forward(q.tolist()),'official FK origin'))
    jac,samples=official_jacobian(rc,q)
    second,other=official_jacobian(rc,q,2e-4)
    weight=np.diag([1,1,1,.2,.2,.2]);weighted=weight@jac
    singular=np.linalg.svd(weighted,compute_uv=False)
    derivative_error=float(np.linalg.norm(weight@(jac-second))/max(np.linalg.norm(weighted),1e-12))
    if derivative_error>.005:raise SafetyFault('official FK derivative step comparison exceeds 0.5%')
    if singular[-1]<.005:raise SafetyFault('official local weighted Jacobian singularity screening')
    damping=.0001
    target=np.zeros(6);target[index]=1. # 1m/s单位参考仅用于算增益，不发送该速度。
    gain=np.linalg.solve(weighted.T@weighted+np.eye(6)*damping**2,weighted.T@(weight@target))/1000
    twist=jac@gain
    if np.linalg.norm(twist[:3]-target[:3]/1000)>.000005 or np.linalg.norm(twist[3:])>1e-5:
        raise SafetyFault('local damped '+axis+' gain residual exceeds numerical check')
    maximum_z_mm=min(.5,float(np.deg2rad(.1)/max(np.max(np.abs(gain)),1e-12)))
    if maximum_z_mm<.05:raise SafetyFault('local Z commissioning envelope too small; review singularity')
    line=[]
    for dz in np.linspace(0,maximum_z_mm,21):
        candidate=q+gain*dz;p=vector(checked(rc.kine_forward(candidate.tolist()),'official '+axis+' sweep'))
        line.append({'axis_target_delta_mm':float(dz),'z_target_delta_mm':float(dz) if axis=='Z' else None,'q_rad':candidate.tolist(),
                     'official_pose_mm_rad':p.tolist(),
                     'translation_residual_mm':float(np.linalg.norm(p[:3]-pose[:3]-target[:3]*dz)),
                     'orientation_residual_deg':float(np.rad2deg(np.linalg.norm(rotation_vector(rpy_matrix(p)@rpy_matrix(pose).T))))})
    if max(r['translation_residual_mm'] for r in line)>.01 or max(r['orientation_residual_deg'] for r in line)>.005:
        raise SafetyFault('official local Z sweep linearization residual too large')
    return {'scope':'official_FK_local_math_only_not_collision_or_physical_calibration',
            'q_origin_rad':q.tolist(),'tcp_origin_mm_rad':pose.tolist(),
            'axis':axis,'jacobian_m_rad':jac.tolist(),'joint_gain_rad_per_mm_'+axis:gain.tolist(),
            'differential_steps_rad':[1e-4,2e-4],'derivative_relative_error':derivative_error,
            'rotation_weight_m':.2,'damping_m':damping,'weighted_singular_values':singular.tolist(),
            'maximum_axis_mm':maximum_z_mm,'maximum_z_mm':maximum_z_mm if axis=='Z' else None,'joint_relative_envelope_deg':.1,
            'sweep':line,'fk_samples':samples+other,'physical_calibration_passed':False}


def local_z_model(rc,q):
    return local_translation_model(rc,q,'Z')
