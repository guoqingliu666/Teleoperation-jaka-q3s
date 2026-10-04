"""H6官方局部三维数学模型；仅静止阶段调用FK，非物理/碰撞标定。"""
import itertools
import numpy as np
from ..configuration import vector
from ..jaka import checked
from ..safety import SafetyFault
from .official_local import local_z_model,rpy_matrix,rotation_vector


def local_xyz_model(rc,q):
    base=local_z_model(rc,q);q=vector(q);jac=np.array(base['jacobian_m_rad'])
    weight=np.diag([1,1,1,.2,.2,.2]);w=weight@jac
    gain=np.linalg.solve(w.T@w+np.eye(6)*base['damping_m']**2,w.T@weight[:,:3])/1000
    residual=jac@gain-np.r_[np.eye(3)/1000,np.zeros((3,3))]
    if np.max(np.linalg.norm(residual[:3],axis=0))>5e-6 or np.max(np.linalg.norm(residual[3:],axis=0))>1e-5:
        raise SafetyFault('H6 local translation DLS residual')
    radius=min(.5,float(np.deg2rad(.08)/np.max(np.linalg.norm(gain,axis=1))))
    if radius<.05:raise SafetyFault('H6 local translation trust sphere too small')
    pose=np.array(base['tcp_origin_mm_rad']);rotation=rpy_matrix(pose);samples=[]
    # 同时平移的角点以及整个0.1度关节信任区域角点；采样一致不是连续域证明。
    cart=[q+gain@(np.array(c)*radius/np.sqrt(3)) for c in itertools.product((-1,0,1),repeat=3)]
    joints=[q+np.array(c)*np.deg2rad(.1) for c in itertools.product((-1,1),repeat=6)]
    for candidate in cart+joints:
        actual=vector(checked(rc.kine_forward(candidate.tolist()),'H6 local FK sweep'))
        delta=candidate-q;linear=1000*jac[:3]@delta;angular=jac[3:]@delta
        samples.append({'q_rad':candidate.tolist(),'official_pose_mm_rad':actual.tolist(),
            'translation_residual_mm':float(np.linalg.norm(actual[:3]-pose[:3]-linear)),
            'orientation_residual_deg':float(np.rad2deg(np.linalg.norm(rotation_vector(rpy_matrix(actual)@rotation.T)-angular)))})
    if max(s['translation_residual_mm'] for s in samples)>.01 or max(s['orientation_residual_deg'] for s in samples)>.005:
        raise SafetyFault('H6 local 3D FK sweep residual exceeds mathematical gate')
    return {'scope':'official_local_XYZ_math_only_not_physical_or_collision_calibration',
        'q_origin_rad':q.tolist(),'tcp_origin_mm_rad':pose.tolist(),'jacobian_m_rad':jac.tolist(),
        'gain_rad_per_mm_xyz':gain.tolist(),'maximum_radius_mm':radius,
        'joint_relative_envelope_deg':.1,'weighted_singular_values':base['weighted_singular_values'],
        'derivative_relative_error':base['derivative_relative_error'],'damping_m':base['damping_m'],
        'rotation_weight_m':.2,'sweep':samples,'physical_calibration_passed':False}
