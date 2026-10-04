"""静止阶段官方FK局部姿态轴数学模型；无运动模式或发送调用。"""
import numpy as np
from ..configuration import vector
from ..jaka import checked
from ..safety import SafetyFault
from .official_local import local_z_model,rpy_matrix,rotation_vector


def local_rotation_model(rc,q,axis='RX'):
    if axis not in ('RX','RY','RZ'):raise ValueError('rotation axis must be RX/RY/RZ')
    base=local_z_model(rc,q);jac=np.array(base['jacobian_m_rad']);q=vector(q)
    weight=np.diag([1,1,1,.2,.2,.2]);w=weight@jac;twist=np.zeros(6);twist[3+('RX','RY','RZ').index(axis)]=np.deg2rad(1.)
    gain=np.linalg.solve(w.T@w+np.eye(6)*base['damping_m']**2,w.T@(weight@twist))
    if np.linalg.norm((jac@gain-twist)[:3])>5e-6 or np.linalg.norm((jac@gain-twist)[3:])>1e-5:raise SafetyFault('rotation DLS residual')
    maximum=min(.02,float(np.deg2rad(.08)/np.max(np.abs(gain))))
    if maximum<.001:raise SafetyFault('rotation mathematical domain too small')
    pose=np.array(base['tcp_origin_mm_rad']);origin_r=rpy_matrix(pose);sweep=[]
    for angle in np.linspace(-maximum,maximum,21):
        candidate=q+gain*angle;p=vector(checked(rc.kine_forward(candidate.tolist()),'rotation local FK'))
        err=rotation_vector(rpy_matrix(p)@origin_r.T)-twist[3:]*angle
        sweep.append({'target_angle_deg':float(angle),'q_rad':candidate.tolist(),'pose_mm_rad':p.tolist(),
                      'tcp_drift_mm':float(np.linalg.norm(p[:3]-pose[:3])),'angular_residual_deg':float(np.rad2deg(np.linalg.norm(err)))})
    if max(s['tcp_drift_mm'] for s in sweep)>.01 or max(s['angular_residual_deg'] for s in sweep)>.005:raise SafetyFault('rotation FK sweep failed')
    return {'scope':'official_rotation_axis_math_only_not_live_clearance','axis':axis,'q_origin_rad':q.tolist(),
            'tcp_origin_mm_rad':pose.tolist(),'jacobian_m_rad':jac.tolist(),'gain_rad_per_deg':gain.tolist(),
            'maximum_angle_deg':maximum,'sweep':sweep,'physical_calibration_passed':False}
