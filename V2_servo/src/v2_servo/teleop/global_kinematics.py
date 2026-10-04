"""六转动关节的乘积指数模型；由官方FK/Jacobian识别，不引用预览CAD尺寸。

模型没有10mm局部半径。识别后的全域精度必须用独立官方FK验证；
即使数学通过，也不代表物理标定、碰撞或真机控制已放行。
"""
import math
import numpy as np
from ..configuration import vector
from ..kinematics import CalibratedKinematics
from .official_local import rpy_matrix

def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])

def rotation_log(r):
    r=np.asarray(r,float)
    if r.shape!=(3,3) or not np.isfinite(r).all():raise ValueError('invalid rotation')
    theta=math.acos(float(np.clip((np.trace(r)-1)/2,-1,1)))
    s=np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])
    if theta<1e-7:return s*.5
    if math.pi-theta<1e-5:
        _,vectors=np.linalg.eigh((r+r.T)*.5)
        axis=vectors[:,-1]
        if axis@s<0:axis=-axis
        return axis*theta
    return s*theta/(2*math.sin(theta))

def pose_matrix(pose_mm_rad):
    p=vector(pose_mm_rad);t=np.eye(4);t[:3,:3]=rpy_matrix(p);t[:3,3]=p[:3]/1000
    return t

class SerialPoE:
    def __init__(self,record):
        self.q0=vector(record['q_reference_rad'])
        self.home=CalibratedKinematics.rigid(record['T_reference_m'])
        self.screws=np.asarray(record['space_screws_v_w'],float)
        if self.screws.shape!=(6,6) or not np.isfinite(self.screws).all():raise ValueError('invalid screw axes')
        if not np.allclose(np.linalg.norm(self.screws[3:],axis=0),1.,atol=1e-7):raise ValueError('non-unit angular axes')
        if not np.allclose(np.sum(self.screws[:3]*self.screws[3:],axis=0),0.,atol=1e-7):raise ValueError('not revolute screw axes')

    @staticmethod
    def identify(q,pose,jacobian):
        t=pose_matrix(pose);j=np.asarray(jacobian,float)
        if j.shape!=(6,6) or not np.isfinite(j).all():raise ValueError('invalid official Jacobian')
        norms=np.linalg.norm(j[3:],axis=0)
        if np.max(np.abs(norms-1))>.001:raise ValueError('official derivative not six revolute joints')
        w=j[3:]/norms
        v=j[:3]/norms-np.cross(w.T,t[:3,3]).T
        # 数值微分可能产生微小螺距；超阈值拒绝，不能用投影掩盖模型错误。
        pitch=np.sum(v*w,axis=0)
        if np.max(np.abs(pitch))>1e-5:raise ValueError('official Jacobian non-revolute pitch')
        v-=w*pitch
        return dict(schema=1,scope='official_FK_identified_PoE_math_candidate_not_live_release',
            q_reference_rad=vector(q).tolist(),T_reference_m=t.tolist(),space_screws_v_w=np.r_[v,w].tolist(),
            numerical_pitch_removed_m=pitch.tolist(),physical_calibration_validated=False,collision_validated=False)

    def forward(self,q):
        delta=vector(q)-self.q0;t=np.eye(4);js=np.empty((6,6))
        for i,angle in enumerate(delta):
            v=self.screws[:3,i];w=self.screws[3:,i]
            rw=t[:3,:3]@w
            js[:,i]=np.r_[t[:3,:3]@v+np.cross(t[:3,3],rw),rw]
            k=skew(w);k2=k@k;sa=math.sin(angle);ca=math.cos(angle)
            e=np.eye(4);e[:3,:3]=np.eye(3)+sa*k+(1-ca)*k2
            e[:3,3]=(np.eye(3)*angle+(1-ca)*k+(angle-sa)*k2)@v
            t=t@e
        tcp=t@self.home
        jac=js.copy();jac[:3]+=np.cross(js[3:].T,tcp[:3,3]).T
        # 无CAD/碰撞连杆表，不生成虚假的连杆几何。
        return tcp,jac,None

def evaluate_samples(model,samples):
    rows=[]
    for sample in samples:
        expected=pose_matrix(sample['official_pose_mm_rad']);actual,_,_=model.forward(sample['q_rad'])
        rows.append(dict(q_rad=sample['q_rad'],official_pose_mm_rad=sample['official_pose_mm_rad'],
            translation_error_mm=float(np.linalg.norm(actual[:3,3]-expected[:3,3])*1000),
            rotation_error_deg=float(np.rad2deg(np.linalg.norm(rotation_log(actual[:3,:3]@expected[:3,:3].T))))))
    return rows
