"""从来源 URDF 读取运动链，仅离线 FK/Jacobian；不自编真机 IK。"""
import xml.etree.ElementTree as ET
import math
import numpy as np
from .configuration import vector

def rotate(axis,angle):
    x,y,z=axis; K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)

def origin(element):
    t=np.eye(4)
    if element is None: return t
    t[:3,3]=[float(x) for x in element.get('xyz','0 0 0').split()]
    r,p,y=[float(x) for x in element.get('rpy','0 0 0').split()]
    t[:3,:3]=rotate([0,0,1],y)@rotate([0,1,0],p)@rotate([1,0,0],r)
    return t

class ArmKinematics:
    def __init__(self,path):
        root=ET.parse(path).getroot(); self.chain=[]
        base_joint=root.find("joint[@name='world_to_link_00']")
        if base_joint is None: base_joint=root.find("joint[@name='world_to_base']")
        self.base=origin(None if base_joint is None else base_joint.find('origin'))
        for i in range(1,7):
            j=root.find(f"joint[@name='joint_{i}']")
            if j is None: raise ValueError('missing six-joint chain')
            axis=vector([float(x) for x in j.find('axis').get('xyz').split()],3)
            axis=axis/np.linalg.norm(axis)
            self.chain.append((origin(j.find('origin')),axis))
    def forward(self,q):
        q=vector(q); t=self.base.copy(); points=[t[:3,3].copy()]; axes=[]
        for angle,(o,axis) in zip(q,self.chain):
            t=t@o; points.append(t[:3,3].copy()); axes.append(t[:3,:3]@axis)
            r=np.eye(4); r[:3,:3]=rotate(axis,angle); t=t@r
        end=t[:3,3]
        jac=np.zeros((6,6))
        for i in range(6):
            jac[:3,i]=np.cross(axes[i],end-points[i+1]); jac[3:,i]=axes[i]
        # 上三行单位 m，按 1 m 特征长度归一化；仅用于筛查，不是真机奇异安全证明。
        return t,jac,np.array(points)

class CalibratedKinematics:
    """显式模型→机器人基座、Link06→TCP与关节零偏；未知变换不能用预览值顶替。"""
    def __init__(self,arm,model_to_robot,link06_to_tcp,joint_offsets):
        self.arm=arm; self.base=self.rigid(model_to_robot); self.tip=self.rigid(link06_to_tcp); self.offsets=vector(joint_offsets)
    @staticmethod
    def rigid(value):
        t=np.array(value,float)
        if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1]): raise ValueError('invalid rigid transform')
        r=t[:3,:3]
        if not np.allclose(r.T@r,np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(r),1): raise ValueError('transform rotation must be SO(3)')
        return t
    def forward(self,q):
        t,j,points=self.arm.forward(vector(q)+self.offsets); offset=t[:3,:3]@self.tip[:3,3]; r=self.base[:3,:3]
        jac=np.empty((6,6)); jac[:3]=r@(j[:3]+np.cross(j[3:].T,offset).T); jac[3:]=r@j[3:]
        return self.base@t@self.tip,jac,(r@points.T).T+self.base[:3,3]
