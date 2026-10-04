"""七次 Bernstein 轨迹：保持 q/v/a/jerk 连续，凸包给出整段导数上界。

这是独立软件轨迹生成器，不是 JAKA 内部 S 规划器，也不声称时间最优。
重定时不裁剪下发点；找不到满足边界的轨迹就明确拒绝。
"""
from dataclasses import dataclass
import math
import numpy as np
from .configuration import vector

@dataclass
class Motion:
    q: np.ndarray
    v: np.ndarray
    a: np.ndarray
    j: np.ndarray
    @classmethod
    def rest(cls,q): return cls(vector(q),np.zeros(6),np.zeros(6),np.zeros(6))

class Segment:
    def __init__(self,start,target,duration):
        if not math.isfinite(duration) or duration<=0: raise ValueError('duration')
        self.duration=duration; T=duration
        p=np.empty((8,6)); p[0]=vector(start.q)
        p[1]=p[0]+vector(start.v)*T/7
        p[2]=vector(start.a)*T*T/42+2*p[1]-p[0]
        p[3]=vector(start.j)*T**3/210+3*p[2]-3*p[1]+p[0]
        p[4:]=vector(target)
        self.controls=[p]
        for k in range(3): self.controls.append(np.diff(self.controls[-1],axis=0)*(7-k)/T)
    def sample(self,t):
        s=min(1.,max(0.,float(t)/self.duration))
        values=[]
        for p in self.controls:
            n=len(p)-1
            b=np.array([math.comb(n,i)*s**i*(1-s)**(n-i) for i in range(n+1)])
            values.append(b@p)
        return Motion(*values)
    def valid(self,limits):
        lo,hi=np.array(limits['lower_rad']),np.array(limits['upper_rad'])
        if np.any(self.controls[0]<lo-1e-12) or np.any(self.controls[0]>hi+1e-12): return False
        for p,key in zip(self.controls[1:],['velocity_rad_s','acceleration_rad_s2','jerk_rad_s3']):
            if np.any(np.abs(p)>np.asarray(limits[key])+1e-10): return False
        return True

def plan(start,target,limits,min_duration=.12,max_duration=3600.):
    target=vector(target)
    if np.any(target<limits['lower_rad']) or np.any(target>limits['upper_rad']): raise ValueError('joint target outside range')
    T=min_duration
    while T<=max_duration:
        segment=Segment(start,target,T)
        if segment.valid(limits): return segment
        T*=1.2
    raise ValueError('no certified trajectory: state, boundary or horizon infeasible')

class OnlinePlanner:
    def __init__(self,q,limits,dt):
        self.state=Motion.rest(q); self.limits=limits; self.dt=dt
        self.segment=None; self.elapsed=0.
    def retarget(self,target,min_duration=.3):
        # 从当前承诺的 q/v/a/j 出发，不能偷偷归零导数。
        seg=plan(self.state,target,self.limits,min_duration=min_duration)
        self.segment=seg; self.elapsed=0.
    def step(self):
        if self.segment:
            self.elapsed=min(self.elapsed+self.dt,self.segment.duration)
            self.state=self.segment.sample(self.elapsed)
        return self.state
