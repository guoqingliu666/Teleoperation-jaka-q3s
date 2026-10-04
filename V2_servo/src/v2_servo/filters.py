"""控制目标与观测信号分开滤波；安全阈值始终另查原始值。"""
import math
from collections import deque
import numpy as np
from .configuration import vector

class LowPass:
    def __init__(self, hz, dt, initial):
        self.dt=dt; self.y=np.asarray(initial,dtype=float).copy(); self.set_cutoff(hz)
    def set_cutoff(self, hz):
        if not math.isfinite(hz) or not 0 < hz < .5/self.dt: raise ValueError('cutoff outside Nyquist')
        self.hz=hz; self.alpha=-math.expm1(-2*math.pi*hz*self.dt)
    def update(self,x):
        x=np.asarray(x,dtype=float)
        if x.shape != self.y.shape or not np.isfinite(x).all(): raise ValueError('invalid filter input')
        self.y += self.alpha*(x-self.y)
        return self.y.copy()

class Notch:
    def __init__(self,hz,q,dt,initial):
        if not 0<hz<.5/dt or not math.isfinite(q) or q<=0: raise ValueError('notch')
        w=2*math.pi*hz*dt; a=math.sin(w)/(2*q)
        self.b=np.array([1,-2*math.cos(w),1])/(1+a)
        self.a=np.array([1+a,-2*math.cos(w),1-a])/(1+a)
        self.x1=np.array(initial,dtype=float); self.x2=self.x1.copy()
        self.y1=self.x1.copy(); self.y2=self.x1.copy()
    def update(self,x):
        x=np.asarray(x,dtype=float)
        y=self.b[0]*x+self.b[1]*self.x1+self.b[2]*self.x2-self.a[1]*self.y1-self.a[2]*self.y2
        self.x2,self.x1=self.x1,x.copy(); self.y2,self.y1=self.y1,y.copy()
        return y

class CommandFilter:
    def __init__(self, config, dt, initial, *, comparison=False):
        self.c=dict(config); self.dt=dt
        if not config['enabled'] and not comparison: raise ValueError('filter bypass only allowed in offline comparison')
        self.lowpass=LowPass(config['command_hz'],dt,initial)
        self.window=deque([vector(initial)]*config['moving_average'], maxlen=config['moving_average'])
        self.notch=Notch(config['notch_hz'],config['notch_q'],dt,initial) if config['notch_hz'] else None
    def update(self,x):
        x=vector(x)
        if not self.c['enabled']: return x
        self.window.append(x)
        y=np.mean(self.window,axis=0)
        if self.notch: y=self.notch.update(y)
        return self.lowpass.update(y)
    def tune_cutoff(self,hz):
        # 保留滤波状态；轨迹规划器在下游保证导数不跳变。
        self.lowpass.set_cutoff(hz); self.c['command_hz']=hz

class TelemetryFilter:
    def __init__(self,c,dt,q):
        self.dt=dt; self.q=vector(q); self.v=np.zeros(6)
        self.v_filter=LowPass(c['velocity_hz'],dt,self.v)
        self.a_filter=LowPass(c['acceleration_hz'],dt,self.v)
        self.t_filter=LowPass(c['torque_hz'],dt,self.v)
    def update(self,q,torque,elapsed=None):
        dt=self.dt if elapsed is None else elapsed
        if not math.isfinite(dt) or dt<=0: raise ValueError('invalid feedback interval')
        q=vector(q); v=(q-self.q)/dt; a=(v-self.v)/dt
        self.q,self.v=q,v
        return {'velocity_raw':v,'acceleration_raw':a,'velocity_filtered':self.v_filter.update(v),
                'acceleration_filtered':self.a_filter.update(a),
                'torque_filtered':None if torque is None else self.t_filter.update(vector(torque))}

class CartesianFilter:
    """位置用低通，姿态在单位四元数上球面插值，避免欧拉角跨界。"""
    def __init__(self,hz,dt,xyz,quat):
        self.lp=LowPass(hz,dt,vector(xyz,3)); self.q=self.unit(quat)
    @staticmethod
    def unit(q):
        q=vector(q,4); n=np.linalg.norm(q)
        if n<1e-9: raise ValueError('zero quaternion')
        return q/n
    def update(self,xyz,quat):
        b=self.unit(quat); dot=float(self.q@b)
        if dot<0: b=-b; dot=-dot
        a=self.lp.alpha; dot=min(1.,dot)
        if dot>.9995: q=(1-a)*self.q+a*b
        else:
            w=math.acos(dot); q=(math.sin((1-a)*w)*self.q+math.sin(a*w)*b)/math.sin(w)
        self.q=self.unit(q)
        return self.lp.update(vector(xyz,3)),self.q.copy()
