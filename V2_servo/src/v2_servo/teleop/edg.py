"""JAKA 1.7.2 EDG 扩展模式1反馈候选；部署审查通过前不可用于运动。"""
import threading
import time

import numpy as np

from ..configuration import vector
from ..jaka import checked
from ..safety import Feedback, SafetyFault


def _flag(value,name):
    if type(value) not in (bool,int) or value not in (0,1):
        raise SafetyFault('invalid EDG status '+name)
    return bool(value)


def decode_status_pair(motion,simple):
    """按官方1.7.2返回顺序，保持原始故障状态不经过滤波。"""
    if len(motion)!=11 or len(simple)!=4 or type(simple[0]) is not int:
        raise SafetyFault('unknown lightweight status schema')
    return {'soft_limit':_flag(motion[8],'soft_limit'),
            'estop':_flag(motion[9],'estop'),
            'collision':_flag(motion[10],'collision'),
            'error':simple[0],
            'powered':_flag(simple[2],'powered'),
            'enabled':_flag(simple[3],'enabled')}


class StaggeredSafetyPoll:
    """两类只读查询错开50ms，各10Hz；原始故障单独拒绝，不等另一类返回。

    时间取查询开始值。分散请求不改变150ms年龄约束，不伪造新的采样时间。
    """
    def __init__(self,rc,emit):
        self.rc=rc;self.emit=emit;self.motion=None;self.simple=None
        self.motion_at=None;self.simple_at=None;self.next_kind='motion'

    def query_one(self):
        kind=self.next_kind;start=time.perf_counter()
        if kind=='motion':
            raw=self.rc.get_motion_status();value=checked(raw,'get_motion_status')
            if not isinstance(value,(tuple,list)) or len(value)!=11:
                raise SafetyFault('unknown motion status schema')
            if any(_flag(value[i],name) for i,name in ((8,'soft_limit'),(9,'estop'),(10,'collision'))):
                raise SafetyFault('raw motion safety fault: '+str(raw))
            self.motion=raw;self.motion_at=start;self.next_kind='simple'
        else:
            raw=self.rc.get_robot_status_simple();value=checked(raw,'get_robot_status_simple')
            if not isinstance(value,(tuple,list)) or len(value)!=4 or type(value[0]) is not int:
                raise SafetyFault('unknown simple status schema')
            if value[0] or not _flag(value[2],'powered') or not _flag(value[3],'enabled'):
                raise SafetyFault('raw simple safety fault: '+str(raw))
            self.simple=raw;self.simple_at=start;self.next_kind='motion'
        returned=time.perf_counter()
        self.emit(event='raw_safety_part',kind=kind,query_start_s=start,query_end_s=returned,raw=raw)
        flags=None;stamp=None
        if self.motion is not None and self.simple is not None:
            flags=decode_status_pair(checked(self.motion,'get_motion_status'),checked(self.simple,'get_robot_status_simple'))
            stamp=min(self.motion_at,self.simple_at)
            self.emit(event='raw_safety',query_start_s=start,query_end_s=returned,
                      oldest_query_start_s=stamp,motion_query_start_s=self.motion_at,
                      simple_query_start_s=self.simple_at,motion=self.motion,simple=self.simple)
        return flags,stamp,max(0.,start+.05-time.perf_counter())


def decode_edg_state(state,details):
    """EDGState: q(rad), dq(rad/s), torque(单位待现场核实), TCP(mm/rad)。"""
    if not isinstance(state,(tuple,list)) or len(state)!=6:
        raise SafetyFault('unknown EDG state schema')
    if not isinstance(details,(tuple,list)) or len(details)!=3:
        raise SafetyFault('unknown EDG timestamp schema')
    q=vector(state[0]); velocity=vector(state[1]); tcp=vector(state[3])
    seconds,nanos,count=details
    if type(seconds) is not int or type(nanos) is not int or type(count) is not int or not 0<=nanos<1_000_000_000:
        raise SafetyFault('invalid EDG source timestamp')
    if not np.isfinite(q).all() or not np.isfinite(velocity).all() or not np.isfinite(tcp).all():
        raise SafetyFault('nonfinite EDG feedback')
    return q,velocity,tcp,seconds+nanos*1e-9,count


class EdgMode1Feedback:
    """只在一个SDK所有者内读取EDG；轻量故障线程不得触碰伺服周期的规划器。"""
    def __init__(self,rc,client_ip,port,source_age_bound_s,status_timeout_s=.15):
        if not 0<=source_age_bound_s<.016:
            raise ValueError('source age bound requires site validation')
        self.rc=rc;self.client_ip=client_ip;self.port=port
        self.age_bound=source_age_bound_s;self.status_timeout=status_timeout_s
        self.entered=False;self.stop_event=threading.Event();self.thread=None
        self.status=None;self.status_at=0.;self.status_error=None;self.lock=threading.Lock()
        self.last_source=None;self.last_fb=None;self.last_tcp=None;self.seq=0

    def start(self):
        if self.entered: raise RuntimeError('EDG already active')
        checked(self.rc.edg_init_extend(True,self.client_ip,self.port,1),'edg_init_extend(true)')
        self.entered=True
        self.thread=threading.Thread(target=self._status_loop,daemon=True,name='JAKA-lightweight-status')
        self.thread.start()
        deadline=time.perf_counter()+.3
        while time.perf_counter()<deadline:
            with self.lock:
                ready=self.status is not None or self.status_error is not None
            if ready: break
            time.sleep(.005)
        with self.lock:
            if self.status_error: raise SafetyFault('lightweight status unavailable: '+self.status_error)
            if self.status is None: raise SafetyFault('lightweight status timeout')

    def _status_loop(self):
        while not self.stop_event.is_set():
            begin=time.perf_counter()
            try:
                motion=checked(self.rc.get_motion_status(),'get_motion_status')
                simple=checked(self.rc.get_robot_status_simple(),'get_robot_status_simple')
                flags=decode_status_pair(motion,simple)
                with self.lock:self.status=flags;self.status_at=begin;self.status_error=None
            except Exception as exc:
                with self.lock:self.status_error=str(exc)
            self.stop_event.wait(max(0.,begin+.1-time.perf_counter()))

    def sample(self,now):
        if not self.entered: raise SafetyFault('EDG not entered')
        with self.lock: flags=self.status;stamp=self.status_at;error=self.status_error
        if error or flags is None or now-stamp>self.status_timeout:
            raise SafetyFault('lightweight safety status stale: '+str(error))
        begin=time.perf_counter()
        state=checked(self.rc.edg_get_stat(0),'edg_get_stat')
        details=checked(self.rc.edg_stat_details(),'edg_stat_details')
        q,velocity,tcp,source,count=decode_edg_state(state,details)
        if self.last_source is not None:
            delta=source-self.last_source
            if delta<0 or delta>.024: raise SafetyFault('EDG source timestamp gap/reversal')
            if delta==0:
                return self.last_fb,self.last_tcp
        self.last_source=source;self.seq+=1
        self.last_fb=Feedback(q,begin-self.age_bound,self.seq,velocity=velocity,
                              controller_timestamp=source,**flags)
        self.last_tcp=tcp
        return self.last_fb,self.last_tcp

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(5)
            if self.thread.is_alive(): raise RuntimeError('EDG status thread did not stop; preserve SDK owner')
        if self.entered:
            checked(self.rc.edg_init_extend(False,self.client_ip,self.port,1),'edg_init_extend(false)')
            self.entered=False
