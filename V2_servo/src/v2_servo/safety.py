"""所有真实驱动必须经过此层；原始故障不能被滤波抹去。"""
from dataclasses import dataclass
from enum import Enum
import math
import numpy as np
from .configuration import vector

class State(str,Enum):
    DISCONNECTED='DISCONNECTED'
    READY='READY'
    SERVO='SERVO'
    STOPPING='STOPPING'
    STOPPED='STOPPED'
    FAULT='FAULT'

class SafetyFault(RuntimeError): pass

@dataclass(frozen=True)
class Feedback:
    q: object
    sampled_at: float       # 本地主机 perf_counter；采用查询开始时间，禁止用返回时间刷新旧样本。
    sequence: int
    enabled: bool=True
    powered: bool=True
    estop: bool=False
    collision: bool=False
    soft_limit: bool=False
    error: int=0
    torque: object=None
    velocity: object=None
    acceleration: object=None
    controller_timestamp: float|None=None

class Guard:
    def __init__(self,cfg):
        self.cfg=cfg; self.state=State.DISCONNECTED; self.reason=None
        self.last_feedback_seq=-1; self.last_feedback_time=None; self.last_input_seq=-1; self.last_input_time=None
        self.last_command_q=None; self.last_command_time=None; self.last_command_dv=None; self.last_command_da=None
    def ready(self):
        if self.state not in {State.DISCONNECTED,State.STOPPED}: raise SafetyFault('no automatic fault recovery')
        self.state=State.READY
        self.last_command_q=None; self.last_command_time=None; self.last_command_dv=None; self.last_command_da=None
    def arm(self):
        if self.state!=State.READY: raise SafetyFault('not ready')
        if not self.cfg['filters']['enabled']: self.fail('filters_disabled')
        self.state=State.SERVO
    def fail(self,reason):
        self.state=State.FAULT; self.reason=reason; raise SafetyFault(reason)
    def stop(self,confirmed=False):
        if self.state==State.FAULT: return
        self.state=State.STOPPED if confirmed else State.STOPPING
    def check(self,motion,feedback,now,input_at,input_seq,*,deadman=True,lateness=0.,collision_clear=True,command_period=None):
        if self.state!=State.SERVO: self.fail('not_in_servo')
        s=self.cfg['safety']; lim=self.cfg['limits']
        if not all(math.isfinite(t) for t in [now,input_at,feedback.sampled_at,lateness]): self.fail('invalid_time')
        if not deadman: self.fail('deadman_released')
        if not collision_clear: self.fail('path_not_clear')
        if lateness>s['max_lateness_s']: self.fail('deadline_miss')
        for stamp,timeout,label in [(input_at,s['input_timeout_s'],'input'),(feedback.sampled_at,s['feedback_timeout_s'],'feedback')]:
            if now<stamp or now-stamp>timeout: self.fail(label+'_stale_or_future')
        if feedback.sequence<self.last_feedback_seq: self.fail('feedback_reordered')
        if self.last_feedback_time is not None:
            if feedback.sampled_at<self.last_feedback_time: self.fail('feedback_time_reversed')
            if feedback.sequence==self.last_feedback_seq and feedback.sampled_at!=self.last_feedback_time: self.fail('feedback_retimestamped')
        if input_seq<self.last_input_seq: self.fail('input_reordered')
        if self.last_input_time is not None:
            if input_at<self.last_input_time: self.fail('input_time_reversed')
            if input_seq==self.last_input_seq and input_at!=self.last_input_time: self.fail('input_retimestamped')
        self.last_feedback_seq=feedback.sequence; self.last_feedback_time=feedback.sampled_at; self.last_input_seq=input_seq
        self.last_input_time=input_at
        if feedback.estop or feedback.collision or feedback.soft_limit or feedback.error or not feedback.enabled or not feedback.powered:
            self.fail('robot_fault')
        try:
            actual=vector(feedback.q)
            for values,key in [(motion.q,None),(actual,None),(motion.v,'velocity_rad_s'),(motion.a,'acceleration_rad_s2'),(motion.j,'jerk_rad_s3')]:
                values=vector(values)
                if key is None:
                    if np.any(values<lim['lower_rad']) or np.any(values>lim['upper_rad']): self.fail('joint_limit')
                elif np.any(np.abs(values)>np.array(lim[key])+1e-8): self.fail(key)
            if np.max(np.abs(motion.q-actual))>s['tracking_error_rad']: self.fail('tracking_error')
            # 不信任输入对象自报的 v/a/j。独立核查相邻实际下发目标的差分，防止伪造零速度跳点。
            dt=self.cfg['period_s'] if self.last_command_time is None else now-self.last_command_time
            # servo_j step_num=1 的参考序列按 8 ms 插补。主机迟到仍由独立 deadline 门槛检查。
            # 这里检查参考序列导数，不将其宣传为物理反馈导数。
            if command_period is not None:
                if abs(command_period-self.cfg['period_s'])>1e-12: self.fail('invalid_command_period')
                dt=command_period
            if dt<=0: self.fail('command_time_reversed')
            previous=actual if self.last_command_q is None else self.last_command_q
            dv=(motion.q-previous)/dt
            if np.any(np.abs(dv)>np.array(lim['velocity_rad_s'])+1e-8): self.fail('command_step_velocity')
            da=None
            if self.last_command_dv is not None:
                da=(dv-self.last_command_dv)/dt
                if np.any(np.abs(da)>np.array(lim['acceleration_rad_s2'])+1e-8): self.fail('command_step_acceleration')
                if self.last_command_da is not None:
                    dj=(da-self.last_command_da)/dt
                    if np.any(np.abs(dj)>np.array(lim['jerk_rad_s3'])+1e-8): self.fail('command_step_jerk')
            for values,key in [(feedback.velocity,'velocity_rad_s'),(feedback.acceleration,'acceleration_rad_s2')]:
                if values is not None and np.any(np.abs(vector(values))>np.array(lim[key])*s['measured_dynamics_factor']): self.fail('measured_'+key)
            if lim['torque_nm'] is not None:
                if feedback.torque is None: self.fail('torque_missing')
                if np.any(np.abs(vector(feedback.torque))>lim['torque_nm']): self.fail('raw_torque_limit')
        except (TypeError,ValueError): self.fail('invalid_numeric')
        self.last_command_q=motion.q.copy(); self.last_command_time=now; self.last_command_dv=dv; self.last_command_da=da
        return True
