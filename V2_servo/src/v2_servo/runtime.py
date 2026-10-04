"""周期层无 IK、模型推理或反馈查询。时间源始终 monotonic。"""
import json, os, time, threading, queue, gc, sys
from pathlib import Path
import numpy as np
from .safety import SafetyFault


class CommissioningPythonScheduling:
    """仅用于独占SDK的短时试验子进程；不改变GUI或其它进程。

    自动循环GC可能扫描已保留的审计对象，默认5ms线程交接间隔也大于
    4ms下发保护预算。启动前收集，短时试验内延后循环GC并申请1ms交接。
    引用计数仍工作；这不是硬实时保证，原迟到/反馈/SDK保护全部保留。
    调用者必须先停止机器人、结束SDK线程，再close恢复；不能运动中collect。
    只能包裹现有H1/H2/H3有限时试验，不能用于无限遥操作循环。
    """
    def __init__(self):
        self.active=False;self.original_gc=None;self.original_switch=None
        self.started=None;self.before_stats=None;self.window_stats=None
    def start(self):
        if self.active:raise RuntimeError('commissioning Python scheduling already active')
        self.original_gc=gc.isenabled();self.original_switch=sys.getswitchinterval()
        self.before_stats=gc.get_stats()
        # 此时还没有机器人登录/使能伺服，不把收集停顿带入运动。
        collected=gc.collect()
        try:
            sys.setswitchinterval(.001);gc.disable()
        except BaseException:
            sys.setswitchinterval(self.original_switch)
            if self.original_gc:gc.enable()
            raise
        self.started=time.perf_counter();self.window_stats=gc.get_stats();self.active=True
        return {'policy':'finite_commissioning_child_only','gc_before':self.original_gc,
                'pre_sdk_collected':collected,'automatic_cycle_gc_in_window':False,
                'reference_counting_enabled':True,'switch_interval_s':sys.getswitchinterval(),
                'original_switch_interval_s':self.original_switch,'hard_realtime':False,
                'motion_window_limit_s':32.,'guards_unchanged':True}
    def close(self):
        if not self.active:return None
        elapsed=time.perf_counter()-self.started;after=gc.get_stats()
        # 不主动收集；只在机器人停止/SDK清理后恢复进程原设置。
        sys.setswitchinterval(self.original_switch)
        if self.original_gc:gc.enable()
        else:gc.disable()
        self.active=False
        return {'scope':'commissioning_child_lifetime_not_motion_duration',
                'elapsed_s':elapsed,'gc_original':self.original_gc,
                'gc_restored':gc.isenabled()==self.original_gc,
                'switch_interval_restored':sys.getswitchinterval()==self.original_switch,
                'collections_since_before_start':[
                    b['collections']-a['collections'] for a,b in zip(self.before_stats,after)],
                'collections_in_deferred_window':[
                    b['collections']-a['collections'] for a,b in zip(self.window_stats,after)]}

class WindowsTimerResolution:
    """控制子进程内申请1ms定时等待精度；退出必须配对恢复，不保证硬实时。

    影响wait/sleep唤醒，不修改单调时钟、周期或任何安全超时。
    """
    def __init__(self,winmm=None):
        self.api=winmm;self.active=False
    def start(self):
        if self.active:raise RuntimeError('timer resolution already active')
        if self.api is None:
            if os.name!='nt':return False
            import ctypes
            self.api=ctypes.WinDLL('winmm')
            for name in ('timeBeginPeriod','timeEndPeriod'):
                f=getattr(self.api,name);f.argtypes=[ctypes.c_uint];f.restype=ctypes.c_uint
        result=int(self.api.timeBeginPeriod(1))
        if result:raise SafetyFault('Windows timeBeginPeriod failed: '+str(result))
        self.active=True;return True
    def close(self):
        if self.active:
            result=int(self.api.timeEndPeriod(1));self.active=False
            if result:raise SafetyFault('Windows timeEndPeriod failed: '+str(result))


class WindowsThreadPriority:
    """仅控制调用线程提升一级；不用REALTIME/HIGH进程优先级，退出恢复。"""
    def __init__(self,api=None):self.api=api;self.handle=None;self.original=None;self.active=False
    def start(self):
        if self.active:raise RuntimeError('thread priority already active')
        if self.api is None:
            if os.name!='nt':return False
            import ctypes
            self.api=ctypes.WinDLL('kernel32',use_last_error=True)
            self.api.GetCurrentThread.argtypes=[];self.api.GetCurrentThread.restype=ctypes.c_void_p
            self.api.GetThreadPriority.argtypes=[ctypes.c_void_p];self.api.GetThreadPriority.restype=ctypes.c_int
            self.api.SetThreadPriority.argtypes=[ctypes.c_void_p,ctypes.c_int];self.api.SetThreadPriority.restype=ctypes.c_int
        self.handle=self.api.GetCurrentThread();self.original=int(self.api.GetThreadPriority(self.handle))
        if self.original==0x7fffffff:raise SafetyFault('GetThreadPriority failed')
        if not self.api.SetThreadPriority(self.handle,1):raise SafetyFault('SetThreadPriority above-normal failed')
        self.active=True
        if int(self.api.GetThreadPriority(self.handle))!=1:
            self.close();raise SafetyFault('thread priority readback mismatch')
        return True
    def close(self):
        if self.active:
            if not self.api.SetThreadPriority(self.handle,self.original):raise SafetyFault('restore thread priority failed')
            self.active=False

class LatestSlot:
    """只容纳最新样本；旧样本的源时间不因读取而更新。"""
    def __init__(self): self.lock=threading.Lock(); self.value=None; self.sequence=-1; self.stamp=-float('inf')
    def publish(self,value,sequence,stamp):
        with self.lock:
            if not np.isfinite(stamp) or sequence<=self.sequence or stamp<=self.stamp: raise ValueError('non-monotonic source')
            self.value=value; self.sequence=sequence; self.stamp=stamp
    def read(self):
        with self.lock: return self.value,self.sequence,self.stamp
    def clear(self):
        with self.lock: self.value=None # 序号高水位保留，防止重放。

class JsonLogger:
    """有界异步日志：磁盘慢不能阻塞发送；日志丢失必须成为运行故障。"""
    def __init__(self,path):
        self.q=queue.Queue(maxsize=4096); self.error=None; self.path=Path(path)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.thread=threading.Thread(target=self._write,daemon=True); self.thread.start()
    def _write(self):
        try:
            with self.path.open('x',encoding='utf-8') as f:
                while True:
                    item=self.q.get()
                    if item is None: break
                    if isinstance(item,tuple) and item[0]=='flush':
                        f.flush(); item[1].set(); continue
                    f.write(json.dumps(item,ensure_ascii=False,allow_nan=False)+'\n')
                f.flush(); os.fsync(f.fileno())
        except Exception as e: self.error=str(e)
    def emit(self,**record):
        if self.error: raise SafetyFault('logger_error:'+self.error)
        try: self.q.put_nowait(record)
        except queue.Full: raise SafetyFault('logger_backpressure')
    def close(self):
        if not self.error:
            try: self.q.put(None,timeout=1)
            except queue.Full: self.error='logger_close_backpressure'
        self.thread.join(timeout=2)
        if self.thread.is_alive(): self.error='logger_close_timeout'
        if self.error: raise SafetyFault(self.error)
    def flush(self):
        """仅在停止采样后导出/回放前调用；不进入实时发送循环。"""
        done=threading.Event()
        try: self.q.put(('flush',done),timeout=.5)
        except queue.Full: raise SafetyFault('logger_flush_backpressure')
        if not done.wait(1): raise SafetyFault('logger_flush_timeout')

class Deadline:
    def __init__(self,period=.008,clock=time.perf_counter,sleep=time.sleep,spin_s=0.):
        if not np.isfinite(spin_s) or not 0<=spin_s<=min(.001,period/4):raise ValueError('bounded scheduler spin outside range')
        self.period=period; self.clock=clock; self.sleep=sleep; self.spin_s=spin_s; self.next=clock()
    def wait(self):
        left=self.next-self.clock()
        while left>0:
            # 先睡眠释放GIL，最后最多1ms用单调时钟等候，降低唤醒超调。
            # 迟到仍由原4ms门槛拒绝；Windows仍不是硬实时系统。
            if left>self.spin_s:self.sleep(left-self.spin_s)
            left=self.next-self.clock()
        now=self.clock(); late=now-self.next
        # 迟到后不突发补发；下次至少间隔一个完整周期。
        self.next=now+self.period
        return now,late

def run_stream(driver,guard,commands,feedback,logger,stop_event,duration,path_guard):
    """部署层须提供已批准的路径保护器；本工程 CLI 暂只在模拟驱动调用此结构。

    Python/C 扩展阻塞或进程死锁可能阻止该看门狗运行，因此真机还需独立硬件看门狗。
    """
    timer=Deadline(guard.cfg['period_s']); started=time.perf_counter(); failure=None; stop_result=None
    logger.emit(event='startup',filters=guard.cfg['filters'],scope='driver_integration')
    try:
        # 进入伺服会改变控制模式，必须先验证样本和路径，不能等使能后才发现缺反馈。
        # 独立检查器不推进正式发送历史；使能返回后仍重新检查新鲜度。
        if stop_event.is_set(): raise SafetyFault('operator_stop')
        motion,seq,stamp=commands.read(); fb,_,_=feedback.read()
        if motion is None or fb is None: raise SafetyFault('missing_sample')
        preflight=type(guard)(guard.cfg); preflight.ready(); preflight.arm()
        preflight.check(motion,fb,time.perf_counter(),stamp,seq,collision_clear=path_guard(motion,fb))
        driver.start(); guard.ready(); guard.arm()
        while time.perf_counter()-started<duration:
            now,late=timer.wait()
            if stop_event.is_set(): raise SafetyFault('operator_stop')
            motion,seq,stamp=commands.read(); fb,_,_=feedback.read()
            if motion is None or fb is None: raise SafetyFault('missing_sample')
            guard.check(motion,fb,now,stamp,seq,lateness=late,collision_clear=path_guard(motion,fb))
            elapsed=driver.send(motion)
            logger.emit(event='send_accepted',t=now,q=motion.q.tolist(),q_actual=np.asarray(fb.q).tolist(),
                        input_at=stamp,feedback_at=fb.sampled_at,call_s=elapsed,lateness_s=late)
    except BaseException as e:
        failure=str(e); guard.state=type(guard.state).FAULT; guard.reason=failure
    finally:
        try: stop_result=driver.stop()
        except Exception as e:
            stop_result={'physical_stop_confirmed':False,'errors':[str(e)]}
            if failure is None: failure='stop_failed:'+str(e)
        commands.clear(); feedback.clear()
        if failure is None: guard.stop(confirmed=stop_result.get('physical_stop_confirmed',False))
        else: guard.state=type(guard.state).FAULT; guard.reason=failure
        logger.emit(event='stopped',failure=failure,stop_result=stop_result)
    return {'failure':failure,'stop_result':stop_result}
