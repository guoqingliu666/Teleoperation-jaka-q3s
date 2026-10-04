"""GUI 外的周期进程：输入槽 → 轨迹 → 安全层 → 驱动 → 实际反馈 → GUI。"""
import json,queue,time,threading,os
from pathlib import Path
from collections import deque
import numpy as np
from ..configuration import ROOT,load
from ..filters import TelemetryFilter
from ..runtime import Deadline,JsonLogger
from ..safety import Guard,State,SafetyFault
from ..trajectory import Motion
from .motion import VelocityServo
from .backends import SimBackend,LiveBackend

class TeleopEngine:
    def __init__(self,channels,mode,options,session):
        self.ch=channels; self.mode=mode; self.cfg=load(); self.options=options; self.session=Path(session)
        self.backend=(SimBackend if mode=='sim' else LiveBackend)(self.cfg,options)
        self.act_tap=None
        if os.environ.get('V2_ACT_TELEMETRY')=='1':
            from ..data_collection.telemetry import CommandTap
            self.act_tap=CommandTap()
        self.backend.interrupted=lambda: self.ch.stop.is_set() or self.ch.estop.is_set() or self.ch.shutdown.is_set()
        self.state='DISCONNECTED'; self.alarm=''; self.active=False; self.recording=False; self.recorded=0
        self.replay=None; self.replay_index=0; self.replay_start=None; self.await_release=True; self.motion=None
        self.logger=JsonLogger(self.session/'events.jsonl'); self.logger.emit(event='startup',mode=mode,filters=self.cfg['filters'],period_s=.008)
        self.command=None; self.telemetry=None; self.filtered=None; self.fb=None; self.tcp=None; self.last_fb_seq=None
        self.last_fb_stamp=None; self.last_status=0.; self.last_tick=None; self.intervals=deque(maxlen=1250); self.lateness=deque(maxlen=1250)
        self.call_ms=0.; self.samples=0; self.last_stop=None; self.servo_space='joint'; self.last_seq=-1; self.last_input_stamp=0.
        self.servo_source='keyboard'; self.stop_epoch=0.; self.reschedule=False
        self.deployment_blockers=[]
        self.stop_file=ROOT/'logs/STOP.request'; self.stop_file_seen=self.stop_file.stat().st_mtime_ns if self.stop_file.exists() else 0
        self.monitor_stop=threading.Event(); self.monitor_error=None
        self.monitor=threading.Thread(target=self.monitor_stop_file,daemon=True); self.monitor.start()
    def monitor_stop_file(self):
        # 文件系统访问不得占据 8 ms 的发送循环；主停止通道仍为独立 Event。
        while not self.monitor_stop.wait(.04):
            try:
                if self.stop_file.exists():
                    modified=self.stop_file.stat().st_mtime_ns
                    if modified!=self.stop_file_seen: self.stop_file_seen=modified; self.ch.stop.set()
            except OSError as e: self.monitor_error=str(e); self.ch.stop.set(); return
    def stop(self,reason='operator_stop',fault=False,estop=False):
        try: self.last_stop=self.backend.stop()
        except Exception as e: self.last_stop={'physical_stop_confirmed':False,'error':str(e)}; fault=True
        if self.last_stop.get('errors') or self.last_stop.get('servo_exited') is False: fault=True
        self.active=False; self.replay=None; self.await_release=True; self.recording=False
        self.stop_epoch=time.perf_counter()
        self.state='ESTOP' if estop or self.state=='ESTOP' else ('FAULT' if fault or self.state=='FAULT' else ('STOPPED' if self.backend.connected else 'DISCONNECTED'))
        self.alarm=reason; self.logger.emit(event='stop',reason=reason,state=self.state,result=self.last_stop)
    def reset(self):
        if self.active: raise RuntimeError('停止伺服后才能复位')
        self.backend.reset(); self.state='CONNECTED' if self.backend.connected else 'DISCONNECTED'; self.alarm=''; self.await_release=True
        self.ch.stop.clear(); self.ch.estop.clear(); self.logger.emit(event='reset',physical_alarm_cleared_by_V2=False)
    def start(self):
        if self.active: raise RuntimeError('已经在伺服模式，禁止重建运动状态')
        if self.monitor_error: raise RuntimeError('停止文件监视失败：'+self.monitor_error)
        if self.state in ('FAULT','ESTOP'): raise RuntimeError('故障锁定：先停止并复位')
        if not self.backend.connected: raise RuntimeError('请先连接')
        now=time.perf_counter(); cmd=self.ch.read()
        if cmd is None or now-cmd['stamp']>self.cfg['safety']['input_timeout_s']: raise RuntimeError('GUI心跳未就绪')
        if cmd['deadman'] or any(abs(v)>0 for v in cmd['axes']): raise RuntimeError('先松开全部 jog 键，再进入伺服')
        fb,tcp=self.backend.sample(now)
        if fb is None: raise RuntimeError('缺反馈')
        pre=Guard(self.cfg); pre.ready(); pre.arm()
        pre.check(Motion.rest(fb.q),fb,time.perf_counter(),cmd['stamp'],cmd['sequence'],command_period=.008)
        self.motion=VelocityServo(fb.q,self.cfg); self.guard=Guard(self.cfg); self.guard.ready(); self.guard.arm()
        self.telemetry=TelemetryFilter(self.cfg['filters'],.008,fb.q); self.filtered=None
        try: self.backend.start(cmd['space'],cmd['source'])
        except Exception:
            self.stop('servo_start_failed',fault=True); raise
        if self.mode=='live' and cmd['space']=='cartesian': self.motion.arm=self.backend.cartesian_kinematics
        self.active=True; self.state='SERVO'; self.alarm=''; self.reschedule=True
        self.servo_source=cmd['source']
        self.servo_space=cmd['space']; self.command=cmd; self.last_seq=cmd['sequence']; self.last_input_stamp=cmd['stamp']; self.await_release=False
        self.logger.emit(event='servo_started',q=fb.q.tolist(),space=cmd['space'],source=cmd['source'],filters=self.cfg['filters'])
    def operate(self,op):
        if time.perf_counter()-op['stamp']>2: raise RuntimeError('过期操作已丢弃')
        if op['stamp']<=self.stop_epoch: raise RuntimeError('停止前排队的操作已丢弃，请重新点击')
        name=op['name']
        if name=='connect':
            if self.backend.connected: return
            if 'host' in op:
                import ipaddress
                self.options['host']=str(ipaddress.ip_address(op['host']))
            self.backend.connect()
            if self.state not in ('FAULT','ESTOP'): self.state='CONNECTED'; self.alarm=''
        elif name=='disconnect':
            self.stop(); self.backend.disconnect()
            if self.state not in ('FAULT','ESTOP'): self.state='DISCONNECTED'
        elif name=='power':
            if op['on'] and self.state in ('FAULT','ESTOP'): raise RuntimeError('故障锁定，禁止上电')
            if self.active:
                if op['on']: raise RuntimeError('先停止伺服')
                self.stop()
            self.backend.power(bool(op['on']))
        elif name=='enable':
            if op['on'] and self.state in ('FAULT','ESTOP'): raise RuntimeError('故障锁定，先复位')
            if self.active: self.stop()
            self.backend.enable(bool(op['on']))
            if self.state not in ('FAULT','ESTOP'): self.state='ENABLED' if op['on'] else 'CONNECTED'
        elif name=='diagnose':
            if self.active: raise RuntimeError('先停止伺服，再运行文件/SDK诊断')
            self.deployment_blockers=[]
            if self.mode=='sim': self.deployment_blockers=['当前SIM；打开真机窗口才可能控制实机']
            else:
                cmd=self.ch.read() or {'space':'joint','source':'keyboard'}
                try: self.backend.review.verify(cmd['space'],cmd['source'])
                except Exception as e: self.deployment_blockers.append(str(e))
                if self.backend.connected: self.backend._poll()
                if self.options.get('feedback_transport')=='edg_mode1':
                    if self.backend.edg is None: self.deployment_blockers.append('EDG模式1仅在部署审查通过后进入；当前未激活')
                elif self.backend.stream_fb is None: self.deployment_blockers.append('10000独立反馈未收到样本')
            self.logger.emit(event='diagnostics',blockers=self.deployment_blockers)
        elif name=='start': self.start()
        elif name=='reset': self.reset()
        elif name=='record_start':
            if not self.active: raise RuntimeError('先进入伺服后录制')
            self.recording=True; self.recorded=0
        elif name=='record_stop': self.recording=False
        elif name=='replay':
            if self.active: raise RuntimeError('先停止，再载入回放')
            self.logger.flush(); path=Path(op['path']).resolve()
            if path.stat().st_size>100_000_000: raise ValueError('回放文件过大')
            records=[]
            with path.open(encoding='utf-8') as f:
                for line in f:
                    row=json.loads(line)
                    if row.get('event')=='servo_sample' and row.get('recording'): records.append(row)
            if len(records)<2: raise ValueError('文件没有可回放录制')
            cmd=self.ch.read()
            if cmd is None or any(r['input']['space']!=cmd['space'] for r in records): raise ValueError('回放坐标模式必须与录制相同')
            times=[r['t'] for r in records]
            if not np.isfinite(times).all() or any(b<=a for a,b in zip(times,times[1:])): raise ValueError('回放时间戳异常')
            fb,_=self.backend.sample(time.perf_counter())
            if fb is None or np.max(np.abs(fb.q-np.array(records[0]['q_actual'])))>np.deg2rad(.5): raise ValueError('请先 jog 回录制起点附近（≤0.5°），禁止自动跳回起点')
            if any(r['input']['source']=='vr' for r in records): raise ValueError('VR示教已保存；VR回放需独立映射校验，当前仅键盘录制可回放')
            self.start(); self.replay=records; self.replay_index=0; self.replay_start=None; self.state='REPLAY'; self.await_release=True
        elif name=='inject_stale' and self.mode=='sim': self.backend.inject_stale=True
        else: raise ValueError('unknown operation '+name)
        self.logger.emit(event='operation',name=name,state=self.state)
    def tick(self,now,late):
        self.lateness.append(late)
        if self.last_tick is not None: self.intervals.append(now-self.last_tick)
        self.last_tick=now
        if self.ch.estop.is_set(): self.ch.estop.clear(); self.stop('软件急停锁定；物理急停独立',estop=True)
        elif self.ch.stop.is_set(): self.ch.stop.clear(); self.stop()
        for _ in range(4):
            try: op=self.ch.operations.get_nowait()
            except queue.Empty: break
            try: self.operate(op)
            except Exception as e:
                self.alarm=str(e); self.logger.emit(event='operation_rejected',name=op['name'],reason=self.alarm)
        now=time.perf_counter(); self.fb,self.tcp=self.backend.sample(now)
        if self.fb is not None and self.fb.sequence!=self.last_fb_seq:
            if self.telemetry is not None:
                dt=.008 if self.last_fb_stamp is None else self.fb.sampled_at-self.last_fb_stamp
                if dt>0: self.filtered=self.telemetry.update(self.fb.q,self.fb.torque,dt)
            self.last_fb_seq=self.fb.sequence; self.last_fb_stamp=self.fb.sampled_at
        if self.active:
            cmd=self.ch.read()
            if cmd is not None: self.command=cmd
            cmd=self.command
            now=time.perf_counter()
            if cmd is None or not 0<=now-cmd['stamp']<=self.cfg['safety']['input_timeout_s']: raise SafetyFault('GUI/input heartbeat timeout')
            if cmd['sequence']<self.last_seq or cmd['stamp']<self.last_input_stamp: raise SafetyFault('input reordered')
            if cmd['sequence']==self.last_seq and cmd['stamp']!=self.last_input_stamp: raise SafetyFault('input retimestamped')
            self.last_seq=cmd['sequence']; self.last_input_stamp=cmd['stamp']
            if cmd['space']!=self.servo_space: raise SafetyFault('停止伺服后再切换关节/笛卡尔模式')
            if cmd['source']!=self.servo_source: raise SafetyFault('停止伺服后再切换输入来源')
            if cmd['hz']!=self.motion.filter.c['command_hz']:
                self.motion.set_cutoff(cmd['hz']); self.logger.emit(event='filter_tuning',command_hz=cmd['hz'])
            axes=cmd['axes']; speed=cmd['speed']; acceleration=cmd['acceleration']; hold=not cmd['deadman']
            if self.replay is not None:
                if not cmd['deadman']: self.await_release=False
                if cmd['deadman'] and not self.await_release:
                    if self.replay_start is None: self.replay_start=now
                    elapsed=now-self.replay_start
                    while self.replay_index+1<len(self.replay) and self.replay[self.replay_index+1]['t']-self.replay[0]['t']<=elapsed: self.replay_index+=1
                    rec=self.replay[self.replay_index]; axes=rec['input']['axes']; speed=rec['input']['speed']; acceleration=rec['input']['acceleration']; hold=not rec['input']['deadman']
                    self.motion.set_cutoff(rec['input']['hz'])
                    if self.replay_index==len(self.replay)-1:
                        # 末尾不无缝切回仍按着键的实时输入，避免回放结束后继续走。
                        self.logger.emit(event='replay_finished'); self.stop('回放完成，请松键后重新进入伺服'); return
                else:
                    if self.replay_start is not None: self.stop('回放使能键释放'); return
                    hold=True; axes=[0]*6
            motion=self.motion.step(axes,cmd['space'],speed,acceleration,hold=hold)
            if self.fb is None: raise SafetyFault('feedback_missing')
            self.guard.check(motion,self.fb,now,cmd['stamp'],cmd['sequence'],lateness=late,command_period=.008)
            issued_ns=time.perf_counter_ns()
            self.call_ms=self.backend.send(motion)*1000; self.samples+=1
            returned_ns=time.perf_counter_ns()
            if self.act_tap is not None:self.act_tap.offer(self.samples,self.mode,motion,self.fb,issued_ns,returned_ns)
            if self.recording:
                self.recorded+=1
                if self.recorded>75000: raise SafetyFault('recording limit: 10 minutes per segment')
            self.logger.emit(event='servo_sample',t=now,q=motion.q.tolist(),v=motion.v.tolist(),a=motion.a.tolist(),j=motion.j.tolist(),
                             q_actual=self.fb.q.tolist(),feedback_at=self.fb.sampled_at,input=cmd,recording=self.recording,
                             lateness_ms=late*1000,call_ms=self.call_ms,mode=self.mode)
        if now-self.last_status>=.04:
            self.last_status=now; self.ch.emit(self.snapshot(now))
    def snapshot(self,now):
        fb=self.fb; motion=None if self.motion is None else self.motion.state
        try: depth=self.ch.operations.qsize()
        except (NotImplementedError,OSError): depth=None
        return {'state':self.state,'mode':self.mode,'alarm':self.alarm,'connected':self.backend.connected,'active':self.active,
            'queue_depth':depth,'deployment_blockers':self.deployment_blockers,'sdk_version':getattr(self.backend,'sdk_version','SIM' if self.mode=='sim' else '未加载'),
            'controller_servo':self.backend.active if self.mode=='sim' else self.backend.controller_servo,
            'feedback_source':'SIM' if self.mode=='sim' else ('EDG模式1' if self.backend.edg is not None and self.backend.edg.entered else ('10000' if self.backend.stream_fb is not None else '同步只读SDK（非实时）')),
            'feedback_receiver_error':None if self.mode=='sim' else self.backend.feedback_error,
            'estop':None if fb is None else fb.estop,'collision':None if fb is None else fb.collision,'soft_limit':None if fb is None else fb.soft_limit,
            'powered':False if fb is None else fb.powered,'enabled':False if fb is None else fb.enabled,
            'q':None if fb is None else fb.q.tolist(),'q_command':None if motion is None else motion.q.tolist(),
            'velocity':None if self.filtered is None else self.filtered['velocity_filtered'].tolist(),
            'torque':None if self.filtered is None or self.filtered['torque_filtered'] is None else self.filtered['torque_filtered'].tolist(),
            'tcp':None if self.tcp is None else self.tcp.tolist(),'tcp_scope':'model Link_06, not calibrated TCP' if self.mode=='sim' else 'controller TCP',
            'tracking_error_deg':None if fb is None or motion is None else float(np.max(np.abs(fb.q-motion.q))*180/np.pi),
            'period_ms':None if not self.intervals else float(np.mean(self.intervals)*1000),
            'jitter_rms_ms':None if not self.intervals else float(np.sqrt(np.mean((np.array(self.intervals)-.008)**2))*1000),
            'late_max_ms':max(self.lateness,default=0)*1000,'deadline_misses':sum(x>.004 for x in self.lateness),
            'feedback_age_ms':None if fb is None else (now-fb.sampled_at)*1000,'call_ms':self.call_ms,'samples':self.samples,
            'recording':self.recording,'recorded':self.recorded,'session':str(self.session),'worker_pid':__import__('os').getpid(),
            'physical_stop_confirmed':None if self.last_stop is None else self.last_stop.get('physical_stop_confirmed'),
            'input_age_ms':None if self.command is None else (now-self.command['stamp'])*1000,
            'filter_hz':self.cfg['filters']['command_hz'] if self.motion is None else self.motion.filter.c['command_hz']}
    def close(self):
        self.monitor_stop.set(); self.monitor.join(.2)
        if self.act_tap is not None:self.act_tap.close()
        try: self.stop('window_shutdown'); self.backend.disconnect()
        finally: self.logger.close()

def control_process(channels,mode,options,session):
    engine=None
    try:
        engine=TeleopEngine(channels,mode,options,session); timer=Deadline(.008)
        while not channels.shutdown.is_set():
            now,late=timer.wait()
            try:
                engine.tick(now,late)
                if engine.reschedule: timer.next=time.perf_counter()+.008; engine.reschedule=False
            except Exception as e:
                engine.stop(str(e),fault=True); channels.emit(engine.snapshot(time.perf_counter()))
    except Exception as e: channels.emit({'state':'FAILED','alarm':str(e),'mode':mode})
    finally:
        if engine is not None:
            try: engine.close()
            except Exception as e: channels.emit({'state':'FAILED','alarm':'cleanup: '+str(e),'mode':mode})
