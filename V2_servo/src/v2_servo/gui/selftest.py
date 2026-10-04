"""实际 Tk 窗口与独立伺服进程验收；仅允许仿真驱动。"""
import json,os,time,socket
from pathlib import Path
import numpy as np
from ..configuration import ROOT

class GuiCheck:
    def __init__(self,app):
        self.a=app; self.root=app.root; self.steps=[]; self.results=[]; self.deadline=0.; self.index=0
        self.q0=None; self.qpeak=None; self.samples0=0
    def start(self):
        a=self.a
        def click(key): a.buttons[key].invoke()
        def held(on,negative=False):
            self.root.focus_force()
            for key in ('space','q' if negative else '1'):
                self.root.event_generate('<KeyPress>' if on else '<KeyRelease>',keysym=key)
        def origin(): self.q0=np.array(a.snapshot['q']); self.samples0=a.snapshot['samples']; click('record_start')
        def peak():
            self.qpeak=np.array(a.snapshot['q']); assert self.qpeak[0]-self.q0[0]>.03,'keyboard did not jog'; held(False)
        def stopped():
            assert abs(a.snapshot['velocity'][0])<.01,'release did not decelerate'
            assert a.snapshot['worker_pid']!=os.getpid(),'GUI and servo share process'
        def replay(): a.send('replay',path=str(a.session/'events.jsonl'))
        def return_start():
            a.speed.set(1); self.return_negative=a.snapshot['q'][0]>self.q0[0]; held(True,self.return_negative)
        def return_release(): held(False,self.return_negative)
        def check_replayed():
            a.keys.clear(); assert a.snapshot['samples']>self.samples0+650,'too few replay sends'
            self.require(a.snapshot['state']=='STOPPED','replay end did not stop')
        def screenshot():
            from .capture import capture_window
            capture_window(self.root,a.session/'gui.png')
        def cartesian():
            self.cart0=np.array(a.snapshot['tcp']); a.space.set('笛卡尔'); a.change_space()
        def cart_check():
            held(False); self.require(a.snapshot['tcp'][0]-self.cart0[0]>5,'Cartesian X jog did not move model')
        def watchdog(): a.pause_heartbeat_for_test=True
        def watchdog_reset(): a.pause_heartbeat_for_test=False; click('reset')
        def vr_begin():
            self.vr_grip=0.; self.vr_z=0.; self.vr_seq=0; self.vr_sending=True; self.vr_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            a.source.set('Quest VR'); a.change_space(); a.start_vr()
            self.require(a.quest is not None,'Quest UDP listener failed'); self.vr_address=a.quest.socket.getsockname()
            self.vr_origin=np.array(a.snapshot['tcp']); self.vr_packet()
        def vr_grip(): self.vr_grip=1.
        def vr_release():
            self.require(a.snapshot['tcp'][0]-self.vr_origin[0]>1.,'loopback VR velocity did not move model X')
            self.vr_grip=0.
        def vr_end():
            self.vr_sending=False; self.vr_socket.close(); a.start_vr(); a.source.set('键盘 / 鼠标'); a.change_space()
        # 每一步都是窗口按钮/键盘操作，反馈由另一个进程回传。
        self.steps=[('worker_ready',lambda:None,lambda:a.snapshot.get('state')=='DISCONNECTED',8),
            ('connect',lambda:click('connect'),lambda:a.snapshot.get('connected'),4),
            ('power',lambda:click('power_on'),lambda:a.snapshot.get('powered'),3),
            ('enable',lambda:click('enable'),lambda:a.snapshot.get('enabled'),3),
            ('servo',lambda:click('start'),lambda:a.snapshot.get('active'),3),
            ('record',origin,None,.2),('keyboard_positive',lambda:held(True),None,.9),
            ('positive_feedback',peak,None,.6),('release_stop',stopped,None,.1),
            ('keyboard_negative',lambda:held(True,True),None,.9),('release_negative',lambda:held(False,True),None,.6),
            ('record_end',lambda:click('record_stop'),None,.15),
            ('jog_to_replay_origin',return_start,lambda:abs(a.snapshot['q'][0]-self.q0[0])<.0015,5),
            ('origin_release',return_release,None,.3),('restore_speed',lambda:a.speed.set(10),None,.1),
            ('stop',lambda:click('stop'),lambda:a.snapshot.get('state')=='STOPPED',3),
            ('replay_load',replay,lambda:a.snapshot.get('state')=='REPLAY',3),('replay_release',lambda:held(False),None,.15),
            ('replay_hold',lambda:held(True),lambda:a.snapshot.get('state')=='STOPPED',7),('replay_result',check_replayed,None,.1),
            ('stop_after_replay',lambda:click('stop'),lambda:a.snapshot.get('state')=='STOPPED',3),
            ('estop',lambda:click('estop'),lambda:a.snapshot.get('state')=='ESTOP',3),
            ('estop_blocks_start',lambda:click('start'),None,.2),
            ('check_lock',lambda:self.require(a.snapshot.get('state')=='ESTOP','estop lock bypassed'),None,.1),
            ('reset',lambda:click('reset'),lambda:a.snapshot.get('state')=='CONNECTED',3),
            ('cartesian_mode',cartesian,None,.2),('cartesian_start',lambda:click('start'),lambda:a.snapshot.get('active'),3),
            ('cartesian_keyboard',lambda:held(True),None,.6),('cartesian_feedback',cart_check,None,.6),
            ('watchdog_pause',watchdog,lambda:a.snapshot.get('state')=='FAULT',2),
            ('watchdog_reset',watchdog_reset,lambda:a.snapshot.get('state')=='CONNECTED',3),
            ('vr_loopback_open',vr_begin,None,.3),('vr_servo',lambda:click('start'),lambda:a.snapshot.get('active'),3),
            ('vr_grip_velocity',vr_grip,None,.9),('vr_release',vr_release,None,.5),
            ('vr_stop',lambda:click('stop'),lambda:a.snapshot.get('state')=='STOPPED',3),('vr_close',vr_end,None,.2),
            ('screenshot',screenshot,None,.2)]
        self.root.after(200,self.next)
    def vr_packet(self):
        if not self.vr_sending or self.a.closing: return
        self.vr_seq+=1
        if self.vr_grip>.7: self.vr_z+=.001
        packet={'version':2,'sequence':self.vr_seq,'unity_time_s':time.perf_counter(),
                'right':{'connected':True,'tracked':True,'pose_valid':True,'position_m':[0,0,self.vr_z],
                         'rotation_xyzw':[0,0,0,1],'grip':self.vr_grip,'trigger':0}}
        self.vr_socket.sendto(json.dumps(packet).encode(),self.vr_address); self.root.after(20,self.vr_packet)
    def require(self,condition,message):
        if not condition: raise AssertionError(message)
    def next(self):
        if self.index>=len(self.steps): self.finish(True); return
        name,action,predicate,seconds=self.steps[self.index]
        try: action()
        except Exception as e: self.finish(False,str(e)); return
        self.deadline=time.perf_counter()+seconds; self.root.after(40,self.check)
    def check(self):
        name,action,predicate,seconds=self.steps[self.index]; expired=time.perf_counter()>=self.deadline
        if self.a.snapshot.get('state') in ('FAULT','FAILED') and name not in ('watchdog_pause','watchdog_reset'):
            self.finish(False,self.a.snapshot.get('alarm')); return
        if (predicate is None and expired) or (predicate is not None and predicate()):
            self.results.append({'step':name,'passed':True}); self.index+=1; self.root.after(50,self.next)
        elif expired: self.finish(False,'timeout '+name)
        else: self.root.after(40,self.check)
    def finish(self,passed,error=None):
        report={'scope':'actual Tk window + spawned SIM servo process; no hardware motion','passed':passed,'error':error,
                'steps':self.results,'last_status':self.a.snapshot,'initial_q':None if self.q0 is None else self.q0.tolist(),
                'positive_q':None if self.qpeak is None else self.qpeak.tolist()}
        (self.a.session/'gui_test.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'gui_test':passed,'error':error,'session':str(self.a.session)},ensure_ascii=False),flush=True)
        self.a.test_passed=passed; self.a.close()
