"""启动：python gui/main.py。默认仿真；显式 --live 创建真实连接界面。"""
import argparse,json,multiprocessing as mp,os,sys,time,uuid
from pathlib import Path
import tkinter as tk
from tkinter import ttk,filedialog,messagebox
import numpy as np
import subprocess
from ..configuration import ROOT,load
from ..runtime_paths import load_runtime_options
from ..teleop.ipc import Channels
from ..teleop.engine import control_process
from ..teleop.quest import QuestInput
from ..teleop.twin_publish import TwinPublisher

class TeleopWindow:
    def __init__(self,root,mode='sim',options=None,session=None):
        self.root=root; self.mode=mode; self.options=options or load_runtime_options()
        self.cfg=load()
        self.session=Path(session or ROOT/'data'/('teleop_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])); self.session.mkdir(parents=True,exist_ok=False)
        self.ctx=mp.get_context('spawn'); self.ch=Channels(self.ctx); self.snapshot={}; self.seq=0; self.keys=set(); self.mouse=None
        self.closing=False; self.quest=None; self.current_space='joint'; self.last_alarm=''; self.buttons={}; self.key_release_jobs={}
        self.root.title('V2 伺服遥操作 | '+('仿真闭环' if mode=='sim' else 'JAKA S5 真机'))
        self.root.geometry('1180x1000'); self.root.minsize(1060,950)
        self.root.configure(bg='#edf2f6'); style=ttk.Style(); style.theme_use('clam')
        style.configure('.',font=('Microsoft YaHei UI',10)); style.configure('TButton',padding=(8,6)); style.configure('TLabelframe.Label',font=('Microsoft YaHei UI',10,'bold'))
        style.configure('TFrame',background='#edf2f6'); style.configure('TLabel',background='#edf2f6')
        style.configure('Danger.TButton',foreground='white',background='#a93339',font=('Microsoft YaHei UI',12,'bold'))
        style.configure('Title.TLabel',foreground='#123647',font=('Microsoft YaHei UI',19,'bold'))
        self.space=tk.StringVar(value='关节'); self.source=tk.StringVar(value='键盘 / 鼠标')
        self.speed=tk.DoubleVar(value=self.options['joint_speed_scale']*100); self.accel=tk.DoubleVar(value=self.options['acceleration_scale']*100); self.hz=tk.DoubleVar(value=self.cfg['filters']['command_hz'])
        self.filter_on=tk.BooleanVar(value=True); self.state_text=tk.StringVar(value='控制进程启动中…'); self.feedback_text=tk.StringVar(value='等待反馈')
        self.metrics_text=tk.StringVar(value='周期 —   抖动 —   跟踪误差 —'); self.alarm_text=tk.StringVar(value='')
        self.vr_text=tk.StringVar(value='VR 未监听'); self.record_text=tk.StringVar(value='未录制')
        self._build()
        self.twin=TwinPublisher()
        self.process=self.ctx.Process(target=control_process,args=(self.ch,mode,self.options,str(self.session)),name='V2-servo-owner',daemon=False); self.process.start()
        self.root.bind('<KeyPress>',self.key_down); self.root.bind('<KeyRelease>',self.key_up)
        self.root.bind('<FocusOut>',self.focus_out); self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.root.after(20,self.heartbeat); self.root.after(40,self.poll)
    def button(self,parent,key,text,operation=None,**kwargs):
        b=ttk.Button(parent,text=text,command=operation or (lambda:self.send(key)),**kwargs); self.buttons[key]=b; return b
    def _build(self):
        outer=ttk.Frame(self.root,padding=16); outer.pack(fill='both',expand=True)
        header=ttk.Frame(outer); header.pack(fill='x')
        ttk.Label(header,text='V2 伺服遥操作',style='Title.TLabel').pack(side='left')
        ttk.Label(header,text='SIM · 仿真驱动闭环' if self.mode=='sim' else 'LIVE · 左侧 S5 / 192.168.110.224',foreground='#167787' if self.mode=='sim' else '#a93339').pack(side='right')
        ttk.Label(outer,text='按住操作，松手减速。Esc / 一键停止 / Quest右摇杆点击为软件停止；控制柜物理急停独立。').pack(anchor='w',pady=(3,10))
        connect=ttk.LabelFrame(outer,text='连接与模式',padding=8); connect.pack(fill='x')
        actions=[('connect','连接',None),('disconnect','断开',None),('power_on','上电',lambda:self.send('power',on=True)),
                 ('power_off','下电',lambda:self.send('power',on=False)),('enable','使能',lambda:self.send('enable',on=True)),('disable','下使能',lambda:self.send('enable',on=False)),
                 ('start','进入伺服',None),('reset','复位软件锁',None)]
        for key,label,action in actions: self.button(connect,key,label,action,width=10 if key=='reset' else 7).pack(side='left',padx=3)
        self.button(connect,'stop','一键停止',self.stop).pack(side='left',padx=6)
        self.button(connect,'estop','软件急停 / 锁定',self.estop,style='Danger.TButton').pack(side='right',padx=3)
        ttk.Label(outer,textvariable=self.state_text,font=('Microsoft YaHei UI',11,'bold'),foreground='#18576b').pack(anchor='w',pady=7)
        ttk.Label(outer,text='工作空间：完整配置关节域，保留硬限位/奇异/现场安全边界；没有起点小球限制。',font=('Microsoft YaHei UI',9)).pack(side='bottom',anchor='w',pady=(8,0))
        switch=ttk.Frame(outer); switch.pack(fill='x')
        ttk.Label(switch,text='当前：仿真窗口，不能驱动实机' if self.mode=='sim' else 'Quest单轴请先断开，再点右侧入口；READY输入在新控制台。普通6DoF未放行。',foreground='#a93339').pack(side='left')
        ttk.Button(switch,text='打开真机窗口' if self.mode=='sim' else '打开仿真窗口',command=self.open_other_mode).pack(side='right')
        if self.mode=='live':
            ttk.Button(switch,text='Quest位移单轴 / READY入口',command=lambda:self.diagnostics.start_quest_j1()).pack(side='right',padx=6)
        self.tabs=ttk.Notebook(outer); self.tabs.pack(fill='both',expand=True)
        control_tab=ttk.Frame(self.tabs); diagnostic_tab=ttk.Frame(self.tabs); self.tabs.add(control_tab,text='遥操作'); self.tabs.add(diagnostic_tab,text='诊断 / IP / ZED相机')
        from .diagnostics import DiagnosticPanel
        self.diagnostics=DiagnosticPanel(self,diagnostic_tab)
        body=ttk.Frame(control_tab); body.pack(fill='both',expand=True); body.columnconfigure(0,weight=5); body.columnconfigure(1,weight=6); body.rowconfigure(0,weight=1)
        left=ttk.Frame(body); left.grid(row=0,column=0,sticky='nsew',padx=(0,12)); right=ttk.Frame(body); right.grid(row=0,column=1,sticky='nsew')
        jog=ttk.LabelFrame(left,text='连续伺服 jog',padding=10); jog.pack(fill='x')
        row=ttk.Frame(jog); row.pack(fill='x')
        self.space_combo=ttk.Combobox(row,textvariable=self.space,values=['关节','笛卡尔'],state='readonly',width=10); self.space_combo.pack(side='left')
        self.space_combo.bind('<<ComboboxSelected>>',self.change_space)
        self.source_combo=ttk.Combobox(row,textvariable=self.source,values=['键盘 / 鼠标','Quest VR'],state='readonly',width=14); self.source_combo.pack(side='left',padx=8)
        self.source_combo.bind('<<ComboboxSelected>>',self.change_space)
        self.axis_labels=[]
        keys=[('1','Q'),('2','W'),('3','E'),('4','R'),('5','T'),('6','Y')]
        for i,(plus,minus) in enumerate(keys):
            line=ttk.Frame(jog); line.pack(fill='x',pady=3)
            label=ttk.Label(line,text=f'J{i+1}',width=7); label.pack(side='left'); self.axis_labels.append(label)
            for direction,text in [(-1,f'−   {minus}'),(1,f'+   {plus}')]:
                b=ttk.Button(line,text=text,width=13); b.pack(side='left',padx=5)
                b.bind('<ButtonPress-1>',lambda e,a=i,d=direction:self.mouse_down(a,d))
                b.bind('<ButtonRelease-1>',self.mouse_up); b.bind('<Leave>',self.mouse_up)
                self.buttons[f'jog_{i}_{direction}']=b
        ttk.Label(jog,text='键盘：按住 Space + 上述键；鼠标：按住按钮。\n笛卡尔为基坐标系 XYZ / RxRyRz。',font=('Microsoft YaHei UI',9)).pack(anchor='w',pady=5)
        tuning=ttk.LabelFrame(left,text='速度与默认滤波',padding=9); tuning.pack(fill='x',pady=9)
        for label,var in [('速度缩放 %',self.speed),('加速度缩放 %',self.accel)]:
            line=ttk.Frame(tuning); line.pack(fill='x',pady=3); ttk.Label(line,text=label,width=15).pack(side='left')
            ttk.Spinbox(line,from_=1,to=100,textvariable=var,width=8).pack(side='left'); ttk.Label(line,text='1–100%').pack(side='left',padx=6)
        line=ttk.Frame(tuning); line.pack(fill='x',pady=3)
        ttk.Checkbutton(line,text='滤波总开关：强制开启',variable=self.filter_on,command=self.enforce_filter).pack(side='left')
        ttk.Spinbox(line,from_=.1,to=30,increment=.5,textvariable=self.hz,width=7).pack(side='left'); ttk.Label(line,text='Hz 指令低通').pack(side='left')
        fc=self.cfg['filters']
        ttk.Label(tuning,text=f"均值 {fc['moving_average']} 点 · 速度 {fc['velocity_hz']:g} Hz · 加速度 {fc['acceleration_hz']:g} Hz · 力矩 {fc['torque_hz']:g} Hz\n未知力矩显示 —；关闭滤波仅离线对比脚本支持。",font=('Microsoft YaHei UI',9)).pack(anchor='w',pady=4)
        vr=ttk.Frame(left); vr.pack(fill='x'); self.button(vr,'vr_listen','监听 Quest UDP',self.start_vr).pack(side='left'); ttk.Label(vr,textvariable=self.vr_text,font=('Microsoft YaHei UI',9)).pack(side='left',padx=8)
        status=ttk.LabelFrame(right,text='机器人反馈',padding=9); status.pack(fill='x')
        self.table=ttk.Treeview(status,columns=('joint','q','v','tau'),show='headings',height=6)
        for col,label,width in [('joint','轴',48),('q','位置 °',112),('v','速度 °/s',112),('tau','力矩 Nm',105)]: self.table.heading(col,text=label); self.table.column(col,width=width,anchor='center')
        for i in range(6): self.table.insert('','end',iid=str(i),values=(f'J{i+1}','—','—','—'))
        self.table.pack(fill='x'); ttk.Label(status,textvariable=self.feedback_text,wraplength=510).pack(anchor='w',pady=8)
        ttk.Label(status,textvariable=self.metrics_text,wraplength=510,font=('Microsoft YaHei UI',9)).pack(anchor='w')
        self.canvas=tk.Canvas(right,height=105,bg='#ffffff',highlightthickness=1,highlightbackground='#d6e1e8'); self.canvas.pack(fill='x',pady=9); self.trace=[]
        record=ttk.LabelFrame(right,text='示教 / 回放 / 日志',padding=8); record.pack(fill='x')
        line=ttk.Frame(record); line.pack(fill='x')
        self.button(line,'record_start','开始录制').pack(side='left',padx=2); self.button(line,'record_stop','结束录制').pack(side='left',padx=2)
        self.button(line,'replay','载入回放',self.load_replay).pack(side='left',padx=2); self.button(line,'open_logs','打开日志',self.open_logs).pack(side='left',padx=2)
        ttk.Label(record,textvariable=self.record_text,font=('Microsoft YaHei UI',9)).pack(anchor='w',pady=5)
        ttk.Label(record,text='回放需回到录制起点附近，载入后持续按住 Space。\n任意停止、失焦或输入超时均结束回放。',font=('Microsoft YaHei UI',9)).pack(anchor='w')
        alarms=ttk.LabelFrame(right,text='报警与操作记录',padding=8); alarms.pack(fill='both',expand=True,pady=(9,0))
        ttk.Label(alarms,textvariable=self.alarm_text,foreground='#a93339',wraplength=510).pack(anchor='w')
        self.log=tk.Text(alarms,height=6,wrap='word',font=('Microsoft YaHei UI',9),bg='#ffffff',relief='flat'); self.log.pack(fill='both',expand=True,pady=4)
    def note(self,text):
        self.log.insert('end',time.strftime('%H:%M:%S')+'  '+text+'\n'); self.log.see('end')
    def send(self,name,**kwargs):
        try:
            if name=='connect': kwargs['host']=self.diagnostics.host()
            self.ch.operation(name,**kwargs); self.note('操作：'+name)
        except Exception as e: self.alarm_text.set(str(e))
    def open_other_mode(self):
        if self.snapshot.get('connected'): self.alarm_text.set('先停止并断开当前连接，再打开另一模式'); return
        subprocess.Popen([sys.executable,str(ROOT/'gui/main.py'),'--live' if self.mode=='sim' else '--sim'],cwd=ROOT,creationflags=0x08000000 if os.name=='nt' else 0)
    def stop(self): self.keys.clear(); self.mouse=None; self.ch.stop.set()
    def estop(self): self.keys.clear(); self.mouse=None; self.ch.estop.set()
    def enforce_filter(self): self.filter_on.set(True); self.note('伺服链强制滤波开启；旁路仅限离线对比。')
    def change_space(self,event=None):
        self.stop()
        if self.source.get()=='Quest VR': self.space.set('笛卡尔')
        self.current_space='joint' if self.space.get()=='关节' else 'cartesian'
        for i,label in enumerate(self.axis_labels): label.configure(text=f'J{i+1}' if self.current_space=='joint' else ['X','Y','Z','Rx','Ry','Rz'][i])
    def key_down(self,event):
        key=event.keysym.lower()
        if key=='escape': self.stop(); return 'break'
        if isinstance(self.root.focus_get(),(ttk.Entry,ttk.Spinbox,tk.Text)): return
        if key in self.key_release_jobs:
            self.root.after_cancel(self.key_release_jobs.pop(key))
        self.keys.add(key)
        if key=='space': return 'break'
    def key_up(self,event):
        key=event.keysym.lower()
        # Windows 自动重复会生成合成 release/press，短延迟合并；真正松手最多延后 5 ms。
        self.key_release_jobs[key]=self.root.after(5,lambda k=key:(self.keys.discard(k),self.key_release_jobs.pop(k,None)))
        if key=='space': return 'break'
    def mouse_down(self,axis,direction): self.mouse=(axis,direction); self.root.focus_set()
    def mouse_up(self,event=None): self.mouse=None
    def focus_out(self,event):
        self.root.after(10,self._check_focus)
    def _check_focus(self):
        if self.closing or self.root.focus_displayof() is not None: return
        # 佩戴 Quest 时 Windows 窗口通常失焦；VR 的 Grip/追踪/输入超时在独立
        # 控制进程持续生效。清掉本地键鼠，避免残留；只有键鼠模式需失焦停机。
        self.keys.clear(); self.mouse=None
        if self.source.get()!='Quest VR': self.stop()
    def heartbeat(self):
        if self.closing: return
        if getattr(self,'pause_heartbeat_for_test',False): self.root.after(20,self.heartbeat); return
        self.seq+=1; axes=np.zeros(6); deadman=False; stamp=None; source='keyboard'
        if self.mouse is not None: axes[self.mouse[0]]=self.mouse[1]; deadman=True
        elif 'space' in self.keys:
            deadman=True
            for i,(plus,minus) in enumerate(zip('123456','qwerty')): axes[i]=int(plus in self.keys)-int(minus in self.keys)
        if self.source.get()=='Quest VR':
            source='vr'; axes[:]=0; deadman=False
            if self.quest and self.quest.take_software_stop():
                self.stop(); self.note('右摇杆点击：软件停止；物理急停独立')
            if self.quest and self.quest.latest:
                frame=self.quest.latest; axes=frame['axes']; deadman=frame['deadman']; stamp=frame['stamp']; self.vr_text.set(frame['state'])
            else: stamp=0.; self.vr_text.set(self.quest.error if self.quest else '请先监听')
        try:
            speed=float(self.speed.get())/100; acceleration=float(self.accel.get())/100; hz=float(self.hz.get())
            if not .01<=speed<=1 or not .01<=acceleration<=1 or not .1<=hz<=30: raise ValueError('速度/加速度需1–100%，低通需0.1–30Hz')
            self.ch.publish(self.seq,axes.tolist(),self.current_space,deadman,speed,acceleration,hz,stamp,source)
        except Exception as e: self.ch.stop.set(); self.alarm_text.set(str(e))
        self.root.after(20,self.heartbeat)
    def poll(self):
        if self.closing: return
        value=self.ch.drain_status()
        if value:
            self.snapshot=value; self.state_text.set(f"{value.get('state')}  ·  上电 {value.get('powered',False)}  ·  使能 {value.get('enabled',False)}  ·  已发 {value.get('samples',0)} 帧  ·  控制进程 {value.get('worker_pid','—')}")
            for i in range(6):
                fmt=lambda field,scale: '—' if value.get(field) is None else f'{value[field][i]*scale:.3f}'
                self.table.item(str(i),values=(f'J{i+1}',fmt('q',180/np.pi),fmt('velocity',180/np.pi),fmt('torque',1)))
            tcp=value.get('tcp'); text='TCP：—' if tcp is None else 'XYZ mm: '+', '.join(f'{x:.2f}' for x in tcp[:3])+'\nRPY °: '+', '.join(f'{x*180/np.pi:.2f}' for x in tcp[3:])
            if self.mode=='sim': text+='\n仿真位置为模型 Link_06 原点，未冒充实测 TCP。'
            self.feedback_text.set(text)
            f=lambda key:'—' if value.get(key) is None else f'{value[key]:.3f}'
            if value.get('active'):
                self.metrics_text.set(f"周期 {f('period_ms')} ms  抖动RMS {f('jitter_rms_ms')} ms\n跟踪最大误差 {f('tracking_error_deg')}°  反馈年龄 {f('feedback_age_ms')} ms\n迟到 >4ms: {value.get('deadline_misses',0)}  SDK调用 {f('call_ms')} ms")
            else:
                self.metrics_text.set(f"未进入伺服：周期/抖动/跟踪误差待测\n反馈来源：{value.get('feedback_source','—')} · 反馈年龄 {f('feedback_age_ms')} ms（只读显示）")
            self.record_text.set(('录制中' if value.get('recording') else '录制停止')+f" · {value.get('recorded',0)} 帧 · {self.session.name}")
            alarm=value.get('alarm',''); self.alarm_text.set(alarm)
            if alarm and alarm!=self.last_alarm: self.note(alarm)
            self.last_alarm=alarm
            self.diagnostics.update(value)
            self.twin.offer(value)
            if value.get('q') is not None:
                self.trace.append(value['q'][0]*180/np.pi); self.trace=self.trace[-200:]; self.draw_trace()
        if not self.process.is_alive(): self.alarm_text.set('控制进程已退出；真机请确认物理停稳。')
        self.root.after(40,self.poll)
    def draw_trace(self):
        c=self.canvas; c.delete('all'); w=max(400,c.winfo_width()); h=c.winfo_height(); c.create_text(8,10,text='J1 实际反馈 / 最近约 8 秒',anchor='w',fill='#45606e')
        if len(self.trace)<2: return
        lo=min(self.trace)-.1; hi=max(self.trace)+.1
        pts=[]
        for i,y in enumerate(self.trace): pts.extend([8+(w-16)*i/(len(self.trace)-1),h-8-(h-30)*(y-lo)/(hi-lo)])
        c.create_line(pts,fill='#168799',width=2)
    def start_vr(self):
        try:
            if self.quest: self.quest.close(); self.quest=None; self.vr_text.set('VR 已停止'); self.stop(); return
            self.quest=QuestInput(self.options['vr_port'],self.options['vr_basis']); self.vr_text.set(f"127.0.0.1:{self.options['vr_port']} 等待数据")
        except Exception as e: self.alarm_text.set(str(e))
    def load_replay(self):
        if self.snapshot.get('active'): self.alarm_text.set('请先停止伺服再载入回放'); return
        path=filedialog.askopenfilename(parent=self.root,initialdir=self.session,title='选择录制 events.jsonl',filetypes=[('V2录制','*.jsonl')])
        if path: self.send('replay',path=path)
    def open_logs(self): os.startfile(str(self.session))
    def close(self):
        if self.closing: return
        self.stop(); self.ch.shutdown.set(); self.closing=True
        if self.quest: self.quest.close()
        self.twin.close()
        self.close_started=time.perf_counter(); self.wait_close()
    def wait_close(self):
        if not self.process.is_alive(): self.process.join(); self.root.destroy(); return
        if time.perf_counter()-self.close_started>3:
            self.state_text.set('停止请求已发；SDK进程未返回。请用物理急停确认停稳。')
            # 不通过强杀真实 SDK 所有者来伪装安全退出。
            if self.mode=='sim': self.process.terminate(); self.process.join(); self.root.destroy(); return
        self.root.after(100,self.wait_close)

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__); modes=p.add_mutually_exclusive_group(); modes.add_argument('--live',action='store_true'); modes.add_argument('--sim',action='store_true')
    p.add_argument('--self-test',action='store_true'); p.add_argument('--session',type=Path); args=p.parse_args(argv)
    if args.self_test and args.live: p.error('GUI test is simulation-only')
    root=tk.Tk(); app=TeleopWindow(root,'live' if args.live else 'sim',session=args.session)
    if args.self_test:
        from .selftest import GuiCheck
        GuiCheck(app).start()
    root.mainloop(); return 1 if args.self_test and not getattr(app,'test_passed',False) else 0

if __name__=='__main__': mp.freeze_support(); raise SystemExit(main())
