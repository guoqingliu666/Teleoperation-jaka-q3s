"""灵巧手设置窗独立于主界面，防止新增控件挤掉底部 STOP/状态提示。"""
import time
import tkinter as tk
from tkinter import ttk, messagebox
from .dh116_control import adapters, validate_limits


class HandPanel:
    def __init__(self, parent, controller, allowed, latest_frame, live):
        self.controller, self.allowed, self.latest_frame, self.live = controller, allowed, latest_frame, live
        self.window=tk.Toplevel(parent); self.window.title('DH116 右手｜Trigger 开合设置')
        self.window.geometry('880x550'); self.window.minsize(800,510)
        body=ttk.Frame(self.window,padding=12); body.pack(fill='both',expand=True)
        ttk.Label(body,text='右 Grip：机械臂与手的保持许可；右 Trigger：五路弯曲开合，拇指侧摆暂时固定。',wraplength=820).pack(anchor='w')
        ttk.Label(body,text='实物手首次小行程验收；不自动回零，不绕过报警/限位。软件停止不等于物理急停。',foreground='#b42318').pack(anchor='w',pady=5)
        self.adapter_names=[]; self.selection=ttk.Combobox(body,state='readonly',width=95); self.selection.pack(fill='x',pady=5)
        row=ttk.Frame(body); row.pack(fill='x')
        ttk.Button(row,text='① 枚举本机网卡（不连设备）',command=self.scan).pack(side='left')
        ttk.Button(row,text='② 连接手（不自动使能/回零）',command=self.connect).pack(side='left',padx=5)
        ttk.Button(row,text='断开手',command=self.disconnect).pack(side='left')
        self.closure=tk.DoubleVar(value=30.); self.speed=tk.DoubleVar(value=10.); self.current=tk.DoubleVar(value=150.)
        for label,var,low,high,unit in [('闭合行程上限',self.closure,5,50,'%'),('开合速度',self.speed,1,20,'% 行程/秒'),('电流上限',self.current,50,300,'‰ 额定电流（不是夹持力）')]:
            row=ttk.Frame(body); row.pack(fill='x',pady=6)
            ttk.Label(row,text=label,width=18).pack(side='left')
            ttk.Scale(row,from_=low,to=high,variable=var,length=300).pack(side='left')
            ttk.Entry(row,textvariable=var,width=8).pack(side='left',padx=6)
            ttk.Label(row,text=unit).pack(side='left')
        self.site=tk.BooleanVar(value=False)
        ttk.Checkbutton(body,text='已确认手内无人/无物、已按厂家流程回零，LHandPro 已断开，专用网口只接这只右手',variable=self.site).pack(anchor='w',pady=6)
        row=ttk.Frame(body); row.pack(fill='x')
        ttk.Button(row,text='③ 准备手控制（显式使能，不闭合）',command=self.prepare).pack(side='left')
        ttk.Checkbutton(row,text='④ 允许 Trigger 控手',variable=allowed).pack(side='left',padx=10)
        ttk.Button(row,text='停止手 / 撤销手授权',command=self.stop).pack(side='right')
        ttk.Label(body,text='准备时松开 Grip 和 Trigger。主窗口 ARM 后，按住 Grip，再缓慢扣 Trigger 闭合；\n保持 Grip、松 Trigger 张开；松 Grip 停止并保留当前位置。每段重新 Grip 后先松一次 Trigger。\n参数仅在点击“准备手控制”时生效。未连接手时，机械臂仍可独立遥操作。',wraplength=820).pack(anchor='w',pady=12)
        self.text=tk.StringVar(); ttk.Label(body,textvariable=self.text,wraplength=820,foreground='#1d4e89').pack(anchor='w')
        if not live:
            self.selection['values']=['离线模拟手（不会加载硬件驱动）']; self.selection.current(0)
        self.refresh()

    def scan(self):
        try:
            if not self.live: return
            values=adapters(); self.adapter_names=[v[0] for v in values]
            self.selection['values']=[f'{desc} | {name}' for name,desc in values]
            self.selection.set('请选择实际连接灵巧手的专用网口，不要选择 JAKA/相机网口')
        except Exception as exc: messagebox.showerror('枚举失败',str(exc),parent=self.window)

    def connect(self):
        try:
            if self.live:
                if not self.site.get(): raise ValueError('先完成现场确认并断开 LHandPro')
                index=self.selection.current()
                if index<0: raise ValueError('请明确选择灵巧手专用网口')
                self.controller.connect(self.adapter_names[index])
            else: self.controller.connect()
        except Exception as exc: messagebox.showerror('手连接未启动',str(exc),parent=self.window)

    def prepare(self):
        try:
            q=self.latest_frame()
            if q is None or not q.valid or not q.tracked or not q.connected or (time.time_ns()-q.received_time_ns)/1e9>0.2:
                raise ValueError('请佩戴头显、拿起右手柄，等待有效追踪')
            if q.grip>0.1 or q.trigger>0.1: raise ValueError('准备手控制时请松开 Grip 和 Trigger')
            if self.live and not self.site.get(): raise ValueError('现场确认尚未完成')
            limits=validate_limits(self.closure.get(),self.speed.get(),self.current.get())
            self.controller.prepare(limits)
        except Exception as exc: messagebox.showerror('手尚未准备',str(exc),parent=self.window)

    def stop(self): self.allowed.set(False); self.controller.stop()
    def disconnect(self):
        self.stop()
        try: self.controller.close()
        except Exception as exc: messagebox.showerror('手未确认断开',str(exc),parent=self.window)
    def refresh(self):
        if not self.window.winfo_exists(): return
        state=self.controller.snapshot()
        self.text.set(f"{'模拟' if state.get('simulated') else '实物'}｜连接={state.get('connected')} 新鲜反馈={state.get('feedback_valid')} 手授权={state.get('authorized')}\n"
                      +state.get('status','')+'\n六轴实测角度(°)：'+str(state.get('angles_deg','尚无反馈')))
        self.window.after(150,self.refresh)
