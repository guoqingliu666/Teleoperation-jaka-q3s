"""②中文界面。Tk负责交互；独立进程串行访问SDK；关闭先STOP并等停止确认。

ASCII文件名用于CMD兼容。窗口每100ms给子进程发心跳，窗口失联后子进程请求停机。
参数启动时锁定，运行中不会因为拖滑杆突然改变速度或范围。
"""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))
from vla_lab.sampled_follow import Settings
from vla_lab.adaptive_sampling import AdaptiveSampling


class EndpointTeleopGui:
    """②的显示与操作层；真正的 SDK 调用只在独立子进程里执行。

    这样分层是为了让界面能持续显示状态并发送心跳。窗口失联时，
    子进程会撤销运动许可；但软件停机仍不能替代控制柜和物理急停。
    """

    def __init__(self, root):
        self.root, self.process = root, None
        self.lines = queue.Queue(maxsize=500)
        self.closing = self.stopping = False
        self.stop_confirmed = None
        self.stop_fault_latched = False
        self.session_fault = False
        self.sdk_wait = None
        self.sdk_stall_warned = False
        root.title("② JAKA 受限连续六维遥操作 | Grip保持、松开停止")
        root.geometry("1080x820")
        # 高DPI中文字体会显著放大控件；默认最大化以保证参数、按钮、日志和安全提示都可见。
        try:
            root.state("zoomed")
        except tk.TclError:
            pass
        root.protocol("WM_DELETE_WINDOW", self.close)
        ttk.Label(root, text="V1.5：动态采样 + JAKA 厂商逆解与深度2圆滑队列",
                  font=("Microsoft YaHei UI", 16, "bold"), foreground="#a02020").pack(anchor="w", padx=16, pady=14)
        ttk.Label(root, text="位置模式保持姿态；六维模式同时跟随位置和姿态。①单独时只读JAKA；②启动后独占连接并显示实测关节。首次与恢复后先松开Grip。").pack(anchor="w", padx=16)
        box = ttk.LabelFrame(root, text="启动时生效的参数")
        box.pack(fill="x", padx=16, pady=12)
        self.controls = []
        self.speed = self.slider(box, 0, "TCP速度", 5, 300, 150, "mm/s")
        self.radius = self.slider(box, 1, "活动半径", 20, 100, 100, "cm（以启动TCP为中心）")
        self.acceleration = self.slider(box, 2, "TCP加速度", 10, 800, 400, "mm/s²")
        self.rotation_radius = self.slider(box, 3, "姿态范围", 10, 30, 30, "°（相对启动姿态）")
        self.orientation_speed = self.slider(box, 4, "姿态速度", 1, 60, 30, "°/s")
        self.segment_mm = self.slider(box, 5, "单条最大位移", 20, 100, 60, "mm（端点采样，不补历史轨迹）")
        self.orientation_step = self.slider(box, 6, "单条最大姿态", 2, 10, 6, "°（段内仍逐点厂商逆解）")
        self.live = tk.BooleanVar(value=False)
        check = ttk.Checkbutton(box, text="受限连续真机跟随（不勾选为只读预览）", variable=self.live)
        check.grid(row=7, column=0, columnspan=4, sticky="w", padx=10, pady=10)
        self.controls.append(check)
        ttk.Label(box, text="已验收入口固定30mm/s、5°/s、10cm/10°；150mm/s全范围入口仍是待验收候选。过载时丢弃未接纳增量，不积压补跑。").grid(row=8, column=0, columnspan=4, sticky="w", padx=10, pady=5)
        bar = ttk.Frame(root); bar.pack(fill="x", padx=16)
        self.start_button = ttk.Button(bar, text="启动 / 重新连接", command=self.start)
        self.start_button.pack(side="left")
        self.comm_button = ttk.Button(bar, text="只读通信检查（30秒，不运动）", command=lambda:self.start(communications_only=True))
        self.comm_button.pack(side="left", padx=12)
        self.controls.append(self.comm_button)
        self.acceptance_button = ttk.Button(bar, text="一次≤5mm真机验收", command=lambda:self.start(one_segment=True))
        self.acceptance_button.pack(side="left", padx=12)
        self.controls.append(self.acceptance_button)
        self.two_segment_button = ttk.Button(bar, text="两段各≤10mm真机验收", command=lambda:self.start(two_segment=True))
        self.two_segment_button.pack(side="left", padx=12)
        self.controls.append(self.two_segment_button)
        self.stop_button = ttk.Button(bar, text="停止会话（请求停止并确认）", command=self.stop_waiting, state="disabled")
        self.stop_button.pack(side="left", padx=12)
        advanced_bar = ttk.Frame(root); advanced_bar.pack(fill="x", padx=16, pady=(6, 0))
        self.five_segment_button = ttk.Button(advanced_bar, text="五段累计≤5cm真机验收（15mm/s）",
                                              command=lambda:self.start(five_segment=True))
        self.five_segment_button.pack(side="left")
        self.controls.append(self.five_segment_button)
        self.ten_segment_button = ttk.Button(advanced_bar, text="十段累计≤10cm位置跟随验收（20mm/s）",
                                             command=lambda:self.start(ten_segment=True))
        self.ten_segment_button.pack(side="left", padx=12)
        self.controls.append(self.ten_segment_button)
        self.twenty_segment_button = ttk.Button(
            advanced_bar, text="二十段累计≤20cm位置跟随验收（30mm/s）",
            command=lambda:self.start(twenty_segment=True))
        self.twenty_segment_button.pack(side="left", padx=12)
        self.controls.append(self.twenty_segment_button)
        shadow_bar = ttk.Frame(root); shadow_bar.pack(fill="x", padx=16, pady=(6, 0))
        self.rotation_shadow_button = ttk.Button(
            shadow_bar, text="姿态只读影子（30秒 / ±10° / 零运动）",
            command=lambda:self.start(rotation_shadow=True))
        self.rotation_shadow_button.pack(side="left")
        self.controls.append(self.rotation_shadow_button)
        self.orientation_acceptance_button = ttk.Button(
            shadow_bar, text="一次≤1°姿态真机验收（1°/s）",
            command=lambda:self.start(orientation_acceptance=True))
        self.orientation_acceptance_button.pack(side="left", padx=12)
        self.controls.append(self.orientation_acceptance_button)
        self.three_orientation_acceptance_button = ttk.Button(
            shadow_bar, text="三段累计≤3°姿态验收（2°/s）",
            command=lambda:self.start(three_orientation_acceptance=True))
        self.three_orientation_acceptance_button.pack(side="left", padx=12)
        self.controls.append(self.three_orientation_acceptance_button)
        faster_orientation_bar = ttk.Frame(root); faster_orientation_bar.pack(fill="x", padx=16, pady=(6, 0))
        self.ten_orientation_acceptance_button = ttk.Button(
            faster_orientation_bar, text="十段累计≤10°姿态验收（5°/s）",
            command=lambda:self.start(ten_orientation_acceptance=True))
        self.ten_orientation_acceptance_button.pack(side="left")
        self.controls.append(self.ten_orientation_acceptance_button)
        self.six_dof_button = ttk.Button(
            faster_orientation_bar,
            text="受限连续六维跟随（位置+姿态）",
            command=lambda:self.start(bounded_six_dof=True))
        self.six_dof_button.pack(side="left", padx=12)
        self.controls.append(self.six_dof_button)
        self.expanded_six_dof_button = ttk.Button(
            faster_orientation_bar,
            text="扩展六维跟随（50cm / 50mm/s / ±30°）",
            command=lambda:self.start(expanded_six_dof=True))
        self.expanded_six_dof_button.pack(side="left", padx=12)
        self.controls.append(self.expanded_six_dof_button)
        production_bar = ttk.Frame(root); production_bar.pack(fill="x", padx=16, pady=(6, 0))
        self.production_six_dof_button = ttk.Button(
            production_bar,
            text="可调正式六维（100cm / 5—300mm/s / ±30°）",
            command=lambda:self.start(production_six_dof=True))
        self.production_six_dof_button.pack(side="left")
        self.controls.append(self.production_six_dof_button)
        self.production_shadow_button = ttk.Button(
            production_bar,
            text="v1.2长段六维只读影子（30秒 / 零运动）",
            command=lambda:self.start(production_shadow=True))
        self.production_shadow_button.pack(side="left", padx=12)
        self.controls.append(self.production_shadow_button)
        self.development_pose_acceptance_button = ttk.Button(
            production_bar,
            text="v1.2一次≤60mm/6°真机验收（150mm/s）",
            command=lambda:self.start(development_pose_acceptance=True))
        self.development_pose_acceptance_button.pack(side="left", padx=12)
        self.controls.append(self.development_pose_acceptance_button)
        self.development_continuous_button = ttk.Button(
            production_bar,
            text="v1.2连续长段六维验收（150mm/s / 60mm / 6°）",
            command=lambda:self.start(development_continuous=True))
        self.development_continuous_button.pack(side="left", padx=12)
        self.controls.append(self.development_continuous_button)
        blend_bar = ttk.Frame(root); blend_bar.pack(fill="x", padx=16, pady=(6, 0))
        self.development_pose_blend_button = ttk.Button(
            blend_bar,
            text="v1.2两段六维厂商圆滑验收（2×30mm/3°）",
            command=lambda:self.start(development_pose_blend=True))
        self.development_pose_blend_button.pack(side="left")
        self.controls.append(self.development_pose_blend_button)
        self.development_hand_pose_blend_button = ttk.Button(
            blend_bar,
            text="v1.2手柄两端点圆滑验收（队列2）",
            command=lambda:self.start(development_hand_pose_blend=True))
        self.development_hand_pose_blend_button.pack(side="left", padx=12)
        self.controls.append(self.development_hand_pose_blend_button)
        self.development_rolling_pose_button = ttk.Button(
            blend_bar,
            text="v1.5扩展滚动六维（100cm / 50段）",
            command=lambda:self.start(development_rolling_pose=True))
        self.development_rolling_pose_button.pack(side="left", padx=12)
        self.controls.append(self.development_rolling_pose_button)
        v15_bar = ttk.Frame(root); v15_bar.pack(fill="x", padx=16, pady=(6, 0))
        self.production_rolling_pose_button = ttk.Button(
            v15_bar,
            text="V1.5正式连续六维（100cm / Grip重复启停）",
            command=lambda:self.start(production_rolling_pose=True))
        self.production_rolling_pose_button.pack(side="left")
        self.controls.append(self.production_rolling_pose_button)
        self.pose_priority_button = ttk.Button(
            v15_bar,
            text="V1.5姿态优先（XYZ≤10cm / 姿态±30°）",
            command=lambda:self.start(production_rolling_pose=True,
                                      pose_priority=True))
        self.pose_priority_button.pack(side="left", padx=12)
        self.controls.append(self.pose_priority_button)
        self.expanded_pose_priority_button = ttk.Button(
            v15_bar,
            text="V1.5扩展姿态优先（XYZ≤30cm / ±45° / 200mm/s）",
            command=lambda:self.start(production_rolling_pose=True,
                                      pose_priority=True,
                                      expanded_pose_priority=True))
        self.expanded_pose_priority_button.pack(side="left", padx=12)
        self.controls.append(self.expanded_pose_priority_button)
        diagnostic_bar = ttk.Frame(root)
        diagnostic_bar.pack(fill="x", padx=16, pady=(6, 0))
        self.trajectory_diagnostic_button = ttk.Button(
            diagnostic_bar, text="新轨迹离线诊断（选日志，不连接机器人）",
            command=self.trajectory_diagnostic)
        self.trajectory_diagnostic_button.pack(side="left")
        self.controls.append(self.trajectory_diagnostic_button)
        self.diagnostic_running = False
        self.readonly_test_active = False
        self.readonly_test_result = None
        self.trajectory_readonly_button = ttk.Button(
            diagnostic_bar,text="新调度只读测试（120秒，不运动）",
            command=self.start_trajectory_readonly)
        self.trajectory_readonly_button.pack(side="left",padx=8)
        self.controls.append(self.trajectory_readonly_button)
        self.medium_pilot_readonly_button = ttk.Button(
            diagnostic_bar,text="中等幅同参数只读预检（60秒，不运动）",
            command=lambda:self.start_trajectory_readonly(medium_pilot=True))
        self.medium_pilot_readonly_button.pack(side="left",padx=8)
        self.controls.append(self.medium_pilot_readonly_button)
        self.trajectory_readonly_stop = ttk.Button(
            diagnostic_bar,text="结束只读测试",command=self.stop_waiting,state="disabled")
        self.trajectory_readonly_stop.pack(side="left")
        adaptive_bar=ttk.Frame(root)
        adaptive_bar.pack(fill="x",padx=16,pady=(6,0))
        ttk.Label(adaptive_bar,text="动态检查最大间隔（与运动段长度不同）：").pack(side="left")
        self.check_mm=tk.StringVar(value="5")
        self.check_deg=tk.StringVar(value="1")
        for variable,unit in ((self.check_mm,"mm"),(self.check_deg,"°")):
            entry=ttk.Entry(adaptive_bar,textvariable=variable,width=6)
            entry.pack(side="left",padx=3)
            ttk.Label(adaptive_bar,text=unit).pack(side="left")
            self.controls.append(entry)
        self.adaptive_live_button=ttk.Button(adaptive_bar,
            text="有界意图六维候选（真机待验收）",
            command=lambda:self.start(production_six_dof=True,production_rolling_pose=True,
                                       adaptive_trajectory=True))
        self.adaptive_live_button.pack(side="left",padx=10)
        self.controls.append(self.adaptive_live_button)
        self.adaptive_pilot_button=ttk.Button(adaptive_bar,
            text="V1.5已验收中等幅跟随（10cm/10°）",
            command=lambda:self.start(production_six_dof=True,production_rolling_pose=True,
                                       adaptive_trajectory=True,adaptive_pilot=True))
        self.adaptive_pilot_button.pack(side="left",padx=10)
        self.controls.append(self.adaptive_pilot_button)
        self.state = tk.StringVar(value="尚未启动；先打开①并确认右手TRACKED")
        ttk.Label(root, textvariable=self.state, wraplength=1020, foreground="#205080").pack(anchor="w", padx=16, pady=10)
        self.intent_status = tk.StringVar(value="新调度：黄色为手柄意图；受限时实际接纳目标可能有偏差，松握重新对齐。")
        ttk.Label(root, textvariable=self.intent_status, wraplength=1020,
                  foreground="#8a4b00").pack(anchor="w", padx=16, pady=(0, 4))
        ttk.Label(root, text="软件停止不替代物理急停。程序不自动上电、使能或清报警。修改参数前先停止会话。", foreground="#8a4b00").pack(anchor="w", padx=16, pady=(0, 6))
        self.log = tk.Text(root, height=22, wrap="word", state="disabled", font=("Consolas", 10))
        self.log.pack(fill="both", expand=True, padx=16, pady=6)
        root.after(100, self.pump)

    def slider(self, parent, row, label, low, high, initial, units):
        """创建一组滑杆和数字框；两者同步，但只在合法数值时更新另一侧。"""
        # 数字框编辑时允许暂时为空；不能与Scale共用DoubleVar，否则清空即Tcl异常。
        value = tk.StringVar(value=str(initial))
        scale_value = tk.DoubleVar(value=initial)
        updating = False
        def from_entry(*_):
            nonlocal updating
            if updating:
                return
            try:
                number = float(value.get())
            except ValueError:
                return
            if math.isfinite(number) and low <= number <= high:
                updating = True
                try:
                    scale_value.set(number)
                finally:
                    updating = False
        def from_scale(*_):
            nonlocal updating
            if updating:
                return
            updating = True
            try:
                value.set(f"{scale_value.get():g}")
            finally:
                updating = False
        value.trace_add("write", from_entry)
        scale_value.trace_add("write", from_scale)
        ttk.Label(parent, text=label, width=15).grid(row=row, column=0, padx=10, sticky="w")
        scale = tk.Scale(parent, from_=low, to=high, resolution=1, orient="horizontal", variable=scale_value, length=470, showvalue=False)
        scale.grid(row=row, column=1, padx=5)
        entry = ttk.Entry(parent, textvariable=value, width=8); entry.grid(row=row, column=2, padx=8)
        ttk.Label(parent, text=units).grid(row=row, column=3, sticky="w")
        self.controls.extend([scale, entry])
        return value

    def append(self, line):
        """追加运行日志，最多保留约 500 行，避免长会话撑满界面内存。"""
        self.log.configure(state="normal")
        self.log.insert("end", line)
        if int(self.log.index("end-1c").split(".")[0]) > 500:
            self.log.delete("1.0", "101.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def trajectory_diagnostic(self):
        """独立离线进程检查参考曲线，不复用真机会话/停止状态或SDK所有权。"""
        if self.diagnostic_running:
            return
        if self.process is not None and self.process.poll() is None:
            messagebox.showinfo("先结束当前会话", "请先正常停止当前会话，再进行离线轨迹诊断。")
            return
        source = filedialog.askopenfilename(
            title="选择需要分析的真实会话日志（不要选.sdk.jsonl）",
            initialdir=str(ROOT.parent / "Validation" / "continuous_sampled_follow"),
            filetypes=[("会话日志", "*.jsonl")])
        if not source:
            return
        self.diagnostic_running = True
        self.append("离线分析启动：只生成参考曲线报告，不连接、不运动。\n")

        def work():
            try:
                env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONDONTWRITEBYTECODE="1",
                           TEMP=r"D:\ChatGPT\Temp", TMP=r"D:\ChatGPT\Temp")
                result = subprocess.run(
                    [sys.executable, "-B", str(ROOT / "轨迹连续性诊断.py"), "--replay", source],
                    cwd=str(ROOT.parent), env=env, capture_output=True, text=True,
                    encoding="utf-8", errors="replace", timeout=120,
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
                self.lines.put(result.stdout + result.stderr)
                self.lines.put(f"离线诊断退出码：{result.returncode}；不是实机连续性验收。\n")
            except Exception as error:
                self.lines.put(f"离线诊断失败：{error}\n")
            finally:
                self.diagnostic_running = False

        threading.Thread(target=work, daemon=True, name="trajectory-offline-report").start()

    def start_trajectory_readonly(self, medium_pilot=False):
        """独立白名单子进程；不读取真机勾选框/运动滑杆，不清除既有故障锁。"""
        if self.readonly_test_active or (self.process is not None and self.process.poll() is None):
            messagebox.showinfo("会话正在运行","请先结束当前会话，再开始只读测试。")
            return
        if self.stop_fault_latched:
            messagebox.showerror("先检查现场","之前的停止/故障状态尚未确认，请先处理现场；只读测试不会清除故障锁。")
            return
        try:
            # 中等幅真机验收的预检参数必须固定，不能被界面输入框改大。
            policy=AdaptiveSampling() if medium_pilot else self.sampling_policy()
        except ValueError as error:
            messagebox.showerror("动态采样参数无效",str(error))
            return
        seconds="60" if medium_pilot else "120"
        args=[sys.executable,"-B","-u",str(ROOT/"只读检查新轨迹.py"),
              "--live-readonly","--seconds",seconds,"--ui-control","--sampling","adaptive",
              "--max-check-mm",str(policy.max_check_mm),"--max-check-deg",str(policy.max_check_deg)]
        if medium_pilot:
            args.append("--medium-pilot")
        env=dict(os.environ,PYTHONIOENCODING="utf-8",PYTHONDONTWRITEBYTECODE="1",
                 TEMP=r"D:\ChatGPT\Temp",TMP=r"D:\ChatGPT\Temp")
        try:
            child=subprocess.Popen(args,cwd=str(ROOT.parent),env=env,stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding="utf-8",errors="replace",
                bufsize=1,creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
        except OSError as error:
            messagebox.showerror("只读测试启动失败",str(error))
            return
        self.process=child
        self.readonly_test_active=True
        self.readonly_test_result=None
        self.stopping=False
        self.sdk_wait=None
        self.start_button.configure(state="disabled")
        for widget in self.controls: widget.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.trajectory_readonly_stop.configure(state="normal")
        self.state.set("中等幅同参数只读预检连接中" if medium_pilot else
                       "只读测试连接中：保持实机静止；先松右Grip，再握住移动/转动；不发送运动。")
        self.append("\n" + ("中等幅同参数只读预检：60秒；先松Grip再握住，移动约5cm；不运动。\n"
                            if medium_pilot else
                            "新轨迹只读测试：120秒自动结束；可随时结束。不运动、不使能、不改参数。\n"))
        threading.Thread(target=self.read_output,args=(child,),daemon=True).start()

    def sampling_policy(self):
        """同一份间隔供只读检查和动态执行使用，启动后参数随控件一起锁定。"""
        return AdaptiveSampling(max_check_mm=float(self.check_mm.get()),
                                max_check_deg=float(self.check_deg.get()))

    def readonly_event(self,event):
        """只读完成与真机停机确认严格分离，不能借此清除/生成运动许可。"""
        state=event.get("state")
        if state=="trajectory_input_status":
            grip=event.get("grip")
            trigger=event.get("trigger")
            values=f"Grip={grip:.2f} / Trigger={trigger:.2f}" if grip is not None and trigger is not None else "等待手柄数据"
            self.state.set(f"只读｜{event.get('message','')}｜{values}｜剩余{event.get('remaining_s','?')}秒")
            planning=event.get("planning") or {}
            if planning.get("scheduler")=="bounded_intent_v1":
                self.show_intent_status(planning)
                scale=planning.get("admission_scale",1.)
                self.state.set(self.state.get()+f"｜缓存{planning.get('buffered_samples',0)}点"
                               +f"｜接纳比例{scale:.0%}｜受限{planning.get('limited_frames',0)}帧")
            else:
                age=planning.get("oldest_received_age_s")
                if age is not None:
                    self.state.set(self.state.get()+f"｜待处理{age*1000:.0f}ms｜{planning.get('pressure','normal')}")
        elif state=="trajectory_readonly_finished":
            self.readonly_test_result=event
        elif state=="process_exit":
            result=self.readonly_test_result
            if result is None:
                message=f"只读进程退出（代码{event.get('code')}），未收到完整报告；不是测试通过。"
            else:
                outcome=result.get("outcome")
                label={"cancelled":"已结束","incomplete":"未完成","failed":"失败",
                       "readonly_candidates_observed":"已取得只读逆解候选（非真机验收）"}.get(outcome,"结果未知")
                message=f"只读测试{label}；Grip窗口{result.get('grip_windows',0)}，候选{result.get('candidates',0)}，阻断{result.get('blocked_windows',0)}。"
                if result.get("failure"): message+=str(result["failure"])
            self.state.set(message)
            self.readonly_test_active=False
            self.stopping=False
            self.process=None
            self.start_button.configure(state="disabled" if self.stop_fault_latched else "normal")
            for widget in self.controls: widget.configure(state="normal")
            self.stop_button.configure(state="disabled")
            self.trajectory_readonly_stop.configure(state="disabled")
            if self.closing: self.root.destroy()
        elif "message" in event:
            self.state.set("只读｜"+str(event["message"]))

    def show_intent_status(self, planning):
        """单独保留接纳状态，运动日志刷新时也能看到当前是否缩小了输入。"""
        scale = planning.get("admission_scale", 1.)
        self.intent_status.set(
            f"接纳比例 {scale:.0%}｜缓存 {planning.get('buffered_samples', 0)} 点"
            f"｜受限 {planning.get('limited_frames', 0)} 帧"
            + ("｜跟随受限：不补追未接纳增量，松Grip后重握可对齐"
               if scale < .999 else "｜当前正常接纳；黄色为原始手柄意图"))

    def start(self, communications_only=False, one_segment=False, two_segment=False,
              five_segment=False, ten_segment=False, twenty_segment=False,
              rotation_shadow=False, orientation_acceptance=False,
              three_orientation_acceptance=False,
              ten_orientation_acceptance=False, bounded_six_dof=False,
              expanded_six_dof=False, production_six_dof=False,
              production_shadow=False, development_pose_acceptance=False,
              development_continuous=False, development_pose_blend=False,
              development_hand_pose_blend=False,
              development_rolling_pose=False,
              production_rolling_pose=False, pose_priority=False,
              expanded_pose_priority=False, adaptive_trajectory=False,
              adaptive_pilot=False):
        """启动一个独立会话；真机验收按钮使用固定限值，普通按钮仅预览。"""
        if self.readonly_test_active:
            return
        if self.stop_fault_latched:
            messagebox.showerror("停止未确认", "本窗口禁止重新启动。请先在现场检查停止状态和故障记录，不要反复启动真机。")
            return
        if self.process is not None and self.process.poll() is None:
            return
        try:
            # 真机验收档锁定采样密度；界面输入框不能悄悄改变此次验收含义。
            dynamic_policy=(AdaptiveSampling() if adaptive_pilot else
                            self.sampling_policy() if adaptive_trajectory else None)
            # 受限真机档位使用代码中的固定限值，不采用界面滑杆。
            # 滑杆只在普通只读预览时生效，避免误以为拖动滑杆就能扩大真机许可。
            # 正式真机和同参数只读影子共用一套参数解析，避免“预览通过的不是将要执行的参数”。
            if (production_six_dof or production_shadow or development_pose_acceptance
                    or development_continuous or development_pose_blend
                    or development_hand_pose_blend or development_rolling_pose
                    or production_rolling_pose):
                if adaptive_pilot:
                    settings = Settings(speed_mm_s=30,radius_mm=200,
                                        acceleration_mm_s2=60,segment_mm=20,deadband_mm=3)
                    rotation_radius,orientation_speed,orientation_step=10.,5.,2.
                elif (development_pose_acceptance or development_continuous
                        or development_pose_blend or development_hand_pose_blend
                        or development_rolling_pose or production_rolling_pose):
                    # 已通过单条门槛和下一关连续长段使用同一固定参数，避免滑杆改变验收含义。
                    settings = Settings(
                        speed_mm_s=200 if expanded_pose_priority else 150,
                        radius_mm=1000,
                        acceleration_mm_s2=500 if expanded_pose_priority else 400,
                        segment_mm=80 if expanded_pose_priority else 60,
                        deadband_mm=3)
                    rotation_radius = 45.0 if expanded_pose_priority else 30.0
                    orientation_speed = 45.0 if expanded_pose_priority else 30.0
                    # 姿态优先档把每个厂商队列段再缩到3°，但累计姿态仍可到±30°。
                    # 这样既不是绝对锁位，也不会把一次较大的手腕转动塞进单个逆解段。
                    orientation_step = (4.0 if expanded_pose_priority else
                                        3.0 if pose_priority else 6.0)
                else:
                    settings = Settings(speed_mm_s=float(self.speed.get()),
                                        radius_mm=float(self.radius.get()) * 10,
                                        acceleration_mm_s2=float(self.acceleration.get()),
                                        segment_mm=float(self.segment_mm.get()),deadband_mm=3)
                    rotation_radius = float(self.rotation_radius.get())
                    orientation_speed = float(self.orientation_speed.get())
                    orientation_step = float(self.orientation_step.get())
                if not 200 <= settings.radius_mm <= 1000:
                    raise ValueError("正式六维活动半径必须为20—100cm")
                if not 10 <= rotation_radius <= 45:
                    raise ValueError("正式六维姿态范围必须为10—45°")
                if not 1 <= orientation_speed <= 60:
                    raise ValueError("正式六维姿态速度必须为1—60°/s")
                if not 20 <= settings.segment_mm <= 100:
                    raise ValueError("正式六维单条最大位移必须为20—100mm")
                if not 2 <= orientation_step <= 10:
                    raise ValueError("正式六维单条最大姿态必须为2—10°")
            elif expanded_six_dof:
                settings = Settings(radius_mm=500,speed_mm_s=50,acceleration_mm_s2=100,
                                    segment_mm=20,deadband_mm=3)
            elif bounded_six_dof:
                settings = Settings(radius_mm=200,speed_mm_s=30,acceleration_mm_s2=60,
                                    segment_mm=10,deadband_mm=3)
            elif orientation_acceptance or three_orientation_acceptance or ten_orientation_acceptance:
                settings = Settings(radius_mm=20, speed_mm_s=5, acceleration_mm_s2=10,
                                    segment_mm=5, deadband_mm=3)
            elif one_segment:
                settings = Settings(radius_mm=20, speed_mm_s=5, acceleration_mm_s2=10,
                                    segment_mm=5, deadband_mm=3)
            elif two_segment:
                settings = Settings(radius_mm=30, speed_mm_s=10, acceleration_mm_s2=20,
                                    segment_mm=10, deadband_mm=3)
            elif five_segment:
                settings = Settings(radius_mm=60, speed_mm_s=15, acceleration_mm_s2=30,
                                    segment_mm=10, deadband_mm=3)
            elif ten_segment:
                settings = Settings(radius_mm=100, speed_mm_s=20, acceleration_mm_s2=40,
                                    segment_mm=10, deadband_mm=3)
            elif twenty_segment:
                settings = Settings(radius_mm=200, speed_mm_s=30, acceleration_mm_s2=60,
                                    segment_mm=10, deadband_mm=3)
            else:
                settings = Settings(speed_mm_s=float(self.speed.get()),
                                    radius_mm=float(self.radius.get()) * 10,
                                    acceleration_mm_s2=float(self.acceleration.get()))
        except (ValueError, TypeError, tk.TclError) as error:
            messagebox.showerror("参数无效", str(error))
            return
        if one_segment and not messagebox.askyesno(
            "真机单段现场确认",
            "本次最多下发一条≤5mm直线，速度5mm/s、加速度10mm/s²、半径2cm，保持TCP姿态，30秒超时。\n\n"
            "请确认机器人与整条臂的扫掠空间已清空，人员在外，急停可立即触及；"
            "JAKA App无报警、Tool 1和用户坐标系0正确；先松开Grip。\n\n"
            "本程序不自动上电/使能，也不能替代物理急停。是否进入等待Grip的会话？"):
            return
        if two_segment and not messagebox.askyesno(
            "两段真机现场确认",
            "本次最多下发两条、每条≤10mm的直线；10mm/s、20mm/s²、以启动TCP为中心半径3cm，保持TCP姿态，45秒超时。\n\n"
            "请确认整条机械臂扫掠空间已清空、人员在外、急停可立即触及；JAKA App无报警且机器人静止，Tool 1/用户坐标系0正确。"
            "先松开Grip，再缓慢移动手柄约2cm并持续握持，自动完成两段后结束。\n\n"
            "如有任何抖动、异响或异常，立即松Grip并视现场情况使用物理急停。是否启动？"):
            return
        if five_segment and not messagebox.askyesno(
            "五段真机现场确认",
            "本次最多五条、每条≤10mm直线，指令累计行程≤5cm；15mm/s、30mm/s²、"
            "以启动TCP为中心半径6cm，保持TCP姿态，90秒超时。\n\n"
            "请确认整条机械臂及灵巧手的扫掠空间清空、人员在外、急停可触及；"
            "JAKA App无报警且机器人静止，Tool 1/用户坐标系0正确。先松Grip，再握住沿已验收的安全方向"
            "缓慢移动手柄约5cm并持续握持，自动完成五段后结束。\n\n"
            "若出现抖动、异响、反向或报警，立即松Grip并按现场规程处理，必要时使用物理急停。是否启动？"):
            return
        if ten_segment and not messagebox.askyesno(
            "十段位置跟随现场确认",
            "本次最多十条、每条≤10mm直线，指令累计行程≤10cm；20mm/s、40mm/s²、"
            "以启动TCP为中心半径10cm，保持TCP姿态，120秒超时。\n\n"
            "请确认机械臂、灵巧手及整条扫掠空间清空，人员在外，急停可立即触及；"
            "JAKA App无报警且机器人静止，Tool 1/用户坐标系0正确。先松开Grip，再握住缓慢连续移动；"
            "程序只采用最新目标，不补跑手柄历史轨迹。\n\n"
            "若出现抖动、异响、反向、明显停顿异常或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if twenty_segment and not messagebox.askyesno(
            "二十段位置跟随现场确认",
            "本次最多二十条、每条≤10mm直线，指令累计行程≤20cm；30mm/s、60mm/s²、"
            "以启动TCP为中心半径20cm，保持TCP姿态，180秒超时。\n\n"
            "请确认机械臂、灵巧手及整条扫掠空间清空，人员在外，急停可立即触及；"
            "JAKA App无报警且机器人静止，Tool 1/用户坐标系0正确。先松开Grip，再握住连续移动；"
            "程序只采用最新目标，不补跑手柄历史轨迹。\n\n"
            "若出现抖动、异响、反向、异常大幅运动或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if orientation_acceptance and not messagebox.askyesno(
            "一次≤1°姿态真机现场确认",
            "本次XYZ位置锁定，最多下发一条≤1°姿态命令；姿态速度1°/s、加速度2°/s²，30秒超时。\n\n"
            "请确认机械臂与灵巧手整条扫掠空间清空、人员在外、急停可立即触及；JAKA App无报警且机械臂静止，"
            "Tool 1/用户坐标系0正确。先松Grip，再握住并缓慢转动约0.5—1°，命令实测到位后自动结束。\n\n"
            "若出现反向、抖动、异响、位置漂移或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if three_orientation_acceptance and not messagebox.askyesno(
            "三段累计≤3°姿态真机现场确认",
            "本次XYZ位置锁定，最多下发三条、每条≤1°的姿态命令；累计≤3°，姿态速度2°/s、"
            "加速度4°/s²，45秒超时。\n\n"
            "请确认机械臂与灵巧手整条扫掠空间清空、人员在外、急停可立即触及；JAKA App无报警且机械臂静止，"
            "Tool 1/用户坐标系0正确。先松Grip，再握住并缓慢连续转动约2—3°，三段实测到位后自动结束。\n\n"
            "若出现反向、抖动、异响、位置漂移、明显停顿异常或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if ten_orientation_acceptance and not messagebox.askyesno(
            "十段累计≤10°姿态真机现场确认",
            "本次XYZ位置锁定，最多下发十条、每条≤1°的姿态命令；累计≤10°，姿态速度5°/s、"
            "加速度10°/s²，90秒超时。手柄跳动不会直接放大成机器人命令。\n\n"
            "请确认机械臂与灵巧手整条扫掠空间清空、人员在外、急停可立即触及；JAKA App无报警且机械臂静止，"
            "Tool 1/用户坐标系0正确。先松Grip，再握住并平稳转动约8—10°后保持，逐段实测到位后自动结束。\n\n"
            "若出现反向、抖动、异响、位置漂移、异常大幅关节运动或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if bounded_six_dof and not messagebox.askyesno(
            "受限连续六维真机确认",
            f"本次位置与姿态同时跟随：启动TCP周围半径{settings.radius_mm/10:g}cm、"
            f"TCP速度{settings.speed_mm_s:g}mm/s、加速度{settings.acceleration_mm_s2:g}mm/s²；"
            "姿态限制在会话初始姿态±10°，5°/s、10°/s²。每条命令≤10mm且≤1°，最长10分钟。\n\n"
            "请确认机械臂、灵巧手及整个工作球的扫掠空间清空，人员在外，急停可立即触及；"
            "JAKA App无报警且机器人静止，Tool 1/用户坐标系0正确。先松Grip，再握住缓慢做位置和姿态组合运动。\n\n"
            "任何反向、抖动、异响、异常大幅关节运动、位置漂移或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if expanded_six_dof and not messagebox.askyesno(
            "扩展六维真机确认",
            "本次位置与姿态同时跟随：启动TCP周围半径50cm、TCP速度50mm/s、加速度100mm/s²；"
            "姿态限制在会话初始姿态±30°，10°/s、20°/s²。每条命令≤20mm且≤2°，最长10分钟。\n\n"
            "请确认机械臂、灵巧手及50cm工作球对应的整条扫掠空间清空，人员在外，急停可立即触及；"
            "JAKA App无报警且机器人静止，Tool 1/用户坐标系0正确。先松Grip，再握住做连续位置和姿态组合运动。\n\n"
            "任何反向、抖动、异响、异常大幅关节运动或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if production_six_dof and not adaptive_pilot and not messagebox.askyesno(
            "正式可调六维真机确认",
            f"本次位置与姿态同时跟随：启动TCP周围半径{settings.radius_mm/10:g}cm、"
            f"TCP速度{settings.speed_mm_s:g}mm/s、加速度{settings.acceleration_mm_s2:g}mm/s²；"
            f"姿态范围±{rotation_radius:g}°、速度{orientation_speed:g}°/s。"
            f"每条命令最多{settings.segment_mm:g}mm / {orientation_step:g}°，最长10分钟。\n\n"
            "100cm是软件包络，不表示每个点都可达；不可达路径由厂商逆解拒绝。"
            "请确认设定工作球及整条机械臂扫掠空间清空，人员在外，急停可立即触及；"
            "JAKA App无报警且机器人静止，Tool 1/用户坐标系0正确。先松Grip，再握住操作。\n\n"
            "任何反向、抖动、异响、异常大幅关节运动或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if development_pose_acceptance and not messagebox.askyesno(
            "v1.2单条长段真机确认",
            "本次最多下发一条六维命令：位移≤60mm、姿态≤6°；TCP速度150mm/s、"
            "加速度400mm/s²、姿态速度30°/s，60秒超时。段内每2mm/0.2°调用厂商逆解；"
            "关节路径检查不通过时不会运动。\n\n"
            "请先完成同参数只读影子；确认机械臂、灵巧手和100cm软件包络对应的整条扫掠空间清空，"
            "人员在外、急停可立即触及，JAKA App无报警且机械臂静止。先松Grip，再握住做一次"
            "清晰但缓慢的组合动作；命令到位后会自动结束，不会继续追随。\n\n"
            "任何反向、抖动、异响、异常大幅关节运动或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if development_continuous and not messagebox.askyesno(
            "v1.2连续长段六维真机确认",
            "本次连续跟随固定使用：每条位移≤60mm、姿态≤6°；TCP速度150mm/s、"
            "加速度400mm/s²、姿态速度30°/s；启动TCP周围100cm软件包络、相对姿态±30°，"
            "最长180秒。每条仍由厂商逆解逐点检查，只追踪最新手柄目标，不补跑历史轨迹。\n\n"
            "该档验证的是连续长段采样，仍为单命令精确到位，不是厂商队列圆滑。请确认整条机械臂、"
            "灵巧手及整个扫掠空间清空，人员在外、急停可立即触及，JAKA App无报警且机械臂静止。"
            "先松Grip，再握住连续操作；松Grip立即请求停止。\n\n"
            "任何反向、明显顿挫恶化、抖动、异响、异常大幅关节运动或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if development_pose_blend and not messagebox.askyesno(
            "v1.2两段六维厂商圆滑现场确认",
            "本次执行代码中固定的两段路径：先沿基坐标+Z 30mm并转动3°，随后沿基坐标+X "
            "30mm并再转动3°。TCP速度150mm/s、加速度400mm/s²、姿态速度30°/s。"
            "第一段使用5mm厂商圆滑容差，末段精确停止，队列最多2条。\n\n"
            "两段均会先按2mm/0.2°调用JAKA厂商逆解检查；检查不通过不会运动。"
            "请确认+Z、+X方向和两段完整扫掠空间均有净空，人员在外、急停可立即触及，"
            "JAKA App无报警且机械臂静止并保持使能。先松Grip，再握住并持续保持到结束；"
            "松Grip会请求厂商停止。\n\n"
            "若方向错误、拐角停顿加重、关节突变、抖动、异响或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        if development_hand_pose_blend and not messagebox.askyesno(
            "v1.2手柄两端点厂商圆滑现场确认",
            "本次只执行一个手柄采样批次。先松Grip，再持续握住：平稳移动或转动至第一个端点"
            "（相对起点至少20mm或2°），继续移动至第二个端点（相对第一点至少20mm或2°），"
            "随后保持手柄不动。程序只采用这两个端点，不补跑中间手柄历史轨迹。\n\n"
            "每段最多60mm/6°；TCP速度150mm/s、加速度400mm/s²、姿态速度30°/s；"
            "第一段tol=5mm，第二段精确停止，厂商队列深度最多2。两个端点均先按2mm/0.2°"
            "调用JAKA厂商逆解检查。\n\n"
            "请确认预计两个方向及整条机械臂扫掠空间均有净空，人员在外、急停可立即触及，"
            "JAKA App无报警且机械臂静止并保持使能。松Grip会停止；若有反向、关节突变、"
            "抖动、异响、报警或SDK超时，立即松Grip并按现场规程处理。是否启动？"):
            return
        if development_rolling_pose and not messagebox.askyesno(
            "v1.5滚动六维厂商队列现场确认",
            "本次最多下发50段六维命令、最长10分钟，持续按住Grip移动手柄。启动TCP周围活动"
            "半径保持100cm；扩展档只增加累计段数，不提高刚才通过的速度和单段上限。当前段"
            "运行时queue=1/active_queue=1，下一段确实预排后必须观测到queue=2/active_queue=1；"
            "交接回到queue=1时立即补入最新黄色目标，不补跑旧轨迹。前9段tol=5mm，"
            "第50段tol=0精确停止。\n\n"
            "每段≤60mm/6°，小目标先合并，只有相对队列末端达到20mm或2°才下发；TCP速度150mm/s、"
            "加速度400mm/s²、姿态速度30°/s。每段仍按2mm/0.2°调用JAKA厂商逆解。"
            "若短段在queue=2可见前已经完成，只有实测TCP精确到达最后目标才允许安全重建队列；"
            "真正4秒无进展或目标不一致仍会停止。完成时还必须观察到至少8次连续交接。"
            "达到50段后自动结束；途中松Grip会调用厂商停止并结束本次验收，需要重新启动后再试。\n\n"
            "请确认预计运动方向和启动TCP周围100cm软件包络对应的整条机械臂扫掠空间有净空，"
            "人员在外、急停可立即触及，JAKA App无报警且机械臂静止并保持使能。若出现反向、"
            "关节突变、明显顿挫恶化、抖动、异响、报警或SDK超时，立即松Grip。是否启动？"):
            return
        if adaptive_pilot and not messagebox.askyesno(
            "V1.5已验收中等幅跟随",
            "本次仅运行60秒、最多10条运动命令；启动TCP周围半径10cm，姿态相对起点≤10°。"
            "每条≤20mm/2°，速度30mm/s、加速度60mm/s²、姿态5°/s。"
            "动态检查固定5mm/1°；厂商逆解或关节余量异常时停止并等待重新握持。\n\n"
            "这不是全范围遥操作许可。请再次确认整条机械臂扫掠空间有净空、人员在外、"
            "JAKA App无报警且机械臂静止、急停可触及。先松Grip，再缓慢握持移动；"
            "方向异常、关节突变、抖动、异响或报警时立即松Grip并按现场规程处理。是否启动？"):
            return
        if adaptive_trajectory and not adaptive_pilot and not messagebox.askyesno(
            "V1.5动态采样候选现场验收",
            f"这是新的动态采样调度，尚未完成本版真机验收。\n"
            f"检查最大间隔{dynamic_policy.max_check_mm:g}mm/{dynamic_policy.max_check_deg:g}°，"
            "遇到关节风险时细分。厂商队列深度2，取消固定20mm/2°起发门槛。\n"
            "本次150mm/s、400mm/s²、姿态30°/s，位置范围100cm、开放全部朝向。\n"
            "不再使用启动姿态±30°测试限制；关节限位和路径筛查仍有效，不保证任意姿态可达。\n"
            "过载时减小接纳增量并显示跟随受限，不补追未接纳动作；松握重新对齐。\n"
            "黄色显示手柄意图，实际接纳目标可能落后；路径风险/断流仍会停止。\n"
            "请先完成相同采样参数的只读检查，确认现场净空、静止无报警、急停可用。启动真机候选？"):
            return
        if production_rolling_pose and not adaptive_trajectory and not messagebox.askyesno(
            "V1.5正式连续厂商队列现场确认",
            ("本次不再按50段自动结束，最长30分钟；按住Grip跟随，松开Grip调用厂商停止，"
             "停止确认后可再次握持，从新的实测TCP重新捕获。不可达、关节变化过大等厂商逆解"
             "筛查结果只丢弃当前目标，不会终止整个会话。\n\n"
             + ("扩展姿态优先已启用：XYZ按手柄1:1跟随并限制在启动TCP周围30cm，姿态范围±45°；"
                "速度200mm/s、加速度500mm/s²、单段≤80mm/4°。\n\n"
                if expanded_pose_priority else
                "姿态优先已启用：姿态范围±30°，XYZ仍按手柄1:1小幅跟随，但被限制在启动TCP"
                "周围10cm内；这不是绝对锁位，接触物体时仍可做小范围位置修正。\n\n"
                if pose_priority else
                "标准六维已启用：位置限制在启动TCP周围100cm，姿态范围±30°。\n\n")
             + ("累计姿态可到±45°；" if expanded_pose_priority else
                "单段仍≤60mm/3°，累计姿态可到±30°；" if pose_priority else
                "单段仍≤60mm/6°，累计姿态可到±30°；")
             + ("TCP速度200mm/s、加速度500mm/s²、姿态速度45°/s；"
                if expanded_pose_priority else
                "TCP速度150mm/s、加速度400mm/s²、姿态速度30°/s；")
             + "20mm/2°合并采样，控制柜队列深度最多2。JAKA碰撞、报警、状态异常、4秒无进展、"
               "实测路径偏离或停止未确认仍会结束会话。\n\n"
               "请确认整个预期扫掠空间有净空、人员在外、急停可立即触及，JAKA App无报警且"
               "机械臂静止并保持使能。任何反向、关节突变、抖动、异响或报警立即松Grip。是否启动？")):
            return
        staged = (one_segment or two_segment or five_segment or ten_segment
                  or twenty_segment or orientation_acceptance
                  or three_orientation_acceptance or ten_orientation_acceptance
                  or bounded_six_dof or expanded_six_dof or production_six_dof
                  or production_shadow or development_pose_acceptance
                  or development_continuous or development_pose_blend
                  or development_hand_pose_blend or development_rolling_pose
                  or production_rolling_pose)
        bounded_continuous = bool(self.live.get() and not staged and not communications_only)
        if rotation_shadow or production_shadow:
            bounded_continuous = False
        if bounded_continuous and not messagebox.askyesno(
            "受限连续位置遥操作确认",
            f"本次在启动TCP周围半径{settings.radius_mm/10:g}cm内连续跟随；"
            f"速度{settings.speed_mm_s:g}mm/s、加速度{settings.acceleration_mm_s2:g}mm/s²、"
            "每次规划短段≤10mm，最长10分钟，保持TCP姿态。\n\n"
            "请确认机械臂、灵巧手及整个设定工作球的扫掠空间清空，人员在外，急停可立即触及；"
            "JAKA App无报警且机器人静止，Tool 1/用户坐标系0正确。先松Grip捕获，再按住Grip跟随。\n\n"
            "松Grip会请求停止；出现抖动、异响、反向、异常大幅运动或报警时按现场规程处理。是否启动？"):
            return
        # 运行模式明确分开，减少嵌套条件表达式导致的误读。
        if rotation_shadow:
            mode = "--rotation-shadow"
        elif production_shadow:
            mode = "--shadow"
        elif communications_only:
            mode = "--comm-check"
        elif (one_segment or two_segment or five_segment or ten_segment
              or twenty_segment or orientation_acceptance
              or three_orientation_acceptance or ten_orientation_acceptance
              or bounded_six_dof or expanded_six_dof or production_six_dof
              or development_pose_acceptance or development_continuous
              or development_pose_blend or development_hand_pose_blend
              or development_rolling_pose or production_rolling_pose
              or self.live.get()):
            mode = "--live"
        else:
            mode = "--shadow"
        args = [sys.executable, "-u", str(ROOT / "quest_endpoint_teleop_entry.py"), mode,
                "--ui-heartbeat", "--ui-protocol", "2",
                "--single-owner-display",
                "--speed-mm-s", str(settings.speed_mm_s), "--radius-mm", str(settings.radius_mm),
                "--acceleration-mm-s2", str(settings.acceleration_mm_s2)]
        if mode == "--live":
            args.append("--configure-frames")
        if communications_only or staged or bounded_continuous or rotation_shadow:
            session_seconds = (60 if adaptive_pilot else
                               1800 if production_rolling_pose else
                               30 if production_shadow else
                               600 if development_rolling_pose else
                               90 if development_hand_pose_blend else
                               60 if (development_pose_acceptance or development_pose_blend) else
                               180 if development_continuous else
                               600 if (bounded_continuous or bounded_six_dof or expanded_six_dof or production_six_dof) else 180 if twenty_segment else
                               120 if ten_segment else 90 if (five_segment or ten_orientation_acceptance) else
                               45 if (two_segment or three_orientation_acceptance) else 30)
            args.extend(["--session-seconds", str(session_seconds)])
        if one_segment:
            args.append("--one-segment-acceptance")
        if two_segment:
            args.append("--two-segment-acceptance")
        if five_segment:
            args.append("--five-segment-acceptance")
        if ten_segment:
            args.append("--ten-segment-acceptance")
        if twenty_segment:
            args.append("--twenty-segment-acceptance")
        if orientation_acceptance:
            args.append("--one-degree-orientation-acceptance")
        if three_orientation_acceptance:
            args.append("--three-degree-orientation-acceptance")
        if ten_orientation_acceptance:
            args.append("--ten-degree-orientation-acceptance")
        if bounded_six_dof:
            args.append("--bounded-six-dof")
        if expanded_six_dof:
            args.append("--expanded-six-dof")
        if (production_six_dof or production_shadow or development_pose_acceptance
                or development_continuous or development_pose_blend
                or development_hand_pose_blend or development_rolling_pose
                or production_rolling_pose):
            args.extend(["--production-six-dof",
                         "--rotation-radius-deg", str(rotation_radius),
                         "--orientation-speed-deg-s", str(orientation_speed),
                         "--segment-mm", str(settings.segment_mm),
                         "--max-orientation-step-deg", str(orientation_step)])
        if development_pose_acceptance:
            args.append("--development-one-pose-acceptance")
        if development_continuous:
            args.append("--development-continuous-long")
        if development_pose_blend:
            args.append("--development-two-pose-blend-acceptance")
        if development_hand_pose_blend:
            args.append("--development-hand-two-pose-blend")
        if development_rolling_pose:
            args.append("--development-rolling-pose-queue")
        if production_rolling_pose:
            args.append("--production-rolling-pose")
        if adaptive_trajectory:
            args.extend(("--adaptive-trajectory","--max-check-mm",str(dynamic_policy.max_check_mm),
                         "--max-check-deg",str(dynamic_policy.max_check_deg)))
        if adaptive_pilot:
            args.append("--adaptive-pilot")
        if pose_priority:
            args.append("--pose-priority")
        if expanded_pose_priority:
            args.append("--expanded-pose-priority")
        if bounded_continuous:
            args.append("--bounded-continuous")
        env = os.environ.copy()
        task_temp = env.get("QUEST_JAKA_TEMP", r"D:\ChatGPT\Temp")
        task_cache = env.get("QUEST_JAKA_CACHE", r"D:\ChatGPT\Cache\quest-jaka-pycache")
        env.update(PYTHONPATH=str(ROOT / "src"), PYTHONIOENCODING="utf-8", TEMP=task_temp,
                   TMP=task_temp, PYTHONPYCACHEPREFIX=task_cache)
        try:
            self.process = subprocess.Popen(args, cwd=str(ROOT), env=env, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding="utf-8", errors="replace",
                bufsize=1, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except OSError as error:
            messagebox.showerror("启动失败", str(error))
            return
        self.stopping = False
        self.stop_confirmed = None
        self.session_fault = False
        self.sdk_wait = None
        self.sdk_stall_warned = False
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        for widget in self.controls:
            widget.configure(state="disabled")
        self.state.set("正在连接；保持Grip松开")
        self.append("\n启动" + ("只读通信检查" if communications_only else
                             "姿态只读影子（零运动）" if rotation_shadow else
                             "v1.2长段六维只读影子（零运动）" if production_shadow else
                             "v1.2一次≤60mm/6°真机验收" if development_pose_acceptance else
                             "v1.2固定参数连续长段六维验收" if development_continuous else
                             "v1.2两段六维厂商队列圆滑验收" if development_pose_blend else
                             "v1.2手柄两端点厂商队列圆滑验收" if development_hand_pose_blend else
                             "v1.5滚动六维厂商队列圆滑验收" if development_rolling_pose else
                             ("V1.5扩展姿态优先连续六维" if expanded_pose_priority else
                              "V1.5姿态优先连续六维" if pose_priority else
                              "V1.5正式连续六维") if production_rolling_pose else
                             "正式可调六维跟随" if production_six_dof else
                             "扩展六维跟随" if expanded_six_dof else
                             "受限连续六维跟随" if bounded_six_dof else
                             "十段累计≤10°姿态真机验收" if ten_orientation_acceptance else
                             "三段累计≤3°姿态真机验收" if three_orientation_acceptance else
                             "一次≤1°姿态真机验收" if orientation_acceptance else
                             "一次≤5mm真机验收" if one_segment else
                             "两段各≤10mm真机验收" if two_segment else
                             "五段累计≤5cm真机验收" if five_segment else
                             "十段累计≤10cm位置跟随验收" if ten_segment else
                             "二十段累计≤20cm位置跟随验收" if twenty_segment else
                             "受限连续位置遥操作" if bounded_continuous else "只读预览") + "会话\n")
        threading.Thread(target=self.read_output, args=(self.process,), daemon=True).start()

    def read_output(self, process):
        """后台读取子进程输出；只有主界面线程负责更新 Tk 控件。"""
        for line in process.stdout:
            self.lines.put(line)
        self.lines.put(json.dumps({"state": "process_exit", "code": process.wait()}) + "\n")

    def send(self, command):
        """向运行进程写入 HEARTBEAT 或 STOP；写失败时只能报告，不能假称已停稳。"""
        if self.process is not None and self.process.poll() is None:
            try:
                self.process.stdin.write(command + "\n")
                self.process.stdin.flush()
            except (OSError, ValueError):
                self.state.set("运行进程通信中断；等待停止确认")

    def pump(self):
        """每 100 ms 处理日志、维持许可，并监视 SDK 调用是否长时间未返回。"""
        if not self.stopping:
            self.send("HEARTBEAT")
        try:
            for _ in range(200):
                line = self.lines.get_nowait()
                try:
                    event = json.loads(line)
                except (ValueError, TypeError):
                    self.append(line)
                    continue
                if not isinstance(event, dict):
                    continue
                if self.readonly_test_active:
                    if event.get("state")!="trajectory_input_status": self.append(line)
                    self.readonly_event(event)
                    if self.closing and not self.readonly_test_active: return
                    continue
                if event.get("state") == "sdk_begin":
                    self.sdk_wait = (event.get("call_id"), event.get("method"), time.perf_counter())
                    continue
                if event.get("state") == "sdk_end":
                    if self.sdk_wait and self.sdk_wait[0] == event.get("call_id"):
                        self.sdk_wait = None
                    if event.get("late") or event.get("code") == -3:
                        self.session_fault = True
                        self.append(f"SDK异常：{event.get('method')}，{event.get('elapsed_ms'):.1f}ms，返回{event.get('code')}\n")
                    continue
                self.append(line)
                if event.get("state") == "intent_admission":
                    self.show_intent_status(event.get("planning") or {})
                if "message" in event:
                    self.state.set(event["message"])
                if event.get("state") in ("fault", "acceptance_incomplete", "stop_unconfirmed", "logout_warning"):
                    self.session_fault = True
                if event.get("state") == "finished":
                    self.stop_confirmed = event.get("stop_confirmed") is True
                    self.state.set("会话结束；停止已确认" if event.get("stop_confirmed") else "停止未确认！请检查JAKA App并处理现场停止")
                if event.get("state") == "process_exit":
                    self.stop_fault_latched = self.stop_confirmed is not True or self.session_fault
                    self.start_button.configure(state="disabled" if self.stop_fault_latched else "normal")
                    self.stop_button.configure(state="disabled")
                    for widget in self.controls:
                        widget.configure(state="normal")
                    if self.stop_fault_latched:
                        self.closing = False
                        self.state.set("本次停止未确认或验收异常！已禁止本窗口重启，请现场检查JAKA App和日志。")
                    elif self.closing:
                        self.root.destroy()
                        return
        except queue.Empty:
            pass
        # GUI是SDK进程之外的观察者。即使原生调用占住GIL，窗口仍能提示失联。
        # STOP只撤销许可，不能声称穿透阻塞中的SDK或已经停稳。
        if self.sdk_wait and not self.sdk_stall_warned:
            limit = 5.0 if self.sdk_wait[1] in ("login", "logout") else .3
            if time.perf_counter() - self.sdk_wait[2] > limit:
                self.sdk_stall_warned = True
                self.stopping = True
                self.send("STOP")
                message = f"SDK {self.sdk_wait[1]} 未返回；已撤销跟随许可，停止尚未确认。请查看平板；若有异常立即物理急停。"
                self.state.set(message)
                self.append(message+"\n")
        self.root.after(100, self.pump)

    def stop_waiting(self):
        """只发停止请求，等待子进程报告停止确认。"""
        self.stopping = True
        self.send("STOP")
        self.state.set("已请求结束只读测试；等待SDK返回并注销（不强杀进程）" if self.readonly_test_active
                       else "已请求停止；等待SDK确认和注销")

    def close(self):
        """窗口关闭时先请求停机；子进程仍在运行就不直接销毁界面。"""
        if self.readonly_test_active or (self.process is not None and self.process.poll() is None):
            self.closing = True
            self.stop_waiting()
        else:
            self.root.destroy()


def main():
    """CMD 与 PyCharm 共用的 GUI 入口；端口 5008 防止重复打开②。"""
    if os.environ.get("QUEST_JAKA_GUI_CHECK") == "1":
        print("GUI_CHECK_OK")
        return
    guard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        guard.bind(("127.0.0.1", 5008))
    except OSError:
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("已经在运行", "②界面已打开，请使用现有窗口。")
        root.destroy()
        return
    root = tk.Tk()
    root._teleop_single_instance_guard = guard
    EndpointTeleopGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
