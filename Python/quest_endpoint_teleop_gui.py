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
from tkinter import messagebox, ttk

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))
from vla_lab.sampled_follow import Settings


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
        ttk.Label(root, text="100cm六维功能已通过；下一关：100mm/s高速六维验收",
                  font=("Microsoft YaHei UI", 16, "bold"), foreground="#a02020").pack(anchor="w", padx=16, pady=14)
        ttk.Label(root, text="位置模式保持姿态；六维模式同时跟随位置和姿态。①单独时只读JAKA；②启动后独占连接并显示实测关节。首次与恢复后先松开Grip。").pack(anchor="w", padx=16)
        box = ttk.LabelFrame(root, text="启动时生效的参数")
        box.pack(fill="x", padx=16, pady=12)
        self.controls = []
        self.speed = self.slider(box, 0, "TCP速度", 5, 100, 100, "mm/s")
        self.radius = self.slider(box, 1, "活动半径", 20, 100, 100, "cm（以启动TCP为中心）")
        self.acceleration = self.slider(box, 2, "TCP加速度", 10, 400, 400, "mm/s²")
        self.rotation_radius = self.slider(box, 3, "姿态范围", 10, 30, 30, "°（相对启动姿态）")
        self.orientation_speed = self.slider(box, 4, "姿态速度", 1, 20, 20, "°/s")
        self.live = tk.BooleanVar(value=False)
        check = ttk.Checkbutton(box, text="受限连续真机跟随（不勾选为只读预览）", variable=self.live)
        check.grid(row=5, column=0, columnspan=4, sticky="w", padx=10, pady=10)
        self.controls.append(check)
        ttk.Label(box, text="正式六维采用上方参数并在启动时锁定；旧验收按钮仍使用按钮标注的固定限值。").grid(row=6, column=0, columnspan=4, sticky="w", padx=10, pady=5)
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
            text="高速正式六维（最大100cm / 100mm/s / ±30°）",
            command=lambda:self.start(production_six_dof=True))
        self.production_six_dof_button.pack(side="left")
        self.controls.append(self.production_six_dof_button)
        self.state = tk.StringVar(value="尚未启动；先打开①并确认右手TRACKED")
        ttk.Label(root, textvariable=self.state, wraplength=1020, foreground="#205080").pack(anchor="w", padx=16, pady=10)
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

    def start(self, communications_only=False, one_segment=False, two_segment=False,
              five_segment=False, ten_segment=False, twenty_segment=False,
              rotation_shadow=False, orientation_acceptance=False,
              three_orientation_acceptance=False,
              ten_orientation_acceptance=False, bounded_six_dof=False,
              expanded_six_dof=False, production_six_dof=False):
        """启动一个独立会话；真机验收按钮使用固定限值，普通按钮仅预览。"""
        if self.stop_fault_latched:
            messagebox.showerror("停止未确认", "本窗口禁止重新启动。请先在现场检查停止状态和故障记录，不要反复启动真机。")
            return
        if self.process is not None and self.process.poll() is None:
            return
        try:
            # 受限真机档位使用代码中的固定限值，不采用界面滑杆。
            # 滑杆只在普通只读预览时生效，避免误以为拖动滑杆就能扩大真机许可。
            if production_six_dof:
                settings = Settings(speed_mm_s=float(self.speed.get()),
                                    radius_mm=float(self.radius.get()) * 10,
                                    acceleration_mm_s2=float(self.acceleration.get()),
                                    segment_mm=20,deadband_mm=3)
                rotation_radius = float(self.rotation_radius.get())
                orientation_speed = float(self.orientation_speed.get())
                if not 200 <= settings.radius_mm <= 1000:
                    raise ValueError("正式六维活动半径必须为20—100cm")
                if not 10 <= rotation_radius <= 30:
                    raise ValueError("正式六维姿态范围必须为10—30°")
                if not 1 <= orientation_speed <= 20:
                    raise ValueError("正式六维姿态速度必须为1—20°/s")
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
        if production_six_dof and not messagebox.askyesno(
            "正式可调六维真机确认",
            f"本次位置与姿态同时跟随：启动TCP周围半径{settings.radius_mm/10:g}cm、"
            f"TCP速度{settings.speed_mm_s:g}mm/s、加速度{settings.acceleration_mm_s2:g}mm/s²；"
            f"姿态范围±{rotation_radius:g}°、速度{orientation_speed:g}°/s。"
            "每条命令仍≤20mm且≤2°，最长10分钟。\n\n"
            "100cm是软件包络，不表示每个点都可达；不可达路径由厂商逆解拒绝。"
            "请确认设定工作球及整条机械臂扫掠空间清空，人员在外，急停可立即触及；"
            "JAKA App无报警且机器人静止，Tool 1/用户坐标系0正确。先松Grip，再握住操作。\n\n"
            "任何反向、抖动、异响、异常大幅关节运动或报警，立即松Grip并按现场规程处理。是否启动？"):
            return
        staged = (one_segment or two_segment or five_segment or ten_segment
                  or twenty_segment or orientation_acceptance
                  or three_orientation_acceptance or ten_orientation_acceptance
                  or bounded_six_dof or expanded_six_dof or production_six_dof)
        bounded_continuous = bool(self.live.get() and not staged and not communications_only)
        if rotation_shadow:
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
        elif communications_only:
            mode = "--comm-check"
        elif (one_segment or two_segment or five_segment or ten_segment
              or twenty_segment or orientation_acceptance
              or three_orientation_acceptance or ten_orientation_acceptance
              or bounded_six_dof or expanded_six_dof or production_six_dof
              or self.live.get()):
            mode = "--live"
        else:
            mode = "--shadow"
        args = [sys.executable, "-u", str(ROOT / "quest_endpoint_teleop_entry.py"), mode,
                "--ui-heartbeat", "--ui-protocol", "2",
                "--single-owner-display",
                "--speed-mm-s", str(settings.speed_mm_s), "--radius-mm", str(settings.radius_mm),
                "--acceleration-mm-s2", str(settings.acceleration_mm_s2)]
        if communications_only or staged or bounded_continuous or rotation_shadow:
            session_seconds = (600 if (bounded_continuous or bounded_six_dof or expanded_six_dof or production_six_dof) else 180 if twenty_segment else
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
        if production_six_dof:
            args.extend(["--production-six-dof",
                         "--rotation-radius-deg", str(rotation_radius),
                         "--orientation-speed-deg-s", str(orientation_speed)])
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
        self.state.set("已请求停止；等待SDK确认和注销")

    def close(self):
        """窗口关闭时先请求停机；子进程仍在运行就不直接销毁界面。"""
        if self.process is not None and self.process.poll() is None:
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
