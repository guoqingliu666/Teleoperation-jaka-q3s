"""Tkinter GUI with keyboard jogging and a 3D arm view for the JAKA S5."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from tkinter import ttk, messagebox
import tkinter as tk

import numpy as np

# Matplotlib keeps a per-user cache under ~/.matplotlib, which can be
# read-only on some Windows profiles. Keep it inside this project instead.
os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parents[2] / ".mplcache"))

# Support both normal package execution
#
#     python -m vla_lab.jaka_jog_gui
#
# and PyCharm's convenient "run this file" action.  In the latter case Python
# does not know that this file belongs to the ``vla_lab`` package, so relative
# imports such as ``from .jaka_jog_controller ...`` would otherwise fail.
if __package__ in (None, ""):
    source_root = Path(__file__).resolve().parents[1]
    if str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))
    __package__ = "vla_lab"

from .jaka_jog_controller import (
    COORD_BASE,
    COORD_TOOL,
    JakaJogController,
    SDK_DIRECTORY_DEFAULT,
)
from .gui_demonstration import GuiDemonstrationSession, GUI_RECORD_MIN_HZ, GUI_RECORD_MAX_HZ
from .evo1_export_launcher import DEFAULT_FPS, launch_wsl_export
from .misumi_gripper_controller import MisumiGripperController
from .demo_pose_recording import DemoPoseRecorder
from .monitor_config import default_monitor_config_path
from .quest_vr_input import (
    HAND_DIRECTIONS,
    JAKA_DIRECTIONS,
    QuestUdpReceiver,
    RelativeQuestTracker,
    ContinuousRotation,
    limit_rotation_step,
    matmul,
    matrix_rpy,
    rotation_angle_rad,
    rpy_matrix,
    transpose,
)


def _unwrap_rpy_near(candidate: tuple[float, float, float], reference: tuple[float, float, float]) -> tuple[float, float, float]:
    """Choose equivalent Euler angles nearest the previous ServoP target.

    `+pi` and `-pi` denote the same orientation but are six radians apart as
    raw numbers.  JAKA ServoP interpolates the supplied values, so sending the
    opposite representation on adjacent frames causes a violent wrist swing.
    """

    return tuple(value + 2.0 * math.pi * round((near - value) / (2.0 * math.pi)) for value, near in zip(candidate, reference, strict=True))


_KEY_DISPLAY = {
    "w": "W",
    "s": "S",
    "a": "A",
    "d": "D",
    "e": "E",
    "q": "Q",
    "up": "↑",
    "down": "↓",
    "left": "←",
    "right": "→",
    "prior": "PgUp",
    "next": "PgDn",
}

_BINDING_LABELS = {
    "jog_x_neg": "X-", "jog_x_pos": "X+", "jog_y_neg": "Y-", "jog_y_pos": "Y+",
    "jog_z_neg": "Z-", "jog_z_pos": "Z+", "jog_rz_pos": "RZ+",
    "jog_rz_neg": "RZ-", "gripper_open": "夹爪打开", "gripper_close": "夹爪闭合",
    "gripper_soft_stop": "夹爪软停止",
}
_JOG_ACTIONS = {
    "jog_x_neg": (0, -1), "jog_x_pos": (0, 1), "jog_y_neg": (1, -1),
    "jog_y_pos": (1, 1), "jog_z_neg": (2, -1), "jog_z_pos": (2, 1),
    "jog_rz_pos": (5, 1), "jog_rz_neg": (5, -1),
}


def _rotation_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(roll), -math.sin(roll)],
            [0.0, math.sin(roll), math.cos(roll)],
        ]
    )
    ry = np.array(
        [
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ]
    )
    rz = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return rz @ ry @ rx


class JakaS5Kinematics:
    """Forward kinematics for the official JAKA S5 URDF (link origins only)."""

    JOINTS: tuple[tuple[tuple[float, float, float], tuple[float, float, float]], ...] = (
        ((0.0, -0.00022535, 0.12015), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
        ((0.43, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.3685, 0.0, -0.114), (0.0, 0.0, 0.0)),
        ((0.0, -0.1135, 0.0), (math.pi / 2.0, 0.0, 0.0)),
        ((0.00026137, 0.1175, 0.00025782), (-math.pi / 2.0, 0.0, 0.0)),
    )

    def forward(self, q: tuple[float, ...]) -> np.ndarray:
        transform = np.eye(4)
        points = [np.zeros(3)]
        for (xyz, rpy), angle in zip(self.JOINTS, q, strict=True):
            origin = np.asarray(xyz, dtype=np.float64)
            frame = np.eye(4)
            frame[:3, :3] = _rotation_matrix(rpy)
            frame[:3, 3] = origin
            transform = transform @ frame
            points.append(transform[:3, 3].copy())
            rotation = np.eye(4)
            rotation[:3, :3] = _rotation_matrix((0.0, 0.0, angle))
            transform = transform @ rotation
        return np.asarray(points)


class JogApplication:
    HOME_QPOS = (0.0, 1.55, 0.25, 1.45, 0.0, 0.0)
    RESTORE_SPEED_RAD_S = 0.30  # previous 0.12 rad/s × 2.5

    def __init__(
        self,
        root: tk.Tk,
        *,
        host: str,
        sdk_directory: str,
        demo: bool,
        speed: float,
        step: float,
        coord: str,
        mode: str,
        poll_hz: float,
        robot_model_label: str,
        key_bindings: dict[str, str],
        config_path: Path,
        tools: list[tuple[int, str]],
        gripper: dict,
        recorder_config_path: Path,
        record_format: str = "raw",
        evo1_output: str = "/home/yufeng/datasets/jaka_evo1",
        default_tool_id: int | None = None,
    ) -> None:
        self.root = root
        self.host = host
        self.sdk_directory = sdk_directory
        self.demo = demo
        self._demo_recorder = DemoPoseRecorder(Path(__file__).resolve().parents[2] / "datasets" / "demo_pose") if demo else None
        self.poll_hz = poll_hz
        # The installed jkrc SDK exposes version/status getters but no
        # documented read-only robot-model getter.  Keep this explicitly as
        # configuration metadata rather than pretending it was detected.
        self.robot_model_label = robot_model_label.strip() or "未配置型号"
        self.controller: JakaJogController | None = None
        self._last_robot_snapshot = None
        self._latest_robot_snapshot = None
        self._latest_gripper_snapshot = None
        self._telemetry_relay_stop = threading.Event()
        self._telemetry_relay_thread: threading.Thread | None = None

        self.kinematics = JakaS5Kinematics()
        self._pressed: dict[str, str] = {}
        self._key_bindings = {action: key.lower() for action, key in key_bindings.items()}
        self._config_path = config_path
        quest = dict(self._load_config_payload().get("quest_vr", {}))
        default_mapping = {
            "forward": "Y+", "backward": "Y-", "left": "X-",
            "right": "X+", "up": "Z+", "down": "Z-",
        }
        self._quest_config = quest
        self._quest_mapping = dict(quest.get("direction_mapping", default_mapping))
        self._quest_sensitivity = tk.DoubleVar(value=max(0.0, min(3.0, float(quest.get("translation_scale", 1.0)))))
        self._quest_rotation_enabled = tk.BooleanVar(value=bool(quest.get("rotation_enabled", False)))
        # Rotation intentionally uses the visible 0--3 sensitivity control as
        # well.  Older config files had a hidden, separate rotation_scale,
        # which made an apparently 1.6x controller feel like 0.5x in rotation.
        self._quest_rotation_scale = self._quest_sensitivity.get()
        self._quest_continuous_rotation = ContinuousRotation()
        # Migrate settings written back by a pre-continuous-rotation GUI.
        self._quest_max_angular_speed_deg_s = (
            max(1.0, float(quest.get("max_angular_speed_deg_s", 280.0)))
            if quest.get("rotation_control_version", 0) >= 2 else 280.0
        )
        self._quest_max_linear_speed_mm_s = max(1.0, float(quest.get("max_linear_speed_mm_s", 500.0)))
        self._quest_confirm = tk.BooleanVar(value=False)
        self._quest_status = tk.StringVar(value="Quest：等待 Unity UDP")
        self._quest_detail = tk.StringVar(value="右手柄：尚未收到 UDP 数据")
        self._quest_armed = False
        self._quest_heading_locked = False
        self._quest_servo_active = False
        self._quest_tcp_reference: tuple[float, ...] | None = None
        self._quest_latest_delta_mm = (0.0, 0.0, 0.0)
        self._quest_last_sent_target: tuple[float, ...] | None = None
        self._quest_last_target_sent_s = 0.0
        self._quest_require_grip_release = False
        self._quest_target_deadband_mm = max(0.1, float(quest.get("target_deadband_mm", 0.5)))
        self._quest_last_frame_s = 0.0
        self._quest_latest_frame = None
        self._quest_timeout_s = max(0.1, float(quest.get("packet_timeout_s", 0.5)))
        self._quest_max_range_mm = max(10.0, float(quest.get("max_relative_translation_mm", 500.0)))
        self._quest_receiver = QuestUdpReceiver(str(quest.get("udp_bind", "127.0.0.1")), int(quest.get("udp_port", 5005)))
        self._quest_tracker = RelativeQuestTracker(
            self._quest_mapping,
            grip_on=float(quest.get("grip_on", 0.70)), grip_off=float(quest.get("grip_off", 0.55)),
            trigger_on=float(quest.get("trigger_on", 0.70)), trigger_off=float(quest.get("trigger_off", 0.20)),
        )
        self._binding_by_key: dict[str, str] = {}
        self._capturing_action: str | None = None
        self._rebuild_bindings()
        self._gripper = dict(gripper)
        self._gripper_unit_mm = float(self._gripper.get("position_unit_mm", 0.01))
        self._gripper_max_units = int(self._gripper.get("fully_closed_position_units", 5000))
        self._gripper_close_mm_var = tk.DoubleVar(
            value=float(self._gripper.get("close_position_units", self._gripper_max_units)) * self._gripper_unit_mm
        )
        self._gripper_speed_var = tk.IntVar(value=int(self._gripper.get("speed_percent", 50)))
        configured_force = int(self._gripper.get("force_percent", 80))
        # Earlier GUI builds allowed 1--19%, which makes this installed
        # gripper immediately report holding without actually closing. Treat
        # such stale settings as invalid and restore the verified 80% default.
        self._gripper_force_var = tk.IntVar(
            value=configured_force if 20 <= configured_force <= 100 else 80
        )
        self.gripper_controller: MisumiGripperController | None = (
            # 演示模式也必须隔离夹爪，不能只隔离机械臂。
            MisumiGripperController(self._gripper) if self._gripper.get("enabled") and not demo else None
        )
        self.demonstration: GuiDemonstrationSession | None = None
        self._cameras_enabled = False
        self._record_notice: tuple[str, float] | None = None
        self._record_format_var = tk.StringVar(value=record_format)
        # Lock the selected target at record start.  Changing the combobox
        # while a demonstration is in progress only affects the next episode.
        self._active_record_format = record_format
        self._evo1_output = evo1_output
        self._evo1_export_jobs: dict[Path, tuple[object, Path]] = {}
        self._evo1_export_messages: dict[Path, str] = {}
        self._record_instruction_var = tk.StringVar(value="将目标物抓取并放入指定区域")
        self._record_status_var = tk.StringVar(value="示教采集源：启动中")
        self._record_hz_var = tk.IntVar(value=10)
        self._active_record_hz = 10.0
        try:
            if not demo:
                self.demonstration = GuiDemonstrationSession(recorder_config_path)
                self._cameras_enabled = True
                self._record_hz_var.set(int(round(min(GUI_RECORD_MAX_HZ, max(GUI_RECORD_MIN_HZ, self.demonstration.sample_hz)))))
        except Exception as error:
            self._record_status_var.set(f"示教录制不可用：{error}")
        self._tools = list(tools)
        # Keep user-assigned names even after the controller refreshes its TCP
        # list (some firmware returns only "Tool 9" rather than its saved name).
        self._configured_tool_names = {tool_id: name for tool_id, name in self._tools}
        self._tool_display = [f"{name} (ID {tool_id})" for tool_id, name in self._tools]
        self._tool_by_display = {f"{name} (ID {tool_id})": tool_id for tool_id, name in self._tools}
        self._tool_box: ttk.Combobox | None = None
        self._tool_profiles_signature: tuple[tuple[int, str, tuple[float, ...]], ...] = ()
        self._last_tool_id: int | None = None

        root.title("[DEMO 机器人模拟 / 相机手动连接] JAKA + Quest 姿态验收" if demo else "[LIVE 真实硬件] JAKA S5 键盘 XYZ 控制")
        root.minsize(1120, 760)

        self._mode_var = tk.StringVar(value="step" if mode == "step" else "continuous")
        self._coord_var = tk.StringVar(value="tool" if coord == "tool" else "base")
        self._speed_var = tk.DoubleVar(value=max(1.0, min(float(speed), 190.0)))
        self._step_var = tk.DoubleVar(value=max(0.5, min(float(step), 50.0)))
        self._ip_var = tk.StringVar(value=host)
        initial_tool_display = ""
        if default_tool_id is not None:
            initial_tool_display = next(
                (display for display, tool_id in self._tool_by_display.items() if tool_id == default_tool_id),
                "",
            )
        self._tool_var = tk.StringVar(value=initial_tool_display or (self._tool_display[0] if self._tool_display else ""))
        self._selected_tool_id_for_relay = self._tool_by_display.get(self._tool_var.get())

        self._status_text = tk.StringVar(value="未连接")
        self._power_text = tk.StringVar(value="上电：—")
        self._enable_text = tk.StringVar(value="使能：—")
        self._moving_text = tk.StringVar(value="运动：—")
        self._tool_text = tk.StringVar(value="工具：—")
        self._safety_text = tk.StringVar(value="")
        self._error_text = tk.StringVar(value="")
        self._tcp_text = tk.StringVar(value="TCP: —")
        self._joints_text = tk.StringVar(value="关节: —")
        self._gripper_text = tk.StringVar(value="夹爪：未连接")

        self._build_widgets()
        self._bind_keys()
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_loop()
        self._telemetry_relay_thread = threading.Thread(
            target=self._telemetry_relay_loop,
            name="jaka-gui-telemetry-relay",
            daemon=True,
        )
        self._telemetry_relay_thread.start()
        root.after(150, self._auto_connect)
        root.after(20, self._quest_tick)

    # ------------------------------------------------------------ layout
    def _build_widgets(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="控制器 IP:").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self._ip_var, width=14).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="连接", command=self._connect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="断开", command=self._disconnect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="上电", command=lambda: self._command("power_on")).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="下电", command=lambda: self._command("power_off")).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="使能", command=lambda: self._command("enable")).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="去使能", command=lambda: self._command("disable")).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="急停 STOP", command=self._emergency_stop).pack(side=tk.LEFT, padx=8)

        # These controls deliberately release the physical camera handles
        # without closing the jog/recording GUI, so ACT inference can own both
        # devices between demonstrations.
        ttk.Button(top, text="开启相机", command=self._open_cameras).pack(side=tk.RIGHT, padx=2)
        ttk.Button(top, text="关闭相机", command=self._close_cameras).pack(side=tk.RIGHT, padx=2)

        ttk.Label(top, text="工具(中心点):").pack(side=tk.LEFT, padx=(10, 2))
        self._tool_box = ttk.Combobox(top, textvariable=self._tool_var, values=self._tool_display, width=47, state="readonly")
        self._tool_box.pack(side=tk.LEFT, padx=2)
        self._tool_box.bind("<<ComboboxSelected>>", self._on_tool_selected)
        ttk.Button(top, text="刷新 TCP", command=self._refresh_tool_profiles).pack(side=tk.LEFT, padx=2)

        status = ttk.Frame(self.root, padding=(10, 4))
        status.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(status, textvariable=self._status_text, font=("Consolas", 11, "bold")).pack(side=tk.LEFT)
        ttk.Label(status, textvariable=self._power_text, font=("Consolas", 10)).pack(side=tk.LEFT, padx=12)
        ttk.Label(status, textvariable=self._enable_text, font=("Consolas", 10)).pack(side=tk.LEFT, padx=12)
        ttk.Label(status, textvariable=self._moving_text, font=("Consolas", 10)).pack(side=tk.LEFT, padx=12)
        ttk.Label(status, textvariable=self._tool_text, font=("Consolas", 10)).pack(side=tk.LEFT, padx=12)
        ttk.Label(status, textvariable=self._safety_text, foreground="#b00020").pack(side=tk.LEFT, padx=12)
        ttk.Label(status, textvariable=self._error_text, foreground="#b00020", font=("Consolas", 9)).pack(side=tk.LEFT, padx=12)

        telemetry = ttk.Frame(self.root, padding=(10, 2))
        telemetry.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(telemetry, textvariable=self._tcp_text, font=("Consolas", 10)).pack(side=tk.LEFT)
        ttk.Label(telemetry, textvariable=self._joints_text, font=("Consolas", 10)).pack(side=tk.LEFT, padx=12)
        ttk.Label(telemetry, textvariable=self._gripper_text, font=("Consolas", 10), foreground="#7a3e00").pack(side=tk.LEFT, padx=12)

        body = ttk.Frame(self.root, padding=8)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self._figure_frame = ttk.Frame(body)
        self._figure_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._init_camera_preview()

        # Keep the control panel usable on short displays.  The controls below
        # (notably the safety helpers) are intentionally retained in one
        # column and the vertical scrollbar exposes the lower portion.
        control_holder = ttk.Frame(body)
        control_holder.pack(side=tk.RIGHT, fill=tk.Y, padx=(8, 0))
        control_scroll = ttk.Scrollbar(control_holder, orient=tk.VERTICAL)
        control_canvas = tk.Canvas(
            control_holder,
            width=485,
            highlightthickness=0,
            yscrollcommand=control_scroll.set,
        )
        control_scroll.configure(command=control_canvas.yview)
        control_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        control_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        control = ttk.LabelFrame(control_canvas, text="Jog 控制 / 键盘映射", padding=10)
        control_window = control_canvas.create_window((0, 0), window=control, anchor="nw")

        def sync_control_scroll(_event: tk.Event | None = None) -> None:
            control_canvas.configure(scrollregion=control_canvas.bbox("all"))

        def fit_control_width(event: tk.Event) -> None:
            control_canvas.itemconfigure(control_window, width=event.width)

        control.bind("<Configure>", sync_control_scroll)
        control_canvas.bind("<Configure>", fit_control_width)

        ttk.Label(control, text="运动模式:").grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Radiobutton(control, text="步进 (按一下动一下)", variable=self._mode_var, value="step").grid(row=1, column=0, columnspan=2, sticky="w")
        ttk.Radiobutton(control, text="连续 (按住移动)", variable=self._mode_var, value="continuous").grid(row=2, column=0, columnspan=2, sticky="w")

        ttk.Label(control, text="坐标系:").grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Radiobutton(control, text="基座", variable=self._coord_var, value="base").grid(row=4, column=0, sticky="w")
        ttk.Radiobutton(control, text="工具", variable=self._coord_var, value="tool").grid(row=4, column=1, sticky="w")

        ttk.Label(control, text="速度 (mm/s 或 °/s):").grid(row=5, column=0, sticky="w", pady=(10, 0))
        ttk.Scale(control, from_=1, to=80, variable=self._speed_var, orient=tk.HORIZONTAL, length=130).grid(row=6, column=0, sticky="w")
        self._speed_value = ttk.Label(control, text=f"{self._speed_var.get():.0f}")
        self._speed_value.grid(row=6, column=1, sticky="w")
        self._speed_var.trace_add("write", lambda *_: self._speed_value.configure(text=f"{self._speed_var.get():.0f}"))

        ttk.Label(control, text="步进量 (mm 或 °):").grid(row=7, column=0, sticky="w", pady=(8, 0))
        ttk.Scale(control, from_=0.5, to=20, variable=self._step_var, orient=tk.HORIZONTAL, length=130).grid(row=8, column=0, sticky="w")
        self._step_value = ttk.Label(control, text=f"{self._step_var.get():.1f}")
        self._step_value.grid(row=8, column=1, sticky="w")
        self._step_var.trace_add("write", lambda *_: self._step_value.configure(text=f"{self._step_var.get():.1f}"))

        ttk.Label(control, text="键盘映射:", font=("Consolas", 9, "bold")).grid(row=9, column=0, sticky="w", pady=(10, 0))
        ttk.Button(control, text="配置按键", command=self._show_key_settings).grid(row=9, column=1, sticky="e", pady=(10, 0))
        row = 10
        for action in _BINDING_LABELS:
            ttk.Label(control, text=_BINDING_LABELS[action]).grid(row=row, column=0, sticky="w")
            ttk.Label(control, text=self._key_label(self._key_bindings.get(action, "—")), foreground="#444").grid(row=row, column=1, sticky="w", padx=8)
            row += 1

        jog = ttk.LabelFrame(control, text="按钮点动", padding=8)
        jog.grid(row=row, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        for col, (axis, direction, label) in enumerate(
            ((0, -1, "X-"), (0, 1, "X+"), (1, -1, "Y-"), (1, 1, "Y+"), (2, -1, "Z-"), (2, 1, "Z+"))
        ):
            button = tk.Button(jog, text=label, width=4, height=1)
            button.grid(row=0, column=col, padx=2)
            button.bind("<ButtonPress-1>", lambda event, a=axis, d=direction: self._jog_down(a, d))
            button.bind("<ButtonRelease-1>", lambda event: self._jog_up())
        for col, (axis, direction, label) in enumerate(((5, 1, "RZ+"), (5, -1, "RZ-"))):
            button = tk.Button(jog, text=label, width=4, height=1)
            button.grid(row=1, column=col, padx=2, pady=(4, 0))
            button.bind("<ButtonPress-1>", lambda event, a=axis, d=direction: self._jog_down(a, d))
            button.bind("<ButtonRelease-1>", lambda event: self._jog_up())

        gripper = ttk.LabelFrame(control, text="MISUMI 夹爪（人工控制）", padding=8)
        gripper.grid(row=row + 1, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        max_close_mm = self._gripper_max_units * self._gripper_unit_mm
        ttk.Label(gripper, text="闭合量(mm):").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(gripper, from_=0.0, to=max_close_mm, increment=0.1, textvariable=self._gripper_close_mm_var, width=6).grid(row=0, column=1, padx=(2, 12))
        ttk.Label(gripper, text="速度(%):").grid(row=0, column=2, sticky="w")
        ttk.Spinbox(gripper, from_=1, to=100, increment=1, textvariable=self._gripper_speed_var, width=5).grid(row=0, column=3, padx=(2, 0))
        ttk.Label(gripper, text="力(%，最小20):").grid(row=1, column=0, sticky="w", pady=(5, 0))
        ttk.Spinbox(gripper, from_=20, to=100, increment=1, textvariable=self._gripper_force_var, width=5).grid(row=1, column=1, padx=(2, 12), pady=(5, 0))
        ttk.Button(gripper, text="保存参数", command=self._persist_gripper_settings).grid(row=1, column=2, columnspan=2, sticky="w", pady=(5, 0))
        ttk.Label(gripper, text="打开 0 mm；闭合用上方闭合量；速度与力共用", foreground="#555").grid(row=2, column=0, columnspan=4, sticky="w", pady=(4, 4))
        ttk.Button(gripper, text="连接夹爪", command=lambda: self._gripper_command("connect")).grid(row=3, column=0, padx=2, sticky="w")
        ttk.Button(gripper, text="使能", command=lambda: self._gripper_command("enable")).grid(row=3, column=1, padx=2, sticky="w")
        ttk.Button(gripper, text="打开", command=lambda: self._gripper_command("open")).grid(row=3, column=2, padx=2, sticky="w")
        ttk.Button(gripper, text="闭合", command=lambda: self._gripper_command("close")).grid(row=3, column=3, padx=2, sticky="w")
        ttk.Button(gripper, text="软停止", command=lambda: self._gripper_command("soft_stop")).grid(row=4, column=0, padx=2, sticky="w", pady=(4, 0))

        quest = ttk.LabelFrame(control, text="Quest 3S（Unity UDP；本 GUI 唯一控制硬件）", padding=8)
        quest.grid(row=row + 2, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        ttk.Label(quest, textvariable=self._quest_status, foreground="#075985", wraplength=430).grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(quest, textvariable=self._quest_detail, foreground="#444", font=("Consolas", 8), justify="left", wraplength=430).grid(row=1, column=0, columnspan=4, sticky="w", pady=(3, 0))
        ttk.Checkbutton(quest, text="仅允许 Grip 驱动模拟 TCP（无真实硬件）" if self.demo else "我确认现场安全，允许 Grip 控制 TCP", variable=self._quest_confirm).grid(row=2, column=0, columnspan=4, sticky="w", pady=(5, 0))
        ttk.Button(quest, text="ARM 手柄伺服", command=self._quest_arm).grid(row=3, column=0, padx=(0, 4), pady=(5, 0))
        ttk.Button(quest, text="解除 / 停止", command=self._quest_disarm).grid(row=3, column=1, padx=4, pady=(5, 0))
        ttk.Label(quest, text="平移/旋转灵敏度 0–3:").grid(row=3, column=2, padx=(12, 2), pady=(5, 0))
        ttk.Spinbox(quest, from_=0.0, to=3.0, increment=0.1, textvariable=self._quest_sensitivity, width=5).grid(row=3, column=3, pady=(5, 0), sticky="w")
        ttk.Checkbutton(quest, text="启用手柄旋转", variable=self._quest_rotation_enabled).grid(row=4, column=0, columnspan=2, sticky="w", pady=(5, 0))
        ttk.Button(quest, text="方向映射…", command=self._show_quest_mapping).grid(row=4, column=2, columnspan=2, padx=(4, 0), pady=(5, 0), sticky="e")
        ttk.Button(quest, text="锁定当前 VR 正前方", command=self._quest_lock_heading).grid(row=5, column=0, columnspan=2, sticky="w", pady=(5, 0))
        ttk.Label(quest, text="Grip：按住控制、松开暂停、再按继续；扳机控制夹爪；摇杆 ARM/解除\nA 开始录制；X 保存成功；Y 放弃本次；B 复原保存位置", foreground="#555", wraplength=430).grid(row=6, column=0, columnspan=4, sticky="w", pady=(5, 0))

        automation = ttk.LabelFrame(control, text="安全辅助运动", padding=8)
        automation.grid(row=row + 3, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        ttk.Button(automation, text="法兰找平", command=self._level_flange).pack(side=tk.LEFT, padx=2)
        ttk.Button(automation, text="保存当前位置", command=self._save_position).pack(side=tk.LEFT, padx=2)
        ttk.Button(automation, text="复原保存位置", command=self._restore_position).pack(side=tk.LEFT, padx=2)

        # Mouse-wheel scrolling when the pointer is over the control column;
        # the visible scrollbar remains available for precise dragging.
        def scroll_control(event: tk.Event) -> str:
            if event.delta:
                control_canvas.yview_scroll(-int(event.delta / 120), "units")
            return "break"

        control_canvas.bind("<MouseWheel>", scroll_control)
        control.bind("<MouseWheel>", scroll_control)

        lower = tk.PanedWindow(self.root, orient=tk.VERTICAL, sashwidth=8, sashrelief=tk.RAISED, height=230)
        # ``body`` was packed as an expanding pane earlier.  Reserving this
        # lower pane before it prevents Tk from giving the entire remaining
        # height to the 3-D/control area and making the log disappear.
        lower.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(0, 4), before=body)
        recorder = ttk.LabelFrame(lower, text="示教 Episode 录制（人工控制；不自动运动）", padding=(10, 5))
        ttk.Label(recorder, text="录制格式（下一条 Episode）:").grid(row=0, column=0, sticky="w", pady=(0, 4))
        ttk.Combobox(
            recorder,
            textvariable=self._record_format_var,
            values=("raw", "raw+evo1"),
            state="readonly",
            width=14,
        ).grid(row=0, column=1, sticky="w", padx=4, pady=(0, 4))
        ttk.Label(
            recorder,
            text="raw：仅原始数据；raw+evo1：原始数据 + 后台 Evo-1 导出",
        ).grid(row=0, column=2, columnspan=7, sticky="w", pady=(0, 4))
        ttk.Label(recorder, text="采样频率:").grid(row=1, column=0, sticky="w")
        ttk.Spinbox(
            recorder,
            from_=GUI_RECORD_MIN_HZ,
            to=GUI_RECORD_MAX_HZ,
            increment=1,
            textvariable=self._record_hz_var,
            width=5,
        ).grid(row=1, column=1, sticky="w", padx=4)
        ttk.Label(recorder, text=f"Hz（{GUI_RECORD_MIN_HZ:g}–{GUI_RECORD_MAX_HZ:g}；建议15 Hz；>30为压力测试，保存前自动校验）").grid(
            row=1, column=2, columnspan=7, sticky="w"
        )
        depth_mode_text = "RGB-D（保存深度）" if (self.demonstration and self.demonstration.capture_depth) else "RGB-only（不采集、不保存 ZED 深度）"
        ttk.Label(recorder, text="DEMO：VR 输入 + 模拟 TCP；无图像/真实硬件；按收到的帧记录，上方导出格式和频率不适用" if self.demo else f"本次采集内容：ZED {depth_mode_text}；全局 RGB；JAKA/夹爪状态").grid(
            row=2, column=0, columnspan=9, sticky="w", pady=(2, 0)
        )
        ttk.Label(recorder, text="任务指令:").grid(row=3, column=0, sticky="w")
        ttk.Entry(recorder, textvariable=self._record_instruction_var, width=56).grid(row=3, column=1, columnspan=4, sticky="ew", padx=4)
        ttk.Button(recorder, text="开始录制", command=self._record_start).grid(row=3, column=5, padx=3)
        ttk.Button(recorder, text="保存成功", command=lambda: self._record_finish(True)).grid(row=3, column=6, padx=3)
        ttk.Button(recorder, text="保存失败", command=lambda: self._record_finish(False)).grid(row=3, column=7, padx=3)
        ttk.Button(recorder, text="放弃本次", command=self._record_cancel).grid(row=3, column=8, padx=3)
        ttk.Label(recorder, textvariable=self._record_status_var, foreground="#006d3c").grid(row=4, column=0, columnspan=9, sticky="w", pady=(4, 2))
        ttk.Label(recorder, text="阶段标记:").grid(row=5, column=0, sticky="w")
        for column, label in enumerate(("接近", "夹取", "抬起", "放置", "任务完成（标记）"), start=1):
            ttk.Button(recorder, text=label, command=lambda value=label: self._record_mark(value)).grid(row=5, column=column, padx=3, pady=(2, 0))

        log_frame = ttk.LabelFrame(lower, text="JAKA / MISUMI 运行日志（拖动中间横线可调整高度）", padding=(4, 2))
        log_scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL)
        self._log = tk.Text(
            log_frame,
            height=7,
            state="disabled",
            font=("Consolas", 9),
            wrap=tk.NONE,
            yscrollcommand=log_scroll.set,
        )
        log_scroll.configure(command=self._log.yview)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self._log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        lower.add(recorder, minsize=90)
        lower.add(log_frame, minsize=70)
        self._make_buttons_mouse_only(self.root)

    def _make_buttons_mouse_only(self, parent: tk.Misc) -> None:
        """Do not leave any button focused after a mouse click.

        Jog keys, including Space, are global bindings.  A focused Tk button
        would also invoke itself on Space, so buttons must remain mouse-only.
        """

        for widget in parent.winfo_children():
            if isinstance(widget, (tk.Button, ttk.Button)):
                widget.configure(takefocus=False)
                widget.bind("<FocusIn>", lambda _event: self.root.after_idle(self.root.focus_set), add="+")
            self._make_buttons_mouse_only(widget)

    def _key_label(self, key: str) -> str:
        return _KEY_DISPLAY.get(key, key.upper())

    def _rebuild_bindings(self) -> None:
        seen: dict[str, str] = {}
        for action, key in self._key_bindings.items():
            if action not in _BINDING_LABELS or not key:
                continue
            previous = seen.get(key)
            if previous is not None and previous != action:
                raise ValueError(f"按键冲突：{key} 同时映射到 {previous} 和 {action}")
            seen[key] = action
        self._binding_by_key = seen

    def _init_3d_view(self) -> None:
        try:
            import matplotlib

            matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
            matplotlib.rcParams["axes.unicode_minus"] = False
            matplotlib.use("TkAgg")
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
        except Exception:
            self._view = None
            ttk.Label(self._figure_frame, text="Matplotlib 不可用，无法显示 3D 视图。").pack()
            return

        figure = Figure(figsize=(6.5, 6.5), dpi=100)
        self._axes = figure.add_subplot(111, projection="3d")
        self._canvas = FigureCanvasTkAgg(figure, master=self._figure_frame)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._axes.set_xlim(-0.3, 1.2)
        self._axes.set_ylim(-0.8, 0.8)
        self._axes.set_zlim(-0.3, 1.0)
        self._axes.set_xlabel("X (m)")
        self._axes.set_ylabel("Y (m)")
        self._axes.set_zlabel("Z (m)")
        self._axes.set_title("JAKA S5 机械臂姿态")
        (self._arm_line,) = self._axes.plot([], [], [], "-o", lw=3, color="#3b6ea5")
        (self._tcp_marker,) = self._axes.plot([], [], [], "o", ms=7, color="#e67e22")
        self._view = "matplotlib"

    def _init_camera_preview(self) -> None:
        """Show the two frames used by the Episode recorder, side by side."""

        self._view = "camera_preview"
        if self.demo:
            from .hik_camera_preview import HikPreviewPanel
            self._hik_preview = HikPreviewPanel(self._figure_frame)
            self._hik_preview.pack(fill=tk.BOTH, expand=True)
            return
        cameras = ttk.Frame(self._figure_frame, padding=4)
        cameras.pack(fill=tk.BOTH, expand=True)
        cameras.columnconfigure(0, weight=1)
        cameras.columnconfigure(1, weight=1)
        cameras.rowconfigure(0, weight=1)
        left = ttk.LabelFrame(cameras, text="ZED Mini（录制 RGB：左目）", padding=4)
        right = ttk.LabelFrame(cameras, text="DroidCam（全局 RGB）", padding=4)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        self._zed_preview = tk.Label(left, text="等待 ZED 左目画面…", anchor="center", background="#101418", foreground="#e5e7eb")
        self._global_preview = tk.Label(right, text="等待 DroidCam 全局画面…", anchor="center", background="#101418", foreground="#e5e7eb")
        self._zed_preview.pack(fill=tk.BOTH, expand=True)
        self._global_preview.pack(fill=tk.BOTH, expand=True)
        self._zed_preview_frame = left
        self._global_preview_frame = right
        self._preview_container = cameras

    # ------------------------------------------------------------ actions
    def _command(self, name: str) -> None:
        if self.controller is None:
            self._error_text.set("请先点击“连接”")
            self._append_log(["⚠ 请先点击“连接”"])
            return
        getattr(self.controller, name)()

    def _connect(self) -> None:
        if self.demonstration is not None and self.demonstration.status.phase in {"starting", "recording", "finishing"}:
            self._error_text.set("请先结束本次录制，再切换控制器连接")
            return
        host = self._ip_var.get().strip() or "10.5.5.100"
        self.host = host
        self._persist_jaka_connection()
        if self.controller is not None:
            self.controller.shutdown()
        self.controller = JakaJogController(
            host,
            self.sdk_directory,
            poll_hz=self.poll_hz,
            demo=self.demo,
        )
        self.controller.login()
        tool_id = self._tool_by_display.get(self._tool_var.get())
        if tool_id is not None:
            self.controller.set_tool_id(tool_id)
        if self.demonstration is not None:
            self.demonstration.reset_robot_bridge()

    def _persist_jaka_connection(self) -> None:
        """Persist the typed controller address; this has no SDK side effect."""

        payload = self._load_config_payload()
        jaka = dict(payload.get("jaka", {}))
        jaka["host"] = self.host
        jaka["model_label"] = self.robot_model_label
        payload["jaka"] = jaka
        self._write_config_payload(payload)

    def _auto_connect(self) -> None:
        """Connect telemetry endpoints on startup; never power or move hardware."""

        self._connect()
        if self.gripper_controller is not None:
            self.gripper_controller.connect()
        if self.demonstration is not None:
            self.demonstration.start_sources()

    def _close_cameras(self) -> None:
        if self.demo and hasattr(self, "_hik_preview"):
            self._hik_preview.stop()
            return
        if self.demonstration is None:
            self._set_record_notice("相机预览不可用：示教采集未初始化")
            return
        try:
            self.demonstration.stop_sources()
        except Exception as error:
            self._set_record_notice(f"无法关闭相机：{error}")
            return
        self._cameras_enabled = False
        self._clear_camera_preview("相机已关闭：设备已释放给 ACT 推理")
        self._record_status_var.set("相机已关闭；JAKA Jog 与夹爪控制仍可使用")
        self._append_log(["已关闭 ZED 与 DroidCam，并释放设备占用"])

    def _open_cameras(self) -> None:
        if self.demo and hasattr(self, "_hik_preview"):
            self._hik_preview.start()
            return
        if self.demonstration is None:
            self._set_record_notice("相机预览不可用：示教采集未初始化")
            return
        try:
            self.demonstration.start_sources()
        except Exception as error:
            self._set_record_notice(f"无法开启相机：{error}")
            return
        self._cameras_enabled = True
        self._clear_camera_preview("正在开启 ZED 与 DroidCam…")
        self._record_status_var.set("正在开启相机；等待两路画面与同步")
        self._append_log(["正在开启 ZED 与 DroidCam 预览"])

    # ---------------------------------------------------- demonstration
    def _record_start(self) -> None:
        if self._demo_recorder is not None:
            try:
                if not self._quest_fresh():
                    raise RuntimeError("请先运行 VR 预览并确认右手追踪正常")
                self._demo_recorder.start(self._record_instruction_var.get(), {
                    "mapping": self._quest_mapping, "sensitivity": self._quest_sensitivity.get(),
                    "rotation_enabled": self._quest_rotation_enabled.get(), "quest_config": self._quest_config})
                self._record_status_var.set("DEMO 录制中：VR + 模拟 TCP（无图像/真实硬件）")
            except Exception as error:
                self._record_status_var.set(f"DEMO 无法开始：{error}")
            return
        if self.demonstration is None:
            self._record_status_var.set("示教录制不可用；请检查 readonly_monitor.json")
            return
        if not self._cameras_enabled:
            self._set_record_notice("请先点击右上角“开启相机”")
            return
        try:
            self._active_record_format = self._record_format_var.get()
            self._active_record_hz = self.demonstration.set_sample_hz(
                float(self._record_hz_var.get()), persist=True
            )
            self.demonstration.start_recording(self._record_instruction_var.get())
            self._record_notice = None
            self._record_status_var.set(
                f"正在创建 Episode（{self._active_record_hz:g} Hz，{self._active_record_format}）…"
                + (f" {self.demonstration.sampling_rate_advisory}" if self.demonstration.sampling_rate_advisory else "")
            )
        except Exception as error:
            self._set_record_notice(f"无法开始：{error}")

    def _record_finish(self, success: bool) -> None:
        if self._demo_recorder is not None:
            try:
                path = self._demo_recorder.finish("success" if success else "failure")
                self._record_status_var.set(f"DEMO 已保存 {self._demo_recorder.frames} 帧：{path}")
            except Exception as error:
                self._record_status_var.set(f"DEMO 保存失败：{error}")
            return
        if self.demonstration is None:
            self._set_record_notice("无法保存：示教录制不可用")
            return
        if self.demonstration.status.phase != "recording":
            self._set_record_notice("无法保存：当前没有正在录制的 Episode")
            return
        self.demonstration.finish(success)

    def _record_cancel(self) -> None:
        if self._demo_recorder is not None and self._demo_recorder.active:
            path = self._demo_recorder.finish("discarded")
            self._record_status_var.set(f"DEMO 已标记放弃（文件保留）：{path}")
            return
        if self.demonstration is not None:
            self.demonstration.cancel()

    def _start_evo1_export(self, episode_path: Path) -> None:
        """Start a post-recording export without blocking the control GUI."""

        # ``_refresh_recording`` runs continuously after an Episode finishes.
        # Keep a terminal message for completed/failed exports so that the
        # next refresh cannot start a second export for the same Episode.
        if (
            self._active_record_format != "raw+evo1"
            or episode_path in self._evo1_export_jobs
            or episode_path in self._evo1_export_messages
        ):
            return
        # A batch file started from Explorer need not inherit the project as
        # its current directory.  Keep logs beside the project data instead
        # of relying on a relative ``datasets/`` path.
        project_root = Path(__file__).resolve().parents[2]
        log_path = project_root / "datasets" / "evo1_export_logs" / f"{episode_path.name}.log"
        try:
            process = launch_wsl_export(
                episode_path,
                self._evo1_output,
                fps=DEFAULT_FPS,
                log_path=log_path,
            )
        except Exception as error:
            self._evo1_export_messages[episode_path] = f"Evo-1 导出未启动：{error}"
            self._append_log([self._evo1_export_messages[episode_path]])
            return
        self._evo1_export_jobs[episode_path] = (process, log_path)
        self._evo1_export_messages[episode_path] = "Evo-1 后台导出中（原始 Episode 已安全保存）"
        self._append_log([
            f"Evo-1 导出已启动：{episode_path.name}",
            f"目标：{self._evo1_output}",
            f"日志：{log_path}",
        ])

    def _poll_evo1_export(self, episode_path: Path) -> str:
        job = self._evo1_export_jobs.get(episode_path)
        if job is None:
            return self._evo1_export_messages.get(episode_path, "")
        process, log_path = job
        return_code = process.poll()  # type: ignore[union-attr]
        if return_code is None:
            return self._evo1_export_messages[episode_path]
        if return_code == 0:
            message = f"Evo-1 导出完成：{self._evo1_output}"
        else:
            message = f"Evo-1 导出失败（退出码 {return_code}），查看日志：{log_path}"
        self._evo1_export_messages[episode_path] = message
        del self._evo1_export_jobs[episode_path]
        self._append_log([message])
        return message

    def _set_record_notice(self, message: str, duration_s: float = 4.0) -> None:
        self._record_notice = (message, time.monotonic() + duration_s)
        self._record_status_var.set(message)

    def _record_mark(self, label: str) -> None:
        if self._demo_recorder is not None:
            self._demo_recorder.mark(label)
        if self.demonstration is not None:
            self.demonstration.mark_stage(label)

    # ------------------------------------------------------ saved pose
    def _level_flange(self) -> None:
        if self.controller is None:
            self._error_text.set("请先连接 JAKA")
            return
        self._append_log(["已点击法兰找平：等待安全确认"])
        if not messagebox.askyesno(
            "确认法兰找平",
            "将保持法兰当前位置，只旋转到平行基座 XOY。\n"
            "请确认夹爪和相机周围无障碍，且机械臂已在安全高度。",
            parent=self.root,
        ):
            self._append_log(["法兰找平已取消"])
            return
        self._append_log(["法兰找平已发送：正在计算 IK 与安全路径……"])
        self.controller.level_flange(0.12)

    def _save_position(self) -> None:
        snapshot = self._last_robot_snapshot
        if snapshot is None or not snapshot.connected or snapshot.joints_rad is None or snapshot.tcp_pose is None:
            self._error_text.set("无法保存：等待完整 JAKA 关节与 TCP 状态")
            return
        payload = self._load_config_payload()
        payload["saved_position"] = {
            "joint_positions_rad": [float(value) for value in snapshot.joints_rad],
            "tcp_pose_mm_rad": [float(value) for value in snapshot.tcp_pose],
            "tool_id": snapshot.tool_id,
            "saved_at_ns": time.time_ns(),
        }
        self._write_config_payload(payload)
        self._error_text.set("")
        self._append_log(["已保存当前位置：J1-J6、TCP 与 Tool ID 已写入 jaka_jog.json"])

    def _restore_position(self, *, confirm: bool = True) -> None:
        if self.controller is None:
            self._error_text.set("请先连接 JAKA")
            return
        payload = self._load_config_payload()
        saved = payload.get("saved_position")
        if not isinstance(saved, dict) or not isinstance(saved.get("joint_positions_rad"), list):
            self._error_text.set("尚未保存位置")
            return
        try:
            joints = tuple(float(value) for value in saved["joint_positions_rad"])
            if len(joints) != 6:
                raise ValueError
        except (TypeError, ValueError):
            self._error_text.set("保存的位置数据无效")
            return
        if confirm and not messagebox.askyesno(
            "确认复原位置",
            "机械臂将以关节运动复原到已保存的位置。\n"
            "请确认路径与周围无障碍。",
            parent=self.root,
        ):
            return
        tool_id = saved.get("tool_id")
        if isinstance(tool_id, int):
            self.controller.set_tool_id(tool_id)
        self.controller.restore_joints(joints, self.RESTORE_SPEED_RAD_S)

    def _refresh_recording(self, snapshot: object, gripper: object) -> None:
        if self.demonstration is None:
            return
        if not self._cameras_enabled:
            self._record_status_var.set("相机已关闭；JAKA Jog 与夹爪控制仍可使用")
            return
        status = self.demonstration.drain_events()
        if self._record_notice is not None:
            message, expires_s = self._record_notice
            if time.monotonic() < expires_s:
                self._record_status_var.set(message)
                return
            self._record_notice = None
        if status.phase == "recording":
            ready, readiness, aligned = self.demonstration.readiness()
            robot_gap = aligned.software_time_delta_ms
            global_gap = aligned.global_camera_delta_ms
            sync = (
                f"ZED-JAKA {robot_gap:.1f} ms" if robot_gap is not None else "ZED-JAKA —"
            ) + (
                f" | ZED-手机 {global_gap:.1f} ms" if global_gap is not None else " | ZED-手机 —"
            )
            diagnostics = (
                f" | 诊断：{self.demonstration.sampling_diagnostics()}"
                if status.skipped_samples
                else ""
            )
            self._record_status_var.set(
                f"录制中：{self._active_record_hz:g} Hz，{status.step_count} steps，跳过 {status.skipped_samples}，{sync}"
                + " | JAKA 单SDK采集"
                + diagnostics
                + ("" if ready else f"（{readiness}）")
            )
        elif status.phase == "idle":
            ready, readiness, aligned = self.demonstration.readiness()
            if ready:
                self._record_status_var.set("采集源已就绪；可点击“开始录制” | JAKA 单SDK采集")
            else:
                errors = []
                if aligned.camera_error:
                    errors.append(f"ZED: {aligned.camera_error}")
                if aligned.global_camera_error:
                    errors.append(f"手机: {aligned.global_camera_error}")
                if aligned.robot_error:
                    errors.append(f"JAKA: {aligned.robot_error}")
                detail = f"（{' | '.join(errors)}）" if errors else ""
                self._record_status_var.set(f"采集源未就绪：{readiness}{detail} | JAKA 单SDK采集")
        else:
            extra = f"  保存于 {status.result_path}" if status.result_path else ""
            error = f"：{status.error}" if status.error else ""
            export = ""
            if status.phase == "finished" and status.result_path is not None:
                self._start_evo1_export(status.result_path)
                export = " | " + self._poll_evo1_export(status.result_path)
            self._record_status_var.set(status.message + error + extra + export)

    def _disconnect(self) -> None:
        self._quest_disarm()
        if self.demonstration is not None:
            self.demonstration.reset_robot_bridge()
        if self.controller is not None:
            self.controller.logout()

    def _emergency_stop(self) -> None:
        self._quest_disarm()
        if self.controller is not None:
            self.controller.stop_jog()
            self.controller.disable()
        if self.gripper_controller is not None:
            self.gripper_controller.soft_stop()
        self._pressed.clear()

    def _quest_fresh(self) -> bool:
        frame = self._quest_latest_frame
        return (frame is not None and frame.connected and frame.tracked and frame.valid
                and (not self._quest_rotation_enabled.get() or frame.rotation_valid)
                and not self._quest_receiver.error
                and time.monotonic() - frame.received_s <= self._quest_timeout_s)

    def _quest_arm(self) -> None:
        """Arm input only; Grip still supplies the separate dead-man action."""

        snapshot = self._latest_robot_snapshot
        if not self._quest_fresh():
            self._quest_status.set("Quest：输入未连接、未追踪或已超时，拒绝 ARM")
            return
        if not self._quest_heading_locked:
            self._quest_status.set("Quest：请先锁定当前 VR 正前方")
            return
        if self._quest_latest_frame.grip > float(self._quest_config.get("grip_off", 0.55)):
            self._quest_require_grip_release = True
            self._quest_status.set("Quest：请先松开 Grip，再 ARM")
            return
        if not self._quest_confirm.get():
            self._quest_status.set("Quest：请先勾选现场安全确认")
            return
        if snapshot is None or not getattr(snapshot, "connected", False):
            self._quest_status.set("Quest：JAKA 未连接")
            return
        if not getattr(snapshot, "powered_on", False) or not getattr(snapshot, "enabled", False):
            self._quest_status.set("Quest：请由操作员先完成上电和使能")
            return
        if any(bool(getattr(snapshot, name, False)) for name in ("estop", "collision", "on_limit")):
            self._quest_status.set("Quest：检测到急停、碰撞或限位，拒绝 ARM")
            return
        self._quest_armed = True
        self._quest_status.set("Quest：已 ARM；按住 Grip 才启动相对 TCP 伺服")

    def _quest_disarm(self) -> None:
        self._quest_armed = False
        self._quest_stop_grip()
        self._quest_status.set("Quest：已解除；未进入 / 已退出伺服")

    def _quest_stop_grip(self) -> None:
        """Exit only the current Grip servo session; preserve master ARM."""

        was_active = self._quest_servo_active
        self._quest_servo_active = False
        self._quest_tcp_reference = None
        self._quest_latest_delta_mm = (0.0, 0.0, 0.0)
        self._quest_last_sent_target = None
        self._quest_last_target_sent_s = 0.0
        self._quest_tracker.reset_grip()
        if was_active and self.controller is not None:
            self.controller.stop_cartesian_servo()

    def _quest_lock_heading(self) -> None:
        """Capture current HMD yaw as the fixed hand-control coordinate frame."""

        frame = self._quest_latest_frame
        if not self._quest_fresh() or not frame.head_rotation_valid:
            self._quest_status.set("Quest：未收到头显朝向；请等待 Unity version 2 UDP")
            return
        if self._quest_servo_active or frame.grip > float(self._quest_config.get("grip_off", 0.55)):
            self._quest_status.set("Quest：请先松开 Grip，再锁定当前 VR 正前方")
            return
        try:
            yaw_deg = self._quest_tracker.lock_heading(frame.head_rotation_xyzw)
        except ValueError as error:
            self._quest_status.set(f"Quest：正前方锁定失败：{error}")
            return
        self._quest_status.set(f"Quest：当前 VR 正前方已锁定（头显 yaw={yaw_deg:+.1f}°）；ARM 状态保持")
        self._quest_heading_locked = True

    def _show_quest_mapping(self) -> None:
        window = tk.Toplevel(self.root)
        window.title("Quest 手柄方向映射")
        window.resizable(False, False)
        ttk.Label(window, text="把 Unity 右手柄方向映射到 JAKA 基坐标轴。六个方向必须一一对应。", padding=10).grid(row=0, column=0, columnspan=2, sticky="w")
        selections: dict[str, tk.StringVar] = {}
        for row, direction in enumerate(HAND_DIRECTIONS, start=1):
            value = tk.StringVar(value=self._quest_mapping.get(direction, ""))
            selections[direction] = value
            ttk.Label(window, text=f"手柄 {direction}:", padding=5).grid(row=row, column=0, sticky="w")
            ttk.Combobox(window, textvariable=value, values=JAKA_DIRECTIONS, state="readonly", width=8).grid(row=row, column=1, padx=8, sticky="w")

        def save() -> None:
            mapping = {name: variable.get() for name, variable in selections.items()}
            try:
                self._quest_tracker = RelativeQuestTracker(
                    mapping,
                    grip_on=float(self._quest_config.get("grip_on", 0.70)), grip_off=float(self._quest_config.get("grip_off", 0.55)),
                    trigger_on=float(self._quest_config.get("trigger_on", 0.70)), trigger_off=float(self._quest_config.get("trigger_off", 0.20)),
                )
            except ValueError as error:
                messagebox.showerror("方向映射无效", str(error), parent=window)
                return
            # Mapping is part of the live coordinate transform.  Never swap
            # it while an old Grip reference or servo target can still exist.
            self._quest_disarm()
            self._quest_heading_locked = False
            self._quest_require_grip_release = True
            self._quest_mapping = mapping
            self._persist_ui_state()
            self._quest_status.set("Quest：方向映射已保存；请松开 Grip、重新锁定 VR 正前方，再 ARM")
            window.destroy()

        ttk.Button(window, text="保存", command=save).grid(row=len(HAND_DIRECTIONS) + 1, column=0, pady=10)
        ttk.Button(window, text="取消", command=window.destroy).grid(row=len(HAND_DIRECTIONS) + 1, column=1, pady=10)
        self._make_buttons_mouse_only(window)

    def _quest_tick(self) -> None:
        """Consume input frames in Tk; all physical calls still go via one SDK worker."""

        frame = None
        try:
            frame = self._quest_receiver.latest()
            now = time.monotonic()
            if frame is not None:
                self._quest_latest_frame = frame
                self._quest_last_frame_s = frame.received_s
                state = "已连接" if frame.connected else "未连接"
                tracking = "追踪正常" if frame.tracked and frame.valid else "追踪丢失"
                self._quest_status.set(f"Quest：{state}，{tracking}" + ("，已 ARM" if self._quest_armed else ""))
                self._quest_detail.set(
                    f"位置[m] X={frame.position_m[0]:+.3f} Y={frame.position_m[1]:+.3f} Z={frame.position_m[2]:+.3f}\n"
                    f"Grip={frame.grip:.2f}  Trigger={frame.trigger:.2f}  Stick={int(frame.thumbstick_click)}  "
                    f"旋转={'正常' if frame.rotation_valid else '不可用（平移仍可用）'}  "
                    f"头显={'正常' if frame.head_rotation_valid else '无（无法锁定正前方）'}\n"
                    f"按键 A={int(frame.button_a)} B={int(frame.button_b)} X={int(frame.button_x)} Y={int(frame.button_y)}"
                )
                if (not (frame.connected and frame.tracked and frame.valid)
                        or (self._quest_rotation_enabled.get() and not frame.rotation_valid)):
                    if self._quest_armed or self._quest_servo_active:
                        self._quest_disarm()
                        self._quest_require_grip_release = True
                        self._quest_status.set("Quest：追踪丢失，已解除 ARM；恢复后松开 Grip 再 ARM")
                    return
                if self._quest_require_grip_release:
                    if frame.grip > float(self._quest_config.get("grip_off", 0.55)):
                        self._quest_status.set("Quest：方向已更新；请先松开 Grip，当前输入已忽略")
                        return
                    self._quest_require_grip_release = False
                    self._quest_status.set("Quest：Grip 已释放；可重新 ARM")
                for event, value in self._quest_tracker.update(frame, float(self._quest_sensitivity.get())):
                    if event == "record_start":
                        self._record_start()
                    elif event == "record_success":
                        self._record_finish(True)
                    elif event == "record_cancel":
                        self._record_cancel()
                    elif event == "restore_saved_position":
                        if not self._quest_confirm.get():
                            self._quest_status.set("Quest：请先勾选现场安全确认，才能用 B 复原位置")
                        elif self._quest_servo_active:
                            self._quest_status.set("Quest：请先松开 Grip，再用 B 复原位置")
                        else:
                            # Physical B press is the explicit confirmation;
                            # reuse the same saved-joint restore implementation.
                            self._restore_position(confirm=False)
                    elif event == "stick_click":
                        self._quest_disarm() if self._quest_armed else self._quest_arm()
                    elif event == "gripper_close":
                        self._gripper_command("close")
                    elif event == "gripper_open":
                        self._gripper_command("open")
                    elif event == "grip_start":
                        snapshot = self._latest_robot_snapshot
                        pose = getattr(snapshot, "tcp_pose", None) if snapshot is not None else None
                        if self._quest_armed and pose is not None and self.controller is not None:
                            self._quest_tcp_reference = tuple(float(item) for item in pose)
                            self._quest_continuous_rotation = ContinuousRotation()
                            self._quest_last_sent_target = self._quest_tcp_reference
                            self._quest_last_target_sent_s = now
                            self.controller.start_cartesian_servo()
                            self._quest_servo_active = True
                            self._quest_status.set("Quest：Grip 已按住，笛卡尔伺服启动")
                        elif not self._quest_armed:
                            self._quest_status.set("Quest：未 ARM；Grip 已忽略")
                    elif event == "pose_delta" and self._quest_servo_active and self._quest_tcp_reference is not None and self.controller is not None:
                        raw_delta, rotation_delta = value
                        delta = tuple(float(item) for item in raw_delta)
                        if math.dist(delta, (0.0, 0.0, 0.0)) > self._quest_max_range_mm:
                            self._quest_status.set("Quest：相对位移超范围，已停止")
                            self._quest_disarm()
                        else:
                            reference = self._quest_tcp_reference
                            self._quest_latest_delta_mm = delta
                            target_rpy = tuple(reference[3:])
                            if self._quest_rotation_enabled.get() and frame.rotation_valid and rotation_delta is not None:
                                # Use exactly the same visible sensitivity as
                                # translation.  It is read per frame, so a GUI
                                # change takes effect while the bridge runs.
                                scaled = self._quest_continuous_rotation.update(
                                    rotation_delta, max(0.0, min(3.0, float(self._quest_sensitivity.get())))
                                )
                                raw_rpy = matrix_rpy(matmul(scaled, rpy_matrix(tuple(reference[3:]))))
                                previous_rpy = self._quest_last_sent_target[3:] if self._quest_last_sent_target is not None else reference[3:]
                                target_rpy = _unwrap_rpy_near(raw_rpy, tuple(previous_rpy))
                            self._quest_send_target((reference[0] + delta[0], reference[1] + delta[1], reference[2] + delta[2], *target_rpy))
                    elif event == "grip_stop" and self._quest_servo_active:
                        self._quest_stop_grip()
                        self._quest_status.set("Quest：Grip 已松开，本次伺服已退出；仍处于 ARM，可再次按住 Grip")
            elif self._quest_latest_frame is not None and now - self._quest_last_frame_s > self._quest_timeout_s:
                # 即使尚未 ARM，也要显示停止 Play 后的数据超时，不能保留旧的“已连接”。
                if self._quest_servo_active or self._quest_armed:
                    self._quest_disarm()
                    self._quest_require_grip_release = True
                self._quest_status.set("Quest：UDP 超时（Unity 已停止或数据中断）；已停止")
            if self._quest_receiver.error:
                self._quest_status.set(f"Quest UDP 错误：{self._quest_receiver.error}")
                if self._quest_servo_active or self._quest_armed:
                    self._quest_disarm()
                    self._quest_require_grip_release = True
                self._quest_status.set(f"Quest UDP 错误：{self._quest_receiver.error}；请排查发送端后重启 GUI")
        finally:
            if frame is not None and self._demo_recorder is not None and self._demo_recorder.active:
                try:
                    self._demo_recorder.append(frame, self._latest_robot_snapshot, self._quest_last_sent_target,
                                               self._quest_armed, self._quest_servo_active)
                except Exception as error:
                    self._record_status_var.set(f"DEMO 写入失败：{error}；请停止录制")
            if self.root.winfo_exists():
                self.root.after(20, self._quest_tick)

    def _quest_send_target(self, target: tuple[float, ...]) -> None:
        """Apply the stable bridge's per-frame limits, then send one full pose.

        There must be exactly one XYZ+RPY target for each controller frame.
        Do not independently rate-limit translation and rotation: doing so can
        alternate the reference RPY and rotated RPY and make the wrist shake.
        """

        if self.controller is None:
            return
        previous = self._quest_last_sent_target
        now = time.monotonic()
        if previous is not None:
            dt = max(1e-3, now - self._quest_last_target_sent_s)
            requested_distance = math.dist(target[:3], previous[:3])
            allowed_distance = self._quest_max_linear_speed_mm_s * dt
            if requested_distance > allowed_distance:
                ratio = allowed_distance / requested_distance if requested_distance > 1e-9 else 0.0
                limited_xyz = tuple(
                    previous[index] + (target[index] - previous[index]) * ratio
                    for index in range(3)
                )
                target = (*limited_xyz, *target[3:])
                self._quest_status.set(
                    f"Quest：手柄输入较快，目标已平滑限制为 {self._quest_max_linear_speed_mm_s:.0f} mm/s"
                )

            previous_rotation = rpy_matrix(tuple(previous[3:]))
            requested_rotation = rpy_matrix(tuple(target[3:]))
            rotation_step = matmul(requested_rotation, transpose(previous_rotation))
            rotation_angle = rotation_angle_rad(rotation_step)
            angular_speed = math.degrees(rotation_angle) / dt
            if angular_speed > self._quest_max_angular_speed_deg_s:
                limited_rotation = limit_rotation_step(
                    previous_rotation,
                    requested_rotation,
                    math.radians(self._quest_max_angular_speed_deg_s) * dt,
                )
                limited_rpy = _unwrap_rpy_near(matrix_rpy(limited_rotation), tuple(previous[3:]))
                target = (*target[:3], *limited_rpy)
                self._quest_status.set(
                    f"Quest：手柄旋转较快，目标已平滑限制为 {self._quest_max_angular_speed_deg_s:.0f}°/s"
                )
        self.controller.set_cartesian_servo_target(target)
        self._quest_last_sent_target = target
        self._quest_last_target_sent_s = now

    def _gripper_command(self, name: str) -> None:
        if self.gripper_controller is None:
            self._error_text.set("MISUMI 控制未启用；请检查 config/jaka_jog.json")
            return
        try:
            settings = self._gripper_settings()
        except (tk.TclError, ValueError) as error:
            self._error_text.set(f"夹爪参数无效：{error}")
            return
        if name == "open":
            self.gripper_controller.open(
                speed_percent=settings["speed_percent"],
                force_percent=settings["force_percent"],
            )
        elif name == "close":
            self.gripper_controller.close(
                closing_units=settings["close_position_units"],
                speed_percent=settings["speed_percent"],
                force_percent=settings["force_percent"],
            )
        else:
            getattr(self.gripper_controller, name)()

    def _show_key_settings(self) -> None:
        window = tk.Toplevel(self.root)
        window.title("配置按键")
        window.resizable(False, False)
        ttk.Label(window, text="点击“录入”，随后直接按一个键。重复按键会被拒绝。", padding=10).grid(row=0, column=0, columnspan=3, sticky="w")
        for row, action in enumerate(_BINDING_LABELS, start=1):
            value = tk.StringVar(value=self._key_label(self._key_bindings.get(action, "—")))
            ttk.Label(window, text=_BINDING_LABELS[action], padding=5).grid(row=row, column=0, sticky="w")
            ttk.Label(window, textvariable=value, width=12).grid(row=row, column=1, sticky="w")
            ttk.Button(window, text="录入", command=lambda a=action, v=value: self._capture_key(a, v)).grid(row=row, column=2, padx=5)
        ttk.Button(window, text="关闭", command=window.destroy).grid(row=len(_BINDING_LABELS) + 1, column=2, pady=8)
        self._make_buttons_mouse_only(window)

    def _capture_key(self, action: str, value: tk.StringVar) -> None:
        self._capturing_action = action
        value.set("请按键…")

    def _persist_bindings(self) -> None:
        payload = self._load_config_payload()
        payload["key_bindings"] = self._key_bindings
        self._write_config_payload(payload)

    def _load_config_payload(self) -> dict:
        return json.loads(self._config_path.read_text(encoding="utf-8")) if self._config_path.is_file() else {}

    def _write_config_payload(self, payload: dict) -> None:
        temporary = self._config_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self._config_path)

    def _persist_ui_state(self) -> None:
        payload = self._load_config_payload()
        jaka = dict(payload.get("jaka", {}))
        typed_host = self._ip_var.get().strip()
        if typed_host:
            jaka["host"] = typed_host
        payload["jaka"] = jaka
        jog = dict(payload.get("jog", {}))
        jog.update(speed_mm_s=float(self._speed_var.get()), step_mm=float(self._step_var.get()), coord=self._coord_var.get(), mode=self._mode_var.get())
        payload["jog"] = jog
        gripper = dict(payload.get("gripper", {}))
        try:
            gripper.update(self._gripper_settings())
        except (tk.TclError, ValueError):
            pass
        payload["gripper"] = gripper
        quest = dict(payload.get("quest_vr", {}))
        quest.update(
            translation_scale=float(self._quest_sensitivity.get()),
            direction_mapping=self._quest_mapping,
            rotation_enabled=bool(self._quest_rotation_enabled.get()),
            # Kept in the config for compatibility, but deliberately mirrors
            # the visible sensitivity instead of becoming a hidden multiplier.
            rotation_scale=float(self._quest_sensitivity.get()),
            packet_timeout_s=self._quest_timeout_s,
            max_relative_translation_mm=self._quest_max_range_mm,
            rotation_control_version=2,
            max_linear_speed_mm_s=self._quest_max_linear_speed_mm_s,
            max_angular_speed_deg_s=self._quest_max_angular_speed_deg_s,
        )
        payload["quest_vr"] = quest
        quest.pop("max_relative_rotation_deg", None)
        self._write_config_payload(payload)

    def _gripper_settings(self) -> dict[str, int]:
        closing_mm = float(self._gripper_close_mm_var.get())
        speed_percent = int(self._gripper_speed_var.get())
        force_percent = int(self._gripper_force_var.get())
        closing_units = int(round(closing_mm / self._gripper_unit_mm))
        if not 0 <= closing_units <= self._gripper_max_units:
            raise ValueError(f"闭合量必须在 0 到 {self._gripper_max_units * self._gripper_unit_mm:.2f} mm 之间")
        if not 1 <= speed_percent <= 100:
            raise ValueError("夹爪速度必须在 1 到 100% 之间")
        if not 20 <= force_percent <= 100:
            raise ValueError("夹爪力必须在 20 到 100% 之间")
        return {
            "close_position_units": closing_units,
            "speed_percent": speed_percent,
            "force_percent": force_percent,
        }

    def _persist_gripper_settings(self) -> None:
        try:
            settings = self._gripper_settings()
        except (tk.TclError, ValueError) as error:
            self._error_text.set(f"夹爪参数无效：{error}")
            return
        self._gripper.update(settings)
        if self.gripper_controller is not None:
            self.gripper_controller.config.update(settings)
        payload = self._load_config_payload()
        gripper = dict(payload.get("gripper", {}))
        gripper.update(settings)
        payload["gripper"] = gripper
        self._write_config_payload(payload)
        self._append_log([
            f"夹爪参数已保存：闭合量 {settings['close_position_units'] * self._gripper_unit_mm:.2f} mm，"
            f"速度 {settings['speed_percent']}%，力 {settings['force_percent']}%"
        ])

    def _on_tool_selected(self, _event: tk.Event | None = None) -> None:
        display = self._tool_var.get()
        tool_id = self._tool_by_display.get(display)
        self._selected_tool_id_for_relay = tool_id
        if tool_id is not None and self.controller is not None:
            self.controller.set_tool_id(tool_id)

    def _telemetry_relay_loop(self) -> None:
        """Drain measured hardware state independently of Tk rendering."""

        while not self._telemetry_relay_stop.is_set():
            controller = self.controller
            gripper_controller = self.gripper_controller
            try:
                robot = controller.get_snapshot() if controller is not None else None
                gripper = (
                    gripper_controller.get_snapshot()
                    if gripper_controller is not None
                    else None
                )
                self._latest_robot_snapshot = robot
                self._latest_gripper_snapshot = gripper
                if self.demonstration is not None:
                    # One SDK owner for control AND measured state. Never
                    # open a second JSON connection for recording: it can
                    # displace the controller's existing SDK connection.
                    measured = controller.take_telemetry() if controller is not None else []
                    self.demonstration.controller_telemetry_dropped = (
                        controller.telemetry_dropped if controller is not None else 0
                    )
                    for sample in measured:
                        matched_gripper = (
                            gripper_controller.snapshot_at(sample.timestamp_ns)
                            if gripper_controller is not None else None
                        )
                        self.demonstration.publish_gui_state(
                            sample, matched_gripper,
                            selected_tool_id=self._selected_tool_id_for_relay,
                        )
            except Exception:
                # A reconnect can replace/close a controller during one loop;
                # the next iteration automatically binds the new instance.
                pass
            # The JAKA child publishes at up to 30 Hz.  Drain its pipe much
            # faster than that so a short camera/UI scheduling pause cannot
            # fill the pipe and leave the recorder seeing an old snapshot.
            self._telemetry_relay_stop.wait(0.01)

    def _refresh_tool_profiles(self) -> None:
        if self.controller is not None:
            self.controller.read_tool_profiles()

    def _update_tool_profiles(self, profiles: list[dict]) -> None:
        """Replace configured tool labels with live controller TCP values."""

        normalized: list[tuple[int, str, tuple[float, ...]]] = []
        for profile in profiles:
            try:
                tool_id = int(profile["id"])
                tcp = tuple(float(value) for value in profile["tcp"])
            except (KeyError, TypeError, ValueError):
                continue
            if len(tcp) != 6:
                continue
            # Local names are intentional human labels (for_test, misumi,
            # etc.) and take precedence over SDK fallbacks.
            name = self._configured_tool_names.get(tool_id) or str(profile.get("name") or f"Tool {tool_id}")
            normalized.append((tool_id, name, tcp))
        signature = tuple(normalized)
        if not signature or signature == self._tool_profiles_signature:
            return
        self._tool_profiles_signature = signature
        self._tools = [(tool_id, name) for tool_id, name, _tcp in normalized]
        self._tool_display = [f"{name} (ID {tool_id})" for tool_id, name, _tcp in normalized]
        self._tool_by_display = {
            display: tool_id for display, (tool_id, _name, _tcp) in zip(self._tool_display, normalized)
        }
        active_id = self._last_tool_id
        selected_id = self._tool_by_display.get(self._tool_var.get())
        desired_id = active_id if active_id is not None else selected_id
        selected_display = next((display for display, tool_id in self._tool_by_display.items() if tool_id == desired_id), self._tool_display[0])
        self._tool_var.set(selected_display)
        if self._tool_box is not None:
            self._tool_box.configure(values=self._tool_display)

    def _jog_down(self, axis: int, direction: int) -> None:
        if self._quest_armed or self._quest_servo_active:
            self._quest_disarm()
        if self._mode_var.get() == "continuous":
            if self.controller is not None:
                self.controller.stop_jog()
        self._actuate(axis, direction)

    def _jog_up(self) -> None:
        if self._mode_var.get() == "continuous" and self.controller is not None:
            self.controller.stop_jog()

    def _actuate(self, axis: int, direction: int) -> None:
        if self.controller is None:
            return
        speed = float(self._speed_var.get())
        coord = COORD_TOOL if self._coord_var.get() == "tool" else COORD_BASE
        if self._mode_var.get() == "continuous":
            self.controller.start_jog(axis, direction, coord, speed)
        else:
            self.controller.step_jog(axis, direction, coord, speed, float(self._step_var.get()))

    def _bind_keys(self) -> None:
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.bind_all("<Button-1>", self._clear_input_focus, add="+")

    def _clear_input_focus(self, event: tk.Event) -> None:
        """Return keyboard jog control after a click outside an input widget."""

        widget = event.widget
        editable_classes = {"Entry", "TEntry", "Spinbox", "TSpinbox", "Text", "TCombobox"}
        # Some Tcl/Tk callbacks report the widget path as a string rather
        # than a widget instance.  Those clicks are safe to treat as outside
        # editable fields, and must not break the GUI event loop.
        if hasattr(widget, "winfo_class") and widget.winfo_class() in editable_classes:
            return
        # Run after the target click so a button still receives its command.
        self.root.after_idle(self.root.focus_set)

    def _on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if self._capturing_action is not None:
            action = self._capturing_action
            self._capturing_action = None
            if key in self._binding_by_key and self._binding_by_key[key] != action:
                self._error_text.set(f"按键 {self._key_label(key)} 已绑定给 {_BINDING_LABELS[self._binding_by_key[key]]}")
                return "break"
            self._key_bindings[action] = key
            self._rebuild_bindings()
            self._persist_bindings()
            self._append_log([f"键位保存：{_BINDING_LABELS[action]} = {self._key_label(key)}"])
            return "break"
        focused = self.root.focus_get()
        if focused is not None and focused.winfo_class() in {"Entry", "TEntry", "Text", "TCombobox"}:
            return
        if key in self._pressed:
            return
        action = self._binding_by_key.get(key)
        if action is None:
            return
        self._pressed[key] = action
        if action in _JOG_ACTIONS:
            axis, direction = _JOG_ACTIONS[action]
            self._jog_down(axis, direction)
        elif action == "gripper_open":
            self._gripper_command("open")
        elif action == "gripper_close":
            self._gripper_command("close")
        elif action == "gripper_soft_stop":
            self._gripper_command("soft_stop")

    def _on_key_release(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if key not in self._pressed:
            return
        action = self._pressed.pop(key)
        if action in _JOG_ACTIONS:
            self._jog_up()

    # ------------------------------------------------------------ refresh
    def _refresh_loop(self) -> None:
        self._refresh()
        self.root.after(80, self._refresh_loop)

    def _refresh(self) -> None:
        # The relay continuously drains hardware pipes. Tk only renders the
        # latest copies, so preview/key events cannot stall Episode telemetry.
        snapshot = self._latest_robot_snapshot
        gripper = self._latest_gripper_snapshot
        self._last_robot_snapshot = snapshot
        self._refresh_recording(snapshot, gripper)
        self._update_camera_preview()
        if gripper is None:
            self._gripper_text.set("夹爪：未启用")
        elif not gripper.connected:
            self._gripper_text.set("夹爪：未连接" + (f" ({gripper.error})" if gripper.error else ""))
        else:
            opening = gripper.opening_mm if gripper.opening_mm is not None else float("nan")
            self._gripper_text.set(
                f"夹爪：开口 {opening:.2f} mm  使能 {gripper.enable_status}  "
                f"夹持 {gripper.holding_status}  故障 {gripper.fault}"
            )

        if snapshot is None or not snapshot.connected:
            self._status_text.set("未连接（演示模式）" if self.demo else "未连接")
            self._power_text.set("上电：—")
            self._enable_text.set("使能：—")
            self._moving_text.set("运动：—")
            self._tool_text.set("工具：—")
            self._safety_text.set("")
            self._error_text.set(snapshot.error if snapshot and snapshot.error else "")
            self._tcp_text.set("TCP: —")
            self._joints_text.set("关节: —")
            self._update_3d(self.HOME_QPOS)
            # Keep the short JAKA command/error history visible.  The gripper
            # emits telemetry frequently, so letting it occupy the tail of the
            # list used to hide a failed robot command immediately.
            self._append_log((gripper.log[-4:] if gripper else []) + (snapshot.log[-12:] if snapshot else []))
            return

        if self.demo:
            self._status_text.set("已连接（演示）")
        else:
            self._status_text.set(
                f"已连接 {self.host} · 型号：{self.robot_model_label}（配置）"
            )
        self._power_text.set(f"上电：{'● 是' if snapshot.powered_on else '○ 否'}")
        self._enable_text.set(f"使能：{'● 是' if snapshot.enabled else '○ 否'}")
        self._moving_text.set(f"运动：{'● 是' if snapshot.moving else '○ 否'}")
        self._error_text.set(snapshot.error)

        self._update_tool_profiles(snapshot.tool_profiles)

        tool_name = next((name for tool_id, name in self._tools if tool_id == snapshot.tool_id), None)
        self._tool_text.set(f"工具：{tool_name} (ID {snapshot.tool_id})" if tool_name else (f"工具：ID {snapshot.tool_id}" if snapshot.tool_id is not None else "工具：—"))
        if snapshot.tool_id is not None and snapshot.tool_id != self._last_tool_id:
            self._last_tool_id = snapshot.tool_id
            display = next((d for d, tid in self._tool_by_display.items() if tid == snapshot.tool_id), None)
            if display:
                self._tool_var.set(display)

        safety_parts = []
        if snapshot.estop:
            safety_parts.append("急停触发")
        if snapshot.collision:
            safety_parts.append("碰撞")
        if snapshot.on_limit:
            safety_parts.append("限位")
        self._safety_text.set("⚠ " + "/".join(safety_parts) if safety_parts else "")

        if snapshot.tcp_pose is not None:
            x, y, z, rx, ry, rz = snapshot.tcp_pose
            self._tcp_text.set(
                f"TCP: X {x:7.1f}  Y {y:7.1f}  Z {z:7.1f} mm   "
                f"RX {math.degrees(rx):6.1f}  RY {math.degrees(ry):6.1f}  RZ {math.degrees(rz):6.1f}°"
            )
        if snapshot.joints_rad is not None:
            deg = [math.degrees(value) for value in snapshot.joints_rad]
            self._joints_text.set("  |  " + "  ".join(f"J{i+1} {value:6.1f}°" for i, value in enumerate(deg)))

        joints = snapshot.joints_rad or self.HOME_QPOS
        self._update_3d(joints)
        # See the explanation above: robot messages take precedence over the
        # high-rate MISUMI status stream in the visible console.
        self._append_log((gripper.log[-4:] if gripper else []) + snapshot.log[-12:])

    def _update_3d(self, joints: tuple[float, ...]) -> None:
        if getattr(self, "_view", None) != "matplotlib":
            return
        try:
            points = self.kinematics.forward(joints)
        except Exception:
            return
        xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
        self._arm_line.set_data(xs, ys)
        self._arm_line.set_3d_properties(zs)
        self._tcp_marker.set_data([xs[-1]], [ys[-1]])
        self._tcp_marker.set_3d_properties([zs[-1]])
        self._canvas.draw_idle()

    def _update_camera_preview(self) -> None:
        if getattr(self, "_view", None) != "camera_preview" or self.demonstration is None:
            return
        if not self._cameras_enabled:
            self._clear_camera_preview("相机已关闭：设备已释放给 ACT 推理")
            return
        snapshot = self.demonstration.preview_snapshot()
        zed_name = (
            f"ZED Mini（录制 RGB：{snapshot.camera.camera_name}）"
            if snapshot.camera is not None
            else "ZED Mini（录制 RGB）"
        )
        global_name = (
            f"DroidCam（全局 RGB：{snapshot.global_camera.camera_name}）"
            if snapshot.global_camera is not None
            else "DroidCam（全局 RGB）"
        )
        self._set_preview_image(
            self._zed_preview,
            snapshot.camera.image_bgr if snapshot.camera is not None else None,
            self._zed_preview_frame,
            zed_name,
            snapshot.camera.frame_id if snapshot.camera is not None else None,
        )
        self._set_preview_image(
            self._global_preview,
            snapshot.global_camera.image_bgr if snapshot.global_camera is not None else None,
            self._global_preview_frame,
            global_name,
            snapshot.global_camera.frame_id if snapshot.global_camera is not None else None,
        )

    def _clear_camera_preview(self, message: str) -> None:
        """Show an intentionally black preview after camera handles are released."""

        if not hasattr(self, "_zed_preview"):
            return
        for widget in (self._zed_preview, self._global_preview):
            widget.configure(image="", text=message)
            widget.image = None

    def _set_preview_image(
        self,
        widget: ttk.Label,
        image_bgr: np.ndarray | None,
        frame: ttk.LabelFrame,
        name: str,
        frame_id: int | None,
    ) -> None:
        if image_bgr is None or image_bgr.size == 0:
            widget.configure(image="", text=f"等待{name}画面…")
            return
        try:
            from PIL import Image, ImageTk

            height, width = image_bgr.shape[:2]
            container_width = max(640, int(self._preview_container.winfo_width()))
            panel_width = max(260, (container_width - 24) // 2)
            panel_height = max(210, int(self._preview_container.winfo_height()) - 42)
            scale = min(panel_width / width, panel_height / height, 1.0)
            output_size = (max(1, round(width * scale)), max(1, round(height * scale)))
            image_rgb = np.ascontiguousarray(image_bgr[:, :, :3][:, :, ::-1])
            photo = ImageTk.PhotoImage(Image.fromarray(image_rgb).resize(output_size, Image.Resampling.LANCZOS))
            widget.configure(image=photo, text="")
            widget.image = photo  # keep the Tk image alive
            frame.configure(text=f"{name}   frame {frame_id if frame_id is not None else '—'}")
        except Exception as error:
            widget.configure(image="", text=f"预览失败：{error}")

    def _append_log(self, lines: list[str]) -> None:
        self._log.configure(state="normal")
        self._log.delete("1.0", tk.END)
        self._log.insert(tk.END, "\n".join(lines[-12:]))
        self._log.configure(state="disabled")

    def _on_close(self) -> None:
        if hasattr(self, "_hik_preview"):
            self._hik_preview.close()
        if self._demo_recorder is not None and self._demo_recorder.active:
            try:
                self._demo_recorder.finish("interrupted_by_window_close")
            except OSError as error:
                print(f"DEMO 录制收尾失败（已保存帧仍保留）：{error}", file=sys.stderr)
        self._persist_ui_state()
        self._quest_disarm()
        self._quest_receiver.close()
        self._telemetry_relay_stop.set()
        if self._telemetry_relay_thread is not None:
            self._telemetry_relay_thread.join(0.5)
        if self.demonstration is not None:
            self.demonstration.stop()
        if self.controller is not None:
            self.controller.stop_jog()
            self.controller.shutdown()
        if self.gripper_controller is not None:
            self.gripper_controller.shutdown()
        self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--recorder-config", type=Path, default=default_monitor_config_path())
    parser.add_argument("--host", default="10.5.5.100")
    parser.add_argument("--sdk-directory", default=SDK_DIRECTORY_DEFAULT)
    execution_mode = parser.add_mutually_exclusive_group()
    execution_mode.add_argument(
        "--demo",
        dest="demo",
        action="store_true",
        help="simulate robot and gripper; physical camera preview requires an explicit GUI action",
    )
    execution_mode.add_argument(
        "--live",
        dest="demo",
        action="store_false",
        help="explicitly allow connection to configured physical hardware",
    )
    parser.set_defaults(demo=True)
    parser.add_argument("--speed", type=float, default=15.0)
    parser.add_argument("--step", type=float, default=2.0)
    parser.add_argument("--coord", choices=("base", "tool"), default="base")
    parser.add_argument("--mode", choices=("step", "continuous"), default="step")
    parser.add_argument("--poll-hz", type=float, default=15.0)
    parser.add_argument(
        "--record-format",
        choices=("raw", "raw+evo1"),
        default="raw",
        help="raw keeps only vla_lab.episode.v1; raw+evo1 also exports a WSL Evo-1 dataset after a successful recording",
    )
    parser.add_argument(
        "--evo1-output",
        default="/home/yufeng/datasets/jaka_evo1",
        help="WSL output root used only with --record-format raw+evo1",
    )
    return parser


def _default_config_path() -> Path:
    return Path(__file__).resolve().parents[2] / "config" / "jaka_jog.json"


def _load_config(path: Path | None) -> dict:
    config_path = path or _default_config_path()
    if not Path(config_path).is_file():
        return {}
    return json.loads(Path(config_path).read_text(encoding="utf-8"))


def _parse_keymap(payload: dict) -> dict[str, tuple[int, int]]:
    keymap: dict[str, tuple[int, int]] = {}
    for key, value in payload.items():
        if not isinstance(value, dict):
            continue
        axis = int(value.get("axis", -1))
        direction = int(value.get("direction", 0))
        if 0 <= axis <= 2 and direction in (-1, 1):
            keymap[key.lower()] = (axis, direction)
    return keymap


def _parse_bindings(payload: object, fallback: dict[str, str]) -> dict[str, str]:
    if not isinstance(payload, dict):
        return dict(fallback)
    bindings = dict(fallback)
    for action, key in payload.items():
        if action in _BINDING_LABELS and isinstance(key, str) and key.strip():
            bindings[action] = key.strip().lower()
    return bindings


def _parse_tools(payload: list) -> list[tuple[int, str]]:
    tools: list[tuple[int, str]] = []
    for item in payload:
        if isinstance(item, dict) and "id" in item:
            tools.append((int(item["id"]), str(item.get("name", f"工具 {item['id']}"))))
    return tools


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = _load_config(args.config)
    jaka = config.get("jaka", {})
    jog = config.get("jog", {})

    host = args.host if args.host != "10.5.5.100" else str(jaka.get("host", args.host))
    sdk_directory = (
        args.sdk_directory
        if args.sdk_directory != SDK_DIRECTORY_DEFAULT
        else str(jaka.get("sdk_directory", args.sdk_directory))
    )
    speed = args.speed if args.speed != 15.0 else float(jog.get("speed_mm_s", args.speed))
    step = args.step if args.step != 2.0 else float(jog.get("step_mm", args.step))
    coord = args.coord if args.coord != "base" else str(jog.get("coord", args.coord))
    mode = args.mode if args.mode != "step" else str(jog.get("mode", args.mode))
    poll_hz = args.poll_hz if args.poll_hz != 15.0 else float(jaka.get("poll_hz", args.poll_hz))
    robot_model_label = str(jaka.get("model_label", "JAKA S5"))

    default_keymap = {
        "w": {"axis": 1, "direction": 1},
        "s": {"axis": 1, "direction": -1},
        "a": {"axis": 0, "direction": -1},
        "d": {"axis": 0, "direction": 1},
        "e": {"axis": 2, "direction": 1},
        "q": {"axis": 2, "direction": -1},
        "left": {"axis": 0, "direction": -1},
        "right": {"axis": 0, "direction": 1},
        "up": {"axis": 1, "direction": 1},
        "down": {"axis": 1, "direction": -1},
        "prior": {"axis": 2, "direction": 1},
        "next": {"axis": 2, "direction": -1},
    }
    keymap = _parse_keymap(config.get("keys", default_keymap))
    default_bindings = {
        "jog_x_neg": "a", "jog_x_pos": "d", "jog_y_neg": "s", "jog_y_pos": "w",
        "jog_z_neg": "q", "jog_z_pos": "e", "jog_rz_pos": "z", "jog_rz_neg": "c", "gripper_open": "o",
        "gripper_close": "p", "gripper_soft_stop": "f12",
    }
    configured_bindings = config.get("key_bindings", {})
    for key, (axis, direction) in keymap.items():
        action = next((name for name, pair in _JOG_ACTIONS.items() if pair == (axis, direction)), None)
        if action is not None and action not in configured_bindings:
            default_bindings[action] = key
    key_bindings = _parse_bindings(configured_bindings, default_bindings)

    default_tools = [{"id": 0, "name": "法兰(flange)"}, {"id": 8, "name": "for_test"}]
    tools = _parse_tools(config.get("tools", default_tools))
    gripper = config.get("gripper", {"enabled": False})
    default_tool_id = int(jog.get("default_tool_id", tools[0][0] if tools else 0))

    root = tk.Tk()
    JogApplication(
        root,
        host=host,
        sdk_directory=sdk_directory,
        demo=args.demo,
        speed=speed,
        step=step,
        coord=coord,
        mode=mode,
        poll_hz=poll_hz,
        robot_model_label=robot_model_label,
        key_bindings=key_bindings,
        config_path=args.config or _default_config_path(),
        tools=tools,
        gripper=gripper,
        recorder_config_path=args.recorder_config,
        record_format=args.record_format,
        evo1_output=args.evo1_output,
        default_tool_id=default_tool_id,
    )
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
