"""Quest -> JAKA 第一次真机微动调试窗口。

本入口故意不是完整遥操作器：只允许右手柄相对位移映射为 TCP 平移，
姿态冻结、夹爪禁用、快换禁用，也绝不代替操作者给机器人上电或使能。
"""

from __future__ import annotations

import argparse
import json
import math
import time
import tkinter as tk
import uuid
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

from .jaka_jog_controller import (
    COMMISSION_MAX_RELATIVE_MM,
    COMMISSION_MAX_SESSION_S,
    COMMISSION_MAX_SPEED_MM_S,
    COMMISSION_REQUIRED_TOOL_ID,
    COMMISSION_TARGET_WATCHDOG_S,
    JakaJogController,
    SHOWCASE_MAX_RELATIVE_MM,
    SHOWCASE_MAX_SESSION_S,
    SHOWCASE_MAX_SPEED_MM_S,
    SHOWCASE_REQUIRED_TOOL_ID,
    SHOWCASE_TARGET_WATCHDOG_S,
)
from .jaka_telemetry import SDK_DIRECTORY, validate_host
from .quest_vr_input import QuestFrame, QuestUdpReceiver, axis_map, matvec, quaternion_matrix


ROOT = Path(__file__).resolve().parents[3]
CONFIG_PATH = ROOT / "Python" / "config" / "jaka_jog.json"
SESSION_ROOT = ROOT / "Validation" / "live_micro_teleop_sessions"

# 界面先在 4 mm 处饱和，给 SDK 子进程的 5 mm 硬边界保留测量抖动余量。
UI_MAX_RELATIVE_MM = 4.0
HAND_TO_ROBOT_SCALE = 0.10
SHOWCASE_UI_MAX_RELATIVE_MM = 48.0
SHOWCASE_HAND_TO_ROBOT_SCALE = 1.0
QUEST_PACKET_MAX_AGE_S = 0.20
ROBOT_SAMPLE_MAX_AGE_S = 0.50
DEADMAN_ON = 0.75
DEADMAN_OFF = 0.55


@dataclass(frozen=True)
class GuardedTeleopProfile:
    key: str
    title: str
    ui_radius_mm: float
    worker_radius_mm: float
    speed_mm_s: float
    watchdog_s: float
    session_s: float
    hand_scale: float
    confirmation: str
    required_tool_id: int = 0


MICRO_PROFILE = GuardedTeleopProfile(
    key="micro", title="5 mm 微动调试",
    ui_radius_mm=UI_MAX_RELATIVE_MM,
    worker_radius_mm=COMMISSION_MAX_RELATIVE_MM,
    speed_mm_s=COMMISSION_MAX_SPEED_MM_S,
    watchdog_s=COMMISSION_TARGET_WATCHDOG_S,
    session_s=COMMISSION_MAX_SESSION_S,
    hand_scale=HAND_TO_ROBOT_SCALE,
    confirmation="5MM",
    required_tool_id=COMMISSION_REQUIRED_TOOL_ID,
)

SHOWCASE_PROFILE = GuardedTeleopProfile(
    key="showcase", title="约 5 cm 展示遥操作",
    ui_radius_mm=SHOWCASE_UI_MAX_RELATIVE_MM,
    worker_radius_mm=SHOWCASE_MAX_RELATIVE_MM,
    speed_mm_s=20.0,
    watchdog_s=SHOWCASE_TARGET_WATCHDOG_S,
    session_s=SHOWCASE_MAX_SESSION_S,
    hand_scale=SHOWCASE_HAND_TO_ROBOT_SCALE,
    confirmation="5CM",
    required_tool_id=SHOWCASE_REQUIRED_TOOL_ID,
)


def clamp_vector(vector: tuple[float, float, float], radius: float) -> tuple[float, float, float]:
    """把三维向量限制在球形工作区内，而不是逐轴裁剪成方盒。"""

    length = math.dist(vector, (0.0, 0.0, 0.0))
    if length <= float(radius) or length <= 1e-12:
        return vector
    ratio = float(radius) / length
    return tuple(value * ratio for value in vector)


def project_single_axis(
    vector: tuple[float, float, float],
    axis_mode: str,
) -> tuple[float, float, float]:
    """For acceptance runs, guarantee that only the selected robot axis changes."""

    if axis_mode == "free":
        return vector
    if axis_mode not in ("x", "y", "z"):
        raise ValueError(f"unknown axis mode: {axis_mode}")
    selected = {"x": 0, "y": 1, "z": 2}[axis_mode]
    return tuple(value if index == selected else 0.0 for index, value in enumerate(vector))


def translation_only_target(
    reference_tcp: tuple[float, ...],
    delta_xyz_mm: tuple[float, float, float],
    radius_mm: float = UI_MAX_RELATIVE_MM,
) -> tuple[float, ...]:
    """只改变 XYZ；RX/RY/RZ 永远取启动瞬间的机器人反馈。"""

    if len(reference_tcp) != 6:
        raise ValueError("reference TCP must contain six values")
    delta = clamp_vector(delta_xyz_mm, radius_mm)
    return (
        reference_tcp[0] + delta[0],
        reference_tcp[1] + delta[1],
        reference_tcp[2] + delta[2],
        reference_tcp[3], reference_tcp[4], reference_tcp[5],
    )


def dual_deadman_pressed(frame: QuestFrame, *, already_active: bool) -> bool:
    """Grip 与 Trigger 必须同时按住；释放任意一个即返回 False。"""

    threshold = DEADMAN_OFF if already_active else DEADMAN_ON
    return bool(frame.grip >= threshold and frame.trigger >= threshold)


class MicroMotionMapper:
    """把锁定后的头显水平坐标系映射到 JAKA 基坐标平移。"""

    def __init__(self, mapping: dict[str, str]) -> None:
        self._axis_map = axis_map(mapping)
        self._heading = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        self._hand_reference: tuple[float, float, float] | None = None

    def lock_heading(self, head_rotation_xyzw: tuple[float, float, float, float]) -> float:
        rotation = quaternion_matrix(head_rotation_xyzw)
        forward_x, forward_z = rotation[0][2], rotation[2][2]
        length = math.hypot(forward_x, forward_z)
        if length < 1e-6:
            raise ValueError("头显正前方接近竖直，无法锁定水平朝向")
        forward = (forward_x / length, 0.0, forward_z / length)
        right = (forward[2], 0.0, -forward[0])
        self._heading = (right, (0.0, 1.0, 0.0), forward)
        self._hand_reference = None
        return math.degrees(math.atan2(forward[0], forward[2]))

    def begin(self, hand_position_m: tuple[float, float, float]) -> None:
        self._hand_reference = tuple(float(value) for value in hand_position_m)

    def reset(self) -> None:
        self._hand_reference = None

    def delta_mm(
        self,
        hand_position_m: tuple[float, float, float],
        *,
        hand_scale: float = HAND_TO_ROBOT_SCALE,
        radius_mm: float = UI_MAX_RELATIVE_MM,
    ) -> tuple[float, float, float]:
        if self._hand_reference is None:
            raise RuntimeError("手柄参考点尚未建立")
        world_delta = tuple(
            float(hand_position_m[index]) - self._hand_reference[index]
            for index in range(3)
        )
        heading_delta = matvec(self._heading, world_delta)
        mapped = matvec(self._axis_map, heading_delta)
        raw_mm = tuple(value * 1000.0 * float(hand_scale) for value in mapped)
        return clamp_vector(raw_mm, radius_mm)


class SessionLog:
    """为每次 ARM 保存可审计事件；只写 D: 复现目录。"""

    def __init__(self) -> None:
        self.path: Path | None = None
        self.events = None
        self.meta: dict = {}

    def start(
        self,
        *,
        mode: str,
        host: str,
        profile: GuardedTeleopProfile,
        reference_tcp: tuple[float, ...] | None = None,
    ) -> None:
        SESSION_ROOT.mkdir(parents=True, exist_ok=True)
        self.path = SESSION_ROOT / (
            datetime.now().strftime(f"{profile.key}_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
        )
        self.path.mkdir(parents=True, exist_ok=False)
        self.events = (self.path / "events.jsonl").open("a", encoding="utf-8")
        self.meta = {
            "schema": "quest_jaka_guarded_teleop.v2",
            "mode": mode,
            "profile": profile.key,
            "host": host,
            "started_at": datetime.now().astimezone().isoformat(),
            "limits": {
                "worker_radius_mm": profile.worker_radius_mm,
                "ui_radius_mm": profile.ui_radius_mm,
                "speed_mm_s": profile.speed_mm_s,
                "watchdog_s": profile.watchdog_s,
                "session_s": profile.session_s,
                "tool_id": profile.required_tool_id,
                "rotation_enabled": False,
                "gripper_enabled": False,
            },
            "reference_tcp_mm_rad": list(reference_tcp) if reference_tcp else None,
            "outcome": "armed",
        }
        self.write("armed")

    def write(self, event: str, **values) -> None:
        if self.events is None:
            return
        row = {"time_ns": time.time_ns(), "event": event, **values}
        self.events.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.events.flush()

    def finish(self, outcome: str) -> Path | None:
        if self.path is None:
            return None
        self.write("finished", outcome=outcome)
        if self.events is not None:
            self.events.close()
        self.events = None
        self.meta["finished_at"] = datetime.now().astimezone().isoformat()
        self.meta["outcome"] = outcome
        (self.path / "metadata.json").write_text(
            json.dumps(self.meta, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        result, self.path = self.path, None
        self.meta = {}
        return result


class MicroTeleopWindow:
    def __init__(
        self,
        root: tk.Tk,
        *,
        live: bool,
        host: str,
        quest_port: int = 5005,
        profile: GuardedTeleopProfile = MICRO_PROFILE,
    ) -> None:
        self.root = root
        self.live = bool(live)
        self.profile = profile
        self.host = tk.StringVar(value=host)
        config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))["quest_vr"]
        self.mapper = MicroMotionMapper(config["direction_mapping"])
        self.quest = QuestUdpReceiver("127.0.0.1", int(quest_port))
        self.controller = JakaJogController(
            host=host,
            sdk_directory=SDK_DIRECTORY,
            poll_hz=30.0,
            demo=not self.live,
        )
        self.latest_frame: QuestFrame | None = None
        self.heading_locked = False
        self.armed = False
        self.active = False
        self.reference_tcp: tuple[float, ...] | None = None
        self.session_deadline_s = 0.0
        self.log = SessionLog()
        self.last_logged_target_s = 0.0
        self.last_log_count = 0
        self.range_mm = tk.DoubleVar(value=self.profile.ui_radius_mm)
        self.speed_mm_s = tk.DoubleVar(value=self.profile.speed_mm_s)
        self.axis_mode = tk.StringVar(value="free")
        self.axis_status = tk.StringVar(value="单轴验收：尚未开始")
        self.active_ui_radius_mm = self.profile.ui_radius_mm
        self.active_worker_radius_mm = self.profile.worker_radius_mm
        self.active_speed_mm_s = self.profile.speed_mm_s
        self.active_hand_scale = self.profile.hand_scale
        self.active_axis_mode = "free"
        self.axis_peak_mm = 0.0

        self.mode_text = "真机模式" if self.live else "模拟模式（不会连接真机）"
        self.robot_text = tk.StringVar(value="尚未连接")
        self.quest_text = tk.StringVar(value="等待 Quest UDP 127.0.0.1:5005")
        self.motion_text = tk.StringVar(value="未 ARM；机器人不会响应手柄")
        self.confirm_text = tk.StringVar(value="")
        self.checks = [tk.BooleanVar(value=False) for _ in range(4)]

        root.title(f"Quest → JAKA {self.profile.title} · {self.mode_text}")
        root.geometry("1080x900" if self.profile.key == "showcase" else "1040x760")
        root.minsize(940, 820 if self.profile.key == "showcase" else 680)
        outer = ttk.Frame(root, padding=12); outer.pack(fill="both", expand=True)
        banner_color = "#b42318" if self.live else "#146c43"
        ttk.Label(
            outer,
            text=(f"{self.mode_text}｜{self.profile.title}｜仅平移｜"
                  f"界面 {self.profile.ui_radius_mm:.0f} mm / 硬限制 {self.profile.worker_radius_mm:.0f} mm｜"
                  f"{self.profile.speed_mm_s:.0f} mm/s｜{self.profile.session_s:.0f} 秒｜双按钮保持"),
            foreground=banner_color,
            font=("Microsoft YaHei UI", 13, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            outer,
            text="本窗口不提供上电、使能、旋转、夹爪或快换控制。软件 STOP 不能替代控制柜物理急停。",
            foreground="#b42318",
        ).pack(anchor="w", pady=(2, 10))

        connection = ttk.LabelFrame(outer, text="1. 连接与实时状态", padding=10)
        connection.pack(fill="x")
        row = ttk.Frame(connection); row.pack(fill="x")
        ttk.Label(row, text="2 号控制柜 IP").pack(side="left")
        ttk.Entry(row, textvariable=self.host, width=18).pack(side="left", padx=6)
        ttk.Button(row, text="连接（不自动上电/使能）", command=self.connect).pack(side="left")
        ttk.Button(row, text="断开", command=self.disconnect).pack(side="left", padx=5)
        tk.Button(
            row, text="软件 STOP / 退出伺服", command=lambda: self.stop("operator_stop"),
            bg="#d92d20", fg="white", activebackground="#b42318", font=("Microsoft YaHei UI", 10, "bold"),
        ).pack(side="right")
        ttk.Label(connection, textvariable=self.robot_text, justify="left").pack(anchor="w", pady=5)
        ttk.Label(connection, textvariable=self.quest_text, justify="left").pack(anchor="w")

        gates = ttk.LabelFrame(outer, text=f"2. {self.profile.title}现场门槛", padding=10)
        gates.pack(fill="x", pady=10)
        labels = (
            "机器人及整套末端周围已清空，人员在运动范围之外",
            "现场监护人守在可用的物理急停旁（第一次真机测试）",
            "JAKA App 的安全速度限制已由现场负责人确认",
            (f"已确认当前 Tool ID = {self.profile.required_tool_id}，且整套末端有明显大于 "
             f"{self.profile.worker_radius_mm:.0f} mm 的净空"),
        )
        for variable, label in zip(self.checks, labels, strict=True):
            ttk.Checkbutton(gates, text=label, variable=variable).pack(anchor="w")
        confirm_row = ttk.Frame(gates); confirm_row.pack(fill="x", pady=(6, 0))
        ttk.Label(confirm_row, text=f"真机时输入 {self.profile.confirmation}：").pack(side="left")
        ttk.Entry(confirm_row, textvariable=self.confirm_text, width=12).pack(side="left")
        if not self.live:
            ttk.Label(confirm_row, text="模拟模式不要求现场勾选，仅用于熟悉流程", foreground="#146c43").pack(side="left", padx=8)

        controls = ttk.LabelFrame(outer, text="3. 锁定、ARM、双按钮保持", padding=10)
        controls.pack(fill="x")
        row = ttk.Frame(controls); row.pack(fill="x")
        ttk.Button(row, text="① 锁定当前头显正前方", command=self.lock_heading).pack(side="left")
        ttk.Button(row, text=f"② ARM 一次 {self.profile.session_s:.0f} 秒", command=self.arm).pack(side="left", padx=6)
        ttk.Label(
            controls,
            text="ARM 后先保持两个按钮松开。真正运动时同时按住右手 Grip（侧握键）+ Trigger（食指扳机）；松开任意一个立即停止并解除 ARM。",
            wraplength=950, justify="left",
        ).pack(anchor="w", pady=8)
        ttk.Label(
            controls, textvariable=self.motion_text, wraplength=950, justify="left",
            foreground="#7a2e0e", font=("Consolas", 11),
        ).pack(anchor="w")

        if self.profile.key == "showcase":
            tuning = ttk.LabelFrame(outer, text="4. 展示参数与 X/Y/Z 单轴验收", padding=10)
            tuning.pack(fill="x", pady=(10, 0))
            radius_row = ttk.Frame(tuning); radius_row.pack(fill="x")
            ttk.Label(radius_row, text="本次任务空间半径（10—48 mm）").pack(side="left")
            ttk.Scale(
                radius_row, from_=10.0, to=SHOWCASE_UI_MAX_RELATIVE_MM,
                variable=self.range_mm, orient=tk.HORIZONTAL, length=270,
            ).pack(side="left", padx=8)
            ttk.Entry(radius_row, textvariable=self.range_mm, width=8).pack(side="left")
            ttk.Label(radius_row, text="mm；底层额外保留 2 mm 后仍不超过 50 mm").pack(side="left", padx=5)

            speed_row = ttk.Frame(tuning); speed_row.pack(fill="x", pady=(6, 0))
            ttk.Label(speed_row, text="TCP 平移速度（5—30 mm/s）").pack(side="left")
            ttk.Scale(
                speed_row, from_=5.0, to=SHOWCASE_MAX_SPEED_MM_S,
                variable=self.speed_mm_s, orient=tk.HORIZONTAL, length=270,
            ).pack(side="left", padx=8)
            ttk.Entry(speed_row, textvariable=self.speed_mm_s, width=8).pack(side="left")
            ttk.Label(speed_row, text="mm/s；设置在 ARM 时锁定").pack(side="left", padx=5)

            axis_row = ttk.Frame(tuning); axis_row.pack(fill="x", pady=(8, 0))
            ttk.Label(axis_row, text="运动轴：").pack(side="left")
            for value, label in (("x", "X 单轴"), ("y", "Y 单轴"), ("z", "Z 单轴"), ("free", "自由 XYZ")):
                ttk.Radiobutton(
                    axis_row, text=label, value=value, variable=self.axis_mode,
                ).pack(side="left", padx=5)
            ttk.Label(axis_row, textvariable=self.axis_status, foreground="#175cd3").pack(side="left", padx=16)
            ttk.Label(
                tuning,
                text="展示档已改为手柄位移 : TCP 位移 = 1 : 1。单轴模式会在软件中把另外两轴强制归零；达到 20 mm 仅表示命令目标达到，仍需在平板确认真实 TCP 后才算通过。",
                wraplength=950, justify="left", foreground="#344054",
            ).pack(anchor="w", pady=(7, 0))

        ttk.Label(
            outer,
            text=("本次只做单方向平移；不要接触物体。完成后松开任意按钮，确认提示“已停止”，"
                  "再在平板上去使能、下电。若方向、速度或幅度异常，立即按物理急停。"),
            wraplength=980, justify="left", foreground="#344054",
        ).pack(anchor="w", pady=12)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.timer = root.after(20, self.tick)

    def connect(self) -> None:
        try:
            host = validate_host(self.host.get())
            if self.live and not messagebox.askokcancel(
                "仅连接确认",
                f"只登录 {host}。本按钮不会上电或使能。\n请确保其他运动程序均已退出。",
                parent=self.root,
            ):
                return
            self.controller.login()
            self.motion_text.set("正在连接；请等待状态与 Tool ID 刷新。")
        except Exception as exc:
            self.motion_text.set(f"连接未开始：{exc}")

    def disconnect(self) -> None:
        self.stop("disconnect")
        self.controller.logout()
        self.robot_text.set("已请求断开")

    def _frame_ready(self, frame: QuestFrame | None) -> bool:
        return bool(
            frame is not None
            and frame.connected and frame.tracked and frame.valid
            and frame.head_rotation_valid
            and (time.time_ns() - frame.received_time_ns) / 1e9 <= QUEST_PACKET_MAX_AGE_S
        )

    def _robot_ready(self, snap) -> bool:
        fresh = bool(
            snap.timestamp_ns is not None
            and (time.time_ns() - snap.timestamp_ns) / 1e9 <= ROBOT_SAMPLE_MAX_AGE_S
        )
        return bool(
            fresh and snap.connected and snap.powered_on and snap.enabled
            and not snap.estop and not snap.collision and not snap.on_limit
            and snap.tool_id == self.profile.required_tool_id
            and snap.tcp_pose is not None and len(snap.tcp_pose) == 6
            and not snap.error
        )

    def lock_heading(self) -> None:
        frame = self.latest_frame
        if not self._frame_ready(frame):
            self.motion_text.set("锁定失败：请佩戴头显、拿起右手柄，并确认 Preview 正在运行。")
            return
        assert frame is not None
        if frame.grip > DEADMAN_OFF or frame.trigger > DEADMAN_OFF:
            self.motion_text.set("锁定失败：请先完全松开右手 Grip 和 Trigger。")
            return
        self.stop("heading_relocked", quiet=True)
        try:
            yaw = self.mapper.lock_heading(frame.head_rotation_xyzw)
        except ValueError as exc:
            self.motion_text.set(f"锁定失败：{exc}")
            return
        self.heading_locked = True
        self.motion_text.set(f"正前方已锁定 yaw={yaw:+.1f}°；机器人仍不会运动。")

    def arm(self) -> None:
        snap = self.controller.get_snapshot()
        frame = self.latest_frame
        if not self.heading_locked:
            self.motion_text.set("拒绝 ARM：请先锁定当前头显正前方。")
            return
        if not self._frame_ready(frame):
            self.motion_text.set("拒绝 ARM：Quest 右手或头显追踪无效/过期。")
            return
        assert frame is not None
        if frame.grip > DEADMAN_OFF or frame.trigger > DEADMAN_OFF:
            self.motion_text.set("拒绝 ARM：请先完全松开 Grip 和 Trigger。")
            return
        if not self._robot_ready(snap):
            self.motion_text.set(
                "拒绝 ARM：需连接、状态新鲜、已由平板上电/使能、无急停/碰撞/限位、"
                f"Tool ID={self.profile.required_tool_id}。"
            )
            return
        if self.live and (
            not all(item.get() for item in self.checks)
            or self.confirm_text.get().strip().upper() != self.profile.confirmation
        ):
            self.motion_text.set(
                f"拒绝 ARM：请完成四项现场确认，并输入 {self.profile.confirmation}。"
            )
            return
        if self.profile.key == "showcase":
            try:
                radius = float(self.range_mm.get())
                speed = float(self.speed_mm_s.get())
            except (tk.TclError, ValueError):
                self.motion_text.set("拒绝 ARM：范围和速度必须是有效数字。")
                return
            if not 10.0 <= radius <= SHOWCASE_UI_MAX_RELATIVE_MM:
                self.motion_text.set("拒绝 ARM：真机展示半径只能设置为 10—48 mm。")
                return
            if not 5.0 <= speed <= SHOWCASE_MAX_SPEED_MM_S:
                self.motion_text.set(
                    f"拒绝 ARM：真机展示速度只能设置为 5—{SHOWCASE_MAX_SPEED_MM_S:.0f} mm/s。"
                )
                return
            self.active_ui_radius_mm = radius
            self.active_worker_radius_mm = min(SHOWCASE_MAX_RELATIVE_MM, radius + 2.0)
            self.active_speed_mm_s = speed
            self.active_hand_scale = 1.0
            self.active_axis_mode = self.axis_mode.get()
            self.axis_peak_mm = 0.0
        else:
            self.active_ui_radius_mm = self.profile.ui_radius_mm
            self.active_worker_radius_mm = self.profile.worker_radius_mm
            self.active_speed_mm_s = self.profile.speed_mm_s
            self.active_hand_scale = self.profile.hand_scale
            self.active_axis_mode = "free"
        self.stop("rearm", quiet=True)
        self.armed = True
        self.reference_tcp = None
        self.mapper.reset()
        effective_profile = replace(
            self.profile,
            ui_radius_mm=self.active_ui_radius_mm,
            worker_radius_mm=self.active_worker_radius_mm,
            speed_mm_s=self.active_speed_mm_s,
            hand_scale=self.active_hand_scale,
        )
        self.log.start(
            mode="live" if self.live else "demo",
            host=self.host.get(),
            profile=effective_profile,
        )
        self.log.meta["axis_mode"] = self.active_axis_mode
        self.motion_text.set(
            "已 ARM，但尚未启动伺服。现在同时按住右手 Grip + Trigger "
            f"才会开始 {self.profile.session_s:.0f} 秒{self.profile.title}。"
        )

    def _begin_deadman(self, frame: QuestFrame, snap) -> None:
        self.reference_tcp = tuple(float(value) for value in snap.tcp_pose)
        self.mapper.begin(frame.position_m)
        self.session_deadline_s = time.monotonic() + self.profile.session_s
        if self.profile.key == "showcase":
            self.controller.start_showcase_cartesian_servo(
                self.profile.required_tool_id,
                radius_mm=self.active_worker_radius_mm,
                speed_mm_s=self.active_speed_mm_s,
            )
        else:
            self.controller.start_commissioning_cartesian_servo(self.profile.required_tool_id)
        self.active = True
        self.log.meta["reference_tcp_mm_rad"] = list(self.reference_tcp)
        self.log.write("deadman_started", reference_tcp_mm_rad=list(self.reference_tcp))

    def stop(self, reason: str, *, quiet: bool = False) -> None:
        was_running = self.armed or self.active
        if self.profile.key == "showcase":
            self.controller.stop_showcase_cartesian_servo()
        else:
            self.controller.stop_commissioning_cartesian_servo()
        self.active = False
        self.armed = False
        self.reference_tcp = None
        self.mapper.reset()
        if was_running and self.profile.key == "showcase":
            reached = self.active_axis_mode in ("x", "y", "z") and self.axis_peak_mm >= 20.0
            self.log.write(
                "axis_target_summary",
                axis=self.active_axis_mode,
                peak_mm=self.axis_peak_mm,
                target_reached=reached,
                actual_robot_confirmation_required=True,
            )
            if self.active_axis_mode in ("x", "y", "z"):
                state = "目标已达到，等待平板实测确认" if reached else "目标未达到 20 mm"
                self.axis_status.set(
                    f"{self.active_axis_mode.upper()}：{state}（峰值 {self.axis_peak_mm:.1f} mm）"
                )
        path = self.log.finish(reason)
        if was_running and not quiet:
            self.motion_text.set(f"已停止并解除 ARM（{reason}）。日志：{path}")

    def tick(self) -> None:
        frame = self.quest.latest()
        if frame is not None:
            self.latest_frame = frame
        snap = self.controller.get_snapshot()
        robot_age = ((time.time_ns() - snap.timestamp_ns) / 1e9) if snap.timestamp_ns else math.inf
        tcp_text = "-" if snap.tcp_pose is None else " ".join(f"{v:.2f}" for v in snap.tcp_pose[:3])
        self.robot_text.set(
            f"JAKA connected={snap.connected} power={snap.powered_on} enabled={snap.enabled} "
            f"E-stop={snap.estop} collision={snap.collision} limit={snap.on_limit} Tool={snap.tool_id} age={robot_age:.2f}s\n"
            f"TCP XYZ mm: {tcp_text}" + (f"\n错误：{snap.error}" if snap.error else "")
        )
        if self.latest_frame is not None:
            q = self.latest_frame
            q_age = (time.time_ns() - q.received_time_ns) / 1e9
            self.quest_text.set(
                f"Quest source={q.udp_source} age={q_age:.2f}s connected={q.connected} "
                f"tracked={q.tracked} valid={q.valid} Grip={q.grip:.2f} Trigger={q.trigger:.2f}"
            )

        # SDK 子进程的失败或看门狗消息也会在这里解除界面 ARM，防止界面误报。
        if len(snap.log) > self.last_log_count:
            for message in snap.log[self.last_log_count:]:
                if self.active and ("自动停止" in message or "失败" in message):
                    self.active = self.armed = False
                    self.mapper.reset()
                    path = self.log.finish("worker_stopped")
                    self.motion_text.set(
                        f"SDK 子进程已停止{self.profile.title}：{message}；日志：{path}"
                    )
            self.last_log_count = len(snap.log)

        if self.armed:
            q = self.latest_frame
            if not self._frame_ready(q):
                self.stop("quest_tracking_or_packet_lost")
            elif not self._robot_ready(snap):
                self.stop("robot_state_not_ready")
            elif self.active:
                assert q is not None
                if not dual_deadman_pressed(q, already_active=True):
                    self.stop("deadman_released")
                elif time.monotonic() >= self.session_deadline_s:
                    self.stop(f"{self.profile.session_s:.0f}_second_limit")
                elif self.reference_tcp is not None:
                    delta = self.mapper.delta_mm(
                        q.position_m,
                        hand_scale=self.active_hand_scale,
                        radius_mm=self.active_ui_radius_mm,
                    )
                    if self.active_axis_mode in ("x", "y", "z"):
                        index = {"x": 0, "y": 1, "z": 2}[self.active_axis_mode]
                        delta = project_single_axis(delta, self.active_axis_mode)
                        self.axis_peak_mm = max(self.axis_peak_mm, abs(delta[index]))
                    target = translation_only_target(
                        self.reference_tcp,
                        delta,
                        self.active_ui_radius_mm,
                    )
                    if self.profile.key == "showcase":
                        self.controller.set_showcase_cartesian_target(target)
                    else:
                        self.controller.set_commissioning_cartesian_target(target)
                    remaining = max(0.0, self.session_deadline_s - time.monotonic())
                    self.motion_text.set(
                        "微动进行中 / 双按钮保持 / 松开任意一个立即停止\n"
                        + "ΔXYZ mm: " + " ".join(f"{value:+.2f}" for value in delta)
                        + "\n目标 XYZ mm: " + " ".join(f"{value:.2f}" for value in target[:3])
                        + f"\n剩余 {remaining:.1f} s"
                    )
                    now = time.monotonic()
                    if now - self.last_logged_target_s >= 0.1:
                        self.log.write("target", delta_xyz_mm=list(delta), target_tcp_mm_rad=list(target))
                        self.last_logged_target_s = now
            elif q is not None and dual_deadman_pressed(q, already_active=False):
                self._begin_deadman(q, snap)

        self.timer = self.root.after(20, self.tick)

    def close(self) -> None:
        try:
            self.root.after_cancel(self.timer)
        except Exception:
            pass
        self.stop("window_close", quiet=True)
        self.quest.close()
        self.controller.shutdown()
        self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quest -> JAKA fixed-envelope micro teleoperation")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--demo", action="store_true", default=True, help="模拟模式（默认）")
    mode.add_argument("--live", action="store_true", help="真机微动模式；仍需窗口内现场门槛")
    parser.add_argument(
        "--showcase",
        action="store_true",
        help="使用约 5 cm 展示档；默认仍为 5 mm 微动档",
    )
    # 旧微动入口默认不能命中现场控制柜；现行真机入口在根目录②中。
    parser.add_argument("--host", default="127.0.0.1")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    root = tk.Tk()
    profile = SHOWCASE_PROFILE if args.showcase else MICRO_PROFILE
    MicroTeleopWindow(root, live=bool(args.live), host=str(args.host), profile=profile)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
