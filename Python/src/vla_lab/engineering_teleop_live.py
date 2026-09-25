"""Quest → JAKA 六自由度遥操作主界面。

数据流：Unity/Quest UDP 5005 → 本界面做离合与坐标映射 → JAKA 子进程做最终安全限制、
逆解和 ``servo_p`` → 控制器真实关节反馈 → UDP 5006 → Unity JAKA S5 数字孪生。

为什么界面与 JAKA SDK 分进程：厂商二进制 SDK 的同步调用可能阻塞；若和 Tk 界面处在
同一进程，界面卡住时操作者看不到状态，也不能可靠地产生软件停止请求。这里的 GUI 只发
小消息，最终范围、速度、逆解与看门狗仍由子进程执行，GUI 滑杆不能突破硬上限。

本文件不提供上电或使能按钮。真机动作还必须经过现场勾选、POSE5、ARM、底层伺服确认
以及 Grip 保持；Trigger 独立控制 DH116，任何门槛缺失都不发送连续运动目标。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import tkinter as tk
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

# 支持两种启动方式：
# 1. 中文 .cmd 使用 ``python -m vla_lab.engineering_teleop_live``；
# 2. 在 PyCharm 中直接点本文件左侧绿色三角。
# 直接运行文件时 Python 不知道 ``vla_lab`` 包的父目录，因此这里仅补充搜索路径；
# 不连接机器人、不上电，也不改变默认的安全模拟模式。
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "vla_lab"

from .jaka_jog_controller import (
    ENGINEERING_LIVE_LOCKOUT_REASON,
    ENGINEERING_MAX_ANGULAR_SPEED_DEG_S,
    ENGINEERING_MAX_JOINT_SPEED_DEG_S,
    ENGINEERING_MAX_LINEAR_SPEED_MM_S,
    ENGINEERING_MAX_RELATIVE_MM,
    ENGINEERING_MAX_ROTATION_DEG,
    ENGINEERING_MAX_SESSION_S,
    ENGINEERING_REQUIRED_TOOL_ID,
    JakaJogController,
)
from .jaka_telemetry import SDK_DIRECTORY, validate_host
from .micro_teleop_gui import (
    DEADMAN_OFF, QUEST_PACKET_MAX_AGE_S, ROBOT_SAMPLE_MAX_AGE_S,
    clamp_vector,
)
from .quest_vr_input import (
    QuestFrame, QuestUdpReceiver, RelativeQuestTracker, matmul, matrix_rpy,
    rotation_angle_rad, rpy_matrix, scaled_rotation,
)
from .vr_robot_visualization import RobotVrBroadcaster
from .dh116_control import HandController
from .dh116_panel import HandPanel


ROOT = Path(__file__).resolve().parents[3]
CONFIG_PATH = ROOT / "Python" / "config" / "jaka_jog.json"
SESSION_ROOT = ROOT / "Validation" / "live_engineering_teleop_sessions"


def grip_deadman_pressed(frame: QuestFrame, *, already_active: bool) -> bool:
    """仅 Grip 作为运动保持键；Trigger 完全不参与机械臂离合判断。"""
    return math.isfinite(frame.grip) and (frame.grip > DEADMAN_OFF if already_active else frame.grip >= 0.75)


@dataclass(frozen=True)
class LevelASettings:
    radius_mm: float
    translation_scale: float
    linear_speed_mm_s: float
    rotation_enabled: bool
    rotation_scale: float
    rotation_deg: float
    angular_speed_deg_s: float
    joint_speed_deg_s: float
    xyz_axes: tuple[bool, bool, bool]
    rpy_axes: tuple[bool, bool, bool]

    def validate(self) -> None:
        """同时检查界面参数与底层硬上限；任何一项失败都拒绝 ARM。"""
        checks = (
            (50.0 <= self.radius_mm <= ENGINEERING_MAX_RELATIVE_MM, "任务空间半径必须为 5—100 cm"),
            (0.1 <= self.translation_scale <= 1.0, "位置比例必须为 0.1—1.0"),
            (5.0 <= self.linear_speed_mm_s <= ENGINEERING_MAX_LINEAR_SPEED_MM_S,
             "平移速度必须为 5—150 mm/s"),
            (0.1 <= self.rotation_scale <= 1.0, "姿态比例必须为 0.1—1.0"),
            (1.0 <= self.rotation_deg <= ENGINEERING_MAX_ROTATION_DEG,
             "姿态范围必须为 1—45°"),
            (1.0 <= self.angular_speed_deg_s <= ENGINEERING_MAX_ANGULAR_SPEED_DEG_S,
             "姿态速度必须为 1—30°/s"),
            (1.0 <= self.joint_speed_deg_s <= ENGINEERING_MAX_JOINT_SPEED_DEG_S,
             "逆解关节速度必须为 1—30°/s"),
            (any(self.xyz_axes) or (self.rotation_enabled and any(self.rpy_axes)), "至少选择一个位置轴或姿态轴"),
        )
        for passed, message in checks:
            if not passed:
                raise ValueError(message)


def compose_target(
    reference: tuple[float, ...],
    delta_xyz_mm: tuple[float, float, float],
    mapped_rotation,
    settings: LevelASettings,
) -> tuple[float, ...]:
    """由“按下离合时的 TCP”与手柄相对变化生成一个绝对 TCP 目标。

    没有勾选的轴严格冻结。自由 6D 使用旋转矩阵组合，而不是直接相加 RX/RY/RZ，
    否则同时转动多个轴时会受到欧拉角顺序与跳变影响。
    """

    xyz_delta = tuple(v if enabled else 0.0 for v, enabled in zip(delta_xyz_mm, settings.xyz_axes, strict=True))
    xyz_delta = clamp_vector(xyz_delta, settings.radius_mm)
    target_rpy = tuple(reference[3:])
    if settings.rotation_enabled and mapped_rotation is not None:
        scaled = scaled_rotation(mapped_rotation, settings.rotation_scale)
        angle = math.degrees(rotation_angle_rad(scaled))
        if angle > settings.rotation_deg:
            scaled = scaled_rotation(scaled, settings.rotation_deg / angle)
        if all(settings.rpy_axes):
            # Free 6-D mode composes rotations as matrices.  Adding Euler
            # components makes simultaneous roll/pitch/yaw depend on order and
            # was the reason multi-axis hand rotation felt ineffective.
            composed = matmul(scaled, rpy_matrix(tuple(reference[3:])))
            raw_rpy = matrix_rpy(composed)
            target_rpy = tuple(
                value + round((near - value) / (2.0 * math.pi)) * 2.0 * math.pi
                for value, near in zip(raw_rpy, reference[3:], strict=True)
            )
        else:
            delta_rpy = matrix_rpy(scaled)
            delta_rpy = tuple(v if enabled else 0.0 for v, enabled in zip(delta_rpy, settings.rpy_axes, strict=True))
            # Advanced single-axis mode keeps unchecked Euler fields frozen.
            target_rpy = tuple(reference[3 + i] + delta_rpy[i] for i in range(3))
    return tuple(reference[i] + xyz_delta[i] for i in range(3)) + tuple(target_rpy)


class LevelALiveWindow:
    """遥操作状态机和界面；不直接调用任何 JAKA 二进制 SDK 函数。"""
    def __init__(self, root: tk.Tk, *, live: bool, host: str, quest_port: int = 5005) -> None:
        if live:
            # 涉险事故未查清前，直接从 PyCharm 构造窗口也不能进入真机模式。
            raise RuntimeError(ENGINEERING_LIVE_LOCKOUT_REASON)
        self.root, self.live = root, bool(live)
        config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))["quest_vr"]
        self.tracker = RelativeQuestTracker(
            config["direction_mapping"], grip_on=0.75, grip_off=0.55,
            trigger_on=0.75, trigger_off=0.55,
        )
        self.quest = QuestUdpReceiver("127.0.0.1", quest_port)
        self.controller = JakaJogController(host=host, sdk_directory=SDK_DIRECTORY, poll_hz=30.0, demo=not live)
        self.vr_robot = RobotVrBroadcaster()
        self.hand = HandController(live=live)
        self.hand_allowed = tk.BooleanVar(value=False)
        self.hand_status = tk.StringVar(value="DH116 未连接；右 Grip 控臂，Trigger 控手需单独准备")
        self.hand_panel = None
        self.hand_was_authorized = False
        self.host = tk.StringVar(value=host)
        self.latest: QuestFrame | None = None
        self.heading_locked = self.armed = self.active = False
        # starting=True 表示 GUI 已发送启动请求，但底层 SDK 尚未确认伺服开启。
        # 它与 active 分开，避免旧版“按钮一按就显示运动中、实际机器人没接管”的假成功。
        self.starting = False
        self.start_requested_s = 0.0
        self.paused = False
        self.must_release_before_resume = False
        self.reference: tuple[float, ...] | None = None
        self.last_target: tuple[float, ...] | None = None
        # 25 Hz is fast enough for the outer target loop while leaving time for
        # one SDK IK/guard calculation to finish.  The worker still emits
        # servo_p at its own higher cadence and always coalesces to the newest
        # target, so the arm follows the hand instead of replaying stale poses.
        self.target_send_interval_s = 1.0 / 25.0
        self.next_target_send_s = 0.0
        self.settings: LevelASettings | None = None
        self.deadline = 0.0
        self.last_log_count = 0
        self.rate_notice = ""
        self.rate_notice_until = 0.0
        self.log_handle = None
        self.log_path: Path | None = None

        self.radius_cm = tk.DoubleVar(value=100.0)
        self.position_scale = tk.DoubleVar(value=1.0)
        self.linear_speed = tk.DoubleVar(value=50.0)
        self.rotation_enabled = tk.BooleanVar(value=True)
        self.rotation_scale = tk.DoubleVar(value=1.0)
        self.rotation_range = tk.DoubleVar(value=20.0)
        self.angular_speed = tk.DoubleVar(value=15.0)
        self.joint_speed = tk.DoubleVar(value=15.0)
        self.xyz_axes = [tk.BooleanVar(value=True) for _ in range(3)]
        self.rpy_axes = [tk.BooleanVar(value=True) for _ in range(3)]
        self.checks = [tk.BooleanVar(value=False) for _ in range(5)]
        self.confirm = tk.StringVar()
        self.robot_text = tk.StringVar(value="尚未连接")
        self.quest_text = tk.StringVar(value="等待 Quest UDP")
        self.status = tk.StringVar(value="未 ARM；机器人不会响应手柄")

        root.title(f"Quest → JAKA Level A 六维遥操作 · {'真机' if live else '模拟'}")
        root.geometry("1220x980"); root.minsize(1080, 900)
        outer = ttk.Frame(root, padding=10); outer.pack(fill="both", expand=True)
        ttk.Label(outer, text=(("真机" if live else "模拟（无硬件）") + "自由 6D｜灵巧手 Tool 1｜1:1 可调｜"
                               "最大位移预算 100 cm（受真实可达域约束）｜Grip 离合 / Trigger 开合"),
                  foreground="#b42318" if live else "#146c43",
                  font=("Microsoft YaHei UI", 13, "bold")).pack(anchor="w")
        ttk.Label(outer, text="机械臂上电/使能仍由平板操作；灵巧手需单独准备，快换不受控；软件 STOP 不能替代物理急停。",
                  foreground="#b42318").pack(anchor="w")

        conn = ttk.LabelFrame(outer, text="1. 连接与状态", padding=8); conn.pack(fill="x", pady=(8, 4))
        row = ttk.Frame(conn); row.pack(fill="x")
        ttk.Label(row, text="2号控制柜 IP").pack(side="left")
        ttk.Entry(row, textvariable=self.host, width=18).pack(side="left", padx=5)
        ttk.Button(row, text="连接（不自动上电/使能）", command=self.connect).pack(side="left")
        ttk.Button(row, text="断开", command=self.disconnect).pack(side="left", padx=5)
        tk.Button(row, text="软件 STOP / 退出伺服", command=lambda: self.stop("operator_stop"),
                  bg="#d92d20", fg="white", font=("Microsoft YaHei UI", 10, "bold")).pack(side="right")
        ttk.Label(conn, textvariable=self.robot_text).pack(anchor="w", pady=3)
        ttk.Label(conn, textvariable=self.quest_text).pack(anchor="w")
        hand_row = ttk.Frame(conn); hand_row.pack(fill="x", pady=3)
        ttk.Button(hand_row, text="灵巧手设置 / 连接 DH116", command=self.open_hand_panel).pack(side="left")
        ttk.Label(hand_row, textvariable=self.hand_status, wraplength=860).pack(side="left", padx=8)

        tune = ttk.LabelFrame(outer, text="2. 本次真机参数（ARM 时锁定）", padding=8); tune.pack(fill="x", pady=4)
        modes = ttk.Frame(tune); modes.pack(fill="x", pady=(0, 4))
        ttk.Label(modes, text="控制模式", width=22).pack(side="left")
        for label, mode in (("自由 6D", "free6d"), ("仅位置", "position"),
                            ("仅姿态", "rotation"), ("高级单轴", "advanced")):
            ttk.Button(modes, text=label, command=lambda value=mode: self.set_mode(value)).pack(side="left", padx=3)
        self._scale_row(tune, "任务空间半径", self.radius_cm, 5, 100, "cm")
        self._scale_row(tune, "手柄 : TCP 位置比例", self.position_scale, 0.1, 1.0, "倍")
        self._scale_row(tune, "TCP 平移速度", self.linear_speed, 5, 150, "mm/s")
        axes = ttk.Frame(tune); axes.pack(fill="x", pady=2)
        ttk.Label(axes, text="位置轴", width=22).pack(side="left")
        for name, var in zip(("X", "Y", "Z"), self.xyz_axes, strict=True):
            ttk.Checkbutton(axes, text=name, variable=var).pack(side="left", padx=8)
        rot = ttk.Frame(tune); rot.pack(fill="x", pady=(5, 1))
        ttk.Checkbutton(rot, text="启用姿态跟随", variable=self.rotation_enabled).pack(side="left")
        for name, var in zip(("RX", "RY", "RZ"), self.rpy_axes, strict=True):
            ttk.Checkbutton(rot, text=name, variable=var).pack(side="left", padx=8)
        self._scale_row(tune, "手柄 : TCP 姿态比例", self.rotation_scale, 0.1, 1.0, "倍")
        self._scale_row(tune, "相对姿态范围", self.rotation_range, 1, 45, "°")
        self._scale_row(tune, "TCP 姿态速度", self.angular_speed, 1, 30, "°/s")
        self._scale_row(tune, "逆解关节速度门槛", self.joint_speed, 1, 30, "°/s")
        ttk.Button(tune, text="应用并检查参数（不会运动）", command=self.apply_settings).pack(anchor="w", pady=4)

        gate = ttk.LabelFrame(outer, text="3. 真机现场门槛", padding=8); gate.pack(fill="x", pady=4)
        labels = (
            "机器人、末端和整个设置半径范围已清空，人员在范围外",
            "现场监护人守在可用的控制柜物理急停旁",
            "JAKA App 已确认 Tool ID=1（灵巧手末端）、无碰撞/限位报警",
            "已按本次参数确认实际可用空间；100 cm 只是位移预算，不等于完整球体均可达",
            "已关闭其他会向 2 号 JAKA 发送运动命令的程序",
        )
        for variable, text in zip(self.checks, labels, strict=True):
            ttk.Checkbutton(gate, text=text, variable=variable).pack(anchor="w")
        token = ttk.Frame(gate); token.pack(fill="x")
        ttk.Label(token, text="真机输入 POSE5：").pack(side="left")
        ttk.Entry(token, textvariable=self.confirm, width=12).pack(side="left")

        arm = ttk.LabelFrame(outer, text="4. 锁定、ARM、保持", padding=8); arm.pack(fill="x", pady=4)
        line = ttk.Frame(arm); line.pack(fill="x")
        ttk.Button(line, text="① 锁定 / 重捕获正前方", command=self.lock_heading).pack(side="left")
        ttk.Button(line, text="② ARM（保持到手动解除）", command=self.arm).pack(side="left", padx=6)
        ttk.Button(line, text="暂停（ARM 保持）", command=lambda: self.pause("operator_pause", require_release=True)).pack(side="left", padx=3)
        ttk.Button(line, text="清除提示 / 重新检查", command=self.clear_notice).pack(side="left", padx=3)
        ttk.Button(line, text="解除 ARM", command=lambda: self.stop("operator_disarm")).pack(side="left", padx=3)
        ttk.Label(arm, text=("ARM 后松开 Grip；按住右 Grip 才运动。松开暂停，不解除 ARM；重新握住从当前位姿捕获。"
                             "Trigger 不再参与控臂；在灵巧手设置中准备后，按住 Grip、先松 Trigger 再扣动即可开合。"),
                  wraplength=1080).pack(anchor="w", pady=5)
        ttk.Label(arm, textvariable=self.status, foreground="#7a2e0e", font=("Consolas", 10),
                  wraplength=1080).pack(anchor="w")
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.timer = root.after(20, self.tick)

    def open_hand_panel(self):
        if self.hand_panel is not None and self.hand_panel.window.winfo_exists():
            self.hand_panel.window.lift(); return
        self.hand_panel = HandPanel(self.root, self.hand, self.hand_allowed, lambda: self.latest, self.live)

    def _scale_row(self, parent, label, variable, low, high, unit) -> None:
        row = ttk.Frame(parent); row.pack(fill="x", pady=1)
        ttk.Label(row, text=label, width=22).pack(side="left")
        ttk.Scale(row, from_=low, to=high, variable=variable, orient="horizontal", length=320).pack(side="left")
        ttk.Entry(row, textvariable=variable, width=8).pack(side="left", padx=5)
        ttk.Label(row, text=unit).pack(side="left")

    def set_mode(self, mode: str) -> None:
        """Apply an explicit axis preset; this never sends a robot command."""

        if mode == "free6d":
            self.rotation_enabled.set(True)
            for variable in (*self.xyz_axes, *self.rpy_axes):
                variable.set(True)
            label = "自由 6D：XYZ + RX/RY/RZ"
        elif mode == "position":
            self.rotation_enabled.set(False)
            for variable in self.xyz_axes:
                variable.set(True)
            for variable in self.rpy_axes:
                variable.set(False)
            label = "仅位置：XYZ"
        elif mode == "rotation":
            self.rotation_enabled.set(True)
            for variable in self.xyz_axes:
                variable.set(False)
            for variable in self.rpy_axes:
                variable.set(True)
            label = "仅姿态：RX/RY/RZ"
        else:
            label = "高级单轴：请手动选择轴"
        self.status.set(f"已选择{label}；点“应用并检查参数”后才进入 ARM 流程。")

    def current_settings(self) -> LevelASettings:
        settings = LevelASettings(
            float(self.radius_cm.get()) * 10.0, float(self.position_scale.get()), float(self.linear_speed.get()),
            bool(self.rotation_enabled.get()), float(self.rotation_scale.get()), float(self.rotation_range.get()),
            float(self.angular_speed.get()), float(self.joint_speed.get()),
            tuple(v.get() for v in self.xyz_axes), tuple(v.get() for v in self.rpy_axes),
        )
        settings.validate(); return settings

    def apply_settings(self) -> None:
        try:
            settings = self.current_settings()
            if settings.rotation_enabled and not any(settings.rpy_axes):
                raise ValueError("已启用姿态，但未选择 RX/RY/RZ")
            self.status.set("参数检查通过；尚未 ARM，机器人不会运动。")
        except (ValueError, tk.TclError) as exc:
            self.status.set(f"参数不通过：{exc}")

    def connect(self) -> None:
        try:
            host = validate_host(self.host.get())
            if self.controller.get_snapshot().connected:
                self.status.set("JAKA 已连接；已忽略重复连接，不会创建第二个 SDK 会话。")
                return
            if self.live and not messagebox.askokcancel("仅连接", f"只登录 {host}，不会上电或使能。", parent=self.root):
                return
            self.controller.login()
            if not self.live:
                self.controller.set_tool_id(ENGINEERING_REQUIRED_TOOL_ID)
            self.status.set("正在连接；请等待状态刷新。")
        except Exception as exc:
            self.status.set(f"连接失败：{exc}")

    def disconnect(self) -> None:
        self.stop("disconnect"); self.controller.logout()

    def clear_notice(self) -> None:
        """Clear a latched software error after the operator checks live state."""

        self.controller.clear_error()
        self.status.set("已请求清除软件提示；机器人不会运动，请等待状态刷新后重新检查。")

    def _frame_ready(self) -> bool:
        q = self.latest
        return bool(q and q.connected and q.tracked and q.valid and q.rotation_valid and q.head_rotation_valid
                    and (time.time_ns() - q.received_time_ns) / 1e9 <= QUEST_PACKET_MAX_AGE_S)

    def _robot_ready(self, snap) -> bool:
        """判断本帧能否继续运动；测量过期与锁存错误也视为未就绪。"""
        fresh = snap.timestamp_ns and (time.time_ns() - snap.timestamp_ns) / 1e9 <= ROBOT_SAMPLE_MAX_AGE_S
        return bool(fresh and snap.connected and snap.powered_on and snap.enabled and not snap.estop
                    and not snap.collision and not snap.on_limit and snap.tool_id == ENGINEERING_REQUIRED_TOOL_ID
                    and snap.tcp_pose and len(snap.tcp_pose) == 6 and not snap.error)

    @staticmethod
    def _hard_robot_fault(snap) -> str:
        """Return only conditions that must remove ARM immediately."""

        if not snap.connected:
            return "JAKA 已断开"
        if not snap.powered_on:
            return "JAKA 已下电"
        if not snap.enabled:
            return "JAKA 已去使能"
        if snap.estop:
            return "JAKA 急停状态"
        if snap.collision:
            return "JAKA 碰撞状态"
        if snap.on_limit:
            return "JAKA 限位状态"
        if snap.tool_id != ENGINEERING_REQUIRED_TOOL_ID:
            return f"Tool 已变化为 {snap.tool_id}"
        return ""

    def _arm_blockers(self, snap) -> list[str]:
        """列出全部 ARM 阻塞原因，让操作者不用反复猜是哪一项失败。"""

        blockers: list[str] = []
        if not self.heading_locked:
            blockers.append("尚未锁定头显正前方")
        q = self.latest
        if q is None:
            blockers.append("尚未收到 Quest UDP")
        else:
            q_age = (time.time_ns() - q.received_time_ns) / 1e9
            if q_age > QUEST_PACKET_MAX_AGE_S:
                blockers.append(f"Quest 数据过期 {q_age:.2f}s")
            if not q.connected:
                blockers.append("右手柄 connected=False")
            if not q.tracked:
                blockers.append("右手柄 tracked=False")
            if not q.valid:
                blockers.append("右手柄 valid=False")
            if not q.rotation_valid:
                blockers.append("右手柄姿态 rotation=False")
            if not q.head_rotation_valid:
                blockers.append("头显姿态无效")
            if q.grip > DEADMAN_OFF:
                blockers.append("Grip 尚未完全松开")
        robot_age = (time.time_ns() - snap.timestamp_ns) / 1e9 if snap.timestamp_ns else math.inf
        if robot_age > ROBOT_SAMPLE_MAX_AGE_S:
            blockers.append(f"JAKA 状态过期 {robot_age:.2f}s")
        if not snap.connected:
            blockers.append("JAKA 未连接")
        if not snap.powered_on:
            blockers.append("JAKA 未上电")
        if not snap.enabled:
            blockers.append("JAKA 未使能")
        if snap.estop:
            blockers.append("JAKA 急停状态=True")
        if snap.collision:
            blockers.append("JAKA 碰撞状态=True")
        if snap.on_limit:
            blockers.append("JAKA 限位状态=True")
        if snap.tool_id != ENGINEERING_REQUIRED_TOOL_ID:
            blockers.append(f"当前 Tool={snap.tool_id}，本档要求 Tool={ENGINEERING_REQUIRED_TOOL_ID}")
        if not snap.tcp_pose or len(snap.tcp_pose) != 6:
            blockers.append("没有有效的六维 TCP 反馈")
        if snap.error:
            blockers.append(f"JAKA 错误：{snap.error}")
        if snap.engineering_servo_active and not self.active:
            blockers.append("底层仍报告六维伺服开启；请先点“软件 STOP / 退出伺服”并等待 servo=False")
        if self.live and not all(v.get() for v in self.checks):
            blockers.append("五项现场门槛尚未全部勾选")
        if self.live and self.confirm.get().strip().upper() != "POSE5":
            blockers.append("确认词不是 POSE5")
        return blockers

    def lock_heading(self) -> None:
        if not self._frame_ready():
            q = self.latest
            details = (
                "尚未收到有效右手帧" if q is None
                else f"connected={q.connected}, tracked={q.tracked}, valid={q.valid}, "
                     f"rotation={q.rotation_valid}, head={q.head_rotation_valid}"
            )
            self.status.set(
                "锁定失败：请佩戴头显、拿起右手柄，并只运行一个“①启动VR与JAKA数字孪生.cmd”。\n"
                f"诊断：{details}；UDP来源={self.quest.source_endpoint or '-'}；"
                f"接收错误={self.quest.error or '-'}。"
            )
            return
        assert self.latest is not None
        if self.latest.grip > DEADMAN_OFF:
            self.status.set("锁定失败：请先松开右 Grip。")
            return
        if self.active:
            self.pause("relock", require_release=True)
        try:
            yaw = self.tracker.lock_heading(self.latest.head_rotation_xyzw)
            self.heading_locked = True
            suffix = "ARM 保持；松开 Grip 后可重新抓取。" if self.armed else "尚未 ARM。"
            self.status.set(f"正前方已锁定 yaw={yaw:+.1f}°；{suffix}")
        except ValueError as exc:
            self.status.set(f"锁定失败：{exc}")

    def arm(self) -> None:
        snap = self.controller.get_snapshot()
        try:
            settings = self.current_settings()
            if settings.rotation_enabled and not any(settings.rpy_axes):
                raise ValueError("启用姿态时至少选一个姿态轴")
        except (ValueError, tk.TclError) as exc:
            self.status.set(f"拒绝 ARM：{exc}"); return
        blockers = self._arm_blockers(snap)
        if blockers:
            self.status.set("拒绝 ARM，未通过：\n• " + "\n• ".join(blockers)); return
        assert self.latest is not None
        self.stop("rearm", quiet=True)
        self.settings, self.armed, self.active, self.paused = settings, True, False, True
        self.starting = False
        self.must_release_before_resume = True
        self.reference = None; self.tracker.reset_grip()
        self._log_start(settings)
        self.status.set("已 ARM；先完全松开右 Grip，再按住 Grip 开始。Trigger 不影响机械臂启停。")

    def _log_start(self, settings: LevelASettings) -> None:
        SESSION_ROOT.mkdir(parents=True, exist_ok=True)
        self.log_path = SESSION_ROOT / (datetime.now().strftime("level_a_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
        self.log_path.mkdir()
        (self.log_path / "metadata.json").write_text(json.dumps({
            "schema": "quest_jaka_level_a.v1", "mode": "live" if self.live else "demo",
            "host": self.host.get(), "started_at": datetime.now().astimezone().isoformat(),
            "settings": settings.__dict__, "worker_hard_limits": {
                "radius_mm": ENGINEERING_MAX_RELATIVE_MM,
                "rotation_deg": ENGINEERING_MAX_ROTATION_DEG,
                "linear_mm_s": ENGINEERING_MAX_LINEAR_SPEED_MM_S,
                "angular_deg_s": ENGINEERING_MAX_ANGULAR_SPEED_DEG_S,
                "joint_deg_s": ENGINEERING_MAX_JOINT_SPEED_DEG_S,
                "continuous_burst_s": ENGINEERING_MAX_SESSION_S,
            }}, ensure_ascii=False, indent=2), encoding="utf-8")
        self.log_handle = (self.log_path / "events.jsonl").open("a", encoding="utf-8")
        self._log("armed")

    def _log(self, event: str, **values) -> None:
        if self.log_handle:
            self.log_handle.write(json.dumps({"time_ns": time.time_ns(), "event": event, **values}, ensure_ascii=False) + "\n")
            self.log_handle.flush()

    def _begin(self, q: QuestFrame, snap) -> None:
        """请求底层开启伺服，但在收到 ``servo=True`` 前绝不标记 active。"""
        assert self.settings is not None
        self.reference = tuple(float(v) for v in snap.tcp_pose)
        self.tracker.reset_grip(); self.tracker.update(q, self.settings.translation_scale)
        self.controller.start_engineering_cartesian_servo(
            radius_mm=self.settings.radius_mm,
            linear_speed_mm_s=self.settings.linear_speed_mm_s,
            rotation_deg=self.settings.rotation_deg,
            angular_speed_deg_s=self.settings.angular_speed_deg_s,
            joint_speed_deg_s=self.settings.joint_speed_deg_s,
        )
        self.start_requested_s = time.monotonic()
        self.next_target_send_s = 0.0
        self.deadline = 0.0
        self.starting, self.active, self.paused, self.must_release_before_resume = True, False, False, False
        self._log("servo_start_requested", reference=list(self.reference))
        self.status.set("已收到 Grip；正在等待 JAKA SDK 确认伺服启动，确认前不会发送运动目标……")

    def pause(self, reason: str, *, require_release: bool = False, quiet: bool = False) -> None:
        """Stop the live servo while retaining ARM and the process pipeline."""

        self.hand.update(permit=False, trigger=0.)
        if not self.armed:
            if not quiet:
                self.status.set("当前未 ARM；无需暂停。")
            return
        was_active = self.active or self.starting
        if was_active or self.controller.get_snapshot().engineering_servo_active:
            self.controller.stop_engineering_cartesian_servo()
        self.starting = self.active = False
        self.paused = True
        self.reference = None
        self.last_target = None
        self.next_target_send_s = 0.0
        self.tracker.reset_grip()
        self.must_release_before_resume = self.must_release_before_resume or require_release
        self._log("paused", reason=reason)
        if not quiet:
            self.status.set(
                f"已暂停（{reason}），ARM 保持。完全松开 Grip，移动手柄后重新握住即可继续；手不会自动张开。"
            )

    def stop(self, reason: str, *, quiet: bool = False) -> None:
        servo_reported = self.controller.get_snapshot().engineering_servo_active
        was = self.armed or self.active or self.starting or servo_reported
        if was or reason != 'rearm': self.hand.stop()
        if self.active or self.starting or servo_reported:
            self.controller.stop_engineering_cartesian_servo()
        self.armed = self.starting = self.active = self.paused = False
        self.must_release_before_resume = False
        self.reference = None; self.tracker.reset_grip()
        self.last_target = None
        self.next_target_send_s = 0.0
        if self.log_handle:
            self._log("finished", outcome=reason); self.log_handle.close(); self.log_handle = None
        if was and not quiet:
            self.status.set(f"已停止并解除 ARM（{reason}）。日志：{self.log_path}")

    def tick(self) -> None:
        """20 ms 状态机：刷新反馈、处理故障/离合，并广播数字孪生数据。"""
        q = self.quest.latest()
        if q is not None: self.latest = q
        snap = self.controller.get_snapshot()
        age = (time.time_ns() - snap.timestamp_ns) / 1e9 if snap.timestamp_ns else math.inf
        tcp = "-" if not snap.tcp_pose else " ".join(f"{v:.2f}" for v in snap.tcp_pose)
        self.robot_text.set(f"JAKA connected={snap.connected} power={snap.powered_on} enabled={snap.enabled} "
                            f"servo={snap.engineering_servo_active} E-stop={snap.estop} collision={snap.collision} "
                            f"limit={snap.on_limit} Tool={snap.tool_id} age={age:.2f}s\nTCP: {tcp}")
        if self.latest:
            qa = (time.time_ns() - self.latest.received_time_ns) / 1e9
            self.quest_text.set(f"Quest age={qa:.2f}s tracked={self.latest.tracked} valid={self.latest.valid} "
                                f"rotation={self.latest.rotation_valid} Grip={self.latest.grip:.2f} Trigger={self.latest.trigger:.2f}\n"
                                f"UDP来源={self.quest.source_endpoint or '-'} 忽略其它来源={self.quest.ignored_other_source_packets} "
                                f"接收错误={self.quest.error or '-'}")
        if len(snap.log) > self.last_log_count:
            for message in snap.log[self.last_log_count:]:
                if "关节限速介入" in message or "不可达边界" in message:
                    self.rate_notice = message
                    self.rate_notice_until = time.monotonic() + 1.5
                if (self.active or self.starting) and ("自动停止" in message or "失败" in message or "已停机" in message):
                    if "安全状态" in message or "servo_p" in message:
                        self.stop("worker_hard_fault")
                        self.status.set(f"底层硬停机并解除 ARM：{message}")
                    else:
                        self.pause("worker_paused", require_release=True)
                        self.status.set(f"底层暂停但 ARM 保持：{message}\n请松开 Grip，检查状态后可重新抓取。")
            self.last_log_count = len(snap.log)
        if self.armed:
            hard_fault = self._hard_robot_fault(snap)
            if hard_fault:
                self.stop("robot_hard_fault")
                self.status.set(f"已解除 ARM：{hard_fault}。请现场检查后重新 ARM。")
            elif self.starting:
                if not self._frame_ready():
                    self.pause("servo_start_tracking_lost", require_release=True)
                elif snap.error:
                    error = snap.error
                    self.pause("servo_start_rejected", require_release=True)
                    self.status.set(f"底层拒绝启动，ARM 保持但运动已暂停：{error}\n清除原因后可重新按住 Grip。")
                elif self.latest and not grip_deadman_pressed(self.latest, already_active=True):
                    self.pause("deadman_released_before_start")
                elif snap.engineering_servo_active:
                    self.starting = False
                    self.active = True
                    self.deadline = time.monotonic() + ENGINEERING_MAX_SESSION_S
                    self._log("servo_start_confirmed", reference=list(self.reference or ()))
                    self.status.set("JAKA SDK 已确认 servo=True；自由 6D 已开始，松开 Grip 即暂停。")
                elif time.monotonic() - self.start_requested_s > 2.0:
                    self.pause("servo_start_timeout", require_release=True)
                    self.status.set("等待 JAKA SDK 启动确认超过 2 秒；已发停止命令，ARM 保持。请检查下方错误后重试。")
            elif not self._frame_ready():
                if self.active:
                    self.pause("quest_tracking_lost", require_release=True)
            elif not self._robot_ready(snap):
                if self.active:
                    self.pause("robot_data_not_ready", require_release=True)
            elif self.active:
                assert self.latest is not None and self.settings is not None
                if not grip_deadman_pressed(self.latest, already_active=True):
                    self.pause("deadman_released")
                elif time.monotonic() >= self.deadline:
                    self.pause("continuous_burst_limit", require_release=True)
                elif self.reference:
                    target = None; delta = (0.0, 0.0, 0.0)
                    for event, value in self.tracker.update(self.latest, self.settings.translation_scale):
                        if event == "pose_delta":
                            delta, rotation = value
                            target = compose_target(self.reference, delta, rotation, self.settings)
                    if target:
                        self.last_target = target
                        now_target_s = time.monotonic()
                        if now_target_s >= self.next_target_send_s:
                            self.controller.set_engineering_cartesian_target(target)
                            self.next_target_send_s = now_target_s + self.target_send_interval_s
                        dr = tuple(math.degrees(target[i] - self.reference[i]) for i in range(3, 6))
                        self.status.set("自由 6D 进行中｜松开 Grip 即暂停（ARM 保持）\nΔXYZ mm: "
                                        + " ".join(f"{target[i]-self.reference[i]:+.1f}" for i in range(3))
                                        + "｜ΔRPY °: " + " ".join(f"{v:+.2f}" for v in dr)
                                        + f"｜本次连续段剩余 {max(0, self.deadline-time.monotonic()):.1f}s"
                                        + (("\n" + self.rate_notice) if time.monotonic() < self.rate_notice_until else ""))
                        self._log("target", target=list(target))
            elif self.latest:
                held = grip_deadman_pressed(self.latest, already_active=self.must_release_before_resume)
                if self.must_release_before_resume:
                    if not held:
                        self.must_release_before_resume = False
                        self.status.set("ARM 保持，Grip 已释放；移动手柄后重新握住即可继续。")
                elif grip_deadman_pressed(self.latest, already_active=False):
                    self._begin(self.latest, snap)
        hand_state = self.hand.snapshot()
        if self.hand_was_authorized and not hand_state.get('authorized') and self.hand_allowed.get():
            self.pause('hand_fault_or_stop', require_release=True)
        self.hand_was_authorized = bool(hand_state.get('authorized'))
        hand_permit = (self.active and self._frame_ready() and self._robot_ready(snap)
                       and self.hand_allowed.get() and self.latest is not None
                       and grip_deadman_pressed(self.latest, already_active=True))
        self.hand.update(permit=hand_permit, trigger=self.latest.trigger if self.latest else 0.)
        self.hand_status.set(('模拟手｜' if hand_state.get('simulated') else '实物手｜') + hand_state.get('status',''))
        # 数字孪生只显示实测 joints_rad；目标位姿另画成坐标轴，避免把“已请求”误当成
        # “真机已完成”。这个显示通道失败也不会影响底层伺服看门狗。
        self.vr_robot.publish(
            snap,
            armed=self.armed,
            starting=self.starting,
            active=self.active,
            target_tcp_mm_rad=self.last_target,
            hand_state=hand_state,
            robot_simulated=not self.live,
        )
        self.timer = self.root.after(20, self.tick)

    def close(self) -> None:
        try: self.root.after_cancel(self.timer)
        except Exception: pass
        self.stop("window_close", quiet=True)
        try: self.hand.close()
        except RuntimeError as exc:
            messagebox.showerror('灵巧手未确认退出', str(exc)); return
        self.vr_robot.close()
        self.quest.close()
        self.controller.shutdown()
        self.root.destroy()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(); mode.add_argument("--demo", action="store_true"); mode.add_argument("--live", action="store_true")
    # 公共源码默认只指向本机；真实控制柜地址必须由本机配置显式传入。
    parser.add_argument("--host", default="127.0.0.1")
    args = parser.parse_args(argv)
    if args.live:
        print(ENGINEERING_LIVE_LOCKOUT_REASON, file=sys.stderr)
        return 90
    root = tk.Tk(); LevelALiveWindow(root, live=bool(args.live), host=args.host); root.mainloop(); return 0


if __name__ == "__main__":
    raise SystemExit(main())
