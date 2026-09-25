"""Quest 1:1 参数化遥操作的零命令预览器。

本模块只读取 Quest UDP，在屏幕计算位置/姿态目标。它不导入 JAKA SDK，
不创建机器人对象，也没有任何真机连接或运动接口。
"""

from __future__ import annotations

import argparse
import math
import tkinter as tk
import time
from dataclasses import dataclass
from tkinter import ttk

from .quest_vr_input import (
    QuestFrame,
    QuestUdpReceiver,
    RelativeQuestTracker,
    matrix_rpy,
    rotation_angle_rad,
    scaled_rotation,
)


DEADMAN_ON = 0.75
DEADMAN_OFF = 0.55
QUEST_PACKET_MAX_AGE_S = 0.20


def clamp_vector(
    vector: tuple[float, float, float], radius: float
) -> tuple[float, float, float]:
    length = math.dist(vector, (0.0, 0.0, 0.0))
    if length <= float(radius) or length <= 1e-12:
        return vector
    ratio = float(radius) / length
    return tuple(value * ratio for value in vector)


def dual_deadman_pressed(frame: QuestFrame, *, already_active: bool) -> bool:
    threshold = DEADMAN_OFF if already_active else DEADMAN_ON
    return bool(frame.grip >= threshold and frame.trigger >= threshold)


@dataclass(frozen=True)
class PreviewSettings:
    workspace_radius_cm: float
    translation_scale: float
    linear_speed_mm_s: float
    rotation_enabled: bool
    rotation_scale: float
    orientation_radius_deg: float
    angular_speed_deg_s: float
    joint_speed_ceiling_deg_s: float = 10.0

    def validate(self) -> "PreviewSettings":
        checks = (
            (5.0 <= self.workspace_radius_cm <= 100.0, "任务空间半径必须为 5—100 cm"),
            (0.1 <= self.translation_scale <= 1.0, "位置比例必须为 0.1—1.0"),
            (5.0 <= self.linear_speed_mm_s <= 200.0, "位置速度必须为 5—200 mm/s"),
            (0.1 <= self.rotation_scale <= 1.0, "姿态比例必须为 0.1—1.0"),
            (1.0 <= self.orientation_radius_deg <= 90.0, "姿态范围必须为 1—90°"),
            (1.0 <= self.angular_speed_deg_s <= 90.0, "姿态速度必须为 1—90°/s"),
            (1.0 <= self.joint_speed_ceiling_deg_s <= 30.0, "关节速度上限必须为 1—30°/s"),
        )
        for passed, message in checks:
            if not passed:
                raise ValueError(message)
        return self


def rate_limit_vector(
    current: tuple[float, float, float],
    requested: tuple[float, float, float],
    max_speed: float,
    dt: float,
) -> tuple[float, float, float]:
    """Move a preview vector toward its target with one Euclidean speed bound."""

    distance = math.dist(current, requested)
    allowed = max(0.0, float(max_speed) * max(0.0, float(dt)))
    if distance <= allowed or distance <= 1e-12:
        return requested
    ratio = allowed / distance
    return tuple(a + ratio * (b - a) for a, b in zip(current, requested, strict=True))


class EngineeringTeleopPreview:
    def __init__(self, root: tk.Tk, *, quest_port: int = 5005) -> None:
        self.root = root
        self.quest = QuestUdpReceiver("127.0.0.1", int(quest_port))
        self.tracker = RelativeQuestTracker(
            {"forward": "X-", "backward": "X+", "left": "Y-", "right": "Y+", "up": "Z+", "down": "Z-"},
            grip_on=0.75, grip_off=0.55, trigger_on=0.75, trigger_off=0.55,
        )
        self.latest: QuestFrame | None = None
        self.heading_locked = False
        self.armed = False
        self.active = False
        self.settings: PreviewSettings | None = None
        self.last_update_s = time.monotonic()
        self.output_xyz = (0.0, 0.0, 0.0)
        self.output_rpy_deg = (0.0, 0.0, 0.0)

        self.radius_cm = tk.DoubleVar(value=100.0)
        self.translation_scale = tk.DoubleVar(value=1.0)
        self.linear_speed = tk.DoubleVar(value=100.0)
        self.rotation_enabled = tk.BooleanVar(value=True)
        self.rotation_scale = tk.DoubleVar(value=1.0)
        self.orientation_radius = tk.DoubleVar(value=30.0)
        self.angular_speed = tk.DoubleVar(value=30.0)
        self.joint_speed_ceiling = tk.DoubleVar(value=10.0)
        self.translation_axes = [tk.BooleanVar(value=True) for _ in range(3)]
        self.rotation_axes = [tk.BooleanVar(value=True) for _ in range(3)]
        self.quest_status = tk.StringVar(value="等待 Quest UDP 127.0.0.1:5005")
        self.preview_status = tk.StringVar(value="零命令预览未 ARM")

        root.title("Quest 1:1 参数化遥操作设计预览｜零机器人命令")
        root.geometry("1080x820")
        root.minsize(940, 720)
        outer = ttk.Frame(root, padding=12); outer.pack(fill="both", expand=True)
        ttk.Label(
            outer,
            text="零机器人命令｜1:1 位置与姿态参数预览｜最大半径 100 cm",
            foreground="#146c43", font=("Microsoft YaHei UI", 14, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            outer,
            text=("这里的 100 cm 只用于验证交互、方向、姿态和速度参数，不代表 JAKA 当前姿态下可达，"
                  "也不会发送上电、使能、伺服或运动命令。"),
            foreground="#b42318", wraplength=1030, justify="left",
        ).pack(anchor="w", pady=(2, 10))

        position = ttk.LabelFrame(outer, text="1. 位置参数", padding=10); position.pack(fill="x")
        self._slider_row(position, "任务空间半径", self.radius_cm, 5.0, 100.0, "cm")
        self._slider_row(position, "手柄 : TCP 比例", self.translation_scale, 0.1, 1.0, "倍")
        self._slider_row(position, "TCP 位置速度", self.linear_speed, 5.0, 200.0, "mm/s")
        axes = ttk.Frame(position); axes.pack(fill="x", pady=(6, 0))
        ttk.Label(axes, text="允许平移轴：").pack(side="left")
        for name, var in zip(("X", "Y", "Z"), self.translation_axes, strict=True):
            ttk.Checkbutton(axes, text=name, variable=var).pack(side="left", padx=8)

        rotation = ttk.LabelFrame(outer, text="2. 姿态参数（仍为零命令预览）", padding=10)
        rotation.pack(fill="x", pady=10)
        ttk.Checkbutton(rotation, text="启用手柄姿态映射", variable=self.rotation_enabled).pack(anchor="w")
        self._slider_row(rotation, "手柄 : 姿态比例", self.rotation_scale, 0.1, 1.0, "倍")
        self._slider_row(rotation, "相对姿态范围", self.orientation_radius, 1.0, 90.0, "°")
        self._slider_row(rotation, "姿态速度", self.angular_speed, 1.0, 90.0, "°/s")
        self._slider_row(
            rotation, "关节速度上限（设计门）", self.joint_speed_ceiling,
            1.0, 30.0, "°/s",
        )
        axes = ttk.Frame(rotation); axes.pack(fill="x", pady=(6, 0))
        ttk.Label(axes, text="允许姿态轴：").pack(side="left")
        for name, var in zip(("RX", "RY", "RZ"), self.rotation_axes, strict=True):
            ttk.Checkbutton(axes, text=name, variable=var).pack(side="left", padx=8)

        actions = ttk.LabelFrame(outer, text="3. 应用、锁定、预览", padding=10); actions.pack(fill="x")
        row = ttk.Frame(actions); row.pack(fill="x")
        ttk.Button(row, text="① 应用参数", command=self.apply_settings).pack(side="left")
        ttk.Button(row, text="② 锁定当前头显正前方", command=self.lock_heading).pack(side="left", padx=6)
        ttk.Button(row, text="③ ARM 零命令预览", command=self.arm).pack(side="left")
        ttk.Button(row, text="停止预览", command=lambda: self.stop("operator_stop")).pack(side="left", padx=6)
        ttk.Label(actions, textvariable=self.quest_status, justify="left").pack(anchor="w", pady=(10, 3))
        ttk.Label(
            actions, textvariable=self.preview_status, justify="left", wraplength=1020,
            foreground="#175cd3", font=("Consolas", 11),
        ).pack(anchor="w")
        ttk.Label(
            outer,
            text=("ARM 后同时按住右手 Grip+Trigger，以按下瞬间为零点。松开任意按钮立即清零并停止预览。"
                  "设置在每次 ARM 时冻结；若将来进入真机，还必须增加逐目标逆解、关节裕量、奇异点和工作区碰撞门。"),
            wraplength=1030, justify="left", foreground="#344054",
        ).pack(anchor="w", pady=12)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.timer = root.after(20, self.tick)

    @staticmethod
    def _slider_row(parent, label, variable, low, high, unit) -> None:
        row = ttk.Frame(parent); row.pack(fill="x", pady=3)
        ttk.Label(row, text=label, width=18).pack(side="left")
        ttk.Scale(row, from_=low, to=high, variable=variable, orient=tk.HORIZONTAL, length=420).pack(side="left")
        ttk.Entry(row, textvariable=variable, width=10).pack(side="left", padx=8)
        ttk.Label(row, text=unit).pack(side="left")

    def _frame_ready(self) -> bool:
        frame = self.latest
        return bool(
            frame is not None and frame.connected and frame.tracked and frame.valid
            and frame.head_rotation_valid
            and (time.time_ns() - frame.received_time_ns) / 1e9 <= QUEST_PACKET_MAX_AGE_S
        )

    def _read_settings(self) -> PreviewSettings:
        return PreviewSettings(
            workspace_radius_cm=float(self.radius_cm.get()),
            translation_scale=float(self.translation_scale.get()),
            linear_speed_mm_s=float(self.linear_speed.get()),
            rotation_enabled=bool(self.rotation_enabled.get()),
            rotation_scale=float(self.rotation_scale.get()),
            orientation_radius_deg=float(self.orientation_radius.get()),
            angular_speed_deg_s=float(self.angular_speed.get()),
            joint_speed_ceiling_deg_s=float(self.joint_speed_ceiling.get()),
        ).validate()

    def apply_settings(self) -> None:
        try:
            settings = self._read_settings()
        except (tk.TclError, ValueError) as exc:
            self.preview_status.set(f"参数未应用：{exc}")
            return
        self.settings = settings
        self.preview_status.set(
            f"参数已应用：半径 {settings.workspace_radius_cm:.1f} cm，位置 {settings.translation_scale:.2f}:1，"
            f"{settings.linear_speed_mm_s:.0f} mm/s；姿态={'开' if settings.rotation_enabled else '关'}。"
            f"关节速度设计上限 {settings.joint_speed_ceiling_deg_s:.0f}°/s（尚未进入真机 IK）。"
        )

    def lock_heading(self) -> None:
        if not self._frame_ready():
            self.preview_status.set("锁定失败：请佩戴头显、拿起右手柄并运行 Preview。")
            return
        assert self.latest is not None
        if self.latest.grip > DEADMAN_OFF or self.latest.trigger > DEADMAN_OFF:
            self.preview_status.set("锁定失败：先完全松开 Grip 和 Trigger。")
            return
        try:
            yaw = self.tracker.lock_heading(self.latest.head_rotation_xyzw)
        except ValueError as exc:
            self.preview_status.set(f"锁定失败：{exc}")
            return
        self.heading_locked = True
        self.preview_status.set(f"头显水平正前方已锁定 yaw={yaw:+.1f}°；仍是零命令。")

    def arm(self) -> None:
        if not self.heading_locked:
            self.preview_status.set("拒绝 ARM：请先锁定当前头显正前方。")
            return
        if not self._frame_ready():
            self.preview_status.set("拒绝 ARM：Quest 数据无效或过期。")
            return
        assert self.latest is not None
        if self.latest.grip > DEADMAN_OFF or self.latest.trigger > DEADMAN_OFF:
            self.preview_status.set("拒绝 ARM：请先完全松开 Grip 和 Trigger。")
            return
        try:
            self.settings = self._read_settings()
        except (tk.TclError, ValueError) as exc:
            self.preview_status.set(f"拒绝 ARM：{exc}")
            return
        self.tracker.reset_grip()
        self.armed = True
        self.active = False
        self.output_xyz = (0.0, 0.0, 0.0)
        self.output_rpy_deg = (0.0, 0.0, 0.0)
        self.preview_status.set("已 ARM 零命令预览；同时按住 Grip+Trigger 才开始计算。")

    def stop(self, reason: str) -> None:
        self.armed = self.active = False
        self.tracker.reset_grip()
        self.output_xyz = (0.0, 0.0, 0.0)
        self.output_rpy_deg = (0.0, 0.0, 0.0)
        self.preview_status.set(f"预览已停止并清零（{reason}）；未发送机器人命令。")

    def tick(self) -> None:
        frame = self.quest.latest()
        if frame is not None:
            self.latest = frame
        now = time.monotonic()
        if self.latest is not None:
            age = (time.time_ns() - self.latest.received_time_ns) / 1e9
            self.quest_status.set(
                f"Quest age={age:.2f}s connected={self.latest.connected} tracked={self.latest.tracked} "
                f"valid={self.latest.valid} rotation={self.latest.rotation_valid} "
                f"Grip={self.latest.grip:.2f} Trigger={self.latest.trigger:.2f}"
            )
        if self.armed:
            if not self._frame_ready():
                self.stop("tracking_or_udp_lost")
            else:
                assert self.latest is not None and self.settings is not None
                pressed = dual_deadman_pressed(self.latest, already_active=self.active)
                if not pressed:
                    if self.active:
                        self.stop("deadman_released")
                else:
                    if not self.active:
                        self.tracker.reset_grip()
                        self.active = True
                        self.last_update_s = now
                    requested_xyz = self.output_xyz
                    requested_rpy = self.output_rpy_deg
                    for event, value in self.tracker.update(
                        self.latest, self.settings.translation_scale
                    ):
                        if event != "pose_delta":
                            continue
                        raw_xyz, rotation = value
                        requested_xyz = clamp_vector(
                            tuple(float(v) for v in raw_xyz),
                            self.settings.workspace_radius_cm * 10.0,
                        )
                        requested_xyz = tuple(
                            value if enabled.get() else 0.0
                            for value, enabled in zip(requested_xyz, self.translation_axes, strict=True)
                        )
                        requested_rpy = (0.0, 0.0, 0.0)
                        if self.settings.rotation_enabled and rotation is not None:
                            scaled = scaled_rotation(rotation, self.settings.rotation_scale)
                            angle = math.degrees(rotation_angle_rad(scaled))
                            if angle > self.settings.orientation_radius_deg:
                                scaled = scaled_rotation(
                                    scaled, self.settings.orientation_radius_deg / angle
                                )
                            requested_rpy = tuple(math.degrees(v) for v in matrix_rpy(scaled))
                            requested_rpy = tuple(
                                value if enabled.get() else 0.0
                                for value, enabled in zip(requested_rpy, self.rotation_axes, strict=True)
                            )
                    dt = max(0.001, min(0.1, now - self.last_update_s))
                    self.output_xyz = rate_limit_vector(
                        self.output_xyz, requested_xyz,
                        self.settings.linear_speed_mm_s, dt,
                    )
                    self.output_rpy_deg = rate_limit_vector(
                        self.output_rpy_deg, requested_rpy,
                        self.settings.angular_speed_deg_s, dt,
                    )
                    self.last_update_s = now
                    self.preview_status.set(
                        "零命令预览进行中｜双按钮保持｜松开即清零\n"
                        + "请求 ΔXYZ mm: " + " ".join(f"{v:+.1f}" for v in requested_xyz)
                        + "   限速输出: " + " ".join(f"{v:+.1f}" for v in self.output_xyz)
                        + "\n请求 ΔRPY °: " + " ".join(f"{v:+.1f}" for v in requested_rpy)
                        + "   限速输出: " + " ".join(f"{v:+.1f}" for v in self.output_rpy_deg)
                    )
        self.timer = self.root.after(20, self.tick)

    def close(self) -> None:
        try:
            self.root.after_cancel(self.timer)
        except Exception:
            pass
        self.quest.close()
        self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quest 1:1 zero-command engineering preview")
    parser.add_argument("--quest-port", type=int, default=5005)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    root = tk.Tk()
    EngineeringTeleopPreview(root, quest_port=args.quest_port)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
