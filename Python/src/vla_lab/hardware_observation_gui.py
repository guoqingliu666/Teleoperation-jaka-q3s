"""真实相机 + JAKA + Quest 的只读单帧观察采集；绝不发送硬件命令。"""
from __future__ import annotations

from collections import deque
from datetime import datetime
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import shutil
import time
import tkinter as tk
from tkinter import messagebox, ttk
import uuid

from .hik_camera_preview import HikPreviewPanel
from .jaka_telemetry import SDK_DIRECTORY, TelemetryProcess, fresh, validate_host
from .quest_vr_input import QuestUdpReceiver
from .quest_vr_input import RelativeQuestTracker


ROOT = Path(__file__).resolve().parents[3]
OUTPUT_ROOT = ROOT / "Python" / "datasets" / "hardware_observations"
ROBOT_MAX_GAP_MS = 300.0
QUEST_MAX_GAP_MS = 150.0
SHADOW_ROOT = ROOT / "Validation" / "shadow_sessions"
EPISODE_DURATION_S = 5.0
EPISODE_MAX_FRAMES = 20


def atomic_json(path: Path, value: dict):
    path = path.resolve()
    if path.drive.upper() != "D:" or OUTPUT_ROOT.resolve() not in path.parents:
        raise ValueError("真实观察记录只允许写到项目 D 盘数据目录")
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def nearest(history, timestamp_ns, timestamp_key):
    return min(history, key=lambda value: abs(timestamp_key(value) - timestamp_ns), default=None)


def shadow_target_tcp(reference_tcp_mm_rad, mapped_delta_xyz_mm):
    """Apply a translation-only Quest delta to a frozen JAKA TCP reference."""
    if len(reference_tcp_mm_rad) != 6 or len(mapped_delta_xyz_mm) != 3:
        raise ValueError("影子目标要求 6 维 TCP 参考和 3 维平移量")
    return tuple(float(reference_tcp_mm_rad[i]) + float(mapped_delta_xyz_mm[i])
                 if i < 3 else float(reference_tcp_mm_rad[i]) for i in range(6))


def match_capture(capture, robots, quests):
    """Pair one camera frame with safe, nearest robot and valid Quest states."""
    camera_ns = int(capture["host_time_ns"])
    robot = nearest(robots, camera_ns, lambda value: value["host_time_unix_ns"])
    valid_quests = [value for value in quests if value.connected and value.tracked and value.valid]
    quest = nearest(valid_quests, camera_ns, lambda value: value.received_time_ns)
    robot_gap = None if robot is None else abs(robot["host_time_unix_ns"] - camera_ns) / 1e6
    quest_gap = None if quest is None else abs(quest.received_time_ns - camera_ns) / 1e6
    problems = []
    if robot is None or robot_gap > ROBOT_MAX_GAP_MS:
        problems.append(f"JAKA 与图像时间差不合格：{robot_gap} ms")
    if quest is None or quest_gap > QUEST_MAX_GAP_MS:
        problems.append(f"Quest 与图像时间差不合格：{quest_gap} ms")
    if robot is not None and robot.get("active_tool_id") != 0:
        problems.append("当前工具 ID 不是已验收的法兰中心 0")
    if robot is not None:
        status = robot.get("status", {})
        if status.get("powered_on") is not False or status.get("enabled") is not False:
            problems.append("本阶段要求 JAKA 下电且下使能；当前状态不是明确的 False/False")
    return robot, quest, robot_gap, quest_gap, problems


def compact_robot(sample):
    import math
    joints = "  ".join(f"{math.degrees(value):.2f}" for value in sample["joints_rad"])
    tcp = sample["tcp_mm_rad"]
    pose = "  ".join(f"{value:.3f}" for value in tcp[:3])
    angle = "  ".join(f"{math.degrees(value):.3f}" for value in tcp[3:])
    status = sample["status"]
    return (f"真实只读 · {sample['host']} · 工具 ID {sample['active_tool_id']} · 用户系 {sample['active_user_frame_id']}\n"
            f"J1—J6 度：{joints}\nTCP XYZ mm：{pose}\nTCP RX/RY/RZ 度：{angle}\n"
            f"上电 {status['powered_on']} · 使能 {status['enabled']} · 错误码 {status['error_code']}")


class ShadowTrace:
    """影子目标验收日志；只记录计算结果，不能转换成机器人命令。"""
    def __init__(self):
        self.path = self.stream = None
        self.rows = 0
        self.meta = {}

    def start(self, settings):
        if self.stream is not None:
            raise RuntimeError("已有影子日志正在记录")
        SHADOW_ROOT.mkdir(parents=True, exist_ok=True)
        self.path = SHADOW_ROOT / (datetime.now().strftime("shadow_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8])
        self.path.mkdir()
        self.rows = 0
        self.meta = {"schema": "quest_jaka_shadow_trace.v1", "phase": "recording",
            "started_host_time_ns": time.time_ns(), "settings": settings,
            "safety": {"read_only": True, "robot_command_sent": False, "hand_command_sent": False},
            "purpose": "translation mapping validation only; not a training episode"}
        self._save()
        self.stream = (self.path / "trace.jsonl").open("x", encoding="utf-8")

    def _save(self):
        temporary = self.path / "metadata.json.tmp"
        temporary.write_text(json.dumps(self.meta, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        temporary.replace(self.path / "metadata.json")

    def append(self, frame, robot, delta, target, grip_active):
        if self.stream is None:
            return
        row = {"row": self.rows, "host_time_ns": time.time_ns(), "udp_source": frame.udp_source,
            "unity_sequence": frame.raw_packet.get("sequence"), "quest_received_time_ns": frame.received_time_ns,
            "right": frame.raw_packet.get("right"), "robot": robot,
            "mapped_delta_xyz_mm": list(delta), "shadow_target_tcp_mm_rad": list(target) if target else None,
            "grip_active": bool(grip_active), "robot_command_sent": False}
        self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        self.rows += 1
        if self.rows % 20 == 0:
            self.stream.flush()

    def finish(self, reason):
        if self.stream is None:
            return self.path
        self.stream.flush(); os.fsync(self.stream.fileno()); self.stream.close(); self.stream = None
        self.meta.update(phase="finished", finished_host_time_ns=time.time_ns(), rows=self.rows,
                         finish_reason=reason, robot_command_sent=False)
        self._save()
        return self.path


class ContinuousObservationEpisode:
    """Bounded full-resolution synchronization trial; still contains no action."""
    def __init__(self):
        self.path = self.stream = self.token = None
        self.started_s = self.deadline_s = 0.0
        self.awaiting_token = None
        self.awaiting_path = None
        self.awaiting_since_s = 0.0
        self.stop_reason = None
        self.frame_count = self.rejected_count = self.total_image_bytes = 0
        self.stop_reason = None
        self.meta = {}

    @property
    def active(self):
        return self.stream is not None

    def start(self, instruction, camera_panel, duration_s=EPISODE_DURATION_S, max_frames=EPISODE_MAX_FRAMES):
        instruction = instruction.strip()
        if not instruction:
            raise ValueError("请填写连续观察任务说明")
        if self.active:
            raise RuntimeError("已有连续观察正在记录")
        OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
        free_bytes = shutil.disk_usage(OUTPUT_ROOT).free
        if free_bytes < 500 * 1024 * 1024:
            raise RuntimeError("D 盘可用空间不足 500 MB，拒绝开始连续原图记录")
        self.token = datetime.now().strftime("episode_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
        self.path = OUTPUT_ROOT / ("." + self.token + ".pending")
        (self.path / "frames").mkdir(parents=True, exist_ok=False)
        self.frame_count = self.rejected_count = self.total_image_bytes = 0
        self.started_s = time.monotonic()
        self.deadline_s = self.started_s + float(duration_s)
        self.meta = {
            "schema": "quest_jaka_hik_continuous_observation.v1", "phase": "recording",
            "instruction": instruction, "started_host_time_ns": time.time_ns(),
            "requested_duration_s": float(duration_s), "max_frames": int(max_frames),
            "free_bytes_before_start": int(free_bytes),
            "safety": {"read_only": True, "robot_command_sent": False,
                       "hand_command_sent": False, "quick_change_command_sent": False},
            "quality_limits_ms": {"robot": ROBOT_MAX_GAP_MS, "quest": QUEST_MAX_GAP_MS},
            "limitations": {"contains_action": False, "contains_hand_feedback": False,
                "is_demonstration_episode": False, "usable_for_training": False,
                "reason": "bounded continuous synchronization acceptance run only"},
        }
        atomic_json(self.path / "metadata.json", self.meta)
        self.stream = (self.path / "frames.jsonl").open("x", encoding="utf-8")
        self.request_next(camera_panel, int(max_frames))

    def request_next(self, camera_panel, max_frames=EPISODE_MAX_FRAMES):
        if not self.active or self.awaiting_token is not None or self.stop_reason is not None:
            return
        if self.frame_count + self.rejected_count >= int(max_frames):
            return
        index = self.frame_count + self.rejected_count
        self.awaiting_token = f"{self.token}:{index:04d}"
        self.awaiting_path = self.path / "frames" / f"global_{index:04d}.jpg"
        self.awaiting_since_s = time.monotonic()
        camera_panel.request_full_frame(self.awaiting_token, self.awaiting_path)

    def handle_capture(self, capture, robots, quests, camera_panel):
        if not self.active or capture.get("token") != self.awaiting_token:
            return None
        image_path = Path(capture["path"]).resolve()
        if image_path != self.awaiting_path.resolve() or not image_path.is_file() or image_path.stat().st_size <= 0:
            raise RuntimeError("连续观察收到的原图路径/文件无效")
        robot, quest, robot_gap, quest_gap, problems = match_capture(capture, robots, quests)
        index = self.frame_count + self.rejected_count
        record = {
            "index": index, "quality_gate_passed": not problems, "rejection_reasons": problems,
            "image_path": image_path.relative_to(self.path).as_posix(),
            "image_bytes": image_path.stat().st_size,
            "camera": {k: v for k, v in capture.items() if k not in ("token", "path")},
            "robot": robot,
            "quest": None if quest is None else {"received_host_time_ns": quest.received_time_ns,
                "udp_source": quest.udp_source, "raw_unity_packet": quest.raw_packet},
            "alignment": {"camera_to_robot_ms": robot_gap, "camera_to_quest_ms": quest_gap,
                "type": "nearest host receive/query time; NOT hardware triggering"},
            "robot_command_sent": False,
        }
        self.stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self.stream.flush()
        self.total_image_bytes += record["image_bytes"]
        self.awaiting_token = self.awaiting_path = None
        if problems:
            self.rejected_count += 1
            return self.finish("quality_gate_failed")
        self.frame_count += 1
        if self.stop_reason is not None:
            return self.finish(self.stop_reason)
        if time.monotonic() >= self.deadline_s or self.frame_count >= int(self.meta["max_frames"]):
            reason = "max_frames_completed" if self.frame_count >= int(self.meta["max_frames"]) else "duration_completed"
            return self.finish(reason)
        self.request_next(camera_panel, int(self.meta["max_frames"]))
        return None

    def request_stop(self, reason):
        if not self.active:
            return self.path
        self.stop_reason = reason
        return None if self.awaiting_token is not None else self.finish(reason)

    def finish(self, reason):
        if not self.active:
            return self.path
        self.stream.flush(); os.fsync(self.stream.fileno()); self.stream.close(); self.stream = None
        passed = self.frame_count > 0 and self.rejected_count == 0 and reason in {"duration_completed", "max_frames_completed"}
        self.meta.update(phase="accepted_continuous_observation" if passed else "rejected_continuous_observation",
            finished_host_time_ns=time.time_ns(), finish_reason=reason, quality_gate_passed=passed,
            accepted_frames=self.frame_count, rejected_frames=self.rejected_count,
            total_image_bytes=self.total_image_bytes, robot_command_sent=False)
        atomic_json(self.path / "metadata.json", self.meta)
        final_path = OUTPUT_ROOT / (("accepted_" if passed else "rejected_") + self.token)
        self.path.replace(final_path)
        self.path = final_path
        self.awaiting_token = self.awaiting_path = None
        self.stop_reason = None
        return final_path


def begin_capture(instruction: str, camera_panel: HikPreviewPanel):
    instruction = instruction.strip()
    if not instruction:
        raise ValueError("请填写这条观察对应的任务说明，例如：观察培养瓶抓取区域")
    token = datetime.now().strftime("obs_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
    path = OUTPUT_ROOT / ("." + token + ".pending")
    path.mkdir(parents=True, exist_ok=False)
    request_ns = time.time_ns()
    initial = {
        "schema": "quest_jaka_hik_observation.v1", "phase": "waiting_for_next_complete_camera_frame",
        "instruction": instruction, "requested_host_time_ns": request_ns,
        "safety": {"read_only": True, "robot_command_sent": False, "hand_command_sent": False,
                   "quick_change_command_sent": False},
    }
    atomic_json(path / "metadata.json", initial)
    try:
        camera_panel.request_full_frame(token, path / "global_rgb.jpg")
    except Exception:
        initial["phase"] = "camera_request_failed"
        atomic_json(path / "metadata.json", initial)
        raise
    return token, path, initial


def finish_capture(pending, capture, robots, quests):
    token, pending_path, initial = pending
    if capture["token"] != token:
        raise ValueError("相机快照 token 不匹配")
    robot, quest, robot_gap, quest_gap, problems = match_capture(capture, robots, quests)
    payload = dict(initial)
    payload.update(
        phase="accepted_observation" if not problems else "rejected_observation",
        completed_host_time_ns=time.time_ns(),
        quality_gate_passed=not problems,
        rejection_reasons=problems,
        camera={k: v for k, v in capture.items() if k not in ("token", "path")},
        image_path="global_rgb.jpg",
        robot=robot,
        quest=None if quest is None else {
            "received_host_time_ns": quest.received_time_ns,
            "received_monotonic_s": quest.received_s,
            "udp_source": quest.udp_source,
            "raw_unity_packet": quest.raw_packet,
        },
        alignment={
            "type": "nearest host receive/query time; NOT hardware triggering",
            "camera_to_robot_ms": robot_gap, "camera_to_quest_ms": quest_gap,
            "limits_ms": {"robot": ROBOT_MAX_GAP_MS, "quest": QUEST_MAX_GAP_MS},
            "robot_batch_note": "joint/TCP/status are sequential SDK queries within query interval",
        },
        limitations={
            "contains_hand_feedback": False, "contains_action": False, "is_complete_episode": False,
            "camera_role": "global_rgb_only", "usable_for_training": False,
            "reason": "single observation acceptance artifact; continuous synchronized demonstrations not implemented yet",
        },
    )
    atomic_json(pending_path / "metadata.json", payload)
    prefix = "accepted_" if not problems else "rejected_"
    final_path = OUTPUT_ROOT / (prefix + token)
    pending_path.replace(final_path)
    return final_path, payload


class HardwareObservationWindow:
    def __init__(self, root):
        self.root = root
        self.backend = TelemetryProcess()
        self.quest = QuestUdpReceiver("127.0.0.1", 5005)
        self.robot_history = deque(maxlen=600)
        self.quest_history = deque(maxlen=1200)
        self.pending = None
        self.pending_started_s = 0.0
        self.robot_connected = False
        config = json.loads((ROOT / "Python" / "config" / "jaka_jog.json").read_text(encoding="utf-8"))["quest_vr"]
        self.quest_config = config
        self.shadow_tracker = RelativeQuestTracker(config["direction_mapping"],
            grip_on=config["grip_on"], grip_off=config["grip_off"],
            trigger_on=config["trigger_on"], trigger_off=config["trigger_off"])
        self.shadow_heading_locked = False
        self.shadow_armed = False
        self.shadow_reference_tcp = None
        self.shadow_target = None
        self.shadow_trace = ShadowTrace()
        self.episode = ContinuousObservationEpisode()
        # 安全默认值不能连接现场控制柜；需要时由操作者显式填写。
        self.host = tk.StringVar(value="127.0.0.1")
        self.instruction = tk.StringVar(value="观察实验箱内目标物体（只读单帧）")
        self.robot_text = tk.StringVar(value="JAKA 未连接")
        self.quest_text = tk.StringVar(value="Quest UDP：等待 127.0.0.1:5005")
        self.status = tk.StringVar(value="准备就绪。依次打开相机、只读连接 JAKA、运行 Quest Preview。")
        self.shadow_text = tk.StringVar(value="影子目标未启用。先保持 JAKA 下电/下使能，锁定正前方。")
        self.episode_text = tk.StringVar(value="连续同步验收未开始（5 秒 / 最多 20 张原始图；不是训练 episode）。")
        root.title("Quest + JAKA + Hikrobot · 只读观察采集（无硬件动作）")
        root.geometry("1500x900")
        root.minsize(1200, 720)
        outer = ttk.Frame(root, padding=8); outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="只读采集：不控制机械臂、灵巧手或快换；当前仅生成单条观察验收包。", foreground="#b02a22").pack(anchor="w")
        body = ttk.Panedwindow(outer, orient="horizontal"); body.pack(fill="both", expand=True, pady=5)
        camera_box = ttk.Frame(body); body.add(camera_box, weight=3)
        self.camera = HikPreviewPanel(camera_box); self.camera.pack(fill="both", expand=True)
        self.camera.set_context_text("Hikrobot GigE 实物相机 / 只读硬件观察采集",
            "预览缩图；点击右侧采集时由相机进程保存下一张完整帧。曝光、分辨率、IP、触发模式保持原配置。")
        side = ttk.Frame(body, padding=8); body.add(side, weight=2)
        ttk.Label(side, text="2 号 JAKA 真实状态（只读）", font=("Microsoft YaHei UI", 12, "bold")).pack(anchor="w")
        row = ttk.Frame(side); row.pack(fill="x", pady=5)
        ttk.Label(row, text="控制柜 IP").pack(side="left")
        ttk.Entry(row, textvariable=self.host, width=18).pack(side="left", padx=5)
        ttk.Button(row, text="只读连接", command=self.connect_robot).pack(side="left")
        ttk.Button(row, text="断开", command=self.disconnect_robot).pack(side="left", padx=4)
        ttk.Label(side, textvariable=self.robot_text, wraplength=540, justify="left").pack(anchor="w", pady=5)
        ttk.Separator(side).pack(fill="x", pady=6)
        ttk.Label(side, text="Quest 状态", font=("Microsoft YaHei UI", 12, "bold")).pack(anchor="w")
        ttk.Label(side, textvariable=self.quest_text, wraplength=540, justify="left").pack(anchor="w", pady=5)
        shadow = ttk.LabelFrame(side, text="已完成的可选诊断：影子目标（连续采集时不要点）", padding=6)
        shadow.pack(fill="x", pady=5)
        shadow_row = ttk.Frame(shadow); shadow_row.pack(fill="x")
        ttk.Button(shadow_row, text="1. 锁定当前 VR 正前方", command=self.lock_heading).pack(side="left")
        ttk.Button(shadow_row, text="2. ARM 影子计算", command=self.arm_shadow).pack(side="left", padx=4)
        ttk.Button(shadow_row, text="停止影子", command=lambda: self.disarm_shadow("operator_stop")).pack(side="left")
        ttk.Label(shadow, textvariable=self.shadow_text, wraplength=520, justify="left", foreground="#704400").pack(anchor="w", pady=4)
        ttk.Separator(side).pack(fill="x", pady=6)
        ttk.Label(side, text="任务说明（写入样本）").pack(anchor="w")
        ttk.Entry(side, textvariable=self.instruction).pack(fill="x", pady=4)
        self.capture_button = ttk.Button(side, text="采集下一张完整帧＋最近状态", command=self.capture, state="disabled")
        self.capture_button.pack(fill="x", pady=6)
        episode_box = ttk.LabelFrame(side, text="当前下一步：连续多帧同步验收（只读 / 固定上限）", padding=6)
        episode_box.pack(fill="x", pady=5)
        episode_row = ttk.Frame(episode_box); episode_row.pack(fill="x")
        self.episode_start_button = ttk.Button(episode_row, text="只点击这里：开始 5 秒同步验收", command=self.start_continuous, state="disabled")
        self.episode_start_button.pack(side="left")
        self.episode_stop_button = ttk.Button(episode_row, text="提前停止并隔离", command=self.stop_continuous, state="disabled")
        self.episode_stop_button.pack(side="left", padx=5)
        ttk.Label(episode_box, textvariable=self.episode_text, wraplength=520, justify="left").pack(anchor="w", pady=4)
        ttk.Label(side, textvariable=self.status, foreground="#146783", wraplength=540, justify="left").pack(anchor="w", pady=5)
        ttk.Label(side, text="通过条件：相机实时、JAKA 数据新鲜、Quest 右手有效；时间差超限会保留为 rejected，不进入可用数据。\n图片为相机完整 5120×5120 JPG；界面缩略图不作为训练图。", wraplength=540, justify="left").pack(anchor="w", pady=5)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.timer = root.after(50, self.tick)

    def connect_robot(self):
        try:
            if self.robot_connected:
                self.status.set("JAKA 已在只读连接中，无需重复点击；如需重连请先点“断开”。")
                return
            host = validate_host(self.host.get())
            if not messagebox.askokcancel("只读连接确认", f"确认 {host} 是 robot2 控制柜。\n保持机器人静止并暂停其他自动控制程序。\n本窗口只查询，不具备急停功能。", parent=self.root):
                return
            self.backend.start("read", host, SDK_DIRECTORY)
            self.robot_connected = True
            self.status.set("正在等待 JAKA 第一帧只读状态……")
        except Exception as exc:
            self.status.set(str(exc))

    def disconnect_robot(self):
        self.disarm_shadow("robot_disconnect")
        forced = self.backend.close()
        self.robot_connected = False
        self.robot_history.clear()
        self.robot_text.set("JAKA 已断开；历史显示已清空")
        self.status.set("SDK 阻塞，已终止本程序子进程。" if forced else "JAKA 只读会话已断开。")

    def safe_static_robot(self):
        return bool(self.robot_history and fresh(self.robot_history[-1])
                    and self.robot_history[-1]["status"].get("powered_on") is False
                    and self.robot_history[-1]["status"].get("enabled") is False)

    def lock_heading(self):
        if not self.quest_history:
            self.shadow_text.set("未收到有效 Quest 右手数据。")
            return
        frame = self.quest_history[-1]
        if not frame.head_rotation_valid or time.time_ns() - frame.received_time_ns > 500_000_000:
            self.shadow_text.set("头显朝向不可用或已过期。")
            return
        if frame.grip > float(self.quest_config["grip_off"]):
            self.shadow_text.set("请先松开右手 Grip，再锁定正前方。")
            return
        self.disarm_shadow("heading_relocked")
        try:
            yaw = self.shadow_tracker.lock_heading(frame.head_rotation_xyzw)
        except ValueError as exc:
            self.shadow_text.set("锁定失败：" + str(exc)); return
        self.shadow_heading_locked = True
        self.shadow_text.set(f"已锁定头显水平正前方 yaw={yaw:+.1f}°；可 ARM 影子计算。")

    def arm_shadow(self):
        if self.shadow_armed:
            self.shadow_text.set("影子计算已经 ARM。")
            return
        if not self.shadow_heading_locked:
            self.shadow_text.set("请先松开 Grip 并锁定当前 VR 正前方。")
            return
        if not self.safe_static_robot():
            self.shadow_text.set("拒绝 ARM：本阶段要求 JAKA 状态新鲜，并明确下电、下使能。")
            return
        if not self.quest_history or not (self.quest_history[-1].valid and self.quest_history[-1].tracked):
            self.shadow_text.set("拒绝 ARM：Quest 右手没有有效追踪。")
            return
        if self.quest_history[-1].grip > float(self.quest_config["grip_off"]):
            self.shadow_text.set("拒绝 ARM：请先松开 Grip，防止参考点跳变。")
            return
        self.shadow_tracker.reset_grip()
        self.shadow_reference_tcp = self.shadow_target = None
        self.shadow_trace.start({"mapping": self.quest_config["direction_mapping"],
            "translation_scale": self.quest_config["translation_scale"],
            "max_relative_translation_mm": self.quest_config["max_relative_translation_mm"],
            "rotation_enabled": False, "controller_ip": self.host.get(), "required_tool_id": 0})
        self.shadow_armed = True
        self.shadow_text.set("影子计算已 ARM；按住右手 Grip 移动，只改变屏幕上的拟目标。")

    def disarm_shadow(self, reason):
        was_armed = self.shadow_armed
        self.shadow_armed = False
        self.shadow_reference_tcp = self.shadow_target = None
        self.shadow_tracker.reset_grip()
        path = self.shadow_trace.finish(reason)
        if was_armed:
            self.shadow_text.set(f"影子计算已停止；日志：{path}")

    def update_shadow(self, frame):
        if not self.shadow_armed:
            return
        if not self.safe_static_robot():
            self.disarm_shadow("robot_not_fresh_or_not_powered_off_disabled")
            self.shadow_text.set("已自动停止：JAKA 不再满足新鲜且下电/下使能。")
            return
        if not (frame.connected and frame.tracked and frame.valid):
            self.disarm_shadow("quest_tracking_lost")
            self.shadow_text.set("已自动停止：Quest 右手追踪丢失。恢复后需重新锁定并 ARM。")
            return
        for event, value in self.shadow_tracker.update(frame, float(self.quest_config["translation_scale"])):
            if event == "grip_start":
                self.shadow_reference_tcp = tuple(self.robot_history[-1]["tcp_mm_rad"])
                self.shadow_target = self.shadow_reference_tcp
            elif event == "grip_stop":
                self.shadow_reference_tcp = self.shadow_target = None
                self.shadow_text.set("Grip 已松开；影子 ARM 保持，等待下一次按住。")
            elif event == "pose_delta" and self.shadow_reference_tcp is not None:
                delta, _rotation = value
                distance = math.dist(delta, (0.0, 0.0, 0.0))
                if distance > float(self.quest_config["max_relative_translation_mm"]):
                    self.disarm_shadow("relative_translation_limit")
                    self.shadow_text.set(f"已自动停止：相对位移 {distance:.1f} mm 超过限制。")
                    return
                self.shadow_target = shadow_target_tcp(self.shadow_reference_tcp, delta)
                self.shadow_text.set("影子 ARM / 零命令\n当前 TCP XYZ mm：" + "  ".join(f"{v:.2f}" for v in self.robot_history[-1]["tcp_mm_rad"][:3])
                    + "\n拟目标 XYZ mm：" + "  ".join(f"{v:.2f}" for v in self.shadow_target[:3])
                    + "\n相对 ΔXYZ mm：" + "  ".join(f"{v:+.2f}" for v in delta))
                self.shadow_trace.append(frame, self.robot_history[-1], delta, self.shadow_target, True)

    def capture(self):
        if self.episode.active:
            self.status.set("连续同步验收正在进行，不能同时采集单帧。")
            return
        if self.pending is not None:
            self.status.set("已有一个快照请求在等待下一张完整相机帧。")
            return
        try:
            if not self.robot_history or not fresh(self.robot_history[-1]):
                raise RuntimeError("JAKA 状态不新鲜")
            if not self.quest_history or time.time_ns() - self.quest_history[-1].received_time_ns > 500_000_000:
                raise RuntimeError("Quest UDP 超过 0.5 秒没有新数据")
            self.pending = begin_capture(self.instruction.get(), self.camera)
            self.pending_started_s = time.monotonic()
            self.status.set("已请求相机下一张完整帧；正在等待并做时间对齐……")
        except Exception as exc:
            self.status.set(f"未开始采集：{exc}")

    def start_continuous(self):
        try:
            if self.pending is not None:
                raise RuntimeError("请先等待当前单帧采集完成")
            if self.shadow_armed:
                raise RuntimeError("请先停止影子计算；连续观察不记录拟动作")
            if not self.safe_static_robot():
                raise RuntimeError("JAKA 必须状态新鲜、下电且下使能")
            if not self.quest_history or not (self.quest_history[-1].connected
                    and self.quest_history[-1].tracked and self.quest_history[-1].valid):
                raise RuntimeError("Quest 右手当前必须连接、追踪且有效")
            self.episode.start(self.instruction.get(), self.camera)
            self.episode_text.set("正在记录：等待第 1 张完整原图……")
            self.status.set("连续同步验收已开始；不要放下头显或改变机器人状态。")
        except Exception as exc:
            self.episode_text.set("未开始：" + str(exc))

    def stop_continuous(self):
        path = self.episode.request_stop("operator_stopped_early")
        if path is not None:
            self.episode_text.set(f"已提前停止并隔离：{path}")
        elif self.episode.active:
            self.episode_text.set("已请求停止；保存正在写入的一张原图后隔离。")

    def tick(self):
        for kind, payload in self.backend.poll():
            if kind == "sample":
                self.robot_history.append(payload)
                self.robot_text.set(compact_robot(payload))
            elif kind in ("error", "warning"):
                self.status.set(payload)
                self.robot_connected = False
            elif kind == "closed":
                self.robot_connected = False
        frame = self.quest.latest()
        if frame is not None:
            self.quest_history.append(frame)
            self.update_shadow(frame)
        elif self.quest.error and not self.quest_history:
            self.quest_text.set("Quest UDP 接收失败：" + self.quest.error + "；请关闭占用 5005 的旧 Python GUI")
        if self.quest_history:
            q = self.quest_history[-1]
            age = (time.time_ns() - q.received_time_ns) / 1e9
            seq = q.raw_packet.get("sequence", "?")
            self.quest_text.set(f"已锁定 UDP 来源 {q.udp_source} · seq {seq} · 帧龄 {age:.2f}s\n右手连接 {q.connected} / 追踪 {q.tracked} / 有效 {q.valid} · Grip {q.grip:.2f} · Trigger {q.trigger:.2f}\n忽略其他来源 {self.quest.ignored_other_source_packets} 包；锁定前无效包 {self.quest.ignored_untracked_before_lock}")
        for capture in self.camera.pop_captures():
            if self.episode.active and capture.get("token") == self.episode.awaiting_token:
                try:
                    path = self.episode.handle_capture(capture, self.robot_history, self.quest_history, self.camera)
                    if path is None:
                        self.episode_text.set(f"正在记录：已通过 {self.episode.frame_count} 张，等待下一张完整原图……")
                    else:
                        passed = self.episode.meta.get("quality_gate_passed", False)
                        self.episode_text.set(("连续同步验收通过：" if passed else "连续同步验收已隔离：") + str(path))
                except Exception as exc:
                    path = self.episode.finish("capture_processing_error")
                    self.episode_text.set(f"连续同步处理失败并隔离：{exc}；{path}")
                continue
            if self.pending is None or capture["token"] != self.pending[0]:
                self.status.set("收到未知相机快照，未写配对元数据。")
                continue
            try:
                path, payload = finish_capture(self.pending, capture, self.robot_history, self.quest_history)
                state = "通过单帧质量门" if payload["quality_gate_passed"] else "时间/状态门未通过，已隔离"
                self.status.set(f"{state}：{path}")
            except Exception as exc:
                self.status.set(f"快照整理失败；pending 文件保留供排查：{exc}")
            finally:
                self.pending = None
        if self.pending is not None and time.monotonic() - self.pending_started_s > 15:
            metadata = dict(self.pending[2]); metadata["phase"] = "camera_capture_timeout"
            atomic_json(self.pending[1] / "metadata.json", metadata)
            self.status.set(f"15 秒未收到完整相机帧，未生成可用观察；诊断保留：{self.pending[1]}")
            self.pending = None
        if self.episode.active:
            latest_q_ok = bool(self.quest_history and self.quest_history[-1].connected
                and self.quest_history[-1].tracked and self.quest_history[-1].valid
                and time.time_ns() - self.quest_history[-1].received_time_ns < 500_000_000)
            if not self.safe_static_robot():
                self.episode.request_stop("robot_state_gate_lost")
            elif not latest_q_ok:
                self.episode.request_stop("quest_tracking_gate_lost")
            elif self.episode.awaiting_token and time.monotonic() - self.episode.awaiting_since_s > 3:
                # 相机子进程可能仍在写这一张，因此先请求停止；收到它后再安全封包。
                self.episode.request_stop("camera_frame_timeout")
        camera_ok = self.camera.last_sample is not None and time.monotonic() - self.camera.last_sample["host_monotonic"] < 2
        robot_ok = bool(self.robot_history and fresh(self.robot_history[-1])
                        and self.robot_history[-1]["status"].get("powered_on") is False
                        and self.robot_history[-1]["status"].get("enabled") is False)
        quest_ok = bool(self.quest_history
                        and self.quest_history[-1].connected
                        and self.quest_history[-1].tracked
                        and self.quest_history[-1].valid
                        and time.time_ns() - self.quest_history[-1].received_time_ns < 500_000_000)
        ready = camera_ok and robot_ok and quest_ok and self.pending is None and not self.episode.active
        self.capture_button.configure(state="normal" if ready else "disabled")
        self.episode_start_button.configure(state="normal" if ready and not self.shadow_armed else "disabled")
        self.episode_stop_button.configure(state="normal" if self.episode.active else "disabled")
        if not self.episode.active and self.episode.path is None:
            reasons = []
            if not camera_ok:
                reasons.append("相机尚未实时预览")
            if not self.robot_history or not fresh(self.robot_history[-1]):
                reasons.append("JAKA 只读状态尚未到达或已过期")
            elif not robot_ok:
                rs = self.robot_history[-1]["status"]
                reasons.append(f"JAKA 必须下电/下使能（当前上电 {rs.get('powered_on')}、使能 {rs.get('enabled')}）")
            if not quest_ok:
                reasons.append("Quest 右手必须连接/追踪/有效")
            if self.pending is not None:
                reasons.append("单帧采集尚未完成")
            if self.shadow_armed:
                reasons.append("影子诊断仍在 ARM，请点“停止影子”")
            if reasons:
                self.episode_text.set("按钮暂不可点：" + "；".join(reasons) + "。")
            else:
                self.episode_text.set("条件齐全：现在只点击左侧“开始 5 秒同步验收”。VR 画面和机械臂不会变化；这里会显示已保存张数。")
        self.timer = self.root.after(50, self.tick)

    def close(self):
        self.root.after_cancel(self.timer)
        self.disarm_shadow("window_close")
        self.quest.close()
        self.backend.close()
        self.camera.close()
        if self.episode.active:
            self.episode.finish("window_closed")
        self.root.destroy()


def main():
    os.environ["TEMP"] = os.environ["TMP"] = r"D:\ChatGPT\Temp"
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    root = tk.Tk(); HardwareObservationWindow(root); root.mainloop()


if __name__ == "__main__":
    mp.freeze_support(); main()
