"""可直接 Run File 或双击入口；默认不加载 SDK、不连接任何设备。"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import messagebox, ttk

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vla_lab.jaka_telemetry import SDK_DIRECTORY, TelemetryProcess, fresh, validate_host

ROOT = Path(__file__).resolve().parents[3]


def write_json(path, value):
    path = Path(path).resolve()
    if path.drive.upper() != "D:":
        raise ValueError("验收输出仅允许写到 D:。")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def display_sample(sample):
    def vector(values, degrees=False):
        if values is None:
            return "未知 / 本次未读到"
        import math
        return "   ".join(f"{math.degrees(v) if degrees else v: .3f}" for v in values)
    def bit(v):
        return "未知" if v is None else "是" if v else "否"
    tcp, status = sample["tcp_mm_rad"], sample["status"]
    tool = sample["active_tool_id"]
    lines = [
        "真实 SDK 只读反馈（不是 DEMO；尚未通过现场坐标验收）",
        f"连接目标：{sample['host']}    设备铭牌/序列号仍需现场核对",
        f"工具 ID：{tool} {'（法兰中心，不是手掌抓取点）' if tool == 0 else '（已配置工具，具体抓取点待标定）'}",
        f"当前用户坐标系 ID：{sample['active_user_frame_id']}（0 为基座）",
        "",
        "TCP X / Y / Z（毫米）： " + vector(tcp[:3]),
        "TCP RX / RY / RZ（度，仅显示换算）： " + vector(tcp[3:], True),
        "TCP RX / RY / RZ（SDK 弧度）： " + vector(tcp[3:]),
        "J1—J6（度，仅显示换算）： " + vector(sample["joints_rad"], True),
        "J1—J6（SDK 弧度）： " + vector(sample["joints_rad"]),
        "",
        "实际 TCP 原始反馈（mm / rad）： " + vector(sample["actual_tcp_mm_rad"]),
        "工具相对法兰偏置（mm / rad）： " + vector(sample["tool_offset_mm_rad"]),
        f"上电：{bit(status['powered_on'])}   使能：{bit(status['enabled'])}   错误码：{status['error_code']}",
        "急停状态：本面板未查询，不能据此判断安全。   灵巧手状态：未接入。",
        f"本轮查询耗时：{sample['query_span_ms']:.1f} ms；各接口顺序读取，不是硬件同步采样。",
        "",
        "警告：" + ("\n".join(sample["warnings"]) or "无接口格式警告；仍需现场核对坐标/工具定义。"),
    ]
    return "\n".join(lines)


class ReadOnlyWindow:
    def __init__(self, root):
        self.root, self.backend = root, TelemetryProcess()
        self.latest = None
        self.active_read = False
        self.mode = None
        self.preflight_seen = False
        self.error_seen = False
        root.title("JAKA 2 号 · 只读验收（无运动 / 无 IO 写入）")
        root.geometry("1180x760")
        root.minsize(980, 650)
        self.status = tk.StringVar(value="未连接；启动窗口不会连接任何设备。")
        self.host = tk.StringVar(value="")
        self.sdk = tk.StringVar(value=str(SDK_DIRECTORY))
        wrapper = ttk.Frame(root, padding=14)
        wrapper.pack(fill="both", expand=True)
        ttk.Label(wrapper, text="只读阶段：保持机器人静止，不上电、不使能、不改 TCP、不控制快换/灵巧手。", foreground="#ac2c24").pack(anchor="w")
        ttk.Label(wrapper, text="对照资料：robot2 / S5160217 / 控制器 1.7.2_40_X64_cab2_1（平板照片记录，并非联网识别结果）。").pack(anchor="w", pady=(5, 10))
        sdk_row = ttk.Frame(wrapper); sdk_row.pack(fill="x")
        ttk.Label(sdk_row, text="SDK 文件夹：").pack(side="left")
        self.sdk_entry = ttk.Entry(sdk_row, textvariable=self.sdk); self.sdk_entry.pack(side="left", fill="x", expand=True)
        controls = ttk.Frame(wrapper); controls.pack(fill="x", pady=10)
        ttk.Label(controls, text="2 号控制柜 IP：").pack(side="left")
        self.host_entry = ttk.Entry(controls, textvariable=self.host, width=19); self.host_entry.pack(side="left", padx=(0, 8))
        self.preflight_button = ttk.Button(controls, text="1. 离线检查 SDK", command=lambda: self.start("preflight")); self.preflight_button.pack(side="left", padx=4)
        self.connect_button = ttk.Button(controls, text="2. 确认并只读连接", command=lambda: self.start("read")); self.connect_button.pack(side="left", padx=4)
        ttk.Button(controls, text="断开只读会话", command=self.disconnect).pack(side="left", padx=4)
        self.save_button = ttk.Button(controls, text="保存新鲜状态快照", command=self.save, state="disabled"); self.save_button.pack(side="left", padx=4)
        ttk.Label(wrapper, textvariable=self.status, foreground="#146783", wraplength=1100).pack(anchor="w", pady=6)
        self.text = tk.Text(wrapper, wrap="word", font=("Microsoft YaHei UI", 11), height=23)
        self.text.pack(fill="both", expand=True)
        self.show("等待读取真实状态。\n\n1. 先检查 SDK，不会连接机器人。\n2. 从 2 号平板核对控制柜 IP，填入后再确认连接。\n3. 比较 J1—J6、TCP、工具 ID，保持机械臂静止即可。\n\n6502 是截图中的 Modbus 端口，不是 IP。\nhand 工具偏置只是平板设置记录，不代表当前已启用，也不是标定验收结果。\n关闭窗口只释放本程序的读取连接，不会停止其他程序正在控制的机器人。")
        ttk.Label(wrapper, text="只读快照不含相机、VR 同步数据或灵巧手反馈，不能作为已完成的训练 episode。", wraplength=1100).pack(anchor="w", pady=(10, 0))
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.timer = root.after(100, self.tick)

    def show(self, value):
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", value)
        self.text.configure(state="disabled")

    def set_busy(self, value):
        for widget in (self.host_entry, self.sdk_entry, self.preflight_button, self.connect_button):
            widget.configure(state="disabled" if value else "normal")

    def start(self, mode):
        try:
            host = validate_host(self.host.get()) if mode == "read" else ""
            if mode == "read" and not messagebox.askokcancel("现场核对后只读连接", f"确认 {host} 是 2 号 JAKA 控制柜，而不是相机/电脑？\n\n机器人应静止，暂停其他自动控制程序。只读程序不发运动或 IO 指令，也没有急停功能。", parent=self.root):
                return
            self.latest = None
            self.active_read = mode == "read"
            self.mode = mode
            self.preflight_seen = self.error_seen = False
            self.backend.start(mode, host, self.sdk.get())
            self.set_busy(True)
            self.status.set("正在离线加载 SDK……" if mode == "preflight" else f"正在只读连接 {host}；等待首帧……")
        except Exception as exc:
            self.active_read = False
            self.status.set(str(exc))

    def tick(self):
        for kind, payload in self.backend.poll():
            if kind == "preflight":
                self.preflight_seen = True
                if self.mode == "preflight":
                    self.show("SDK 离线加载成功；未创建机器人对象、未建立机器人连接。\n\n" + json.dumps(payload, ensure_ascii=False, indent=2))
                    self.status.set("SDK 离线检查通过；控制器兼容性仍须真实只读连接验证。")
            elif kind == "sample":
                self.latest = payload
                self.show(display_sample(payload))
            elif kind in ("error", "warning"):
                self.error_seen = True
                self.active_read = False
                self.status.set(payload)
                self.show("本次会话无有效实时状态：\n" + payload)
            elif kind == "closed":
                if self.active_read:
                    self.status.set("SDK 连接已结束；没有实时状态。")
                elif self.mode == "preflight" and not self.preflight_seen and not self.error_seen:
                    self.status.set("SDK 子进程未返回结果；离线检查未通过，请检查 SDK/DLL 兼容性。")
                self.active_read = False
                self.set_busy(False)
        if self.active_read:
            if fresh(self.latest):
                age = (time.monotonic_ns() - self.latest["host_query_started_monotonic_ns"]) / 1e9
                self.status.set(f"正在只读 · 帧龄 {age:.2f} 秒 · 真实通信成功不等于运动安全验收通过")
            elif self.latest is not None:
                self.status.set("数据过期：下面数值仅为历史帧，禁止用来判断当前状态。")
        if self.backend.process is None:
            self.set_busy(False)
        self.save_button.configure(state="normal" if self.active_read and fresh(self.latest) else "disabled")
        self.timer = self.root.after(100, self.tick)

    def save(self):
        if not self.active_read or not fresh(self.latest):
            self.status.set("没有新鲜的真实状态，不保存快照。")
            return
        path = ROOT / "Validation" / "robot_readonly" / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
        try:
            write_json(path, self.latest)
            messagebox.showinfo("只读快照已保存", str(path), parent=self.root)
        except Exception as exc:
            messagebox.showerror("保存失败", str(exc), parent=self.root)

    def disconnect(self):
        forced = self.backend.close()
        self.latest = None
        self.active_read = False
        self.set_busy(False)
        self.save_button.configure(state="disabled")
        self.status.set("已结束本程序读取；SDK 阻塞时已终止自己的子进程，需等待连接释放。" if forced else "已断开只读会话；不会停止其他控制源。")
        self.show("已断开，历史数值已清空。")

    def close(self):
        self.root.after_cancel(self.timer)
        self.backend.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--preflight", action="store_true", help="仅离线加载 SDK，随后退出")
    parser.add_argument("--report", type=Path, default=ROOT / "Validation" / "jaka_sdk_preflight.json")
    parser.add_argument("--sdk-directory", type=Path, default=SDK_DIRECTORY)
    args = parser.parse_args()
    # 直接在 PyCharm Run File 也使用 D:，SDK 自建相对日志不会落到源文件 F:。
    os.environ["TEMP"] = os.environ["TMP"] = r"D:\ChatGPT\Temp"
    os.chdir(ROOT / "Python")
    if args.preflight:
        backend = TelemetryProcess()
        backend.start("preflight", directory=args.sdk_directory)
        result = None
        try:
            while backend.process is not None:
                for kind, payload in backend.poll():
                    if kind == "preflight":
                        result = payload
                    elif kind == "error":
                        raise RuntimeError(payload)
                time.sleep(0.05)
            if result is None:
                raise RuntimeError("SDK 子进程未返回检查结果")
            write_json(args.report, result)
            print("SDK offline preflight PASS. No robot constructed or connected.")
        finally:
            backend.close()
        return
    root = tk.Tk()
    ReadOnlyWindow(root)
    root.mainloop()


if __name__ == "__main__":
    mp.freeze_support()
    main()
