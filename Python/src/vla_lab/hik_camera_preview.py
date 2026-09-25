"""Hikrobot GigE 预览：独立进程隔离 SDK，默认不发现、不打开任何相机。

只在用户点击后枚举/采集；不写网络、曝光、ROI、触发模式等参数。
这是预览，不是 episode 图像记录器，也不含任何机器人接口。
"""
from __future__ import annotations

import ctypes as ct
import importlib
import multiprocessing as mp
import os
from pathlib import Path
import queue
import sys
import time
import tkinter as tk
from tkinter import ttk, messagebox

DEFAULT_SERIAL = os.environ.get("HIK_PREFERRED_SERIAL", "").strip()


def load_sdk():
    """直接使用已安装的官方包装器，避免复制或猜测 C 结构体布局。"""
    development = Path(os.environ.get("MVCAM_COMMON_RUNENV", r"D:\通信\海康视觉\MVS\Development"))
    wrapper = development / "Samples/Python/MvImport"
    runtime = Path(os.environ.get("HIK_MVS_RUNTIME", r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"))
    if ct.sizeof(ct.c_void_p) != 8:
        raise RuntimeError("请使用 64 位 Python（当前项目的 jaka 环境）")
    if not (wrapper / "MvCameraControl_class.py").is_file() or not (runtime / "MvCameraControl.dll").is_file():
        raise RuntimeError("找不到已安装的 MVS Python SDK / Win64 DLL；检查 MVCAM_COMMON_RUNENV 和 HIK_MVS_RUNTIME")
    # 官方包装器以文件名加载 DLL；这里只改当前子进程搜索路径，不改系统环境。
    os.environ["PATH"] = str(runtime) + os.pathsep + os.environ.get("PATH", "")
    dll_handle = os.add_dll_directory(str(runtime))
    sys.path.insert(0, str(wrapper))
    sdk = importlib.import_module("MvCameraControl_class")
    sdk._repro_dll_directory_handle = dll_handle
    return sdk


def check(code, operation):
    if code:
        raise RuntimeError(f"{operation}失败：0x{code & 0xffffffff:08X}")


def ip_text(value):
    return ".".join(str((value >> shift) & 255) for shift in (24, 16, 8, 0))


def decode(value):
    raw = bytes(value).split(b"\0", 1)[0]
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw.decode("gbk", errors="replace")


def enumerate_devices(sdk):
    """只枚举 GigE，不开句柄、不扫描任意 IP、不修改设备。保留不同网卡路径。"""
    listing = sdk.MV_CC_DEVICE_INFO_LIST()
    check(sdk.MvCamera.MV_CC_EnumDevices(sdk.MV_GIGE_DEVICE, listing), "枚举")
    result = []
    for index in range(listing.nDeviceNum):
        info = ct.cast(listing.pDeviceInfo[index], ct.POINTER(sdk.MV_CC_DEVICE_INFO)).contents
        gige = info.SpecialInfo.stGigEInfo
        result.append(dict(index=index, name=decode(gige.chUserDefinedName),
                           model=decode(gige.chModelName), serial=decode(gige.chSerialNumber),
                           ip=ip_text(gige.nCurrentIp), interface=ip_text(gige.nNetExport)))
    return listing, result


def select_device(devices, requested):
    # 重新枚举后按序列号和网卡匹配，绝不能用可能变动的列表下标打开另一台。
    candidates = [d for d in devices if d["serial"] == requested["serial"]
                  and d["interface"] == requested["interface"]]
    if len(candidates) != 1:
        raise RuntimeError("所选相机/网卡路径已变化，请重新发现设备；不会自动切换到其他相机")
    return candidates[0]


def _capture_path(value):
    path = Path(value).resolve()
    allowed = (Path(__file__).resolve().parents[2]
               / "datasets" / "hardware_observations").resolve()
    if path.drive.upper() != "D:" or path.suffix.lower() != ".jpg" or allowed not in path.parents:
        raise ValueError("相机快照只允许写到项目 D 盘 hardware_observations 目录下的 JPG")
    return path


def camera_worker(mode, requested, events, frames, stop, capture_requests=None):
    """SDK 生命周期全部在子进程；阻塞/异常不阻塞 VR 与模拟控制界面。"""
    sdk = camera = None
    initialized = created = opened = grabbing = False
    try:
        sdk = load_sdk()
        check(sdk.MvCamera.MV_CC_Initialize(), "SDK 初始化")
        initialized = True
        listing, devices = enumerate_devices(sdk)
        if mode == "enum":
            events.put(("devices", devices))
            return
        selected = select_device(devices, requested)
        info = ct.cast(listing.pDeviceInfo[selected["index"]], ct.POINTER(sdk.MV_CC_DEVICE_INFO)).contents
        camera = sdk.MvCamera()
        check(camera.MV_CC_CreateHandle(info), "创建相机句柄")
        created = True
        # Exclusive 不是抢占模式。被 MVS/其他程序占用时失败，不使用切断其他用户的权限。
        check(camera.MV_CC_OpenDevice(sdk.MV_ACCESS_Exclusive, 0), "打开相机（请确认 MVS 已关闭该设备）")
        opened = True
        trigger = sdk.MVCC_ENUMVALUE()
        check(camera.MV_CC_GetEnumValue("TriggerMode", trigger), "读取触发模式")
        if trigger.nCurValue != 0:
            raise RuntimeError("相机处于触发模式，预览不会自行修改。请在 MVS 确认连续采集配置后重试")
        # 这两项仅设置 SDK 的主机内存队列，不改变相机拍摄参数。
        check(camera.MV_CC_SetImageNodeNum(3), "设置主机缓冲数")
        check(camera.MV_CC_SetGrabStrategy(sdk.MV_GrabStrategy_LatestImagesOnly), "设置最新帧策略")
        check(camera.MV_CC_StartGrabbing(), "启动取流")
        grabbing = True
        events.put(("status", "已打开；等待完整图像（不改相机参数）"))
        from PIL import Image
        started = last_frame = time.monotonic()
        received = incomplete = timeouts = 0
        rgb_buffer = None
        buffer_size = 0
        while not stop.is_set():
            frame = sdk.MV_FRAME_OUT()
            code = camera.MV_CC_GetImageBuffer(frame, 500)
            now = time.monotonic()
            if code:
                if code != sdk.MV_E_NODATA:
                    check(code, "读取图像")
                timeouts += 1
                events.put(("status", f"等待完整帧 {now-last_frame:.1f}s；超时次数 {timeouts}；检查 MVS 原配置/网络，不自动重连"))
                if now - last_frame > 12:
                    raise RuntimeError("连续 12 秒没有完整帧，已停止并释放相机。请检查链路、包大小和触发配置")
                continue
            try:
                details = frame.stFrameInfo
                if details.nLostPacket:
                    incomplete += 1
                    if now - last_frame > 12:
                        raise RuntimeError("持续丢包，12 秒没有可用完整帧，停止预览")
                    events.put(("status", f"丢弃不完整帧 {incomplete}；本帧丢包 {details.nLostPacket}"))
                    continue
                width, height = int(details.nWidth), int(details.nHeight)
                size = width * height * 3
                if not frame.pBufAddr or not (0 < size <= 300_000_000):
                    raise RuntimeError("SDK 返回了无效图像尺寸/指针")
                if size != buffer_size:
                    rgb_buffer = (ct.c_ubyte * size)()
                    buffer_size = size
                conversion = sdk.MV_CC_PIXEL_CONVERT_PARAM_EX()
                conversion.nWidth, conversion.nHeight = width, height
                conversion.pSrcData = frame.pBufAddr
                conversion.nSrcDataLen = details.nFrameLen
                conversion.enSrcPixelType = details.enPixelType
                conversion.enDstPixelType = sdk.PixelType_Gvsp_RGB8_Packed
                conversion.pDstBuffer = rgb_buffer
                conversion.nDstBufferSize = size
                check(camera.MV_CC_ConvertPixelTypeEx(conversion), "转换 RGB")
                if conversion.nDstLen != size:
                    raise RuntimeError("转换后的 RGB 字节数不符合图像尺寸")
                image = Image.frombytes("RGB", (width, height), bytes(rgb_buffer))
                capture_time_ns = time.time_ns()
                if capture_requests is not None:
                    requests = []
                    try:
                        while True:
                            requests.append(capture_requests.get_nowait())
                    except queue.Empty:
                        pass
                    # 多次点击只保存下一完整帧一次，并给每个请求返回同一帧证据。
                    for token, requested_path in requests:
                        final_path = _capture_path(requested_path)
                        final_path.parent.mkdir(parents=True, exist_ok=True)
                        temporary = final_path.with_suffix(".tmp")
                        image.save(temporary, format="JPEG", quality=95, subsampling=0)
                        temporary.replace(final_path)
                        events.put(("capture", dict(token=token, path=str(final_path),
                            host_time_ns=capture_time_ns, frame_id=int(details.nFrameNum),
                            width=width, height=height, camera=selected)))
                image.thumbnail((960, 960))
                received += 1
                last_frame = now
                sample = dict(size=image.size, rgb=image.tobytes(), width=width, height=height,
                              frame_id=int(details.nFrameNum), host_monotonic=now,
                              host_time_ns=capture_time_ns,
                              received=received, incomplete=incomplete, timeouts=timeouts,
                              fps=received/max(now-started, .001), camera=selected)
                # 最多保留一帧。只丢预览刷新帧，不谎报这是相机网络丢包。
                try:
                    frames.put_nowait(sample)
                except queue.Full:
                    pass
            finally:
                check(camera.MV_CC_FreeImageBuffer(frame), "释放图像缓存")
    except Exception as error:
        events.put(("error", str(error)))
    finally:
        cleanup_errors = []
        for active, operation in ((grabbing, "MV_CC_StopGrabbing"), (opened, "MV_CC_CloseDevice"),
                                  (created, "MV_CC_DestroyHandle")):
            if active:
                try:
                    check(getattr(camera, operation)(), operation)
                except Exception as error:
                    cleanup_errors.append(str(error))
        if initialized:
            try:
                check(sdk.MvCamera.MV_CC_Finalize(), "SDK 释放")
            except Exception as error:
                cleanup_errors.append(str(error))
        events.put(("done", "; ".join(cleanup_errors)))
        # GUI 可能已关闭，不让大图像的队列刷写阻止进程退出。
        frames.cancel_join_thread()


class HikPreviewPanel(ttk.Frame):
    """可嵌入 Tk 的相机面板。构造时不加载 SDK，也不枚举或连接设备。"""
    def __init__(self, parent):
        super().__init__(parent, padding=6)
        self.context = mp.get_context("spawn")
        self.process = self.events = self.frames = self.stop_event = None
        self.capture_requests = None
        self.captures = []
        self.devices = []
        self.last_sample = None
        self._last_event = ""
        self._closing = False
        self._stop_deadline = None
        self._after = None
        self.choice = tk.StringVar()
        self.status = tk.StringVar(value="未连接。先发现设备，再选择相机预览；不会自动打开")
        self.title_label = ttk.Label(self, text="Hikrobot GigE 实物相机预览 / 机器人仍为 DEMO", font=("Microsoft YaHei UI", 12, "bold"))
        self.title_label.pack(anchor="w")
        row = ttk.Frame(self)
        row.pack(fill="x", pady=5)
        self.discover_button = ttk.Button(row, text="1. 发现相机", command=self.discover)
        self.discover_button.pack(side="left")
        self.selector = ttk.Combobox(row, textvariable=self.choice, state="readonly", width=67)
        self.selector.pack(side="left", fill="x", expand=True, padx=5)
        self.open_button = ttk.Button(row, text="2. 开始预览", command=self.start)
        self.open_button.pack(side="left")
        ttk.Button(row, text="停止 / 释放", command=self.stop).pack(side="left", padx=5)
        self.note_label = ttk.Label(self, text="只预览，不写入当前 DEMO episode。曝光、分辨率、IP、触发模式均保持原配置。", foreground="#9b4800")
        self.note_label.pack(anchor="w")
        ttk.Label(self, textvariable=self.status, wraplength=950).pack(anchor="w", pady=5)
        self.canvas = tk.Canvas(self, background="#101418", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.photo = None
        self._after = self.after(100, self._poll)

    def set_context_text(self, title, note):
        """嵌入不同只读界面时，明确说明相机数据用途，避免 DEMO/真机混淆。"""
        self.title_label.configure(text=title)
        self.note_label.configure(text=note)

    def _launch(self, mode, requested=None):
        if self.process is not None:
            return
        self.events = self.context.Queue()
        self.frames = self.context.Queue(maxsize=1)
        self.capture_requests = self.context.Queue()
        self.stop_event = self.context.Event()
        self.last_sample = None
        self._last_event = "正在发现相机…" if mode == "enum" else "正在打开所选相机…"
        self.status.set(self._last_event)
        self.process = self.context.Process(target=camera_worker,
            args=(mode, requested, self.events, self.frames, self.stop_event, self.capture_requests), daemon=True)
        try:
            self.process.start()
        except Exception:
            self._release_process()
            raise
        self.discover_button.configure(state="disabled")
        self.open_button.configure(state="disabled")
        self.selector.configure(state="disabled")

    def discover(self):
        self._launch("enum")

    def start(self):
        index = self.selector.current()
        if self.process is not None:
            return
        if not 0 <= index < len(self.devices):
            self.status.set("请先发现并选择相机")
            return
        selected = self.devices[index]
        if messagebox.askokcancel("打开实物相机（不操作机器人）",
                f"将采集 {selected['name']} / {selected['ip']}\n序列号 {selected['serial']}\n\n"
                "请先在 MVS 停止采集并关闭此相机。\n只启动/停止图像采集，不改拍摄参数，也不录入 episode。", parent=self):
            self._launch("preview", selected)

    def stop(self):
        if self.stop_event is not None:
            self.stop_event.set()
            self._stop_deadline = time.monotonic() + 4
            self.status.set("正在停止并释放相机…")

    def request_full_frame(self, token, path):
        if self.process is None or self.capture_requests is None or self.last_sample is None:
            raise RuntimeError("相机尚无实时完整帧")
        if time.monotonic() - self.last_sample["host_monotonic"] >= 2:
            raise RuntimeError("相机画面已过期，拒绝保存")
        self.capture_requests.put((str(token), str(_capture_path(path))))

    def pop_captures(self):
        values, self.captures = self.captures, []
        return values

    def _handle_event(self, kind, value):
        if kind == "devices":
            self.devices = value
            self.selector.configure(values=[f"{d['name']} | {d['ip']} | {d['serial']} | 网卡 {d['interface']}" for d in value])
            if value:
                preferred = next((i for i, d in enumerate(value)
                                  if DEFAULT_SERIAL and d['serial'] == DEFAULT_SERIAL), 0)
                self.selector.current(preferred)
            else:
                self.choice.set("")
            self._last_event = f"发现 {len(value)} 条设备路径（尚未连接）。选择目标后开始预览"
        elif kind in {"error", "status"}:
            self._last_event = value
        elif kind == "done":
            if value:
                self._last_event = "释放异常：" + value
            elif self._stop_deadline is not None:
                self._last_event = "相机已停止；正在退出 SDK 进程"
        elif kind == "capture":
            self.captures.append(value)
        self.status.set(self._last_event)

    def _show_sample(self, sample):
        from PIL import Image, ImageTk
        self.last_sample = sample
        image = Image.frombytes("RGB", sample["size"], sample["rgb"])
        image.thumbnail((max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())))
        self.photo = ImageTk.PhotoImage(image)
        self.canvas.delete("all")
        self.canvas.create_image(self.canvas.winfo_width()//2, self.canvas.winfo_height()//2, image=self.photo)

    def _release_process(self):
        if self.process is not None and self.process.pid is not None:
            self.process.join(0)
            self.process.close()
        for channel in (self.events, self.frames, self.capture_requests):
            if channel is not None:
                channel.close()
        self.process = self.events = self.frames = self.capture_requests = self.stop_event = None
        self._stop_deadline = None
        self.discover_button.configure(state="normal")
        self.open_button.configure(state="normal")
        self.selector.configure(state="readonly")

    def _poll(self):
        if self._closing:
            return
        if self.process is not None:
            try:
                while True:
                    self._handle_event(*self.events.get_nowait())
            except queue.Empty:
                pass
            try:
                self._show_sample(self.frames.get_nowait())
            except queue.Empty:
                pass
            if self.last_sample is not None:
                s = self.last_sample
                age = time.monotonic() - s["host_monotonic"]
                state = "实时预览" if age < 2 else "画面已过期（最后一帧，不是实时画面）"
                self.status.set(f"{state} | {s['camera']['name']} | {s['width']}×{s['height']} | frame {s['frame_id']} | "
                    f"接收均值 {s['fps']:.2f} fps | 帧龄 {age:.1f}s | 不完整帧 {s['incomplete']} | 超时 {s['timeouts']}\n{self._last_event}")
            if self._stop_deadline and time.monotonic() > self._stop_deadline and self.process.is_alive():
                self.process.terminate()
                self.process.join(1)
                self._last_event = "SDK 未及时退出，已终止本程序的采集子进程。相机可能需等待心跳超时才能重新打开"
            if not self.process.is_alive():
                # done 在进程退出前入队；再排空一次，避免丢失最后的错误信息。
                try:
                    while True:
                        self._handle_event(*self.events.get_nowait())
                except queue.Empty:
                    pass
                if self.process.exitcode:
                    self._last_event += f"；采集进程退出码 {self.process.exitcode}"
                self.status.set(self._last_event + ("；预览已停止" if self.last_sample else ""))
                self.canvas.delete("all")
                self.photo = self.last_sample = None
                self._release_process()
        self._after = self.after(100, self._poll)

    def close(self):
        self._closing = True
        if self._after:
            self.after_cancel(self._after)
        if self.process is not None:
            self.stop_event.set()
            self.process.join(3)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(1)
            self._release_process()


def main():
    root = tk.Tk()
    root.title("Hikrobot 相机预览验收（无机器人接口）")
    root.geometry("1100x780")
    panel = HikPreviewPanel(root)
    panel.pack(fill="both", expand=True)
    def close():
        panel.close()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", close)
    root.mainloop()


if __name__ == "__main__":
    mp.freeze_support()
    main()
