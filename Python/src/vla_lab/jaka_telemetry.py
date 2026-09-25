"""独立 JAKA 只读验收器；不依赖 Jog 控制器，不提供运动或 IO 写接口。"""
from __future__ import annotations

import importlib
import ipaddress
import math
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time


SDK_DIRECTORY = Path(os.environ.get(
    "QUEST_JAKA_SDK_DIR",
    "__SET_QUEST_JAKA_SDK_DIR_IN_LOCAL_CONFIG__",
))
READ_OPERATIONS = frozenset({
    "login", "logout", "get_sdk_version", "get_actual_joint_position",
    "get_tcp_position", "get_actual_tcp_position", "get_tool_id",
    "get_tool_data", "get_user_frame_id", "get_robot_status_simple",
    "get_motion_status",
})
REQUIRED_METHODS = ("login", "logout", "get_actual_joint_position", "get_tcp_position", "get_tool_id")
_DLL_HANDLES = []  # 必须保持句柄存活，否则 Windows 会移除 DLL 搜索目录。


def validate_host(value: str) -> str:
    """只接受用户明确填写的实验室私网 IPv4，不扫描、不猜地址。"""
    try:
        address = ipaddress.IPv4Address(value.strip())
    except ipaddress.AddressValueError as exc:
        raise ValueError("请填写平板显示的控制柜 IPv4；不能填写端口或相机 IP。") from exc
    networks = ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16")
    if not any(address in ipaddress.IPv4Network(n) for n in networks):
        raise ValueError("只读验收仅接受实验室私网 IPv4。")
    return str(address)


def load_sdk(directory: str | Path):
    """仅在隔离子进程中加载 SDK；此函数不会创建 RC，更不会登录。"""
    directory = Path(directory).resolve()
    for filename in ("jkrc.pyd", "jakaAPI.dll"):
        if not (directory / filename).is_file():
            raise FileNotFoundError(f"SDK 缺少 {filename}：{directory}")
    if os.name != "nt":
        raise RuntimeError("这个入口使用 Windows x64 SDK；Ubuntu 须使用对应 Linux SDK。")
    _DLL_HANDLES.append(os.add_dll_directory(str(directory)))
    sys.path.insert(0, str(directory))
    sdk = importlib.import_module("jkrc")
    if Path(sdk.__file__).resolve().parent != directory:
        raise RuntimeError("实际加载的 jkrc 来自其他目录，请重启独立检查窗口。")
    missing = [name for name in REQUIRED_METHODS if not hasattr(sdk.RC, name)]
    if missing:
        raise RuntimeError(f"SDK 缺少必需接口：{missing}")
    return sdk


def preflight(directory: str | Path) -> dict:
    sdk = load_sdk(directory)
    return {
        "python": sys.version, "module": sdk.__file__,
        "methods": {name: hasattr(sdk.RC, name) for name in sorted(READ_OPERATIONS)},
        "robot_constructed": False, "robot_connected": False,
        "scope": "import_and_method_presence_only",
    }


def six_numbers(values) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError("SDK 位姿/关节返回值不是 6 个数。")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
        raise ValueError("SDK 位姿包含非数值。")
    result = [float(v) for v in values]
    if not all(math.isfinite(v) for v in result):
        raise ValueError("SDK 位姿包含 NaN/Inf，拒绝作为实时状态。")
    return result


def frame_id(value) -> int:
    if type(value) is not int or not 0 <= value <= 15:
        raise ValueError("SDK 坐标系 ID 返回格式异常。")
    return value


class ReadOnlySession:
    """所有 SDK 调用必须经过精确白名单；未知状态不会填成正常/零。"""

    def __init__(self, sdk, host: str):
        self.host = validate_host(host)
        self.robot = sdk.RC(self.host)

    def call(self, name: str, *args):
        if name not in READ_OPERATIONS:
            raise PermissionError(f"只读入口禁止接口：{name}")
        result = getattr(self.robot, name)(*args)
        if not isinstance(result, tuple) or not result or type(result[0]) is not int:
            raise ValueError(f"{name} 返回结构异常")
        if result[0] != 0:
            raise RuntimeError(f"{name} 失败，SDK 返回码 {result[0]}")
        return result

    def connect(self):
        self.call("login")

    def close(self):
        # 连接部分失败时也尝试释放；绝不调用 motion_abort / power_off。
        self.call("logout")

    def sample(self) -> dict:
        started = time.monotonic_ns()
        warnings = []
        def optional(name, *args):
            try:
                return self.call(name, *args)
            except Exception as exc:
                warnings.append(f"{name}: {exc}")
                return None

        tool_before = frame_id(self.call("get_tool_id")[1])
        user_before = optional("get_user_frame_id")
        joints = six_numbers(self.call("get_actual_joint_position")[1])
        tcp = six_numbers(self.call("get_tcp_position")[1])
        actual_result = optional("get_actual_tcp_position")
        actual_tcp = None
        if actual_result is not None:
            try:
                actual_tcp = six_numbers(actual_result[1])
            except (ValueError, IndexError) as exc:
                warnings.append(f"get_actual_tcp_position: {exc}")
        status_result = optional("get_robot_status_simple")
        status = {"error_code": None, "powered_on": None, "enabled": None}
        if status_result is not None:
            try:
                data = status_result[1]
                if not isinstance(data, (tuple, list)) or len(data) != 4:
                    raise ValueError("状态结构与 1.7.2 simple 文档不一致")
                if type(data[0]) is not int or data[2] not in (0, 1) or data[3] not in (0, 1):
                    raise ValueError("状态值异常")
                status = {"error_code": data[0], "powered_on": bool(data[2]), "enabled": bool(data[3])}
            except (ValueError, IndexError) as exc:
                warnings.append(f"get_robot_status_simple: {exc}")
        # 工具偏置不是世界坐标；只显示，不自动把截图 hand 参数写入控制柜。
        tool_result = optional("get_tool_data", tool_before)
        tool_offset = None
        if tool_result is not None:
            try:
                if len(tool_result) != 3 or frame_id(tool_result[1]) != tool_before:
                    raise ValueError("工具数据格式或 ID 与文档不一致")
                tool_offset = six_numbers(tool_result[2])
            except (ValueError, IndexError) as exc:
                warnings.append(f"get_tool_data: {exc}")
        tool_after = frame_id(self.call("get_tool_id")[1])
        user_after = optional("get_user_frame_id")
        if tool_before != tool_after:
            raise ValueError("采样期间工具 ID 改变：此帧丢弃，请核对平板设置。")
        user_id = None
        if user_before is not None and user_after is not None:
            user_id = frame_id(user_before[1])
            if user_id != frame_id(user_after[1]):
                raise ValueError("采样期间用户坐标系 ID 改变：此帧丢弃。")
        else:
            warnings.append("用户坐标系未知；不要直接与平板 XYZ 比较。")
        finished = time.monotonic_ns()
        return {
            "schema": "jaka.readonly.telemetry.v1", "source": "REAL_JAKA_READ_ONLY",
            "host": self.host, "host_time_unix_ns": time.time_ns(),
            "host_query_started_monotonic_ns": started,
            "host_query_finished_monotonic_ns": finished,
            "query_span_ms": (finished - started) / 1e6,
            "timestamp_note": "host query interval, NOT controller capture time; sequential SDK queries",
            "joints_rad": joints, "tcp_mm_rad": tcp, "actual_tcp_mm_rad": actual_tcp,
            "active_tool_id": tool_after, "active_user_frame_id": user_id,
            "tool_offset_mm_rad": tool_offset, "status": status,
            "warnings": warnings, "hand_feedback": None,
            "hardware_motion_commanded": False,
            "tcp_frame_note": "SDK current-tool pose; verify reference frame against tablet before use",
        }


def worker(pipe, stop, mode: str, host: str, directory: str):
    """SDK 网络等待不得冻结主界面；出错后退出，不自动重连。"""
    session = None
    try:
        info = preflight(directory)
        pipe.send(("preflight", info))
        if mode == "preflight":
            return
        session = ReadOnlySession(sys.modules["jkrc"], host)
        session.connect()
        while not stop.is_set():
            pipe.send(("sample", session.sample()))
            stop.wait(0.2)  # 约 5 Hz 验收，不用它宣称完成高频同步数据采集。
    except Exception as exc:
        pipe.send(("error", f"{type(exc).__name__}: {exc}"))
    finally:
        if session is not None:
            try:
                session.close()
            except Exception as exc:
                try:
                    pipe.send(("warning", f"SDK 注销未确认：{exc}"))
                except (OSError, EOFError):
                    pass
        pipe.close()


class TelemetryProcess:
    """启动、轮询和终止本程序自己的 SDK 子进程；没有重连逻辑。"""

    def __init__(self):
        self.process = self.pipe = self.stop_event = None
        self.started = self.last_message = 0.0

    def start(self, mode, host="", directory=SDK_DIRECTORY):
        if mode not in ("preflight", "read"):
            raise ValueError("未知模式")
        if self.process is not None:
            raise RuntimeError("请先断开上次会话")
        if mode == "read":
            host = validate_host(host)
        context = mp.get_context("spawn")
        self.pipe, child = context.Pipe(duplex=False)
        self.stop_event = context.Event()
        self.process = context.Process(target=worker, args=(child, self.stop_event, mode, host, str(directory)), daemon=True)
        self.process.start()
        child.close()
        self.started = self.last_message = time.monotonic()

    def poll(self):
        messages = []
        if self.pipe is None:
            return messages
        try:
            while self.pipe.poll():
                messages.append(self.pipe.recv())
                self.last_message = time.monotonic()
        except (EOFError, OSError):
            messages.append(("closed", "SDK 会话已结束"))
            self.close()
        if self.process is not None and time.monotonic() - self.last_message > 10:
            messages.append(("error", "SDK 超过 10 秒无回应；关闭本次会话，不自动重连。"))
            self.close()
        elif self.process is not None and not self.process.is_alive():
            if self.process.exitcode:
                messages.append(("error", f"SDK 子进程异常退出（{self.process.exitcode}），请检查 DLL/控制器兼容性。"))
            messages.append(("closed", "SDK 子进程已退出"))
            self.close()
        return messages

    def close(self):
        forced = False
        if self.stop_event is not None:
            self.stop_event.set()
        if self.process is not None:
            self.process.join(0.3)
            if self.process.is_alive():
                forced = True
                self.process.terminate()
                self.process.join(0.3)
            self.process.close()
        if self.pipe is not None:
            self.pipe.close()
        self.process = self.pipe = self.stop_event = None
        return forced


def fresh(sample, max_age_s=2.0):
    # 采用查询起点，避免把一个耗时很久的批次当作新鲜帧。
    return sample is not None and 0 <= (time.monotonic_ns() - sample["host_query_started_monotonic_ns"]) / 1e9 <= max_age_s
