"""启动 Unity 数字孪生，并把 JAKA 实测关节反馈只读广播到 UDP 5006。

这个进程是中文“①启动VR与JAKA数字孪生.cmd”的后台主管：

* Unity 仍负责 Quest 输入和三维显示；
* 本文件仅调用 JAKA SDK 的读取接口；
* 单独运行①时，实测关节目标采样 60 Hz；TCP 30 Hz；状态/工具 5 Hz；
* ②开始时①先注销 SDK 并停止广播，由②独占连接与实测显示；②结束后①恢复；
* Unity 退出时，本文件注销只读连接并退出；
* JAKA 暂时不可达时，Unity 仍可启动，HUD 会显示只读反馈错误。

默认运行只显示说明。只有 ``--live-readonly`` 才会启动 Unity 并连接 JAKA。
本文件没有上电、使能、清报警、逆解或任何运动接口。
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace

if __package__ in (None, ""):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    __package__ = "vla_lab"

from .jaka_telemetry import ReadOnlySession, SDK_DIRECTORY, load_sdk, validate_host
from .vr_robot_visualization import RobotVrBroadcaster
from .sdk_trace import TraceFile, TracedSdk
from .sdk_owner_lease import SdkOwnerLease, READONLY_HANDOFF_PORT


# 按单调时钟的截止时间调度，避免“调用耗时 + 固定休眠”导致实际频率低于设定。
POLL_PERIOD_S = 1.0 / 60.0
RECONNECT_DELAY_S = 2.0
TARGET_OVERLAY_PORT = 5007


class TargetOverlayReceiver:
    """接收真机遥操作程序的“显示目标”，不接收也不转发运动命令。"""

    def __init__(self, port: int = TARGET_OVERLAY_PORT) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", int(port)))
        self._socket.setblocking(False)
        self._target = None
        self._expires_s = float("-inf")

    def latest(self):
        while True:
            try:
                data, _address = self._socket.recvfrom(8192)
            except BlockingIOError:
                break
            try:
                payload = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
                continue
            if payload.get("schema") != "quest_jaka_target_overlay.v1":
                continue
            raw = payload.get("target_tcp_mm_rad")
            if raw is None:
                self._target = None
                self._expires_s = float("-inf")
                continue
            try:
                target = tuple(float(value) for value in raw)
            except (TypeError, ValueError):
                continue
            if len(target) != 6 or not all(math.isfinite(value) for value in target):
                continue
            try:
                hold_s = max(0.1, min(60.0, float(payload.get("hold_s", 1.0))))
            except (TypeError, ValueError):
                continue
            self._target = target
            self._expires_s = time.monotonic() + hold_s
        if time.monotonic() > self._expires_s:
            self._target = None
        return self._target

    def close(self) -> None:
        self._socket.close()


class ReadonlyHandoffServer:
    """只接收本机②的“释放只读连接”请求，不接收任何运动目标。"""

    def __init__(self, port: int = READONLY_HANDOFF_PORT) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", port))
        self._socket.setblocking(False)

    def requests(self):
        while True:
            try:
                data, address = self._socket.recvfrom(512)
            except BlockingIOError:
                return
            except OSError:
                return
            if address[0] != "127.0.0.1":
                continue
            try:
                packet = json.loads(data.decode("ascii"))
            except (UnicodeError, ValueError):
                continue
            if (not isinstance(packet, dict)
                    or packet.get("schema") != "quest_jaka_readonly_handoff.v1"
                    or packet.get("action") != "release"):
                continue
            nonce = packet.get("nonce")
            if isinstance(nonce, str) and len(nonce) == 24 and all(c in "0123456789abcdef" for c in nonce):
                yield address, nonce

    def released(self, address, nonce: str) -> None:
        packet = {"schema": "quest_jaka_readonly_handoff.v1",
                  "action": "released", "nonce": nonce}
        self._socket.sendto(json.dumps(packet).encode("ascii"), address)

    def close(self) -> None:
        self._socket.close()


def snapshot(*, connected: bool, joints=None, tcp=None, tool_id=None,
             powered_on=False, enabled=False, error=""):
    """构造 ``RobotVrBroadcaster`` 所需的最小只读快照。"""
    return SimpleNamespace(
        connected=bool(connected),
        powered_on=bool(powered_on),
        enabled=bool(enabled),
        engineering_servo_active=False,
        tool_id=tool_id,
        joints_rad=joints,
        tcp_pose=tcp,
        error=str(error),
    )


def read_snapshot(session: ReadOnlySession):
    """只读四组必要反馈；拒绝异常维度、NaN 和未知状态格式。"""
    simple = session.call("get_robot_status_simple")[1]
    if not isinstance(simple, (tuple, list)) or len(simple) != 4:
        raise RuntimeError(f"get_robot_status_simple 格式异常：{simple!r}")
    if simple[2] not in (0, 1) or simple[3] not in (0, 1):
        raise RuntimeError(f"上电/使能状态异常：{simple!r}")
    joints = tuple(float(v) for v in session.call("get_actual_joint_position")[1])
    tcp = tuple(float(v) for v in session.call("get_actual_tcp_position")[1])
    if len(joints) != 6 or len(tcp) != 6 or not all(math.isfinite(v) for v in joints + tcp):
        raise RuntimeError("JAKA 实测关节或 TCP 不是有限六维值")
    tool_id = int(session.call("get_tool_id")[1])
    error_code = int(simple[0])
    error = "" if error_code == 0 else f"控制器错误码 {error_code}"
    return snapshot(
        connected=True,
        joints=joints,
        tcp=tcp,
        tool_id=tool_id,
        powered_on=bool(simple[2]),
        enabled=bool(simple[3]),
        error=error,
    )


class FastMeasuredReader:
    """关节每帧读取，较慢变化的状态分频读取；帧率是实际完成的采样率。

    每个关节包带主机读取起点和耗时。该时间不是控制器采集时间；Unity 不会把
    SDK 卡住数秒后返回的数据当成刚采集的新姿态。只读进程不做任何逆解。
    """
    def __init__(self):
        self.state = None
        self.status_due = self.tcp_due = 0.0
        self.window_start = time.perf_counter()
        self.count = 0
        self.hz = 0.0

    def read(self, session):
        started = time.perf_counter()
        if self.state is None or started >= self.status_due:
            self.state = read_snapshot(session)
            self.status_due = time.perf_counter() + 0.2
            self.tcp_due = time.perf_counter() + 1.0 / 30.0
        elif started >= self.tcp_due:
            tcp = tuple(float(v) for v in session.call("get_actual_tcp_position")[1])
            if len(tcp) != 6 or not all(math.isfinite(v) for v in tcp):
                raise RuntimeError("TCP 反馈无效")
            self.state.tcp_pose = tcp
            self.tcp_due = time.perf_counter() + 1.0 / 30.0
        sample_ns = time.time_ns()
        query_start = time.perf_counter()
        joints = tuple(float(v) for v in session.call("get_actual_joint_position")[1])
        if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
            raise RuntimeError("关节反馈无效")
        finished = time.perf_counter()
        if finished - started > 0.15:
            raise RuntimeError(f"只读采样耗时 {(finished - started)*1000:.1f} ms，丢弃迟到反馈")
        self.count += 1
        if finished - self.window_start >= 1.0:
            self.hz = self.count / (finished - self.window_start)
            self.window_start, self.count = finished, 0
        self.state.joints_rad = joints
        self.state.feedback_source = "readonly_bridge"
        self.state.sample_time_ns = sample_ns
        self.state.feedback_hz = self.hz
        self.state.query_ms = (finished - query_start) * 1000.0
        return self.state


def run(*, host: str, sdk_dir: Path, player_exe: Path, player_log: Path) -> int:
    host = validate_host(host)
    sdk_dir = sdk_dir.resolve()
    player_exe = player_exe.resolve()
    player_log = player_log.resolve()
    if not player_exe.is_file():
        raise FileNotFoundError(f"找不到 Unity Player：{player_exe}")
    player_log.parent.mkdir(parents=True, exist_ok=True)

    # ①在②未启动时持有SDK只读连接；②开始前请求①释放，结束后①再获取。
    # 5007保留为旧桥检测端口，防止旧版②误与①并行连接。
    handoff = ReadonlyHandoffServer()
    overlay = TargetOverlayReceiver()
    # 不使用 shell，路径中的中文、空格和括号都作为单独参数传给 Unity。
    player = subprocess.Popen(
        [str(player_exe), "-logFile", str(player_log)],
        cwd=str(player_exe.parent.parent),
    )
    # 读取循环负责 60 Hz 调度；发送端不再二次限到 30 Hz。
    broadcaster = RobotVrBroadcaster(max_hz=120.0)
    reader = FastMeasuredReader()
    deadline = time.perf_counter()
    sdk = None
    session = None
    lease = None
    next_connect_s = 0.0
    last_error = ""
    sdk_trace = TraceFile(player_log.with_name(player_log.stem + "_SDK只读.jsonl"))
    try:
        while player.poll() is None:
            now = time.monotonic()
            for address, nonce in handoff.requests():
                # 先注销真实连接，再释放进程互斥锁；任何注销失败都不回ACK。
                released = True
                if session is not None:
                    try:
                        session.close()
                    except Exception as error:
                        released = False
                        last_error = f"JAKA只读连接注销失败：{error}"
                    session = None
                if lease is not None:
                    lease.close()
                    lease = None
                next_connect_s = time.monotonic() + 3.0
                if released:
                    handoff.released(address, nonce)
                    print("[①] 已注销只读 SDK 并让位给②；②结束后自动恢复实测反馈。", flush=True)
                else:
                    print(f"[①] 注销失败，拒绝向②确认交接：{last_error}", flush=True)
            if lease is None and now >= next_connect_s:
                try:
                    lease = SdkOwnerLease()
                except RuntimeError:
                    next_connect_s = now + RECONNECT_DELAY_S
            if lease is not None and session is None and now >= next_connect_s:
                try:
                    if sdk is None:
                        sdk = load_sdk(sdk_dir)
                    session = ReadOnlySession(sdk, host)
                    session.robot = TracedSdk(session.robot, sdk_trace)
                    session.connect()
                    reader = FastMeasuredReader()
                    last_error = ""
                    print("[①] JAKA 只读连接成功，正在向 Unity 广播实测关节。", flush=True)
                except Exception as error:
                    if session is not None:
                        try: session.close()
                        except Exception: pass
                    session = None
                    last_error = f"JAKA只读连接失败：{error}"
                    print(f"[①] {last_error}；VR 保持打开，稍后重试。", flush=True)
                    next_connect_s = now + RECONNECT_DELAY_S
            if lease is None:
                # ②独占SDK时，①不发送“断连”包覆盖②的实测关节反馈。
                deadline += POLL_PERIOD_S
                now_tick = time.perf_counter()
                if deadline < now_tick:
                    deadline = now_tick
                time.sleep(max(0.0, deadline - now_tick))
                continue
            if session is not None:
                try:
                    state = reader.read(session)
                    last_error = ""
                except Exception as error:
                    last_error = f"JAKA只读反馈失败：{error}"
                    print(f"[①] {last_error}；丢弃迟到/无效数据并重新连接。", flush=True)
                    try:
                        session.close()
                    except Exception:
                        pass
                    session = None
                    next_connect_s = now + RECONNECT_DELAY_S
                    state = snapshot(connected=False, error=last_error)
            else:
                state = snapshot(connected=False, error=last_error or "等待JAKA只读连接")
            broadcaster.publish(
                state,
                armed=False,
                starting=False,
                active=False,
                target_tcp_mm_rad=overlay.latest(),
                robot_simulated=False,
            )
            deadline += POLL_PERIOD_S
            now = time.perf_counter()
            if deadline < now:
                deadline = now  # 不补发积压的旧帧。
            time.sleep(max(0.0, deadline - now))
        return int(player.returncode or 0)
    finally:
        if session is not None:
            try:
                session.close()
            except Exception:
                pass
        broadcaster.close()
        overlay.close()
        handoff.close()
        sdk_trace.close()
        if lease is not None:
            lease.close()
        if player.poll() is None:
            player.terminate()
            try:
                player.wait(2.0)
            except subprocess.TimeoutExpired:
                player.kill()


def run_vr_only(*, player_exe: Path, player_log: Path) -> int:
    """仅供离线VR诊断：只启动 Unity，绝不连接控制器。

    没有实测反馈时数字孪生应显示反馈过期，不能用手柄目标伪装机器人姿态。
    """
    player_exe = player_exe.resolve()
    player_log = player_log.resolve()
    if not player_exe.is_file():
        raise FileNotFoundError(f"找不到 Unity Player：{player_exe}")
    player_log.parent.mkdir(parents=True, exist_ok=True)
    player = subprocess.Popen(
        [str(player_exe), "-logFile", str(player_log)], cwd=str(player_exe.parent.parent)
    )
    try:
        return int(player.wait() or 0)
    finally:
        if player.poll() is None:
            player.terminate()
            try:
                player.wait(2.0)
            except subprocess.TimeoutExpired:
                player.kill()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-readonly", action="store_true")
    parser.add_argument("--vr-only", action="store_true", help="仅离线VR诊断；不连接 JAKA")
    # 公共源码不携带现场控制柜地址。
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--sdk-dir", type=Path, default=SDK_DIRECTORY)
    parser.add_argument("--player-exe", type=Path)
    parser.add_argument("--player-log", type=Path)
    args = parser.parse_args(argv)
    if args.live_readonly and args.vr_only:
        parser.error("--live-readonly 与 --vr-only 不能同时使用")
    if not (args.live_readonly or args.vr_only):
        print("默认不启动 Unity、不连接 JAKA。中文①入口显式使用 --live-readonly。")
        return 0
    if args.player_exe is None or args.player_log is None:
        parser.error("启动 Unity 必须同时提供 --player-exe 与 --player-log")
    if args.vr_only:
        return run_vr_only(player_exe=args.player_exe, player_log=args.player_log)
    return run(
        host=args.host,
        sdk_dir=args.sdk_dir,
        player_exe=args.player_exe,
        player_log=args.player_log,
    )


if __name__ == "__main__":
    raise SystemExit(main())
