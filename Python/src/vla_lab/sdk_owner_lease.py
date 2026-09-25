"""本机 JAKA SDK 连接独占闸门。

只在 Python 进程间协调，不声称能限制厂商上位机或另一台电脑连接。端口仅用作
本机互斥锁，不接受外来命令。进程退出时由操作系统释放，不留陈旧锁文件。
"""
from __future__ import annotations

import socket
import json
import secrets

PORT = 5011
LEGACY_BRIDGE_PORT = 5007
READONLY_HANDOFF_PORT = 5012


def request_readonly_handoff(*, port: int = READONLY_HANDOFF_PORT,
                             timeout_s: float = 1.5) -> bool:
    """请求新版①先注销只读 SDK；旧版/不存在的①不会获得运动许可。

    这里只发送本机进程间的交接信号，不包含任何机器人运动指令。返回 True
    仅表示①确认已经释放自己的连接；②仍须独立获取互斥锁并检查机器人。
    """
    nonce = secrets.token_hex(12)
    message = json.dumps({"schema": "quest_jaka_readonly_handoff.v1",
                          "action": "release", "nonce": nonce}).encode("ascii")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            probe.settimeout(max(0.01, timeout_s))
            probe.sendto(message, ("127.0.0.1", port))
            data, address = probe.recvfrom(512)
    except (socket.timeout, OSError):
        # Windows 对无人监听的本机 UDP 端口可能返回 WSAECONNRESET。
        return False
    try:
        answer = json.loads(data.decode("ascii"))
    except (UnicodeError, ValueError):
        return False
    return (isinstance(answer, dict) and address[0] == "127.0.0.1"
            and answer.get("schema") == "quest_jaka_readonly_handoff.v1"
            and answer.get("action") == "released"
            and answer.get("nonce") == nonce)


def reject_legacy_bridge(*, port: int = LEGACY_BRIDGE_PORT):
    """防止更新前已经运行的①桥仍持有SDK，却不知道新的互斥闸门。"""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("127.0.0.1", port))
    except OSError:
        raise RuntimeError("检测到旧①只读桥/端口5007占用；请先关闭旧①再启动②") from None
    finally:
        probe.close()


class SdkOwnerLease:
    def __init__(self, *, port: int = PORT):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind(("127.0.0.1", port))
            sock.listen(1)
        except OSError:
            sock.close()
            raise RuntimeError("本机已有另一个 JAKA SDK 连接所有者；拒绝双连接") from None
        self._socket = sock

    def close(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None
