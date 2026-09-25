"""把 JAKA 的真实反馈发送给 Unity 数字孪生（只读 UDP）。

为什么单独做这一层
------------------
Unity 负责 VR 显示，JAKA SDK 负责真机通信；两者如果直接互相引用，会把厂商 SDK、
Unity 主线程和机器人控制线程绑死在一起。这里采用一个很小的 UDP 数据契约：Python
只广播“已测量关节角/末端位姿/当前状态”，Unity 只接收并画模型，绝不向机器人下命令。

这样做还有一个重要的安全含义：VR 中的实体模型跟随 ``joints_rad``（控制器真实反馈），
而不是跟随手柄目标。若逆解失败或机器人未运动，VR 模型也不会假装已经运动。
"""

from __future__ import annotations

import json
import socket
import time
from typing import Any


class RobotVrBroadcaster:
    """在 GUI 主循环中以有限频率广播最新机器人反馈。

    这是“尽力而为”的显示通道：发包失败只记录错误，绝不能反过来卡住或改变机器人
    控制。UDP 默认发往本机 5006，与 Quest 手柄输入的 5005 完全分离。
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 5006, *, max_hz: float = 30.0) -> None:
        self.destination = (host, int(port))
        self.minimum_interval_s = 1.0 / max(1.0, float(max_hz))
        self._last_send_s = float("-inf")
        self._sequence = 0
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.error = ""

    @staticmethod
    def _numbers(values: Any) -> list[float] | None:
        if values is None:
            return None
        return [float(value) for value in values]

    def publish(
        self,
        snapshot: Any,
        *,
        armed: bool,
        starting: bool,
        active: bool,
        target_tcp_mm_rad: tuple[float, ...] | None,
        hand_state: dict | None = None,
        robot_simulated: bool = False,
    ) -> None:
        """发送一帧；调用者可每 20 ms 调用，本类会自动限频。"""

        now_s = time.perf_counter()
        if now_s - self._last_send_s < self.minimum_interval_s:
            return
        self._last_send_s = now_s
        self._sequence += 1
        packet = {
            "schema": "quest_jaka_robot_state.v1",
            # ①只读与②独占会话均可广播实测反馈；来源用于诊断而非运动许可。
            # 此字段不参与任何机器人控制。
            "feedback_source": str(getattr(snapshot, "feedback_source", "readonly_bridge")),
            "robot_simulated": bool(robot_simulated),
            "sequence": self._sequence,
            "sent_time_ns": time.time_ns(),
            "sample_time_ns": int(getattr(snapshot, "sample_time_ns", 0)),
            "feedback_hz": float(getattr(snapshot, "feedback_hz", 0)),
            "query_ms": float(getattr(snapshot, "query_ms", 0)),
            "connected": bool(snapshot.connected),
            "powered_on": bool(snapshot.powered_on),
            "enabled": bool(snapshot.enabled),
            "armed": bool(armed),
            "servo_starting": bool(starting),
            "servo_active": bool(snapshot.engineering_servo_active and active),
            # Unity 的 JsonUtility 对值类型 null 兼容性差；未知工具用 -1 明确表示。
            "tool_id": -1 if snapshot.tool_id is None else int(snapshot.tool_id),
            # 真实测量值：Unity 实体模型只允许使用这两个字段。
            "joints_rad": self._numbers(snapshot.joints_rad),
            "tcp_pose_mm_rad": self._numbers(snapshot.tcp_pose),
            # 期望值只画成小型坐标轴，不可拿来冒充真机反馈。
            "target_tcp_mm_rad": self._numbers(target_tcp_mm_rad),
            "error": str(snapshot.error or ""),
        }
        hand = hand_state or {}
        packet.update({
            'hand_connected': bool(hand.get('connected')),
            'hand_feedback_valid': bool(hand.get('feedback_valid')),
            'hand_simulated': bool(hand.get('simulated')),
            'hand_angles_deg': self._numbers(hand.get('angles_deg')),
            'hand_status': str(hand.get('status', '未连接灵巧手')),
        })
        try:
            payload = json.dumps(packet, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            self._socket.sendto(payload, self.destination)
            self.error = ""
        except OSError as exc:
            # 数字孪生断开不能影响机器人 STOP/看门狗，所以这里只留下诊断文字。
            self.error = str(exc)

    def close(self) -> None:
        try:
            self._socket.close()
        except OSError:
            pass
