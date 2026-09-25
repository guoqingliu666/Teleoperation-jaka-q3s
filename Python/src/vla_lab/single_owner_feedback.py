"""②独占 JAKA SDK 连接时给 Unity 发送实测反馈。

此模块只读关节、TCP 和状态，不计算目标、不下发运动。①在此架构下只负责
Unity/Quest，不再另开一个 JAKA SDK 连接。显示故障不能伪造姿态；读数故障
交给②的会话异常处理，保留停止确认流程。
"""
from __future__ import annotations

from types import SimpleNamespace
import math
import time

from .sampled_follow import checked, six
from .vr_robot_visualization import RobotVrBroadcaster


class SingleOwnerFeedback:
    """与运动调用在同一线程串行读 SDK，避免跨进程/跨线程共享厂商句柄。"""

    def __init__(self, robot, *, period_s: float = 1 / 30, broadcaster=None,
                 clock=time.monotonic, wall_clock_ns=time.time_ns):
        if not math.isfinite(period_s) or period_s < 1 / 120:
            raise ValueError("实测反馈周期无效")
        self.robot = robot
        self.period_s = period_s
        self.broadcaster = broadcaster or RobotVrBroadcaster(max_hz=120)
        self.clock = clock
        self.wall_clock_ns = wall_clock_ns
        self.next_due = 0.0
        self.status_due = 0.0
        self.status = None
        self.count = 0
        self.window_start = clock()
        self.hz = 0.0

    def tick(self) -> bool:
        """到期时串行读取一帧并广播；未到期不访问 SDK。

        先读状态，再读关节/TCP；若这一帧读取超过时效门槛就报错，
        不把迟到的旧姿态包装成“实时”反馈。返回值表示本轮是否发出新帧。
        """
        now = self.clock()
        if now < self.next_due:
            return False
        self.next_due = now + self.period_s
        started = self.clock()
        sample_ns = self.wall_clock_ns()
        if self.status is None or now >= self.status_due:
            raw = checked(self.robot.get_robot_status_simple(), "实测显示状态")
            if not isinstance(raw, (tuple, list)) or len(raw) != 4 or raw[2] not in (0, 1) or raw[3] not in (0, 1):
                raise RuntimeError("实测显示状态格式异常")
            tool = checked(self.robot.get_tool_id(), "实测显示Tool")
            if type(tool) is not int:
                raise RuntimeError("实测显示Tool格式异常")
            self.status = (bool(raw[2]), bool(raw[3]), tool, int(raw[0]))
            # 运动安全状态由SampledFollower在10Hz及每次下发前核对；显示层只需
            # 每秒刷新电源/使能/Tool，避免和控制层重复轰击同一SDK连接。
            self.status_due = self.clock() + 1.0
        joints = six(checked(self.robot.get_actual_joint_position(), "实测显示关节"))
        tcp = six(checked(self.robot.get_actual_tcp_position(), "实测显示TCP"))
        ended = self.clock()
        if ended - started > 0.15:
            raise RuntimeError("实测显示反馈迟到，拒绝当作新姿态")
        self.count += 1
        if ended - self.window_start >= 1.0:
            self.hz = self.count / (ended - self.window_start)
            self.window_start, self.count = ended, 0
        power, enabled, tool, error_code = self.status
        state = SimpleNamespace(
            connected=True, powered_on=power, enabled=enabled,
            engineering_servo_active=False, tool_id=tool,
            joints_rad=joints, tcp_pose=tcp,
            feedback_source="single_owner", sample_time_ns=sample_ns,
            feedback_hz=self.hz, query_ms=(ended - started) * 1000,
            error="" if error_code == 0 else f"控制器错误码 {error_code}",
        )
        self.broadcaster.publish(state, armed=False, starting=False, active=False,
                                 target_tcp_mm_rad=None, robot_simulated=False)
        return True

    def close(self):
        """释放 UDP 广播器；SDK 连接仍由上层会话统一注销。"""
        self.broadcaster.close()
