"""新②本机空载节拍自检：只调用假 SDK，不连接机器人。

PyCharm 可直接运行本文件。测试期间按 25 Hz 更新模拟目标、按 125 Hz 调用
离线会话，持续约 3 秒。它只能衡量此 Python 进程的空载表现，不能代表真实
JAKA SDK 逆解、网络、控制器、Windows 背景负载和物理停机性能。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from vla_lab.vendor_joint_teleop_candidate import JointLpfCutoff, RejectLimits, Sample  # noqa: E402
from vla_lab.vendor_joint_teleop_session import OfflineJointServoSession, SessionTiming  # noqa: E402


class FakeSdk:
    __vla_offline_test_double__ = True

    def __init__(self) -> None:
        self.servo_enabled = False

    def kine_inverse(self, reference, pose):
        return (0, reference)

    def servo_move_use_joint_LPF(self, *limits):
        return (0,)

    def servo_move_enable(self, enabled, is_block):
        self.servo_enabled = enabled
        return (0,)

    def is_in_servomove(self):
        return (0, self.servo_enabled)

    def servo_j(self, joints, mode, steps):
        return (0,)


def sample(now: float) -> Sample:
    pose = (100.0, 200.0, 300.0, 0.0, 0.0, 0.0)
    return Sample(
        target_pose=pose, actual_tcp_pose=pose, actual_joints=(0.0,) * 6,
        input_time_s=now, feedback_time_s=now,
        tracked=True, grip_held=True, powered=True, enabled=True,
        estop=False, collision=False, on_limit=False, tool_id=1,
    )


def main() -> int:
    now = time.perf_counter()
    worker = OfflineJointServoSession(
        FakeSdk(),
        reject_limits=RejectLimits(0.1, 0.1, 5.0, 5.0, 0.2, 0.007, 50.0, 20.0),
        # 下面数值只是让假 SDK 接受调用，不是供真机使用的滤波器参数。
        lpf_cutoff=JointLpfCutoff(0.5),
        timing=SessionTiming(0.003, 0.004, 0.09, 4.0),
    )
    if not worker.start(sample(now)) or not worker.tick():
        print(f"初始离线发送失败：{worker.stop_reason}")
        return 1
    started = time.perf_counter()
    errors_ms = []
    for index in range(1, 376):
        deadline = started + index * 0.008
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        current = time.perf_counter()
        errors_ms.append((current - deadline) * 1000.0)
        if index % 5 == 0 and not worker.update_target(sample(current)):
            break
        if not worker.tick():
            break
    worker.stop("空载基准结束")
    print(
        f"仅假 SDK：已确认发送={worker.sent_count}，尝试发送={worker.send_attempt_count}，"
        f"最大调度迟到={max(errors_ms, default=0):.3f} ms，"
        f"会话结果={worker.stop_reason}，停止调用确认={worker.stop_confirmed}"
    )
    return 0 if worker.stop_reason == "空载基准结束" else 2


if __name__ == "__main__":
    raise SystemExit(main())
