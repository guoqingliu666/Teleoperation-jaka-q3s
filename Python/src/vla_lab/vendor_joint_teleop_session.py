"""新②的 SDK 会话离线执行器；只能作用于显式标记的测试替身。

运行顺序：检查新鲜反馈和输入 → JAKA kine_inverse → 配置厂商关节 LPF →
启动伺服 → 定期向测试替身发送 *原样逆解* 的 servo_j 目标。

为什么单独写：真机入口目前硬锁，而且 Windows/Python + SDK 是否能满足
8 ms 节拍尚未实测。本文件不导入 jkrc、不创建 RC、不登录机器人，并拒绝
任何没有 ``__vla_offline_test_double__`` 标记的对象。它是协议演练，不是
真机适配器或安全认证。超时、SDK 错误均锁存停机，不自动重新 ARM。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Protocol

from .vendor_joint_teleop_candidate import (
    JointLpfCutoff,
    OfflineVendorIkCandidate,
    RejectLimits,
    Sample,
)


SDK_SERVO_PERIOD_S = 0.008  # 本机 JAKA SDK 头文件 servo_j 的 step_num=1 周期。


class OfflineSdk(Protocol):
    __vla_offline_test_double__: bool

    def kine_inverse(self, reference: tuple[float, ...], pose: tuple[float, ...]) -> object: ...
    def servo_move_use_joint_LPF(self, cutoff_frequency: float) -> object: ...
    def servo_move_enable(self, enabled: bool, is_block: bool) -> object: ...
    def is_in_servomove(self) -> object: ...
    def servo_j(self, joints: tuple[float, ...], mode: int, steps: int) -> object: ...


@dataclass(frozen=True)
class SessionTiming:
    """节拍拒绝门槛。数值无默认值；只能用于离线故障注入。"""

    max_tick_lateness_s: float
    max_servo_call_s: float
    max_target_silence_s: float
    max_session_s: float

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须为正的有限数")
        if self.max_tick_lateness_s >= SDK_SERVO_PERIOD_S:
            raise ValueError("节拍迟到门槛不得达到完整 8 ms 周期")
        if self.max_servo_call_s >= SDK_SERVO_PERIOD_S:
            raise ValueError("SDK 调用耗时门槛不得达到完整 8 ms 周期")


def _ok(result: object, method: str) -> None:
    if not isinstance(result, tuple) or not result or type(result[0]) is not int or result[0] != 0:
        raise RuntimeError(f"{method} 返回失败：{result!r}")


def _servo_mode_is(sdk: OfflineSdk, expected: bool) -> None:
    """API 成功不等于状态已切换，必须用 SDK 读回确认。"""
    result = sdk.is_in_servomove()
    _ok(result, "is_in_servomove")
    if len(result) < 2 or result[1] is not expected:
        raise RuntimeError(f"伺服模式读回不是 {expected}：{result!r}")


class OfflineJointServoSession:
    """假 SDK 上的单次 ARM 会话；实例停机后不可重新启动。"""

    def __init__(
        self,
        sdk: OfflineSdk,
        *,
        reject_limits: RejectLimits,
        lpf_cutoff: JointLpfCutoff,
        timing: SessionTiming,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        if getattr(sdk, "__vla_offline_test_double__", False) is not True:
            raise RuntimeError("本执行器只能用于离线 SDK 测试替身，不允许连接真机")
        lpf_cutoff.sdk_call()  # 在任何状态变化前验证参数；不写入控制器。
        self._sdk = sdk
        self._lpf_cutoff = lpf_cutoff
        self._reject_limits = reject_limits
        self._timing = timing
        self._clock = clock
        self._evaluator = OfflineVendorIkCandidate(reject_limits, clock=clock)
        self.active = False
        self.stopped = False
        self.stop_reason: str | None = None
        self.stop_confirmed = False
        self._target: tuple[float, ...] | None = None
        self._target_time = 0.0
        self._input_time = 0.0
        self._feedback_time = 0.0
        self._started = 0.0
        self._next_tick = 0.0
        self.sent_count = 0
        self.send_attempt_count = 0

    def _disable(self, reason: str) -> None:
        """停用伺服并锁存。失败必须显式报告，不能假装机器人已经停住。"""
        if self.stopped:
            return
        self.active = False
        self.stopped = True
        self.stop_reason = reason
        try:
            _ok(self._sdk.servo_move_enable(False, True), "servo_move_enable(False)")
            _servo_mode_is(self._sdk, False)
            self.stop_confirmed = True
        except Exception as error:
            self.stop_confirmed = False
            self.stop_reason += f"；SDK 停止未确认：{error}"

    def start(self, sample: Sample) -> bool:
        """先求解/核验，再配置厂商滤波器和开启伺服；不自动上电或使能。"""
        if self.active or self.stopped:
            raise RuntimeError("会话已启动或已锁存停机，不允许重用")
        decision = self._evaluator.evaluate(self._sdk, sample)
        if not decision.accepted:
            # 逆解失败时尚未开启伺服，无须调用 SDK 停止。
            self.stopped = True
            self.stop_reason = decision.reason
            return False
        assert decision.servo_call is not None
        self._target = decision.servo_call[1][0]
        self._target_time = self._clock()
        self._input_time = sample.input_time_s
        self._feedback_time = sample.feedback_time_s
        try:
            _ok(
                self._sdk.servo_move_use_joint_LPF(self._lpf_cutoff.cutoff_frequency),
                "servo_move_use_joint_LPF",
            )
            _ok(self._sdk.servo_move_enable(True, True), "servo_move_enable(True)")
            _servo_mode_is(self._sdk, True)
        except Exception as error:
            self._disable(f"启动 SDK 失败：{error}")
            return False
        self.active = True
        self._started = self._clock()
        self._next_tick = self._started
        # 第一帧也交给统一节拍函数；此处不额外发送一条可能重复的运动命令。
        return True

    def update_target(self, sample: Sample) -> bool:
        """每帧重新用实测关节参考 SDK 逆解；拒绝后必须先确认停伺服。"""
        if not self.active:
            return False
        decision = self._evaluator.evaluate(self._sdk, sample)
        if not decision.accepted:
            self._disable(decision.reason)
            return False
        assert decision.servo_call is not None
        self._target = decision.servo_call[1][0]
        self._target_time = self._clock()
        self._input_time = sample.input_time_s
        self._feedback_time = sample.feedback_time_s
        return True

    def tick(self) -> bool:
        """一次 8 ms 发送机会；调用者负责调度，本函数检测迟到而不补发旧包。"""
        if not self.active:
            return False
        now = self._clock()
        if now - self._started >= self._timing.max_session_s:
            self._disable("会话时限已到")
            return False
        if now - self._target_time > self._timing.max_target_silence_s:
            self._disable("目标长时间未更新")
            return False
        if (now - self._input_time > self._reject_limits.max_input_age_s
                or now - self._feedback_time > self._reject_limits.max_feedback_age_s):
            self._disable("发送前输入或反馈过期")
            return False
        if now < self._next_tick:
            return False
        if now - self._next_tick > self._timing.max_tick_lateness_s:
            self._disable("伺服发送节拍迟到")
            return False
        assert self._target is not None
        started = self._clock()
        try:
            self.send_attempt_count += 1
            _ok(self._sdk.servo_j(self._target, 0, 1), "servo_j")
        except Exception as error:
            self._disable(f"servo_j 失败：{error}")
            return False
        self.sent_count += 1  # 命令已被 SDK 确认；哪怕随后发现耗时超限，也不能漏记。
        finished = self._clock()
        if finished - started > self._timing.max_servo_call_s:
            self._disable("servo_j 调用耗时超限")
            return False
        self._next_tick += SDK_SERVO_PERIOD_S
        return True

    def stop(self, reason: str = "人工停止") -> None:
        """显式停止；若控制器无响应，保留“未确认”状态供上层报警。"""
        self._disable(reason)
