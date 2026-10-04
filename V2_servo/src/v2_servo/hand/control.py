"""Trigger动作状态机；SDK阻塞和总线watchdog仍须由独立主站进程保障。"""
import math
from dataclasses import dataclass
from .profiles import vector6, unit


@dataclass(frozen=True)
class TriggerSample:
    value: float
    stamp: float             # 主站进程收到消息时的monotonic时间，不接收远端时钟。
    sequence: int
    deadman: bool
    tracking_valid: bool = True
    software_stop: bool = False

    def validate(self):
        unit(self.value)
        if type(self.stamp) not in (float, int) or not math.isfinite(self.stamp):
            raise ValueError('输入时间无效')
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError('输入序号无效')
        if any(type(v) is not bool for v in (self.deadman, self.tracking_valid, self.software_stop)):
            raise ValueError('使能、追踪和停止字段必须是bool')


class SimHandBackend:
    is_sim = True

    def __init__(self):
        self.q = (0.,) * 6
        self.sequence = 0
        self.stops = 0
        self.commands = []

    def snapshot(self, now):
        self.sequence += 1
        return dict(position=self.q, stamp=now, sequence=self.sequence, enabled=[True]*6,
                    alarm=[0]*6, status=[0]*6, current_raw=[0]*6, current_unit='SIM', wkc_ok=True)

    def command(self, target, now, profile):
        self.q = vector6(target)
        self.commands.append((now, self.q, profile))

    def stop(self):
        self.stops += 1


class HandController:
    def __init__(self, library, backend, profile='cylinder', slew_per_s=.35,
                 input_timeout_s=.25, feedback_timeout_s=.15):
        if profile not in library.profiles:
            raise ValueError('未知动作')
        for v in (slew_per_s, input_timeout_s, feedback_timeout_s):
            if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
                raise ValueError('控制参数须为正的有限数')
        self.library, self.backend, self.profile = library, backend, profile
        self.slew = slew_per_s
        self.input_timeout, self.feedback_timeout = input_timeout_s, feedback_timeout_s
        self.sample = None
        self.sequence = -1
        self.stamp = -math.inf
        self.q = None
        self.last_tick = None
        self.released = False
        self.fault = None
        self.state = '等待松开Trigger'
        self.manual_target = None
        self.last_feedback = None
        self.stop_errors = []

    def submit(self, sample):
        try:
            sample.validate()
            if sample.sequence <= self.sequence or sample.stamp < self.stamp:
                raise ValueError('重复、乱序或倒退的输入')
        except Exception:
            self.stop('输入无效')
            raise
        self.sequence, self.stamp, self.sample = sample.sequence, sample.stamp, sample

    def stop(self, reason='操作者停止'):
        self.fault = reason
        self.state = '停止：' + reason
        self.released = False
        self.manual_target = None
        self.stop_errors = []
        try:
            self.backend.stop()  # 停止运动，不自动张开，避免掉落物体。
            self.stop_errors = list(getattr(self.backend, 'stop_errors', []))
        except Exception as exc:
            self.stop_errors = [repr(exc)]
        if self.stop_errors:
            self.state += '；停止请求失败，须检查主站/设备停止状态'

    def reset(self, now):
        s = self.sample
        if (s is None or not 0 <= now-s.stamp <= self.input_timeout or s.value > .05
                or s.deadman or not s.tracking_valid or s.software_stop):
            raise RuntimeError('恢复前须松开Trigger和Grip，并收到新鲜有效输入')
        feedback = self.backend.snapshot(now)
        self._check_feedback(feedback, now)
        self.q = vector6(feedback['position'])
        self.fault = None
        self.released = True
        self.state = '就绪'
        self.last_tick = now

    def _check_feedback(self, f, now):
        vector6(f['position'])
        if type(f['stamp']) not in (int, float) or not math.isfinite(f['stamp']):
            raise RuntimeError('反馈时间无效')
        if not 0 <= now-f['stamp'] <= self.feedback_timeout or f.get('wkc_ok') is not True:
            raise RuntimeError('反馈陈旧或WKC无效')
        for key in ('enabled', 'alarm', 'status'):
            if len(f[key]) != 6:
                raise RuntimeError('反馈通道数错误')
        if any(x is not True for x in f['enabled']) or any(type(x) is not int or x != 0 for x in f['alarm']):
            raise RuntimeError('关节未使能或有报警')
        if any(type(x) is not int or x not in (0, 1) for x in f['status']):
            raise RuntimeError('关节限位、急停或回零状态')

    def select(self, name):
        if name not in self.library.profiles:
            raise ValueError('未知动作')
        if self.sample is not None and (self.sample.value > .05 or self.sample.deadman):
            raise RuntimeError('切换动作前请松开Trigger和Grip')
        if self.last_feedback and any(x != 0 for x in self.last_feedback['status']):
            raise RuntimeError('切换动作前请等待关节停止')
        self.profile, self.manual_target = name, None

    def set_manual(self, target):
        if not self.backend.is_sim:
            raise RuntimeError('关节自由编辑仅在模拟调试中启用')
        self.select(self.profile)
        self.manual_target = vector6(target)

    def tick(self, now):
        if type(now) not in (int, float) or not math.isfinite(now):
            self.stop('控制时间无效')
            return self.state
        if self.fault:
            return self.state
        try:
            f = self.backend.snapshot(now)
            self._check_feedback(f, now)
            self.last_feedback = f
            if self.q is None:
                self.q = vector6(f['position'])  # 从实测当前位置启动，禁止默认零位跳转。
            dt = 0. if self.last_tick is None else now-self.last_tick
            if not 0 <= dt <= .15:
                raise RuntimeError('控制周期超时或时间倒退')
            self.last_tick = now
            s = self.sample
            if s is None:
                self.state = '等待Trigger输入'
                return self.state
            if not 0 <= now-s.stamp <= self.input_timeout:
                raise RuntimeError('Trigger输入超时')
            if s.software_stop or not s.tracking_valid:
                raise RuntimeError('软件停止或追踪无效')
            if s.value <= .05:
                self.released = True
            if not s.deadman:
                if s.value > .05:
                    self.released = False
                if self.state == '运动':
                    self.backend.stop()
                    self.q = vector6(f['position'])
                self.state = '保持当前位置：握住Grip才能开合'
                return self.state
            if not self.released:
                self.state = '等待松开Trigger'
                return self.state
            target = self.manual_target if self.manual_target is not None else self.library.target(self.profile, s.value)
            step = self.slew * dt
            q = tuple(x + max(-step, min(step, y-x)) for x, y in zip(self.q, target))
            if any(abs(x-y) > 1e-8 for x, y in zip(q, self.q)):
                self.backend.command(q, now, self.profile)
                self.q, self.state = q, '运动'
            else:
                self.state = '保持'
        except Exception as exc:
            self.stop(str(exc))
        return self.state
