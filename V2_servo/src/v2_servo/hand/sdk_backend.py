"""接入已经验收的PyLHandProLib/主站；此模块不接网卡、不上电、不回零。"""
import math
import time
from .profiles import MOTOR_IDS, vector6


def raw_six(values, positive=False):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError('SDK参数需要六通道整数')
    if any(type(v) is not int or not -(2**31) <= v < 2**31 or (positive and v <= 0) for v in values):
        raise ValueError('SDK参数须为有效整数；速度和电流上限须为正')
    return tuple(values)


class SdkHandBackend:
    is_sim = False

    def __init__(self, sdk, config, receive_info, authorize, clock=time.monotonic):
        # 接收元数据由同一个总线工作线程提供：本机收帧时间、递增序号及WKC有效性。
        # 多线程主站必须用自己的锁保证这些元数据与SDK缓存的一致快照。
        self.sdk, self.config = sdk, config
        self.receive_info, self.authorize, self.clock = receive_info, authorize, clock
        self.stop_errors = []
        self.last_meta = None
        self._validate_config()

    def _validate_config(self):
        c = self.config
        if c.get('hardware_model') != 'DH116EC-R1-M' or c.get('motor_ids') != list(MOTOR_IDS):
            raise RuntimeError('手型号或SDK电机编号未经确认')
        for key in ('motion_approved', 'calibration_validated', 'transport_validated', 'vendor_return_contract_validated'):
            if c.get(key) is not True:
                raise RuntimeError('实机门槛未完成：' + key)
        for key in ('motion_evidence', 'calibration_evidence', 'transport_evidence',
                    'current_unit_evidence', 'position_unit_evidence', 'velocity_unit_evidence'):
            if not isinstance(c.get(key), str) or not c[key].strip():
                raise RuntimeError('缺少证据：' + key)
        if not c.get('current_unit') or not c.get('position_unit') or not c.get('velocity_unit'):
            raise RuntimeError('SDK原始单位未确认')
        self.open_raw, self.closed_raw = raw_six(c['open_raw']), raw_six(c['closed_raw'])
        self.lower, self.upper = raw_six(c['position_lower_raw']), raw_six(c['position_upper_raw'])
        self.velocity, self.current = raw_six(c['velocity_limit_raw'], True), raw_six(c['current_limit_raw'], True)
        for a, b, lo, hi in zip(self.open_raw, self.closed_raw, self.lower, self.upper):
            if a == b or not lo <= a <= hi or not lo <= b <= hi:
                raise ValueError('张开/闭合标定端点和位置边界不一致')
        age = c.get('feedback_timeout_s')
        if type(age) not in (int, float) or not math.isfinite(age) or not 0 < age <= .15:
            raise ValueError('实机反馈超时须为(0,0.15]秒')
        if not isinstance(c.get('approved_profiles'), list) or not c['approved_profiles']:
            raise RuntimeError('尚未逐项验收抓握动作')

    def snapshot(self, now):
        meta = dict(self.receive_info())
        stamp, seq = meta.get('stamp'), meta.get('sequence')
        if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                or type(seq) is not int or seq < 0 or meta.get('wkc_ok') is not True
                or not 0 <= now-stamp <= self.config['feedback_timeout_s']):
            raise RuntimeError('缺少本次新鲜有效PDO反馈')
        if self.last_meta is not None:
            old_seq, old_stamp = self.last_meta
            if seq < old_seq or stamp < old_stamp or (seq == old_seq and stamp != old_stamp):
                raise RuntimeError('PDO序号倒退或未收帧却更新时间')
        self.last_meta = (seq, stamp)
        raw = tuple(self.sdk.get_now_position(i) for i in MOTOR_IDS)
        raw_six(raw)
        # 不裁剪异常反馈，否则会把越界位置伪装成安全端点。
        q = vector6((r-a)/(b-a) for r, a, b in zip(raw, self.open_raw, self.closed_raw))
        current = tuple(self.sdk.get_now_current(i) for i in MOTOR_IDS)
        raw_six(current)
        if any(abs(v) > lim for v, lim in zip(current, self.current)):
            raise RuntimeError('实测电流超过已验收限额')
        return dict(position=q, position_raw=raw, current_raw=current,
                    current_unit=self.config['current_unit'], stamp=stamp, sequence=seq, wkc_ok=True,
                    enabled=[self.sdk.get_enable(i) for i in MOTOR_IDS],
                    alarm=[self.sdk.get_now_alarm(i) for i in MOTOR_IDS],
                    status=[self.sdk.get_now_status(i) for i in MOTOR_IDS])

    def _safe(self):
        self._validate_config()  # 运行中撤销门槛也必须生效。
        self.authorize('hand_command')
        f = self.snapshot(self.clock())
        if any(v is not True for v in f['enabled']) or any(type(v) is not int or v != 0 for v in f['alarm']):
            raise RuntimeError('SDK关节未使能或有报警')
        if any(type(v) is not int or v not in (0, 1) for v in f['status']):
            raise RuntimeError('SDK关节状态阻止运动')

    def command(self, target, now, profile):
        target = vector6(target)
        self._validate_config()
        if profile not in self.config['approved_profiles']:
            raise RuntimeError('该抓握动作尚未通过实机验收')
        raw = tuple(round(a + (b-a)*q) for a, b, q in zip(self.open_raw, self.closed_raw, target))
        if any(not lo <= r <= hi for r, lo, hi in zip(raw, self.lower, self.upper)):
            raise ValueError('抓握目标超出SDK原始位置限额')
        try:
            for channel, motor in enumerate(MOTOR_IDS):
                # 每一次setter和启动前重新检查授权、反馈和报警；不使用广播ID=0。
                for name, args in (('set_control_mode', (motor, 4)),
                                   ('set_position_velocity', (motor, self.velocity[channel])),
                                   ('set_max_current', (motor, self.current[channel])),
                                   ('set_target_position', (motor, raw[channel]))):
                    self._safe()
                    if getattr(self.sdk, name)(*args) is not None:
                        raise RuntimeError('SDK包装返回契约不符：' + name)
            for motor in MOTOR_IDS:
                self._safe()
                if self.sdk.move_motors(motor) is not None:
                    raise RuntimeError('SDK包装返回契约不符：move_motors')
        except Exception:
            self.stop()
            raise

    def stop(self):
        self.stop_errors = []
        for motor in MOTOR_IDS:
            try:
                result = self.sdk.stop_motors(motor)
                if result is not None:
                    self.stop_errors.append((motor, 'SDK停止返回契约不符'))
            except Exception as exc:
                self.stop_errors.append((motor, repr(exc)))
        # 记录失败供主站上报；发出stop不等于物理停止，主站/设备watchdog必须独立验收。
