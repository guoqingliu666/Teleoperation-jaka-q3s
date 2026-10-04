from pathlib import Path
import json, math
import numpy as np

ROOT = Path(__file__).resolve().parents[2]

def vector(value, size=6):
    x = np.asarray(value, dtype=float)
    if x.shape != (size,) or not np.isfinite(x).all():
        raise ValueError(f'expected finite vector[{size}]')
    return x.copy()

def load(path=None):
    cfg = json.loads(Path(path or ROOT/'config/default.json').read_text(encoding='utf-8'))
    validate(cfg)
    return cfg

def validate(c):
    dt = c['period_s']
    if not math.isfinite(dt) or abs(dt - 0.008) > 1e-12:
        raise ValueError('this SDK profile requires 8 ms / step_num=1')
    f = c['filters']
    if type(f['enabled']) is not bool: raise ValueError('enabled must be boolean')
    if f.get('controller_lpf_policy','preserve') not in ('preserve','set_before_servo'):
        raise ValueError('controller_lpf_policy')
    for k in ['command_hz','cartesian_hz','velocity_hz','acceleration_hz','torque_hz','controller_lpf_hz']:
        if not math.isfinite(f[k]) or not 0 < f[k] < .5/dt: raise ValueError(k)
    if type(f['moving_average']) is not int or not 1 <= f['moving_average'] <= 31: raise ValueError('moving_average')
    if f['notch_hz'] is not None and not 0 < f['notch_hz'] < .5/dt: raise ValueError('notch_hz')
    if not math.isfinite(f['notch_q']) or f['notch_q'] <= 0: raise ValueError('notch_q')
    l = c['limits']
    lo,hi=vector(l['lower_rad']),vector(l['upper_rad'])
    if np.any(lo >= hi): raise ValueError('empty joint range')
    for k in ['velocity_rad_s','acceleration_rad_s2','jerk_rad_s3']:
        if np.any(vector(l[k]) <= 0): raise ValueError(k)
    if l['torque_nm'] is not None and np.any(vector(l['torque_nm']) <= 0): raise ValueError('torque_nm')
    for k in ['feedback_timeout_s','input_timeout_s','max_lateness_s','max_call_s','tracking_error_rad']:
        if not math.isfinite(c['safety'][k]) or c['safety'][k] <= 0: raise ValueError(k)
    return c
