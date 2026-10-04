"""六通道归一化动作。0为校准张开端，1为校准闭合端，不代表真实角度。"""
import json
import math
from pathlib import Path

CHANNELS = ('大拇指侧摆', '大拇指弯曲', '食指弯曲', '中指弯曲', '无名指弯曲', '小拇指弯曲')
MOTOR_IDS = (1, 2, 3, 4, 5, 6)


def vector6(values, lower=0., upper=1.):
    values = tuple(values)
    if len(values) != 6 or any(type(v) not in (int, float) or not math.isfinite(v)
                              or not lower <= v <= upper for v in values):
        raise ValueError('需要六个有限且在范围内的通道值')
    return tuple(float(v) for v in values)


def unit(value):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError('Trigger必须是0到1的有限数值')
    return float(value)


class GraspLibrary:
    def __init__(self, path):
        self.path = Path(path)
        self.data = json.loads(self.path.read_text(encoding='utf-8'))
        if self.data.get('schema') != 1 or self.data.get('motor_ids') != list(MOTOR_IDS):
            raise ValueError('动作库版本或电机顺序错误')
        if self.data.get('channels') != list(CHANNELS):
            raise ValueError('动作库通道顺序错误')
        self.profiles = self.data['profiles']
        if not self.profiles:
            raise ValueError('动作库为空')
        for key, profile in self.profiles.items():
            for stage in ('open', 'pregrasp', 'closed'):
                vector6(profile[stage])
            if not isinstance(profile.get('name'), str) or not profile['name']:
                raise ValueError('动作名称不能为空')

    def target(self, name, trigger):
        p = self.profiles[name]
        t = unit(trigger)
        # 前25%先调整拇指对掌，再用剩余行程渐进闭合；松开时沿原路径反向展开。
        if t <= .25:
            a, b, blend = p['open'], p['pregrasp'], t / .25
        else:
            a, b, blend = p['pregrasp'], p['closed'], (t - .25) / .75
        return tuple(x + (y - x) * blend for x, y in zip(a, b))

    def save_custom(self, name, target):
        target = vector6(target)
        if not isinstance(name, str) or not name.strip():
            raise ValueError('请填写动作名称')
        self.profiles['custom'] = dict(name=name.strip(), description='用户编辑的模拟草案，需实机标定',
                                      open=[0.] * 6, pregrasp=[target[0], 0., 0., 0., 0., 0.],
                                      closed=list(target), validation='simulation_draft')
        # 同目录临时文件原子替换；路径由程序限定在D盘项目中。
        temporary = self.path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(self.path)
