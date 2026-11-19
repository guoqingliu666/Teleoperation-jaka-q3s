"""DH116 presets and analogue trigger policy; no transport or hardware calls."""
import copy
import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STORE = ROOT / '.config/grasp_profiles.json'
AXES = ('拇指侧摆', '拇指弯曲', '食指', '中指', '无名指', '小指')
FINGERS = ('拇指', '食指', '中指', '无名指', '小指')
AXIS_FINGER = (0, 0, 1, 2, 3, 4)
MAX_SPEED = 20000  # DH116 SDK position velocity, counts/s


def action_settings(profile):
    """Old presets retain their original velocity, endpoints and simultaneous timing."""
    duration = profile.get('duration_ms', 1000)
    tracks = profile.get('tracks')
    if tracks is None:
        tracks = [dict(start_ms=0, end_ms=duration, force_n=profile['normal_force_limit_n']) for _ in FINGERS]
    percent = profile['velocity'] * 100 / MAX_SPEED
    return dict(duration_ms=duration, tracks=copy.deepcopy(tracks),
                close_percent=profile.get('close_percent', percent),
                open_percent=profile.get('open_percent', percent))


def action_words(profile):
    action = action_settings(profile)
    speeds = [round(action[k] * MAX_SPEED / 100) for k in ('close_percent', 'open_percent')]
    # An old per-axis speed equal to the old master speed follows the new master.
    # Retain any deliberately lower per-axis limit.
    manual = [max(speeds) if v == profile['velocity'] else v
              for v in profile.get('manual_velocity', [profile['velocity']] * 6)]
    words = [action['duration_ms'], *speeds, profile['current_per_mille'], profile['normal_force_limit_n'],
             *profile['open'], *profile['closed'], *manual,
             *profile.get('manual_current', [profile['current_per_mille']] * 6)]
    for track in action['tracks']:
        words.extend([track['start_ms'], track['end_ms'], track['force_n']])
    return ' '.join(map(str, words))


def defaults():
    return {'version': 1, 'active': 0, 'profiles': [
        {'name': name, 'style': style, 'open': [0] * 6, 'closed': closed,
         'velocity': 2000, 'current_per_mille': 200, 'normal_force_limit_n': 3.0,
         'force_limit_enabled': True}
        for name, style, closed in (
            ('方案一', '包覆', [1500] * 6),
            ('方案二', '两指捏', [1500, 1500, 1500, 0, 0, 0]),
            ('方案三', '三指捏', [1500, 1500, 1500, 1500, 0, 0]))]}


def validate(data):
    if not isinstance(data, dict):
        raise ValueError('抓握配置应为对象')
    if data.get('version') != 1 or type(data.get('active')) is not int or not 0 <= data['active'] < 3:
        raise ValueError('无效的抓握方案选择')
    if not isinstance(data.get('profiles'), list) or len(data['profiles']) != 3:
        raise ValueError('需要三套抓握方案')
    for p in data['profiles']:
        if not isinstance(p, dict):
            raise ValueError('方案应为对象')
        if not isinstance(p.get('name'), str) or not 1 <= len(p['name']) <= 24:
            raise ValueError('方案名称长度应为 1～24')
        for key in ('open', 'closed'):
            if not isinstance(p.get(key), list) or len(p[key]) != 6 or any(type(x) is not int or not 0 <= x <= 10000 for x in p[key]):
                raise ValueError('六轴位置必须在 0～10000 当量内')
        if any(a>b for a,b in zip(p['open'],p['closed'])):
            raise ValueError('最小位置不能大于最大位置')
        for key, low, high in [('velocity', 100, 20000), ('current_per_mille', 1, 1000)]:
            if type(p.get(key)) is not int or not low <= p[key] <= high:
                raise ValueError('速度或电流上限超出厂家范围')
        for key,low,high in [('manual_velocity',100,MAX_SPEED),('manual_current',1,900)]:
            if key in p and (not isinstance(p[key],list) or len(p[key])!=6 or any(type(x) is not int or not low<=x<=high for x in p[key])):
                raise ValueError('手动六轴参数超出当前授权范围')
        force = p.get('normal_force_limit_n')
        if type(force) not in (int, float) or not math.isfinite(force) or not 0.1 <= force <= 20:
            raise ValueError('单指法向力上限应为 0.1～20 N')
        if type(p.get('force_limit_enabled')) is not bool:
            raise ValueError('力反馈限制开关无效')
        action = action_settings(p)
        duration = action['duration_ms']
        if type(duration) is not int or not 100 <= duration <= 30000:
            raise ValueError('动作时长应为 0.1～30 秒')
        for key in ('close_percent','open_percent'):
            value=action[key]
            if type(value) not in (int,float) or not .5 <= value <= 100:
                raise ValueError('开合速度应为 0.5%～100%')
        if not isinstance(action['tracks'],list) or len(action['tracks'])!=5:
            raise ValueError('动作需要五条手指轨道')
        for track in action['tracks']:
            if (not isinstance(track,dict) or any(type(track.get(k)) is not int for k in ('start_ms','end_ms'))
                    or not 0 <= track['start_ms'] < track['end_ms'] <= duration):
                raise ValueError('每根手指的开始时间必须早于结束时间，且在动作时长内')
            value=track.get('force_n')
            if type(value) not in (int,float) or not .1 <= value <= force:
                raise ValueError('单指接触阈值应在 0.1 N 和方案保护上限之间')
    return data


class Presets:
    def __init__(self, path=STORE):
        self.path = Path(path)
        self.error = None
        try:
            self.data = validate(json.loads(self.path.read_text()))
        except FileNotFoundError:
            self.data = defaults()
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.error = str(exc)
            self.data = defaults()

    @property
    def profile(self):
        return copy.deepcopy(self.data['profiles'][self.data['active']])

    def save(self):
        validate(self.data)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.data, ensure_ascii=False, indent=2) + '\n')
        os.replace(tmp, self.path)


def targets(profile, trigger):
    if type(trigger) not in (int, float) or not math.isfinite(trigger) or not 0 <= trigger <= 1:
        raise ValueError('Trigger 必须是 0～1 的有效数值')
    action=action_settings(profile)
    t=trigger*action['duration_ms']
    progress=[max(0.,min(1.,(t-r['start_ms'])/(r['end_ms']-r['start_ms']))) for r in action['tracks']]
    return [math.floor(progress[finger]*end+.5) for finger,end in zip(AXIS_FINGER,profile['closed'])]


class TriggerInput:
    """Reject replay/invalid packets; a released trigger is required after reset."""
    def __init__(self):
        self.sequence = -1
        self.client_time = None
        self.received = None
        self.trigger = 0.0
        self.released = False
        self.stopped = False
        self.offset = None

    def consume(self, packet, now):
        try:
            if not isinstance(packet, dict) or not isinstance(packet.get('right'), dict):
                raise ValueError('手柄包格式无效')
            r = packet['right']
            seq, stamp = packet['sequence'], packet['unity_time_s']
            if packet.get('source') != 'quest-webxr' or type(seq) is not int or seq <= self.sequence:
                raise ValueError('手柄序号无效或重复')
            if type(stamp) not in (int, float) or not math.isfinite(stamp):
                raise ValueError('手柄时间无效')
            if not math.isfinite(now):
                raise ValueError('本机时间无效')
            if self.received is not None and now - self.received >= .08:
                self.released = False
            offset = now - stamp
            if self.offset is not None and offset - self.offset > .08:
                raise ValueError('手柄累计延迟超过门槛')
            self.offset = offset if self.offset is None else min(offset, self.offset)
            if self.client_time is not None and (stamp <= self.client_time or now - self.received - (stamp - self.client_time) > .08):
                raise ValueError('手柄数据延迟或时间倒退')
            self.sequence, self.client_time, self.received = seq, stamp, now
            if r.get('thumbstick_click') is True:
                self.stopped = True
            if any(r.get(key) is not True for key in ('connected', 'tracked', 'pose_valid')):
                raise ValueError('手柄跟踪不可用')
            trigger = r['trigger']
            if type(trigger) not in (float,int) or not math.isfinite(trigger) or not 0<=trigger<=1:
                raise ValueError('Trigger 必须在 0～1 范围内')
            self.trigger = float(trigger)
            if trigger < .05:
                self.released = True
            return self.valid(now)
        except (ValueError, KeyError, TypeError):
            self.released = False
            raise

    def valid(self, now):
        fresh = self.received is not None and 0 <= now - self.received < .08
        if not fresh:
            self.released = False
        return fresh and self.released and not self.stopped
