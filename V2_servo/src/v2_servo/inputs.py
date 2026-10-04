"""遥操作/VLA 统一入口契约：时间、单位、来源、动作类型均显式。"""
from dataclasses import dataclass
import numpy as np
from .configuration import vector

@dataclass(frozen=True)
class JointIntent:
    q_rad: object
    source: str
    sequence: int
    received_monotonic: float
    source_time: float

class InputGate:
    def __init__(self,timeout=.08):
        self.timeout=timeout; self.source=None; self.seq=-1; self.stamp=None; self.release_seen=False
    def reset(self):
        self.source=None; self.seq=-1; self.stamp=None; self.release_seen=False
    def accept(self,intent,now,deadman,tracked=True):
        if not deadman:
            self.source=None; self.seq=-1; self.stamp=None; self.release_seen=True
            return None
        if not self.release_seen: raise ValueError('release/regrip required')
        if not tracked: self.reset(); raise ValueError('tracking lost; anchor invalidated')
        if not all(np.isfinite(t) for t in [now,intent.received_monotonic,intent.source_time]): raise ValueError('timestamp')
        age=now-intent.received_monotonic
        if not 0<=age<=self.timeout: self.reset(); raise ValueError('stale or future receive time')
        if self.source is not None and intent.source!=self.source: self.reset(); raise ValueError('source changed')
        if intent.sequence<=self.seq or (self.stamp is not None and intent.source_time<=self.stamp):
            self.reset(); raise ValueError('reordered/replayed input')
        # 远端时钟只做顺序检查，不直接拿 Unity uptime 与 Python monotonic 相减。
        # 真机还需要经测量的时钟偏移/网络延迟上界，当前未提供此放行证据。
        q=vector(intent.q_rad)
        self.source=intent.source; self.seq=intent.sequence; self.stamp=intent.source_time
        return q

def vla_joint_intent(action,metadata,sequence,received,source_time):
    if metadata.get('action_space')!='absolute_joint_position' or metadata.get('units')!='rad':
        raise ValueError('model action space must be explicit absolute joint radians')
    if metadata.get('joint_order')!=['J1','J2','J3','J4','J5','J6']: raise ValueError('joint order mismatch')
    return JointIntent(vector(action),metadata.get('model_sha256','offline_fixture'),sequence,received,source_time)
