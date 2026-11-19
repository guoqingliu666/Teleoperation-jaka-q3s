"""纯输入逻辑：时效、松握离合和相对位姿；不含逆解或机器人通信。"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, names):
    if isinstance(value, dict): value = [value[k] for k in names]
    a = np.asarray(value, dtype=float)
    if a.shape != (len(names),) or not np.isfinite(a).all(): raise ValueError('invalid vector')
    return a


@dataclass
class Target:
    position: np.ndarray
    rotation: np.ndarray
    sequence: int
    received: float


class Clutch:
    def __init__(self, basis, timeout=.08):
        self.basis = np.asarray(basis, dtype=float)
        if self.basis.shape != (3,3) or not np.allclose(self.basis.T @ self.basis, np.eye(3), atol=1e-8):
            raise ValueError('tracking basis must be orthogonal')
        self.timeout = timeout
        self.source = None; self.sequence = -1; self.source_time = -1.
        self.received = -math.inf; self.previous = None
        self.released = False; self.held = False; self.anchor = None
        self.target = None; self.reason = 'WAIT_RELEASE'; self.stopped = False
        self.minimum_clock_offset = None

    def invalidate(self, reason):
        self.released = False; self.held = False; self.anchor = None
        self.target = None; self.previous = None; self.reason = reason

    def expire(self, now):
        if now - self.received > self.timeout: self.invalidate('INPUT_STALE')

    def consume(self, packet, source, received, robot_position, robot_rotation):
        try:
            seq, stamp = packet['sequence'], packet['unity_time_s']
            if type(seq) is not int or isinstance(stamp, bool) or not math.isfinite(stamp):
                raise ValueError('missing source sequence/time')
            if self.source is not None and source != self.source: raise ValueError('source changed')
            if seq <= self.sequence or stamp <= self.source_time: raise ValueError('reordered/replayed packet')
            offset=received-stamp
            if self.minimum_clock_offset is not None and offset-self.minimum_clock_offset>self.timeout:
                raise ValueError('source stream delayed/backlogged')
            self.minimum_clock_offset=offset if self.minimum_clock_offset is None else min(offset,self.minimum_clock_offset)
            right = packet['right']
            if any(type(right.get(k)) is not bool for k in ('connected', 'tracked', 'pose_valid')):
                raise ValueError('invalid tracking flags')
            p = vector(right['position_m'], 'xyz')
            q = vector(right['rotation_xyzw'], 'xyzw')
            if abs(np.linalg.norm(q) - 1) > .01: raise ValueError('non-unit quaternion')
            r = Rotation.from_quat(q).as_matrix()
            grip = right['grip']
            if isinstance(grip, bool) or not math.isfinite(grip) or not 0 <= grip <= 1:
                raise ValueError('invalid grip')
            click = right.get('thumbstick_click', False)
            if type(click) is not bool: raise ValueError('invalid stop button')
            if self.previous is not None:
                pp, pr = self.previous
                if (stamp - self.source_time > self.timeout or received - self.received > self.timeout
                    or np.linalg.norm(p-pp) > .08 or Rotation.from_matrix(r @ pr.T).magnitude() > .5):
                    self.invalidate('INPUT_GAP_OR_JUMP')
            self.source = source; self.sequence = seq; self.source_time = stamp; self.received = received
            self.previous = (p, r)
            if click:
                self.stopped = True; self.invalidate('SOFTWARE_STOP'); return None
            if not all(right[k] for k in ('connected','tracked','pose_valid')):
                self.invalidate('TRACKING_LOST'); return None
            if grip < .3:
                self.invalidate('RELEASED'); self.released = True; self.stopped = False; return None
            if self.stopped or not self.released: return None
            if grip > .7 and not self.held:
                self.anchor = (p.copy(), r.copy(), vector(robot_position, 'xyz'), np.array(robot_rotation))
                self.held = True
            if not self.held: return None
            hp, hr, rp, rr = self.anchor
            target_p = rp + self.basis @ (p-hp)
            target_r = self.basis @ (r @ hr.T) @ self.basis.T @ rr
            self.target = Target(target_p, target_r, seq, received)
            self.reason = 'FOLLOWING'
            return self.target
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.invalidate('REJECTED: ' + str(exc))
            raise ValueError(self.reason) from exc
