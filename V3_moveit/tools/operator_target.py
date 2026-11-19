"""Read-only target samples and five-second trails in robot base coordinates."""
import math
from collections import deque

TTL = 5.0
FRESH = .15


def finite_vector(value, size):
    return (isinstance(value, (list, tuple)) and len(value) == size and
            all(type(v) in (int, float) and math.isfinite(v) for v in value))


class TargetHistory:
    def __init__(self, started):
        self.started = started
        self.points = deque(maxlen=750)
        self.target = None
        self.segment = 0
        self.held = False
        self.input_at = -math.inf
        self.reason = 'WAIT_RELEASE'
        self.last_stamp = -math.inf
        self.last_sample = -math.inf

    def inputs(self, data, now):
        held = (data.get('held') is True and type(data.get('age_s')) in (int, float)
                # The bridge timestamps its tick before receiving the packet,
                # so same-tick input age can be a few milliseconds negative.
                and -.01 <= data['age_s'] <= .08 and not data.get('software_stop'))
        if held and not self.held:
            self.segment += 1
            self.last_sample = -math.inf
            self.target = None
        self.held = held
        self.reason = str(data.get('reason', '等待输入'))
        self.input_at = now

    def pose(self, position, quaternion, frame, source_stamp, source_age, now):
        if not (frame == 'base_link' and finite_vector(position, 3) and
                finite_vector(quaternion, 4) and abs(sum(v*v for v in quaternion)-1) < .02 and
                math.isfinite(source_stamp) and source_stamp > self.last_stamp and
                0 <= source_age <= .08 and self.held and 0 <= now-self.input_at <= .08):
            return False
        if self.target is not None and now-self.target['received_monotonic'] > FRESH:
            self.segment += 1
            self.last_sample = -math.inf
        self.last_stamp = source_stamp
        self.target = dict(position_m=list(position), quaternion_xyzw=list(quaternion),
                           received_monotonic=now, source_stamp=source_stamp)
        if now-self.last_sample >= 1/60:
            self.points.append(dict(position_m=list(position), received_monotonic=now, segment=self.segment))
            self.last_sample = now
        return True

    def snapshot(self, now):
        while self.points and now-self.points[0]['received_monotonic'] >= TTL:
            self.points.popleft()
        active = (self.held and 0 <= now-self.input_at <= .08 and self.target is not None and
                  0 <= now-self.target['received_monotonic'] < FRESH)
        return dict(schema=1, frame='base_link', source='/v3/hand_target', updated=now,
                    session_started=self.started, active=active, reason=self.reason,
                    target=self.target if active else None, trail=list(self.points), trail_seconds=TTL)


def current_target(data, now, session_started=0):
    """Reject stale/old-session/malformed display data; never retain old axes."""
    if not (isinstance(data, dict) and data.get('frame') == 'base_link' and
            data.get('source') == '/v3/hand_target' and data.get('session_started', -1) >= session_started and
            0 <= now-data.get('updated', -math.inf) < FRESH and data.get('active') is True):
        return None
    target = data.get('target')
    if not isinstance(target, dict):
        return None
    q = target.get('quaternion_xyzw')
    if (not finite_vector(target.get('position_m'), 3) or not finite_vector(q, 4) or
            abs(sum(v*v for v in q)-1) >= .02 or
            not 0 <= now-target.get('received_monotonic', -math.inf) < FRESH):
        return None
    return target
