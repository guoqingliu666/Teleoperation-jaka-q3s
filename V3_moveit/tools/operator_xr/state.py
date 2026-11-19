"""Read existing operator snapshots; expose ages, never browser/host clock offsets."""
import json
import math
import time
from pathlib import Path
from operator_target import current_target, finite_vector


def read(path):
    try:
        value = json.loads(Path(path).read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def age(data, key, now):
    stamp = data.get(key)
    if type(stamp) not in (int, float) or not math.isfinite(stamp):
        return math.inf
    value = now - stamp
    return value if value >= 0 else math.inf


def guidance(session, telemetry, floor, now):
    stage = session.get('stage')
    started = session.get('started')
    if (type(started) not in (int, float) or not math.isfinite(started) or not floor <= started <= now
            or session.get('mode') not in ('fake','hardware') or session.get('task') not in ('teleop','commission','prepare')):
        return dict(phase='idle')
    remaining = (10. if stage in ('ended','failed') else 1.5) - age(session, 'updated', now)
    if remaining <= 0:
        return dict(phase='idle' if stage in ('ended','failed') else 'unavailable')
    phase = stage
    instruction = ''
    if stage == 'running':
        same_run = telemetry.get('session_started') == started and telemetry.get('mode') == session['mode']
        live_for = .4 - age(telemetry, 'guide_updated', now) if same_run else 0.
        if live_for > 0:
            remaining = min(remaining, live_for)
            phase = telemetry.get('guide_phase','waiting_feedback')
            if session['task']=='commission' and phase in ('ready','paused','following','engaging','braking'):
                instruction = telemetry.get('check_instruction','')
        else:
            phase = 'waiting_feedback' if same_run else 'starting'
    tool = session.get('tool_id')
    return dict(phase=phase, mode=session['mode'], tool_id=tool if type(tool) is int and 0<=tool<=10 else None,
                ttl_s=remaining, instruction=instruction, task=session['task'])


def snapshot(cache, now=None):
    scene = read(cache / 'scene_state.json')
    session_state = read(cache / 'teleop_session.json')
    telemetry = read(cache / 'telemetry.json')
    media = read(cache / 'media/status.json')
    hands = [read(cache / name) for name in ('dh116_status.json', 'grasp_state.json')]
    data = read(cache / 'target_state.json')
    # Each writer can replace its file during a poll. Date the completed reads.
    now = time.monotonic() if now is None else now
    floor = scene.get('target_session_started', 0)
    if type(floor) not in (int, float) or not math.isfinite(floor):
        floor = now
    fresh = (age(telemetry, 'received_monotonic', now) < .5 and
             telemetry.get('received_monotonic', 0) >= floor and finite_vector(telemetry.get('position'), 6))
    joints = {'joint_' + str(i + 1): v for i, v in enumerate(telemetry['position'])} if fresh else {}
    # Same sources/priority as the native display, including direct hand feedback.
    devices = media.get('devices', {})
    monitor = devices.get('hand', {}) if isinstance(devices, dict) else {}
    hand_age = math.inf
    hand_ttl = .08
    candidates = [(monitor, 1.5)] if age(media, 'updated', now) < 1.5 else []
    candidates += [(hand, .08) for hand in hands]
    for hand, ttl in candidates:
        if not isinstance(hand, dict):
            continue
        names, values = hand.get('joint_names'), hand.get('position')
        if (hand.get('ok') is True and age(hand, 'updated', now) < ttl and
                isinstance(names, list) and names and finite_vector(values, len(names)) and
                all(isinstance(name, str) for name in names)):
            joints.update({'DH_' + name.removeprefix('DH_'): value for name, value in zip(names, values)})
            hand_age = age(hand, 'updated', now)
            hand_ttl = ttl
    # Malformed snapshots must not turn a display error into a retained target.
    try:
        target = current_target(data, now, floor)
    except (TypeError, ValueError):
        target = None
    trail = []
    session = data.get('session_started', -1)
    if (type(session) in (int, float) and session >= floor and data.get('frame') == 'base_link' and
            data.get('source') == '/v3/hand_target' and isinstance(data.get('trail'), list)):
        for point in data['trail'][-750:]:
            if (isinstance(point, dict) and finite_vector(point.get('position_m'), 3) and
                    type(point.get('segment')) is int and age(point, 'received_monotonic', now) < 5):
                trail.append(dict(position_m=point['position_m'], segment=point['segment'],
                                  age_s=age(point, 'received_monotonic', now)))
    return dict(schema=1, arm_fresh=bool(fresh), arm_age_s=age(telemetry, 'received_monotonic', now) if fresh else None,
                mode=telemetry.get('mode') if fresh else 'preview', joints=joints,
                hand_fresh=math.isfinite(hand_age), hand_age_s=hand_age if math.isfinite(hand_age) else None, hand_ttl_s=hand_ttl,
                target=dict(position_m=target['position_m'], quaternion_xyzw=target['quaternion_xyzw'],
                            age_s=age(target, 'received_monotonic', now)) if target else None,
                target_session=session if type(session) in (int, float) and math.isfinite(session) else None,
                trail=trail, trail_seconds=5, guidance=guidance(session_state,telemetry,floor,now))
