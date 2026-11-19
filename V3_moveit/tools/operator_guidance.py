"""Read-only interpretation of controller and Quest feedback for the operator."""
import math


def live_phase(controller, inputs, now, state_at, input_at, joints_at, followed):
    if not state_at or not 0 <= now - state_at < .5:
        return 'waiting_feedback'
    mode = controller.get('mode')
    if mode == 'FAULT':
        return 'fault'
    if mode == 'BOOT':
        return 'starting'
    if not joints_at or not 0 <= now - joints_at < .5 or controller.get('scene_ready') is not True:
        return 'waiting_feedback'
    if inputs.get('software_stop') is True:
        return 'software_stop'
    packet_age = inputs.get('age_s')
    if (type(packet_age) not in (int, float) or not math.isfinite(packet_age)
            or not 0 <= now - input_at < .08 or max(0., packet_age) + now - input_at >= .08):
        return 'tracking'
    reason = inputs.get('reason')
    if reason == 'WAIT_RELEASE':
        return 'release'
    if reason not in ('RELEASED', 'FOLLOWING'):
        return 'tracking'
    if mode in ('BRAKING', 'CANCELING'):
        return 'braking'
    if mode == 'IDLE' and reason == 'RELEASED' and inputs.get('held') is False:
        return 'paused' if followed else 'ready'
    if mode == 'FOLLOW' and controller.get('held') is True and inputs.get('held') is True and reason == 'FOLLOWING':
        return 'following'
    return 'engaging' if inputs.get('held') is True else 'braking'
