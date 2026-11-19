"""Explicit simulation tuning; no hardware collision override."""
import copy
import math


def servo_config(config, mode, fake_self_threshold=''):
    result = copy.deepcopy(config)
    if mode not in ('fake', 'hardware'):
        raise ValueError('mode must be fake or hardware')
    if not result.get('check_collisions'):
        raise ValueError('collision checking must remain enabled')
    if fake_self_threshold != '':
        if mode != 'fake':
            raise ValueError('fake_self_collision_proximity_threshold requires mode=fake')
        threshold = float(fake_self_threshold)
        if not math.isfinite(threshold) or threshold <= 0:
            raise ValueError('fake collision proximity threshold must be positive and finite')
        result['self_collision_proximity_threshold'] = threshold
    return result


def tracking_config(config, mode, fake_override=False):
    if mode not in ('fake', 'hardware'):
        raise ValueError('mode must be fake or hardware')
    value = config.get('velocity_tracking', False)
    if type(value) is not bool or type(fake_override) is not bool:
        raise ValueError('velocity_tracking must be a boolean')
    if fake_override and mode != 'fake':
        raise ValueError('fake_velocity_tracking requires mode=fake')
    return value or fake_override
