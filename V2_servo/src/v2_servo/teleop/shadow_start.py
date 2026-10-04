"""仅在无 SDK 的影子阶段等待首次握持；运动开始后绝不自动恢复追踪故障。"""
from .input_gate import StableReleasedInput


class ShadowGestureStart:
    def __init__(self):
        self.release = StableReleasedInput()
        self.armed = False
        self.started = False
        self.resets = 0

    def update(self, frame, now, error=''):
        if self.started:
            return True, 'SHADOW_GESTURE_ALREADY_STARTED'
        valid = (frame is not None and 0 <= now-frame['stamp'] <= .08
                 and frame.get('tracking_valid') and not error)
        if not valid:
            if self.armed: self.resets += 1
            self.armed = False
            self.release = StableReleasedInput()
            return False, 'WAIT_TRACKING_RELEASE_NO_SDK'
        if self.armed and frame.get('deadman'):
            self.started = True
            return True, 'FIRST_GRIP_STARTS_SHADOW_TIMER'
        if frame.get('grip', 1.) < .3:
            passed, state = self.release.update(frame, now, error)
            self.armed = self.armed or passed
            return False, 'WAIT_FIRST_GRIP_NO_SDK' if self.armed else state
        return False, 'RELEASE_GRIP_TO_ARM_NO_SDK'
