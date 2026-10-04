"""复用现有Quest解码结果，不新开UDP端口，也不改变机械臂Grip语义。"""
from .control import TriggerSample


def trigger_sample_from_quest(payload, arm_intent):
    """仅在QuestVelocity.decode成功后调用；任何异常由主站锁存hand.stop。

    arm_intent已经负责来源锁定、递增序号、Grip重握和姿态跳变检测。
    stamp使用同一主机接收时钟；手主站tick必须使用相同时钟。
    """
    if payload.get('sequence') != arm_intent.get('sequence'):
        raise ValueError('Trigger与机械臂意图不是同一帧')
    right = payload['right']
    flags = [right[name] for name in ('connected', 'tracked', 'pose_valid')]
    if any(type(value) is not bool for value in flags):
        raise ValueError('Quest追踪字段必须为bool')
    sample = TriggerSample(value=right['trigger'], stamp=arm_intent['stamp'],
                           sequence=arm_intent['sequence'], deadman=arm_intent['deadman'],
                           tracking_valid=all(flags), software_stop=arm_intent.get('software_stop', False))
    sample.validate()
    return sample
