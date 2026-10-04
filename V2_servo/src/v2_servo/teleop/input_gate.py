"""连接机器人前的只读输入门。没有SDK，不延长运动中的80ms看门狗。"""
from .quest_j1 import ready_input,input_readiness


class StableReleasedInput:
    def __init__(self):
        self.count=0;self.first=None;self.sequence=None

    def update(self,frame,now,error=''):
        # 连续新鲜且释放的包至少10个、持续至少0.2秒；重复旧包不累计。
        if not ready_input(frame,now):
            self.count=0;self.first=None;self.sequence=None
            return False,input_readiness(frame,now,error)
        if frame['sequence']!=self.sequence:
            if self.first is None:self.first=now
            self.count+=1;self.sequence=frame['sequence']
        passed=self.count>=10 and now-self.first>=.2
        return passed,'QUEST_INPUT_READY_NO_ROBOT_COMMANDS' if passed else 'WAIT_STABLE_RELEASED_INPUT'
