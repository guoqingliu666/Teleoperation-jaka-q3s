"""有限姿态调试的自动结束门，不重判历史失败、不改变运动保护。"""
import numpy as np
from .quest_j1 import ready_input


class GestureCompletion:
    def __init__(self):self.since=None;self.q=[]

    def observe(self,now,intent,planner,q,dq,command_span,actual_span):
        moving_releases=sum(bool(e['moving_at_release']) for e in planner.release_events)
        eligible=(not planner.locked and ready_input(intent,now) and planner.quiet()
            and moving_releases>=2 and command_span>=.005 and actual_span>=.005
            and np.max(np.abs(dq))<=np.deg2rad(.1))
        if not eligible:self.since=None;self.q=[];return False
        if self.since is None:self.since=now
        self.q.append(np.array(q))
        if len(self.q)>100:self.q.pop(0)
        return len(self.q)>=10 and now-self.since>=.2 and np.max(np.ptp(self.q,axis=0))<=np.deg2rad(.01)
