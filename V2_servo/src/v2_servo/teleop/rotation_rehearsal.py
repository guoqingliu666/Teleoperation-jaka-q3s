"""逐轴只读姿态验收：真实新帧、握持旋转、完全释放、候选停稳。"""
import numpy as np


class RotationRehearsal:
    def __init__(self):
        self.index=0;self.phase='ROTATE';self.count=0;self.sequence=None
        self.passed=False;self.completed=[]

    @property
    def axis(self):return ('RX','RY','RZ')[min(self.index,2)]

    def step(self,frame,now,planner):
        if self.passed:return
        if frame is None or not 0<=now-frame['stamp']<=.08 or not frame.get('tracking_valid'):
            self.count=0;return
        if frame['sequence']==self.sequence:return
        self.sequence=frame['sequence']
        if self.phase=='ROTATE':
            good=bool(frame.get('deadman') and planner.gripped and abs(frame['rotation_delta_deg'][self.index])>1.)
            self.count=self.count+1 if good else 0
            if self.count>=3:self.phase='RELEASE';self.count=0
        else:
            # 这里验证的是候选轨迹静止，绝不声称真机已停止。
            quiet=np.max(np.abs(planner.state.v))<1e-8 and np.max(np.abs(planner.state.a))<1e-7
            good=frame.get('grip',1)<.3 and not frame.get('deadman') and quiet
            self.count=self.count+1 if good else 0
            if self.count>=5:
                self.completed.append(self.axis);self.index+=1;self.count=0
                if self.index==3:self.passed=True
                else:self.phase='ROTATE'

    def instruction(self):
        if self.passed:return 'RX/RY/RZ旋转与完全释放已记录；仅影子通过，机器人运动命令0。'
        if self.phase=='RELEASE':return f'{self.axis}已读回：完全松开右Grip，等待候选停稳；不要继续转。'
        return f'第{self.index+1}/3项 {self.axis}：握右Grip，缓慢转手柄3–5度；看该轴数值超过1度。不要转超过30度。'
