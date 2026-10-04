"""右Grip相对手柄位置；明确试验坐标，三方向演练，SDK所有权独立。"""
import numpy as np
from ..configuration import vector
from .quest import QuestInput
from .quest_j1 import QuestJ1Mapping,ready_input


class QuestXYZMapping(QuestJ1Mapping):
    def __init__(self,basis):super().__init__(basis);self.reference_position=None
    def decode(self,payload,address,stamp):
        frame=super().decode(payload,address,stamp)
        value=payload['right'].get('position_m')
        position=vector([value[k] for k in ('x','y','z')] if isinstance(value,dict) else value,3)
        if not frame['tracking_valid'] or not frame.get('deadman'):
            self.reference_position=None;delta=np.zeros(3)
        else:
            if self.reference_position is None:self.reference_position=position.copy()
            delta=position-self.reference_position
        mapped=self.basis@delta
        frame.update(input_mode='xyz',position_m=position.tolist(),
            reference_position_m=None if self.reference_position is None else self.reference_position.tolist(),
            hand_displacement_m=delta.tolist(),base_displacement_mm=(mapped*1000).tolist(),
            axes_xyz=np.clip(mapped/.02,-1,1).tolist(),axis_y=float(np.clip(mapped[2]/.02,-1,1)))
        return frame


class QuestXYZReceiver(QuestInput):
    input_mode='xyz'
    def __init__(self,port,basis,audit=None):
        super().__init__(port,basis,mapper=QuestXYZMapping(basis),audit=audit)


class QuestXYZRehearsal:
    """只有每轴正方向独立实测、释放完成才通过；无SDK，无运动。"""
    def __init__(self):self.stage='RELEASE_INITIAL';self.count=0;self.sequence=None;self.passed=False;self.axes_verified=[]
    def step(self,latest,now):
        if self.passed:return self.stage
        if latest is None or not 0<=now-latest['stamp']<=.08 or not latest.get('tracking_valid') or latest.get('stick_clicked'):
            self.count=0;return self.stage
        if latest['sequence']==self.sequence:return self.stage
        self.sequence=latest['sequence']
        if self.stage.startswith('GRIP_'):
            axis=self.stage[-1];index='XYZ'.index(axis);values=vector(latest.get('axes_xyz',[0,0,0]),3)
            valid=bool(latest.get('deadman') and values[index]>.3 and np.max(np.abs(np.delete(values,index)))<.3)
            required=3
        else:valid=ready_input(latest,now);required=10
        self.count=self.count+1 if valid else 0
        if self.count>=required:
            self.count=0
            stages=['RELEASE_INITIAL','GRIP_X','RELEASE_X','GRIP_Y','RELEASE_Y','GRIP_Z','RELEASE_FINAL']
            if self.stage.startswith('GRIP_'):self.axes_verified.append(self.stage[-1])
            index=stages.index(self.stage)
            if index==len(stages)-1:self.stage='XYZ_REHEARSAL_PASSED_NO_ROBOT_COMMANDS';self.passed=True
            else:self.stage=stages[index+1]
        return self.stage
