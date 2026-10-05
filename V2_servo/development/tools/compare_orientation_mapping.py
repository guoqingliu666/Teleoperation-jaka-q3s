"""同一多轴闭合手势测量目标映射漂移；不是硬件轨迹试验。"""
import json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'.deps'),str(ROOT)];sys.dont_write_bytecode=True
import numpy as np
from v2_servo.kinematics import rotate
from v2_servo.teleop.global_kinematics import rotation_log
from v2_servo.teleop.continuous_pose import anchored_orientation

def main():
    B=np.array([[0,0,1],[-1,0,0],[0,1,0]]);I=np.eye(3);previous=I.copy();old=I.copy()
    rotations=[I,rotate([1,0,0],np.pi/2),rotate([0,1,0],np.pi/2)@rotate([1,0,0],np.pi/2),rotate([0,1,0],np.pi/2),I]
    rows=[]
    for destination in rotations[1:]:
        start=previous.copy();rv=rotation_log(destination@start.T)
        for i in range(1,101):
            hand=rotate(rv/np.linalg.norm(rv),np.linalg.norm(rv)*i/100)@start
            inc=rotation_log(B@(hand@previous.T)@B.T)*.5
            if np.linalg.norm(inc)>1e-12:old=rotate(inc/np.linalg.norm(inc),np.linalg.norm(inc))@old
            new=anchored_orientation(B,hand,I,I);previous=hand
            rows.append(dict(hand=hand.tolist(),frozen_goal=old.tolist(),one_to_one_goal=new.tolist()))
    report=dict(scope='offline_orientation_mapping_only_same_closed_hand_path',samples=len(rows),robot_commands=0,
        frozen_closed_path_goal_drift_deg=float(np.rad2deg(np.linalg.norm(rotation_log(old)))),
        anchored_closed_path_goal_drift_deg=float(np.rad2deg(np.linalg.norm(rotation_log(new)))),
        control_dynamics_unchanged=True,warning='one-to-one target demands larger angular motion than half-gain; not a robot accuracy claim')
    out=ROOT/'data/tracking_review_20261004';out.mkdir(exist_ok=True)
    (out/'orientation_closed_path.json').write_text(json.dumps(dict(report=report,samples=rows),indent=2),encoding='utf-8')
    print(json.dumps(report))
if __name__=='__main__':main()
