"""重放原始Quest包，核对拒绝原因与原始源时钟下的手柄速度。"""
import argparse,collections,json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'.deps'),str(ROOT)];sys.dont_write_bytecode=True
import numpy as np
from v2_servo.teleop.continuous_pose import ContinuousMapping
from v2_servo.teleop.global_kinematics import rotation_log
from vendor.quest_vr_input import quaternion_matrix

def main():
    p=argparse.ArgumentParser();p.add_argument('--sessions',nargs='+',required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    out=a.out.resolve()
    if not out.is_relative_to(ROOT/'data') or out.exists():raise ValueError('Use new V2 data output')
    basis=np.array(json.loads((ROOT/'config/quest_xyz_commissioning.json').read_text())['basis']);reports=[]
    for name in a.sessions:
        source=(ROOT/'data'/name).resolve()
        if not source.is_relative_to(ROOT/'data'):raise ValueError('V2 data only')
        mapper=ContinuousMapping(basis);errors=collections.Counter();decoded=0;prev=None;speeds=[];angles=[]
        for line in (source/'events.jsonl').open(encoding='utf-8'):
            row=json.loads(line)
            if row.get('event')!='raw_quest_input':continue
            try:f=mapper.decode(row['raw'],tuple(row['peer']),row['host_receive_s']);decoded+=1
            except Exception as exc:errors[str(exc)]+=1;prev=None;continue
            if f.get('deadman') and f.get('tracking_valid'):
                pos=np.array(f['position_m']);rot=np.array(quaternion_matrix(f['rotation_xyzw']));t=f['source_time_s']
                if prev is not None and 0<t-prev[2]<=.08:
                    speeds.append(float(np.linalg.norm(pos-prev[0])/(t-prev[2])*1000))
                    angles.append(float(np.linalg.norm(rotation_log(rot@prev[1].T))/(t-prev[2])))
                prev=pos,rot,t
            else:prev=None
        stats=lambda v:{str(p):float(np.percentile(v,p)) for p in (50,95,99,100)} if v else {}
        report=dict(session=name,decoded=decoded,rejected=sum(errors.values()),reasons=dict(errors),
            hand_translation_speed_mm_s=stats(speeds),hand_rotation_speed_rad_s=stats(angles),
            hand_translation_above_120mm_s_fraction=float(np.mean(np.array(speeds)>120)) if speeds else None,
            half_gain_rotation_above_0_5rad_s_fraction=float(np.mean(np.array(angles)*.5>.5)) if angles else None,
            scope='offline_original_raw_packets_decoder_and_source_time_difference_no_interpolation')
        reports.append(report);print(json.dumps(report),flush=True)
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(reports,indent=2),encoding='utf-8')
if __name__=='__main__':main()
