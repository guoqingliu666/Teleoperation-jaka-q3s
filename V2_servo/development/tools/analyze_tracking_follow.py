"""相同已记录输入+可见停止状态离线重放；不访问SDK，保留失败与不一致。

旧日志未逐次记录外部输入门的stop请求，故显式使用记录的LOCKED_STOP状态，
报告此重建条件及关节差异，不把重放算成新的真机证据。
"""
import argparse,csv,hashlib,json,os,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'.deps'),str(ROOT)];sys.dont_write_bytecode=True
for n in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[n]='1'
import numpy as np
from v2_servo.configuration import load
from v2_servo.teleop.continuous_pose import ContinuousPoseServo
from v2_servo.teleop.continuous_shadow import load_model
from v2_servo.teleop.tracking_observation import shadow_tracking

def stats(values):
    return {str(p):float(np.percentile(values,p)) for p in (50,95,99,100)} if values else {}

def replay(source,out,orientation='frozen'):
    saved=json.loads((source/'report.json').read_text(encoding='utf-8'))
    model,q,modelpath=load_model();basis=np.array(json.loads((ROOT/'config/quest_xyz_commissioning.json').read_text())['basis'])
    profile=dict(saved['profile'])
    if orientation=='one-to-one':profile.update(orientation_mapping='anchored_one_to_one',rotation_gain=1.)
    c=ContinuousPoseServo(q,load(),model,profile,basis);q=np.array(q)
    errors=[];angles=[];scales=[];sigmas=[];requested=[];filtered=[];actual=[];diff=0.;rows=0;failure=None;external=0
    trace=out/(source.name+'_'+orientation+'_trace.csv')
    with trace.open('w',newline='',encoding='utf-8') as f, (source/'events.jsonl').open(encoding='utf-8') as records:
        writer=csv.writer(f);writer.writerow(['clock_s','phase','position_error_mm','orientation_error_deg','sigma_min','joint_velocity_scale','requested_angular_rad_s','filtered_angular_rad_s','output_angular_rad_s','original_q_difference_rad'])
        for line in records:
            row=json.loads(line)
            if row.get('event')!='continuous_shadow_sample':continue
            now=row['clock_s'];frame=row['raw_intent']
            if row['phase']=='LOCKED_STOP' and not c.locked:
                c.request_stop('recorded_external_input_gate',now,True);external+=1
            try:m=c.step(frame,now,q,now)
            except Exception as exc:failure=dict(row=rows,reason=str(exc));break
            q=m.q.copy();rows+=1;error=float(np.max(np.abs(q-row['q_command'])));diff=max(diff,error)
            tcp,jac,_=model.forward(q);d=shadow_tracking(c,tcp,frame,now,'analysis')
            if not d['target_valid']:continue
            angular=float(np.linalg.norm((jac@m.v)[3:]));req=float(np.linalg.norm(c.requested_twist[3:]));filt=float(np.linalg.norm(c.servo.filtered_twist[3:]))
            errors.append(d['position_error_mm']);angles.append(d['orientation_error_deg']);scales.append(d['joint_velocity_scale']);sigmas.append(d['sigma_min'])
            requested.append(req);filtered.append(filt);actual.append(angular)
            writer.writerow([now,c.phase,errors[-1],angles[-1],sigmas[-1],scales[-1],req,filt,angular,error])
    return dict(source_session=source.name,source_sha256=hashlib.sha256((source/'events.jsonl').read_bytes()).hexdigest(),
        original_failure=saved['failure'],orientation_mode=orientation,original_input_diagnostics=saved.get('input_diagnostics'),
        replay_failure=failure,replayed_rows=rows,original_samples=saved['samples'],
        replayed_external_stop_transitions=external,max_original_q_difference_rad=diff,
        exact_command_reproduction=diff<1e-10,position_error_mm=stats(errors),orientation_error_deg=stats(angles),
        sigma_min=stats(sigmas),joint_scale=stats(scales),joint_scaled_fraction=float(np.mean(np.array(scales)<.999)) if scales else 0,
        near_singularity_fraction=float(np.mean(np.array(sigmas)<.02)) if sigmas else 0,
        requested_angular_rad_s=stats(requested),filtered_angular_rad_s=stats(filtered),actual_ideal_angular_rad_s=stats(actual),
        original_loop_period_ms=saved.get('loop_period_ms'),physical_feedback=False,robot_commands=0)

def main():
    p=argparse.ArgumentParser();p.add_argument('--sessions',nargs='+',required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--orientation',choices=('frozen','one-to-one'),default='frozen');a=p.parse_args()
    out=a.out.resolve()
    if not out.is_relative_to(ROOT/'data'):raise ValueError('V2 data only')
    out.mkdir(parents=True,exist_ok=False)
    reports=[]
    for session in a.sessions:
        source=(ROOT/'data'/session).resolve()
        if not source.is_relative_to(ROOT/'data'):raise ValueError('V2 data only')
        result=replay(source,out,a.orientation);reports.append(result);print(json.dumps(result),flush=True)
    (out/'report.json').write_text(json.dumps(dict(scope='offline_original_hand_input_with_recorded_stop_state_ideal_feedback',sessions=reports),indent=2),encoding='utf-8')
if __name__=='__main__':main()
