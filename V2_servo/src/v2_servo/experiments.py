"""可重复软件实验；报告中的 scope 不得改成 live。"""
import copy, hashlib, itertools, json, math, re, time, platform, sys
from pathlib import Path
import numpy as np
from .configuration import ROOT,load
from .filters import CommandFilter
from .trajectory import Motion,plan,OnlinePlanner
from .kinematics import ArmKinematics
from .runtime import Deadline

def write_json(path,obj):
    Path(path).write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')

def rms(a): return float(np.sqrt(np.mean(np.asarray(a)**2)))

def compare_filters(cfg,out):
    dt=cfg['period_s']; t=np.arange(0,16,dt)
    clean=.12*np.sin(2*np.pi*.3*t); noise=.006*np.sin(2*np.pi*25*t)+np.random.default_rng(cfg['seed']).normal(0,.002,len(t))
    raw=clean+noise
    f=CommandFilter(cfg['filters'],dt,np.zeros(6)); baseline=CommandFilter(cfg['filters'],dt,np.zeros(6))
    filtered=np.array([f.update([x]*6)[0] for x in raw]); clean_filtered=np.array([baseline.update([x]*6)[0] for x in clean])
    use=t>2; rows=[]
    for label,y in [('bypass_offline',raw),('filters_on',filtered)]:
        v=np.gradient(y,dt); a=np.gradient(v,dt); j=np.gradient(a,dt)
        rows.append({'mode':label,'tracking_rmse_rad':rms((y-clean)[use]),'velocity_rms_rad_s':rms(v[use]),
                     'acceleration_rms_rad_s2':rms(a[use]),'jerk_rms_rad_s3':rms(j[use]),
                     'injected_noise_rms_rad':rms((raw-clean)[use]) if label.startswith('bypass') else rms((filtered-clean_filtered)[use])})
    basis=np.column_stack([np.sin(2*np.pi*.3*t[use]),np.cos(2*np.pi*.3*t[use]),np.ones(np.sum(use))])
    fit=np.linalg.lstsq(basis,filtered[use],rcond=None)[0]
    lag=-math.atan2(fit[1],fit[0])/(2*np.pi*.3)
    attenuation=1-rows[1]['injected_noise_rms_rad']/rows[0]['injected_noise_rms_rad']
    np.savez_compressed(out/'filter_comparison.npz',t=t,clean=clean,raw=raw,filtered=filtered,clean_filtered=clean_filtered)
    result={'scope':'offline_signal_filter_only_not_controller_readback','rows':rows,'noise_rms_reduction':attenuation,
            'estimated_lag_s':lag,'pass_noise_reduction':bool(attenuation>.5),
            'note':'tracking RMSE includes low-pass delay; command signals alone are not physical vibration measurements'}
    write_json(out/'filter_comparison.json',result); return result

def workspace(cfg,out,quick=False):
    arm=ArmKinematics(ROOT/'models/assembly_reference.urdf')
    lo=np.array(cfg['limits']['lower_rad']); hi=np.array(cfg['limits']['upper_rad'])
    n=3 if quick else 5
    grid=np.array(list(itertools.product(*[np.linspace(a,b,n) for a,b in zip(lo,hi)])))
    rnd=np.random.default_rng(cfg['seed']).uniform(lo,hi,size=(100 if quick else 2000,6))
    q=np.vstack([grid,rnd]); xyz=[]; singular=[]
    for values in q:
        t,j,_=arm.forward(values); xyz.append(t[:3,3]); singular.append(np.linalg.svd(j,compute_uv=False)[-1])
    xyz=np.array(xyz); singular=np.array(singular)
    np.savez_compressed(out/'workspace_samples.npz',q=q,xyz=xyz,sigma_min=singular)
    result={'scope':'URDF_kinematics_only','grid_per_axis':n,'grid_count':len(grid),'random_count':len(rnd),
            'q_min_rad':q.min(axis=0).tolist(),'q_max_rad':q.max(axis=0).tolist(),
            'xyz_min_m':xyz.min(axis=0).tolist(),'xyz_max_m':xyz.max(axis=0).tolist(),
            'singularity_screen_threshold':.02,'singularity_screen_reject_count':int(np.sum(singular<.02)),
            'collision_checked':False,'hardware_approved_points':0,'pass_joint_domain_coverage':bool(np.allclose(q.min(axis=0),lo) and np.allclose(q.max(axis=0),hi)),
            'note':'URDF Link_06 origin, not calibrated flange/TCP; characteristic length 1 m; finite samples do not certify swept path'}
    write_json(out/'workspace.json',result); return result

def trajectories(cfg,out,quick=False):
    rng=np.random.default_rng(cfg['seed']); lo=np.array(cfg['limits']['lower_rad']); hi=np.array(cfg['limits']['upper_rad'])
    rows=[]; violations=0
    for i in range(30 if quick else 300):
        limits=copy.deepcopy(cfg['limits']); scale=[.1,.25,.5,1,2][i%5]; accel=[.5,1,2,4][i%4]
        limits['velocity_rad_s']=(np.array(limits['velocity_rad_s'])*scale).tolist()
        limits['acceleration_rad_s2']=(np.array(limits['acceleration_rad_s2'])*accel).tolist()
        start=lo if i==0 else rng.uniform(lo,hi); target=hi if i==0 else rng.uniform(lo,hi)
        segment=plan(Motion.rest(start),target,limits)
        samples=[segment.sample(t) for t in np.linspace(0,segment.duration,101)]
        metrics={k:float(np.max(np.abs(np.array([getattr(s,k) for s in samples])))) for k in ['v','a','j']}
        certified=segment.valid(limits); violations+=not certified
        rows.append({'case':i,'speed_scale':scale,'acceleration_scale':accel,'duration_s':segment.duration,'certified':certified,'peak':metrics})
    # 连续重规划和目标突变，接缝 q/v/a/j 都按解析值检查。
    center=(lo+hi)/2; planner=OnlinePlanner(center,cfg['limits'],cfg['period_s']); filt=CommandFilter(cfg['filters'],cfg['period_s'],center)
    trace=[]; splice=0.; rejections=0
    for i in range(2500):
        desired=center+(.5 if (i//375)%2 else -.5)*np.ones(6)
        target=filt.update(desired)
        if i%8==0:
            before=planner.state
            try:
                planner.retarget(target,.4)
                after=planner.segment.sample(0)
                splice=max(splice,max(float(np.max(np.abs(x-y))) for x,y in zip(vars(before).values(),vars(after).values())))
            except ValueError: rejections+=1
        m=planner.step(); trace.append(np.concatenate(([i*cfg['period_s']],m.q,m.v,m.a,m.j)))
    np.savez_compressed(out/'smooth_trajectory.npz',trace=np.array(trace))
    result={'scope':'analytic_joint_trajectory','random_paths':rows,'constraint_violations':int(violations),'online_retarget_rejections':rejections,
            'max_c3_splice_error':splice,'pass':bool(violations==0 and splice<1e-8),'dt_s':cfg['period_s'],
            'note':'jerk continuous in reference; controller/physical jerk not measured'}
    write_json(out/'trajectory_matrix.json',result); return result

def frequency_load_endurance(cfg,out):
    dt=cfg['period_s']; rows=[]
    for hz in [.1,.5,1,2,5,10,20,40]:
        t=np.arange(0,20,dt); x=.1*np.sin(2*np.pi*hz*t)
        f=CommandFilter(cfg['filters'],dt,np.zeros(6)); y=np.array([f.update([v]*6)[0] for v in x])
        rows.append({'hz':hz,'gain_rms':rms(y[len(y)//2:])/rms(x[len(x)//2:])})
    # 负载未知：只用明确标注的时间常数敏感性模型，不将它标成 kg 或真实关节动力学。
    trace=np.load(out/'smooth_trajectory.npz')['trace']; command=trace[:,1:7]; sensitivity=[]
    for tau in [.008,.016,.032,.064]:
        q=command[0].copy(); error=[]; alpha=-math.expm1(-dt/tau)
        for x in command:
            q=q+alpha*(x-q); error.append(q-x)
        sensitivity.append({'synthetic_time_constant_s':tau,'tracking_rmse_rad':rms(error)})
    # 一小时虚拟信号经过真实滤波 update，计数和有限性检查；不是一小时墙钟或硬件耐久。
    count=int(3600/dt); f=CommandFilter(cfg['filters'],dt,np.zeros(6)); peak=0.; begin=time.perf_counter()
    for i in range(count):
        y=f.update(np.full(6,.1*math.sin(2*math.pi*.5*i*dt)))
        peak=max(peak,float(np.max(np.abs(y))))
    result={'scope':'synthetic_sensitivity_and_virtual_endurance','frequency_sweep':rows,'plant_sensitivity':sensitivity,
            'virtual_duration_s':3600,'filter_updates':count,'wall_time_s':time.perf_counter()-begin,'finite':bool(np.isfinite(peak)),
            'peak_rad':peak,'physical_load_change_test':'NOT_RUN','real_duration_endurance':'NOT_RUN'}
    write_json(out/'frequency_load_endurance.json',result); return result

def timing(cfg,out,seconds=5):
    timer=Deadline(cfg['period_s']); stamps=[]; lateness=[]
    for _ in range(int(seconds/cfg['period_s'])):
        now,late=timer.wait(); stamps.append(now); lateness.append(late)
    intervals=np.diff(stamps); jitter=intervals-cfg['period_s']
    result={'scope':'host_wall_clock_no_SDK_no_robot','samples':len(stamps),'period_ms_mean':float(intervals.mean()*1000),
            'jitter_ms_rms':rms(jitter)*1000,'lateness_ms_p99':float(np.quantile(lateness,.99)*1000),
            'lateness_ms_max':max(lateness)*1000,'deadline_misses':int(np.sum(np.array(lateness)>cfg['safety']['max_lateness_s'])),
            'hardware_period_verified':False,'clock_implementation':time.get_clock_info('perf_counter').implementation,
            'clock_resolution_s':time.get_clock_info('perf_counter').resolution}
    write_json(out/'host_timing.json',result); return result

def accident_replay(cfg,out):
    old=ROOT/'data/evidence/accident'
    reports=[]
    for name in ['level_a_20260919_214725_dd2d0d8e','level_a_20260919_214948_3af0bd8e']:
        p=old/name/'events.jsonl'
        rows=[json.loads(line) for line in p.read_text(encoding='utf-8').splitlines() if line.strip()]
        targets=[r for r in rows if r.get('event')=='target' and 'actual_age_ms' in r]
        rejected=[r for r in targets if r['actual_age_ms']>cfg['safety']['feedback_timeout_s']*1000]
        reports.append({'session':name,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'targets':len(targets),
                        'max_reported_feedback_age_ms':max((r['actual_age_ms'] for r in targets),default=None),
                        'old_gui_queued_with_age_over_250ms':sum(bool(r.get('sent')) and r['actual_age_ms']>250 for r in targets),
                        'would_reject_at_24ms':len(rejected),
                        'first_would_reject_time_ns':rejected[0]['time_ns'] if rejected else None})
    p=old/'jaka_jog_20260919_214648.log'
    text=p.read_text(encoding='utf-8'); costs=[float(v) for v in re.findall(r'TELEMETRY_TIMING elapsed_ms=([\d.]+)',text)]
    r={'scope':'recorded_log_threshold_replay_not_reconstruction_of_physical_fault','sessions':reports,
       'sdk_log_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'max_logged_telemetry_ms':max(costs),'movement_commands_sent':0,
       'J4_undervoltage_root_cause':'unresolved; controller electrical/drive event timeline required'}
    write_json(out/'accident_replay.json',r); return r

def run_all(out,quick=False):
    cfg=load(); out=Path(out); out.mkdir(parents=True,exist_ok=True)
    write_json(out/'run_config.json',cfg)
    write_json(out/'environment.json',{'python':sys.version,'executable':sys.executable,'numpy':np.__version__,'platform':platform.platform(),'seed':cfg['seed']})
    print('FILTER_CONFIG='+json.dumps(cfg['filters'],ensure_ascii=False),flush=True)
    results={}
    for name,fn in [('filter',lambda:compare_filters(cfg,out)),('workspace',lambda:workspace(cfg,out,quick)),
                    ('trajectory',lambda:trajectories(cfg,out,quick)),('fault_log_replay',lambda:accident_replay(cfg,out)),
                    ('frequency_load_endurance',lambda:frequency_load_endurance(cfg,out)),('host_timing',lambda:timing(cfg,out))]:
        print('RUN '+name,flush=True); results[name]=fn()
    summary={'scope':'offline_candidate','hardware_motion_commands':0,'full_hardware_acceptance':'BLOCKED',
             'software_checks_pass':bool(results['filter']['pass_noise_reduction'] and results['workspace']['pass_joint_domain_coverage'] and results['trajectory']['pass']),
             'host_deadline_misses':results['host_timing']['deadline_misses'],'results':list(results)}
    write_json(out/'summary.json',summary)
    print(json.dumps(summary,ensure_ascii=False),flush=True); return summary
