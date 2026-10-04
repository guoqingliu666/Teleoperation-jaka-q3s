"""连续控制链影子运行。仅模拟机器人反馈，不导入/连接SDK。"""
import argparse,datetime,hashlib,json,math,os,time,threading
from pathlib import Path
import numpy as np
from ..configuration import ROOT,load
from ..runtime import JsonLogger,Deadline,WindowsTimerResolution
from .global_kinematics import SerialPoE
from .continuous_pose import ContinuousPoseServo,ContinuousReceiver
from .twin_publish import TwinPublisher
from .shadow_telemetry import ShadowTelemetry

def load_model(path=None):
    if path is None:
        ref=json.loads((ROOT/'config/local/continuous_model.json').read_text(encoding='utf-8'))
        path=ROOT/ref['path']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=ref['sha256']:raise ValueError('model evidence hash mismatch')
    path=Path(path);r=json.loads(path.read_text(encoding='utf-8'))
    if (r.get('scope')!='official_FK_readonly_full_joint_domain_math_only' or not r.get('passed_math_only') or
        r.get('failure') or r.get('cleanup_errors') or r.get('motion_commands')!=0 or r.get('mode_changes')!=0):
        raise ValueError('Not validated readonly official model evidence')
    return SerialPoE(r['model']),r['model']['q_reference_rad'],path

def synthetic_frame(i,dt,basis):
    t=i*dt;phase=t%24;held=1<phase<10 or 13<phase<22
    elapsed=phase-1 if phase<12 else phase-13
    direction=1 if phase<12 else -1
    s=max(0.,min(1.,elapsed/6))
    displacement=np.array([.15,.025,.02])*s*direction
    p=basis.T@displacement;angle=.4*s*direction
    return dict(stamp=t,sequence=i,tracking_valid=True,grip=float(held),deadman=held,
        position_m=p.tolist(),rotation_xyzw=[math.sin(angle/2),0.,0.,math.cos(angle/2)],
        software_stop=False,synthetic=True)

def run(folder,duration=60.,synthetic=False,model_path=None):
    folder=Path(folder).resolve()
    if not folder.is_relative_to(ROOT/'data'):raise ValueError('shadow logs must be under project data')
    folder.mkdir(parents=True,exist_ok=False)
    model,q0,source=load_model(model_path);cfg=load();profile=json.loads((ROOT/'config/continuous.json').read_text(encoding='utf-8'))
    basis=np.array(json.loads((ROOT/'config/quest_xyz_commissioning.json').read_text(encoding='utf-8'))['basis'])
    planner=ContinuousPoseServo(q0,cfg,model,profile,basis);q=np.array(q0);origin=model.forward(q)[0]
    logger=JsonLogger(folder/'events.jsonl');receiver=None;twin=None;telemetry=None;timer=WindowsTimerResolution();stop=threading.Event()
    report=dict(scope='continuous_shadow_synthetic_plant_no_robot_SDK',motion_commands=0,servo_mode_changes=0,
        synthetic_input=synthetic,clock='virtual_time' if synthetic else 'host_monotonic',
        model_evidence=str(source),model_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        profile=profile,filters=cfg['filters'],failure=None,physical_feedback=False,live_accepted=False,
        fixed_start_sphere=False,fixed_30s_stop=False,samples=0,follow_samples=0,
        twin_publish_target_hz=1/cfg['period_s'],status_target_hz=5,twin_offers=0)
    logger.emit(event='startup',**report)
    def watch_stop():
        while not stop.wait(.05):
            if (folder/'STOP.request').exists():stop.set()
    watcher=threading.Thread(target=watch_stop,daemon=True);watcher.start()
    spans=[];peaks=np.zeros(3);durations=[];intervals=[];last_tick=None;feedback=q.copy();start=time.perf_counter();i=0
    latest_status=0.
    try:
        if not synthetic:
            receiver=ContinuousReceiver(5006,basis,audit=lambda **row:logger.emit(event='raw_quest_input',**row))
            twin=TwinPublisher();telemetry=ShadowTelemetry(folder);timer.start()
        deadline=Deadline(.008,spin_s=.0005);wall_start=time.perf_counter()
        while True:
            now=i*.008 if synthetic else time.perf_counter()
            elapsed=i*.008 if synthetic else now-wall_start
            if elapsed>=duration and planner.quiet():break
            if elapsed>duration+profile['controlled_stop_budget_s']+1.:raise RuntimeError('Shadow shutdown budget exceeded')
            if not synthetic:
                now,late=deadline.wait()
                if last_tick is not None:intervals.append((now-last_tick)*1000)
                last_tick=now
            frame=synthetic_frame(i,.008,basis) if synthetic else receiver.latest
            if not synthetic:
                # 点击事件必须锁存；最新槽可能在两个控制周期之间覆盖按下帧。
                if receiver.take_software_stop():planner.request_stop('software stop',now,True)
                if receiver.error and receiver.rejected:
                    # OpenXR启动字段未就绪/坏包不能产生运动；保持锁停并允许新鲜释放恢复。
                    planner.request_stop('input rejected: '+receiver.error,now,True)
            if elapsed>=duration or stop.is_set():
                if planner.stop_reason!='software stop':planner.request_stop('software stop',now,True)
                if planner.quiet():break
            begin=time.perf_counter()
            motion=planner.step(frame,now,feedback,now)
            if planner.phase=='FOLLOW':report['follow_samples']+=1
            durations.append((time.perf_counter()-begin)*1000)
            # 明确的理想模拟反馈；不伪造实际机器人测量、时延或停止距离。
            feedback=motion.q.copy();tcp=model.forward(feedback)[0]
            delta=(tcp[:3,3]-origin[:3,3])*1000;spans.append(delta.tolist())
            peaks=np.maximum(peaks,[np.max(np.abs(motion.v)),np.max(np.abs(motion.a)),np.max(np.abs(motion.j))])
            logger.emit(event='continuous_shadow_sample',clock_s=now,sequence=i,phase=planner.phase,
                q_command=motion.q.tolist(),simulated_feedback_q=feedback.tolist(),v=motion.v.tolist(),a=motion.a.tolist(),
                j=motion.j.tolist(),tcp_displacement_mm=delta.tolist(),raw_intent=frame,robot_commands=0)
            # 加速离线回放可比实时时钟快十几倍；仅虚拟时钟允许等待审计落盘。
            # 实时Quest路径不得在伺服周期中flush，队列过载仍明确失败。
            if synthetic and i%125==0:logger.flush()
            if not synthetic:
                # 关节状态每个8ms周期进入独立UDP最新槽。不要放回5Hz文字刷新分支。
                state=dict(mode='simulation',state='CONTINUOUS_SHADOW_'+planner.phase,connected=True,active=planner.held,
                    q=feedback.tolist(),tcp=None,feedback_age_ms=0.,feedback_age_basis='synthetic_plant_not_robot',session=str(folder))
                twin.offer(state);report['twin_offers']+=1
            if not synthetic and now-latest_status>=.2:
                latest_status=now
                status=dict(state=planner.phase,stop_reason=planner.stop_reason,input_error=receiver.error,
                    input_received=receiver.received,input_decoded=receiver.decoded,input_rejected=receiver.rejected,
                    tracking_valid=bool(frame and frame.get('tracking_valid')),
                    grip=None if frame is None else frame.get('grip'),filters_enabled=cfg['filters']['enabled'],
                    simulated_tcp_delta_mm=delta.tolist(),commands_to_robot=0,elapsed_s=elapsed,
                    twin_publish_target_hz=1/cfg['period_s'],shadow_requested_twist_limit_mm_s=profile['linear_speed_m_s']*1000,
                    shadow_joint_limit_deg_s=profile['joint_velocity_deg_s'])
                telemetry.offer(status,'仅模拟；请求速度上限 '+str(profile['linear_speed_m_s']*1000)+'mm/s，关节 '+str(profile['joint_velocity_deg_s'])+'度/s；状态125Hz。\n模拟TCP位移mm='+str(np.round(delta,1).tolist())+'；'+planner.phase)
            report['samples']=i+1;i+=1
    except BaseException as exc:report['failure']=str(exc)
    finally:
        stop.set();watcher.join(.2)
        if receiver:
            receiver.close()
            report['input_diagnostics']=dict(received=receiver.received,decoded=receiver.decoded,rejected=receiver.rejected,last_error=receiver.error)
        if twin:
            twin.offer(dict(mode='simulation',state='SHADOW_STOPPED',connected=False,active=False));twin.close()
            report['twin_latest_slot_replacements']=twin.dropped
        if telemetry:
            telemetry.close();report['status_writer']=dict(offered=telemetry.offered,written=telemetry.written,dropped=telemetry.dropped,error=telemetry.error)
        timer.close();logger.close()
    report.update(wall_elapsed_s=time.perf_counter()-start,simulated_or_observed_s=i*.008 if synthetic else time.perf_counter()-start,
        command_stop_events=planner.events,maximum_displacement_mm=max((np.linalg.norm(d) for d in spans),default=0.),
        component_spans_mm=np.ptp(spans,axis=0).tolist() if spans else [0.]*3,
        command_peak_v_a_j=peaks.tolist(),planning_ms={str(p):float(np.percentile(durations,p)) for p in (50,95,99,100)} if durations else {},
        loop_period_ms={str(p):float(np.percentile(intervals,p)) for p in (50,95,99,100)} if intervals else {})
    report['passed_process_only']=report['failure'] is None and report['samples']>0
    report['passed_shadow_only']=report['passed_process_only'] and report['follow_samples']>0
    report['input_follow_observed']=report['follow_samples']>0
    (folder/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('passed_shadow_only','failure','samples','maximum_displacement_mm','planning_ms')},ensure_ascii=False))
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--synthetic',action='store_true')
    p.add_argument('--duration',type=float,default=120);p.add_argument('--model',type=Path);p.add_argument('--session',type=Path)
    a=p.parse_args()
    if not 1<=a.duration<=3600:p.error('duration must be 1..3600s; this only limits the observation session')
    folder=a.session or ROOT/'data'/('continuous_shadow_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    r=run(folder,a.duration,a.synthetic,a.model);print('REPORT '+str(folder/'report.json'))
    return 0 if r['passed_shadow_only'] else 2

if __name__=='__main__':raise SystemExit(main())
