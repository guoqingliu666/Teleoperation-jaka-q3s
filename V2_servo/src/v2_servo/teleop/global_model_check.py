"""官方FK全关节域数学检查：只读SDK，禁止上电/使能/模式切换/运动。"""
import argparse,datetime,json,socket,time
import numpy as np
from ..configuration import ROOT,load,vector
from ..runtime_paths import load_runtime_options,sdk_artifacts
from ..jaka import checked
from .global_kinematics import SerialPoE,evaluate_samples,pose_matrix,rotation_log

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--official-readonly',action='store_true')
    parser.add_argument('--samples',type=int,default=512)
    args=parser.parse_args()
    if not args.official_readonly:parser.error('Explicit --official-readonly required; never a motion command')
    if not 128<=args.samples<=4096:parser.error('sample count outside 128..4096')
    options=load_runtime_options();cfg=load()
    expected=json.loads((ROOT/'config/site_observed_20260928.json').read_text(encoding='utf-8'))
    folder=ROOT/'data'/('global_fk_readonly_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(scope='official_FK_readonly_full_joint_domain_math_only',motion_commands=0,mode_changes=0,
        seed=20261003,failure=None,cleanup_errors=[],passed_math_only=False,live_motion_approved=False,
        samples=[],sdk_artifacts=sdk_artifacts(options['sdk_directory']))
    rc=None;logged=False;lease=socket.socket();calls=[]
    try:
        from .backends import controller_owners
        owners=controller_owners(options['host'])
        if owners:raise RuntimeError('Controller owner exists; manually disconnect, no takeover: '+str(owners))
        lease.bind(('127.0.0.1',36151));lease.listen(1)
        with socket.create_connection((options['host'],10001),timeout=2):pass
        from .native_sdk import NativeRC
        native=NativeRC(options['sdk_directory'],options['host'])
        allowed={'login','logout','is_in_servomove','get_tool_id','get_tool_data','get_payload',
            'get_user_frame_id','get_actual_joint_position','get_actual_tcp_position','get_sdk_version','kine_forward'}
        class ReadOnly:
            def __getattr__(self,name):
                if name not in allowed:raise RuntimeError('Read-only API refused: '+name)
                def invoke(*a):
                    begin=time.perf_counter();result=getattr(native,name)(*a)
                    calls.append(dict(api=name,started_s=begin,elapsed_ms=(time.perf_counter()-begin)*1000,code=result[0]))
                    return result
                return invoke
        rc=ReadOnly();checked(rc.login(),'login');logged=True
        if checked(rc.is_in_servomove(),'is_in_servomove'):raise RuntimeError('Existing servo session, refusing')
        if checked(rc.get_user_frame_id(),'user frame')!=0:raise RuntimeError('Needs robot base frame; no automatic change')
        tool=checked(rc.get_tool_id(),'tool id')
        if tool!=expected['tool_id']:raise RuntimeError('Tool1 mismatch')
        raw=rc.get_tool_data(tool);checked(raw,'tool data')
        tcp_parameters=raw[-1]
        if not np.allclose(vector(tcp_parameters),expected['tcp_mm_rad'],rtol=0,atol=.001):raise RuntimeError('TCP parameters mismatch')
        report.update(tool_id=tool,tcp_parameters=tcp_parameters,payload=checked(rc.get_payload(),'payload'),
            sdk_version=checked(rc.get_sdk_version(),'version'),robot_serial_user_configuration=expected['serial'])
        q=vector(checked(rc.get_actual_joint_position(),'q'));actual=pose_matrix(checked(rc.get_actual_tcp_position(),'tcp'))
        from .official_local import official_jacobian
        j,identification=official_jacobian(rc,q)
        origin=checked(rc.kine_forward(q.tolist()),'FK reference')
        record=SerialPoE.identify(q,origin,j);model=SerialPoE(record)
        report['identification_samples']=identification;report['model']=record
        reference=pose_matrix(origin)
        report['current_FK_feedback_mm']=float(np.linalg.norm(reference[:3,3]-actual[:3,3])*1000)
        report['current_FK_feedback_deg']=float(np.rad2deg(np.linalg.norm(rotation_log(reference[:3,:3]@actual[:3,:3].T))))
        lo=vector(cfg['limits']['lower_rad']);hi=vector(cfg['limits']['upper_rad']);rng=np.random.default_rng(report['seed'])
        candidates=list(rng.uniform(lo,hi,(args.samples,6)))
        # 单轴端点和中心，只作FK计算，不让真机去这些姿态。
        center=(lo+hi)/2
        for axis in range(6):
            for value in (lo[axis],hi[axis]):
                candidate=center.copy();candidate[axis]=value;candidates.append(candidate)
        samples=[]
        for i,candidate in enumerate(candidates):
            samples.append(dict(q_rad=candidate.tolist(),official_pose_mm_rad=checked(rc.kine_forward(candidate.tolist()),'FK holdout')))
            if i%128==0:print('READONLY mathematical samples '+str(i)+'; robot movement commands=0',flush=True)
        report['samples']=evaluate_samples(model,samples)
        report['sampled_joint_min_rad']=np.min(candidates,axis=0).tolist();report['sampled_joint_max_rad']=np.max(candidates,axis=0).tolist()
        report['max_translation_error_mm']=max(x['translation_error_mm'] for x in report['samples'])
        report['max_rotation_error_deg']=max(x['rotation_error_deg'] for x in report['samples'])
        q_end=vector(checked(rc.get_actual_joint_position(),'q end'));report['q_end_rad']=q_end.tolist()
        if np.max(np.abs(q-q_end))>np.deg2rad(.005):raise RuntimeError('Robot moved during readonly model audit')
        if report['current_FK_feedback_mm']>.1 or report['current_FK_feedback_deg']>.05:raise RuntimeError('Current FK/feedback mismatch')
        # 数值模型一致性门槛，不是干细胞任务精度、物理绝对精度或安全距离。
        report['numerical_consistency_limits']=dict(translation_mm=.1,rotation_deg=.01,scope='mathematical_model_only')
        if report['max_translation_error_mm']>.1 or report['max_rotation_error_deg']>.01:raise RuntimeError('Full-domain PoE does not match official FK')
        report['passed_math_only']=True
    except Exception as exc:report['failure']=str(exc)
    finally:
        if logged:
            try:report['logout']=rc.logout();checked(report['logout'],'logout')
            except Exception as exc:report['cleanup_errors'].append(str(exc));report['passed_math_only']=False
        lease.close();report['api_calls']=calls
        (folder/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('passed_math_only','failure','motion_commands','mode_changes')},ensure_ascii=False))
    print('REPORT '+str(folder/'report.json'))
    return 0 if report['passed_math_only'] else 2

if __name__=='__main__':raise SystemExit(main())
