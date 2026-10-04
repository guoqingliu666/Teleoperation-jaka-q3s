"""H7前置能力证据核验；不连接SDK，不替代新H1、当前输入或现场READY。"""
import datetime,json,time
from pathlib import Path
from ..configuration import ROOT,load
from .xyz_evidence import SHARED,sha

H7_FILES=('src/v2_servo/teleop/quest_tcp_rotation.py','src/v2_servo/teleop/quest_rotation.py',
          'src/v2_servo/teleop/live_rotation_input.py',
          'src/v2_servo/teleop/gesture_completion.py',
          'src/v2_servo/teleop/local_rotation.py','src/v2_servo/teleop/rotation_rehearsal.py',
          'src/v2_servo/teleop/rotation_evidence.py','src/v2_servo/filters.py',
          'src/v2_servo/trajectory.py','src/v2_servo/configuration.py','src/v2_servo/safety.py',
          'vendor/quest_vr_input.py','.deps/ruckig.cp312-win_amd64.pyd','config/quest_xyz_commissioning.json',
          'config/site_observed_20260928.json')

def report_path(path,prefix):
    path=Path(path).resolve()
    if not path.is_relative_to(ROOT/'data') or not path.is_file():raise ValueError('H7 requires existing V2 evidence')
    try:created=datetime.datetime.strptime(path.parent.name[len(prefix):],'%Y%m%d_%H%M%S_%f').replace(tzinfo=datetime.timezone(datetime.timedelta(hours=8))).timestamp()
    except ValueError:raise ValueError('H7 invalid evidence session name')
    if not path.parent.name.startswith(prefix) or not 0<=time.time()-created<=86400:raise ValueError('H7 capability evidence stale; current site must be checked separately')
    return path,json.loads(path.read_text(encoding='utf-8'))

def verify_h6_baseline(path,sdk_hashes):
    path,r=report_path(path,'h6_');meta=json.loads(path.with_name('metadata.json').read_text(encoding='utf-8'))
    if (r.get('experiment')!='H6' or not r.get('passed_quest_tcp_xyz_session_only') or r.get('failure') or r.get('cleanup_errors')
        or r.get('servo_exited') is not True or not r.get('physical_stationary_observed') or not meta.get('site_confirmed')
        or meta.get('requested_scope')!='H6' or meta.get('session')!=path.parent.name
        or meta.get('robot_serial_user_confirmed')!='S5160217' or meta.get('sdk_binding')!='native-edg0'
        or meta.get('sdk_artifacts_sha256')!=sdk_hashes):raise ValueError('H7 requires successful real H6 baseline with matching robot/SDK')
    # 共用循环新增H7分支，因此H6不能证明新代码已真机通过；另需同代码新H1。
    for f in SHARED+('src/v2_servo/filters.py','src/v2_servo/trajectory.py','.deps/ruckig.cp312-win_amd64.pyd'):
        if meta.get('sha256',{}).get(f)!=sha(ROOT/f):raise ValueError('H6 shared filter/input/SDK changed: '+f)
    return {'path':str(path),'sha256':sha(path),'scope':'H6_capability_not_H7_live_approval'}

def verify_rotation_input(path):
    path,r=report_path(path,'quest_rotation_shadow_')
    if (r.get('scope')!='real_Quest_rotation_input_RX_candidate_shadow_only_no_robot_SDK' or r.get('robot_commands')!=0
        or not r.get('passed_input_and_shadow_only') or r.get('failure') or r.get('active_rejected')!=0
        or not r.get('readiness_passed') or r.get('completed_axes')!=['RX','RY','RZ'] or r.get('normal_releases',0)<3
        or r.get('filters')!=load()['filters']):raise ValueError('H7 requires real passed three-axis shadow input; offline evidence refused')
    for f,digest in r.get('code_sha256',{}).items():
        if f not in ('scripts/quest_rotation_shadow.py','src/v2_servo/teleop/quest_rotation.py','src/v2_servo/teleop/local_rotation.py','src/v2_servo/teleop/rotation_rehearsal.py'):
            raise ValueError('unexpected shadow provenance file')
        if digest!=sha(ROOT/f):raise ValueError('rotation shadow code changed: '+f)
    if len(r.get('code_sha256',{}))!=4:raise ValueError('rotation shadow provenance missing')
    model=Path(r['model_source']).resolve()
    if not model.is_relative_to(ROOT/'data') or sha(model)!=r['model_sha256']:raise ValueError('shadow official model changed')
    official=json.loads(model.read_text(encoding='utf-8'))
    if not official.get('passed_math_only') or official.get('failure') or official.get('robot_serial')!='S5160217' or official.get('axis')!='RX':raise ValueError('shadow official RX model invalid')
    for name in ('samples.jsonl','raw_input.jsonl'):
        if not path.with_name(name).is_file() or not path.with_name(name).stat().st_size:raise ValueError('shadow raw evidence missing')
    return {'path':str(path),'sha256':sha(path),'scope':'real_input_shadow_not_real_rotation_motion'}


def verify_rotation_baseline(path,expected_axis,sdk_hashes):
    """RY须真实RX通过，RZ须真实RY通过；新分支仍需同代码新H1。"""
    path,r=report_path(path,'h7_');meta=json.loads(path.with_name('metadata.json').read_text(encoding='utf-8'))
    axis=r.get('rotation_axis','RX')
    if (axis!=expected_axis or r.get('scope')!='live_H7_Quest_Tool1_base_'+axis+'_local_0p02deg_only'
        or r.get('official_local_tcp_rotation',{}).get('axis')!=axis or meta.get('rotation_axis','RX')!=axis
        or r.get('servo_commands',0)<30 or len(r.get('samples',[]))<30
        or r.get('experiment')!='H7' or not r.get('passed_quest_tcp_rotation_session_only') or r.get('failure') or r.get('cleanup_errors')
        or r.get('servo_exited') is not True or not r.get('physical_stationary_observed') or r.get('moving_release_count',0)<1
        or not meta.get('site_confirmed') or meta.get('requested_scope')!='H7' or meta.get('session')!=path.parent.name
        or meta.get('robot_serial_user_confirmed')!='S5160217' or meta.get('sdk_binding')!='native-edg0'
        or meta.get('sdk_artifacts_sha256')!=sdk_hashes):raise ValueError('requires successful real '+expected_axis+' baseline with same robot/SDK')
    for f in SHARED+('src/v2_servo/teleop/quest_rotation.py','src/v2_servo/teleop/local_rotation.py',
                     'src/v2_servo/filters.py','src/v2_servo/trajectory.py','.deps/ruckig.cp312-win_amd64.pyd'):
        if meta.get('sha256',{}).get(f)!=sha(ROOT/f):raise ValueError('rotation baseline stable core changed: '+f)
    wrapper_hash=meta.get('sha256',{}).get('src/v2_servo/teleop/quest_tcp_rotation.py')
    legacy_RX='02aa8425c8631c34fd4e016ddf3e53a3faf3f2e31fbef73c35a6cb4bb70fc3de'
    if wrapper_hash!=sha(ROOT/'src/v2_servo/teleop/quest_tcp_rotation.py') and not (axis=='RX' and wrapper_hash==legacy_RX):
        raise ValueError('unreviewed rotation wrapper provenance')
    return {'path':str(path),'axis':axis,'sha256':sha(path),'scope':'previous_axis_capability_not_new_axis_live_approval'}
