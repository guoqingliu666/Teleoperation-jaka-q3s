"""H9前置能力核验；已通过H8无需重做影子，仍须当前READY100/新H1/新FK。"""
import json
import numpy as np
from ..configuration import ROOT
from .rotation_evidence import report_path
from .xyz_evidence import SHARED,sha
from ..runtime_paths import same_sdk_artifacts, load_runtime_options

H9_FILES=tuple(dict.fromkeys(SHARED+(
    'src/v2_servo/teleop/static_hold.py','src/v2_servo/teleop/quest_tcp_xyz.py',
    'src/v2_servo/teleop/quest_pose_shadow.py','src/v2_servo/teleop/quest_pose_live.py',
    'src/v2_servo/teleop/live_rotation_input.py','src/v2_servo/teleop/local_xyz.py',
    'src/v2_servo/teleop/pose_expansion.py','src/v2_servo/teleop/expansion_evidence.py',
    'src/v2_servo/filters.py','src/v2_servo/trajectory.py','src/v2_servo/configuration.py',
    'src/v2_servo/safety.py','.deps/ruckig.cp312-win_amd64.pyd',
    'src/v2_servo/teleop/commissioning_guidance.py','src/v2_servo/teleop/input_gate.py',
    'src/v2_servo/teleop/gesture_completion.py','src/v2_servo/teleop/twin_publish.py',
    'config/site_observed_20260928.json','config/quest_pose_expansion.json','config/quest_xyz_commissioning.json',
    'scripts/quest_pose_live.py','src/v2_servo/teleop/workspace_evidence.py')))

def expansion_hashes():
    from pathlib import Path
    result={f:sha(ROOT/f) for f in H9_FILES}
    directory=Path(load_runtime_options()['sdk_directory'])
    for f in [directory/'jakaAPI.dll']+[directory.parents[1]/'c&c++/inc_of_c'/n for n in ('jakaAPI.h','jktypes.h')]:
        result[str(f)]=sha(f)
    return result

def verify_h8_baseline(path,sdk_hashes):
    path,r=report_path(path,'h8_');meta=json.loads(path.with_name('metadata.json').read_text(encoding='utf-8'))
    if (r.get('scope')!='live_H8_Quest_Tool1_combined_6dof_local_finite_only' or r.get('experiment')!='H8'
        or r.get('failure') or r.get('cleanup_errors') or not r.get('passed_quest_pose_finite_session_only')
        or r.get('servo_exited') is not True or not r.get('physical_stationary_observed')
        or r.get('logout')!=[0] or r.get('moving_release_count',0)<2 or r.get('servo_commands',0)<30
        or meta.get('scope')!='live_H8_finite_commissioning' or meta.get('session')!=path.parent.name
        or meta.get('robot_serial_user_confirmed')!='S5160217' or not meta.get('site_confirmed')
        or meta.get('sdk_binding')!='native-edg0' or not same_sdk_artifacts(meta.get('sdk_artifacts_sha256'),sdk_hashes)):
        raise ValueError('H9 requires real passed H8 with matching robot/SDK; offline refused')
    for f in SHARED+('src/v2_servo/teleop/quest_pose_live.py','src/v2_servo/teleop/live_rotation_input.py',
                     'src/v2_servo/teleop/quest_tcp_xyz.py','src/v2_servo/filters.py','.deps/ruckig.cp312-win_amd64.pyd'):
        if meta.get('sha256',{}).get(f)!=sha(ROOT/f):raise ValueError('H8 stable feedback/input/filter changed: '+f)
    rows=r['samples']
    if len(rows)!=r['servo_commands'] or any((x.get('quest_intent') or {}).get('offline_replay') for x in rows):
        raise ValueError('H8 raw sample origin/count invalid')
    for key,minimum in (('TCP_xyz_actual_delta_mm',.02),('TCP_xyz_command_estimate_mm',.02),
                        ('TCP_rotation_actual_deg',.003),('TCP_rotation_command_deg',.003)):
        if not np.all(np.ptp([x[key] for x in rows],axis=0)>=minimum):raise ValueError('H8 combined coverage missing')
    h1=json.loads(path.with_name('h1_report.json').read_text(encoding='utf-8'))
    if not h1.get('passed_static_only') or h1.get('failure') or h1.get('cleanup_errors'):raise ValueError('H8 same-process H1 missing')
    return {'path':str(path),'sha256':sha(path),'metadata_sha256':sha(path.with_name('metadata.json')),
            'scope':'H8_capability_not_H9_live_or_full_workspace_approval'}
