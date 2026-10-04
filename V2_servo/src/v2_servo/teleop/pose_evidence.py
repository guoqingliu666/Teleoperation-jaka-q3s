"""H8前置证据只读核验：真实联合输入必须通过；离线日志不能授予运动。"""
import datetime,json,time
import numpy as np
from ..configuration import ROOT,load
from .xyz_evidence import sha,SHARED
from .rotation_evidence import verify_rotation_baseline,report_path,H7_FILES

SHADOW_FILES=('scripts/quest_pose_shadow.py','src/v2_servo/teleop/shadow_start.py',
    'src/v2_servo/teleop/quest_pose_shadow.py','src/v2_servo/teleop/live_rotation_input.py',
    'src/v2_servo/teleop/quest_tcp_xyz.py','src/v2_servo/filters.py','config/quest_xyz_commissioning.json')
H8_FILES=tuple(dict.fromkeys(SHARED+H7_FILES+SHADOW_FILES+(
    'src/v2_servo/teleop/quest_pose_live.py','src/v2_servo/teleop/static_hold.py',
    'src/v2_servo/teleop/local_xyz.py','src/v2_servo/teleop/pose_evidence.py',
    'scripts/quest_pose_live.py')))


def code_hashes():return {name:sha(ROOT/name) for name in H8_FILES}


def verify_pose_shadow(path,sdk_hashes):
    path,r=report_path(path,'quest_pose_shadow_')
    created=datetime.datetime.strptime(path.parent.name[len('quest_pose_shadow_'):],'%Y%m%d_%H%M%S_%f').replace(
        tzinfo=datetime.timezone(datetime.timedelta(hours=8))).timestamp()
    if r.get('scope')!='real_Quest_combined_6dof_shadow_no_robot_SDK':
        raise ValueError('H8 requires real Quest input; synthetic/offline source refused')
    if time.time()-created>600:raise ValueError('H8 combined input shadow older than ten minutes')
    if (r.get('scope')!='real_Quest_combined_6dof_shadow_no_robot_SDK' or r.get('robot_commands')!=0
        or not r.get('passed_shadow_only') or r.get('failure') or r.get('active_rejected')!=0
        or not r.get('first_grip_observed_s') or r.get('filters')!=load()['filters']):
        raise ValueError('H8 requires actual passed combined Quest shadow; offline/failed logs refused')
    if set(r.get('code_sha256',{}))!=set(SHADOW_FILES):raise ValueError('combined shadow provenance incomplete')
    for name in SHADOW_FILES:
        if r['code_sha256'][name]!=sha(ROOT/name):raise ValueError('combined shadow code changed: '+name)
    source=ROOT/'data'/path.parent.name/'raw_input.jsonl'
    raw=[json.loads(line) for line in source.read_text(encoding='utf-8').splitlines()]
    if not raw or any(row['raw'].get('offline_replay') for row in raw):raise ValueError('synthetic raw input cannot authorize H8')
    rows=[json.loads(line) for line in path.with_name('samples.jsonl').read_text(encoding='utf-8').splitlines()]
    if len(rows)<30 or r.get('samples')!=len(rows):raise ValueError('combined shadow sample audit incomplete')
    if not all(row.get('robot_commands')==0 and row.get('not_robot_feedback') is True for row in rows):
        raise ValueError('shadow sample origin mislabeled')
    xyz=np.ptp([row['candidate_xyz_mm'] for row in rows],axis=0)
    rot=np.ptp([row['candidate_rotation_deg'] for row in rows],axis=0)
    if not np.all(xyz>=.02) or not np.all(rot>=.003):raise ValueError('combined shadow six-axis coverage missing')
    if sum(bool(e.get('moving_at_release')) for e in r.get('release_events',[]))<2:
        raise ValueError('combined shadow requires two moving releases')
    model=report_path(r['model_source'],'h7_')[0]
    if sha(model)!=r['model_sha256']:raise ValueError('combined shadow model source changed')
    baseline=verify_rotation_baseline(model,'RZ',sdk_hashes)
    return {'path':str(path),'sha256':sha(path),'model_baseline':baseline,
            'scope':'real_Quest_combined_shadow_not_live_6dof_approval'}
