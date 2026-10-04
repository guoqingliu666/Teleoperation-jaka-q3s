"""H6基础能力/方向演练只读核验；不连接SDK、不替代当前代码H1和现场READY。"""
import datetime,hashlib,json,re,time
from pathlib import Path
from ..configuration import ROOT


SHARED=('config/default.json','config/teleop.json','src/v2_servo/runtime.py','src/v2_servo/teleop/edg.py',
        'src/v2_servo/teleop/quest.py','src/v2_servo/teleop/quest_j1.py','src/v2_servo/jaka.py',
        'src/v2_servo/teleop/single_axis.py','src/v2_servo/teleop/native_sdk.py',
        'src/v2_servo/teleop/official_local.py','src/v2_servo/teleop/quest_tcp_z.py','src/v2_servo/teleop/quest_tcp_axis.py')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def verify_h5_baselines(paths,sdk_hashes):
    seen=set();result=[]
    for path in paths:
        path=Path(path).resolve()
        if not path.is_relative_to(ROOT/'data') or not path.is_file():raise ValueError('H6 needs V2 real H5 report')
        meta=json.loads(path.with_name('metadata.json').read_text(encoding='utf-8'));r=json.loads(path.read_text(encoding='utf-8'))
        session=meta.get('session','');axis=r.get('tcp_axis')
        try:created=datetime.datetime.strptime(session[3:],'%Y%m%d_%H%M%S_%f').replace(tzinfo=datetime.timezone(datetime.timedelta(hours=8))).timestamp()
        except ValueError:created=0
        if (not re.fullmatch(r'h5_\d{8}_\d{6}_\d+',session) or session!=path.parent.name or not 0<=time.time()-created<=86400
            or meta.get('requested_scope')!='H5' or axis not in ('X','Y') or meta.get('tcp_axis')!=axis
            or not meta.get('site_confirmed') or not r.get('passed_quest_tcp_axis_session_only')
            or r.get('failure') or r.get('cleanup_errors') or r.get('servo_exited') is not True
            or meta.get('robot_serial_user_confirmed')!='S5160217' or meta.get('sdk_artifacts_sha256')!=sdk_hashes):
            raise ValueError('H6 requires successful real H5 X and Y baselines within 24 hours')
        for f in SHARED:
            if meta.get('sha256',{}).get(f)!=sha(ROOT/f):raise ValueError('H5 shared code/filter/SDK changed: '+f)
        if axis in seen:raise ValueError('H6 duplicated axis baseline; both X and Y required')
        seen.add(axis);result.append({'path':str(path),'axis':axis,'sha256':sha(path)})
    if seen!={'X','Y'}:raise ValueError('H6 needs both real X and Y baselines')
    return result


def verify_xyz_input(path):
    path=Path(path).resolve()
    if not path.is_relative_to(ROOT/'data') or not path.is_file():raise ValueError('H6 needs new V2 XYZ input rehearsal')
    r=json.loads(path.read_text(encoding='utf-8'))
    try:created=datetime.datetime.strptime(path.parent.name[len('quest_input_'):],'%Y%m%d_%H%M%S_%f').replace(tzinfo=datetime.timezone(datetime.timedelta(hours=8))).timestamp()
    except ValueError:created=0
    if (not 0<=time.time()-created<=600 or r.get('scope')!='real_Unity_UDP_input_only_no_robot_SDK'
        or r.get('robot_commands')!=0 or not r.get('input_ready') or not r.get('rehearsal_passed') or r.get('failure')
        or r.get('input_mode')!='xyz' or r.get('axes_verified')!=['X','Y','Z']
        or r.get('xyz_profile_sha256')!=sha(ROOT/'config/quest_xyz_commissioning.json')
        or r.get('input_code_sha256')!=sha(ROOT/'src/v2_servo/teleop/quest_xyz.py')
        or r.get('player_assembly_sha256')!=sha(ROOT/'digital_twin/unity/Build/V2Twin_Data/Managed/Assembly-CSharp.dll')):
        raise ValueError('H6 XYZ direction rehearsal failed/stale/changed; no motion')
    return {'path':str(path),'sha256':sha(path),'scope':'direction_rehearsal_not_physical_frame_calibration'}
