"""H10有限实验前置：复用H8响应和H9稳定记录；覆盖幅度仅作描述。"""
import json,datetime,time,re
from pathlib import Path
import numpy as np
from ..configuration import ROOT
from .expansion_evidence import expansion_hashes,verify_h8_baseline
from .rotation_evidence import report_path
from .xyz_evidence import sha,SHARED
from ..runtime_paths import same_sdk_artifacts

def workspace_hashes():
    result=expansion_hashes()
    for f in ('src/v2_servo/teleop/local_nonlinear.py','src/v2_servo/teleop/workspace_evidence.py','config/quest_pose_workspace.json'):
        result[f]=sha(ROOT/f)
    return result

def verify_h9_stable(path,sdk):
    path,r=report_path(path,'h9_');meta=json.loads(path.with_name('metadata.json').read_text(encoding='utf-8'))
    if (r.get('scope')!='live_H9_Quest_combined_6dof_supervised_expansion_only' or r.get('experiment')!='H9' or
        r.get('failure') or r.get('cleanup_errors') or r.get('servo_exited') is not True or not r.get('physical_stationary_observed') or
        r.get('servo_commands',0)<30 or r.get('moving_release_count',0)<2 or r.get('logout')!=[0] or
        meta.get('scope')!='live_H9_supervised_expansion_commissioning' or not meta.get('site_confirmed') or
        meta.get('robot_serial_user_confirmed')!='S5160217' or not same_sdk_artifacts(meta.get('sdk_artifacts_sha256'),sdk) or
        meta.get('sdk_binding')!='native-edg0' or r.get('edg_mode')!=0):
        raise ValueError('H10 requires stable actual H9 commissioning, not SIM/failed session')
    for f in SHARED+('src/v2_servo/filters.py','src/v2_servo/teleop/quest_pose_live.py','.deps/ruckig.cp312-win_amd64.pyd'):
        if meta.get('sha256',{}).get(f)!=sha(ROOT/f):raise ValueError('stable H9 common core changed: '+f)
    rows=r['samples']
    if len(rows)!=r['servo_commands'] or any((s.get('quest_intent') or {}).get('offline_replay') for s in rows):raise ValueError('H9 sample provenance')
    if max(np.max(np.abs(np.array(s['q_target'])-s['q_actual'])) for s in rows)>np.deg2rad(.05):raise ValueError('H9 raw gap')
    if r.get('tracking_governor_final',{}).get('slow_count',0)<1:raise ValueError('H9 governor not observed')
    h1=json.loads(path.with_name('h1_report.json').read_text(encoding='utf-8'))
    if not h1.get('passed_static_only') or h1.get('failure'):raise ValueError('H9 H1 missing')
    covered=all(np.all(np.ptp([s[key] for s in rows],axis=0)>=minimum) for key,minimum in
        (('TCP_xyz_actual_delta_mm',.2),('TCP_xyz_command_estimate_mm',.2),
         ('TCP_rotation_actual_deg',.01),('TCP_rotation_command_deg',.01)))
    # 原H9达到完整覆盖后会提前安全结束；不能让正常提前结束反而挡住完成流程。
    full=bool(r.get('passed_quest_expansion_finite_only') and covered)
    if not h9_window_eligible(r.get('servo_commands',0),r.get('hold_elapsed_s',0),full):
        raise ValueError('Partial H9 evidence needs >=29s/3000 sends; short session must pass full raw six-axis coverage')
    return dict(path=str(path),sha256=sha(path),scope='stable_H9_partial_axis_coverage_not_H10_approval',
        H9_full_six_coverage_passed=full)


def h9_window_eligible(sends,elapsed,full_verified):
    return bool(sends>=30 and (full_verified or (sends>=3000 and elapsed>=29)))


def coverage_union(records):
    """逐轴复用能力证据，不拼接不同起点的坐标，也不重判单会话为通过。"""
    axes=('X','Y','Z','RX','RY','RZ');limits=np.array([.2]*3+[.01]*3)
    sources={};progress={}
    for i,axis in enumerate(axes):
        best=None;ratio=0.
        for record in records:
            c=np.asarray(record['command_spans'],dtype=float);a=np.asarray(record['actual_spans'],dtype=float)
            if c.shape!=(6,) or a.shape!=(6,) or not np.isfinite(np.r_[c,a]).all() or np.any(np.r_[c,a]<0):
                raise ValueError('H9 coverage nonfinite/invalid span')
            value=min(c[i],a[i])/limits[i]
            if value>=ratio:ratio=float(value);best=record
        progress[axis]=dict(required=float(limits[i]),unit='mm' if i<3 else 'deg',ratio=ratio,
            command=0. if best is None else float(best['command_spans'][i]),
            actual=0. if best is None else float(best['actual_spans'][i]))
        if ratio>=1.:sources[axis]=dict(path=best['path'],sha256=best['sha256'])
    return dict(scope='H9_axis_capability_union_not_single_session_pass_or_H10_live_acceptance',
        H9_stage_coverage_passed=bool(records and len(sources)==6),missing_axes=[a for a in axes if a not in sources],
        axis_sources=sources,progress=progress,records=records,full_workspace_approved=False)


def h9_coverage_audit(paths,sdk):
    """仅审核同一V2/S5/SDK/滤波/输入/档位下24h内安全结束的真实会话。"""
    records=[];rejected=[];expected=json.loads((ROOT/'config/quest_pose_expansion.json').read_text(encoding='utf-8'))
    for path in sorted(set(map(str,paths)),reverse=True):
        try:
            p=Path(path)
            capability=verify_h9_stable(p,sdk)
            created=datetime.datetime.strptime(p.parent.name[3:],'%Y%m%d_%H%M%S_%f').replace(tzinfo=datetime.timezone(datetime.timedelta(hours=8))).timestamp()
            if not 0<=time.time()-created<=86400:raise ValueError('H9 coverage evidence outside 24h')
            r=json.loads(p.read_text(encoding='utf-8'));model=r.get('official_local_tcp_xyz',{})
            if model.get('profile')!=expected:raise ValueError('H9 coverage profile differs')
            command=np.r_[np.ptp([s['TCP_xyz_command_estimate_mm'] for s in r['samples']],axis=0),
                          np.ptp([s['TCP_rotation_command_deg'] for s in r['samples']],axis=0)]
            actual=np.r_[np.ptp([s['TCP_xyz_actual_delta_mm'] for s in r['samples']],axis=0),
                         np.ptp([s['TCP_rotation_actual_deg'] for s in r['samples']],axis=0)]
            records.append(dict(**capability,command_spans=command.tolist(),actual_spans=actual.tolist(),
                metadata_sha256=sha(p.with_name('metadata.json')),h1_sha256=sha(p.with_name('h1_report.json'))))
        except (ValueError,KeyError,OSError) as exc:rejected.append(dict(path=str(path),reason=str(exc)))
    result=coverage_union(records);result['rejected']=rejected
    return result


def workspace_entry_audit(h9_paths,h8_paths,sdk):
    """文件预检，不授予运动权限，也不重判旧会话。

    H8已证明六轴响应；最近H9证明实际闭环/释放/清理。H9的0.2mm
    跨度是调试覆盖描述，不是厂商安全条件。不得绕过最近一次H9故障
    回退到更老的成功记录；启动时仍须新H1、当前Tool1和官方模型校验。
    """
    paths=sorted({str(p) for p in h9_paths if re.fullmatch(r'h9_\d{8}_\d{6}_\d+',Path(p).parent.name)},reverse=True)
    result=dict(scope='H10_file_readiness_not_live_acceptance',
        ready_for_site_confirmation=False,coverage_is_diagnostic_only=True,
        automatic_axis_retest=False,robot_commands=0,full_workspace_approved=False,
        continuous_teleop_approved=False,rejected=[])
    if not paths:
        result['rejected'].append(dict(reason='No actual H9 report'))
        return result
    latest=Path(paths[0])
    try:
        stable=verify_h9_stable(latest,sdk)
        meta=json.loads(latest.with_name('metadata.json').read_text(encoding='utf-8'))
        # UI/监督器提示的修改不等于控制内核变化。下列运动/保护内核必须一致。
        for f in ('src/v2_servo/teleop/static_hold.py','src/v2_servo/teleop/pose_expansion.py',
                  'src/v2_servo/teleop/quest_pose_shadow.py','src/v2_servo/teleop/quest_tcp_xyz.py',
                  'src/v2_servo/teleop/live_rotation_input.py','src/v2_servo/trajectory.py',
                  'src/v2_servo/safety.py','src/v2_servo/configuration.py','config/site_observed_20260928.json'):
            if meta.get('sha256',{}).get(f)!=sha(ROOT/f):
                raise ValueError('H9 motion/safety core changed: '+f)
        if meta.get('session')!=latest.parent.name:raise ValueError('H9 metadata session mismatch')
        h1=json.loads(latest.with_name('h1_report.json').read_text(encoding='utf-8'))
        if h1.get('cleanup_errors'):raise ValueError('H9 H1 cleanup failed')
        coverage=h9_coverage_audit([latest],sdk)
        if not coverage['records']:raise ValueError('Latest H9 profile/provenance invalid: '+str(coverage['rejected']))
        result.update(h9_stable=stable,coverage=coverage)
    except (ValueError,KeyError,OSError,TypeError) as exc:
        result['rejected'].append(dict(path=str(latest),reason=str(exc)))
        return result
    for candidate in sorted(set(map(str,h8_paths)),reverse=True):
        try:
            result['h8_six_axis_response']=verify_h8_baseline(candidate,sdk)
            result['ready_for_site_confirmation']=True
            break
        except (ValueError,KeyError,OSError,TypeError) as exc:
            result['rejected'].append(dict(path=candidate,reason=str(exc)))
    if 'h8_six_axis_response' not in result:
        result['rejected'].append(dict(reason='No verified actual H8 six-axis response'))
    return result
