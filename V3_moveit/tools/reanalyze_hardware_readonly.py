#!/usr/bin/env python3
"""Reanalyze preserved diagnostic logs only; never load the SDK or start ROS.

Original reports remain unchanged. The output distinguishes original collection
provenance from the current analysis and verifies raw/model identities first.
"""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from verify_offline import fk, source_digest

ROOT=Path(__file__).resolve().parents[1]
STAMP_LIMIT=.024
SAFETY_LIMIT=.15


def require(condition, reason):
    if not condition:raise RuntimeError(reason)


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def evidence(relative):
    path=(ROOT/relative).resolve()
    require(path.is_relative_to(ROOT) and path.is_file(),f'Invalid evidence path {relative}')
    return dict(path=str(path.relative_to(ROOT)),sha256=sha(path))


def load_log(relative):
    records=[];banners=[]
    for number,line in enumerate((ROOT/relative).read_text().splitlines(),1):
        try:value=json.loads(line)
        except ValueError:
            require(line=='receiver port 30003',f'Unknown non-JSON line {relative}:{number}')
            banners.append(number);continue
        require(isinstance(value,dict),f'Non-object record {relative}:{number}')
        require('error' not in value,f'Probe error recorded at {relative}:{number}')
        records.append(value)
    require(len(banners)<=1,'Unexpected repeated SDK receiver banner')
    return records,banners


def finite_vector(values,size):
    return isinstance(values,list) and len(values)==size and all(isinstance(v,(int,float)) and math.isfinite(v) for v in values)


def stats(values):
    a=np.asarray(values,dtype=float)
    require(len(a)>0 and np.isfinite(a).all(),'Missing/nonfinite timing values')
    return dict(count=len(a),min=float(a.min()),p50=float(np.percentile(a,50)),
                p99=float(np.percentile(a,99)),max=float(a.max()))


def verify_historical_identities(relative):
    original_path=ROOT/relative
    original=json.loads(original_path.read_text())
    require(original.get('measurement_checks_passed') is True,'Historical analysis was not passed')
    require(original.get('evidence'),'Historical evidence list missing')
    mismatches=[]
    for item in original['evidence']:
        current=evidence(item['path'])
        if current['sha256']!=item['sha256']:
            # This analyzer recomputes all checks below with the CURRENT pure FK
            # helper. A changed analysis tool is recorded; changed readings,
            # probe code/binary or model/TCP/load are never silently accepted.
            require(item['path']=='tools/verify_offline.py',f'Historical source evidence changed: {item["path"]}')
            mismatches.append(dict(path=item['path'],historical_sha256=item['sha256'],current_sha256=current['sha256']))
    backup=ROOT/'artifacts/backups'/(original_path.stem+'.pre_reanalysis_20261112.json')
    backup.parent.mkdir(parents=True,exist_ok=True)
    if not backup.exists():shutil.copyfile(original_path,backup)
    require(sha(backup)==sha(original_path),'Historical report backup differs')
    return original,dict(original_report=evidence(relative),preserved_backup=evidence(str(backup.relative_to(ROOT))),
        original_report_generated_at_utc=original.get('generated_at_utc'),
        original_analysis_tool_changes=mismatches,
        acquisition_repeated=False,new_robot_connections=0,new_sdk_calls=0,
        acquisition_wall_time_note='Host steady-clock logs have no host UTC acquisition timestamp. Original report time is retained; EDG source timestamp range below is the cabinet clock, not synchronized to the host.')


def analyze_edg(records,mode):
    samples=[r for r in records if r.get('sample') is True]
    polls=[r for r in records if r.get('safety_poll') is True]
    clean=[r for r in records if r.get('cleanup') is True]
    connects=[r for r in records if r.get('query')=='create_handler']
    starts=[r for r in records if r.get('query')=='edg_init_feedback_only']
    require(len(connects)==1 and connects[0]['rc']==0,'EDG connection was not recorded as successful')
    require(len(starts)==1 and starts[0].get('mode')==mode and starts[0].get('rc')==0,'EDG mode initialization not successful')
    require(len(samples)>1000 and len(polls)>100 and len(clean)==1,'Incomplete diagnostic capture')
    cleanup=clean[0]
    require(all(cleanup.get(k)==0 for k in ('edg_off_rc','servo_query_rc','servo_active','disconnect_rc','motion_commands','servo_enable_calls')),'SDK cleanup/motion counts not clean')
    accepted=[];last_source=None;last_fresh_host=None;last_host_end=None;last_after=None
    source_intervals=[];host_intervals=[];ages=[];duplicate=0;incoherent=0;all_q=[];all_v=[]
    for r in samples:
        require(r.get('rc')==[0,0,0],'EDG query return code failed')
        require(finite_vector(r.get('q'),6) and finite_vector(r.get('v'),6),'Nonfinite/incomplete EDG position or velocity')
        start,end=r['host_start'],r['host_end']
        require(math.isfinite(start) and math.isfinite(end) and end>=start and (last_host_end is None or start>=last_host_end),'Host sample timestamps invalid/regressed')
        last_host_end=end
        before,after=r['before'],r['after']
        require(len(before)==3 and len(after)==3 and all(type(v) is int and v>=0 for v in before+after),'Invalid EDG timestamp tuple')
        require(after[1]<10**9,'EDG nanoseconds out of range')
        source=after[0]*10**9+after[1]
        require(last_after is None or source>=last_after,'EDG source timestamp regressed')
        last_after=source
        if last_fresh_host is not None:ages.append(end-last_fresh_host)
        all_q.append(r['q']);all_v.append(r['v'])
        if before!=after:incoherent+=1;continue
        if last_source is not None and source==last_source:duplicate+=1;continue
        require(last_source is None or source>last_source,'Accepted source timestamp regressed')
        if last_source is not None:
            source_intervals.append((source-last_source)*1e-9);host_intervals.append(end-last_fresh_host)
        accepted.append((source,end,r['q'],r['v']))
        last_source=source;last_fresh_host=end
    require(len(accepted)>1000,'Too few coherent new source frames')
    duration=samples[-1]['host_end']-samples[0]['host_start']
    require(11.9<=duration<=12.2,'Diagnostic capture duration differs from 12-second observation')
    require(0<min(source_intervals) and max(source_intervals)<STAMP_LIMIT,'Source-frame interval exceeded 24 ms')
    require(0<min(host_intervals) and max(host_intervals)<STAMP_LIMIT,'New-frame host receipt interval exceeded 24 ms')
    require(min(ages)>=0 and max(ages)<STAMP_LIMIT,'Observed previous accepted frame age exceeded 24 ms')
    health={'motion_rc':0,'simple_rc':0,'estop':0,'collision':0,'limit':0,'errcode':0,'powered_on':1,'enabled':1}
    for i,p in enumerate(polls):
        require(all(p.get(k)==v for k,v in health.items()),'Safety poll return code or robot readiness failed')
        require(math.isfinite(p['start']) and math.isfinite(p['end']) and p['end']>=p['start'],'Invalid safety poll timestamps')
        require(i==0 or p['start']>=polls[i-1]['end'],'Safety poll sequence regressed')
    safety_age=[p['end']-p['start'] for p in polls]+[b['end']-a['start'] for a,b in zip(polls,polls[1:])]
    safety_age.append(max(0.,samples[-1]['host_end']-polls[-1]['start']))
    require(max(safety_age)<SAFETY_LIMIT,'Observed safety anchor age exceeded 150 ms')
    q=np.asarray(all_q);v=np.asarray(all_v);drift=float(np.max(np.abs(q-q[0])))
    require(drift<=.001 and float(np.max(np.abs(v)))<=.005,'Historical capture was not within stationary takeover thresholds')
    source_clock=dict(first_sec=accepted[0][0]//10**9,first_nsec=accepted[0][0]%10**9,
                      last_sec=accepted[-1][0]//10**9,last_nsec=accepted[-1][0]%10**9,
                      first_cabinet_time_utc=datetime.fromtimestamp(accepted[0][0]*1e-9,timezone.utc).isoformat(),
                      last_cabinet_time_utc=datetime.fromtimestamp(accepted[-1][0]*1e-9,timezone.utc).isoformat(),
                      synchronized_with_host=False)
    summary=dict(mode=mode,raw_samples=len(samples),accepted_new_frames=len(accepted),incoherent_discarded=incoherent,
        duplicate_not_freshened=duplicate,host_observation_duration_s=duration,
        accepted_source_interval_s=stats(source_intervals),accepted_host_interval_s=stats(host_intervals),
        previous_frame_observed_age_s=stats(ages),safety_poll_count=len(polls),safety_observed_anchor_age_s=stats(safety_age),
        maximum_drift_from_first_rad=drift,peak_to_peak_rad=np.ptp(q,axis=0).tolist(),
        maximum_reported_velocity_rad_s=float(np.max(np.abs(v))),source_clock=source_clock,
        freshness_threshold_s=STAMP_LIMIT,safety_threshold_s=SAFETY_LIMIT)
    return summary,cleanup


def pose_values(transform):
    return np.r_[transform[:3,3]*1000,Rotation.from_matrix(transform[:3,:3]).as_euler('xyz')]


def compare_pose(local,observed):
    require(finite_vector(list(local),6) and finite_vector(observed,6),'Nonfinite FK/TCP data')
    delta=np.asarray(local)-observed;delta[3:]=np.arctan2(np.sin(delta[3:]),np.cos(delta[3:]))
    passed=bool(np.max(np.abs(delta[:3]))<=.1 and np.max(np.abs(delta[3:]))<=.001)
    require(passed,'Current URDF/metadata does not match historical official FK')
    return dict(position_component_delta_mm=delta[:3].tolist(),wrapped_rpy_delta_rad=delta[3:].tolist(),
                passed=passed,position_guard_mm=.1,rotation_guard_rad=.001)


def analyze_readonly(records):
    data={r['data']:r for r in records if 'data' in r}
    queries=[r for r in records if 'query' in r];summaries=[r for r in records if r.get('summary')]
    require(len(queries)==18 and all(r['rc']==0 for r in queries),'Read-only queries incomplete or failed')
    require(len(summaries)==1 and all(summaries[0].get(k)==0 for k in ('query_failures','motion_commands','servo_enable_calls','edg_enable_calls')),'Read-only session summary not clean')
    s=data['robot_status'];m=data['motion_status'];owner=data['ownership']
    require(s['errcode']==0 and s['powered_on']==1 and s['enabled']==1,'Historical robot status not ready')
    require(all(m[k]==0 for k in ('estop','collision','on_limit','queue','active_queue','paused')) and m['inpos']==1,'Historical motion/queue status not ready')
    require(owner['servo_active']==0 and owner['in_position']==1 and data['drag_mode']['value']==0 and data['operating_mode']['program_state']==0,'Historical owner/drag/program status not idle')
    runtime=yaml.safe_load((ROOT/'src/v3_moveit/config/runtime.yaml').read_text())
    payload=json.loads((ROOT/'src/v3_moveit/config/payload.json').read_text())
    meta=json.loads((ROOT/'src/v3_moveit/config/model_metadata.json').read_text())
    from v3_moveit.tool_config import tool_values,apply_tool
    tool,expected_tcp=tool_values(payload)
    tcp=data.get('tool_tcp_mm_rad',data.get('tcp1_mm_rad'))['values'];load=data['payload_kg_cog_mm']['values']
    require(data['tool_id']['value']==tool,'TOOL_MISMATCH: cabinet tool differs from V3 selection')
    require(finite_vector(tcp,6) and all(abs(a-b)<=.001 for a,b in zip(tcp,expected_tcp)),'TCP_MISMATCH: cabinet TCP differs from V3 definition')
    require(finite_vector(load,4) and abs(load[0]-payload['payload_kg'])<=1e-6,'PAYLOAD_MISMATCH: cabinet load differs from V3 configuration')
    require(all(abs(a-b)<=.001 for a,b in zip(load[1:],payload['cog_mm'])),'PAYLOAD_CENTROID_MISMATCH: cabinet centroid differs from V3 configuration')
    require(1<=data['collision_level']['value']<=5,'Historical collision protection disabled')
    robot=ET.parse(ROOT/'src/v3_moveit/description/s5.urdf').getroot()
    robot,meta=apply_tool(robot,meta,payload)
    q=data['joints_rad']['values'];qafter=data['joints_after_rad']['values']
    require(finite_vector(q,6) and finite_vector(qafter,6) and np.max(np.abs(np.array(q)-qafter))<=.001,'Historical joint observation not stationary')
    reference=pose_values(fk(robot,meta['q_reference_rad']));actual=pose_values(fk(robot,q))
    comparisons=dict(reference_urdf=compare_pose(reference,data['reference_fk_mm_rad']['values']),
        reference_metadata=compare_pose(pose_values(np.asarray(meta['tcp_home'])),data['reference_fk_mm_rad']['values']),
        measured_q_urdf_vs_fk=compare_pose(actual,data['actual_fk_mm_rad']['values']),
        measured_q_urdf_vs_tcp=compare_pose(actual,data['actual_tcp_mm_rad']['values']),
        official_fk_vs_tcp=compare_pose(data['actual_fk_mm_rad']['values'],data['actual_tcp_mm_rad']['values']))
    return dict(query_count=len(queries),robot_status=s,motion_status=m,ownership=owner,
                tcp1_mm_rad=tcp,payload_kg_cog_mm=load,collision_level=data['collision_level']['value'],
                fk_comparisons=comparisons,
                reference_q_provenance='The original report identified model_metadata q_reference_rad as the diagnostic CLI input; raw log did not echo argv. Current metadata identity is unchanged.',
                host_acquisition_wall_time_utc=None)


def base_report(original,provenance,relatives):
    now=datetime.now(timezone.utc).isoformat()
    return dict(schema_version=2,generated_at_utc=now,scope='Offline reanalysis of preserved historical readings; no new acquisition',
        reanalysis=True,provenance=provenance,package_sha256=source_digest(),measurement_checks_passed=True,
        analysis_connected_hardware=False,analysis_started_ros=False,robot_connections=0,sdk_calls=0,
        hardware_motion_accepted=False,physical_stopping_accepted=False,physical_acceptance=False,
        teleoperation_accepted=False,absolute_latency_upper_bound_proven=False,
        evidence=[evidence(p) for p in dict.fromkeys(relatives+['tools/reanalyze_hardware_readonly.py','tools/verify_offline.py'])],
        analyzer=dict(path='tools/reanalyze_hardware_readonly.py',sha256=sha(Path(__file__)),generated_at_utc=now),
        limitations=['This is reanalysis of the same historical data, not a new current-state measurement.',
            'Cabinet EDG timestamps and host steady clock were not synchronized; no absolute latency bound is established.',
            'Small measured drift and zero reported velocity do not prove exactly zero physical movement.',
            'No servo enable, EDG motion transmission, physical braking or Quest hardware operation was measured.',
            'Live JakaSystem preflight must repeat model/TCP/payload/status and fresh stationary feedback before servo entry.'])


def run():
    originals=['artifacts/hardware_readonly_validation.json','artifacts/edg_mode0_validation.json']
    histories=[verify_historical_identities(p) for p in originals]
    readonly,_=load_log('artifacts/hardware_readonly_raw.log')
    mode1,banners1=load_log('artifacts/edg_feedback_only.jsonl')
    mode0,banners0=load_log('artifacts/edg_mode0_no_motion.jsonl')
    observed=analyze_readonly(readonly);edg1,cleanup1=analyze_edg(mode1,1);edg0,cleanup0=analyze_edg(mode0,0)
    identity=['src/v3_moveit/config/runtime.yaml','src/v3_moveit/config/payload.json',
              'src/v3_moveit/config/model_metadata.json','src/v3_moveit/description/s5.urdf']
    paths1=[originals[0],'artifacts/hardware_readonly_raw.log','artifacts/edg_feedback_only.jsonl',
            'artifacts/hardware_preflight_readonly.cpp','artifacts/edg_feedback_only.cpp']+identity
    report1=base_report(*histories[0],paths1)
    report1.update(readonly=observed,edg_feedback=edg1,cleanup=cleanup1,
        checks={k:True for k in ('historical_raw_probe_model_identities_unchanged','readonly_query_rc_all_zero',
            'initial_robot_ready_and_stationary_status','tool_id_matches','tcp1_exact_float_value_match',
            'payload_exact_float_value_match','all_fk_comparisons_within_existing_guard',
            'collision_protection_level_valid','mode1_fresh_feedback_within_existing_24ms',
            'mode1_safety_polls_within_existing_150ms','cleanup_edg_off_servo_false_disconnect_succeeded')})
    report1['provenance']['source_capture_time']=edg1['source_clock']
    paths0=[originals[1],'artifacts/edg_mode0_no_motion.jsonl','artifacts/edg_mode0_prepared_NOT_RUN.cpp',
            'artifacts/edg_mode0_prepared_NOT_RUN']+identity
    report0=base_report(*histories[1],paths0)
    imports=subprocess.check_output(['nm','-D','--undefined-only',str(ROOT/'artifacts/edg_mode0_prepared_NOT_RUN')],text=True)
    api_names=set(re.findall(r'DLLEXPORT_API\s+errno_t\s+(\w+)\s*\(', (ROOT/'vendor/jaka/include/jakaAPI.h').read_text()))
    used=sorted(set(line.split()[-1].split('@')[0] for line in imports.splitlines() if line.split())&api_names)
    allowed={'create_handler','destory_handler','edg_get_stat','edg_init_extend','edg_stat_details',
             'get_actual_joint_position','get_motion_status','get_program_state','get_robot_status_simple',
             'is_in_drag_mode','is_in_servomove'}
    require(set(used)==allowed,'Mode0 binary SDK imports changed or include an unexpected operation')
    report0.update(mode0_feedback_measurement_passed=True,
        execution=dict(mode=0,actually_executed=True,acquisition_repeated=False,completed_cleanup_logged=True,
                       historical_executable='artifacts/edg_mode0_prepared_NOT_RUN',filename_note='Historical filename retained; raw successful mode0 init and cleanup establish the old run occurred.'),
        edg_feedback=edg0,cleanup=cleanup0,static_probe_review=dict(sdk_imports=used,forbidden_sdk_references=[]),
        checks={k:True for k in ('historical_raw_probe_model_identities_unchanged','recorded_create_handler_and_mode0_init_rc_zero',
            'mode0_samples_finite_and_fresh_within_existing_24ms','mode0_safety_polls_within_existing_150ms',
            'cleanup_edg_off_servo_false_disconnect_succeeded','motion_and_servo_enable_counts_zero',
            'binary_sdk_imports_match_recorded_feedback_only_probe')})
    report0['provenance']['source_capture_time']=edg0['source_clock']
    outputs=[('artifacts/hardware_readonly_reanalysis.json',report1),('artifacts/edg_mode0_reanalysis.json',report0)]
    for relative,report in outputs:
        (ROOT/relative).write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(path=relative,measurement_checks_passed=True,raw_capture_repeated=False,
                             accepted_frames=report['edg_feedback']['accepted_new_frames'],sha256=sha(ROOT/relative))))


if __name__=='__main__':run()
