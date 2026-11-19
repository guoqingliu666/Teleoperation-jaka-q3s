#!/usr/bin/env python3
"""Fresh status/TCP/load/FK and two bounded EDG feedback checks; no motion APIs.

Default validates a saved capture. --hardware explicitly runs the existing
reviewed query probes sequentially. EDG mode 0 exposes an interface capability,
but these binaries do not import servo-enable, power or motion-command APIs.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import yaml
from reanalyze_hardware_readonly import analyze_readonly, analyze_edg, load_log, require
from verify_offline import source_digest

ROOT=Path(__file__).resolve().parents[1]
REPORT=ROOT/'artifacts/hardware_preflight_current.json'
INPUTS=['src/v3_moveit/config/runtime.yaml','src/v3_moveit/config/payload.json',
        'src/v3_moveit/config/model_metadata.json','src/v3_moveit/description/s5.urdf',
        'site_ubuntu.yaml','vendor/jaka/lib/libjakaAPI.so','vendor/jaka/include/jakaAPI.h',
        'tools/hardware_current_preflight.py','tools/reanalyze_hardware_readonly.py','tools/verify_offline.py']
SOURCES=['artifacts/hardware_preflight_readonly.cpp','artifacts/edg_feedback_only.cpp',
         'artifacts/edg_mode0_prepared_NOT_RUN.cpp']
READONLY={'create_handler','destory_handler','get_robot_status_simple','get_motion_status',
          'is_in_servomove','is_in_pos','is_in_drag_mode','get_tool_id','get_tool_data',
          'get_payload','get_collision_level','get_auto_manual_mode','get_program_state',
          'get_actual_joint_position','get_actual_tcp_position','kine_forward'}
FEEDBACK={'create_handler','destory_handler','edg_get_stat','edg_init_extend','edg_stat_details',
          'get_actual_joint_position','get_motion_status','get_program_state',
          'get_robot_status_simple','is_in_drag_mode','is_in_servomove'}


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def identity(path):return {'path':str(path.relative_to(ROOT)),'sha256':sha(path)}
def write(report):REPORT.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')

def imports(binary,allowed):
    api=set(re.findall(r'DLLEXPORT_API\s+errno_t\s+(\w+)\s*\(', (ROOT/'vendor/jaka/include/jakaAPI.h').read_text()))
    output=subprocess.check_output(['nm','-D','--undefined-only',str(binary)],text=True)
    used={line.split()[-1].split('@')[0] for line in output.splitlines() if line.split()}&api
    require(used==allowed,'Unexpected/missing SDK imports: '+str(used^allowed))
    return sorted(used)

def analyze(record):
    captures=record['captures']
    readonly,_=load_log(captures['readonly']);observed=analyze_readonly(readonly)
    observed['reference_q_provenance']='Current model reference q explicitly recorded in acquisition argv.'
    observed['host_acquisition_wall_time_utc']=record['started_utc']
    feedback={}
    for mode in (1,0):
        records,_=load_log(captures[f'mode{mode}'])
        summary,cleanup=analyze_edg(records,mode)
        feedback[str(mode)]={'summary':summary,'cleanup':cleanup}
    return observed,feedback

def validate(root,digest):
    require(root.resolve()==ROOT.resolve(),'Unexpected workspace for current preflight')
    record=json.loads(REPORT.read_text())
    require(record.get('measurement_checks_passed') is True and not record.get('failure'),'Fresh hardware preflight not passed')
    require(record.get('package_sha256')==digest,'Fresh hardware preflight source version mismatch')
    require(record.get('acquisition_repeated') is True and record.get('robot_connections_started')==3 and
            record.get('servo_enable_calls')==0 and record.get('motion_commands')==0 and
            record.get('physical_acceptance') is False,'Fresh preflight scope incomplete')
    age=(datetime.now(timezone.utc)-datetime.fromisoformat(record['completed_utc'])).total_seconds()
    require(0<=age<3600,'Fresh preflight is older than one hour or timestamp is in future; repeat acquisition')
    paths=set()
    for item in record['evidence']:
        path=(ROOT/item['path']).resolve()
        require(path.is_relative_to(ROOT) and path.is_file() and sha(path)==item['sha256'],'Fresh preflight identity changed: '+item['path'])
        paths.add(item['path'])
    require(set(INPUTS+SOURCES+list(record['captures'].values())+record['binaries']).issubset(paths),'Fresh preflight evidence missing')
    observed,feedback=analyze(record)
    require(observed==record['readonly'] and feedback==record['edg_feedback'],'Fresh analysis differs from saved results')
    for i,relative in enumerate(record['binaries']):imports(ROOT/relative,READONLY if i==0 else FEEDBACK)
    return record

def run():
    from hardware_hold_check import validate_site
    site=validate_site(ROOT);digest=source_digest();stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out=ROOT/'artifacts'/('hardware_preflight_'+stamp);out.mkdir()
    record={'scope':'Fresh sequential controller queries and 12 seconds each mode 1/mode 0 EDG feedback; no servo enable or motion commands',
            'started_utc':datetime.now(timezone.utc).isoformat(),'package_sha256':digest,'acquisition_repeated':True,
            'measurement_checks_passed':False,'robot_connections_started':0,'servo_enable_calls':0,'motion_commands':0,
            'physical_acceptance':False,'site':site,'captures':{},'binaries':[],'acquisition_argv':[],
            'limitations':['Measured intervals are observations, not absolute latency guarantees.',
                           'Small reported drift does not prove zero physical movement.',
                           'Live driver preflight and HOLD remain separate; no motion teleoperation acceptance.']}
    write(record)
    try:
        binaries=[]
        for i,source in enumerate(SOURCES):
            binary=out/('readonly' if i==0 else f'edg_mode{2-i}')
            subprocess.run(['g++','-std=c++17','-O2','-pthread','-I',str(ROOT/'vendor/jaka/include'),str(ROOT/source),
                            '-L',str(ROOT/'vendor/jaka/lib'),'-Wl,-rpath,'+str(ROOT/'vendor/jaka/lib'),'-ljakaAPI','-o',str(binary)],check=True)
            imports(binary,READONLY if i==0 else FEEDBACK);binaries.append(binary);record['binaries'].append(str(binary.relative_to(ROOT)))
        reference=json.loads((ROOT/'src/v3_moveit/config/model_metadata.json').read_text())['q_reference_rad']
        for i,binary in enumerate(binaries):
            key='readonly' if i==0 else f'mode{2-i}';capture=out/(key+'.jsonl')
            args=[str(binary),site['robot_ip']]+([str(x) for x in reference] if i==0 else [site['edg_client_ip']])
            record['captures'][key]=str(capture.relative_to(ROOT));record['acquisition_argv'].append(args)
            record['robot_connections_started']+=1;write(record)
            print('Starting '+key+' diagnostic (no servo enable or motion APIs)',flush=True)
            with capture.open('w') as stream:subprocess.run(args,stdout=stream,stderr=subprocess.STDOUT,check=True,cwd=ROOT)
            rows,_=load_log(str(capture.relative_to(ROOT)))
            if i==0:analyze_readonly(rows)
            else:analyze_edg(rows,2-i)
            print('Passed '+key,flush=True)
        require(source_digest()==digest,'Source changed during hardware acquisition')
        record['readonly'],record['edg_feedback']=analyze(record)
        record['completed_utc']=datetime.now(timezone.utc).isoformat()
        record['evidence']=[identity(ROOT/p) for p in INPUTS+SOURCES+list(record['captures'].values())+record['binaries']]
        record['measurement_checks_passed']=True
    except Exception as exc:
        record['failure']=str(exc);raise
    finally:
        write(record);(out/'result.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    validate(ROOT,digest)
    print('Fresh preflight passed; '+str(REPORT),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--hardware',action='store_true');args=parser.parse_args()
    if args.hardware:run()
    else:validate(ROOT,source_digest());print('Saved fresh preflight validated; no hardware calls.')
