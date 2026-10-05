"""Export auditable old/new tool and J6-only comparisons. Never loads an SDK."""
import hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'.deps')]
import numpy as np
from v2_servo.teleop.continuous_shadow import load_model
from v2_servo.teleop.global_kinematics import rotation_log

def main():
    out=ROOT/'data/tcp_view_fix_20261004';out.mkdir(exist_ok=True)
    new,q,source=load_model();old,_,_=load_model(source);q=np.array(q)
    start=new.forward(q)[0];oldstart=old.forward(q)[0];rows=[]
    for a in np.linspace(-180,180,73):
        candidate=q.copy();candidate[5]+=np.deg2rad(a)
        tcp=new.forward(candidate)[0];previous=old.forward(candidate)[0];ref=new.tcp2_model.forward(candidate)[0]
        rows.append(dict(j6_delta_deg=float(a),tcp1_position_m=tcp[:3,3].tolist(),tcp2_position_m=ref[:3,3].tolist(),
            local_tcp1_from_tcp2_mm=(ref[:3,:3].T@(tcp[:3,3]-ref[:3,3])*1000).tolist(),
            corrected_position_change_mm=float(np.linalg.norm(tcp[:3,3]-start[:3,3])*1000),
            old_position_change_mm=float(np.linalg.norm(previous[:3,3]-oldstart[:3,3])*1000)))
    derived=dict(scope='user_tool_transform_derived_shadow_only_not_new_official_validation',
        source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        tool_provenance=new.tool_provenance,q_reference_rad=new.q0.tolist(),T_reference_m=new.home.tolist(),
        space_screws_v_w=new.screws.tolist(),robot_commands=0,physical_feedback=False)
    target=out/'derived_TCP1_model.json';target.write_text(json.dumps(derived,indent=2),encoding='utf-8')
    report=dict(scope='offline_coordinate_and_j6_sweep_only',robot_commands=0,controller_readback_verified=False,
        derived_model_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),samples=rows,
        old_new_position_difference_at_reference_mm=float(np.linalg.norm(start[:3,3]-oldstart[:3,3])*1000),
        old_new_orientation_difference_deg=float(np.rad2deg(np.linalg.norm(rotation_log(start[:3,:3]@oldstart[:3,:3].T)))),
        corrected_J6_max_position_change_mm=max(r['corrected_position_change_mm'] for r in rows),
        old_J6_max_position_change_mm=max(r['old_position_change_mm'] for r in rows))
    (out/'tool_math_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
if __name__=='__main__':main()
