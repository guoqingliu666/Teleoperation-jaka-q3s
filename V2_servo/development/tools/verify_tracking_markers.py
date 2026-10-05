"""Actual built Player + Python UDP + independent matrix/visibility checks. No SDK."""
import copy,datetime,json,os,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'.deps'),str(ROOT/'scripts'),str(ROOT)]
sys.dont_write_bytecode=True
for n in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[n]='1'
import numpy as np
from v2_servo.configuration import load
from v2_servo.teleop.continuous_pose import ContinuousPoseServo
from v2_servo.teleop.continuous_shadow import load_model
from v2_servo.teleop.tracking_observation import shadow_tracking
from v2_servo.teleop.twin_publish import TwinPublisher
from v2_servo.teleop.official_local import rpy_matrix
from v2_servo.kinematics import rotate
from process_util import close_own_player

def main():
    running=subprocess.run(['tasklist','/FI','IMAGENAME eq V2Twin.exe','/FO','CSV','/NH'],capture_output=True)
    if running.returncode or b'V2Twin.exe' in running.stdout:raise RuntimeError('Existing Player: no takeover')
    out=ROOT/'data/tracking_markers_20261004'/datetime.datetime.now().strftime('%H%M%S');out.mkdir()
    model,q,_=load_model();profile=json.loads((ROOT/'config/continuous.json').read_text())
    c=ContinuousPoseServo(q,load(),model,profile,np.eye(3));t=model.forward(q)[0]
    c.goal=t.copy();c.goal[:3,3]+=[.08,.06,.02];c.goal[:3,:3]=rotate(np.array([0.,0.,1.]),.6)@t[:3,:3]
    c.held=True;c.phase='FOLLOW';c.servo.sigma=.04
    frame=dict(tracking_valid=True,stamp=0);tracking=shadow_tracking(c,t,frame,0,'marker-probe-current-tool')
    valid=dict(mode='simulation',state='MARKER_BOUND',connected=True,active=True,q=q,feedback_age_ms=0,tracking=tracking)
    states=[('MARKER_BOUND',valid),('MARKER_RELEASE',dict(valid,active=False,tracking=dict(tracking,target_valid=False,target_T=None))),
        ('MARKER_WRONG_FRAME',dict(valid,tracking=dict(tracking,frame='quest_tracking'))),
        ('MARKER_FAKE_LIVE',dict(valid,mode='live')),
        ('MARKER_INVALID_MATRIX',dict(valid,tracking=dict(tracking,feedback_T=[0.]*16))),
        ('MARKER_REBOUND',copy.deepcopy(valid)),
        ('MARKER_NO_TRACKING',dict(valid,tracking=None)),('MARKER_AGAIN',copy.deepcopy(valid))]
    env=os.environ.copy();env.update(TEMP='D:/ChatGPT/Temp',TMP='D:/ChatGPT/Temp',V2_TWIN_DATA_DIR=str(out),V2_TRACKING_MARKER_PROBE=str(out))
    player=ROOT/'digital_twin/unity/Build/V2Twin.exe'
    p=subprocess.Popen([str(player),'-screen-fullscreen','0','-screen-width','1280','-screen-height','800','-logFile',str(out/'player.log')],cwd=ROOT,env=env)
    pub=TwinPublisher();report=dict(scope='actual_Unity_markers_synthetic_UDP_not_robot',robot_commands=0,headset_visual_checked=False)
    try:
        time.sleep(4)
        for label,s in states:
            s['state']=label;start=time.perf_counter();observed=False
            while time.perf_counter()-start<8:
                pub.offer(s);time.sleep(.008)
                try:
                    observed=any(row['phase']==label for row in json.loads((out/'markers.json').read_text())['samples'])
                except (OSError,ValueError):pass
                if observed and time.perf_counter()-start>.7:break
            if not observed:raise RuntimeError('Player did not apply '+label)
        time.sleep(.6)
    finally:pub.close();report['exit_code']=close_own_player(p)
    recorded=json.loads((out/'markers.json').read_text())['samples'];samples={s['phase']:s for s in recorded}
    cfg=json.loads((ROOT/'digital_twin/unity/Assets/V2Twin/Data/twin_unity.json').read_text())
    vec=lambda d:np.array([d[k] for k in ('x','y','z')]);B=np.array([[1,0,0],[0,0,1],[0,1,0]])
    R=rpy_matrix([0,0,0,*vec(cfg['base_rpy_cad_rad'])]);origin=vec(cfg['base_position_cad_m'])
    def expected(T):
        p=B@(origin+R@T[:3,3]);axes=[B@(origin+R@(T[:3,3]+T[:3,i]*.075)) for i in range(3)];return p,axes
    errors=[]
    for name,T in [('feedback',t),('target',c.goal)]:
        pos,axes=expected(T);s=samples['MARKER_BOUND']
        errors.append(float(np.linalg.norm(vec(s[name+'_world'])-pos)))
        errors.extend(float(np.linalg.norm(vec(a)-b)) for a,b in zip(s[name+'_axes'],axes))
    checks={'bound':samples['MARKER_BOUND']['target_visible'] and samples['MARKER_BOUND']['feedback_visible'],
        'release':samples['MARKER_RELEASE']['feedback_visible'] and not samples['MARKER_RELEASE']['target_visible'],
        'rebound':samples['MARKER_REBOUND']['target_visible'],
        'stale':not samples['MARKER_AGAIN_STALE']['target_visible'] and not samples['MARKER_AGAIN_STALE']['feedback_visible']}
    for phase in ('MARKER_WRONG_FRAME','MARKER_FAKE_LIVE','MARKER_INVALID_MATRIX','MARKER_NO_TRACKING'):
        checks[phase]=not samples[phase]['feedback_visible'] and not samples[phase]['target_visible']
    checks['transform_and_axes']=max(errors)<2e-6
    bound=samples['MARKER_BOUND'];p2=vec(bound['tcp2_world']);p1=vec(bound['feedback_world'])
    z2=(vec(bound['tcp2_axes'][2])-p2)/.075
    checks['tcp2_visible']=bound['tcp2_visible']
    checks['tcp1_200mm_along_tcp2_z']=float(np.linalg.norm((p1-p2)-.2*z2))<2e-6
    checks['tcp2_hides_with_stale_feedback']=not samples['MARKER_AGAIN_STALE']['tcp2_visible']
    report.update(checks=checks,max_coordinate_error_m=max(errors),passed=all(checks.values()) and report['exit_code']==0)
    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(dict(out=str(out),**report)),flush=True)
    return 0 if report['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
