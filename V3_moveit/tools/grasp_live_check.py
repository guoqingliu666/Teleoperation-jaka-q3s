"""Explicit hardware check; records actual feedback, never substitutes software results."""
import argparse,json,time
from PyQt5 import QtCore
from operator_grasp import GraspConnection
from grasp_profiles import defaults
from operator_settings import ROOT,atomic

p=argparse.ArgumentParser();p.add_argument('--hardware',action='store_true',required=True);p.add_argument('--mode',choices=['monitor','acceptance','acceptance_profile','open'],required=True);p.add_argument('--interface',default='enp11s0');p.add_argument('--seconds',type=float,default=90);p.add_argument('--current',type=int,default=200,choices=range(1,901));p.add_argument('--velocity',type=int,default=2000,choices=range(100,2001));p.add_argument('--closed',type=int,nargs=6,metavar='COUNT');args=p.parse_args()
if (args.mode=='acceptance_profile')!=(args.closed is not None):p.error('--closed is required only for acceptance_profile')
if args.closed is not None and any(x<0 or x>10000 for x in args.closed):p.error('--closed counts must be in 0..10000')
app=QtCore.QCoreApplication([]);hand=GraspConnection();profile=defaults()['profiles'][0];profile['current_per_mille']=args.current;profile['velocity']=args.velocity
if args.closed is not None:profile['closed']=args.closed
hand.start(args.interface,args.mode,profile)
start=time.monotonic();ready_at=0;last_detail='';last_sequence=0;peaks=[0.]*5;directions=[set() for _ in range(5)];nonzero=[0]*5;last_joint={}
report=dict(mode=args.mode,hardware=True,current_per_mille=args.current,velocity_counts_per_s=args.velocity,closed_targets=args.closed,physical_acceptance=False,force_limit_contact_verified=False)
while hand.process is not None:
    app.processEvents();now=time.monotonic()
    if hand.detail!=last_detail:print(hand.detail,flush=True);last_detail=hand.detail
    if getattr(hand,'ready',False) and not ready_at:ready_at=now;print('READY: motors disabled; tactile contact sampling' if args.mode=='monitor' else 'READY: bounded all-open recovery' if args.mode=='open' else 'READY: bounded full-fist sequence',flush=True)
    if hand.state:last_joint=dict(hand.state)
    if hand.tactile and hand.tactile['sequence']!=last_sequence:
        last_sequence=hand.tactile['sequence']
        for i,f in enumerate(hand.tactile['fingers']):
            value=sum(f['normal_n']);peaks[i]=max(peaks[i],value)
            if value>0:nonzero[i]+=1
            for t,d in zip(f['tangent_n'],f['direction_deg']):
                if t>0 and d is not None:directions[i].add(round(d))
    if ready_at and now-ready_at>=args.seconds:hand.stop()
    if not ready_at and now-start>180:hand.stop()
    time.sleep(.005)
report.update(motion_passed=hand.result.get('motion_passed') is True,open_passed=hand.result.get('open_passed') is True,cleanup=hand.result,tactile_response_observed=all(x>.1 for x in peaks) and all(x>=3 for x in nonzero),elapsed_s=time.monotonic()-start,detail=hand.detail,init_confirmed=hand.closed,normal_peak_n=peaks,nonzero_samples=nonzero,tangent_directions_deg=[sorted(x) for x in directions],last_joint=last_joint)
report['profile_motion_passed']=hand.result.get('profile_motion_passed') is True
atomic(ROOT/'artifacts'/('grasp_'+args.mode+'_live.json'),report);print(json.dumps(report,ensure_ascii=False),flush=True)
result_key={'open':'open_passed','acceptance_profile':'profile_motion_passed','acceptance':'motion_passed'}.get(args.mode)
raise SystemExit(0 if hand.closed and (args.mode=='monitor' or hand.result.get(result_key) is True) else 1)
