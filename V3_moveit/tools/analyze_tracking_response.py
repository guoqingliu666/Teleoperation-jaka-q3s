import json,sys
from pathlib import Path
import numpy as np
p=Path(sys.argv[1]); r=json.loads((p/'result.json').read_text()); rows=[json.loads(s) for s in (p/'capture.jsonl').read_text().splitlines()]
phases={x['name']:x for x in r['phases']}; ramp=phases['ramp_5cm']; stop=phases['hold_5s']['end_host_s']; start=ramp['start_host_s']; anchor=np.array(r['anchor']['tcp_m'])
tf=[]; targets=[]; seen=set()
for x in rows:
 if x.get('kind')!='message':continue
 t=x['host_monotonic_ns']*1e-9
 if not start<=t<=stop:continue
 m=x['message']
 if x['topic']=='/diagnostic/tcp_tf':
  stamp=(m['header']['stamp']['sec'],m['header']['stamp']['nanosec'])
  if stamp in seen:continue
  seen.add(stamp); tr=m['transform']['translation']; tf.append([t,*[tr[k] for k in ('x','y','z')]])
 elif x['topic']=='/v3/hand_target':
  tr=m['pose']['position']; targets.append([t,*[tr[k] for k in ('x','y','z')]])
a=np.array(tf); b=np.array(targets)
if len(a)<10 or len(b)<10:raise ValueError('Insufficient trajectory capture')
expected=np.column_stack([np.interp(a[:,0],b[:,0],b[:,j]) for j in (1,2,3)])
e=np.linalg.norm(expected-a[:,1:4],axis=1); idx=a[:,0]<=ramp['end_host_s']; axis=1+'xyz'.index(r.get('input_axis','x')); delta=a[:,axis]-anchor[axis-1]
out={'source':str(p),'mode':'fake','physical_acceptance':False,'approximate_observer_receipt_clock':True,'ramp_tracking_error_mm':{'rms':float(np.sqrt(np.mean(e[idx]**2))*1000),'maximum':float(e[idx].max()*1000)},'final_error_mm':float(e[-1]*1000),'milestone_delay_s':{}}
for mm in (10,20,40):
 at=np.flatnonzero(delta>=mm/1000); bt=np.flatnonzero(b[:,axis]-anchor[axis-1]>=mm/1000)
 out['milestone_delay_s'][str(mm)+'mm']=float(a[at[0],0]-b[bt[0],0]) if len(at) and len(bt) else None
settle=None
for i in range(len(a)):
 if a[i,0]>=ramp['end_host_s'] and np.all(e[i:]<=.001):settle=float(a[i,0]-ramp['end_host_s']);break
out['settle_within_1mm_after_ramp_s']=settle
print(json.dumps(out,indent=2))
(p/'tracking_analysis.json').write_text(json.dumps(out,indent=2)+'\n')
