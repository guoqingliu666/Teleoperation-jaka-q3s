"""先打时间戳、在线配对、异步落盘；没有插值或缺失值补零。"""
import csv, hashlib, json, queue, threading, time, uuid
from pathlib import Path
import numpy as np

STREAMS = ('s5_state','hand_state','arm_action','hand_action','zed_left','zed_right','zed_depth','scene')
FIELDS = ('stream','seq','host_ns','start_ns','end_ns','source_ns','clock','uncertainty_ns','received_ns','mapping_bracket_ns','path')

def dump(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')

def six(value):
    a=np.asarray(value,dtype=float)
    if a.shape!=(6,) or not np.isfinite(a).all(): raise ValueError('expected six finite values')
    return a

def action_vector(arm, hand):
    a,h=six(arm),six(hand)
    if np.any(h<0) or np.any(h>10000): raise ValueError('hand position outside vendor equivalent 0..10000')
    return np.r_[a,h].tolist()

def decode_action(value):
    a=np.asarray(value,float)
    if a.shape!=(12,): raise ValueError('ACT action must have 12 values')
    action_vector(a[:6],a[6:])
    return {'q_target_rad':a[:6].tolist(),'position_target_eq':a[6:].tolist()}

def event(stream, seq, payload, *, host_ns=None, start_ns=None, end_ns=None,
          source_ns=None, clock='host_receive_only', uncertainty_ns=None):
    now=time.perf_counter_ns()
    if stream not in STREAMS: raise ValueError(stream)
    return dict(stream=stream,seq=int(seq),host_ns=now if host_ns is None else int(host_ns),
        start_ns=now if start_ns is None else int(start_ns),end_ns=now if end_ns is None else int(end_ns),
        source_ns=source_ns,clock=clock,uncertainty_ns=uncertainty_ns,payload=payload)

class Episode:
    def __init__(self, root, metadata, queue_size=64):
        self.id=str(uuid.uuid4()); self.path=Path(root)/self.id; self.path.mkdir(parents=True,exist_ok=False)
        for d in ('images','states','actions'): (self.path/d).mkdir()
        self.meta=dict(metadata,uuid=self.id,schema='v2_act_raw_v1',created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            clock='PC time.perf_counter_ns / Windows QPC',complete=False,success=None)
        dump(self.path/'metadata.json',self.meta)
        self.q=queue.Queue(queue_size);self.dropped=0;self.errors=[];self.counts={};self.closed=False
        self.thread=threading.Thread(target=self._write,name='ACT-disk-writer',daemon=True); self.thread.start()

    def offer(self,e):
        if self.closed: raise RuntimeError('episode closed')
        # 调用者拥有数组时必须交出副本，不能在提交后修改。
        try:self.q.put_nowait(e);return True
        except queue.Full:self.dropped+=1;return False

    def _write(self):
        try:self._write_loop()
        except Exception as exc:
            # 连 timestamps.csv 都无法打开时也要消费结束信号；不让 close 永久阻塞。
            self.errors.append('writer startup/fatal: '+repr(exc))
            while True:
                e=self.q.get();self.q.task_done()
                if e is None:return

    def _write_loop(self):
        from PIL import Image
        with (self.path/'timestamps.csv').open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=FIELDS);writer.writeheader()
            while True:
                e=self.q.get()
                try:
                    if e is None: return
                    s=e['stream']; payload=e['payload']; row={k:e.get(k) for k in FIELDS}
                    if s in ('zed_left','zed_right','zed_depth','scene'):
                        folder=self.path/'images'/s;folder.mkdir(exist_ok=True)
                        p=folder/(f"{e['seq']:08d}"+('.npy' if s=='zed_depth' else '.png'))
                        if s=='zed_depth':np.save(p,payload)
                        else:Image.fromarray(payload).save(p)
                        if s=='scene' and not (self.path/'start_scene.png').exists():Image.fromarray(payload).save(self.path/'start_scene.png')
                    else:
                        p=self.path/('actions' if s.endswith('action') else 'states')/(s+'.jsonl')
                        with p.open('a',encoding='utf-8') as state:
                            state.write(json.dumps({**{k:e[k] for k in e if k!='payload'},'data':payload},allow_nan=False)+'\n')
                    row['path']=p.relative_to(self.path).as_posix();writer.writerow(row);f.flush()
                    self.counts[s]=self.counts.get(s,0)+1
                except Exception as exc:self.errors.append(repr(exc))
                finally:self.q.task_done()

    def close(self, **extra):
        if self.closed:return
        self.closed=True;self.q.put(None);self.thread.join()
        self.meta.update(extra,counts=self.counts,dropped=self.dropped,writer_errors=self.errors,
                         complete=not self.errors and self.dropped==0)
        dump(self.path/'metadata.json',self.meta)
        hashes={p.relative_to(self.path).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                for p in self.path.rglob('*') if p.is_file()}
        dump(self.path/'sha256.json',hashes)

class OnlineMatcher:
    """只匹配已实际接收的帧。拒绝迟到/重复帧；不推测未来、不插值。"""
    def __init__(self,path,target_ns=5_000_000):
        self.path=Path(path);self.target=target_ns;self.latest={};self.used={};self.lock=threading.Lock()
    def push(self,e):
        with self.lock:
            old=self.latest.get(e['stream'])
            if old and (e['seq']<=old['seq'] or e['host_ns']<old['host_ns']):return False
            self.latest[e['stream']]={k:v for k,v in e.items() if k!='payload'}
            return True
    def match(self,reference_ns):
        with self.lock:
            values=dict(self.latest);missing=[s for s in STREAMS if s not in values]
            errors={s:abs(e['host_ns']-reference_ns) for s,e in values.items()}
            uncertainty_known=all(e['uncertainty_ns'] is not None for e in values.values())
            worst=max(errors.values(),default=0)
            conservative=max((errors[s]+(e['uncertainty_ns'] or 0) for s,e in values.items()),default=0)
            span=(max((e['host_ns']+(e['uncertainty_ns'] or 0) for e in values.values()),default=0)-
                  min((e['host_ns']-(e['uncertainty_ns'] or 0) for e in values.values()),default=0))
            unique=all(e['seq']>self.used.get(s,-1) for s,e in values.items())
            accepted=not missing and unique and uncertainty_known and conservative<self.target and span<self.target
            if accepted:self.used={s:e['seq'] for s,e in values.items()}
            row={'reference_ns':reference_ns,'refs':{s:e['seq'] for s,e in values.items()},'missing':missing,
                 'offset_ns':errors,'max_offset_ns':worst,'conservative_offset_ns':conservative,
                 'pairwise_span_ns':span,
                 'uncertainty_known':uncertainty_known,'unique_samples':unique,'accepted':accepted}
            with self.path.open('a',encoding='utf-8') as f:f.write(json.dumps(row)+'\n')
            return row

def validate(path):
    path=Path(path);m=json.loads((path/'metadata.json').read_text(encoding='utf-8'))
    rows=list(csv.DictReader((path/'timestamps.csv').open(encoding='utf-8')))
    issues=[];seen={};last={}
    if not m['complete']:issues.append('writer incomplete/drop/error')
    for r in rows:
        s=r['stream'];seq=int(r['seq']);t=int(r['host_ns'])
        if seq<=seen.get(s,-1) or t<last.get(s,0):issues.append('nonmonotonic '+s)
        seen[s]=seq;last[s]=t
        if not (path/r['path']).is_file():issues.append('missing '+r['path'])
    for s in STREAMS:
        if s not in seen:issues.append('missing stream '+s)
    for folder in ('states','actions'):
        for p in (path/folder).glob('*.jsonl'):
            for line in p.read_text().splitlines():
                d=json.loads(line)['data']
                key={'s5_state':'q_rad','hand_state':'position_eq','arm_action':'q_target_rad','hand_action':'position_target_eq'}[p.stem]
                try:six(d[key])
                except (KeyError,ValueError):issues.append('invalid vector '+p.stem)
    matches=[json.loads(l) for l in (path/'alignment.jsonl').read_text().splitlines()] if (path/'alignment.jsonl').exists() else []
    accepted=sum(x['accepted'] for x in matches)
    ids={(r['stream'],int(r['seq'])) for r in rows}
    for match in matches:
        if match['accepted'] and (set(match['refs'])!=set(STREAMS) or any((s,i) not in ids for s,i in match['refs'].items())):
            issues.append('alignment references missing samples')
    if not accepted or accepted!=len(matches):issues.append('alignment not accepted for all frames')
    hashes=json.loads((path/'sha256.json').read_text())
    for name,digest in hashes.items():
        p=path/name
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:issues.append('hash mismatch '+name)
    report={'episode':str(path),'stage':m['stage'],'rows':len(rows),'streams':m['counts'],'matched':len(matches),
        'accepted':accepted,'max_offset_ms':max(x['max_offset_ns'] for x in matches)/1e6 if matches else None,
        'max_pairwise_span_ms':max(x.get('pairwise_span_ns',0) for x in matches)/1e6 if matches else None,
        'simulation_pipeline_pass':not issues and m['stage']=='A','physical_sync_verified':False,
        'training_ready':False,'issues':sorted(set(issues)),
        'reason':'Simulation validates snapshot correspondence only; live exposure/sample timing and site gates remain required.'}
    dump(path/'validation.json',report);return report
