"""真实运行的离线演示，终端输出并导出轨迹。Ctrl+C 或 STOP.request 一键停止。"""
import time,json
from pathlib import Path
import numpy as np
from .configuration import load,ROOT
from .filters import CommandFilter
from .trajectory import OnlinePlanner
from .runtime import Deadline,JsonLogger

def run_demo(out):
    cfg=load(); out=Path(out); out.mkdir(parents=True,exist_ok=True)
    q=(np.array(cfg['limits']['lower_rad'])+cfg['limits']['upper_rad'])/2
    planner=OnlinePlanner(q,cfg['limits'],cfg['period_s']); filt=CommandFilter(cfg['filters'],cfg['period_s'],q)
    logger=JsonLogger(out/'demo.jsonl'); timer=Deadline(cfg['period_s']); stop=ROOT/'logs/STOP.request'
    started=time.time(); rows=[]; reason='completed'; late_count=0
    tuning=ROOT/'config/runtime_tuning.json'; tuning_mtime=None
    print('OFFLINE MODEL ONLY; robot SDK not loaded. Ctrl+C / launch/stop.cmd stops demo.',flush=True)
    print('FILTER_CONFIG='+json.dumps(cfg['filters']),flush=True)
    try:
        logger.emit(event='startup',scope='offline_demo',config=cfg)
        for i in range(1250):
            now,late=timer.wait()
            if stop.exists() and stop.stat().st_mtime>=started: reason='operator_stop'; break
            if i%25==0 and tuning.exists() and tuning.stat().st_mtime_ns!=tuning_mtime:
                tuning_mtime=tuning.stat().st_mtime_ns
                try:
                    new=json.loads(tuning.read_text(encoding='utf-8'))
                    if set(new)!={'command_hz'}: raise ValueError('only command_hz may be hot adjusted')
                    filt.tune_cutoff(float(new['command_hz']))
                    logger.emit(event='software_filter_tuned',command_hz=filt.lowpass.hz,scope='offline_demo')
                except (ValueError,TypeError) as e: logger.emit(event='tuning_rejected',reason=str(e))
            late_count+=late>cfg['safety']['max_lateness_s']
            desired=q+np.sin(i*.008*.8)*.4*np.array([1,.4,-.6,.2,.5,-.3]); target=filt.update(desired)
            if i%8==0: planner.retarget(target,.5)
            m=planner.step(); row={'event':'sample','time':now,'q':m.q.tolist(),'v':m.v.tolist(),'a':m.a.tolist(),'j':m.j.tolist(),'late_s':late}
            rows.append([i*.008,*m.q,*m.v,*m.a,*m.j]); logger.emit(**row)
            if i%250==0: print(f't={i*.008:.1f}s  J1={np.degrees(m.q[0]):.2f} deg  late={late*1000:.3f} ms',flush=True)
    except KeyboardInterrupt: reason='keyboard_stop'
    finally:
        logger.emit(event='stopped',reason=reason,scope='offline_demo',hardware_commands=0,deadline_misses=int(late_count)); logger.close()
        np.savez_compressed(out/'demo_trace.npz',trace=np.array(rows))
    print(f'{reason}; {len(rows)} samples; result={out}; hardware commands=0',flush=True)
    return 0
