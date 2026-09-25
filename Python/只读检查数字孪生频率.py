"""独立①读取链路实测频率。默认不连接；--live-readonly只读5秒，无运动API。"""
import argparse
import json
import os
from pathlib import Path
import statistics
import sys
import time
sys.path.insert(0,str(Path(__file__).parent/'src'))
from vla_lab.jaka_telemetry import SDK_DIRECTORY, load_sdk, ReadOnlySession
from vla_lab.jaka_vr_readonly_bridge import FastMeasuredReader, POLL_PERIOD_S


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live-readonly',action='store_true')
    args=parser.parse_args()
    if not args.live_readonly:
        print('默认不连接。使用 --live-readonly 测量5秒。'); return
    session=ReadOnlySession(load_sdk(SDK_DIRECTORY), os.environ.get('QUEST_JAKA_HOST', '127.0.0.1'))
    samples=[]; errors=[]; times=[]
    try:
        session.connect(); reader=FastMeasuredReader(); started=deadline=time.perf_counter()
        while time.perf_counter()-started<5:
            t=time.perf_counter()
            try:
                state=reader.read(session)
                times.append(time.perf_counter()); samples.append((time.perf_counter()-t)*1000)
            except Exception as e:
                errors.append(str(e))
            deadline+=POLL_PERIOD_S
            if deadline<time.perf_counter(): deadline=time.perf_counter()
            time.sleep(max(0,deadline-time.perf_counter()))
    finally:
        session.close()
    report={'samples':len(samples),'measured_hz':(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0,
            'read_ms_median':statistics.median(samples) if samples else None,
            'read_ms_max':max(samples,default=0),'max_gap_ms':max((1000*(b-a) for a,b in zip(times,times[1:])),default=0),
            'errors':errors,'movement_commands_sent':0,'note':'静态实测，未证明运动期间频率'}
    output=Path(__file__).resolve().parents[1]/'Validation'/'数字孪生独立反馈频率.json'
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2)); print(output)


if __name__=='__main__': main()
