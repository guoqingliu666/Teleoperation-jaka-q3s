"""隔离随机UDP端口，验证真实CLI进程/无追踪启动/手柄解码/规划/JSON日志闭环。"""
import copy,json,os,socket,subprocess,sys,threading,time,unittest
from pathlib import Path
import numpy as np
from v2_servo.configuration import ROOT


class RotationProcessTests(unittest.TestCase):
    def test_full_process_waits_for_tracking_then_records_numpy_intent(self):
        packet=None
        with (ROOT/'data/h6_20261003_130933_206345/events.jsonl').open(encoding='utf-8') as lines:
            for line in lines:
                row=json.loads(line)
                if row.get('event')=='Quest_raw_input':packet=row['raw'];break
        env=os.environ.copy();env.update(TEMP='D:/ChatGPT/Temp',TMP='D:/ChatGPT/Temp',PYTHONDONTWRITEBYTECODE='1')
        proc=subprocess.Popen([sys.executable,str(ROOT/'scripts/quest_rotation_shadow.py'),'--model',
             str(ROOT/'data/quest_tcp_readonly_20261003_131937_133610/report.json'),'--port','0','--offline-test',
             '--seconds','8','--ready-timeout','5'],cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding='utf-8')
        producer=None;stop=threading.Event()
        try:
            first=proc.stdout.readline();info=json.loads(first);port=info['listening_port'];folder=Path(info['session'])
            def produce():
                origin=time.perf_counter();sequence=0
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
                    while not stop.is_set():
                        elapsed=time.perf_counter()-origin;p=copy.deepcopy(packet);sequence+=1
                        p.update(sequence=sequence,unity_time_s=elapsed,offline_test=True)
                        block=max(0,int((elapsed-.5)/.85));phase=max(0,elapsed-.5)% .85
                        held=elapsed>.5 and .1<phase<.6
                        angle=min(5.,max(0.,(phase-.1)*18)) if held else 0.
                        axis=(2,0,1)[block%3];q=[0.,0.,0.,float(np.cos(np.deg2rad(angle)/2))];q[axis]=float(np.sin(np.deg2rad(angle)/2))
                        p['right'].update(connected=elapsed>.2,tracked=elapsed>.2,pose_valid=elapsed>.2,
                                          grip_available=elapsed>.2,
                                          grip=1. if held else 0.,thumbstick={'x':0.,'y':0.},thumbstick_click=False,
                                          rotation_xyzw=dict(zip(('x','y','z','w'),q)))
                        sock.sendto(json.dumps(p).encode(),('127.0.0.1',port));stop.wait(.012)
            producer=threading.Thread(target=produce,daemon=True);producer.start()
            stdout,stderr=proc.communicate(timeout=16)
            self.assertEqual(proc.returncode,0,stdout+stderr)
            report=json.loads((folder/'report.json').read_text(encoding='utf-8'))
            self.assertTrue(report['passed_input_and_shadow_only'],report);self.assertIsNone(report['failure'])
            self.assertEqual(report['completed_axes'],['RX','RY','RZ'])
            self.assertGreater(report['warmup_rejected'],0);self.assertEqual(report['active_rejected'],0)
            self.assertGreater(report['readiness_wait_s'],.25);self.assertEqual(report['robot_commands'],0)
            self.assertIn('offline_',report['scope']);self.assertFalse(report['real_rotation_motion_validated'])
            rows=[json.loads(line) for line in (folder/'samples.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertGreater(len(rows),200);self.assertTrue(all(isinstance(r['input']['axes'],list) for r in rows))
            self.assertTrue(all(not r['sent_to_robot'] for r in rows));self.assertTrue(rows[0]['input']['tracking_valid'])
            raw=[json.loads(line) for line in (folder/'raw_input.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertFalse(raw[0]['raw']['right']['tracked'])
        finally:
            stop.set()
            if producer is not None:producer.join(1)
            if proc.poll() is None:proc.terminate();proc.communicate(timeout=5)


if __name__=='__main__':unittest.main()
