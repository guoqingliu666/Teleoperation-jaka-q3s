import json,socket,tempfile,time,unittest
from pathlib import Path
from v2_servo.configuration import ROOT
from v2_servo.teleop.shadow_telemetry import ShadowTelemetry
from v2_servo.teleop.twin_publish import TwinPublisher

class ShadowTelemetryTests(unittest.TestCase):
    def test_slow_status_files_do_not_hold_up_joint_udp(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'data',prefix='slow_ui_') as d, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as listener:
            listener.bind(('127.0.0.1',0));listener.settimeout(.5)
            p=TwinPublisher((listener.getsockname()[1],));t=ShadowTelemetry(Path(d));original=t.writer.update
            def slow(*a,**k):time.sleep(.3);return original(*a,**k)
            t.writer.update=slow
            try:
                t.offer({'test':1},'slow file check');start=time.perf_counter();count=0
                for i in range(20):
                    p.offer(dict(mode='simulation',state='TEST',connected=True,q=[i*.001]*6,feedback_age_ms=0))
                    packet=json.loads(listener.recv(10000));self.assertAlmostEqual(packet['q_rad'][0],i*.001);count+=1
                self.assertEqual(count,20);self.assertLess(time.perf_counter()-start,.25)
            finally:p.close();t.close()
            self.assertIsNone(t.error);self.assertEqual(json.loads((Path(d)/'status.json').read_text())['test'],1)
