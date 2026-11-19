"""End-to-end local input/editor checks; no SDK or hardware process is started."""
import copy,json,os,socket,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
os.environ['QT_QPA_PLATFORM']='offscreen'
from PyQt5 import QtWidgets as W
from grasp_profiles import Presets
from grasp_profiles_tests import packet
from operator_tactile import decode_tactile,fresh_tactile
from operator_panel import Panel
from operator_grasp import hardware_grasp_readiness

def tactile(stamp=100):
    raw=bytearray(192);raw[:2]=b'\x40\x06'
    return dict(received_monotonic=stamp,sequence=8,raw_hex=raw.hex(),fingers=[dict(normal_n=[.4,.8],tangent_n=[.2,0],direction_deg=[65535,45]) for _ in range(5)])

class Tactile(unittest.TestCase):
    def test_unknown_direction_and_stale_samples_never_draw(self):
        d=decode_tactile(tactile(),100.01);self.assertIsNone(d['fingers'][0]['direction_deg'][0]);self.assertEqual(d['fingers'][0]['direction_deg'][1],45);self.assertFalse(fresh_tactile(d,100.09));self.assertFalse(fresh_tactile(d,99))
        for stamp,previous in ((99,0),(100,8),(100.1,0)):
            with self.assertRaises(ValueError):decode_tactile(tactile(stamp),100.01,previous)
    def test_missing_invalid_force_and_wrong_frame_rejected(self):
        for key,value in [('normal_n',[float('nan'),0]),('normal_n',[-1,0]),('tangent_n',[]),('direction_deg',[])]:
            d=tactile();d['fingers'][0][key]=value
            with self.assertRaises(ValueError):decode_tactile(d,100.01)
        d=tactile();d['raw_hex']=bytes(192).hex()
        with self.assertRaises(ValueError):decode_tactile(d,100.01)

class Integration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=W.QApplication.instance() or W.QApplication([])
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.preset=Presets(Path(self.tmp.name)/'presets.json')
        with patch('operator_grasp.Presets',return_value=self.preset):self.panel=Panel(preview=True)
    def tearDown(self):self.panel.close();self.tmp.cleanup()
    def test_three_ui_slots_save_and_load(self):
        p=self.panel
        for i in range(3):
            p.grasp_choice.setCurrentIndex(i);p.load_grasp();p.grasp_endpoints[0][1].setValue(2200+i*100);p.grasp_force.setValue(1+i);p.save_grasp()
        restored=Presets(self.preset.path)
        self.assertEqual([x['closed'][0] for x in restored.data['profiles']],[2200,2300,2400]);self.assertEqual([x['normal_force_limit_n'] for x in restored.data['profiles']],[1,2,3])
        p.grasp_choice.setCurrentIndex(0);p.load_grasp();self.assertEqual(p.grasp_endpoints[0][1].value(),2200);self.assertEqual(p.nav[2].text(),'检查与验收');self.assertEqual(p.nav[4].text(),'灵巧手抓握')
    def test_udp_trigger_targets_stop_and_expiry(self):
        p=self.panel;p.grasp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);p.grasp_socket.bind(('127.0.0.1',0));p.grasp_socket.setblocking(False);port=p.grasp_socket.getsockname()[1];p.tick_grasp();self.assertIsNotNone(p.grasp_socket)
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as tx:
            for i,t in enumerate((0,.5,1)):
                tx.sendto(json.dumps(packet(i+1,time.monotonic(),t)).encode(),('127.0.0.1',port));time.sleep(.002);p.tick_grasp()
                self.assertEqual(p.grasp_progress.value(),round(t*100));self.assertEqual(p.grasp_endpoints[0][2].text(),str(round(1500*t)))
            self.assertIsNone(p.grasp.process)
            tx.sendto(json.dumps(packet(4,time.monotonic(),1,True)).encode(),('127.0.0.1',port));time.sleep(.002);p.tick_grasp();self.assertEqual(p.grasp_endpoints[0][2].text(),'—')
            p.reset_grasp_input();tx.sendto(json.dumps(packet(5,time.monotonic(),0)).encode(),('127.0.0.1',port));time.sleep(.002);p.tick_grasp();time.sleep(.09);p.tick_grasp();self.assertEqual(p.grasp_endpoints[0][2].text(),'—')
    def test_hardware_requires_motion_and_real_tactile_evidence(self):
        root=Path(self.tmp.name);(root/'artifacts').mkdir()
        motion=root/'artifacts/grasp_acceptance_live.json';tactile=root/'artifacts/grasp_monitor_live.json'
        self.assertFalse(hardware_grasp_readiness(root)[0])
        motion.write_text(json.dumps(dict(hardware=True,motion_passed=True)))
        tactile.write_text(json.dumps(dict(hardware=True,tactile_response_observed=False)))
        self.assertFalse(hardware_grasp_readiness(root)[0])
        tactile.write_text(json.dumps(dict(hardware=False,tactile_response_observed=True)))
        self.assertFalse(hardware_grasp_readiness(root)[0])
        tactile.write_text(json.dumps(dict(hardware=True,tactile_response_observed=True)))
        self.assertTrue(hardware_grasp_readiness(root)[0])
    def test_preview_never_starts_hardware(self):
        with patch.object(self.panel.grasp,'start') as start:self.panel.start_grasp('control');start.assert_not_called()

if __name__=='__main__':unittest.main()
