import json,subprocess,sys,tempfile,unittest,hashlib
from pathlib import Path
from v2_servo.configuration import ROOT,load

class P1RevisionTests(unittest.TestCase):
    def test_hardware_presence_does_not_grant_readback_or_motion(self):
        c=json.loads((ROOT/'config/dh116.json').read_text(encoding='utf-8'))
        self.assertEqual(c['hardware_model'],'DH116EC-R1-M');self.assertTrue(c['tactile_present'])
        self.assertFalse(c['tactile_readback_verified']);self.assertFalse(c['motion_approved'])
        self.assertIsNone(c['force_target_N_api']);self.assertIsNone(c['tactile_normal_N_calibration'])
    def test_default_is_single_command_lpf(self):
        c=load()['filters'];self.assertTrue(c['enabled']);self.assertEqual(c['moving_average'],1);self.assertIsNone(c['notch_hz'])
    def test_annotation_keeps_raw_hashes_unchanged(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'data',prefix='p1_test_') as d:
            ep=Path(d)/'raw_episode';ep.mkdir();(ep/'metadata.json').write_text('{"success":null}',encoding='utf-8')
            (ep/'sha256.json').write_text('{}');before={p.name:p.read_bytes() for p in ep.iterdir()}
            r=subprocess.run([sys.executable,str(ROOT/'scripts/annotate_act_episode.py'),str(ep),'--outcome','failure','--operator','synthetic-test','--note','offline fixture'],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr);self.assertEqual(before,{p.name:p.read_bytes() for p in ep.iterdir()})
            files=list((Path(d)/'annotations/raw_episode').glob('*.json'));self.assertEqual(len(files),1)
            record=json.loads(files[0].read_text(encoding='utf-8'));self.assertFalse(record['raw_episode_modified'])
            self.assertEqual(record['raw_metadata_sha256'],hashlib.sha256(before['metadata.json']).hexdigest())
