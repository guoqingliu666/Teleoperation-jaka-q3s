import datetime,json,os,subprocess,sys,tempfile,unittest
from unittest.mock import patch
from pathlib import Path
from v2_servo.configuration import ROOT
from v2_servo.teleop.rotation_evidence import verify_rotation_input,verify_h6_baseline,verify_rotation_baseline


class RotationEvidenceTests(unittest.TestCase):
    def setUp(self):
        # 历史原始文件只作门槛解析夹具，不随着日历变化冒充当前现场证据。
        instant=datetime.datetime(2026,10,3,16,tzinfo=datetime.timezone(datetime.timedelta(hours=8))).timestamp()
        self.clock=patch('v2_servo.teleop.rotation_evidence.time.time',return_value=instant);self.clock.start();self.addCleanup(self.clock.stop)
    def test_actual_input_and_h6_capability_provenance(self):
        p=ROOT/'data/quest_rotation_shadow_20261003_145038_013301/report.json'
        self.assertIn('not_real_rotation_motion',verify_rotation_input(p)['scope'])
        base=ROOT/'data/h6_20261003_130933_206345/report.json'
        hashes=json.loads(base.with_name('metadata.json').read_text())['sdk_artifacts_sha256']
        self.assertIn('not_H7',verify_h6_baseline(base,hashes)['scope'])
        with self.assertRaisesRegex(ValueError,'matching robot/SDK'):verify_h6_baseline(base,{})
    def test_offline_or_failed_or_active_rejection_cannot_be_motion_gate(self):
        actual=ROOT/'data/quest_rotation_shadow_20261003_145038_013301/report.json'
        for key,value in [('scope','offline_UDP_rotation_shadow_process_no_SDK_not_hardware_evidence'),('active_rejected',1),('completed_axes',['RX']),('normal_releases',0)]:
            with tempfile.TemporaryDirectory(dir=ROOT/'data') as temporary:
                folder=Path(temporary)/actual.parent.name;folder.mkdir();record=json.loads(actual.read_text());record[key]=value
                p=folder/'report.json';p.write_text(json.dumps(record),encoding='utf8')
                with self.assertRaisesRegex(ValueError,'real passed three-axis'):verify_rotation_input(p)
    def test_cli_missing_current_site_rejected_before_sdk(self):
        env=os.environ.copy();env.update(TEMP='D:/ChatGPT/Temp',TMP='D:/ChatGPT/Temp',PYTHONDONTWRITEBYTECODE='1')
        r=subprocess.run([sys.executable,str(ROOT/'scripts/h1_static_hold.py'),'--live-quest-tcp-rotation','--quest-input-mode','rotation_shadow'],cwd=ROOT,env=env,capture_output=True,text=True)
        self.assertEqual(r.returncode,2);self.assertIn('current on-site readiness',r.stderr)
    def test_RZ_cannot_reuse_RX_as_RY_and_existing_RX_provenance_valid(self):
        p=ROOT/'data/h7_20261003_152448_150509/report.json';hashes=json.loads(p.with_name('metadata.json').read_text())['sdk_artifacts_sha256']
        self.assertEqual(verify_rotation_baseline(p,'RX',hashes)['axis'],'RX')
        with self.assertRaisesRegex(ValueError,'real RY'):verify_rotation_baseline(p,'RY',hashes)
    def test_axis_label_cannot_disagree_with_official_model_or_metadata(self):
        original=ROOT/'data/h7_20261003_152448_150509/report.json'
        with tempfile.TemporaryDirectory(dir=ROOT/'data') as temporary:
            folder=Path(temporary)/original.parent.name;folder.mkdir()
            record=json.loads(original.read_text());record['rotation_axis']='RY';record['scope']='live_H7_Quest_Tool1_base_RY_local_0p02deg_only'
            meta=json.loads(original.with_name('metadata.json').read_text());meta['rotation_axis']='RY'
            p=folder/'report.json';p.write_text(json.dumps(record));p.with_name('metadata.json').write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError,'real RY'):verify_rotation_baseline(p,'RY',meta['sdk_artifacts_sha256'])
