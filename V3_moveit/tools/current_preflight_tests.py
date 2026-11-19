"""Filesystem gate regressions; synthetic data, no SDK/ROS execution."""
import copy
from datetime import datetime,timezone,timedelta
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import hardware_current_preflight as m

class CurrentPreflightGateTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=m.ROOT/'.tmp');self.root=Path(self.temp.name)
        self.paths=m.INPUTS+m.SOURCES+['raw/readonly','raw/mode1','raw/mode0','bin/read','bin/one','bin/zero']
        for relative in self.paths:
            p=self.root/relative;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(relative)
        self.record=dict(measurement_checks_passed=True,package_sha256='digest',acquisition_repeated=True,
          robot_connections_started=3,servo_enable_calls=0,motion_commands=0,physical_acceptance=False,
          completed_utc=datetime.now(timezone.utc).isoformat(),captures=dict(readonly='raw/readonly',mode1='raw/mode1',mode0='raw/mode0'),
          binaries=['bin/read','bin/one','bin/zero'],readonly={'checked':True},edg_feedback={'checked':True},
          evidence=[dict(path=p,sha256=m.sha(self.root/p)) for p in self.paths])
    def tearDown(self):self.temp.cleanup()
    def validate(self):
        report=self.root/'record.json';report.write_text(json.dumps(self.record))
        with patch.object(m,'ROOT',self.root),patch.object(m,'REPORT',report),patch.object(m,'analyze',return_value=({'checked':True},{'checked':True})),patch.object(m,'imports',return_value=[]):
            return m.validate(self.root,'digest')
    def test_matching_fresh_evidence_passes(self):self.assertTrue(self.validate()['measurement_checks_passed'])
    def test_modified_raw_or_model_rejected(self):
        for relative in ['raw/mode0','src/v3_moveit/description/s5.urdf']:
            p=self.root/relative;s=p.read_text();p.write_text('changed')
            with self.assertRaises(RuntimeError):self.validate()
            p.write_text(s)
    def test_missing_identity_rejected(self):
        self.record['evidence']=self.record['evidence'][1:]
        with self.assertRaises(RuntimeError):self.validate()
    def test_stale_future_or_version_mismatch_rejected(self):
        for delta in (-3601,10):
            self.record['completed_utc']=(datetime.now(timezone.utc)+timedelta(seconds=delta)).isoformat()
            with self.assertRaises(RuntimeError):self.validate()
        self.record['completed_utc']=datetime.now(timezone.utc).isoformat();self.record['package_sha256']='old'
        with self.assertRaises(RuntimeError):self.validate()
    def test_failed_partial_or_motion_capture_rejected(self):
        original=copy.deepcopy(self.record)
        for key,value in [('measurement_checks_passed',False),('robot_connections_started',2),('servo_enable_calls',1),('motion_commands',1),('physical_acceptance',True),('acquisition_repeated',False)]:
            self.record=copy.deepcopy(original);self.record[key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):self.validate()
    def test_saved_analysis_cannot_substitute_for_raw_reanalysis(self):
        self.record['readonly']={'checked':False}
        with self.assertRaises(RuntimeError):self.validate()

if __name__=='__main__':unittest.main(verbosity=2)
