import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from v2_servo import runtime_paths
from v2_servo.configuration import ROOT

class RuntimePathsTests(unittest.TestCase):
    def test_missing_local_override_does_not_use_historical_external_sdk(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'data',prefix='runtime_path_test_') as d:
            p=Path(d);(p/'config').mkdir();(p/'config/teleop.json').write_text(json.dumps({'sdk_directory':'F:/old/sdk','host':'192.168.110.224'}),encoding='utf-8')
            with patch.object(runtime_paths,'ROOT',p):
                r=runtime_paths.load_runtime_options()
            self.assertTrue(Path(r['sdk_directory']).is_relative_to(p));self.assertEqual(r['host'],'192.168.110.224')
    def test_escape_from_local_runtime_refused(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'data',prefix='runtime_path_test_') as d:
            p=Path(d);(p/'config/local').mkdir(parents=True);(p/'config/teleop.json').write_text('{}')
            (p/'config/local/runtime.json').write_text(json.dumps({'sdk_directory':'../other/sdk'}))
            with patch.object(runtime_paths,'ROOT',p),self.assertRaises(ValueError):runtime_paths.load_runtime_options()
    def test_same_sdk_rejects_missing_or_extra_artifact(self):
        d={n:'a'*64 for n in ('jakaAPI.dll','jakaAPI.h','jktypes.h')}
        for invalid in ({},dict(d,unrelated='a'*64),{'jakaAPI.dll':'a'*64}):
            self.assertFalse(runtime_paths.same_sdk_artifacts(d,invalid))
