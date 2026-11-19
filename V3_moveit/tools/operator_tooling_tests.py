"""Tool transforms, precision, stale writes and actionable diagnostics; no SDK."""
import copy
import hashlib
import json
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from operator_tooling import ROOT,apply_tool,tool_values,mismatch,recovery,save_readback
from v3_moveit.tool_config import pose_matrix,BASELINE_TCP
from verify_offline import fk

class Tooling(unittest.TestCase):
    def test_site_uses_the_wired_route_to_the_selected_cabinet(self):
        from hardware_hold_check import validate_site
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);net=root/'net';(net/'wired0/device').mkdir(parents=True)
            cfg=root/'src/v3_moveit/config';cfg.mkdir(parents=True)
            contents='robot_ip: 192.168.110.224\nedg_port: 30003\nedg_client_ip: '
            (cfg/'runtime.yaml').write_text(contents+'192.168.110.67\n')
            (root/'site_ubuntu.yaml').write_text(contents+'192.168.110.36\n')
            route=[dict(dev='wired0',prefsrc='192.168.110.67')]
            addresses=[dict(flags=['UP','LOWER_UP'],addr_info=[dict(local='192.168.110.67',scope='global')])]
            def lookup(args,**kwargs):return json.dumps(route if 'route' in args else addresses)
            with patch('hardware_hold_check.Path',return_value=net),patch('hardware_hold_check.subprocess.check_output',side_effect=lookup):
                with self.assertRaisesRegex(RuntimeError,'wired0 当前使用 192.168.110.67') as error:validate_site(root)
                self.assertEqual(recovery(str(error.exception))[0],'network')
                (root/'site_ubuntu.yaml').write_text(contents+'192.168.110.67\n')
                self.assertEqual(validate_site(root)['interface'],'wired0')
                route[0]['gateway']='192.168.110.1'
                with self.assertRaisesRegex(RuntimeError,'未找到直连'):validate_site(root)
    def test_load_refresh_keeps_software_identity_but_changes_calibration(self):
        import verify_offline as verification
        from unittest.mock import patch
        with tempfile.TemporaryDirectory(dir=ROOT/'.tmp') as d:
            root=Path(d);pkg=root/'src/v3_moveit';(pkg/'config').mkdir(parents=True)
            path=pkg/'config/payload.json';path.write_text(json.dumps(self.config))
            with patch.object(verification,'ROOT',root),patch.object(verification,'PKG',pkg):
                code=verification.source_digest();calibration=verification.calibration_digest(root)
                altered=dict(self.config,cog_mm=[1.,2.,3.],payload_kg=2.1,source='new calibration');path.write_text(json.dumps(altered))
                self.assertEqual(verification.source_digest(),code);self.assertNotEqual(verification.calibration_digest(root),calibration)
                altered['tool_id']=3;path.write_text(json.dumps(altered));self.assertNotEqual(verification.source_digest(),code)
    def setUp(self):
        self.config=json.loads((ROOT/'src/v3_moveit/config/payload.json').read_text())
        self.observed=dict(tool_id=1,tcp_mm_rad=BASELINE_TCP.copy(),payload_kg=self.config['payload_kg'],cog_mm=self.config['cog_mm'].copy())
    def test_tcp_changes_match_independent_fk_at_multiple_joint_poses(self):
        robot=ET.parse(ROOT/'src/v3_moveit/description/s5.urdf').getroot()
        meta=json.loads((ROOT/'src/v3_moveit/config/model_metadata.json').read_text())
        for tcp in ([0.,0.,0.,0.,0.,0.],[12.,-25.,178.,.4,-.2,1.2],BASELINE_TCP):
            updated,model=apply_tool(robot,meta,dict(tool_id=3,tcp_mm_rad=tcp))
            delta=np.linalg.inv(pose_matrix(BASELINE_TCP))@pose_matrix(tcp)
            for q in [meta['q_reference_rad'],[.1,-.3,.5,.2,.1,-.2]]:
                np.testing.assert_allclose(fk(updated,q),fk(robot,q)@delta,atol=1e-12)
            np.testing.assert_allclose(model['tcp_home'],np.array(meta['tcp_home'])@delta,atol=1e-12)
    def test_default_and_invalid_values(self):
        self.assertEqual(tool_values({}),(1,BASELINE_TCP))
        for data in (dict(tool_id=True),dict(tool_id=-1),dict(tcp_mm_rad=[float('nan')]*6),dict(tcp_mm_rad=[0]*5)):
            with self.assertRaises(ValueError):tool_values(data)
    def test_actionable_diagnostics(self):
        self.assertEqual(mismatch(self.observed,self.config),('',''))
        self.observed['cog_mm'][0]+=1
        self.assertEqual(mismatch(self.observed,self.config)[0],'payload')
        for text in ('PAYLOAD_CENTROID_MISMATCH','Historical payload does not match','实际重心与工程不符'):
            self.assertIn('已校准，刷新负载',recovery(text)[1])
        self.assertIn('V3 配置 2.3559 kg',recovery('Latched stop: PAYLOAD_MISMATCH: 控制柜负载 2.2763 kg，V3 配置 2.3559 kg。')[1])
        self.assertEqual(recovery('TOOL_MISMATCH')[0],'tool')
        self.assertEqual(recovery('site_ubuntu EDG client IP is not assigned to this host')[0],'network')
    def test_atomic_refresh_preserves_precision_and_tool_selection_preserves_load(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'.tmp') as d:
            root=Path(d);path=root/'src/v3_moveit/config/payload.json';path.parent.mkdir(parents=True)
            (root/'.cache/operator').mkdir(parents=True);path.write_text(json.dumps(self.config))
            raw=root/'capture.jsonl';raw.write_text('{"event":"result","ok":true}\n')
            digest=lambda:hashlib.sha256(path.read_bytes()).hexdigest()
            self.observed['cog_mm'][0]=19.123456789123456
            self.assertTrue(save_readback('refresh_payload',self.observed,raw,digest(),root))
            saved=json.loads(path.read_text());self.assertEqual(saved['cog_mm'],self.observed['cog_mm'])
            old=digest();path.write_text(path.read_text()+' ')
            with self.assertRaises(ValueError):save_readback('refresh_payload',self.observed,raw,old,root)
            self.observed.update(tool_id=3,tcp_mm_rad=[12.,0.,170.,0.,0.,0.],payload_kg=4.1)
            self.assertTrue(save_readback('select_tool',self.observed,raw,digest(),root))
            switched=json.loads(path.read_text());self.assertEqual(switched['tool_id'],3);self.assertEqual(switched['payload_kg'],saved['payload_kg'])
            self.observed['tool_id']=1
            before=path.read_bytes()
            with self.assertRaises(ValueError):save_readback('refresh_payload',self.observed,raw,digest(),root)
            self.assertEqual(path.read_bytes(),before)

if __name__=='__main__':unittest.main()
