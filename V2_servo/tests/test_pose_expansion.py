"""扩展档位回归：全是假SDK/离线，不产生真机证据。"""
import copy,json,unittest
import numpy as np
from unittest.mock import patch
from v2_servo.configuration import ROOT,load
from v2_servo.safety import SafetyFault
from v2_servo.teleop.pose_expansion import expansion_model,validate_expansion,QuestPoseExpansionPlanner
from v2_servo.teleop.commissioning_guidance import guidance_packet
from test_quest_tcp_z import SyntheticTCPRC


class ExpansionTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load();self.rc=SyntheticTCPRC()
        self.profile=json.loads((ROOT/'config/quest_pose_expansion.json').read_text(encoding='utf-8'))
        self.model=expansion_model(self.rc,self.rc.q,self.profile,self.cfg['limits'])
    def planner(self):return QuestPoseExpansionPlanner(self.rc.q,self.cfg,self.model,self.model['tcp_origin_mm_rad'],self.profile)
    def test_real_coupled_domain_and_fixed_profile(self):
        self.assertGreaterEqual(self.model['maximum_radius_mm'],1)
        self.assertEqual(len(self.model['attempts'][-1]['samples']),729)
        self.assertFalse(self.model['robot_full_workspace_approved'])
        for k in ('maximum_radius_mm','joint_velocity_deg_s','rotation_gain'):
            p=copy.deepcopy(self.profile);p[k]*=2
            with self.assertRaises(SafetyFault):validate_expansion(p)
        cfg=copy.deepcopy(self.cfg);cfg['filters']['enabled']=False
        with self.assertRaises(SafetyFault):QuestPoseExpansionPlanner(self.rc.q,cfg,self.model,self.model['tcp_origin_mm_rad'],self.profile)
    def test_six_dof_release_limits_and_stale_latch(self):
        p=self.planner();peak=0
        for i in range(450):
            t=i*.008;held=20<=i<240
            f=dict(stamp=t,tracking_valid=True,deadman=held,grip=float(held),base_displacement_mm=[20.]*3,rotation_delta_deg=[5.]*3)
            s=p.step(f,t);p.check(s.q,s.q,np.zeros(6));p.check_tcp(self.rc.kine_forward(s.q)[1])
            peak=max(peak,np.linalg.norm(p.xyz_target_mm))
        self.assertGreater(peak,.2);self.assertTrue(p.quiet());self.assertEqual(len(p.release_events),1)
        self.assertLessEqual(p.release_events[0]['command_stop_time_ms'],1000)
        self.assertFalse(p.release_events[0]['feedback_stop_validated'])
        p.step(None,3.7)
        for i in range(150):
            t=3.708+i*.008;p.step(dict(stamp=t,tracking_valid=True,deadman=True,grip=1.,base_displacement_mm=[20.]*3,rotation_delta_deg=[5.]*3),t)
        self.assertTrue(p.locked);self.assertTrue(p.finished)
    def test_raw_protection_and_guidance(self):
        p=self.planner()
        with self.assertRaises(SafetyFault):p.check(p.q0,p.q0,np.ones(6))
        with self.assertRaises(SafetyFault):p.check(p.q0,p.q0+.01,np.zeros(6))
        tcp=p.origin_tcp.copy();tcp[0]+=10
        with self.assertRaises(SafetyFault):p.check_tcp(tcp)
        g=guidance_packet('LIVE','pose6dof_live',control_scope='TCP_6DOF_EXPANDED')
        self.assertIn('扩展',g['title']);self.assertIn('不是全空间放行',g['instruction'])
        with self.assertRaises(ValueError):guidance_packet('LIVE','pose6dof_shadow',control_scope='TCP_6DOF_EXPANDED')
    def test_failed_or_sim_baseline_refused_and_real_baseline_accepted(self):
        from v2_servo.teleop.expansion_evidence import verify_h8_baseline
        path=ROOT/'data/h8_20261003_183549_894989/report.json'
        r=json.loads(path.read_text(encoding='utf-8'));meta=json.loads(path.with_name('metadata.json').read_text(encoding='utf-8'))
        self.assertIn('H8_capability',verify_h8_baseline(path,meta['sdk_artifacts_sha256'])['scope'])
        for mutation in (dict(failure='injected'),dict(scope='offline_H8_no_SDK'),dict(logout=[-1])):
            bad=dict(r,**mutation)
            with patch('v2_servo.teleop.expansion_evidence.report_path',return_value=(path,bad)):
                with self.assertRaises(ValueError):verify_h8_baseline(path,meta['sdk_artifacts_sha256'])
    def test_delayed_plant_ablation_freeze_and_fault_line_unchanged(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('h9_offline_verify',ROOT/'scripts/verify_h9_tracking_governor.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        model=json.loads((ROOT/'data/h9_20261003_201540_389438/report.json').read_text(encoding='utf-8'))['official_local_tcp_xyz']
        old=module.simulate(model,self.profile,tau=.25,governor=False)
        new=module.simulate(model,self.profile,tau=.25,governor=True)
        self.assertIn('tracking',old['failure']);self.assertIsNone(new['failure']);self.assertTrue(new['command_stopped'])
        self.assertLess(new['maximum_raw_candidate_gap_deg'],.05);self.assertGreater(new['slow_count'],0)
        frozen=module.simulate(model,self.profile,governor=True,frozen=True)
        self.assertIn('stalled',frozen['failure'])
        p=self.planner();q=p.q0.copy();q[0]+=.001
        with self.assertRaisesRegex(SafetyFault,'0.05'):p.observe_feedback(q,np.zeros(6))
