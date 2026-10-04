"""H10本地非线性、自然手柄意图和保护回归；不连接机器人。"""
import copy,json,unittest,subprocess,sys,os,datetime
from unittest.mock import patch
import numpy as np
from v2_servo.configuration import ROOT,load
from v2_servo.safety import SafetyFault
from v2_servo.teleop.local_nonlinear import CubicFK,QuestWorkspacePlanner,validate_workspace
from v2_servo.teleop.commissioning_guidance import guidance_packet,workspace_mapping_detail
from v2_servo.teleop.workspace_evidence import verify_h9_stable,coverage_union,h9_coverage_audit,h9_window_eligible,workspace_entry_audit

class WorkspacePilotTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load();self.profile=json.loads((ROOT/'config/quest_pose_workspace.json').read_text(encoding='utf-8'))
        self.model=json.loads((ROOT/'data/quest_tcp_readonly_20261003_204517_202920/report.json').read_text(encoding='utf-8'))['model']
    def planner(self):return QuestWorkspacePlanner(self.model['q_origin_rad'],self.cfg,self.model,self.model['tcp_origin_mm_rad'],self.profile)
    def test_cubic_derivative_and_outside_domain_fail_closed(self):
        m=CubicFK(self.model['q_origin_rad'],self.model['normalized_scale_rad'],self.model['coefficients']);q=m.q+np.array([.001,.003,-.002,.001,0,.003])
        numeric=np.empty((6,6))
        for j in range(6):
            d=np.zeros(6);d[j]=1e-6;numeric[:,j]=(m.evaluate(q+d)-m.evaluate(q-d))/2e-6
        np.testing.assert_allclose(m.jacobian(q),numeric,atol=1e-7)
        with self.assertRaises(SafetyFault):m.evaluate(m.q+1)
        with self.assertRaises(SafetyFault):CubicFK(m.q,0,m.c)
        with self.assertRaises(SafetyFault):CubicFK(m.q,m.scale,[[0.]])
    def test_delayed_follow_large_hand_gesture_release_and_stale(self):
        p=self.planner();actual=p.q0.copy();dq=np.zeros(6);peak=0.
        for i in range(1300):
            t=i*.008;held=20<=i<900
            f=dict(stamp=t,tracking_valid=True,grip=float(held),deadman=held,base_displacement_mm=[20.]*3,rotation_delta_deg=[15.,0.,0.])
            p.observe_feedback(actual,dq);m=p.step(f,t);p.check(m.q,actual,dq);peak=max(peak,np.linalg.norm(p.xyz_target_mm))
            old=actual.copy();actual+=(m.q-actual)*(1.-np.exp(-.008/.25));dq=(actual-old)/.008
        self.assertGreater(peak,1.);self.assertTrue(p.quiet());self.assertLessEqual(np.linalg.norm(p.rotation_target_deg),.5)
        self.assertEqual(len(p.release_events),1);self.assertGreater(p.tracking_slow_count,0)
        p.step(None,10.5)
        for i in range(150):p.step(None,10.508+i*.008)
        self.assertTrue(p.locked);self.assertTrue(p.finished);self.assertIn('H10',p.stop_reason)
    def test_profile_filter_and_raw_model_protection(self):
        invalid=copy.deepcopy(self.profile);invalid['maximum_radius_mm']=100
        with self.assertRaises(SafetyFault):validate_workspace(invalid)
        p=self.planner();tcp=p.origin_tcp.copy();tcp[0]+=1
        with self.assertRaisesRegex(SafetyFault,'disagreement'):p.check_model_feedback(p.q0,tcp)
        cfg=load();cfg['filters']['enabled']=False
        with self.assertRaises(SafetyFault):QuestWorkspacePlanner(p.q0,cfg,self.model,p.origin_tcp,self.profile)
        g=guidance_packet('LIVE','pose6dof_live',control_scope='TCP_6DOF_WORKSPACE')
        self.assertIn('10mm',g['title']);self.assertIn('不是1:1',g['instruction'])
    def test_stable_partial_H9_not_relabeled_full_six(self):
        path=ROOT/'data/h9_20261003_203318_417742/report.json';meta=json.loads(path.with_name('metadata.json').read_text(encoding='utf-8'))
        e=verify_h9_stable(path,meta['sdk_artifacts_sha256']);self.assertFalse(e['H9_full_six_coverage_passed'])
        report=json.loads(path.read_text(encoding='utf-8'));report['passed_quest_expansion_finite_only']=True
        with patch('v2_servo.teleop.workspace_evidence.report_path',return_value=(path,report)):
            self.assertFalse(verify_h9_stable(path,meta['sdk_artifacts_sha256'])['H9_full_six_coverage_passed'])
        report['failure']='injected tracking fault'
        with patch('v2_servo.teleop.workspace_evidence.report_path',return_value=(path,report)):
            with self.assertRaises(ValueError):verify_h9_stable(path,meta['sdk_artifacts_sha256'])

    def test_live_entry_refuses_missing_capability_before_input_or_sdk(self):
        # 新门槛仍拒绝证据缺失，不能把取消补X变成无条件放行。
        code="""import runpy,sys
from unittest.mock import patch
from v2_servo.configuration import ROOT
with patch('v2_servo.teleop.workspace_evidence.workspace_entry_audit',return_value={'ready_for_site_confirmation':False,'rejected':[{'reason':'missing capability'}]}):
    sys.argv=['quest_pose_live.py','--workspace','--live']
    runpy.run_path(str(ROOT/'scripts/quest_pose_live.py'),run_name='__main__')
"""
        env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'src');env['PYTHONDONTWRITEBYTECODE']='1'
        r=subprocess.run([sys.executable,'-c',code],cwd=ROOT,env=env,capture_output=True,text=True,encoding='utf-8',timeout=8)
        self.assertEqual(r.returncode,2,r.stderr)
        result=json.loads(r.stdout);self.assertFalse(result['ready_for_site_confirmation'])
        self.assertEqual(result['robot_commands'],0);self.assertNotIn('READY200 >',r.stdout)

    def test_cancel_not_treated_as_success_by_launcher(self):
        code="""import runpy,sys
from unittest.mock import patch
from v2_servo.configuration import ROOT
with patch('v2_servo.teleop.workspace_evidence.workspace_entry_audit',return_value={'ready_for_site_confirmation':True,'rejected':[]}),patch('builtins.input',return_value='CANCEL'):
    sys.argv=['quest_pose_live.py','--workspace','--live']
    runpy.run_path(str(ROOT/'scripts/quest_pose_live.py'),run_name='__main__')
"""
        env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'src');env['PYTHONDONTWRITEBYTECODE']='1'
        r=subprocess.run([sys.executable,'-c',code],cwd=ROOT,env=env,capture_output=True,text=True,encoding='utf-8',timeout=8)
        self.assertEqual(r.returncode,1,r.stderr);self.assertIn('Cancelled before SDK connection.',r.stdout)

    def test_union_requires_each_axis_from_one_real_source_pair(self):
        self.assertTrue(h9_window_eligible(3000,29,False))
        self.assertFalse(h9_window_eligible(1000,8,False))
        self.assertTrue(h9_window_eligible(1000,8,True))
        self.assertFalse(h9_window_eligible(20,.16,True))
        a=dict(path='fixture_a',sha256='fixture_a',command_spans=[.19,.3,.3,.02,.02,.02],actual_spans=[.18,.3,.3,.02,.02,.02])
        b=dict(path='fixture_b',sha256='fixture_b',command_spans=[.3,0,0,0,0,0],actual_spans=[.21,0,0,0,0,0])
        self.assertEqual(coverage_union([a])['missing_axes'],['X'])
        u=coverage_union([a,b]);self.assertTrue(u['H9_stage_coverage_passed']);self.assertEqual(u['axis_sources']['X']['path'],'fixture_b')
        # 不得把来自不同会话的指令跨度和反馈跨度凑成同一轴通过。
        a['command_spans'][0]=.3;a['actual_spans'][0]=.1;b['command_spans'][0]=.1;b['actual_spans'][0]=.3
        self.assertEqual(coverage_union([a,b])['missing_axes'],['X'])
        b['actual_spans'][0]=float('nan')
        with self.assertRaises(ValueError):coverage_union([a,b])
        self.assertFalse(coverage_union([])['H9_stage_coverage_passed'])

    def test_actual_union_preserves_failed_single_session_flags(self):
        p=ROOT/'data/h9_20261003_212057_578846/report.json';before=p.read_bytes()
        sdk=json.loads(p.with_name('metadata.json').read_text(encoding='utf-8'))['sdk_artifacts_sha256']
        fixed_time=datetime.datetime(2026,10,3,22,tzinfo=datetime.timezone(datetime.timedelta(hours=8))).timestamp()
        with patch('v2_servo.teleop.workspace_evidence.time.time',return_value=fixed_time):
            audit=h9_coverage_audit([p,ROOT/'data/h9_20261003_203318_417742/report.json'],sdk)
            self.assertFalse(h9_coverage_audit([p],{'fake_sdk':'wrong'})['records'])
        with patch('v2_servo.teleop.workspace_evidence.time.time',return_value=fixed_time+86401):
            expired=h9_coverage_audit([p],sdk)
            self.assertFalse(expired['H9_stage_coverage_passed']);self.assertFalse(expired['records'])
        self.assertFalse(audit['H9_stage_coverage_passed']);self.assertEqual(audit['missing_axes'],['X'])
        self.assertEqual(p.read_bytes(),before);self.assertFalse(json.loads(before)['passed_quest_expansion_finite_only'])
        self.assertEqual(audit['progress']['X']['required'],.2)
        g=guidance_packet('LIVE','pose6dof_live',control_scope='TCP_6DOF_COVERAGE')
        self.assertIn('只补',g['instruction']);self.assertIn('不需重做',g['instruction'])

    def test_real_prior_response_and_stability_reused_without_relabeling_coverage(self):
        p=ROOT/'data/h9_20261003_212057_578846/report.json';before=p.read_bytes()
        sdk=json.loads(p.with_name('metadata.json').read_text(encoding='utf-8'))['sdk_artifacts_sha256']
        fixed=datetime.datetime(2026,10,3,23,tzinfo=datetime.timezone(datetime.timedelta(hours=8))).timestamp()
        with patch('time.time',return_value=fixed):
            audit=workspace_entry_audit([p],(ROOT/'data').glob('h8_*/report.json'),sdk)
            self.assertTrue(audit['ready_for_site_confirmation'],audit['rejected'])
            self.assertFalse(audit['coverage']['H9_stage_coverage_passed'])
            self.assertEqual(audit['coverage']['missing_axes'],['X'])
            self.assertFalse(audit['automatic_axis_retest'])
            self.assertFalse(audit['full_workspace_approved'])
            self.assertEqual(audit['robot_commands'],0)
            self.assertFalse(workspace_entry_audit([p],[],sdk)['ready_for_site_confirmation'])
            self.assertFalse(workspace_entry_audit([p],[],{'wrong':'sdk'})['ready_for_site_confirmation'])
        self.assertEqual(p.read_bytes(),before)

    def test_newest_failure_blocks_older_success_and_core_change_blocks_reuse(self):
        new=ROOT/'data/h9_20261003_212057_578846/report.json';old=ROOT/'data/h9_20261003_203318_417742/report.json'
        sdk=json.loads(new.with_name('metadata.json').read_text(encoding='utf-8'))['sdk_artifacts_sha256']
        with patch('v2_servo.teleop.workspace_evidence.verify_h9_stable',side_effect=ValueError('latest failure')) as verify:
            audit=workspace_entry_audit([old,new],[],sdk)
            self.assertFalse(audit['ready_for_site_confirmation']);verify.assert_called_once_with(new,sdk)
        from v2_servo.teleop.xyz_evidence import sha
        def changed(path):return 'changed' if str(path).endswith('static_hold.py') else sha(path)
        with patch('v2_servo.teleop.workspace_evidence.sha',side_effect=changed):
            audit=workspace_entry_audit([new],[],sdk)
            self.assertFalse(audit['ready_for_site_confirmation'])
            self.assertIn('core changed',audit['rejected'][0]['reason'])

    def test_display_distinguishes_hand_goal_command_and_actual(self):
        row=dict(hand_base_displacement_mm=[20.,0,0],robot_goal_mm=[7.5,0,0],robot_command_mm=[6.,0,0],
            TCP_xyz_actual_delta_mm=[5.8,0,0],hand_rotation_deg=[15.,0,0],robot_target_rotation_deg=[.2,0,0],
            TCP_rotation_actual_deg=[.18,0,0],phase='GRIP_6DOF_WORKSPACE')
        detail=workspace_mapping_detail(row)
        for text in ('20.0','7.5','6.0','5.8','15.0','0.200','0.180','0.300','真机反馈','规划指令'):
            self.assertIn(text,detail)
        g=guidance_packet('LIVE','pose6dof_live',control_scope='TCP_6DOF_WORKSPACE')
        self.assertIn('不要求逐轴打卡',g['instruction']);self.assertIn('姿态目标≤0.3',g['instruction'])
