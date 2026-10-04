import copy,json,threading,time,unittest
import numpy as np
from v2_servo.configuration import load,ROOT
from v2_servo.safety import SafetyFault
from v2_servo.teleop.local_xyz import local_xyz_model
from v2_servo.teleop.quest_xyz import QuestXYZMapping,QuestXYZRehearsal
from v2_servo.teleop.quest_tcp_xyz import QuestTCPXYZJog,validate_xyz_profile
from test_quest_tcp_z import SyntheticTCPRC


class QuestXYZTests(unittest.TestCase):
    def setUp(self):
        self.rc=SyntheticTCPRC();self.cfg=load();self.profile=json.loads((ROOT/'config/quest_xyz_commissioning.json').read_text(encoding='utf-8'))
        self.model=local_xyz_model(self.rc,self.rc.q)
    def plan(self):return QuestTCPXYZJog(self.rc.q,self.cfg,self.model,self.model['tcp_origin_mm_rad'],self.profile)
    def frame(self,t,delta=(20,20,20),grip=.9):
        return {'stamp':t,'sequence':int(t*1000),'grip':grip,'stick_y':0.,'axis_y':0.,
                'deadman':grip>.7,'tracking_valid':True,'base_displacement_mm':list(delta),'input_mode':'xyz'}
    def run_motion(self,p,n=120,delta=(20,20,20),grip=.9,start=0):
        for i in range(n):
            t=start+i*.008;m=p.step(self.frame(t,delta,grip),t);p.check(m.q,m.q,np.zeros(6));p.check_tcp(self.rc.kine_forward(m.q)[1])
        return p.state
    def test_simultaneous_xyz_position_holds_and_release_brakes(self):
        p=self.plan();p.released=True;self.run_motion(p,n=120)
        self.assertTrue(np.all(p.xyz_target_mm>.02));self.assertTrue(p.last_brake_bounds['safe'])
        before=p.xyz_target_mm.copy();v0=np.max(np.abs(p.state.v));self.assertGreater(v0,1e-6)
        self.run_motion(p,n=100,grip=0,start=.96)
        self.assertEqual(p.release_stops,1);self.assertTrue(p.quiet())
        self.assertLess(np.linalg.norm(p.xyz_target_mm-before),.15)
        stationary=p.state.q.copy();self.run_motion(p,n=100,delta=(0,0,0),start=1.76)
        np.testing.assert_allclose(p.state.q,stationary,atol=1e-10)
    def test_start_held_grip_does_not_move_and_tracking_loss_never_rearms(self):
        p=self.plan();self.run_motion(p,80);np.testing.assert_allclose(p.state.q,p.q0,atol=1e-12)
        self.run_motion(p,10,grip=0,start=.64);self.run_motion(p,80,start=.72)
        p.step(dict(self.frame(1.36),tracking_valid=False),1.36)
        self.run_motion(p,100,start=1.368);self.assertTrue(p.locked and p.finished)
        q=p.state.q.copy();self.run_motion(p,50,grip=0,start=2.168);self.run_motion(p,50,start=2.568)
        np.testing.assert_allclose(q,p.state.q,atol=1e-10)
    def test_random_sustained_goals_and_reversals_keep_brake_containment(self):
        p=self.plan();p.released=True;rng=np.random.default_rng(173)
        for block in range(12):
            self.run_motion(p,100,delta=rng.uniform(-100,100,3),start=block*.8)
            self.assertTrue(p.last_brake_bounds['safe']);self.assertLessEqual(np.linalg.norm(p.xyz_target_mm),p.radius)
        self.run_motion(p,100,grip=0,start=9.6);self.assertTrue(p.quiet())
    def test_unsafe_profile_and_raw_feedback_fail(self):
        for key,value in [('translation_gain',1.),('joint_jerk_deg_s3',2.),('position_gain_s_inv',float('nan')),('controlled_stop_budget_s',5.),('brake_samples',1000)]:
            with self.subTest(key=key),self.assertRaises(SafetyFault):validate_xyz_profile(dict(self.profile,**{key:value}))
        p=self.plan();q=p.q0.copy();q[0]+=.01
        with self.assertRaises(SafetyFault):p.check(q,p.q0,np.zeros(6))
        tcp=np.array(p.origin_tcp);tcp[0]+=1
        with self.assertRaisesRegex(SafetyFault,'raw TCP'):p.check_tcp(tcp)
    def test_local_model_uses_three_columns_and_official_FK_grid(self):
        self.assertEqual(np.array(self.model['gain_rad_per_mm_xyz']).shape,(6,3));self.assertEqual(len(self.model['sweep']),91)
        self.assertFalse(self.model['physical_calibration_passed'])
    def test_wrong_receiver_rejected_before_arming_servo(self):
        from v2_servo.teleop.static_hold import run_quest_tcp_xyz
        quest=type('Quest',(),{'input_mode':'pose'})()
        r=run_quest_tcp_xyz(self.rc,self.cfg,{'quest_xyz_profile':self.profile,'edg_client_ip':'192.168.110.96','edg_port':30003}, {},threading.Event(),lambda **row:None,quest)
        self.assertIn('dedicated XYZ receiver',r['failure']);self.assertEqual(r['servo_commands'],0);self.assertEqual(self.rc.calls,[])
    def test_current_robot_readonly_model_reversals_and_moving_software_stop(self):
        # 来自本轮只读FK；此处仍是离线指令测试，不是机器人反馈测试。
        record=json.loads((ROOT/'data/quest_tcp_readonly_20261002_233442_340660/report.json').read_text(encoding='utf-8'))
        model=record['model'];p=QuestTCPXYZJog(model['q_origin_rad'],self.cfg,model,model['tcp_origin_mm_rad'],self.profile)
        p.released=True;peak=0
        for i in range(900):
            t=i*.008;delta=([100,-100,100] if (i//150)%2==0 else [-100,100,-100])
            m=p.step(self.frame(t,delta),t);p.check(m.q,m.q,np.zeros(6));peak=max(peak,np.linalg.norm(p.xyz_target_mm))
            self.assertTrue(p.last_brake_bounds['safe'])
        self.assertGreater(peak,.05);self.assertLessEqual(peak,p.radius)
        self.assertFalse(p.quiet());p.step(self.frame(7.2),7.2,software_stop=True)
        for i in range(120):
            t=7.208+i*.008;m=p.step(self.frame(t),t);p.check(m.q,m.q,np.zeros(6))
        self.assertTrue(p.finished and p.locked);self.assertEqual(p.stop_reason,'right stick software stop')
    def test_predicted_stop_rejects_infeasible_outward_velocity(self):
        p=self.plan();q=p.q0.copy();q[0]=p.upper[0]-1e-8;v=np.zeros(6);v[0]=p.vmax[0]
        self.assertFalse(p.brake_bounds(q,v,np.zeros(6)))
    def test_partial_grip_release_requires_full_release_before_rebind(self):
        p=self.plan();p.released=True;self.run_motion(p,100)
        self.run_motion(p,100,grip=.5,start=.8);self.assertTrue(p.quiet());q=p.state.q.copy()
        self.run_motion(p,100,delta=(0,0,0),grip=.9,start=1.6)
        np.testing.assert_allclose(q,p.state.q,atol=1e-10);self.assertFalse(p.released)
        self.run_motion(p,20,grip=0,start=2.4);self.run_motion(p,100,delta=(0,0,0),start=2.56)
        np.testing.assert_allclose(q,p.state.q,atol=1e-10)
    def test_input_three_directions_and_reference_rebind(self):
        # 用真实保存的Quest报文结构测试顺序/身份/参考；不冒充实时硬件。
        source=ROOT/'data/h5_20261002_230308_047401/events.jsonl'
        packet=None
        with source.open(encoding='utf-8') as lines:
            for line in lines:
                row=json.loads(line)
                if row.get('event')=='Quest_raw_input':packet=row['raw'];break
        self.assertIsNotNone(packet);mapper=QuestXYZMapping(self.profile['basis']);sequence=0
        def decode(position,grip):
            nonlocal sequence
            sequence+=1;p=copy.deepcopy(packet);p.update(sequence=sequence,unity_time_s=sequence*.02)
            p['right'].update(position_m=dict(zip(('x','y','z'),position)),grip=grip,thumbstick_click=False)
            return mapper.decode(p,('127.0.0.1',45000),sequence*.02)
        decode((0,0,0),0);decode((0,0,0),1)
        r=decode((-.01,.02,.03),1);np.testing.assert_allclose(r['base_displacement_mm'],[30,10,20])
        decode((1,1,1),0);r=decode((1,1,1),1);np.testing.assert_allclose(r['base_displacement_mm'],[0,0,0])
        packet.update(sequence=1,unity_time_s=.02)
        with self.assertRaisesRegex(ValueError,'reordered'):mapper.decode(packet,('127.0.0.1',45000),1)
    def test_rehearsal_requires_all_three_axes_and_rejects_diagonal(self):
        r=QuestXYZRehearsal();sequence=0
        def feed(grip,axes,n):
            nonlocal sequence
            for _ in range(n):
                sequence+=1;f=self.frame(sequence*.02,grip=grip);f.update(sequence=sequence,axes_xyz=axes)
                r.step(f,f['stamp'])
        feed(0,[0,0,0],10);feed(1,[1,1,0],10);self.assertEqual(r.stage,'GRIP_X')
        for axes in ([1,0,0],[0,1,0],[0,0,1]):feed(1,axes,3);feed(0,[0,0,0],10)
        self.assertTrue(r.passed);self.assertEqual(r.axes_verified,['X','Y','Z'])


if __name__=='__main__':unittest.main()
