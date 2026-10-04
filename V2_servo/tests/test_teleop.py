import unittest,json,time,tempfile,copy,socket
from pathlib import Path
import numpy as np
from unittest.mock import Mock,patch
from v2_servo.configuration import load,ROOT
from v2_servo.teleop.motion import VelocityServo
from v2_servo.teleop.ipc import Channels
from v2_servo.teleop.engine import TeleopEngine
from v2_servo.teleop.feedback import JsonFrames,decode_frame
from v2_servo.teleop.quest import QuestVelocity,QuestInput
from v2_servo.safety import Guard,SafetyFault
from v2_servo.safety import Feedback
from v2_servo.teleop.backends import LiveBackend,DeploymentReview,tcp_owners_from_netstat
from v2_servo.teleop.edg import EdgMode1Feedback,decode_status_pair

Q=np.array([-1.1159,2.2865,-2.3207,3.0411,2.4042,2.5245])
class TeleopTests(unittest.TestCase):
    def test_sdk_owner_query_matches_only_controller_remote_port_and_keeps_pending_owner(self):
        raw='''TCP 192.168.110.96:45001 192.168.110.224:10001 ESTABLISHED 1234
TCP 192.168.110.96:45002 192.168.110.224:10001 SYN_SENT 5678
TCP 192.168.110.96:10001 192.168.110.224:45003 ESTABLISHED 9999
TCP 192.168.110.96:45004 192.168.110.223:10001 ESTABLISHED 9999
TCP 192.168.110.96:45005 192.168.110.224:10001 ESTABLISHED 4321
TCP 192.168.110.96:45006 192.168.110.224:10001 TIME_WAIT 0
TCP 192.168.110.96:45007 192.168.110.224:10001 FIN_WAIT_2 432
TCP 192.168.110.96:45008 192.168.110.224:10001 CLOSE_WAIT 6789'''
        self.assertEqual(tcp_owners_from_netstat(raw,'192.168.110.224',4321),[1234,5678,6789])
    def test_edg_mode1_decoder_keeps_source_timestamp_and_raw_safety(self):
        flags=decode_status_pair((0,0,1,0,0,0,0,0,0,0,1),(0,'',1,1))
        self.assertTrue(flags['collision']); self.assertFalse(flags['estop'])
        with self.assertRaises(SafetyFault):decode_status_pair((0,)*10,(0,'',1,1))
        rc=Mock(); state=([.1]*6,[0.]*6,[1.]*6,[1,2,3,0,0,0],[0.]*6,())
        rc.edg_get_stat.return_value=(0,state)
        rc.edg_stat_details.side_effect=[(0,(100,0,1)),(0,(100,0,1)),(0,(100,8_000_000,2))]
        feed=EdgMode1Feedback(rc,'192.168.110.96',30003,.002)
        feed.entered=True;feed.status=flags;feed.status_at=time.perf_counter()
        first,_=feed.sample(time.perf_counter())
        repeated,_=feed.sample(time.perf_counter())
        newer,_=feed.sample(time.perf_counter())
        self.assertIs(first,repeated)
        self.assertEqual((first.sequence,newer.sequence),(1,2))
        self.assertTrue(first.collision)
        self.assertIsNone(first.torque)
        self.assertAlmostEqual(newer.controller_timestamp-first.controller_timestamp,.008)
    def test_focus_loss_preserves_vr_deadman_loop_but_drops_keyboard(self):
        from v2_servo.gui.main import TeleopWindow
        window=Mock(); window.closing=False; window.root.focus_displayof.return_value=None
        window.keys={'space','1'}; window.mouse=(0,1); window.source.get.return_value='Quest VR'
        TeleopWindow._check_focus(window)
        self.assertEqual(window.keys,set()); self.assertIsNone(window.mouse)
        window.stop.assert_not_called()
        window.source.get.return_value='键盘 / 鼠标'; window.keys={'space'}
        TeleopWindow._check_focus(window)
        window.stop.assert_called_once()
    def test_velocity_reversal_and_release(self):
        cfg=load(); p=VelocityServo(Q,cfg); rows=[]
        for i in range(1000):
            m=p.step([1 if i<350 else -1,0,0,0,0,0],speed=1,acceleration=1,hold=i>=700); rows.append(m)
        self.assertLess(np.max(np.abs(rows[-1].v)),1e-8)
        self.assertLessEqual(np.max(np.abs([r.a for r in rows])),max(cfg['limits']['acceleration_rad_s2'])+1e-8)
        self.assertLessEqual(np.max(np.abs([r.j for r in rows])),max(cfg['limits']['jerk_rad_s3'])+1e-8)
        self.assertGreater(rows[349].q[0]-Q[0],1.)
    def test_full_domain_boundary_brakes(self):
        cfg=load(); q=Q.copy(); q[0]=cfg['limits']['upper_rad'][0]-.3; p=VelocityServo(q,cfg)
        braked=False
        for _ in range(500):
            m=p.step([1,0,0,0,0,0],speed=1,acceleration=1); braked|=p.boundary_braking
            self.assertLessEqual(m.q[0],cfg['limits']['upper_rad'][0])
        self.assertTrue(braked)
    def test_cartesian_velocity_moves_model_x(self):
        p=VelocityServo(Q,load()); before=p.arm.forward(Q)[0][:3,3]
        for _ in range(125): p.step([1,0,0,0,0,0],'cartesian')
        delta=p.arm.forward(p.state.q)[0][:3,3]-before
        self.assertGreater(delta[0],.01); self.assertLess(np.linalg.norm(delta[1:]),.002)
    def test_json_split_coalesced_and_units(self):
        raw={'joint_actual_position':[180]*6,'actual_position':[1,2,3,90,0,0],'errcode':'0x0',
             'enabled':1,'powered_on':1,'emergency_stop':0,'protective_stop':0,'on_soft_limit':0}
        b=json.dumps(raw).encode(); p=JsonFrames(); self.assertEqual(p.feed(b[:20]),[])
        frames=p.feed(b[20:]+b'\n'+b); self.assertEqual(len(frames),2)
        f,t=decode_frame(frames[0],10,1); self.assertAlmostEqual(f.q[0],np.pi); self.assertAlmostEqual(t[3],np.pi/2)
        raw['enabled']='false'
        with self.assertRaises(ValueError): decode_frame(raw,10,1)
    def test_vr_requires_release_and_rejects_replay(self):
        m=QuestVelocity(np.eye(3)); addr=('127.0.0.1',1)
        packet={'sequence':1,'unity_time_s':1.,'right':{'connected':True,'tracked':True,'pose_valid':True,'position_m':[0,0,0],'rotation_xyzw':[0,0,0,1],'grip':.9}}
        self.assertFalse(m.decode(packet,addr,1)['deadman'])
        packet.update(sequence=2,unity_time_s=1.01); packet['right']['grip']=0; m.decode(packet,addr,1.01)
        packet.update(sequence=3,unity_time_s=1.02); packet['right']['grip']=1; self.assertTrue(m.decode(packet,addr,1.02)['deadman'])
        packet.update(sequence=4,unity_time_s=1.03); packet['right']['position_m'][0]=.001
        self.assertGreater(m.decode(packet,addr,1.03)['axes'][0],0)
        with self.assertRaises(ValueError): m.decode(packet,addr,2)
    def test_observed_right_stick_click_latches_software_stop_and_requires_regrip(self):
        m=QuestVelocity(np.eye(3)); addr=('127.0.0.1',5006)
        right={'connected':True,'tracked':True,'pose_valid':True,'position_m':[0,0,0],
               'rotation_xyzw':[0,0,0,1],'grip':0.,'stick_click_available':True,'thumbstick_click':False}
        packet={'sequence':1,'unity_time_s':1.,'right':right}
        m.decode(packet,addr,1.)
        right['grip']=.9;packet['sequence']=2;packet['unity_time_s']=1.01
        self.assertTrue(m.decode(packet,addr,1.01)['deadman'])
        right['thumbstick_click']=True;packet['sequence']=3;packet['unity_time_s']=1.02
        clicked=m.decode(packet,addr,1.02)
        self.assertTrue(clicked['software_stop']);self.assertFalse(clicked['deadman'])
        packet['sequence']=4;packet['unity_time_s']=1.03
        self.assertFalse(m.decode(packet,addr,1.03)['software_stop'])
        right['thumbstick_click']=False;packet['sequence']=5;packet['unity_time_s']=1.04
        self.assertFalse(m.decode(packet,addr,1.04)['deadman'])
        right['grip']=0.;packet['sequence']=6;packet['unity_time_s']=1.05
        m.decode(packet,addr,1.05)
        right['grip']=.9;packet['sequence']=7;packet['unity_time_s']=1.06
        self.assertTrue(m.decode(packet,addr,1.06)['deadman'])
    def test_quest_udp_click_survives_gui_poll_gap(self):
        receiver=QuestInput(0,np.eye(3))
        self.addCleanup(receiver.close)
        port=receiver.socket.getsockname()[1]
        right={'connected':True,'tracked':True,'pose_valid':True,'position_m':[0,0,0],
               'rotation_xyzw':[0,0,0,1],'grip':0.,'stick_click_available':True,'thumbstick_click':False}
        sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            for sequence,clicked in ((1,False),(2,True),(3,False)):
                right['thumbstick_click']=clicked
                sender.sendto(json.dumps({'sequence':sequence,'unity_time_s':1+sequence*.01,
                                          'right':right}).encode(),('127.0.0.1',port))
            deadline=time.perf_counter()+.5
            while receiver.latest is None or receiver.latest['sequence']!=3:
                if time.perf_counter()>deadline:self.fail('Quest UDP reader did not receive three packets')
                time.sleep(.005)
            self.assertTrue(receiver.take_software_stop())
            self.assertFalse(receiver.take_software_stop())
        finally:sender.close()
    def engine(self):
        path=Path(tempfile.mkdtemp(prefix='v2test_',dir=str(ROOT/'data'))); ch=Channels()
        e=TeleopEngine(ch,'sim',{},path); self.addCleanup(e.close)
        e.backend.connect(); e.backend.power(True); e.backend.enable(True); ch.publish(1,[0]*6,'joint',False,.1,.25,6)
        e.start(); return e,ch
    def test_stale_gui_rejected_before_send(self):
        e,ch=self.engine(); ch.publish(2,[1,0,0,0,0,0],'joint',True,.1,.25,6,time.perf_counter()-.2)
        with self.assertRaises(SafetyFault): e.tick(time.perf_counter(),0)
        self.assertEqual(e.backend.sent,0)
    def test_stale_feedback_rejected(self):
        e,ch=self.engine(); e.backend.inject_stale=True; time.sleep(.03); ch.publish(2,[0]*6,'joint',False,.1,.25,6)
        with self.assertRaisesRegex(SafetyFault,'feedback_stale'): e.tick(time.perf_counter(),0)
    def test_stop_never_clears_estop_and_discards_queued_start(self):
        e,ch=self.engine(); old={'name':'start','stamp':time.perf_counter()}; e.stop(estop=True); e.stop()
        self.assertEqual(e.state,'ESTOP')
        with self.assertRaises(RuntimeError): e.operate(old)
        with self.assertRaises(RuntimeError): e.start()
    def test_source_change_fault(self):
        e,ch=self.engine(); ch.publish(2,[0]*6,'joint',False,.1,.25,6,source='vr')
        with self.assertRaises(SafetyFault): e.tick(time.perf_counter(),0)
    def test_partial_start_attempt_cleanup(self):
        path=Path(tempfile.mkdtemp(prefix='v2test_',dir=str(ROOT/'data'))); ch=Channels(); e=TeleopEngine(ch,'sim',{},path); self.addCleanup(e.close)
        e.backend.connect(); e.backend.power(True); e.backend.enable(True); ch.publish(1,[0]*6,'joint',False,.1,.25,6)
        def failing_start(*args): e.backend.active=True; raise RuntimeError('simulated mode-confirmation failure')
        e.backend.start=failing_start
        with self.assertRaises(RuntimeError): e.start()
        self.assertFalse(e.backend.active); self.assertEqual(e.state,'FAULT')
    def test_active_start_does_not_reset_planner(self):
        e,ch=self.engine(); planner=e.motion
        with self.assertRaises(RuntimeError): e.start()
        self.assertIs(e.motion,planner)
    def test_estop_allows_disable_but_not_reconnect_bypass(self):
        e,ch=self.engine(); e.stop(estop=True)
        e.operate({'name':'enable','on':False,'stamp':time.perf_counter()})
        self.assertFalse(e.backend.enabled); self.assertEqual(e.state,'ESTOP')
        e.operate({'name':'disconnect','stamp':time.perf_counter()})
        e.operate({'name':'connect','stamp':time.perf_counter()})
        self.assertEqual(e.state,'ESTOP')
        with self.assertRaises(RuntimeError): e.operate({'name':'power','on':True,'stamp':time.perf_counter()})
    def test_live_owner_refusal_never_loads_sdk(self):
        b=LiveBackend(load(),{'host':'192.168.110.224','deployment_review':'config/live_deployment.json'})
        with patch('v2_servo.teleop.backends.controller_owners',return_value=[123]),patch('v2_servo.teleop.backends.load_sdk') as loader:
            with self.assertRaisesRegex(RuntimeError,'123'): b.connect()
            loader.assert_not_called()
    def test_missing_deployment_makes_no_sdk_calls(self):
        b=LiveBackend(load(),{'deployment_review':'config/live_deployment.json'}); b.connected=True; b.rc=Mock()
        with self.assertRaises(RuntimeError): b.start()
        self.assertEqual(b.rc.mock_calls,[])
    def test_live_active_sample_never_sync_queries(self):
        import queue
        b=LiveBackend(load(),{'deployment_review':'config/live_deployment.json'}); b.connected=True; b.active=True; b.rc=Mock(); b.fp_queue=queue.Queue()
        stamp=time.perf_counter(); b.review.profile={'controller_feedback_age_bound_s':.005}
        b.fp_queue.put({'feedback':Feedback(Q,stamp,3),'tcp':np.zeros(6)})
        first,_=b.sample(stamp+.002); second,_=b.sample(stamp+.006)
        self.assertAlmostEqual(first.sampled_at,stamp-.005); self.assertEqual(first.sampled_at,second.sampled_at)
        self.assertEqual(b.rc.mock_calls,[])
    def test_live_send_stop_event_prevents_native_call(self):
        from v2_servo.jaka import JakaServo
        from v2_servo.trajectory import Motion
        b=LiveBackend(load(),{'deployment_review':'config/live_deployment.json'}); b.rc=Mock(); b.review.deadline=time.perf_counter()+10
        b.driver=JakaServo(b.rc,b.cfg,b.authorize); b.driver.entered=True; b.driver.filter_set=True; b.interrupted=lambda:True
        with self.assertRaises(RuntimeError): b.send(Motion.rest(Q))
        self.assertEqual(b.rc.mock_calls,[])
    def test_unconfirmed_stop_retains_driver_for_retry(self):
        b=LiveBackend(load(),{'deployment_review':'config/live_deployment.json'}); d=Mock(); b.driver=d
        d.stop.return_value={'servo_exited':False,'physical_stop_confirmed':False,'errors':['network']}
        b.stop(); self.assertIs(b.driver,d)
        d.stop.return_value={'servo_exited':True,'physical_stop_confirmed':False,'errors':[]}; b.stop(); self.assertIsNone(b.driver)
    def test_live_missing_stream_not_replaced_by_fresh_sync_status(self):
        b=LiveBackend(load(),{'deployment_review':'config/live_deployment.json'}); b.connected=True; b.rc=Mock()
        expected=json.loads((ROOT/'config/site_observed_20260928.json').read_text(encoding='utf-8'))
        b.review.verify=Mock(return_value={'network_abort_timeout_ms':40,'controller_feedback_age_bound_s':.001})
        b.rc.get_tool_id.return_value=(0,1); b.rc.get_tool_data.return_value=(0,1,expected['tcp_mm_rad'])
        b.rc.get_payload.return_value=(0,[expected['payload_kg'],expected['cog_mm']]); b.last_poll=time.perf_counter()
        b.fb=Feedback(Q,time.perf_counter(),1); b.feedback_error=None
        with self.assertRaisesRegex(RuntimeError,'独立10000反馈'): b.start()
        b.rc.servo_move_enable.assert_not_called(); b.rc.set_network_exception_handle.assert_not_called()
    def test_live_backend_fake_sdk_full_servo_sequence(self):
        from v2_servo.trajectory import Motion
        b=LiveBackend(load(),{'deployment_review':'config/live_deployment.json'}); b.connected=True; b.rc=Mock()
        expected=json.loads((ROOT/'config/site_observed_20260928.json').read_text(encoding='utf-8'))
        profile={'network_abort_timeout_ms':40,'controller_feedback_age_bound_s':0.}
        b.review.profile=profile; b.review.deadline=time.perf_counter()+10; b.review.verify=Mock(return_value=profile)
        b.rc.get_tool_id.return_value=(0,1); b.rc.get_tool_data.return_value=(0,1,expected['tcp_mm_rad'])
        b.rc.get_payload.return_value=(0,[expected['payload_kg'],expected['cog_mm']]); b.rc.get_actual_joint_position.return_value=(0,Q.tolist())
        for name in ('set_network_exception_handle','servo_move_use_joint_LPF','servo_move_enable','servo_j','motion_abort'): getattr(b.rc,name).return_value=(0,)
        b.rc.is_in_servomove.side_effect=[(0,0),(0,1),(0,0)]
        b.last_poll=time.perf_counter(); b.stream_fb=Feedback(Q,time.perf_counter(),1); b.feedback_error=None
        b.start(); b.send(Motion.rest(Q)); outcome=b.stop()
        self.assertTrue(outcome['servo_exited']); self.assertFalse(outcome['physical_stop_confirmed'])
        names=[call[0] for call in b.rc.mock_calls]
        self.assertNotIn('servo_move_use_joint_LPF',names)
        self.assertLess(names.index('servo_j'),names.index('motion_abort'))
        b.rc.servo_j.assert_called_once_with(Q.tolist(),0,1)
    def test_tcp_jacobian_transform_matches_finite_difference(self):
        from v2_servo.kinematics import ArmKinematics,CalibratedKinematics,rotate
        base=np.eye(4); base[:3,:3]=rotate([0,0,1],.6); base[:3,3]=[.2,-.1,.3]
        tip=np.eye(4); tip[:3,3]=[.02,-.06,.2]
        arm=CalibratedKinematics(ArmKinematics(ROOT/'models/assembly_reference.urdf'),base,tip,[.01]*6)
        t,j,_=arm.forward(Q)
        for i in range(6):
            q=Q.copy(); q[i]+=1e-6
            np.testing.assert_allclose((arm.forward(q)[0][:3,3]-t[:3,3])/1e-6,j[:3,i],atol=1e-6)

if __name__=='__main__': unittest.main()
