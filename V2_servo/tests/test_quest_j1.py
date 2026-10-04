import threading,time,unittest,json,socket
import numpy as np
from v2_servo.configuration import load
from v2_servo.teleop.quest_j1 import QuestJ1Mapping,QuestJ1PoseMapping,QuestJ1Receiver,QuestJ1Jog,ready_input,input_readiness,QuestInputRehearsal
from v2_servo.teleop.static_hold import run_quest_j1
from test_static_hold import FakeStaticRC


class QuestJ1Tests(unittest.TestCase):
    def setUp(self):self.cfg=load();self.q=np.array([.02,1.35,-1.71,1.74,2.76,.56])
    def frame(self,t,grip=.9,y=1,deadman=True):
        return {'stamp':t,'sequence':int(t*1000),'source_time_s':t,'grip':grip,'stick_y':y,'deadman':deadman,'state':'Grip跟随'}
    def test_held_grip_at_start_cannot_move(self):
        p=QuestJ1Jog(self.q,self.cfg)
        for i in range(100):p.step(self.frame(i*.008),i*.008)
        np.testing.assert_array_equal(p.state.q,self.q)
    def test_envelope_both_directions_and_release_jerk_are_bounded(self):
        p=QuestJ1Jog(self.q,self.cfg);p.released=True
        maximum=0.
        for i in range(1800):
            t=i*.008;y=1 if i<700 else -1
            m=p.step(self.frame(t,y=y),t);p.check(m.q,m.q,[0]*6)
            maximum=max(maximum,m.q[0]-self.q[0])
        self.assertGreater(maximum,np.deg2rad(.03))
        self.assertLessEqual(maximum,np.deg2rad(.1))
        for i in range(100):
            t=15+i*.008;m=p.step(self.frame(t,grip=0,y=0,deadman=False),t)
            p.check(m.q,m.q,[0]*6)
        self.assertLess(np.max(np.abs(m.v)),1e-8)
    def test_stale_input_locks_and_never_rearms_on_new_grip(self):
        p=QuestJ1Jog(self.q,self.cfg);p.released=True
        for i in range(80):p.step(self.frame(i*.008),i*.008)
        before=p.state.q[0]
        p.step(self.frame(.64),.8)
        for i in range(100):p.step(self.frame(.8+i*.008),.8+i*.008)
        self.assertTrue(p.locked);self.assertTrue(p.finished)
        self.assertLess(np.max(np.abs(p.state.v)),1e-8)
        self.assertGreater(p.state.q[0],before)
    def test_stop_event_is_latched_even_if_input_slot_overwritten(self):
        p=QuestJ1Jog(self.q,self.cfg);p.released=True
        p.step(self.frame(0),0,software_stop=True)
        self.assertTrue(p.locked);self.assertTrue(p.finished)
    def test_readiness_rejects_no_tracking_held_click_grip_and_stick(self):
        frame=self.frame(1,grip=0,y=0,deadman=False)
        self.assertTrue(ready_input(frame,1.01))
        for change in ({'tracking_valid':False},{'grip':.9},{'stick_y':.5},
                       {'stick_clicked':True},{'software_stop':True},{'stamp':.5}):
            with self.subTest(change=change):self.assertFalse(ready_input(dict(frame,**change),1.01))
        self.assertIn('WAIT_UDP',input_readiness(None,1,'waiting'))
        self.assertIn('WAIT_RIGHT_TRACKING',input_readiness(dict(frame,tracking_valid=False),1.01))
    def test_input_rehearsal_requires_actual_stick_then_fresh_release(self):
        rehearsal=QuestInputRehearsal();sequence=0
        def feed(grip,y,deadman,n):
            nonlocal sequence
            for _ in range(n):
                sequence+=1;t=sequence*.014
                frame=dict(self.frame(t,grip,y,deadman),sequence=sequence,tracking_valid=True)
                rehearsal.step(frame,t)
        feed(0,0,False,10);self.assertEqual(rehearsal.stage,'GRIP_STICK_UP')
        feed(1,0,True,30);self.assertFalse(rehearsal.passed);self.assertEqual(rehearsal.stage,'GRIP_STICK_UP')
        feed(1,.6,True,3);self.assertEqual(rehearsal.stage,'RELEASE_FINAL')
        feed(0,0,False,10);self.assertTrue(rehearsal.passed)
    def test_repeated_or_untracked_frames_cannot_pass_rehearsal(self):
        rehearsal=QuestInputRehearsal();frame=dict(self.frame(1,0,0,False),sequence=1,tracking_valid=True)
        for _ in range(40):rehearsal.step(frame,1)
        self.assertEqual(rehearsal.count,1);self.assertEqual(rehearsal.stage,'RELEASE_INITIAL')
        for i in range(20):rehearsal.step(dict(frame,sequence=i+2,tracking_valid=False),1)
        self.assertFalse(rehearsal.passed)
    def test_pose_input_rebinds_on_grip_and_preserves_raw_stick(self):
        mapper=QuestJ1PoseMapping(np.eye(3))
        def decode(seq,grip,y):
            packet={'version':2,'sequence':seq,'unity_time_s':1+seq*.014,
                    'right':{'connected':True,'tracked':True,'pose_valid':True,
                             'position_m':{'x':0.,'y':y,'z':0.},'rotation_xyzw':[0,0,0,1],
                             'grip':grip,'grip_available':True,'thumbstick':{'x':0,'y':0},
                             'thumbstick_available':True,'stick_click_available':True,'thumbstick_click':False}}
            return mapper.decode(packet,('127.0.0.1',1),seq*.014)
        self.assertEqual(decode(0,.9,.4)['axis_y'],0)
        decode(1,0,.4);self.assertEqual(decode(2,.9,.4)['axis_y'],0)
        moved=decode(3,.9,.42);self.assertAlmostEqual(moved['axis_y'],1)
        self.assertEqual(moved['stick_y'],0)
        self.assertEqual(decode(4,0,.42)['axis_y'],0);self.assertIsNone(mapper.reference_y)
        self.assertEqual(decode(5,.9,.43)['axis_y'],0)
        self.assertAlmostEqual(decode(6,.9,.41)['axis_y'],-1)
    def test_pose_mode_uses_same_single_joint_envelope_and_release_stop(self):
        plan=QuestJ1Jog(self.q,self.cfg);plan.released=True;peak=0.
        for i in range(700):
            t=i*.008;frame=dict(self.frame(t,grip=.9,y=0),axis_y=1.,input_mode='pose')
            motion=plan.step(frame,t);plan.check(motion.q,motion.q,[0]*6)
            peak=max(peak,motion.q[0]-self.q[0])
            np.testing.assert_array_equal(motion.q[1:],self.q[1:])
        self.assertGreater(peak,np.deg2rad(.02));self.assertLessEqual(peak,np.deg2rad(.1))
        for i in range(100):
            t=6+i*.008;motion=plan.step(dict(self.frame(t,0,0,False),axis_y=0,input_mode='pose'),t)
        self.assertLess(np.max(np.abs(motion.v)),1e-8)
    def test_private_udp_pose_rehearsal_records_packets_without_robot(self):
        audited=[];receiver=QuestJ1Receiver(0,input_mode='pose',audit=lambda **row:audited.append(row))
        rehearsal=QuestInputRehearsal();destination=receiver.socket.getsockname()
        try:
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
                for seq in range(36):
                    held=12<=seq<24;y=.4+(min(max(seq-12,0),6)*.004 if held else 0)
                    packet={'version':2,'sequence':seq,'unity_time_s':1+seq*.014,
                            'right':{'connected':True,'tracked':True,'pose_valid':True,
                                     'position_m':{'x':0.,'y':y,'z':0.},'rotation_xyzw':[0,0,0,1],
                                     'grip':.9 if held else 0.,'grip_available':True,
                                     'thumbstick':{'x':0,'y':0},'thumbstick_available':True,
                                     'stick_click_available':True,'thumbstick_click':False}}
                    sender.sendto(json.dumps(packet).encode(),destination)
                    until=time.perf_counter()+.2
                    while receiver.latest is None or receiver.latest['sequence']!=seq:
                        if time.perf_counter()>until:self.fail('private UDP sample missing: '+receiver.error)
                        time.sleep(.001)
                    rehearsal.step(receiver.latest,time.perf_counter());time.sleep(.014)
            self.assertTrue(rehearsal.passed);self.assertEqual(receiver.received,36)
            self.assertEqual(receiver.rejected,0);self.assertEqual(len(audited),36)
            self.assertTrue(all(row['peer'][0]=='127.0.0.1' for row in audited))
            self.assertEqual(receiver.latest['axis_y'],0)
        finally:receiver.close()
    def test_mapper_requires_available_real_controls_and_release(self):
        mapper=QuestJ1Mapping(np.eye(3))
        def packet(seq,grip):
            return {'version':2,'sequence':seq,'unity_time_s':1+seq*.016,
                    'right':{'connected':True,'tracked':True,'pose_valid':True,'position_m':[0,0,0],
                             'rotation_xyzw':[0,0,0,1],'grip':grip,'grip_available':True,
                             'thumbstick':{'x':0,'y':.5},'thumbstick_available':True,
                             'stick_click_available':True,'thumbstick_click':False}}
        self.assertFalse(mapper.decode(packet(0,.9),('127.0.0.1',1),0)['deadman'])
        mapper.decode(packet(1,0),('127.0.0.1',1),.016)
        self.assertTrue(mapper.decode(packet(2,.9),('127.0.0.1',1),.032)['deadman'])
        bad=packet(3,.9);bad['right']['thumbstick_available']=False
        with self.assertRaises(ValueError):mapper.decode(bad,('127.0.0.1',1),.048)
    def test_entire_simulated_h3_grip_motion_release_and_exit(self):
        rc=FakeStaticRC();rc.commissioning_edg_mode=0
        def send(q,mode,step):rc.q=q[:];rc.calls.append(('send',q[:],mode,step));return (0,)
        rc.servo_j=send
        class Quest:
            error='';origin=None
            @property
            def latest(self):
                now=time.perf_counter()
                if self.origin is None:self.origin=now
                elapsed=now-self.origin;grip=.9 if .2<elapsed<1.1 else 0.
                return {'stamp':now,'sequence':int(elapsed*1000),'source_time_s':elapsed,
                        'grip':grip,'stick_y':1 if grip else 0,'deadman':bool(grip),'state':'test'}
            def take_software_stop(self):return self.origin is not None and time.perf_counter()-self.origin>1.4
        report=run_quest_j1(rc,self.cfg,{'edg_client_ip':'192.168.110.96','edg_port':30003},
                            {'tool_id':1,'tcp_mm_rad':[-23.703,-60.146,200.963,0,0,np.pi/4],
                             'payload_kg':2.028,'cog_mm':[6.641,-4.455,67.216]},
                            threading.Event(),lambda **row:None,Quest())
        self.assertTrue(report['passed_quest_j1_session_only'],report.get('failure'))
        self.assertFalse(rc.servo);self.assertFalse(rc.edg)
        self.assertFalse(report['quest_6dof_approved'])
        self.assertGreater(max(r['q_target'][0] for r in report['samples'])-self.q[0],np.deg2rad(.005))
