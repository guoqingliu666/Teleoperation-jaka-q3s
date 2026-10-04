import threading,time,unittest
from unittest.mock import patch
import numpy as np
from v2_servo.configuration import load,ROOT
from v2_servo.safety import SafetyFault
from v2_servo.teleop.local_rotation import local_rotation_model
from v2_servo.teleop.quest_tcp_rotation import QuestTCPRotationJog
from v2_servo.teleop.static_hold import run_quest_tcp_rotation
from test_quest_tcp_z import SyntheticTCPRC


class QuestRotationLiveTests(unittest.TestCase):
    def setUp(self):
        self.rc=SyntheticTCPRC();self.cfg=load();self.model=local_rotation_model(self.rc,self.rc.q,'RX')
    def plan(self):return QuestTCPRotationJog(self.rc.q,self.cfg,self.model,self.model['tcp_origin_mm_rad'])
    def frame(self,t,grip=1.,angle=5.,tracked=True):
        return dict(stamp=t,sequence=int(t*1000),source_time_s=t,grip=grip,deadman=grip>.7,
                    tracking_valid=tracked,input_mode='rotation_shadow',rotation_delta_deg=[angle,0,0],software_stop=False)
    def test_positive_negative_motion_and_moving_release(self):
        p=self.plan();p.step(self.frame(0,0),0);angles=[]
        for i in range(1200):
            t=(i+1)*.008;block=i//300;held=i%300<120;angle=5 if block%2==0 else -5
            m=p.step(self.frame(t,1 if held else 0,angle),t)
            p.check(m.q,m.q,np.zeros(6));p.check_tcp(self.rc.kine_forward(m.q)[1]);angles.append(p.angle)
        self.assertGreater(np.ptp(angles),.01);self.assertGreater(p.release_stops,2)
        self.assertTrue(any(e['moving_at_release'] for e in p.release_events))
        self.assertTrue(all(e['command_stop_time_s']<1 and not e['physical_stop_validated'] for e in p.release_events))
        self.assertLess(abs(p.angle),.02);self.assertFalse(p.finished)
    def test_stale_and_duration_stop_lock_and_finish_within_budget(self):
        for reason in ('stale','finish','tracking'):
            p=self.plan();p.step(self.frame(0,0),0)
            for i in range(70):p.step(self.frame((i+1)*.008), (i+1)*.008)
            for i in range(125):
                t=.568+i*.008;f=None if reason=='stale' else self.frame(t,tracked=reason!='tracking')
                p.step(f,t,finish=reason=='finish')
                if p.finished:break
            self.assertTrue(p.finished);self.assertTrue(p.locked);self.assertEqual(p.phase,'LOCKED_STOP')
            self.assertLess(i*.008,1.);self.assertEqual(p.release_events,[])
    def test_raw_tcp_joint_and_speed_checks(self):
        p=self.plan();p.step(self.frame(0,0),0)
        tcp=p.origin_tcp.copy();tcp[0]+=.101
        with self.assertRaisesRegex(SafetyFault,'translation'):p.check_tcp(tcp)
        tcp=p.origin_tcp.copy();tcp[4]+=.001
        with self.assertRaisesRegex(SafetyFault,'orientation'):p.check_tcp(tcp)
        with self.assertRaisesRegex(SafetyFault,'speed'):p.check(p.q0,p.q0,np.full(6,np.deg2rad(.51)))
        off=p.q0.copy();off[0]+=.00001
        with self.assertRaisesRegex(SafetyFault,'angular domain'):p.check(off,p.q0,np.zeros(6))
    def test_held_start_cannot_arm_and_other_axes_not_released(self):
        p=self.plan()
        for i in range(100):p.step(self.frame(i*.008),i*.008)
        np.testing.assert_array_equal(p.state.q,p.q0)
        model=dict(self.model,axis='INVALID')
        with self.assertRaisesRegex(SafetyFault,'reviewed rotation axis'):QuestTCPRotationJog(self.rc.q,self.cfg,model,self.model['tcp_origin_mm_rad'])
    def test_common_loop_rejects_user_frame_before_modes(self):
        self.rc.user_id=1
        source=type('Source',(),{'input_mode':'rotation_shadow','rejected':0})()
        import json
        options=json.loads((ROOT/'config/teleop.json').read_text());expected=json.loads((ROOT/'config/site_observed_20260928.json').read_text())
        r=run_quest_tcp_rotation(self.rc,self.cfg,options,expected,threading.Event(),lambda **row:None,source)
        self.assertIn('user frame 0',r['failure']);self.assertEqual(r['servo_commands'],0)
        self.assertFalse(r['passed_quest_tcp_rotation_session_only'])
    def test_each_axis_motion_release_and_cross_axis_feedback_guard(self):
        for axis in ('RX','RY','RZ'):
            index=('RX','RY','RZ').index(axis);model=local_rotation_model(self.rc,self.rc.q,axis)
            p=QuestTCPRotationJog(self.rc.q,self.cfg,model,model['tcp_origin_mm_rad'])
            f=self.frame(0,0);p.step(f,0);angles=[]
            for i in range(650):
                t=(i+1)*.008;held=i%300<120;f=self.frame(t,1 if held else 0)
                f['rotation_delta_deg']=[0.,0.,0.];f['rotation_delta_deg'][index]=5 if i<300 else -5
                m=p.step(f,t);p.check(m.q,m.q,np.zeros(6));p.check_tcp(self.rc.kine_forward(m.q)[1]);angles.append(p.angle)
                self.assertEqual(p.phase,'GRIP_'+axis if held else 'RELEASE_BRAKE')
            self.assertGreater(np.ptp(angles),.01);self.assertGreater(len(p.release_events),1)
            tcp=p.origin_tcp.copy();tcp[3+(index+1)%3]+=.001
            with self.assertRaisesRegex(SafetyFault,'orientation envelope'):p.check_tcp(tcp)
