import json,threading,time,unittest
from unittest.mock import patch
import numpy as np
from v2_servo.configuration import ROOT,load
from v2_servo.safety import SafetyFault
from v2_servo.runtime import CommissioningPythonScheduling,WindowsTimerResolution
from v2_servo.teleop.quest_pose_live import QuestPoseLivePlanner
from v2_servo.teleop.static_hold import run_quest_pose_finite
from v2_servo.teleop.local_xyz import local_xyz_model
from v2_servo.teleop.commissioning_guidance import guidance_packet
from test_quest_tcp_z import SyntheticTCPRC


class QuestPoseLiveTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load();self.rc=SyntheticTCPRC()
        self.options=json.loads((ROOT/'config/teleop.json').read_text(encoding='utf-8'))
        self.options['quest_xyz_profile']=json.loads((ROOT/'config/quest_xyz_commissioning.json').read_text(encoding='utf-8'))
        self.expected=json.loads((ROOT/'config/site_observed_20260928.json').read_text(encoding='utf-8'))

    def test_wrong_receiver_user_frame_and_filter_refused_before_motion(self):
        for failure in ('input','frame','filter'):
            rc=SyntheticTCPRC();cfg=load()
            source=type('Source',(),{'input_mode':'pose6dof_shadow' if failure=='input' else 'pose6dof_live'})()
            if failure=='frame':rc.user_id=1
            if failure=='filter':cfg['filters']['enabled']=False
            if failure=='filter':
                with self.assertRaises(SafetyFault):run_quest_pose_finite(rc,cfg,self.options,self.expected,threading.Event(),lambda **row:None,source)
                self.assertEqual(rc.calls,[]);continue
            r=run_quest_pose_finite(rc,cfg,self.options,self.expected,threading.Event(),lambda **row:None,source)
            self.assertIsNotNone(r['failure']);self.assertEqual(r['servo_commands'],0);self.assertFalse(rc.servo)

    def test_model_shift_and_raw_faults_refused(self):
        model=local_xyz_model(self.rc,self.rc.q)
        p=QuestPoseLivePlanner(np.array(self.rc.q),self.cfg,model,model['tcp_origin_mm_rad'],self.options['quest_xyz_profile'])
        with self.assertRaises(SafetyFault):p.check(p.q0,p.q0,np.ones(6))
        tcp=p.origin_tcp.copy();tcp[0]+=.7
        with self.assertRaises(SafetyFault):p.check_tcp(tcp)
        tcp=p.origin_tcp.copy();tcp[3]+=.002
        with self.assertRaises(SafetyFault):p.check_tcp(tcp)
        with self.assertRaisesRegex(SafetyFault,'shifted'):
            QuestPoseLivePlanner(np.array(self.rc.q)+.001,self.cfg,model,model['tcp_origin_mm_rad'],self.options['quest_xyz_profile'])

    def test_display_must_use_explicit_live_scope(self):
        r=guidance_packet('LIVE','pose6dof_live',control_scope='TCP_6DOF')
        self.assertIn('联合6DoF',r['title']);self.assertIn('XYZ和姿态同时跟随',r['instruction'])
        with self.assertRaises(ValueError):guidance_packet('LIVE','pose6dof_shadow',control_scope='TCP_6DOF')
        with self.assertRaises(ValueError):guidance_packet('LIVE','pose6dof_live',control_scope='POSE_SHADOW')

    def test_full_finite_loop_with_fake_SDK_and_two_moving_releases(self):
        origin=[None]
        class Source:
            input_mode='pose6dof_live';rejected=0;error=''
            @property
            def latest(self):
                now=time.perf_counter()
                if origin[0] is None:origin[0]=now
                elapsed=now-origin[0];cycle=max(0,elapsed-1)%4;block=int(max(0,elapsed-1)//4)
                held=elapsed>=1 and cycle<2;sign=1 if block%2==0 else -1
                return dict(stamp=now,sequence=int(now*1000),tracking_valid=True,grip=float(held),deadman=held,
                    base_displacement_mm=[20*sign]*3,rotation_delta_deg=[5*sign]*3,stick_y=0.)
            def take_software_stop(self):return False
        scheduling=CommissioningPythonScheduling();timer=WindowsTimerResolution()
        try:
            scheduling.start();timer.start()
            r=run_quest_pose_finite(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:None,Source())
        finally:timer.close();scheduling.close()
        self.assertTrue(r['passed_quest_pose_finite_session_only'],r['failure'])
        self.assertTrue(r['servo_exited']);self.assertFalse(self.rc.servo)
        self.assertGreaterEqual(r['moving_release_count'],2)
        self.assertFalse(r['physical_release_stop_validated']);self.assertFalse(r['continuous_teleop_approved'])
        self.assertIn('combined_six',r['completion_reason'])

    def test_active_rejection_protective_stop_no_further_send(self):
        class Source:
            input_mode='pose6dof_live';error='injected malformed packet';reads=0
            @property
            def rejected(self):return int(self.reads>2)
            @property
            def latest(self):
                self.reads+=1;now=time.perf_counter()
                return dict(stamp=now,sequence=self.reads,tracking_valid=True,grip=0.,deadman=False,
                    base_displacement_mm=[0.]*3,rotation_delta_deg=[0.]*3,stick_y=0.)
            def take_software_stop(self):return False
        scheduling=CommissioningPythonScheduling();timer=WindowsTimerResolution()
        try:
            scheduling.start();timer.start()
            r=run_quest_pose_finite(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:None,Source())
        finally:timer.close();scheduling.close()
        self.assertIn('packet rejected',r['failure']);self.assertTrue(r['servo_exited'])
        self.assertFalse(r['passed_quest_pose_finite_session_only']);self.assertLess(r['servo_commands'],5)
