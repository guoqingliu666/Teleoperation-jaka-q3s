import threading,time,unittest
import numpy as np
from v2_servo.configuration import load
from v2_servo.safety import SafetyFault
from v2_servo.runtime import CommissioningPythonScheduling,WindowsTimerResolution
from v2_servo.teleop.official_local import local_translation_model
from v2_servo.teleop.quest_tcp_axis import QuestTCPAxisJog
from v2_servo.teleop.static_hold import run_quest_tcp_axis
from test_quest_tcp_z import SyntheticTCPRC


class QuestTCPAxisTests(unittest.TestCase):
    def setUp(self):self.rc=SyntheticTCPRC();self.q=np.array(self.rc.q);self.cfg=load()
    def frame(self,t,grip=1.,axis=1.):
        return {'stamp':t,'sequence':int(t*1000),'grip':grip,'stick_y':0.,'axis_y':axis,
                'deadman':grip>.7,'tracking_valid':True,'state':'Grip跟随' if grip>.7 else '松开/等待Grip','input_mode':'pose'}
    def test_each_base_axis_has_correct_units_and_keeps_other_TCP_axes(self):
        for axis in ('X','Y'):
            with self.subTest(axis=axis):
                model=local_translation_model(self.rc,self.q,axis)
                p=QuestTCPAxisJog(self.q,self.cfg,model,model['tcp_origin_mm_rad']);p.released=True
                origin=np.array(model['tcp_origin_mm_rad']);index='XYZ'.index(axis)
                for i in range(160):
                    t=i*.008;m=p.step(self.frame(t),t);p.check(m.q,m.q,np.zeros(6))
                    tcp=np.array(self.rc.kine_forward(m.q)[1]);p.check_tcp(tcp)
                    self.assertLess(np.linalg.norm(np.delete(tcp[:3]-origin[:3],index)),1e-6)
                    self.assertLess(np.linalg.norm(tcp[3:]-origin[3:]),1e-6)
                self.assertGreater(tcp[index]-origin[index],.02)
                self.assertLessEqual(tcp[index]-origin[index],model['maximum_axis_mm'])
                for i in range(160):
                    t=1.28+i*.008;p.step(self.frame(t,0,0),t)
                self.assertEqual(p.release_stops,1);self.assertLess(np.max(np.abs(p.state.v)),1e-8)
    def test_wrong_measured_axis_and_orientation_fail_before_send(self):
        model=local_translation_model(self.rc,self.q,'X');p=QuestTCPAxisJog(self.q,self.cfg,model,model['tcp_origin_mm_rad'])
        tcp=np.array(model['tcp_origin_mm_rad']);tcp[2]+=.3
        with self.assertRaisesRegex(SafetyFault,'H5 raw Tool1 TCP X'):p.check_tcp(tcp)
        tcp=np.array(model['tcp_origin_mm_rad']);tcp[4]+=.01
        with self.assertRaisesRegex(SafetyFault,'orientation'):p.check_tcp(tcp)
    def test_joint_trust_region_limits_cartesian_extent(self):
        class HighGainRC(SyntheticTCPRC):
            def kine_forward(self,q):
                d=np.array(q)-self.origin
                return (0,[500+50*d[0],200+400*d[1],250+600*d[2],d[3],d[4],d[5]])
        rc=HighGainRC();model=local_translation_model(rc,rc.q,'X')
        self.assertGreater(model['maximum_axis_mm'],.05);self.assertLess(model['maximum_axis_mm'],.5)
        self.assertLessEqual(np.max(np.abs(np.array(model['joint_gain_rad_per_mm_X'])*model['maximum_axis_mm'])),np.deg2rad(.1)+1e-9)
    def test_new_axis_cannot_replace_Z_implicitly(self):
        model=local_translation_model(self.rc,self.q,'Z')
        with self.assertRaisesRegex(ValueError,'H5 axis'):QuestTCPAxisJog(self.q,self.cfg,model,model['tcp_origin_mm_rad'])
        with self.assertRaises(ValueError):local_translation_model(self.rc,self.q,'RZ')
    def test_H5_nonbase_coordinate_zero_commands(self):
        self.rc.user_id=1
        quest=type('Quest',(),{'input_mode':'pose'})()
        r=run_quest_tcp_axis(self.rc,self.cfg,{'edg_client_ip':'192.168.110.96','edg_port':30003,'commissioning_tcp_axis':'X'},
            {'tool_id':1,'tcp_mm_rad':[-23.703,-60.146,200.963,0,0,np.pi/4],'payload_kg':2.028,'cog_mm':[6.641,-4.455,67.216]},threading.Event(),lambda **row:None,quest)
        self.assertIn('H5 requires',r['failure']);self.assertEqual(r['servo_commands'],0);self.assertEqual(self.rc.calls,[])
    def test_shared_H5_loop_reports_selected_axis_and_released_stop(self):
        owner=self
        class Quest:
            input_mode='pose';error='';origin=None
            @property
            def latest(self):
                now=time.perf_counter()
                if self.origin is None:self.origin=now
                elapsed=now-self.origin
                return owner.frame(now,1 if .6<elapsed<1.8 else 0,1 if .6<elapsed<1.8 else 0)
            def take_software_stop(self):return self.origin is not None and time.perf_counter()-self.origin>2.8
        policy=CommissioningPythonScheduling();timer=WindowsTimerResolution()
        try:
            policy.start();timer.start()
            r=run_quest_tcp_axis(self.rc,self.cfg,{'edg_client_ip':'192.168.110.96','edg_port':30003,'commissioning_tcp_axis':'Y'},
                {'tool_id':1,'tcp_mm_rad':[-23.703,-60.146,200.963,0,0,np.pi/4],'payload_kg':2.028,'cog_mm':[6.641,-4.455,67.216]},threading.Event(),lambda **row:None,Quest())
        finally:timer.close();policy.close()
        self.assertTrue(r['passed_quest_tcp_axis_session_only'],r['failure']);self.assertEqual(r['tcp_axis'],'Y')
        self.assertFalse(r['quest_6dof_approved']);self.assertTrue(r['servo_exited']);self.assertEqual(r['release_stops'],1)
        self.assertGreater(r['command_TCP_Y_span_mm'],.02)
