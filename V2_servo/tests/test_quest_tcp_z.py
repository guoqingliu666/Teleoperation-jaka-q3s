import threading,time,unittest
import numpy as np
from v2_servo.configuration import load
from v2_servo.safety import SafetyFault
from v2_servo.runtime import CommissioningPythonScheduling,WindowsTimerResolution
from v2_servo.teleop.official_local import local_z_model
from v2_servo.teleop.quest_tcp_z import QuestTCPZJog
from v2_servo.teleop.static_hold import run_quest_tcp_z
from test_static_hold import FakeStaticRC


class SyntheticTCPRC(FakeStaticRC):
    """仅测试单位、轴耦合与停止语义，不是真实机器人动力学。"""
    commissioning_edg_mode=0
    def __init__(self):super().__init__();self.origin=np.array(self.q);self.user_id=0
    def get_user_frame_id(self):return (0,self.user_id)
    def kine_forward(self,q):
        d=np.array(q)-self.origin
        return (0,[500+500*d[0],200+400*d[1],250+600*d[2],d[3]+.2*d[2],d[4],d[5]])
    def get_actual_tcp_position(self):return self.kine_forward(self.q)
    def edg_get_stat(self,i):return (0,(self.q[:],[0.]*6,[999.]*6,self.kine_forward(self.q)[1],[],[]))
    def servo_j(self,q,mode,step):self.q=q[:];return super().servo_j(q,mode,step)


class QuestTCPZTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load();self.rc=SyntheticTCPRC();self.q=np.array(self.rc.q)
        self.model=local_z_model(self.rc,self.q)
    def frame(self,t,grip=1.,axis=1.):
        return {'stamp':t,'sequence':int(t*1000),'source_time_s':t,'grip':grip,
                'stick_y':0.,'axis_y':axis,'deadman':grip>.7,'tracking_valid':True,
                'state':'Grip跟随' if grip>.7 else '松开/等待Grip','input_mode':'pose'}
    def plan(self):return QuestTCPZJog(self.q,self.cfg,self.model,self.model['tcp_origin_mm_rad'])
    def test_tool_Z_projection_couples_joints_and_keeps_orientation(self):
        p=self.plan();p.released=True;peak=0.
        for i in range(180):
            t=i*.008;m=p.step(self.frame(t),t);p.check(m.q,m.q,np.zeros(6))
            tcp=self.rc.kine_forward(m.q)[1];p.check_tcp(tcp);peak=max(peak,tcp[2]-250)
            self.assertAlmostEqual(tcp[0],500);self.assertAlmostEqual(tcp[1],200)
            self.assertLess(abs(tcp[3]),1e-7)
        self.assertGreater(peak,.02);self.assertLessEqual(peak,.5)
        self.assertGreater(abs(m.q[2]-self.q[2]),0);self.assertGreater(abs(m.q[3]-self.q[3]),0)
        for i in range(100):
            t=1.44+i*.008;m=p.step(self.frame(t,grip=0,axis=0),t);p.check(m.q,m.q,np.zeros(6))
        self.assertLess(np.max(np.abs(m.v)),1e-8);self.assertEqual(p.release_stops,1)
    def test_held_start_and_timeout_cannot_arm(self):
        p=self.plan()
        for i in range(20):p.step(self.frame(i*.008),i*.008)
        np.testing.assert_array_equal(p.state.q,self.q)
        p.released=True;p.step(self.frame(.16),.16);p.step(self.frame(.16),.3)
        for i in range(100):p.step(self.frame(.3+i*.008),.3+i*.008)
        self.assertTrue(p.finished);self.assertIn('stale',p.stop_reason)
    def test_command_off_line_and_feedback_wrong_axis_rejected(self):
        p=self.plan();candidate=self.q.copy();candidate[0]+=.0001
        with self.assertRaisesRegex(SafetyFault,'line'):p.check(candidate,self.q,np.zeros(6))
        tcp=np.array(self.model['tcp_origin_mm_rad']);tcp[0]+=.3
        with self.assertRaisesRegex(SafetyFault,'translation'):p.check_tcp(tcp)
    def test_singular_FK_is_refused_before_servo(self):
        with self.assertRaisesRegex(SafetyFault,'singularity'):local_z_model(FakeStaticRC(),self.q)
    def test_non_base_user_frame_refuses_all_motion(self):
        self.rc.user_id=1
        q=type('Quest',(),{'input_mode':'pose'})()
        r=run_quest_tcp_z(self.rc,self.cfg,{'edg_client_ip':'192.168.110.96','edg_port':30003},
            {'tool_id':1,'tcp_mm_rad':[-23.703,-60.146,200.963,0,0,np.pi/4],
             'payload_kg':2.028,'cog_mm':[6.641,-4.455,67.216]},threading.Event(),lambda **row:None,q)
        self.assertIn('frame 0',r['failure']);self.assertEqual(r['servo_commands'],0)
        self.assertFalse(self.rc.servo);self.assertEqual(self.rc.calls,[])
    def test_shared_H4_loop_live_semantics_in_fake_SDK(self):
        owner=self
        class Quest:
            input_mode='pose';error='';origin=None
            @property
            def latest(self):
                now=time.perf_counter()
                if self.origin is None:self.origin=now
                elapsed=now-self.origin
                return owner.frame(now,grip=1 if .6<elapsed<1.8 else 0,
                                   axis=1 if .6<elapsed<1.8 else 0)
            def take_software_stop(self):return self.origin is not None and time.perf_counter()-self.origin>2.8
        policy=CommissioningPythonScheduling();timer=WindowsTimerResolution()
        try:
            policy.start();timer.start()
            r=run_quest_tcp_z(self.rc,self.cfg,{'edg_client_ip':'192.168.110.96','edg_port':30003},
                {'tool_id':1,'tcp_mm_rad':[-23.703,-60.146,200.963,0,0,np.pi/4],
                 'payload_kg':2.028,'cog_mm':[6.641,-4.455,67.216]},threading.Event(),lambda **row:None,Quest())
        finally:timer.close();policy.close()
        self.assertTrue(r['passed_quest_tcp_z_session_only'],r['failure'])
        self.assertTrue(r['servo_exited']);self.assertFalse(self.rc.servo)
        self.assertFalse(r['quest_6dof_approved']);self.assertEqual(r['release_stops'],1)
        self.assertGreater(r['command_TCP_Z_span_mm'],.02);self.assertGreater(r['measured_TCP_Z_span_mm'],.02)
