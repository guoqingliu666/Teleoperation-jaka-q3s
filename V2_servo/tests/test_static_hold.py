import threading
import time
import unittest
from unittest.mock import Mock,patch

import numpy as np

from v2_servo.configuration import load
from v2_servo.safety import SafetyFault
from v2_servo.teleop.static_hold import StaticTarget,StaticFeedback,run_static_hold,run_single_axis_roundtrip
from v2_servo.teleop.single_axis import J1Roundtrip
from v2_servo.teleop.static_hold import StaticStatusWatcher


class FakeStaticRC:
    def __init__(self):
        self.q=[.02,1.35,-1.71,1.74,2.76,.56]
        self.servo=False;self.calls=[];self.edg=False;self.epoch=time.perf_counter()
    def get_robot_status(self):
        s=[0]*24;s[2]=s[3]=s[22]=1;s[19]=self.q[:];return (0,s)
    def is_in_servomove(self):return (0,int(self.servo))
    def is_in_pos(self):return (0,1)
    def get_tool_id(self):return (0,1)
    def get_tool_data(self,i):return (0,i,[-23.703,-60.146,200.963,0,0,np.pi/4])
    def get_payload(self):return (0,(2.028,[6.641,-4.455,67.216]))
    def get_collision_level(self):return (0,5)
    def get_actual_joint_position(self):return (0,self.q[:])
    def get_actual_tcp_position(self):return (0,[500,200,250,0,0,0])
    def get_sdk_version(self):return (0,'FAKE')
    def kine_forward(self,q):return (0,[500*np.cos(q[0]),500*np.sin(q[0]),250,0,0,0])
    def get_motion_status(self):return (0,[0]*11)
    def get_robot_status_simple(self):return (0,[0,'',1,1])
    def set_network_exception_handle(self,ms,action):self.calls.append(('network',ms,action));return (0,)
    def edg_init_extend(self,on,ip,port,mode):self.edg=on;self.calls.append(('edg',on));return (0,)
    def edg_get_stat(self,i):return (0,(self.q[:],[0]*6,[999]*6,[500,200,250,0,0,0],[],[]))
    def edg_stat_details(self):
        ns=int((time.perf_counter()-self.epoch)*1e9);return (0,(100+ns//1_000_000_000,ns%1_000_000_000,ns))
    def servo_move_enable(self,on):self.servo=on;self.calls.append(('servo',on));return (0,)
    def servo_j(self,q,mode,step):self.calls.append(('send',q[:],mode,step));return (0,)
    def motion_abort(self):self.calls.append(('abort',));return (0,)


class StaticHoldTests(unittest.TestCase):
    def test_edg0_readonly_probe_does_not_send_or_enable_servo(self):
        from v2_servo.teleop.static_hold import probe_edg0_safety_readonly
        report=probe_edg0_safety_readonly(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:None)
        self.assertTrue(report['passed_readonly'],report)
        self.assertEqual(self.rc.calls,[('edg',True),('edg',False)])
        self.assertEqual(report['servo_commands'],0)
    def setUp(self):
        self.cfg=load();self.rc=FakeStaticRC()
        self.options={'edg_client_ip':'192.168.110.96','edg_port':30003}
        self.expected={'tool_id':1,'tcp_mm_rad':[-23.703,-60.146,200.963,0,0,np.pi/4],
                       'payload_kg':2.028,'cog_mm':[6.641,-4.455,67.216]}
    def test_static_gate_rejects_changed_target_even_within_joint_limits(self):
        target=StaticTarget(self.rc.q,self.cfg['limits'])
        changed=np.array(self.rc.q);changed[0]+=1e-10
        with self.assertRaisesRegex(SafetyFault,'change'):
            target.check(changed,self.rc.q,[0]*6)
        with self.assertRaises(ValueError):target.q[0]=0
    def test_source_repetition_is_not_fresh_feedback(self):
        self.rc.edg_stat_details=lambda:(0,(100,0,1))
        feedback=StaticFeedback(self.rc);feedback.sample();time.sleep(.026)
        with self.assertRaisesRegex(SafetyFault,'stopped advancing'):feedback.sample()
    def test_h1_sends_only_current_measured_q_and_cleans_up(self):
        report=run_static_hold(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:None)
        self.assertTrue(report['passed_static_only'],report)
        self.assertFalse(report['continuous_teleop_approved'])
        self.assertFalse(report['absolute_feedback_age_validated'])
        self.assertFalse(self.rc.servo);self.assertFalse(self.rc.edg)
        goals=[r for r in self.rc.calls if r[0]=='send']
        self.assertGreaterEqual(len(goals),30)
        self.assertTrue(all(row[1]==self.rc.q and row[2:]==(0,1) for row in goals))
    def test_deadline_failure_preserves_lateness_and_exits_before_send(self):
        class LateTimer:
            spin_s=.0005;period=.008
            def __init__(self,*args,**kwargs):pass
            def wait(self):return time.perf_counter(),.006
        events=[]
        with patch('v2_servo.teleop.static_hold.Deadline',LateTimer):
            report=run_static_hold(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:events.append(row))
        self.assertEqual(report['servo_commands'],0);self.assertTrue(report['servo_exited'])
        self.assertAlmostEqual(report['deadline_miss']['lateness_ms'],6)
        self.assertTrue(any(row.get('event')=='deadline_miss' for row in events))
        self.assertFalse(report['passed_static_only'])
    def test_slow_preparation_is_rejected_before_a_stale_target_send(self):
        from v2_servo.teleop.motion import VelocityServo
        original=VelocityServo.step
        def slow_step(plan,*args,**kwargs):time.sleep(.007);return original(plan,*args,**kwargs)
        with patch('v2_servo.teleop.static_hold.VelocityServo.step',slow_step):
            report=run_static_hold(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:None)
        self.assertEqual(report['servo_commands'],0);self.assertTrue(report['servo_exited'])
        self.assertIn('pre-send deadline miss',report['failure'])
        self.assertGreater(report['deadline_miss']['preparation_ms'],4)
        self.assertGreater(report['deadline_miss']['stage_ms']['planning'],4)
        self.assertEqual(set(report['deadline_miss']['stage_ms']),
                         {'raw_safety','feedback','planning','limit_check'})
    def test_raw_collision_prevents_mode_change(self):
        original=self.rc.get_robot_status
        def fault():
            ret=original();ret[1][5]=1;return ret
        self.rc.get_robot_status=fault
        report=run_static_hold(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:None)
        self.assertIn('raw controller safety fault',report['failure'])
        self.assertEqual(self.rc.calls,[])
    def test_operator_stop_prevents_network_and_servo_setters(self):
        stop=threading.Event();stop.set()
        report=run_static_hold(self.rc,self.cfg,self.options,self.expected,stop,lambda **row:None)
        self.assertEqual(report['servo_commands'],0)
        self.assertEqual(self.rc.calls,[])
    def test_h2_entire_roundtrip_is_filtered_bounded_and_returns(self):
        plan=J1Roundtrip(self.rc.q,self.cfg);rows=[]
        for _ in range(1000):
            motion=plan.step();plan.check(motion.q,motion.q,[0.]*6);rows.append(motion)
            if plan.finished:break
        self.assertTrue(plan.finished)
        self.assertLess(np.max(np.abs(rows[-1].q-np.array(self.rc.q))),1e-7)
        self.assertGreater(max(r.q[0]-self.rc.q[0] for r in rows),np.deg2rad(.099))
        self.assertTrue(all(np.max(np.abs(r.q[1:]-self.rc.q[1:]))<1e-10 for r in rows))
        changed=rows[-1].q.copy();changed[1]+=.00001
        with self.assertRaisesRegex(SafetyFault,'fixed J1'):plan.check(changed,changed,[0.]*6)
    def test_h2_send_failure_exits_servo_without_automatic_return(self):
        self.rc.servo_j=lambda *args:(2,)
        report=run_single_axis_roundtrip(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:None)
        self.assertIn('servo_j failed',report['failure'])
        self.assertFalse(report['passed_single_axis_only'])
        self.assertTrue(report['servo_exited'])
        self.assertFalse(self.rc.servo);self.assertFalse(self.rc.edg)
        self.assertEqual(len(report['samples']),1)
        self.assertFalse(report['samples'][0]['native_send']['accepted'])
    def test_slow_accepted_send_is_recorded_and_stopped(self):
        self.rc.servo_j=lambda *args:(time.sleep(.015) or (0,))
        report=run_static_hold(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:None)
        self.assertIn('SDK call overrun:',report['failure'])
        self.assertEqual(report['servo_commands'],1)
        self.assertEqual(len(report['samples']),1)
        self.assertTrue(report['samples'][0]['native_send']['accepted'])
        self.assertGreater(report['samples'][0]['call_ms'],12)
        self.assertTrue(report['servo_exited'])
    def test_excessive_controller_queue_is_recorded_and_stopped(self):
        self.rc.servo_j=lambda *args:(0,3)
        report=run_static_hold(self.rc,self.cfg,self.options,self.expected,threading.Event(),lambda **row:None)
        self.assertIn('queue depth',report['failure'])
        self.assertEqual(report['servo_commands'],1)
        self.assertEqual(report['samples'][0]['native_send']['controller_queue_depth'],3)
        self.assertTrue(report['servo_exited'])
    def test_safety_poll_period_counts_query_cost(self):
        watcher=StaticStatusWatcher(self.rc,lambda **row:None)
        from unittest.mock import patch
        starts=[];clock=[0.]
        class Event:
            stopped=False
            def is_set(self):return self.stopped
            def set(self):self.stopped=True
            def wait(self,seconds):clock[0]+=seconds
        watcher.stop_event=Event()
        original=self.rc.get_motion_status
        def query():
            starts.append(clock[0]);clock[0]+=.035
            if len(starts)==3:watcher.stop_event.set()
            return original()
        self.rc.get_motion_status=query
        with patch('v2_servo.teleop.edg.time.perf_counter',side_effect=lambda:clock[0]):watcher.run()
        self.assertEqual(len(starts),3)
        # 查询分散，各类仍约100ms一次，35ms调用不会再累加135ms。
        self.assertTrue(np.allclose(np.diff(starts),.1,rtol=0,atol=1e-12))
