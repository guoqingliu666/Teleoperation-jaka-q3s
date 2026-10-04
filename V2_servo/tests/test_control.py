import unittest, copy, math, tempfile, json, threading
from pathlib import Path
import numpy as np
from v2_servo.configuration import load,validate,ROOT
from v2_servo.filters import CommandFilter,LowPass,Notch,TelemetryFilter,CartesianFilter
from v2_servo.trajectory import Motion,Segment,plan,OnlinePlanner
from v2_servo.safety import Guard,Feedback,SafetyFault,State
from v2_servo.runtime import LatestSlot,Deadline,JsonLogger,run_stream
from v2_servo.jaka import JakaServo,checked_bool,decode_status
from v2_servo.inputs import InputGate,JointIntent,vla_joint_intent
from v2_servo.kinematics import ArmKinematics

class Base(unittest.TestCase):
    def setUp(self):
        self.c=load(); self.q=(np.array(self.c['limits']['lower_rad'])+self.c['limits']['upper_rad'])/2

class FilterTests(Base):
    def test_default_and_dc(self):
        self.assertTrue(self.c['filters']['enabled'])
        f=CommandFilter(self.c['filters'],.008,self.q)
        for _ in range(200): np.testing.assert_allclose(f.update(self.q),self.q,atol=1e-12)
    def test_offline_only_bypass(self):
        c=dict(self.c['filters'],enabled=False)
        with self.assertRaises(ValueError): CommandFilter(c,.008,self.q)
        np.testing.assert_allclose(CommandFilter(c,.008,self.q,comparison=True).update(self.q+1),self.q+1)
    def test_lowpass_attenuates_high_frequency(self):
        f=LowPass(6,.008,[0]); values=[float(f.update([math.sin(2*math.pi*30*.008*i)])[0]) for i in range(2000)]
        self.assertLess(np.std(values[500:]),.2)
    def test_hot_tune_retains_state(self):
        f=CommandFilter(self.c['filters'],.008,self.q)
        y=f.update(self.q+.1); f.tune_cutoff(9)
        np.testing.assert_allclose(f.lowpass.y,y)
        for x in [0,-1,62.5,float('nan')]:
            with self.subTest(x=x),self.assertRaises(ValueError): f.tune_cutoff(x)
    def test_notch(self):
        f=Notch(25,4,.008,[0]); y=[float(f.update([math.sin(2*math.pi*25*.008*i)])[0]) for i in range(2000)]
        self.assertLess(np.std(y[500:]),1e-6)
    def test_torque_raw_preserved_outside_filter(self):
        f=TelemetryFilter(self.c['filters'],.008,self.q)
        y=f.update(self.q,[100]*6)
        self.assertLess(y['torque_filtered'][0],100)
        self.assertIsNone(f.update(self.q,None)['torque_filtered'])
    def test_orientation_antipodal_and_wrap(self):
        f=CartesianFilter(6,.008,[0]*3,[0,0,0,1])
        _,q=f.update([0]*3,[0,0,0,-1]); np.testing.assert_allclose(q,[0,0,0,1])
        for angle in np.linspace(0,4*math.pi,500):
            _,q=f.update([0]*3,[0,0,math.sin(angle/2),math.cos(angle/2)])
            self.assertAlmostEqual(np.linalg.norm(q),1)
    def test_nonfinite_rejected(self):
        f=CommandFilter(self.c['filters'],.008,self.q)
        with self.assertRaises(ValueError): f.update([float('nan')]*6)

class TrajectoryTests(Base):
    def test_c3_endpoints(self):
        start=Motion(self.q,np.ones(6)*.01,np.ones(6)*.02,np.ones(6)*.03)
        s=Segment(start,self.q+.5,2)
        for a,b in zip(vars(s.sample(0)).values(),vars(start).values()): np.testing.assert_allclose(a,b,atol=1e-12)
        end=s.sample(2); np.testing.assert_allclose(end.q,self.q+.5)
        for x in [end.v,end.a,end.j]: np.testing.assert_allclose(x,0,atol=1e-10)
    def test_full_span_certification(self):
        lo=np.array(self.c['limits']['lower_rad']); hi=np.array(self.c['limits']['upper_rad'])
        s=plan(Motion.rest(lo),hi,self.c['limits'])
        self.assertTrue(s.valid(self.c['limits']))
        for t in np.linspace(0,s.duration,201):
            m=s.sample(t)
            self.assertTrue(np.all(m.q>=lo-1e-9) and np.all(m.q<=hi+1e-9))
    def test_retarget_splice_preserves_derivatives(self):
        p=OnlinePlanner(self.q,self.c['limits'],.008); p.retarget(self.q+.3)
        for _ in range(100): p.step()
        prior=p.state; p.retarget(self.q-.2)
        for a,b in zip(vars(prior).values(),vars(p.segment.sample(0)).values()): np.testing.assert_allclose(a,b,atol=1e-9)
    def test_reject_invalid_target(self):
        with self.assertRaises(ValueError): plan(Motion.rest(self.q),np.array(self.c['limits']['upper_rad'])+.01,self.c['limits'])
    def test_full_span_at_tenth_speed(self):
        lim=copy.deepcopy(self.c['limits']); lim['velocity_rad_s']=(np.array(lim['velocity_rad_s'])*.1).tolist()
        s=plan(Motion.rest(lim['lower_rad']),lim['upper_rad'],lim)
        self.assertTrue(s.valid(lim)); self.assertGreater(s.duration,180)
    def test_no_feasible_boundary_refused(self):
        q=np.array(self.c['limits']['upper_rad'])
        s=Motion(q,np.ones(6),np.zeros(6),np.zeros(6))
        with self.assertRaises(ValueError): plan(s,q,self.c['limits'])
    def test_monotone_rest_path(self):
        s=plan(Motion.rest(self.q),self.q+1,self.c['limits'])
        q=np.array([s.sample(t).q for t in np.linspace(0,s.duration,400)])
        self.assertGreaterEqual(np.diff(q,axis=0).min(),-1e-12)

class SafetyTests(Base):
    def guard(self):
        g=Guard(self.c); g.ready(); g.arm(); return g
    def test_good_sample(self): self.assertTrue(self.guard().check(Motion.rest(self.q),Feedback(self.q,1,1),1.008,1,1))
    def test_fault_matrix(self):
        variants=[('stale_feedback',{'feedback':Feedback(self.q,.9,1)}),('future_feedback',{'feedback':Feedback(self.q,1.1,1)}),
                  ('stale_input',{'input_at':.8}),('future_input',{'input_at':1.1}),('deadline',{'lateness':.005}),
                  ('deadman',{'deadman':False}),('scene',{'collision_clear':False})]
        for field in ['estop','collision','soft_limit']: variants.append((field,{'feedback':Feedback(self.q,1,1,**{field:True})}))
        for field in ['enabled','powered']: variants.append((field,{'feedback':Feedback(self.q,1,1,**{field:False})}))
        variants += [('error',{'feedback':Feedback(self.q,1,1,error=2)}),('nan',{'feedback':Feedback([float('nan')]*6,1,1)}),
                     ('tracking',{'feedback':Feedback(self.q+.2,1,1)}),('joint_boundary',{'motion':Motion.rest(np.array(self.c['limits']['upper_rad'])+.1)})]
        for name,change in variants:
            with self.subTest(name=name):
                kw=dict(motion=Motion.rest(self.q),feedback=Feedback(self.q,1,1),now=1.008,input_at=1,input_seq=1); kw.update(change)
                g=self.guard()
                with self.assertRaises(SafetyFault): g.check(**kw)
                self.assertEqual(g.state,State.FAULT)
                with self.assertRaises(SafetyFault): g.ready()
    def test_command_dynamics(self):
        for field,key in [('v','velocity_rad_s'),('a','acceleration_rad_s2'),('j','jerk_rad_s3')]:
            m=Motion.rest(self.q); setattr(m,field,np.array(self.c['limits'][key])*1.01)
            with self.subTest(field=field),self.assertRaises(SafetyFault): self.guard().check(m,Feedback(self.q,1,1),1.008,1,1)
    def test_raw_torque_spike_not_hidden(self):
        self.c['limits']['torque_nm']=[10]*6
        with self.assertRaisesRegex(SafetyFault,'raw_torque_limit'): self.guard().check(Motion.rest(self.q),Feedback(self.q,1,1,torque=[11]*6),1.008,1,1)
    def test_missing_torque_fails_closed(self):
        self.c['limits']['torque_nm']=[10]*6
        with self.assertRaisesRegex(SafetyFault,'torque_missing'): self.guard().check(Motion.rest(self.q),Feedback(self.q,1,1),1.008,1,1)
    def test_replayed_feedback_cannot_refresh_age(self):
        g=self.guard(); g.check(Motion.rest(self.q),Feedback(self.q,1,1),1.008,1,1)
        with self.assertRaisesRegex(SafetyFault,'retimestamped'): g.check(Motion.rest(self.q),Feedback(self.q,1.008,1),1.016,1,1)
    def test_replayed_input_cannot_refresh_age(self):
        g=self.guard(); g.check(Motion.rest(self.q),Feedback(self.q,1,1),1.008,1,1)
        with self.assertRaisesRegex(SafetyFault,'retimestamped'): g.check(Motion.rest(self.q),Feedback(self.q,1.008,2),1.016,1.008,1)
    def test_measured_velocity(self):
        with self.assertRaisesRegex(SafetyFault,'measured_'): self.guard().check(Motion.rest(self.q),Feedback(self.q,1,1,velocity=[10]*6),1.008,1,1)
    def test_forged_zero_velocity_step_rejected(self):
        with self.assertRaisesRegex(SafetyFault,'command_step_velocity'):
            self.guard().check(Motion.rest(self.q+.03),Feedback(self.q,1,1),1.008,1,1)
    def test_sampled_certified_path(self):
        g=self.guard(); seg=plan(Motion.rest(self.q),self.q+.3,self.c['limits']); prev=self.q
        for i,t in enumerate(np.arange(.008,seg.duration,.008)):
            m=seg.sample(t)
            self.assertTrue(g.check(m,Feedback(prev,t,i),t,t,i)); prev=m.q

class FakeRC:
    def __init__(self,fail=None): self.calls=[]; self.servo=False; self.fail=fail
    def call(self,name,*args):
        self.calls.append((name,args))
        if self.fail==name: return (2,)
        return (0,)
    def is_in_servomove(self): self.calls.append(('is_in_servomove',())); return (0,int(self.servo))
    def servo_move_use_joint_LPF(self,hz): return self.call('lpf',hz)
    def servo_move_enable(self,value):
        r=self.call('enable',value)
        if r[0]==0: self.servo=value
        return r
    def servo_j(self,q,mode,step): return self.call('send',q,mode,step)
    def motion_abort(self): return self.call('abort')

class SDKTests(Base):
    def test_strict_bool(self):
        for x in [0,1,False,True]: self.assertEqual(checked_bool((0,x),'x'),bool(x))
        for x in ['1',2,None,1.0]:
            with self.subTest(x=x),self.assertRaises(RuntimeError): checked_bool((0,x),'x')
    def test_order_units_and_stop(self):
        rc=FakeRC(); d=JakaServo(rc,self.c,lambda action:None)
        d.start(); d.send(Motion.rest(self.q)); result=d.stop()
        self.assertEqual(rc.calls[1],('enable',(True,)))
        self.assertFalse(any(n=='lpf' for n,_ in rc.calls))
        self.assertEqual(next(v for n,v in rc.calls if n=='send'),(self.q.tolist(),0,1))
        self.assertTrue(result['servo_exited']); self.assertFalse(result['physical_stop_confirmed'])
        self.assertFalse(any(n in ('power','none_filter') for n,_ in rc.calls))
    def test_filter_failure_prevents_servo(self):
        self.c['filters']['controller_lpf_policy']='set_before_servo'
        rc=FakeRC('lpf'); d=JakaServo(rc,self.c,lambda action:None)
        with self.assertRaises(RuntimeError): d.start()
        d.stop(); self.assertFalse(any(n in ('enable','send','abort') for n,_ in rc.calls))
    def test_explicit_controller_lpf_is_set_before_enable(self):
        self.c['filters']['controller_lpf_policy']='set_before_servo'
        rc=FakeRC(); d=JakaServo(rc,self.c,lambda action:None)
        d.start()
        self.assertEqual(rc.calls[1],('lpf',(6.,)))
        self.assertEqual(rc.calls[2],('enable',(True,)))
        d.stop()
    def test_gate_checked_each_send(self):
        rc=FakeRC()
        def gate(action):
            if action=='send': raise RuntimeError('revoked')
        d=JakaServo(rc,self.c,gate); d.start()
        with self.assertRaisesRegex(RuntimeError,'revoked'): d.send(Motion.rest(self.q))
        self.assertFalse(any(n=='send' for n,_ in rc.calls)); d.stop()
    def test_no_takeover_existing_servo(self):
        rc=FakeRC(); rc.servo=True; d=JakaServo(rc,self.c,lambda action:None)
        with self.assertRaises(RuntimeError): d.start()
        d.stop(); self.assertEqual(len(rc.calls),1)
    def test_cleanup_after_enable_failure(self):
        rc=FakeRC('enable'); d=JakaServo(rc,self.c,lambda action:None)
        with self.assertRaises(RuntimeError): d.start()
        out=d.stop(); self.assertTrue(out['errors']); self.assertTrue(any(n=='abort' for n,_ in rc.calls))
    def test_status_schema(self):
        s=[0]*24; s[2]=s[3]=s[22]=1; s[19]=self.q.tolist()
        f=decode_status((0,s),1,1); self.assertIsNone(f.torque)
        with self.assertRaises(RuntimeError): decode_status((0,s[:23]),1,1)
        s[23]='false'
        with self.assertRaises(RuntimeError): decode_status((0,s),1,1)

class RuntimeTests(Base):
    def test_latest_only_and_reorder(self):
        s=LatestSlot(); s.publish('a',1,1); s.publish('b',2,2); self.assertEqual(s.read(),('b',2,2))
        with self.assertRaises(ValueError): s.publish('a',1,3)
        s.clear(); self.assertIsNone(s.read()[0])
        with self.assertRaises(ValueError): s.publish('b',2,2)
    def test_no_catchup_burst(self):
        now=[0.]
        t=Deadline(.008,lambda:now[0],lambda dt:now.__setitem__(0,now[0]+dt))
        t.wait(); now[0]=.02; _,late=t.wait(); self.assertAlmostEqual(late,.012)
        stamp,_=t.wait(); self.assertAlmostEqual(stamp,.028)
    def test_log_real_file(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'logs') as tmp:
            p=Path(tmp)/'events.jsonl'; logger=JsonLogger(p); logger.emit(event='test'); logger.close()
            self.assertEqual(json.loads(p.read_text())['event'],'test')
    def test_configuration_validation(self):
        for change in [('period_s',.01)]:
            c=copy.deepcopy(self.c); c[change[0]]=change[1]
            with self.assertRaises(ValueError): validate(c)
    def test_stop_on_missing_feedback(self):
        rc=FakeRC(); d=JakaServo(rc,self.c,lambda action:None)
        with tempfile.TemporaryDirectory(dir=ROOT/'logs') as tmp:
            logger=JsonLogger(Path(tmp)/'session.jsonl')
            r=run_stream(d,Guard(self.c),LatestSlot(),LatestSlot(),logger,threading.Event(),.1,lambda m,f:True)
            logger.close(); self.assertEqual(r['failure'],'missing_sample'); self.assertFalse(rc.servo)
            self.assertEqual(rc.calls,[]) # 缺反馈不得先开伺服、改滤波再退出。
    def test_preflight_fault_never_enters_servo(self):
        import time
        for blocked_path,robot_fault in [(True,False),(False,True)]:
            rc=FakeRC(); d=JakaServo(rc,self.c,lambda action:None); commands=LatestSlot(); feedback=LatestSlot()
            stamp=time.perf_counter(); commands.publish(Motion.rest(self.q),1,stamp)
            feedback.publish(Feedback(self.q,stamp,1,estop=robot_fault),1,stamp)
            with tempfile.TemporaryDirectory(dir=ROOT/'logs') as tmp:
                logger=JsonLogger(Path(tmp)/'session.jsonl')
                r=run_stream(d,Guard(self.c),commands,feedback,logger,threading.Event(),.1,lambda m,f:not blocked_path)
                logger.close(); self.assertIsNotNone(r['failure']); self.assertEqual(rc.calls,[])

class InputTests(Base):
    def intent(self,seq=1,source='vr',stamp=1): return JointIntent(self.q,source,seq,stamp,stamp)
    def test_release_before_grip(self):
        g=InputGate()
        with self.assertRaises(ValueError): g.accept(self.intent(),1,True)
        g.accept(self.intent(),1,False); np.testing.assert_allclose(g.accept(self.intent(),1,True),self.q)
    def test_replay_and_source_lock(self):
        for source,seq in [('vr',1),('other',2)]:
            g=InputGate(); g.accept(self.intent(),1,False); g.accept(self.intent(),1,True)
            with self.assertRaises(ValueError): g.accept(self.intent(seq,source,1.008),1.008,True)
    def test_tracking_invalidates_anchor(self):
        g=InputGate(); g.accept(self.intent(),1,False)
        with self.assertRaises(ValueError): g.accept(self.intent(),1,True,False)
        self.assertFalse(g.release_seen)
    def test_vla_cannot_confuse_pose_joint_or_units(self):
        for action_space,units in [('delta_tcp','rad'),('absolute_joint_position','deg')]:
            with self.assertRaises(ValueError): vla_joint_intent(self.q,{'action_space':action_space,'units':units},1,1,1)

class KinematicsTests(Base):
    def test_nonidentity_world_mount_is_applied(self):
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory(dir=ROOT/'logs') as tmp:
            tree=ET.parse(ROOT/'models/assembly_reference.urdf')
            mount=tree.getroot().find("joint[@name='world_to_link_00']")
            ET.SubElement(mount,'origin',{'xyz':'1 2 3','rpy':'0 0 0'})
            p=Path(tmp)/'mounted.urdf'; tree.write(p)
            original=ArmKinematics(ROOT/'models/assembly_reference.urdf').forward(self.q)[0]
            mounted=ArmKinematics(p).forward(self.q)[0]
            np.testing.assert_allclose(mounted[:3,3]-original[:3,3],[1,2,3])
    def test_jacobian_matches_finite_difference(self):
        arm=ArmKinematics(ROOT/'models/assembly_reference.urdf'); q=self.q+np.arange(6)*.12
        t,j,_=arm.forward(q)
        for i in range(6):
            q1=q.copy(); q1[i]+=1e-6; t1,_,_=arm.forward(q1)
            np.testing.assert_allclose((t1[:3,3]-t[:3,3])/1e-6,j[:3,i],atol=1e-6)

if __name__=='__main__': unittest.main()
