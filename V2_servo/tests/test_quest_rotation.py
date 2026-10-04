import copy,json,unittest
import numpy as np
from v2_servo.configuration import load,ROOT
from v2_servo.teleop.local_rotation import local_rotation_model
from v2_servo.teleop.quest_rotation import QuestRotationMapping,RotationShadowPlanner
from v2_servo.teleop.commissioning_guidance import guidance_packet
from test_quest_tcp_z import SyntheticTCPRC


class RotationShadowTests(unittest.TestCase):
    def setUp(self):self.rc=SyntheticTCPRC();self.cfg=load()
    def test_official_FK_each_rotation_axis_units_and_fixed_TCP(self):
        for axis in ('RX','RY','RZ'):
            model=local_rotation_model(self.rc,self.rc.q,axis)
            self.assertLessEqual(model['maximum_angle_deg'],.02)
            self.assertEqual(len(model['sweep']),21)
            self.assertLess(max(s['tcp_drift_mm'] for s in model['sweep']),1e-6)
            self.assertLess(max(s['angular_residual_deg'] for s in model['sweep']),1e-6)
            self.assertFalse(model['physical_calibration_passed'])
    def test_mapping_axial_reflection_and_quaternion_sign_and_rebind(self):
        packet=None
        with (ROOT/'data/h6_20261003_130933_206345/events.jsonl').open(encoding='utf-8') as lines:
            for line in lines:
                row=json.loads(line)
                if row.get('event')=='Quest_raw_input':packet=row['raw'];break
        basis=np.array([[0,0,1],[-1,0,0],[0,1,0]]);mapper=QuestRotationMapping(basis);sequence=0
        def decode(grip,angle,sign=1):
            nonlocal sequence
            sequence+=1;p=copy.deepcopy(packet);p.update(sequence=sequence,unity_time_s=sequence*.02)
            theta=np.deg2rad(angle)/2;q=np.array([0,0,np.sin(theta),np.cos(theta)])*sign
            p['right'].update(grip=grip,rotation_xyzw=dict(zip(('x','y','z','w'),q.tolist())),thumbstick_click=False)
            return mapper.decode(p,('127.0.0.1',42000),sequence*.02)
        decode(0,0);decode(1,0);f=decode(1,5)
        np.testing.assert_allclose(f['rotation_delta_deg'],[-5,0,0],atol=1e-8)
        f2=decode(1,5,-1);np.testing.assert_allclose(f2['rotation_delta_deg'],f['rotation_delta_deg'])
        decode(0,5);f=decode(1,5);np.testing.assert_allclose(f['rotation_delta_deg'],[0,0,0])
    def test_shadow_motion_release_and_stale_latch_are_bounded(self):
        model=local_rotation_model(self.rc,self.rc.q,'RX');p=RotationShadowPlanner(model,self.cfg)
        def frame(t,grip,angle):return {'stamp':t,'tracking_valid':True,'grip':grip,'deadman':grip>.7,'rotation_delta_deg':[angle,0,0]}
        p.step(frame(0,0,0),0)
        for i in range(700):
            t=(i+1)*.008;m=p.step(frame(t,1,5 if i<350 else -5),t)
            self.assertLessEqual(abs(p.angle),p.maximum);self.assertLess(np.max(np.abs(m.q-p.q0)),np.deg2rad(.1))
        self.assertGreater(np.max(np.abs(m.q-p.q0)),1e-5)
        for i in range(150):
            t=5.608+i*.008;m=p.step(frame(t,0,0),t)
        self.assertEqual(p.releases,1);self.assertLess(np.max(np.abs(m.v)),1e-8)
        q=m.q.copy();p.step(frame(1,1,5),7.)
        for i in range(100):m=p.step(frame(7.008+i*.008,1,5),7.008+i*.008)
        self.assertTrue(p.locked);np.testing.assert_allclose(m.q,q,atol=1e-10)
    def test_shadow_never_displays_live_permission(self):
        p=guidance_packet('ROTATION_SHADOW','rotation_shadow',control_scope='ROTATION_SHADOW')
        self.assertIn('不动机械臂',p['title']);self.assertIn('运动命令0',p['control_scope'])
        with self.assertRaises(ValueError):guidance_packet('LIVE','rotation_shadow',control_scope='ROTATION_SHADOW')


if __name__=='__main__':unittest.main()
