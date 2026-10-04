import unittest
import numpy as np
from types import SimpleNamespace
from v2_servo.teleop.rotation_rehearsal import RotationRehearsal
from v2_servo.teleop.commissioning_guidance import guidance_packet


class RotationRehearsalTests(unittest.TestCase):
    def setUp(self):
        self.r=RotationRehearsal();self.seq=0
        self.p=SimpleNamespace(gripped=True,state=SimpleNamespace(v=np.zeros(6),a=np.zeros(6)))
    def send(self,grip,delta,repeat=False):
        if not repeat:self.seq+=1
        f=dict(sequence=self.seq,stamp=self.seq*.01,tracking_valid=True,grip=grip,deadman=grip>.7,rotation_delta_deg=delta)
        self.r.step(f,f['stamp'],self.p)
    def test_late_start_and_three_axis_release_are_required(self):
        for _ in range(1000):self.send(0,[0,0,0])
        self.assertFalse(self.r.passed)
        for axis in range(3):
            d=[0,0,0];d[axis]=3
            for _ in range(3):self.send(1,d)
            self.assertEqual(self.r.phase,'RELEASE')
            for _ in range(50):self.send(.4,[0,0,0])
            self.assertEqual(len(self.r.completed),axis)
            self.p.state.v[0]=1e-4
            for _ in range(8):self.send(0,[0,0,0])
            self.assertEqual(len(self.r.completed),axis)
            self.p.state.v[0]=0
            for _ in range(5):self.send(0,[0,0,0])
        self.assertTrue(self.r.passed);self.assertEqual(self.r.completed,['RX','RY','RZ'])
    def test_duplicate_frame_cannot_complete_axis(self):
        self.send(1,[3,0,0])
        for _ in range(50):self.send(1,[3,0,0],repeat=True)
        self.assertEqual(self.r.phase,'ROTATE')
    def test_hud_phases_explicitly_have_no_motion_permission(self):
        for stage in ('ROTATION_RX','ROTATION_RY','ROTATION_RZ','ROTATION_RELEASE'):
            p=guidance_packet(stage,'rotation_shadow',control_scope='ROTATION_SHADOW')
            self.assertIn('运动命令0',p['control_scope']);self.assertNotEqual(p['stage'],'LIVE')
