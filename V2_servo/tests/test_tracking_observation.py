import json,socket,time,unittest
import numpy as np
from v2_servo.configuration import ROOT,load
from v2_servo.teleop.continuous_pose import ContinuousPoseServo,anchored_orientation
from v2_servo.kinematics import rotate
from v2_servo.teleop.global_kinematics import rotation_log
from v2_servo.teleop.continuous_shadow import load_model
from v2_servo.teleop.tracking_observation import shadow_tracking
from v2_servo.teleop.twin_publish import TwinPublisher

class TrackingObservationTests(unittest.TestCase):
    def setUp(self):
        self.model,self.q,_=load_model()
        self.p=json.loads((ROOT/'config/continuous.json').read_text())
        self.c=ContinuousPoseServo(self.q,load(),self.model,self.p,np.eye(3))
    def frame(self,i,held):
        return dict(stamp=i*.008,sequence=i,tracking_valid=True,grip=float(held),deadman=held,
            position_m=[0,0,0],rotation_xyzw=[0,0,0,1],software_stop=False)
    def test_goal_invalid_until_bound_and_cleared_on_release_and_timeout(self):
        for i,held in enumerate([False,True,False]):
            f=self.frame(i,held);motion=self.c.step(f,i*.008,self.q,i*.008)
            t=self.model.forward(motion.q)[0];d=shadow_tracking(self.c,t,f,i*.008,'abc')
            self.assertEqual(d['target_valid'],held)
            if held:np.testing.assert_allclose(np.array(d['target_T']).reshape(4,4),t)
            else:self.assertIsNone(d['target_T'])
        self.c.held=True;self.c.phase='FOLLOW'
        d=shadow_tracking(self.c,t,f,2,'abc');self.assertFalse(d['target_valid'])
    def test_observation_cannot_change_controller_and_error_uses_feedback(self):
        self.c.step(self.frame(0,False),0,self.q,0)
        f=self.frame(1,True);m=self.c.step(f,.008,self.q,.008)
        t=self.model.forward(m.q)[0];t[:3,3]+=[.003,.004,0]
        before=m.q.copy();goal=self.c.goal.copy()
        d=shadow_tracking(self.c,t,f,.008,'abc')
        self.assertAlmostEqual(d['position_error_mm'],5)
        self.assertEqual(d['feedback_source'],'simulated_fk');self.assertFalse(d['physical_feedback'])
        np.testing.assert_array_equal(m.q,before);np.testing.assert_array_equal(self.c.goal,goal)
    def test_udp_preserves_transforms_units_and_no_fabricated_live_feedback(self):
        self.c.step(self.frame(0,False),0,self.q,0);f=self.frame(1,True)
        m=self.c.step(f,.008,self.q,.008);d=shadow_tracking(self.c,self.model.forward(m.q)[0],f,.008,'abc')
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1',0));receiver.settimeout(1)
            p=TwinPublisher((receiver.getsockname()[1],))
            try:
                p.offer(dict(mode='simulation',connected=True,active=True,q=self.q,feedback_age_ms=0,tracking=d))
                packet=json.loads(receiver.recv(32000));self.assertEqual(packet['tracking'],d)
                p.offer(dict(mode='live',connected=True,q=self.q,feedback_age_ms=0))
                packet=json.loads(receiver.recv(32000));self.assertIsNone(packet['tracking'])
            finally:p.close()
    def test_anchored_orientation_closed_path_and_pi_crossing(self):
        B=np.array([[0,0,1],[-1,0,0],[0,1,0]])
        anchor=rotate([1,0,0],.4);robot=rotate([0,1,0],-.7)
        sequence=[anchor,rotate([1,0,0],1)@anchor,rotate([0,1,0],1)@anchor,anchor]
        result=[anchored_orientation(B,h,anchor,robot) for h in sequence]
        np.testing.assert_allclose(result[-1],robot,atol=1e-12)
        a=anchored_orientation(B,rotate([0,0,1],np.deg2rad(179))@anchor,anchor,robot)
        b=anchored_orientation(B,rotate([0,0,1],np.deg2rad(181))@anchor,anchor,robot)
        self.assertAlmostEqual(np.rad2deg(np.linalg.norm(rotation_log(b@a.T))),2,places=9)
    def test_one_to_one_candidate_binds_without_jump_and_keeps_position_goal(self):
        p=dict(self.p,rotation_gain=1.,orientation_mapping='anchored_one_to_one')
        c=ContinuousPoseServo(self.q,load(),self.model,p,np.eye(3))
        c.step(self.frame(0,False),0,self.q,0)
        f=self.frame(1,True);c.step(f,.008,self.q,.008);ref=c.goal.copy()
        f=self.frame(2,True);f['rotation_xyzw']=[0,0,np.sin(.2),np.cos(.2)]
        c.step(f,.016,c.state.q,.016)
        self.assertAlmostEqual(np.linalg.norm(rotation_log(c.goal[:3,:3]@ref[:3,:3].T)),.4,places=9)
        np.testing.assert_allclose(c.goal[:3,3],ref[:3,3])
        f=self.frame(3,True);c.step(f,.024,c.state.q,.024)
        np.testing.assert_allclose(c.goal,ref,atol=1e-12)
