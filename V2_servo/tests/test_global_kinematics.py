import unittest
import numpy as np
from v2_servo.configuration import ROOT,load
from v2_servo.kinematics import ArmKinematics
from v2_servo.teleop.global_kinematics import SerialPoE,rotation_log

class GlobalKinematicsTests(unittest.TestCase):
    def setUp(self):
        self.arm=ArmKinematics(ROOT/'models/assembly_reference.urdf')
        self.q=np.array([.2,.4,-.5,.8,-.2,.1]);t,j,_=self.arm.forward(self.q)
        r=t[:3,:3];pose=np.r_[t[:3,3]*1000,np.arctan2(r[2,1],r[2,2]),np.arcsin(-r[2,0]),np.arctan2(r[1,0],r[0,0])]
        self.record=SerialPoE.identify(self.q,pose,j);self.model=SerialPoE(self.record)
    def test_full_joint_domain_matches_independent_serial_chain(self):
        limits=load()['limits'];rng=np.random.default_rng(20261003)
        for q in rng.uniform(limits['lower_rad'],limits['upper_rad'],(256,6)):
            expected,ej,_=self.arm.forward(q);actual,aj,_=self.model.forward(q)
            np.testing.assert_allclose(actual,expected,atol=1e-10)
            np.testing.assert_allclose(aj,ej,atol=1e-10)
    def test_geometric_jacobian_numeric_and_pi_rotation(self):
        q=self.q+.3;_,j,_=self.model.forward(q)
        for axis in range(6):
            d=np.zeros(6);d[axis]=1e-6;a,_,_=self.model.forward(q+d);b,_,_=self.model.forward(q-d)
            numeric=np.r_[(a[:3,3]-b[:3,3])/2e-6,rotation_log(a[:3,:3]@b[:3,:3].T)/2e-6]
            np.testing.assert_allclose(j[:,axis],numeric,atol=1e-8)
        self.assertAlmostEqual(np.linalg.norm(rotation_log(np.diag([1.,-1.,-1.]))),np.pi)
    def test_reject_bad_screws_and_nonfinite(self):
        bad=dict(self.record,space_screws_v_w=np.zeros((6,6)).tolist())
        with self.assertRaises(ValueError):SerialPoE(bad)
        with self.assertRaises(ValueError):self.model.forward([float('nan')]*6)
