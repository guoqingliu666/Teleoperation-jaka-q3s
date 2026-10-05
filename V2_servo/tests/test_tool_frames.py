import json
import unittest
import numpy as np
from v2_servo.configuration import ROOT
from v2_servo.teleop.continuous_shadow import load_model
from v2_servo.teleop.global_kinematics import pose_matrix
from v2_servo.teleop.tool_frames import configured_shadow_tool


class ToolFramesTests(unittest.TestCase):
    def setUp(self):
        self.model,self.q,self.source=load_model()
        self.old,_,_=load_model(self.source)

    def test_local_z_200mm_and_orientation_across_joint_domain(self):
        rng=np.random.default_rng(20261004)
        for q in rng.uniform(-np.pi,np.pi,(250,6)):
            tcp1,j,_=self.model.forward(q)
            tcp2,_,_=self.model.tcp2_model.forward(q)
            np.testing.assert_allclose(np.linalg.inv(tcp2)@tcp1,pose_matrix([0,0,200,0,0,0]),atol=2e-12)
            np.testing.assert_allclose(j[:3,5],0,atol=3e-9)

    def test_j6_changes_orientation_but_not_tcp1_position(self):
        q=np.array(self.q);reference=self.model.forward(q)[0]
        for a in np.linspace(-np.pi,np.pi,65):
            q[-1]=self.q[-1]+a
            t=self.model.forward(q)[0]
            np.testing.assert_allclose(t[:3,3],reference[:3,3],atol=3e-9)
            expected=reference[:3,:3]@pose_matrix([0,0,0,0,0,a])[:3,:3]
            np.testing.assert_allclose(t[:3,:3],expected,atol=2e-10)

    def test_model_derives_from_official_old_tool_not_visual_mesh(self):
        r=json.loads(self.source.read_text())
        correction=np.linalg.inv(pose_matrix(r['tcp_parameters']))@pose_matrix([0,0,200,0,0,0])
        np.testing.assert_allclose(self.model.home,self.old.home@correction,atol=1e-12)
        self.assertFalse(self.model.tool_provenance['controller_readback_verified'])
        self.assertFalse(self.model.tool_provenance['live_motion_approved'])
        self.assertFalse(hasattr(self.old,'tcp2_model'))

    def test_jacobian_matches_changed_tcp_numerical_derivative(self):
        q=np.array(self.q);_,j,_=self.model.forward(q)
        for i in range(6):
            d=np.zeros(6);d[i]=1e-6
            derivative=(self.model.forward(q+d)[0][:3,3]-self.model.forward(q-d)[0][:3,3])/2e-6
            np.testing.assert_allclose(j[:3,i],derivative,atol=2e-9)
