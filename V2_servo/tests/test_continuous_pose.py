import copy,json,unittest
import numpy as np
from v2_servo.configuration import ROOT,load
from v2_servo.safety import SafetyFault
from v2_servo.teleop.global_kinematics import SerialPoE
from v2_servo.teleop.continuous_pose import ContinuousPoseServo
from v2_servo.teleop.continuous_shadow import synthetic_frame
from v2_servo.runtime_paths import same_sdk_artifacts

class ContinuousPoseTests(unittest.TestCase):
    def setUp(self):
        r=json.loads((ROOT/'data/global_fk_readonly_20261003_233216_333163/report.json').read_text(encoding='utf-8'))
        self.model=SerialPoE(r['model']);self.q=np.array(r['model']['q_reference_rad'])
        self.p=json.loads((ROOT/'config/continuous.json').read_text(encoding='utf-8'));self.cfg=load()
        self.controller=ContinuousPoseServo(self.q,self.cfg,self.model,self.p,np.eye(3))
    def frame(self,i,held=False,p=None):
        return dict(stamp=i*.008,sequence=i,tracking_valid=True,grip=float(held),deadman=held,
            position_m=p or [0.,0.,0.],rotation_xyzw=[0.,0.,0.,1.],software_stop=False)
    def test_more_than_30s_and_100mm_no_start_sphere(self):
        c=self.controller;q=self.q.copy();origin=self.model.forward(q)[0][:3,3];peak=0.;vmax=amax=jmax=0.
        for i in range(4200):
            f=synthetic_frame(i,.008,np.eye(3));m=c.step(f,i*.008,q,i*.008);q=m.q.copy()
            peak=max(peak,np.linalg.norm(self.model.forward(q)[0][:3,3]-origin))
            vmax=max(vmax,np.max(np.abs(m.v)));amax=max(amax,np.max(np.abs(m.a)));jmax=max(jmax,np.max(np.abs(m.j)))
        self.assertGreater(peak,.1);self.assertFalse(c.locked);self.assertGreater(len(c.events),1)
        self.assertLessEqual(vmax,np.deg2rad(self.p['joint_velocity_deg_s'])+1e-9)
        self.assertLessEqual(amax,np.deg2rad(self.p['joint_acceleration_deg_s2'])+1e-9)
        self.assertLessEqual(jmax,np.deg2rad(self.p['joint_jerk_deg_s3'])+1e-8)
        self.assertTrue(all(e['stop_s']<1 for e in c.events))
    def test_loss_requires_release_and_software_stop_remains_latched(self):
        c=self.controller;q=self.q.copy()
        c.step(self.frame(0),0,q,0);c.step(self.frame(1,True),.008,q,.008)
        for i in range(2,70):
            f=self.frame(i,True,[.03,0,0]);m=c.step(f,i*.008,q,i*.008);q=m.q.copy()
        for i in range(70,160):
            m=c.step(None,i*.008,q,i*.008);q=m.q.copy()
        self.assertTrue(c.locked);self.assertTrue(c.quiet())
        c.step(self.frame(160,True),1.28,q,1.28);self.assertTrue(c.locked)
        c.step(self.frame(161),1.288,q,1.288);self.assertFalse(c.locked)
        f=self.frame(162);f['software_stop']=True;c.step(f,1.296,q,1.296)
        c.step(self.frame(163),1.304,q,1.304);self.assertTrue(c.locked)
    def test_regrip_does_not_jump_to_old_hand_reference(self):
        c=self.controller;q=self.q.copy()
        c.step(self.frame(0),0,q,0);c.step(self.frame(1,True,[.5,0,0]),.008,q,.008)
        np.testing.assert_allclose(c.goal,self.model.forward(q)[0],atol=1e-12)
        c.step(self.frame(2),.016,q,.016);c.step(self.frame(3,True,[-.5,0,0]),.024,q,.024)
        np.testing.assert_allclose(c.goal,self.model.forward(q)[0],atol=1e-12)
    def test_protective_faults_no_smoothed_continuation_and_filter_required(self):
        for kwargs in (dict(hardware_fault='collision'),dict(feedback_stamp=-1),dict(actual_q=self.q+.002)):
            c=self.controller;args=dict(frame=self.frame(0),now=0,actual_q=self.q,feedback_stamp=0);args.update(kwargs)
            with self.assertRaises(SafetyFault):c.step(**args)
        cfg=copy.deepcopy(self.cfg);cfg['filters']['enabled']=False
        with self.assertRaises(SafetyFault):ContinuousPoseServo(self.q,cfg,self.model,self.p,np.eye(3))
    def test_retimestamped_input_rejected(self):
        c=self.controller;c.step(self.frame(0),0,self.q,0)
        f=self.frame(0);f['stamp']=.008
        with self.assertRaises(SafetyFault):c.step(f,.008,self.q,.008)
    def test_sdk_copy_changes_location_not_binary_identity(self):
        names=('jakaAPI.dll','jakaAPI.h','jktypes.h')
        old={'F:/source/'+name:str(i)*64 for i,name in enumerate(names)}
        new={'D:/V2/sdk/'+name:str(i)*64 for i,name in enumerate(names)}
        self.assertTrue(same_sdk_artifacts(old,new));new['D:/V2/sdk/jakaAPI.dll']='f'*64
        self.assertFalse(same_sdk_artifacts(old,new));self.assertFalse(same_sdk_artifacts({},{}))
