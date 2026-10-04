import json,unittest
import numpy as np
from v2_servo.configuration import ROOT,load
from v2_servo.teleop.local_xyz import local_xyz_model
from v2_servo.teleop.quest_pose_shadow import QuestPoseShadowPlanner
from v2_servo.teleop.commissioning_guidance import guidance_packet
from test_quest_tcp_z import SyntheticTCPRC


class PoseShadowTests(unittest.TestCase):
    def setUp(self):
        self.rc=SyntheticTCPRC();self.cfg=load();self.model=local_xyz_model(self.rc,self.rc.q)
        self.p=QuestPoseShadowPlanner(self.rc.q,self.cfg,self.model,json.loads((ROOT/'config/quest_xyz_commissioning.json').read_text()))
    def frame(self,t,held,sign=1):
        return {'stamp':t,'sequence':int(t*1000),'tracking_valid':True,'grip':float(held),'deadman':held,
            'base_displacement_mm':[10*sign,10*sign,10*sign],'rotation_delta_deg':[5*sign,5*sign,5*sign],'stick_y':0.}
    def test_combined_translation_and_rotation_with_jerk_brake_limits(self):
        p=self.p;p.step(self.frame(0,False),0);xyz=[];rot=[]
        for i in range(1200):
            t=(i+1)*.008;block=i//300;f=self.frame(t,i%300<120,1 if block%2==0 else -1)
            m=p.step(f,t);p.check(m.q,m.q,np.zeros(6));xyz.append(p.xyz_target_mm.copy());rot.append(p.rotation_target_deg.copy())
        self.assertTrue(np.all(np.ptp(xyz,axis=0)>.02));self.assertTrue(np.all(np.ptp(rot,axis=0)>.003))
        self.assertGreaterEqual(len(p.release_events),3);self.assertTrue(all(e['command_stop_time_ms']<1000 for e in p.release_events))
        self.assertLessEqual(np.max(np.linalg.norm(rot,axis=1)),.02);self.assertLessEqual(np.max(np.linalg.norm(xyz,axis=1)),.5)
    def test_tracking_loss_latches_and_regrip_cannot_resume(self):
        p=self.p;p.step(self.frame(0,False),0)
        for i in range(100):p.step(self.frame((i+1)*.008,True), (i+1)*.008)
        p.step(None,.808)
        for i in range(120):p.step(self.frame(.816+i*.008,True),.816+i*.008)
        self.assertTrue(p.locked);self.assertTrue(p.finished);self.assertTrue(p.quiet())
    def test_shadow_cannot_display_live_permission(self):
        p=guidance_packet('POSE_SHADOW','pose6dof_shadow',control_scope='POSE_SHADOW')
        self.assertIn('不控制机器人',p['title'])
        with self.assertRaises(ValueError):guidance_packet('LIVE','pose6dof_shadow',control_scope='POSE_SHADOW')
