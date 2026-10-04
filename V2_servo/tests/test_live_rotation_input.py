import copy,json,unittest
import numpy as np
from v2_servo.configuration import ROOT,load
from v2_servo.teleop.live_rotation_input import QuestLiveRotationMapping
from v2_servo.teleop.quest_rotation import QuestRotationMapping
from v2_servo.teleop.quest_tcp_rotation import QuestTCPRotationJog
from v2_servo.teleop.local_rotation import local_rotation_model
from test_quest_tcp_z import SyntheticTCPRC


class LiveRotationInputTests(unittest.TestCase):
    def setUp(self):
        self.basis=json.loads((ROOT/'config/quest_xyz_commissioning.json').read_text())['basis']
        with (ROOT/'data/h6_20261003_130933_206345/events.jsonl').open(encoding='utf-8') as lines:
            self.packet=next(json.loads(s)['raw'] for s in lines if json.loads(s).get('event')=='Quest_raw_input')
        self.seq=0

    def decode(self,mapper,grip,angle,**changes):
        self.seq+=1;p=copy.deepcopy(self.packet);p.update(sequence=self.seq,unity_time_s=self.seq*.02)
        q=[0.,float(np.sin(np.deg2rad(angle)/2)),0.,float(np.cos(np.deg2rad(angle)/2))]
        p['right'].update(grip=grip,rotation_xyzw=dict(zip(('x','y','z','w'),q)),thumbstick_click=False)
        p['right'].update(changes)
        return mapper.decode(p,('127.0.0.1',45000),self.seq*.02)

    def test_large_continuous_gesture_saturates_until_release_without_wrap_reversal(self):
        m=QuestLiveRotationMapping(self.basis);self.decode(m,0,0);self.decode(m,1,0)
        for angle in range(1,361):
            f=self.decode(m,1,angle)
            self.assertLessEqual(np.linalg.norm(f['rotation_delta_deg']),30.+1e-8)
            self.assertEqual(f['rotation_input_saturated'],angle>30)
            if angle>30:self.assertAlmostEqual(f['rotation_delta_deg'][2],-30.)
        self.decode(m,0,360);f=self.decode(m,1,360)
        self.assertFalse(f['rotation_input_saturated']);np.testing.assert_allclose(f['rotation_delta_deg'],0,atol=1e-10)

    def test_small_gesture_matches_reviewed_shadow_and_sign(self):
        live=QuestLiveRotationMapping(self.basis);shadow=QuestRotationMapping(self.basis)
        for grip,angle in [(0,0),(1,0),(1,3),(1,7),(1,14),(0,14)]:
            self.seq+=1;p=copy.deepcopy(self.packet);p.update(sequence=self.seq,unity_time_s=self.seq*.02)
            q=[0,float(np.sin(np.deg2rad(angle)/2)),0,float(np.cos(np.deg2rad(angle)/2))]
            p['right'].update(grip=grip,rotation_xyzw=q,thumbstick_click=False)
            a=live.decode(p,('127.0.0.1',45),self.seq*.02);b=shadow.decode(p,('127.0.0.1',45),self.seq*.02)
            np.testing.assert_allclose(a['rotation_delta_deg'],b['rotation_delta_deg'],atol=1e-8)

    def test_missing_grip_and_single_packet_jump_still_rejected(self):
        m=QuestLiveRotationMapping(self.basis)
        with self.assertRaisesRegex(ValueError,'grip_available'):self.decode(m,0,0,grip_available=False)
        m=QuestLiveRotationMapping(self.basis);self.decode(m,0,0);self.decode(m,1,0)
        with self.assertRaisesRegex(ValueError,'orientation jump'):self.decode(m,1,90)

    def test_saturation_does_not_relax_robot_angle_v_a_j_or_stop(self):
        m=QuestLiveRotationMapping(self.basis);rc=SyntheticTCPRC();cfg=load()
        model=local_rotation_model(rc,rc.q,'RZ');p=QuestTCPRotationJog(rc.q,cfg,model,model['tcp_origin_mm_rad'])
        f=self.decode(m,0,0);p.step(f,f['stamp']);angle=0
        for i in range(700):
            angle+=.2;f=self.decode(m,1,angle)
            for j in range(2):
                now=f['stamp']+j*.008;motion=p.step(f,now);p.check(motion.q,motion.q,np.zeros(6))
                self.assertLessEqual(abs(p.angle),.02);self.assertLessEqual(np.max(np.abs(motion.v)),np.deg2rad(.05)+1e-8)
        f=self.decode(m,0,angle)
        for i in range(125):
            now=f['stamp']+i*.008;f['stamp']=now;p.step(f,now)
            if p.quiet():break
        self.assertTrue(p.quiet());self.assertLess(i*.008,1.)
