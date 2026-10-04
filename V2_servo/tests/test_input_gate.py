import unittest
from v2_servo.teleop.input_gate import StableReleasedInput
from v2_servo.teleop.commissioning_guidance import guidance_packet


class InputGateTests(unittest.TestCase):
    def frame(self,t,seq,**changes):
        return dict(stamp=t,sequence=seq,tracking_valid=True,grip=0.,stick_y=0.,stick_clicked=False,software_stop=False,**changes)

    def test_distinct_fresh_packets_and_stable_time_required(self):
        g=StableReleasedInput()
        for i in range(10):self.assertFalse(g.update(self.frame(i*.01,i),i*.01)[0])
        self.assertTrue(g.update(self.frame(.21,10),.21)[0])

    def test_duplicate_packets_do_not_accumulate(self):
        g=StableReleasedInput()
        for i in range(30):self.assertFalse(g.update(self.frame(i*.01,1),i*.01)[0])
        self.assertEqual(g.count,1)

    def test_tracking_stale_grip_click_and_axis_reset_stability(self):
        for change in ({'tracking_valid':False},{'grip':1.},{'stick_clicked':True},{'software_stop':True},{'stick_y':.5}):
            g=StableReleasedInput()
            for i in range(12):
                f=self.frame(i*.02,i);g.update(f,i*.02)
            f=self.frame(.25,13);f.update(change)
            self.assertFalse(g.update(f,.25)[0]);self.assertEqual(g.count,0)
            self.assertFalse(g.update(self.frame(.26,14),.26)[0])
        g=StableReleasedInput();self.assertFalse(g.update(self.frame(0,1),.081)[0])

    def test_waiting_panel_never_claims_live_motion(self):
        p=guidance_packet('INPUT_READY_WAIT','rotation_shadow',control_scope='TCP_RZ')
        self.assertIn('尚未连接',p['title']);self.assertEqual(p['scope'],'operator_display_only_no_motion_permission')
