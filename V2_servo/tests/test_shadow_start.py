import unittest
from v2_servo.teleop.shadow_start import ShadowGestureStart


class ShadowStartTests(unittest.TestCase):
    def frame(self, t, grip=0., tracked=True):
        return dict(stamp=t,sequence=int(t*1000),tracking_valid=tracked,
                    grip=grip,deadman=grip>.7,stick_y=0.)

    def arm(self, gate, start=0.):
        for i in range(30): gate.update(self.frame(start+i*.01),start+i*.01)

    def test_pre_gesture_tracking_loss_requires_new_stable_release(self):
        g=ShadowGestureStart();self.arm(g);self.assertTrue(g.armed)
        self.assertFalse(g.update(self.frame(.3,tracked=False),.3)[0])
        self.assertFalse(g.update(self.frame(.31,1.),.31)[0])
        self.arm(g,.4);self.assertTrue(g.update(self.frame(.71,1.),.71)[0])
        self.assertEqual(g.resets,1)

    def test_repeated_released_packet_and_initial_grip_do_not_start(self):
        g=ShadowGestureStart();self.assertFalse(g.update(self.frame(0,1.),0)[0])
        f=self.frame(.1)
        for i in range(5):self.assertFalse(g.update(f,.1+i*.01)[0])
        self.assertFalse(g.armed)

    def test_start_gate_is_not_a_motion_fault_recovery(self):
        g=ShadowGestureStart();self.arm(g)
        self.assertTrue(g.update(self.frame(.31,1.),.31)[0])
        self.assertTrue(g.started)
        # 启动后控制权交给现有锁停 planner；门不会再次输出解除锁停命令。
        self.assertEqual(g.update(None,.4)[1],'SHADOW_GESTURE_ALREADY_STARTED')
