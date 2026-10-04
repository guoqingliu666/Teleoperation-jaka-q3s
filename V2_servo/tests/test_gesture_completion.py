import unittest
from types import SimpleNamespace
import numpy as np
from v2_servo.teleop.gesture_completion import GestureCompletion


class GestureCompletionTests(unittest.TestCase):
    def observe(self,g,t,p=None,**kw):
        p=p or SimpleNamespace(locked=False,quiet=lambda:True,release_events=[{'moving_at_release':True}]*2)
        f={'stamp':t,'sequence':int(t*1000),'tracking_valid':True,'grip':0.,'stick_y':0.}
        f.update(kw.get('frame',{}))
        return g.observe(t,f,p,kw.get('q',np.zeros(6)),kw.get('dq',np.zeros(6)),kw.get('command_span',.01),kw.get('actual_span',.01))
    def test_two_moving_releases_motion_and_stable_feedback_finish_early(self):
        g=GestureCompletion()
        for i in range(25):self.assertFalse(self.observe(g,i*.008))
        self.assertTrue(self.observe(g,.208))
    def test_invalid_input_or_fault_never_overridden_by_completed_gestures(self):
        for change in ({'tracking_valid':False},{'grip':1.},{'stamp':-1.},{'software_stop':True}):
            g=GestureCompletion()
            for i in range(30):self.assertFalse(self.observe(g,i*.008,frame=change))
        p=SimpleNamespace(locked=True,quiet=lambda:True,release_events=[{'moving_at_release':True}]*2)
        self.assertFalse(self.observe(GestureCompletion(),1,p))
    def test_short_motion_stationary_release_or_moving_feedback_cannot_finish(self):
        for args in ({'actual_span':.001},{'command_span':.001},{'dq':np.full(6,np.deg2rad(.11))}):
            g=GestureCompletion()
            for i in range(30):self.assertFalse(self.observe(g,i*.008,**args))
        for n in (0,1):
            p=SimpleNamespace(locked=False,quiet=lambda:True,release_events=[{'moving_at_release':True}]*n)
            self.assertFalse(self.observe(GestureCompletion(),1,p))
