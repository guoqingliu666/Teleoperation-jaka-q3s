"""Read-only target display tests; no hardware access."""
import math,os,time,unittest
from pathlib import Path
from operator_target import TargetHistory,current_target


class History(unittest.TestCase):
    def history(self):
        h=TargetHistory(10.);h.inputs(dict(held=True,age_s=0,reason='FOLLOWING'),10.)
        return h
    def pose(self,h,t,p=None,q=None):
        h.inputs(dict(held=True,age_s=-.001,reason='FOLLOWING'),t)
        return h.pose(p or [t-10,0,1],q or [0,0,0,1],'base_link',t,.001,t)
    def test_real_target_coordinates_and_quaternion_are_not_remapped(self):
        h=self.history();q=[0,0,math.sqrt(.5),math.sqrt(.5)]
        self.assertTrue(self.pose(h,10.,[.2,-.1,.8],q))
        d=h.snapshot(10.01);target=current_target(d,10.01)
        self.assertEqual(target['position_m'],[.2,-.1,.8]);self.assertEqual(target['quaternion_xyzw'],q)
    def test_release_hides_axes_and_history_expires_without_new_input(self):
        h=self.history();self.pose(h,10.);self.pose(h,10.03)
        h.inputs(dict(held=False,age_s=0,reason='RELEASED'),10.04)
        self.assertIsNone(current_target(h.snapshot(10.05),10.05));self.assertEqual(len(h.snapshot(14.)['trail']),2)
        self.assertEqual(h.snapshot(15.04)['trail'],[])
    def test_new_clutch_and_missing_input_break_the_trail(self):
        h=self.history();self.pose(h,10.);first=h.points[-1]['segment']
        h.inputs(dict(held=False,age_s=0),10.01);self.pose(h,10.02)
        self.assertNotEqual(h.points[-1]['segment'],first)
        second=h.points[-1]['segment'];self.pose(h,10.3)
        self.assertNotEqual(h.points[-1]['segment'],second)
    def test_stale_wrong_frame_and_bad_quaternion_never_show_as_live(self):
        h=self.history()
        for frame,stamp,age,q in [('world',10.,0,[0,0,0,1]),('base_link',10.,.2,[0,0,0,1]),('base_link',10.,0,[0,0,0,0]),('base_link',10.,0,[math.nan,0,0,1])]:
            self.assertFalse(h.pose([0,0,1],q,frame,stamp,age,10.))
        self.pose(h,10.);d=h.snapshot(10.)
        self.assertIsNone(current_target(d,10.16));self.assertIsNone(current_target(d,10.01,11.))
        self.assertFalse(h.pose([0,0,1],[0,0,0,1],'base_link',9.,0,10.01))
    def test_sample_count_and_time_window_are_bounded(self):
        h=self.history()
        for i in range(2000):
            t=10+i*.004;self.pose(h,t);d=h.snapshot(t)
        self.assertLessEqual(len(d['trail']),301)
        self.assertTrue(all(0<=t-v['received_monotonic']<5 for v in d['trail']))
    def test_display_subscribes_to_mapped_hand_target(self):
        text=(Path(__file__).parent/'operator_audio.py').read_text()
        self.assertIn("self.create_subscription(PoseStamped,'/v3/hand_target',self.target_update,1)",text)
        self.assertNotIn('create_publisher',text)


class Readout(unittest.TestCase):
    def test_numbers_match_target_in_mm_and_fixed_xyz_degrees(self):
        os.environ['QT_QPA_PLATFORM']='offscreen'
        from PyQt5.QtWidgets import QApplication
        from operator_panel import Panel
        app=QApplication.instance() or QApplication([])
        p=Panel(preview=True)
        try:
            h=TargetHistory(10.);h.inputs(dict(held=True,age_s=0),10.)
            h.pose([.2,-.1,.8],[0,0,math.sqrt(.5),math.sqrt(.5)],'base_link',10.,0,10.)
            p.show_target(h.snapshot(10.01),10.01)
            self.assertIn('X +200.0',p.target_position.text());self.assertIn('Y -100.0',p.target_position.text());self.assertIn('Rz +90.0',p.target_rotation.text())
            p.show_target({},10.3);self.assertIn('X —',p.target_position.text())
        finally:p.close();app.processEvents()


if __name__=='__main__':unittest.main()
