"""Interaction continuity and nonblocking credits, without hardware or OpenGL."""
import os,time,unittest
os.environ['QT_QPA_PLATFORM']='offscreen'
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from PyQt5 import QtCore,QtWidgets as W,QtTest
from operator_panel import Panel
from operator_hand_model import drag_tangent

class CreditsDragTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=W.QApplication.instance() or W.QApplication([])
    def setUp(self):self.p=Panel(preview=True)
    def tearDown(self):self.p.hand_model.gl=None;self.p.close();self.app.processEvents()
    def test_about_is_single_nonmodal_window_and_follows_theme(self):
        p=self.p;p.show();p.about_button.click();a=p.about_dialog;self.assertTrue(a.isVisible());self.assertEqual(a.windowModality(),QtCore.Qt.NonModal)
        p.about_button.click();self.assertIs(p.about_dialog,a)
        p.brightness.setValue(0);QtTest.QTest.qWait(230);self.assertEqual(a.roll.bg,'#1d1f23')
        a.roll.wait_until=0;a.roll.set_playing(True);QtTest.QTest.qWait(90);self.assertGreater(a.roll.offset,0)
        a.roll.verticalScrollBar().setValue(250);self.assertFalse(a.roll.playing);self.assertEqual(a.roll.offset,250)
        a.roll.restart();self.assertTrue(a.roll.playing);self.assertEqual(a.roll.offset,0)
        a.close();self.assertFalse(a.roll.timer.isActive());self.assertIsNone(p.grasp.process)
        p.show_about();p.close();self.assertFalse(a.isVisible());self.assertFalse(a.roll.timer.isActive())
    def test_near_end_on_projection_cannot_amplify_one_pixel(self):
        for slope in (0,.00001,.0025,.02,.1):
            t=drag_tangent([slope,0],600,2)
            self.assertLessEqual(1/np.linalg.norm(t),10000/450+1e-8)
            self.assertTrue(np.isfinite(t).all())
    def test_sparse_pointer_events_fill_continuously_but_commands_stay_bounded(self):
        m=self.p.hand_model;m.gl=SimpleNamespace(height=lambda:600);item=SimpleNamespace(axis_index=2,part_name='finger21_link',local_center=[0,0,0]);clock=[10.];previews=[];commands=[]
        m.previewChanged.connect(lambda a,v:previews.append((clock[0],a,v)));m.targetChanged.connect(lambda a,v:commands.append((clock[0],a,v)))
        with patch.object(m,'screen_point',side_effect=lambda n,p,v:np.array([v[2]*.0025,0.])),patch.object(m,'update_pose'),patch.object(m,'recolor'),patch('operator_hand_model.time.monotonic',side_effect=lambda:clock[0]):
            m.begin_drag(item,QtCore.QPoint(0,0));m.move_drag(QtCore.QPoint(225,0))
            for i in range(1,61):clock[0]=10+i/120;m.advance_drag()
            values=[v for _,_,v in previews];self.assertGreater(len(values),len(commands));self.assertGreater(len(values),10)
            self.assertTrue(all(a<=b for a,b in zip(values,values[1:])));self.assertEqual(values[-1],5000)
            self.assertTrue(all(b[0]-a[0]>=1/30-1e-8 for a,b in zip(commands,commands[1:])))
            self.assertEqual(commands[-1][2],5000)
            m.move_drag(QtCore.QPoint(300,0),final=True);self.assertEqual(commands[-1][2],6667)
            m.set_allowed(False);count=len(commands);clock[0]+=1;m.advance_drag();self.assertEqual(len(commands),count);self.assertFalse(m.drag_timer.isActive())
    def test_click_without_movement_and_cancel_never_emit_target(self):
        m=self.p.hand_model;m.gl=SimpleNamespace(height=lambda:600);item=SimpleNamespace(axis_index=2,part_name='finger21_link',local_center=[0,0,0]);commands=[];m.targetChanged.connect(lambda *v:commands.append(v))
        with patch.object(m,'screen_point',return_value=np.array([0.,0.])),patch.object(m,'update_pose'),patch.object(m,'recolor'):
            m.begin_drag(item,QtCore.QPoint(5,5));m.move_drag(QtCore.QPoint(5,5),final=True);self.assertEqual(commands,[])
            m.move_drag(QtCore.QPoint(200,200));m.cancel_drag();QtTest.QTest.qWait(30);self.assertEqual(commands,[])

if __name__=='__main__':unittest.main()
