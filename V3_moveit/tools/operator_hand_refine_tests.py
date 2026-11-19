"""Theme consistency, genuine sensor channels, bounded history and drag gating."""
import copy,math,os,tempfile,time,unittest
os.environ['QT_QPA_PLATFORM']='offscreen'
from pathlib import Path
from unittest.mock import patch,MagicMock
import numpy as np
from PyQt5 import QtCore,QtGui,QtWidgets as W,QtTest
from operator_panel import Panel
from operator_settings import Preferences
from operator_grasp_tests import tactile
from operator_tactile import decode_tactile
from operator_tactile_plot import resultant
from operator_hand_model import transforms

class Refine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=W.QApplication.instance() or W.QApplication([])
    def setUp(self):self.p=Panel(preview=True)
    def tearDown(self):self.p.grasp.process=None;self.p.close();self.app.processEvents()
    def test_first_frame_session_matches_both_saved_themes(self):
        self.p.close()
        for value,color in ((0,'#1d1f23'),(100,'#ffffff')):
            prefs=Preferences();prefs.data['brightness']=value
            with patch('operator_studio.Preferences',return_value=prefs):self.p=Panel(preview=True)
            self.p.show();self.app.processEvents()
            for widget in (self.p.session_controls,self.p.scene.parentWidget()):
                self.assertEqual(widget.grab().toImage().pixelColor(50,8).name(),color)
            self.assertFalse(isinstance(self.p.brightness,W.QSlider));self.assertEqual(self.p.brightness.value(),value);self.p.close()
    def test_click_switch_finishes_and_survives_page_changes(self):
        p=self.p;p.show();p.brightness.setValue(100);QtTest.QTest.qWait(230);p.brightness.click();QtTest.QTest.qWait(240)
        self.assertEqual(p.prefs.data['brightness'],0);self.assertIs(type(p.prefs.data['brightness']),int)
        p.set_page(4);p.set_page(0);self.app.processEvents();self.assertEqual(p.session_controls.grab().toImage().pixelColor(50,8).name(),'#1d1f23')
        self.assertFalse(hasattr(p,'tactile_column'))
    def test_optional_pressure_requires_unit_coordinates_and_fresh_frame(self):
        d=tactile();f=d['fingers'][0];f.update(pressure=[.2,.8],pressure_xy=[[.4,.2],[.6,.8]],pressure_unit='normalized')
        good=decode_tactile(d,100.01);self.assertEqual(good['fingers'][0]['pressure'],[.2,.8])
        for key,value in [('pressure',[float('nan'),.8]),('pressure',[1.1,.8]),('pressure_xy',[[.4,.2]]),('pressure_unit','N')]:
            bad=copy.deepcopy(d);bad['fingers'][0][key]=value;decoded=decode_tactile(bad,100.01)
            self.assertEqual(decoded['fingers'][0]['pressure'],[]);self.assertEqual(decoded['fingers'][0]['normal_n'],[.4,.8])
        with self.assertRaises(ValueError):decode_tactile(d,100.09)
    def test_history_deduplicates_preserves_peak_and_marks_gaps(self):
        wave=self.p.grasp_tactile.wave
        for i in range(1000):
            d=tactile(100+i*.001);d['sequence']=i+1;d['fingers'][2]['normal_n']=[9.9 if i==501 else .1,.1];state=decode_tactile(d,100+i*.001+.001);wave.ingest(state);wave.ingest(state)
        self.assertEqual(len(wave.samples),1000);self.assertAlmostEqual(wave.samples[501][1][2][1],10)
        d=tactile(102);d['sequence']=1001;wave.ingest(decode_tactile(d,102.01));self.assertTrue(math.isnan(wave.samples[-2][1][0][0]))
        self.assertEqual(wave.samples.maxlen,12000)
    def test_resultant_does_not_invent_unknown_tangential_direction(self):
        vector,valid=resultant(dict(normal_n=[3],tangent_n=[4],direction_deg=[0]));self.assertTrue(valid);self.assertAlmostEqual(np.linalg.norm(vector),5)
        vector,valid=resultant(dict(normal_n=[3],tangent_n=[4],direction_deg=[None]));self.assertFalse(valid)
    def test_model_and_sliders_sync_without_implicit_hardware(self):
        p=self.p
        with patch.object(p.grasp,'manual_send') as send:
            p.apply_model_joint(2,1100);self.assertEqual(p.manual_targets[2][1].value(),1100);self.assertEqual(p.hand_model.values[2],1100);send.assert_not_called()
            p.grasp_mode.setCurrentIndex(1);p.apply_model_joint(2,1300);self.assertEqual(p.manual_targets[2][1].value(),1100);send.assert_not_called()
    def test_hardware_model_targets_obey_existing_arm_and_limits(self):
        p=self.p;p.preview=False;p.grasp_mode.setCurrentIndex(1);p.grasp.process=MagicMock();p.grasp.mode='manual';p.grasp.armed=True;p.grasp.stopping=False
        p.grasp_endpoints[2][1].setValue(1200)
        with patch.object(p.grasp,'manual_send',return_value=True) as send:
            p.apply_model_joint(2,10000);self.assertEqual(send.call_args.args[1],1200)
            p.grasp.stopping=True;p.apply_model_joint(2,100);self.assertEqual(send.call_count,1)
        p.preview=True
    def test_vendor_linkage_is_articulated_and_finite(self):
        opened=transforms([0]*6);closed=transforms([0,0,10000,0,0,0]);self.assertEqual(len(opened),17)
        self.assertFalse(np.allclose(opened['finger23_link'],closed['finger23_link']))
        self.assertTrue(np.allclose(opened['finger33_link'],closed['finger33_link']))
        self.assertTrue(all(np.isfinite(t).all() and abs(np.linalg.det(t[:3,:3])-1)<1e-9 for t in closed.values()))
        # Vendor pads face +X: closing must travel toward the palm, not its back.
        tip=np.array([0,0,.025,1]);self.assertGreater((closed['finger23_link']@tip)[0],(opened['finger23_link']@tip)[0])
        self.assertGreater((opened['finger14_link']@tip)[0],opened['finger11_link'][0,3])

if __name__=='__main__':unittest.main()
