"""Workspace behavior, transport boundaries and display expiry; no hardware."""
import copy,io,json,os,socket,struct,tempfile,time,unittest
from pathlib import Path
from unittest.mock import MagicMock,patch
os.environ['QT_QPA_PLATFORM']='offscreen'
from PyQt5 import QtWidgets as W,QtCore
from operator_panel import Panel
from operator_grasp import GraspConnection
from operator_hand import decode_sample
from operator_hand_tests import sample
from operator_tactile import decode_tactile,HandForces
from operator_grasp_tests import tactile
from operator_hand_workspace import refresh_monitor
from grasp_profiles import Presets,validate
from operator_hand_access import native_command,connection_error

class Workspace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=W.QApplication.instance() or W.QApplication([])
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.presets=Presets(Path(self.tmp.name)/'profiles.json')
        with patch('operator_grasp.Presets',return_value=self.presets):self.p=Panel(preview=True)
    def tearDown(self):self.p.grasp.process=None;self.p.hand.process=None;self.p.job=None;self.p.preview=True;self.p.close();self.tmp.cleanup()
    def test_five_modules_and_manufacturer_artwork(self):
        p=self.p;p.set_page(4);p.show();self.app.processEvents()
        self.assertEqual([p.hand_bottom_tabs.tabText(i) for i in range(3)],['关节监控','关节配置','日志窗口'])
        self.assertEqual(p.hand_control_tabs.tabText(0),'关节控制');self.assertFalse(p.grasp_hand.image.isNull())
        self.assertEqual((p.grasp_hand.image.width(),p.grasp_hand.image.height()),(517,580))
        self.assertEqual(p.grasp_tactile.table.rowCount(),5);self.assertEqual(p.grasp_tactile.table.item(0,2).text(),'—')
        self.assertFalse(p.session_controls.isVisible());self.assertTrue(p.grasp_all_stop.isVisible());self.assertTrue(p.mode.isVisible())
        p.set_page(0);self.assertTrue(p.session_controls.isVisible())
    def test_preview_sliders_and_zero_never_start_or_send(self):
        p=self.p
        with patch.object(p.grasp,'start') as start,patch.object(p.grasp,'manual_send') as send:
            p.manual_targets[2][0].setValue(1234);p.apply_manual_joint(2);p.zero_joint(2);p.zero_all_joints()
            p.grasp_mode.setCurrentIndex(1);p.request_grasp_mode('manual')
            start.assert_not_called();send.assert_not_called()
    def test_both_home_actions_stay_next_to_connect_on_both_tabs(self):
        p=self.p;p.set_page(4);p.resize(1240,820);p.show()
        for tab in (0,1):
            p.hand_control_tabs.setCurrentIndex(tab);self.app.processEvents()
            buttons=(p.grasp_monitor,p.grasp_home,p.grasp_zero_all)
            for first,second in zip(buttons,buttons[1:]):
                self.assertTrue(first.isVisible() and second.isVisible())
                self.assertEqual(first.geometry().top(),second.geometry().top())
                self.assertTrue(0<=second.x()-first.geometry().right()<=20)
        p.manual_targets[2][0].setValue(1000)
        with patch.object(p.grasp,'start') as start:
            p.grasp_zero_all.click();self.assertEqual(p.manual_targets[2][0].value(),0);start.assert_not_called()
    def test_connect_opens_persistent_hand_and_preview_stays_offline(self):
        p=self.p
        with patch.object(p.grasp,'start') as start:
            p.grasp_mode.setCurrentIndex(1);p.grasp_monitor.click();start.assert_not_called()
            p.preview=False;p.grasp_monitor.click()
            self.assertEqual(start.call_args.args[:2],(p.hand_interface.currentText(),'workspace'))

    def test_workspace_changes_source_in_place_and_natural_teleop_end_disconnects(self):
        p=self.p;p.preview=False;p.grasp_mode.setCurrentIndex(1)
        child=MagicMock();child.write.side_effect=len;child.bytesToWrite.return_value=0
        hand=p.grasp;hand.persistent=True;hand.ready=True;hand.armed=True;hand.mode='manual'
        hand.profile=p.draft_grasp();hand.process=child
        hand.send_input(1,time.monotonic(),0,True)
        self.assertTrue(child.write.call_args.args[0].startswith(b'T 1 '))
        p.request_grasp_mode('control');self.assertEqual(child.write.call_args.args[0],b'S trigger\n')
        hand.mode='control';p.request_grasp_mode('monitor');self.assertEqual(child.write.call_args.args[0],b'S disabled\n')
        child.terminate.assert_not_called();self.assertIs(hand.process,child)
        hand.mode='monitor';p.hand_teleop=True;p.using_hardware_teleop=True;p.session_finished(0)
        child.terminate.assert_called_once();self.assertFalse(p.hand_teleop)
    def test_position_zero_ack_keeps_manual_connection_and_updates_targets(self):
        p=self.p;p.preview=False;p.grasp_mode.setCurrentIndex(1)
        hand=p.grasp;child=MagicMock();child.write.side_effect=len;child.bytesToWrite.return_value=0
        hand.persistent=True;hand.process=child;hand.mode='manual';hand.ready=True;hand.armed=True
        hand.profile=p.draft_grasp();hand.state={'feedback_verified':True,'updated':time.monotonic(),'ok':True,'homed':True,
                                               'axis_status':[0x13]*6,'raw_positions':[200]*6}
        p.refresh_grasp();p.grasp_zero_all.click();self.assertEqual(child.write.call_args.args,(b'S open\n',))
        hand.log=io.StringIO();child.readAllStandardOutput.return_value=b'{"event":"workspace","source":"manual","armed":true,"reason":"opening"}\n'
        with patch('operator_grasp.atomic'):hand.consume()
        self.assertEqual(hand.mode,'manual');self.assertTrue(hand.armed and p.manual_synced)
        self.assertEqual([spin.value() for spin,_ in p.manual_targets],[0]*6)
        child.terminate.assert_not_called()

    def test_editor_tracks_and_speeds_apply_without_reopening(self):
        p=self.p;editor=p.action_editor;editor.finger.setCurrentIndex(1)
        editor.edit_endpoint(1,'start_ms',300);editor.edit_endpoint(1,'end_ms',800)
        editor.force.setValue(1.25);editor.closing.setValue(48);editor.opening.setValue(68)
        hand=p.grasp;child=MagicMock();child.write.side_effect=len
        hand.ready=True;hand.persistent=True;hand.mode='manual';hand.process=child;hand.profile=p.draft_grasp()
        p.save_grasp()
        current=Presets(self.presets.path).profile
        self.assertEqual(current['tracks'][1],dict(start_ms=300,end_ms=800,force_n=1.25))
        self.assertEqual((current['close_percent'],current['open_percent']),(48,68))
        command=child.write.call_args.args[0]
        self.assertTrue(command.startswith(b'P 1000 9600 13600 '));self.assertTrue(command.endswith(b'\nS manual\n'))
        child.terminate.assert_not_called()
    def test_arm_ready_while_hand_is_opening_selects_trigger_on_hand_ready(self):
        p=self.p;p.preview=False;p.grasp_mode.setCurrentIndex(1)
        hand=p.grasp;hand.persistent=True;hand.mode='workspace';hand.initial_source='manual'
        hand.process=MagicMock();p.using_hardware_teleop=True;p.hand_teleop=True
        p.hand_waiting_trigger=True
        p.consume_event(dict(ready=True,message='已就绪'))
        self.assertEqual(hand.initial_source,'trigger')
        hand.process.write.assert_not_called()
        self.assertFalse(p.hand_waiting_trigger)
    def test_unhomed_feedback_keeps_monitor_and_requires_confirmed_home(self):
        p=self.p;p.preview=False;p.grasp_mode.setCurrentIndex(1)
        p.grasp.process=MagicMock();p.grasp.ready=True;p.grasp.mode='monitor'
        p.grasp.state=dict(feedback_verified=True,ok=True,homed=False,updated=time.monotonic(),axis_status=[6]*6)
        p.refresh_grasp();self.assertTrue(p.grasp_home.isEnabled());self.assertFalse(p.grasp_manual_enable.isEnabled())
        p.request_grasp_mode('manual');self.assertIsNone(p.pending_grasp)
        with patch.object(W.QMessageBox,'question',return_value=W.QMessageBox.Cancel):
            p.grasp_home.click();p.grasp.process.terminate.assert_not_called()
        with patch.object(W.QMessageBox,'question',return_value=W.QMessageBox.Yes):
            p.grasp.state['updated']=time.monotonic();p.grasp_home.click()
        self.assertEqual(p.pending_grasp[0],'home');p.grasp.process.terminate.assert_called_once()
        p.stop_grasp();self.assertIsNone(p.pending_grasp)
    def test_home_completion_reopens_only_monitor_after_verified_cleanup(self):
        base=dict(event='closed',init_confirmed=True,homing_passed=True,disabled=True,stop_rc=0,disable_rc=0)
        for changes,code,cancelled,expected in [({},0,False,True),({'homing_passed':False},0,False,False),
                ({'disabled':False},0,False,False),({'init_confirmed':False},0,False,False),
                ({'stop_rc':1},0,False,False),({},1,False,False),({},0,True,False)]:
            with self.subTest(changes=changes,code=code,cancelled=cancelled):
                hand=GraspConnection();hand.mode='home';hand.ready=True;hand.stopping=cancelled
                hand.process=MagicMock();hand.log=io.StringIO()
                hand.process.readAllStandardOutput.return_value=(json.dumps(dict(base,**changes))+'\n').encode()
                hand.home_completed.connect(self.p.reconnect_after_home)
                with patch('operator_grasp.atomic'),patch.object(self.p,'request_grasp_mode') as start:
                    hand.finished(code);self.app.processEvents()
                    if expected:start.assert_called_once_with('monitor')
                    else:start.assert_not_called()
    def test_preview_pose_ignores_live_feedback_and_disconnect_detail(self):
        p=self.p;p.manual_targets[0][0].setValue(910)
        p.hand.state=decode_sample(sample(),100.01);p.hand.state.update(homed=True,updated=time.monotonic())
        p.tick_grasp();self.assertEqual(p.hand_model.values[0],910)
        p.hand.state=dict(ok=False,updated=time.monotonic(),detail='已断开')
        p.tick_grasp();self.assertEqual(p.joint_monitor.item(0,1).text(),'—')
    def test_camera_selection_remains_independent_while_another_streams(self):
        p=self.p;p.device_boxes['c922'].setChecked(False)
        p.device_monitors['zed']=dict(process=MagicMock(),log=MagicMock())
        try:
            p.refresh();self.assertFalse(p.device_boxes['zed'].isEnabled());self.assertTrue(p.device_boxes['c922'].isEnabled())
            p.device_boxes['c922'].click()
            with patch.object(p,'start_device_monitor') as start,patch.object(p,'stop_device_monitor') as stop:
                p.device_module_buttons['c922'].click();start.assert_called_once_with('c922');stop.assert_not_called()
        finally:p.device_monitors.clear()
    def test_slider_release_writes_final_target_to_manual_pipe(self):
        p=self.p;p.preview=False;p.grasp_mode.setCurrentIndex(1)
        proc=MagicMock();proc.bytesToWrite.return_value=0;p.grasp.process=proc;p.grasp.mode='manual';p.grasp.ready=True;p.grasp.armed=True
        p.refresh_grasp();_,slider=p.manual_targets[3]
        slider.setSliderDown(True);slider.setSliderPosition(740);slider.setSliderDown(False)
        command=proc.write.call_args.args[0].decode().split()
        self.assertEqual(command[0],'M');self.assertEqual(command[3:5],['4','740']);self.assertIsNone(p.pending_manual_target)
    def test_monitor_buttons_replace_only_lower_panel_without_touching_connection(self):
        p=self.p;p.show();p.set_page(0);self.app.processEvents();scene=p.scene
        with patch.object(p.grasp,'start') as start,patch.object(p.grasp,'stop') as stop:
            for index in (1,0,1):
                p.monitor_buttons[index].click();self.app.processEvents()
                self.assertEqual(p.pages.currentIndex(),0);self.assertIs(p.scene,scene)
                self.assertEqual(p.monitor_stack.currentIndex(),index)
                self.assertEqual(p.grasp_tactile.isVisible(),index==1)
                self.assertEqual(p.scope_controls.isVisible(),index==0)
            start.assert_not_called();stop.assert_not_called()
        p.set_page(4);self.assertFalse(p.grasp_tactile.isVisible())
    def test_no_password_launcher_requires_only_net_raw_and_explains_busy_bus(self):
        binary=Path(self.tmp.name)/'native'
        with patch('operator_hand_access.os.getxattr',side_effect=OSError):
            self.assertEqual(native_command(binary,['--hardware']),('/usr/bin/pkexec',['--keep-cwd',str(binary),'--hardware']))
        with patch('operator_hand_access.os.getxattr',return_value=struct.pack('<IIIII',0x02000001,1<<13,0,0,0)):
            self.assertEqual(native_command(binary,['--hardware']),(str(binary),['--hardware']))
        self.assertIn('LHandPro',connection_error('Expected one slave in INIT; refusing takeover'))
    def test_per_axis_configuration_round_trips(self):
        p=self.p;p.manual_parameters[2][0].setValue(800);p.manual_parameters[2][1].setValue(400);p.grasp_endpoints[2][1].setValue(1200);p.save_grasp()
        loaded=Presets(self.presets.path).profile
        self.assertEqual(loaded['manual_velocity'][2],800);self.assertEqual(loaded['manual_current'][2],400);self.assertEqual(loaded['closed'][2],1200)
        self.assertEqual(p.manual_targets[2][0].maximum(),1200)
        for key,value in [('manual_current',[901]*6),('manual_velocity',[20001]*6),('manual_current',[200]*5)]:
            bad=copy.deepcopy(self.presets.data);bad['profiles'][0][key]=value
            with self.assertRaises(ValueError):validate(bad)
    def test_monitor_preserves_signed_raw_values_and_clears_stale(self):
        d=sample();raw=bytearray.fromhex(d['raw_hex']);raw[4:6]=(-123).to_bytes(2,'little',signed=True);raw[6:8]=(-17).to_bytes(2,'little',signed=True);d['raw_hex']=raw.hex()
        state=decode_sample(d,100.01);refresh_monitor(self.p,state)
        self.assertEqual(self.p.joint_monitor.item(0,1).text(),'-11');self.assertEqual(self.p.joint_monitor.item(0,2).text(),'-170');self.assertEqual(self.p.joint_monitor.item(0,3).text(),'-123')
        refresh_monitor(self.p,{})
        self.assertTrue(all(self.p.joint_monitor.item(i,j).text()=='—' for i in range(6) for j in range(1,7)))
    def test_optional_proximity_sentinel_and_display_reset(self):
        d=tactile();d['fingers'][0]['proximity']=[65535,24];state=decode_tactile(d,100.01)
        self.assertEqual(state['fingers'][0]['proximity'],[None,24]);self.p.grasp_tactile.set_state(state)
        self.assertEqual(self.p.grasp_tactile.table.rowCount(),10);self.p.grasp_tactile.set_state({})
        self.assertEqual(self.p.grasp_tactile.table.rowCount(),5);self.assertFalse(self.p.grasp_hand.state)
        self.assertEqual(self.p.grasp_tactile.table.item(0,2).text(),'—')
    def test_log_filter_and_clear_do_not_modify_capture(self):
        log=self.p.hand_log;log.clear_view();log.append('debug','packet');log.append('error','fault')
        self.assertEqual(log.table.rowCount(),1);log.filters['debug'].setChecked(True);self.assertEqual(log.table.rowCount(),2)
        log.clear_view();self.assertEqual(log.table.rowCount(),0)
    def test_mode_change_requires_clean_init(self):
        p=self.p;p.preview=False;p.grasp_mode.setCurrentIndex(1);p.grasp.process=MagicMock();p.grasp.mode='monitor';p.grasp.ready=True
        p.grasp.state=dict(feedback_verified=True,ok=True,homed=True,updated=time.monotonic(),axis_status=[22]*6)
        with patch.object(p.grasp,'start') as start,patch.object(QtCore.QTimer,'singleShot') as later:
            p.request_grasp_mode('manual');self.assertIsNotNone(p.pending_grasp);p.grasp.process.terminate.assert_called_once()
            p.grasp.process=None;p.grasp.closed=False;p.refresh_grasp();later.assert_not_called();start.assert_not_called();self.assertIsNone(p.pending_grasp)
    def test_stop_cancels_queued_enable_after_clean_disconnect(self):
        p=self.p;p.preview=False;p.grasp_mode.setCurrentIndex(1);p.grasp.mode='monitor';p.grasp.ready=True;p.grasp.closed=True;p.pending_grasp=('manual',(p.draft_grasp(),'enp11s0'))
        callbacks=[]
        with patch.object(QtCore.QTimer,'singleShot',side_effect=lambda _,fn:callbacks.append(fn)),patch.object(p,'start_grasp') as start:
            p.refresh_grasp();self.assertEqual(len(callbacks),1);p.stop_session();callbacks[0]();start.assert_not_called()
    def test_manual_pipe_requires_arm_and_applies_backpressure(self):
        hand=GraspConnection();hand.process=MagicMock();hand.process.bytesToWrite.return_value=0;hand.mode='manual';hand.ready=True
        self.assertFalse(hand.manual_send(2,500,1000,600));self.assertFalse(hand.manual_send());hand.process.write.assert_not_called()
        hand.armed=True;self.assertTrue(hand.manual_send());self.assertTrue(hand.manual_send(2,500,1000,600));messages=[c.args[0].decode() for c in hand.process.write.call_args_list]
        self.assertTrue(messages[0].startswith('H 1 '));self.assertTrue(messages[1].startswith('M 2 '));self.assertTrue(messages[1].endswith(' 2 500 1000 600\n'))
        hand.process.bytesToWrite.return_value=2048;self.assertFalse(hand.manual_send());hand.process.terminate.assert_called_once();hand.process=None
    def test_manual_start_passes_all_six_parameter_pairs(self):
        root=Path(self.tmp.name);binary=root/'.cache/dh116/build/v3_dh116_grasp';binary.parent.mkdir(parents=True);binary.touch()
        profile=self.presets.profile;profile['manual_velocity']=[1000,1100,1200,1300,1400,1500];profile['manual_current']=[200,300,400,500,600,700]
        child=MagicMock();hand=GraspConnection()
        with patch('operator_grasp.ROOT',root),patch('operator_grasp.QtCore.QProcess',return_value=child),patch('operator_hand_access.os.getxattr',return_value=struct.pack('<IIIII',0x02000001,1<<13,0,0,0)):hand.start('enp11s0','manual',profile)
        args=child.start.call_args.args[1]
        self.assertEqual(args[-12:],[str(v) for v in profile['manual_velocity']+profile['manual_current']]);self.assertEqual(args[3:5],['1500','700'])
        hand.log.close();hand.process=None
    def test_all_open_respects_lowest_axis_parameter_limit(self):
        root=Path(self.tmp.name);binary=root/'.cache/dh116/build/v3_dh116_grasp';binary.parent.mkdir(parents=True);binary.touch()
        profile=self.presets.profile;profile.update(velocity=2000,current_per_mille=600,manual_velocity=[1200]*5+[800],manual_current=[400]*5+[200])
        child=MagicMock();hand=GraspConnection()
        with patch('operator_grasp.ROOT',root),patch('operator_grasp.QtCore.QProcess',return_value=child),patch('operator_hand_access.os.getxattr',return_value=struct.pack('<IIIII',0x02000001,1<<13,0,0,0)):hand.start('enp11s0','open',profile)
        self.assertEqual(child.start.call_args.args[1][3:5],['800','200'])
        self.assertEqual(profile['velocity'],2000);self.assertEqual(profile['current_per_mille'],600)
        hand.log.close();hand.process=None

if __name__=='__main__':unittest.main()
