"""Cabinet preparation safety and UI gates using a fake SDK; no robot sockets."""
import json,os,subprocess,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch,MagicMock
os.environ['QT_QPA_PLATFORM']='offscreen'
from operator_arm import ROOT,native_arguments
from PyQt5 import QtWidgets as W
from operator_panel import Panel

class Native(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory(dir=ROOT/'.tmp');cls.binary=Path(cls.temp.name)/'fake_arm'
        subprocess.run(['g++','-std=c++17','-pthread','-I'+str(ROOT/'src/v3_moveit/include'),'-I'+str(ROOT/'vendor/jaka/include'),str(ROOT/'tools/operator_arm_power.cpp'),str(ROOT/'tools/operator_arm_power_fake.cpp'),'-o',str(cls.binary)],check=True)
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def execute(self,action,case,tool_id=None):
        _,args=native_arguments(action,tool_id=tool_id);env=dict(os.environ,V3_ARM_CASE=case,V3_ARM_PAYLOAD=' '.join(args[3:7]),V3_ARM_FK=' '.join(args[13:19]))
        result=subprocess.run([str(self.binary),*args],cwd=ROOT,env=env,capture_output=True,text=True,timeout=5)
        rows=[json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')];calls=next((r for r in rows if r['event']=='fake_calls'),dict(power_on=0,enable=0,set_tool_id=0))
        return result.returncode,calls,rows
    def test_status_never_powers_or_enables(self):
        rc,calls,_=self.execute('status','off');self.assertEqual(rc,0);self.assertEqual((calls['power_on'],calls['enable']),(0,0))
        rc,calls,_=self.execute('check','off');self.assertEqual(rc,0);self.assertEqual((calls['power_on'],calls['enable']),(0,0))
    def test_power_and_enable_are_separate_and_idempotent(self):
        for action,case,expected in [('power_on','off',(1,0)),('power_on','powered',(0,0)),('enable','powered',(0,1)),('enable','already_enabled',(0,0))]:
            rc,calls,rows=self.execute(action,case);self.assertEqual(rc,0,rows);self.assertEqual((calls['power_on'],calls['enable']),expected)
    def test_enable_requires_power(self):
        rc,calls,_=self.execute('enable','off');self.assertNotEqual(rc,0);self.assertEqual(calls['enable'],0)
    def test_faults_ownership_model_and_feedback_reject_before_setter(self):
        for case in ('alarm','estop','collision','queue','servo','drag','program','feedback_error','tool','tcp','payload','centroid','collision_disabled','model'):
            with self.subTest(case=case):
                rc,calls,rows=self.execute('enable',case);self.assertNotEqual(rc,0,rows);self.assertEqual((calls['power_on'],calls['enable']),(0,0))
    def test_sdk_failure_is_not_reported_as_success(self):
        rc,calls,rows=self.execute('enable','command_error');self.assertNotEqual(rc,0);self.assertEqual(calls['enable'],1);self.assertFalse(rows[-1]['ok'])
    def test_no_motion_or_servo_setter_imports(self):
        used=subprocess.check_output(['nm','-D','--undefined-only',str(ROOT/'.cache/operator/arm_build/v3_arm_power')],text=True)
        for name in ('servo_move_enable','edg_init','joint_move','linear_move','set_tool_data','set_payload'):self.assertNotIn(name,used)
        for name in ('power_on','enable_robot'):self.assertIn(name,used)
    def test_tool_switch_readback_and_precision(self):
        rc,calls,rows=self.execute('select_tool','powered',3)
        self.assertEqual(rc,0,rows);self.assertEqual(calls['set_tool_id'],1)
        last=[r for r in rows if r.get('event')=='tooling'][-1]
        self.assertEqual(last['tool_id'],3);self.assertEqual(last['tcp_mm_rad'][0],12.345678901234567)
        self.assertEqual((calls['power_on'],calls['enable']),(0,0))
    def test_failed_switch_does_not_report_success(self):
        for case in ('servo','drag','program','queue','alarm','missing_tool','tool_error','tool_unconfirmed'):
            rc,calls,rows=self.execute('select_tool',case,3)
            self.assertNotEqual(rc,0,rows);self.assertFalse(rows[-1]['ok'])
            if case not in ('tool_error','tool_unconfirmed'):self.assertEqual(calls['set_tool_id'],0)
    def test_refresh_can_read_changed_centroid_without_motion(self):
        rc,calls,rows=self.execute('refresh_payload','centroid')
        self.assertEqual(rc,0,rows);self.assertEqual((calls['set_tool_id'],calls['power_on'],calls['enable']),(0,0,0))
        rc,_,rows=self.execute('refresh_payload','tool');self.assertNotEqual(rc,0,rows)
    def test_poll_is_readonly_and_connection_errors_are_not_mislabeled_busy(self):
        rc,calls,rows=self.execute('poll','already_enabled');self.assertEqual(rc,0)
        self.assertEqual((calls['power_on'],calls['enable'],calls['set_tool_id']),(0,0,0))
        self.assertFalse(any(r['event']=='tooling' for r in rows))
        for scenario,kind in [('connect_error','error'),('edg_owned','occupied')]:
            rc,_,rows=self.execute('poll',scenario);self.assertNotEqual(rc,0)
            self.assertEqual(next(r for r in rows if r['event']=='error')['kind'],kind)
    def test_async_monitor_reads_occupancy_and_releases_before_handoff(self):
        import operator_arm
        app=W.QApplication.instance() or W.QApplication([])
        root=Path(self.temp.name)/'monitor';(root/'.cache/operator').mkdir(parents=True,exist_ok=True);(root/'src/v3_moveit/config').mkdir(parents=True,exist_ok=True)
        (root/'src/v3_moveit/config/payload.json').write_bytes((ROOT/'src/v3_moveit/config/payload.json').read_bytes())
        _,args=native_arguments('poll');env=dict(V3_ARM_PAYLOAD=' '.join(args[3:7]),V3_ARM_FK=' '.join(args[13:19]))
        control=operator_arm.ArmControl();control.connection_requested=True
        with patch.object(operator_arm,'ROOT',root),patch.object(operator_arm,'native_arguments',return_value=(str(self.binary),args)),patch.dict(os.environ,env):
            for case,expected in [('already_enabled','connected'),('queue','occupied'),('connect_error','error')]:
                with patch.dict(os.environ,V3_ARM_CASE=case):
                    control.start('poll',background=True);deadline=time.monotonic()+3
                    while control.process is not None and time.monotonic()<deadline:app.processEvents();time.sleep(.01)
                    self.assertIsNone(control.process);self.assertEqual(control.connection_state,expected)
            called=[];control.start('poll',background=True);self.assertTrue(control.after_poll(lambda:called.append('released')));self.assertTrue(control.busy)
            deadline=time.monotonic()+3
            while not called and time.monotonic()<deadline:app.processEvents();time.sleep(.01)
            self.assertEqual(called,['released']);self.assertIsNone(control.process)

class PanelGates(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=W.QApplication.instance() or W.QApplication([])
    def setUp(self):self.p=Panel(preview=True)
    def tearDown(self):self.p.arm_control.process=None;self.p.job=None;self.p.preview=True;self.p.close()
    def test_stop_cancels_start_waiting_for_status_poll(self):
        p=self.p;control=p.arm_control;control.background=True;control.process=MagicMock()
        with patch.object(p,'launch_command') as start:
            p.launch('teleop');self.assertIsNotNone(control.deferred)
            control.process=None
            p.stop_session();control.run_deferred()
            start.assert_not_called()
    def test_queued_fake_launch_cannot_become_a_hardware_launch(self):
        p=self.p;control=p.arm_control;control.background=True;control.process=MagicMock()
        with patch('resource.getrlimit',return_value=(50,50)),patch.object(p,'launch_command') as start:
            p.launch('teleop');self.assertIsNotNone(control.deferred)
            control.process=None;p.mode.setCurrentIndex(1);p.site.setChecked(True)
            control.run_deferred();start.assert_not_called()
    def test_preview_simulation_site_and_running_session_block_power(self):
        p=self.p
        with patch.object(p.arm_control,'start') as start:
            p.arm_action('power_on');p.mode.setCurrentIndex(1);p.site.setChecked(True);p.arm_action('enable');start.assert_not_called()
            p.preview=False;p.mode.setCurrentIndex(0);p.arm_action('power_on');start.assert_not_called()
            p.mode.setCurrentIndex(1);p.site.setChecked(False);p.arm_action('power_on');start.assert_not_called()
            p.site.setChecked(True);p.job=MagicMock();p.arm_action('enable');start.assert_not_called()
            p.job=None;p.arm_action('power_on');start.assert_called_once_with('power_on')
    def test_status_is_readonly_and_quest_repair_does_not_replace_session(self):
        p=self.p;p.preview=False;p.mode.setCurrentIndex(1)
        with patch.object(p.arm_control,'start') as start:p.arm_action('status');start.assert_called_once_with('status')
        job=p.job=MagicMock()
        with patch.object(p.quest_link,'repair') as repair,patch.object(p,'launch_command') as launch:
            p.connect_quest();repair.assert_called_once();launch.assert_not_called();self.assertIs(p.job,job);self.assertTrue(p.quest.isEnabled())
    def test_tool_controls_default_and_session_lock(self):
        p=self.p;self.assertEqual(p.tool_select.currentData(),1)
        p.preview=False;p.mode.setCurrentIndex(1);p.site.setChecked(True)
        with patch.object(p.arm_control,'start') as start:
            p.tool_select.setCurrentIndex(3);p.arm_action('select_tool');start.assert_called_once_with('select_tool',tool_id=3)
            start.reset_mock();p.job=MagicMock();p.refresh();self.assertFalse(p.tool_apply.isEnabled());self.assertFalse(p.payload_refresh.isEnabled())
            p.arm_action('refresh_payload');start.assert_not_called()
    def test_failure_popup_is_once_per_attempt_and_repeats_next_attempt(self):
        p=self.p
        event=dict(message='请先在 JAKA 上位机重新检测负载质量和质心位置',error=True,recovery_kind='payload')
        p.consume_event(event);first=p.tool_dialog;self.assertIsNotNone(first)
        p.consume_event(event);self.assertIs(p.tool_dialog,first)
        first.close();self.app.processEvents();p.tool_notices.clear()
        p.consume_event(event);self.assertIsNotNone(p.tool_dialog);p.tool_dialog.close();self.app.processEvents()
    def test_connection_buttons_preserve_fresh_state_and_show_occupied(self):
        p=self.p;c=p.arm_control;c.connection_requested=True;c.connection_state='connected'
        c.state=dict(powered_on=1,enabled=1,updated=time.monotonic())
        p.refresh();self.assertIn('已连接',p.arm_connect_button.text());self.assertIn('已开启',p.arm_power_button.text());self.assertIn('已使能',p.arm_enable_button.text());self.assertTrue(p.arm_power_status.isHidden())
        c.background=True;c.process=MagicMock();p.refresh();self.assertTrue(p.start.isEnabled());self.assertIn('已连接',p.arm_connect_button.text())
        c.process=None;c.connection_state='occupied';p.refresh();self.assertIn('被占用',p.arm_connect_button.text());self.assertEqual(p.arm_connect_button.property('armState'),'occupied')
    def test_active_connection_refreshes_without_clicking_read_again(self):
        p=self.p;p.preview=False;p.arm_control.connection_requested=True;p.arm_control.last_poll=time.monotonic()-40
        with patch.object(p.arm_control,'start') as start:p.poll_arm_connection();start.assert_called_once_with('poll',background=True)
        p.job=MagicMock()
        with patch.object(p.arm_control,'start') as start:p.poll_arm_connection();start.assert_not_called()
    def test_tooling_appears_in_anchored_popup_without_permanent_readout(self):
        p=self.p;p.show();self.app.processEvents();p.open_tooling();self.app.processEvents()
        self.assertTrue(p.tooling_popover.isVisible());self.assertTrue(p.payload_status.isHidden());self.assertIn('TCP1',p.tooling_popover.cells['tool',1].text())
        p.tooling_popover.hide()
    def test_read_failure_stays_visible_in_button_and_popover(self):
        p=self.p;p.preview=False
        with patch.object(p.arm_control,'start',side_effect=ValueError('机器人地址与工程目标不符')):p.arm_action('status')
        self.assertIn('连接失败',p.arm_connect_button.text());self.assertIn('机器人地址',p.tooling_popover.status.text())
        self.assertIn('机器人地址',p.status.text())

if __name__=='__main__':unittest.main()
