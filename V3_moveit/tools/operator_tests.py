"""Operator lifecycle/gate tests; no ROS, SDK or real robot access."""
import argparse,contextlib,hashlib,io,json,os,signal,sys,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch,MagicMock
import operator_session as s

class Gates(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=s.ROOT/'.tmp');self.root=Path(self.tmp.name);self.root_patch=patch.object(s,'ROOT',self.root);self.root_patch.start()
        self.modules=patch.dict(sys.modules,{'verify_offline':SimpleNamespace(source_digest=lambda:'v',calibration_digest=lambda root:'calibration')});self.modules.start()
        for p in ['artifacts/quest_commissioning_hardware','src/v3_moveit/config','tools']:(self.root/p).mkdir(parents=True,exist_ok=True)
        (self.root/'site_ubuntu.yaml').write_text('robot_ip: 192.168.110.224\n')
        (self.root/'src/v3_moveit/config/runtime.yaml').write_text('auto_replan: true\nrobot_ip: 192.168.110.224\njoint_velocity_rad_s: 0.1745\n')
        (self.root/'runtime.yaml').write_text('auto_replan: false\nrobot_ip: 192.168.110.224\n')
        (self.root/'tools/hardware_quest_commissioning.py').write_text('observer')
        capture=self.root/'artifacts/capture.jsonl';capture.write_text('raw observation\n')
        self.d=dict(passed=True,mode='hardware',package_sha256='v',calibration_sha256='calibration',script_sha256=s.sha(self.root/'tools/hardware_quest_commissioning.py'),servo_exit_confirmed=True,launch_exit_code=0,
          site=dict(sha256=s.sha(self.root/'site_ubuntu.yaml')),
          capture=dict(path=str(capture),sha256=s.sha(capture)),motion_episodes=[dict(follow_seen=True,maximum_measured_motion_rad=.002,maximum_command_motion_rad=.003)],
          stop_observations=[dict(reason=reason,confirmed_stationary=True,motion_episode_index=0) for reason in ['RELEASED','SOFTWARE_STOP']])
        self.save();(self.root/'artifacts/hardware_readiness.json').write_text(json.dumps(dict(package_sha256='v',teleoperation_ready=False)))
    def tearDown(self):self.modules.stop();self.root_patch.stop();self.tmp.cleanup()
    def save(self):(self.root/'artifacts/quest_commissioning_hardware/result.json').write_text(json.dumps(self.d))
    def test_valid_two_stops_can_be_locally_accepted(self):
        s.accept_local_observation();s.saved_acceptance()
        record=s.read(self.root/'artifacts/hardware_readiness.json');self.assertTrue(record['teleoperation_ready']);self.assertFalse(record['physical_acceptance'])
    def test_acceptance_preserves_immutable_report(self):
        s.accept_local_observation();self.d['passed']=False;self.save();s.saved_acceptance()
    def test_new_acceptance_replaces_legacy_summary_version(self):
        record=self.root/'artifacts/hardware_readiness.json'
        record.write_text(json.dumps(dict(package_sha256='old',workspace='/old/workspace',teleoperation_ready=True)))
        s.accept_local_observation();s.saved_acceptance()
        self.assertEqual(s.read(record)['package_sha256'],'v');self.assertEqual(s.read(record)['workspace'],str(self.root))
        (self.root/'runtime.yaml').unlink()
        self.assertTrue(s.saved_acceptance()['teleoperation_ready'])
    def test_saved_acceptance_rechecks_archived_capture_and_script(self):
        s.accept_local_observation();record=s.saved_acceptance();report=s.read(record['accepted_hardware_report']['path'])
        capture=Path(report['capture']['path']);original=capture.read_bytes();capture.write_text('changed')
        with self.assertRaisesRegex(RuntimeError,'原始记录'):s.saved_acceptance()
        capture.write_bytes(original);(self.root/'tools/hardware_quest_commissioning.py').write_text('changed observer')
        with self.assertRaises(RuntimeError):s.saved_acceptance()
    def test_changed_site_cannot_be_accepted_as_new_profile(self):
        (self.root/'site_ubuntu.yaml').write_text('robot_ip: 1.2.3.4\n')
        with self.assertRaisesRegex(RuntimeError,'站点'):s.accept_local_observation()
    def test_missing_joystick_does_not_accept(self):
        self.d['stop_observations'].pop();self.save()
        with self.assertRaises(RuntimeError):s.accept_local_observation()
    def test_invalidated_stop_does_not_accept(self):
        self.d['stop_observations'][1]['invalidated_after_confirmation']=True;self.save()
        with self.assertRaises(RuntimeError):s.accept_local_observation()
    def test_noise_without_commanded_motion_does_not_accept(self):
        self.d['motion_episodes'][0]['maximum_command_motion_rad']=0;self.save()
        with self.assertRaises(RuntimeError):s.accept_local_observation()
    def test_missing_servo_exit_and_failed_run_do_not_accept(self):
        for key,value in [('servo_exit_confirmed',False),('passed',False),('mode','fake'),('package_sha256','old'),('cleanup_unconfirmed_live_pid',12)]:
            original=self.d.copy();self.d[key]=value;self.save()
            with self.assertRaises(RuntimeError):s.accept_local_observation()
            self.d=original
    def test_changed_raw_capture_does_not_accept(self):
        Path(self.d['capture']['path']).write_text('changed')
        with self.assertRaises(RuntimeError):s.accept_local_observation()
    def test_auto_replan_and_speed_override_are_rejected(self):
        s.accept_local_observation()
        for text in ['auto_replan: true\n','auto_replan: false\njoint_velocity_rad_s: 10\n']:
            (self.root/'runtime.yaml').write_text(text)
            with self.assertRaises(RuntimeError):s.validate_teleop_runtime(self.root/'runtime.yaml')
    def test_live_fake_requires_current_script_and_regression_records(self):
        from check_ros_pass import SIMULATION_REPORTS
        for name,_ in SIMULATION_REPORTS:
            path=self.root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('{}')
        fake=self.root/'artifacts/quest_commissioning_fake/result.json';fake.parent.mkdir(parents=True)
        data=dict(passed=True,package_sha256='v',site=dict(sha256=s.sha(self.root/'site_ubuntu.yaml')),script_sha256=s.sha(self.root/'tools/hardware_quest_commissioning.py'),prerequisite_reports={name:s.sha(self.root/name) for name,_ in SIMULATION_REPORTS})
        fake.write_text(json.dumps(data));self.assertTrue(s.validate_live_fake('v')['passed'])
        (self.root/SIMULATION_REPORTS[0][0]).write_text('{"passed":true}')
        with self.assertRaisesRegex(RuntimeError,'软件回归记录已更新'):s.validate_live_fake('v')
        data['script_sha256']='old';fake.write_text(json.dumps(data))
        with self.assertRaisesRegex(RuntimeError,'旧版本'):s.validate_live_fake('v')
    def test_site_change_is_rejected(self):
        s.accept_local_observation();(self.root/'site_ubuntu.yaml').write_text('robot_ip: 1.2.3.4\n')
        with self.assertRaises(RuntimeError):s.validate_teleop_runtime(self.root/'runtime.yaml')
    def test_current_runtime_valid_without_development_acceptance(self):
        with patch.object(s,'saved_acceptance',side_effect=AssertionError('daily start read development records')):
            s.validate_teleop_runtime(self.root/'runtime.yaml')
    def test_changed_calibration_requires_new_hardware_evidence(self):
        self.d['calibration_sha256']='previous';self.save()
        with self.assertRaisesRegex(RuntimeError,'负载或 TCP'):s.accept_local_observation()

class DailyStartup(unittest.TestCase):
    def test_fake_start_does_not_depend_on_hardware_regression_gate(self):
        session=object.__new__(s.Session);session.args=SimpleNamespace(task='teleop',hardware=False);session.check_cancel=MagicMock();session.input_ready=MagicMock();session.run_child=MagicMock()
        with tempfile.TemporaryDirectory(dir=s.ROOT/'.tmp') as tmp,patch.object(s,'ROOT',Path(tmp)),patch('check_ros_pass.validate_reports',side_effect=RuntimeError('old digest')) as gate,patch.object(s,'assert_domain_empty'):
            (Path(tmp)/'.cache/operator').mkdir(parents=True);(Path(tmp)/'site_ubuntu.yaml').write_text('auto_replan: false\n')
            session.execute();gate.assert_not_called();session.input_ready.assert_called_once();self.assertIn('mode:=fake',session.run_child.call_args.args[0])
    def test_readiness_reuses_saved_acceptance_without_runtime_cache(self):
        import operator_readiness as readiness
        with patch('check_ros_pass.validate_reports'),patch('hardware_current_preflight.validate'),patch.object(readiness,'validate_live_fake',side_effect=RuntimeError('diagnostic changed')),patch.object(readiness,'acceptance_report',side_effect=RuntimeError('diagnostic replaced')),patch.object(readiness,'read',return_value={}),patch.object(readiness,'saved_acceptance',return_value={'teleoperation_ready':True}):
            state=readiness.probe()
            for key in ('fake','hardware'):
                self.assertTrue(state[key]['ok']);self.assertTrue(state[key]['reused']);self.assertFalse(state[key]['latest_check']['ok'])
            self.assertTrue(state['accepted']['ok'])
        with patch('check_ros_pass.validate_reports'),patch('hardware_current_preflight.validate'),patch.object(readiness,'validate_live_fake',side_effect=RuntimeError('diagnostic changed')),patch.object(readiness,'acceptance_report',side_effect=RuntimeError('diagnostic replaced')),patch.object(readiness,'read',return_value={}),patch.object(readiness,'saved_acceptance',side_effect=RuntimeError('old source')):
            state=readiness.probe();self.assertFalse(state['accepted']['ok']);self.assertFalse(state['fake']['ok']);self.assertFalse(state['hardware']['ok'])
    def test_daily_hardware_start_checks_current_runtime_and_input_without_dev_reports(self):
        session=object.__new__(s.Session);session.args=SimpleNamespace(task='teleop',hardware=True)
        session.check_cancel=MagicMock();session.input_ready=MagicMock();session.run_child=MagicMock()
        with tempfile.TemporaryDirectory(dir=s.ROOT/'.tmp') as tmp,patch.object(s,'ROOT',Path(tmp)):
            root=Path(tmp);(root/'.cache/operator').mkdir(parents=True);(root/'src/v3_moveit/config').mkdir(parents=True)
            (root/'site_ubuntu.yaml').write_text('robot_ip: 192.168.110.224\n')
            (root/'src/v3_moveit/config/runtime.yaml').write_text('auto_replan: true\nrobot_ip: 192.168.110.224\n')
            with patch('hardware_hold_check.validate_site') as route,patch.object(s,'assert_domain_empty') as domain,patch('resource.getrlimit',return_value=(50,50)),patch.object(session,'prepare',side_effect=AssertionError('commissioning')),patch('check_ros_pass.validate_reports',side_effect=AssertionError('dev reports')),patch.object(s,'saved_acceptance',side_effect=AssertionError('acceptance')):
                session.execute();route.assert_called_with(root);domain.assert_called_once();session.input_ready.assert_called_once()
                argv=session.run_child.call_args.args[0];self.assertIn('mode:=hardware',argv);self.assertIn('payload_config:='+str(root/'src/v3_moveit/config/payload.json'),argv)
                self.assertFalse((root/'artifacts').exists())
                session.run_child.reset_mock();route.side_effect=RuntimeError('route changed')
                with self.assertRaisesRegex(RuntimeError,'route changed'):session.execute()
                session.run_child.assert_not_called()

class Desktop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ['QT_QPA_PLATFORM']='offscreen'
        from PyQt5.QtWidgets import QApplication
        cls.app=QApplication.instance() or QApplication([])
    def test_opening_is_fake_and_never_launches_control(self):
        from operator_panel import Panel
        with patch('operator_panel.subprocess.Popen') as launch:
            panel=Panel(preview=True);self.assertEqual(panel.mode.currentIndex(),0);self.assertIsNone(panel.job);launch.assert_not_called();panel.close()
    def test_historical_acceptance_flag_does_not_display_current_clearance(self):
        from operator_panel import Panel
        panel=Panel(preview=True)
        try:
            with patch.object(panel,'request_access_probe'):
                panel.set_page(2)
                panel.access_state={'loaded':True,'accepted':{'ok':False,'detail':'旧模型或原目录的验收'}}
                panel.refresh();self.assertIn('暂无',panel.check_label.text())
                panel.access_state['accepted']['ok']=True
                panel.refresh();self.assertIn('开发验收已保存',panel.check_label.text())
        finally:panel.close()
    def test_saved_theme_matches_scroll_content_on_first_show(self):
        from operator_panel import Panel
        from operator_settings import Preferences
        from PyQt5 import QtGui
        for value,background in [(0,'#1d1f23'),(100,'#ffffff')]:
            prefs=Preferences();prefs.data['brightness']=value
            with patch('operator_studio.Preferences',return_value=prefs):panel=Panel(preview=True)
            panel.show();self.app.processEvents()
            card=panel.scene.parentWidget()
            self.assertEqual(card.palette().color(QtGui.QPalette.Window).name(),background)
            self.assertEqual(card.grab().toImage().pixelColor(50,8).name(),background)
            panel.close()
    def test_real_mode_needs_local_readiness_and_rt(self):
        from operator_panel import Panel
        panel=Panel(preview=True)
        with patch('operator_panel.resource.getrlimit',return_value=(0,0)):
            panel.mode.setCurrentIndex(1);self.assertFalse(panel.test.isEnabled());self.assertTrue(panel.start.isEnabled());self.assertEqual(panel.start.text(),'启动真机遥操作')
        panel.close()
    def test_stop_is_sigint_not_forced_kill(self):
        from operator_panel import Panel
        panel=Panel(preview=True);child=MagicMock();child.poll.return_value=None;panel.job=child;panel.stop_session();child.send_signal.assert_called_once_with(signal.SIGINT);child.kill.assert_not_called();child.terminate.assert_not_called();panel.job=None;panel.close()

    def test_navigation_keeps_stop_and_mode_available(self):
        from operator_panel import Panel
        panel=Panel(preview=True);panel.show()
        for index in range(4):
            panel.set_page(index);self.app.processEvents();self.assertTrue(panel.stop.isVisible());self.assertTrue(panel.mode.isVisible())
        panel.close()
    def test_stale_feedback_clears_values(self):
        from operator_panel import Panel
        panel=Panel(preview=True);panel.job=MagicMock();panel.job_started_at=time.monotonic()-1;panel.ingest_telemetry(dict(position=[.1]*6,received_monotonic=time.monotonic(),mode='fake'))
        self.assertEqual(panel.joint_values[0].text(),'5.7');panel.feedback_at-=3;panel.tick_view();self.assertEqual(panel.joint_values[0].text(),'—');panel.job=None;panel.close()
    def test_nonfinite_feedback_is_not_plotted(self):
        from operator_panel import Panel
        panel=Panel(preview=True);panel.job=MagicMock();panel.job_started_at=time.monotonic()-1;panel.ingest_telemetry(dict(position=[float('nan')]*6,received_monotonic=time.monotonic(),mode='fake'));self.assertFalse(panel.plot.samples);self.assertFalse(panel.feedback_at);panel.job=None;panel.close()
    def test_start_locks_mode_and_stop_remains_enabled(self):
        from operator_panel import Panel
        panel=Panel(preview=True);panel.job=MagicMock();panel.refresh()
        self.assertFalse(panel.mode.isEnabled());self.assertFalse(panel.start.isEnabled());self.assertTrue(panel.stop.isEnabled());panel.job=None;panel.close()

class Access(unittest.TestCase):
    setUpClass=classmethod(Desktop.setUpClass.__func__)
    def panel(self,**flags):
        from operator_panel import Panel
        p=Panel(preview=True);p.access_state=dict(loaded=True,**{k:dict(ok=flags.get(k,True),detail=k) for k in ('software','fake','hardware','accepted')});return p
    def test_start_requests_only_rt_after_site_confirmation(self):
        p=self.panel(accepted=False,hardware=False)
        with patch('operator_panel.resource.getrlimit',return_value=(0,0)),patch.object(p,'authorize') as auth,patch.object(p,'launch') as launch:
            p.mode.setCurrentIndex(1);p.primary_start();auth.assert_not_called()
            p.site.setChecked(True);p.primary_start();self.assertEqual(p.pages.currentIndex(),0);launch.assert_not_called();auth.assert_called_once()
        p.close()
    def test_confirmed_start_resumes_once_after_rt_permission(self):
        p=self.panel(software=False,accepted=False);p.mode.setCurrentIndex(1);p.site.setChecked(True)
        with patch('resource.getrlimit',return_value=(0,0)),patch.object(p,'authorize') as auth:
            p.start.click();auth.assert_called_once()
        p.auth=MagicMock(returncode=0)
        with patch('resource.getrlimit',return_value=(50,50)),patch.object(p,'launch') as launch:
            p.tick();p.tick();launch.assert_called_once_with('teleop')
        p.close()
    def test_rt_permission_alone_or_revoked_confirmation_never_starts(self):
        for action in ('permission_only','uncheck','stop','denied'):
            with self.subTest(action=action):
                p=self.panel();p.mode.setCurrentIndex(1);p.site.setChecked(True)
                if action!='permission_only':
                    with patch('resource.getrlimit',return_value=(0,0)),patch.object(p,'authorize'):
                        p.start.click()
                p.auth=MagicMock(returncode=1 if action=='denied' else 0)
                if action=='uncheck':p.site.setChecked(False);p.site.setChecked(True)
                if action=='stop':p.stop_session()
                with patch('resource.getrlimit',return_value=(50,50)),patch.object(p,'launch') as launch:
                    p.tick();launch.assert_not_called()
                p.close()
    def test_fake_step_explicitly_selects_fake_even_from_hardware(self):
        p=self.panel(fake=False,hardware=False,accepted=False);p.mode.setCurrentIndex(1)
        with patch.object(p,'launch') as launch:p.advance_access();launch.assert_called_once_with('commission');self.assertEqual(p.mode.currentIndex(),0);self.assertEqual(p.pages.currentIndex(),0)
        p.close()
    def test_developer_checks_do_not_hijack_the_next_start(self):
        p=self.panel(fake=False,hardware=False,accepted=False)
        with patch.object(p,'launch') as launch:
            p.advance_access();launch.assert_called_once_with('commission');p.session_finished(1)
            p.access_cancel.click();self.assertEqual(p.pages.currentIndex(),0)
            p.access_state={'loaded':False};p.mode.setCurrentIndex(1);p.site.setChecked(True)
            with patch('resource.getrlimit',return_value=(50,50)),patch.object(p,'advance_access',side_effect=AssertionError('redirected')):
                launch.reset_mock();p.start.click();launch.assert_called_once_with('teleop');self.assertEqual(p.pages.currentIndex(),0)
        p.close()
    def test_instructions_survive_periodic_state_but_not_errors(self):
        p=self.panel();p.consume_event(dict(message='现在松 Grip',instruction='现在松 Grip'));p.consume_event(dict(message='运行状态 FOLLOW',control_mode='FOLLOW'))
        self.assertEqual(p.status.text(),'现在松 Grip');p.consume_event(dict(message='反馈超时',error=True));self.assertEqual(p.status.text(),'反馈超时');p.close()
    def test_observer_failure_is_presented_in_chinese(self):
        text=json.dumps(dict(passed=False,failure="Production/observer fault: ['Grip held before released-input and stationary READY checkpoint']"))
        self.assertIn('头显显示就绪',s.failure_detail(text));self.assertIn('末端移动超过',s.failure_detail(json.dumps(dict(passed=False,failure='Observed TCP excursion exceeded commissioning exit threshold'))))
    def test_site_confirmation_is_not_filled_automatically(self):
        p=self.panel(hardware=False,accepted=False);p.mode.setCurrentIndex(1)
        with patch('operator_panel.resource.getrlimit',return_value=(50,50)),patch.object(p,'launch') as launch:
            self.assertEqual(p.access_action()[0],'site');p.advance_access();self.assertFalse(p.site.isChecked());launch.assert_not_called()
            p.site.setChecked(True);p.advance_access();launch.assert_called_once_with('commission');self.assertEqual(p.mode.currentIndex(),1)
        p.close()
    def test_result_confirmation_requires_user_and_valid_hardware(self):
        p=self.panel(accepted=False);p.mode.setCurrentIndex(1);p.site.setChecked(True)
        with patch('operator_panel.resource.getrlimit',return_value=(50,50)),patch.object(p,'accept_result') as accept:
            p.advance_access();accept.assert_not_called();self.assertFalse(p.observed.isChecked())
            p.observed.setChecked(True);p.advance_access();accept.assert_called_once()
        p.close()
    def test_hand_feedback_does_not_block_ready_arm(self):
        p=self.panel();p.mode.setCurrentIndex(1);p.site.setChecked(True);p.media_states={'hand':{'ok':False}}
        with patch('operator_panel.resource.getrlimit',return_value=(50,50)),patch.object(p,'launch') as launch:
            p.refresh();self.assertEqual(p.access_action()[0],'ready');p.primary_start();launch.assert_called_once_with('teleop')
        p.close()
    def test_saved_acceptance_survives_new_diagnostic_runs_and_panel_reopen(self):
        for _ in range(2):
            p=self.panel(fake=False,hardware=False);p.mode.setCurrentIndex(1);p.site.setChecked(True)
            with patch('operator_panel.resource.getrlimit',return_value=(50,50)),patch.object(p,'launch') as launch:
                p.refresh();self.assertEqual(p.start.text(),'启动真机遥操作');p.primary_start();launch.assert_called_once_with('teleop')
            p.site.setChecked(False)
            with patch('operator_panel.resource.getrlimit',return_value=(50,50)),patch.object(p,'launch') as launch:
                p.refresh();self.assertEqual(p.start.text(),'启动真机遥操作');p.primary_start();self.assertEqual(p.pages.currentIndex(),0);launch.assert_not_called();self.assertFalse(p.site.isChecked())
            with patch('operator_panel.resource.getrlimit',return_value=(0,0)),patch.object(p,'authorize') as auth:
                p.site.setChecked(True);p.primary_start();auth.assert_called_once();self.assertEqual(p.pages.currentIndex(),0)
            p.close()
    def test_save_immediately_refreshes_start_and_discards_older_probe(self):
        p=self.panel(accepted=False);p.mode.setCurrentIndex(1);p.site.setChecked(True);p.observed.setChecked(True);p.set_page(2)
        with patch('operator_panel.resource.getrlimit',return_value=(50,50)),patch.object(s,'accept_local_observation') as save,patch.object(p,'stop_access_probe') as stop,patch.object(p,'launch') as launch:
            p.accept_result();save.assert_called_once();stop.assert_called_once();launch.assert_not_called()
            self.assertEqual(p.pages.currentIndex(),2);self.assertEqual(p.start.text(),'启动真机遥操作');self.assertEqual(p.access_action()[0],'ready')
        p.close()
    def test_failure_and_cleanup_survive_session_completion(self):
        p=self.panel();p.session_error='TCP 不匹配';p.session_cleanup_warning=True;p.session_finished(1)
        self.assertIn('TCP 不匹配',p.status.text());self.assertIn('退出异常',p.status.text());p.close()
    def test_authorization_cannot_launch_in_parallel(self):
        p=self.panel();p.auth=MagicMock()
        with patch('operator_panel.subprocess.Popen') as launch:
            p.authorize();p.launch('teleop');launch.assert_not_called();self.assertEqual(p.access_action()[0],'busy')
        p.auth=None;p.close()

class Studio(unittest.TestCase):
    setUpClass=classmethod(Desktop.setUpClass.__func__)
    def test_display_rate_selection_is_bounded_and_persisted(self):
        from operator_settings import Preferences,RATES,validate,defaults
        with tempfile.TemporaryDirectory(dir=s.ROOT/'.tmp') as d, patch('operator_settings.RUNTIME',Path(d)/'runtime.json'):
            pref=Preferences(Path(d)/'prefs.json');pref.data['active']=2;pref.profile['enabled']['hand']=False;pref.data['scope_hz']=60;pref.data['brightness']=0;pref.save()
            restored=Preferences(pref.path);self.assertEqual(restored.data,pref.data);self.assertTrue(restored.data['profiles'][0]['enabled']['hand'])
            for bad in (0,61,1000):
                data=defaults();data['scope_hz']=bad
                with self.assertRaises(ValueError):validate(data)
    def test_profiles_reject_control_ros_domains(self):
        from operator_settings import validate,defaults
        for domain in (89,90,91):
            d=defaults();d['profiles'][0]['sensor_domain']=domain
            with self.assertRaises(ValueError):validate(d)
    def test_old_wrong_mode_future_feedback_never_updates_model(self):
        from operator_panel import Panel
        panel=Panel(preview=True);panel.job=MagicMock();now=time.monotonic();panel.job_started_at=now-1
        for stamp,mode in [(now-2,'fake'),(now,'hardware'),(now+1,'fake')]:panel.ingest_telemetry(dict(position=[.2]*6,received_monotonic=stamp,mode=mode),now)
        self.assertFalse(panel.plot.samples);self.assertFalse(panel.feedback_at);panel.job=None;panel.close()
    def test_one_hz_plot_has_live_heartbeat_and_missing_velocity_is_explicit(self):
        from operator_panel import Panel
        panel=Panel(preview=True);panel.job=MagicMock();now=time.monotonic();panel.job_started_at=now-1;panel.prefs.data['scope_hz']=1
        for i in range(10):panel.ingest_telemetry(dict(position=[.2]*6,received_monotonic=now+i*.05,mode='fake'),now+i*.05)
        self.assertEqual(len(panel.plot.samples),1);self.assertGreater(panel.feedback_at,now)
        panel.quantity.setCurrentIndex(1);panel.ingest_telemetry(dict(position=[.2]*6,received_monotonic=now+.6,mode='fake'),now+.6)
        self.assertEqual(panel.joint_values[0].text(),'—');self.assertIn('未提供',panel.feedback_label.text());panel.job=None;panel.close()
    def test_selection_can_disable_teleop_without_affecting_stop(self):
        from operator_panel import Panel
        panel=Panel(preview=True);panel.device_boxes['arm'].setChecked(False);self.assertFalse(panel.start.isEnabled());panel.job=MagicMock();panel.refresh();self.assertTrue(panel.stop.isEnabled());panel.job=None;panel.close()

class CanvasGeometry(unittest.TestCase):
    setUpClass=classmethod(Desktop.setUpClass.__func__)
    def test_render_budget_preserves_aspect_at_wide_tall_and_fractional_dpi(self):
        from operator_instruments import render_size
        for w,h,dpr in [(1970,611,1),(3480,1200,1),(680,1600,1),(1070,510,1.5),(4520,420,1)]:
            rw,rh=render_size(w,h,dpr);self.assertLessEqual(rw,1600);self.assertLessEqual(rh,900)
            self.assertLess(abs(rw/rh-w/h)/(w/h),.004)
    def test_pending_frame_is_not_stretched_and_mouse_maps_to_image(self):
        from operator_instruments import SceneWidget
        from PyQt5 import QtCore,QtGui
        import numpy as np
        scene=SceneWidget();scene.resize(2200,600);scene.image=QtGui.QImage(1600,900,QtGui.QImage.Format_RGBA8888);scene.image.fill(QtGui.QColor('black'))
        p=QtGui.QPainter(scene.image);p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor('#ff00ff'));p.drawEllipse(700,350,200,200);p.end();scene.show();self.app.processEvents()
        image=scene.grab().toImage().convertToFormat(QtGui.QImage.Format_RGBA8888);bits=image.bits();bits.setsize(image.byteCount());a=np.frombuffer(bits,np.uint8).reshape(image.height(),image.bytesPerLine()//4,4)[:,:image.width()];y,x=np.where((a[:,:,0]>230)&(a[:,:,1]<20)&(a[:,:,2]>230))
        self.assertLess(abs((x.max()-x.min())/(y.max()-y.min())-1),.02)
        self.assertEqual(scene.image_point(scene.image_rect().center()),(800,450));scene.close()
    def test_camera_focus_keeps_stop_visible_and_esc_restores_view(self):
        from operator_panel import Panel
        from PyQt5 import QtCore,QtTest
        panel=Panel(preview=True);panel.show();self.app.processEvents();tile=panel.camera_tiles['c922'];panel.scene.send_input=MagicMock();QtTest.QTest.mouseClick(tile,QtCore.Qt.LeftButton);self.app.processEvents()
        self.assertTrue(panel.scene.camera_focus.isVisible());self.assertTrue(panel.stop.isVisible());self.assertIsNone(panel.job);panel.scene.send_input.assert_not_called()
        QtTest.QTest.keyClick(panel.scene.camera_focus,QtCore.Qt.Key_Escape);self.assertFalse(panel.scene.camera_focus.isVisible());panel.close()

class Media(unittest.TestCase):
    def test_stale_or_future_stream_does_not_claim_online(self):
        from operator_media import current_state
        for stamp in (0,11):
            d=current_state(dict(ok=True,updated=stamp,detail='画面在线'),10)
            self.assertFalse(d['ok']);self.assertNotIn('在线',d['detail'])
        self.assertTrue(current_state(dict(ok=True,updated=9.9),10)['ok'])

    def test_rgb_padding_and_short_frame(self):
        from operator_media import image_from_ros
        msg=SimpleNamespace(encoding='rgb8',width=1,height=2,step=4,is_bigendian=False,data=bytes([255,0,0,9,0,255,0,9]))
        self.assertEqual(image_from_ros(msg).tolist(),[[[0,0,255]],[[0,255,0]]]);msg.data=b'bad'
        with self.assertRaises(ValueError):image_from_ros(msg)
    def test_depth_endian_units_and_invalid_pixels(self):
        import numpy as np
        from operator_media import image_from_ros,depth_preview
        msg=SimpleNamespace(encoding='16UC1',width=2,height=1,step=4,is_bigendian=True,data=bytes([3,232,0,0]))
        a=image_from_ros(msg);self.assertEqual(a.tolist(),[[1000,0]]);self.assertTrue((depth_preview(a,'mm')[0,1]==0).all())
        values=np.array([[1.,float('nan'),float('inf'),-1,0]])
        self.assertTrue((depth_preview(values)[0,1:]==0).all());self.assertTrue((depth_preview(a,'mm')[0,0]==depth_preview(np.array([[1.]]))[0,0]).all())

if __name__=='__main__':unittest.main()
