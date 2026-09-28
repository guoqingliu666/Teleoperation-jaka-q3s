"""真实Tk控件的离线入口检查；窗口不展示，不启动机器人子进程。"""
import json
from pathlib import Path
import sys
import tkinter as tk
from types import SimpleNamespace
import unittest
from unittest.mock import patch, MagicMock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from quest_endpoint_teleop_gui import EndpointTeleopGui


class DiagnosticGuiTests(unittest.TestCase):
    def setUp(self):
        self.root=tk.Tk()
        self.root.withdraw()
        with patch.object(self.root,"state"):
            self.app=EndpointTeleopGui(self.root)
        self.root.update_idletasks()

    def tearDown(self):
        self.root.destroy()

    def test_diagnostic_button_does_not_change_live_default(self):
        self.assertIn("不连接",str(self.app.trajectory_diagnostic_button["text"]))
        self.assertFalse(self.app.live.get())

    def test_cancel_file_selection_starts_nothing(self):
        with patch("quest_endpoint_teleop_gui.filedialog.askopenfilename",return_value=""), \
                patch("quest_endpoint_teleop_gui.subprocess.run") as run:
            self.app.trajectory_diagnostic()
        run.assert_not_called()

    def test_active_robot_session_does_not_start_diagnostics(self):
        self.app.process=SimpleNamespace(poll=lambda:None)
        with patch("quest_endpoint_teleop_gui.messagebox.showinfo"), \
                patch("quest_endpoint_teleop_gui.filedialog.askopenfilename") as select:
            self.app.trajectory_diagnostic()
        select.assert_not_called()

    def test_offline_report_does_not_clear_stop_fault_or_request_live(self):
        self.app.stop_fault_latched=True
        self.app.stop_confirmed=False
        def immediate_thread(*,target,**kwargs):
            return SimpleNamespace(start=target)
        with patch("quest_endpoint_teleop_gui.filedialog.askopenfilename",return_value="D:/example.jsonl"), \
                patch("quest_endpoint_teleop_gui.threading.Thread",side_effect=immediate_thread), \
                patch("quest_endpoint_teleop_gui.subprocess.run",return_value=SimpleNamespace(
                    stdout=json.dumps({"reference_only":True}),stderr="",returncode=0)) as run:
            self.app.trajectory_diagnostic()
        args,kwargs=run.call_args
        self.assertIn("--replay",args[0])
        self.assertNotIn("--live",args[0])
        self.assertTrue(kwargs["cwd"].startswith("D:"))
        self.assertTrue(self.app.stop_fault_latched)
        self.assertFalse(self.app.stop_confirmed)
        self.assertIsNone(self.app.process)

    def start_readonly(self):
        child=MagicMock()
        child.poll.return_value=None
        with patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=child) as popen, \
                patch("quest_endpoint_teleop_gui.threading.Thread"):
            self.app.trajectory_readonly_button.invoke()
        return child,popen

    def test_readonly_button_ignores_live_checkbox_and_uses_safe_entry(self):
        self.app.live.set(True)
        child,popen=self.start_readonly()
        args=popen.call_args.args[0]
        self.assertIn("--live-readonly",args)
        self.assertIn("--ui-control",args)
        self.assertNotIn("--live",args)
        self.assertEqual(args[args.index("--seconds")+1],"120")
        self.assertTrue(self.app.readonly_test_active)
        self.assertEqual(str(self.app.start_button["state"]),"disabled")

    def test_dynamic_settings_pass_to_readonly_and_invalid_settings_block_launch(self):
        self.app.check_mm.set("8")
        self.app.check_deg.set("1.5")
        _,popen=self.start_readonly()
        args=popen.call_args.args[0]
        self.assertEqual(args[args.index("--max-check-mm")+1],"8.0")
        self.assertEqual(args[args.index("--max-check-deg")+1],"1.5")
        self.assertEqual(args[args.index("--sampling")+1],"adaptive")
        self.app.readonly_event(dict(state="process_exit",code=0))
        self.app.check_deg.set("nan")
        with patch("quest_endpoint_teleop_gui.messagebox.showerror"), \
                patch("quest_endpoint_teleop_gui.subprocess.Popen") as start:
            self.app.start_trajectory_readonly()
        start.assert_not_called()

    def test_dynamic_live_has_separate_explicit_command_and_keeps_same_policy(self):
        child=MagicMock(); child.poll.return_value=None
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
                patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=child) as popen, \
                patch("quest_endpoint_teleop_gui.threading.Thread"):
            self.app.adaptive_live_button.invoke()
        args=popen.call_args.args[0]
        self.assertIn("--adaptive-trajectory",args)
        self.assertIn("--production-rolling-pose",args)
        self.assertIn("--live",args)
        self.assertEqual(args[args.index("--max-check-deg")+1],"1.0")

    def test_stop_readonly_is_cooperative_not_kill(self):
        child,_=self.start_readonly()
        self.app.trajectory_readonly_stop.invoke()
        child.stdin.write.assert_called_with("STOP\n")
        child.terminate.assert_not_called()
        child.kill.assert_not_called()
        self.assertIsNone(self.app.stop_confirmed)

    def test_readonly_active_blocks_motion_even_if_child_already_exited(self):
        child,_=self.start_readonly()
        child.poll.return_value=0
        with patch("quest_endpoint_teleop_gui.subprocess.Popen") as popen:
            self.app.start(production_rolling_pose=True)
        popen.assert_not_called()

    def test_readonly_result_preserves_motion_stop_state_and_can_restart(self):
        self.app.stop_confirmed=False
        self.start_readonly()
        self.app.readonly_event(dict(state="trajectory_readonly_finished",outcome="incomplete",
                                     grip_windows=0,candidates=0,blocked_windows=0))
        self.app.readonly_event(dict(state="process_exit",code=2))
        self.assertFalse(self.app.stop_confirmed)
        self.assertFalse(self.app.stop_fault_latched)
        self.assertFalse(self.app.readonly_test_active)
        self.assertIn("未完成",self.app.state.get())
        self.start_readonly()
        self.assertTrue(self.app.readonly_test_active)

    def test_existing_fault_prevents_readonly_and_is_not_cleared(self):
        self.app.stop_fault_latched=True
        with patch("quest_endpoint_teleop_gui.messagebox.showerror"), \
                patch("quest_endpoint_teleop_gui.subprocess.Popen") as popen:
            self.app.start_trajectory_readonly()
        popen.assert_not_called()
        self.assertTrue(self.app.stop_fault_latched)

    def test_display_shows_grip_trigger_and_countdown(self):
        self.app.readonly_event(dict(state="trajectory_input_status",message="握持采样中",
                                     grip=.9,trigger=.1,remaining_s=98))
        self.assertIn("Grip=0.90",self.app.state.get())
        self.assertIn("Trigger=0.10",self.app.state.get())
        self.assertIn("98",self.app.state.get())

    def test_missing_final_report_is_not_success_or_motion_stop_confirmation(self):
        self.start_readonly()
        self.app.readonly_event(dict(state="process_exit",code=1))
        self.assertIn("未收到完整报告",self.app.state.get())
        self.assertIsNone(self.app.stop_confirmed)


if __name__=="__main__": unittest.main()
