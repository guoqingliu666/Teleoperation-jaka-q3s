"""真实Tk窗口的离线验收：不启动SDK子进程、不占用机器人连接。"""
import json
import io
from pathlib import Path
import tkinter as tk
import time
from types import SimpleNamespace
from unittest.mock import patch

from quest_endpoint_teleop_gui import EndpointTeleopGui


def main():
    root = tk.Tk()
    try:
        app = EndpointTeleopGui(root)
        root.update()
        assert not app.live.get(), "默认必须只读"
        assert "5mm" in str(app.acceptance_button["text"]), "必须提供独立单段验收入口"
        assert "5cm" in str(app.five_segment_button["text"]), "必须提供有限行程的第三关入口"
        assert "10cm" in str(app.ten_segment_button["text"]), "必须提供有限行程的第四关入口"
        assert "20cm" in str(app.twenty_segment_button["text"]), "必须提供有限行程的第五关入口"
        assert "姿态只读影子" in str(app.rotation_shadow_button["text"]), "必须提供零运动姿态验收入口"
        assert "1°姿态真机验收" in str(app.orientation_acceptance_button["text"]), "必须提供独立小角度真机入口"
        assert "3°姿态验收" in str(app.three_orientation_acceptance_button["text"]), "必须提供三段姿态验收入口"
        assert "10°姿态验收" in str(app.ten_orientation_acceptance_button["text"]), "必须提供十段姿态验收入口"
        assert "连续六维" in str(app.six_dof_button["text"]), "必须提供受限六维入口"
        assert "扩展六维" in str(app.expanded_six_dof_button["text"]), "必须提供扩展六维入口"
        assert "高速正式六维" in str(app.production_six_dof_button["text"]), "必须提供高速正式六维入口"
        assert float(app.radius.get()) == 100 and float(app.speed.get()) == 100
        app.controls[0].set(25)
        root.update()
        assert float(app.controls[1].get()) == 25, "滑杆和输入框必须同步"
        app.controls[1].delete(0, "end")
        app.controls[1].insert(0, "15")
        root.update()
        assert float(app.speed.get()) == 15 and app.controls[0].get() == 15
        app.speed.set(10)
        app.live.set(True)
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as locked:
            app.start()
            locked.assert_called_once()
        app.live.set(False)
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as cancelled:
            app.start(one_segment=True)
            cancelled.assert_called_once()
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as cancelled:
            app.start(five_segment=True)
            cancelled.assert_called_once()
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as cancelled:
            app.start(ten_segment=True)
            cancelled.assert_called_once()
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as cancelled:
            app.start(twenty_segment=True)
            cancelled.assert_called_once()
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as cancelled:
            app.start(three_orientation_acceptance=True)
            cancelled.assert_called_once()
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as cancelled:
            app.start(ten_orientation_acceptance=True)
            cancelled.assert_called_once()
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as cancelled:
            app.start(bounded_six_dof=True)
            cancelled.assert_called_once()
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as cancelled:
            app.start(expanded_six_dof=True)
            cancelled.assert_called_once()
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=False) as cancelled:
            app.start(production_six_dof=True)
            cancelled.assert_called_once()
        assert app.process is None, "未确认现场门槛不得启动真机子进程"
        # 截图显示正式档的默认值，不保留前面控件同步测试写入的临时数值。
        app.speed.set(100); app.radius.set(100); app.acceleration.set(400)
        app.rotation_radius.set(30); app.orientation_speed.set(20)
        root.attributes("-topmost", True)
        root.lift()
        root.update()
        # 等待Windows窗口淡入结束，避免截图混入背后桌面内容。
        root.after(600, root.quit)
        root.mainloop()
        from PIL import ImageGrab
        folder = Path(__file__).resolve().parents[2] / "Validation"
        ImageGrab.grab(bbox=(root.winfo_rootx(), root.winfo_rooty(),
                            root.winfo_rootx()+root.winfo_width(), root.winfo_rooty()+root.winfo_height())).save(
            folder / "连续采样界面_离线.png")
        # finished成功才能恢复启动按钮。
        app.lines.put(json.dumps({"state":"finished", "stop_confirmed":True}))
        app.lines.put(json.dumps({"state":"process_exit", "code":0}))
        app.pump()
        assert not app.stop_fault_latched and str(app.start_button["state"]) == "normal"
        app.lines.put(json.dumps({"state":"acceptance_incomplete","message":"唯一短段未到位"}))
        app.lines.put(json.dumps({"state":"finished","stop_confirmed":True}))
        app.lines.put(json.dumps({"state":"process_exit","code":2}))
        app.pump()
        assert app.stop_fault_latched, "单段异常即使停止已确认，也不得一键重试"
        app.stop_fault_latched = False
        app.session_fault = False
        # 即便进程返回0，缺少停止确认也不准当作停止成功。
        app.stop_confirmed = None
        app.closing = True
        app.lines.put(json.dumps({"state":"process_exit", "code":0}))
        app.pump()
        assert app.stop_fault_latched and not app.closing
        assert str(app.start_button["state"]) == "disabled"
        with patch("quest_endpoint_teleop_gui.messagebox.showerror") as warning:
            app.start()
            warning.assert_called_once()
        app.sdk_wait=(999,"get_actual_joint_position",time.perf_counter()-1)
        with patch.object(app,"send") as send:
            app.pump()
            send.assert_any_call("STOP")
        assert app.stopping and app.sdk_stall_warned
        assert "停止尚未确认" in app.state.get()
        app.stop_fault_latched = False
        app.process = None
        fake_process = SimpleNamespace(poll=lambda:None, stdin=io.StringIO(), stdout=iter(()))
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(one_segment=True)
            args = launch.call_args.args[0]
            assert "--one-segment-acceptance" in args and "--live" in args
            assert "--single-owner-display" in args
            assert args[args.index("--speed-mm-s")+1] == "5"
            assert args[args.index("--radius-mm")+1] == "20"
        app.process = None
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(five_segment=True)
            args = launch.call_args.args[0]
            assert "--five-segment-acceptance" in args and "--live" in args
            assert args[args.index("--speed-mm-s")+1] == "15"
            assert args[args.index("--radius-mm")+1] == "60"
        app.process = None
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(ten_segment=True)
            args = launch.call_args.args[0]
            assert "--ten-segment-acceptance" in args and "--live" in args
            assert args[args.index("--speed-mm-s")+1] == "20"
            assert args[args.index("--radius-mm")+1] == "100"
        app.process = None
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(twenty_segment=True)
            args = launch.call_args.args[0]
            assert "--twenty-segment-acceptance" in args and "--live" in args
            assert args[args.index("--speed-mm-s")+1] == "30"
            assert args[args.index("--radius-mm")+1] == "200"
            assert args[args.index("--acceleration-mm-s2")+1] == "60"
            assert args[args.index("--session-seconds")+1] == "180"
        app.process = None
        app.live.set(True)
        app.speed.set(30); app.radius.set(20); app.acceleration.set(60)
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start()
            args = launch.call_args.args[0]
            assert "--bounded-continuous" in args and "--live" in args
            assert args[args.index("--speed-mm-s")+1] == "30.0"
            assert args[args.index("--radius-mm")+1] == "200.0"
            assert args[args.index("--acceleration-mm-s2")+1] == "60.0"
            assert args[args.index("--session-seconds")+1] == "600"
        app.process = None
        app.speed.set(45); app.radius.set(100); app.acceleration.set(90)
        app.rotation_radius.set(25); app.orientation_speed.set(8)
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(production_six_dof=True)
            args = launch.call_args.args[0]
            assert "--production-six-dof" in args and "--live" in args
            assert args[args.index("--speed-mm-s")+1] == "45.0"
            assert args[args.index("--radius-mm")+1] == "1000.0"
            assert args[args.index("--acceleration-mm-s2")+1] == "90.0"
            assert args[args.index("--rotation-radius-deg")+1] == "25.0"
            assert args[args.index("--orientation-speed-deg-s")+1] == "8.0"
        app.process = None
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(expanded_six_dof=True)
            args = launch.call_args.args[0]
            assert "--expanded-six-dof" in args and "--live" in args
            assert args[args.index("--speed-mm-s")+1] == "50"
            assert args[args.index("--radius-mm")+1] == "500"
            assert args[args.index("--acceleration-mm-s2")+1] == "100"
            assert args[args.index("--session-seconds")+1] == "600"
        app.live.set(False)
        app.process = None
        with patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(rotation_shadow=True)
            args = launch.call_args.args[0]
            assert "--rotation-shadow" in args and "--live" not in args
            assert "--single-owner-display" in args
            assert args[args.index("--session-seconds")+1] == "30"
        app.process = None
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(orientation_acceptance=True)
            args = launch.call_args.args[0]
            assert "--one-degree-orientation-acceptance" in args and "--live" in args
            assert args[args.index("--speed-mm-s")+1] == "5"
            assert args[args.index("--session-seconds")+1] == "30"
        app.process = None
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(three_orientation_acceptance=True)
            args = launch.call_args.args[0]
            assert "--three-degree-orientation-acceptance" in args and "--live" in args
            assert args[args.index("--speed-mm-s")+1] == "5"
            assert args[args.index("--session-seconds")+1] == "45"
        app.process = None
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(ten_orientation_acceptance=True)
            args = launch.call_args.args[0]
            assert "--ten-degree-orientation-acceptance" in args and "--live" in args
            assert args[args.index("--session-seconds")+1] == "90"
        app.process = None
        app.speed.set(30); app.radius.set(20); app.acceleration.set(60)
        with patch("quest_endpoint_teleop_gui.messagebox.askyesno",return_value=True), \
             patch("quest_endpoint_teleop_gui.subprocess.Popen",return_value=fake_process) as launch, \
             patch("quest_endpoint_teleop_gui.threading.Thread"):
            app.start(bounded_six_dof=True)
            args = launch.call_args.args[0]
            assert "--bounded-six-dof" in args and "--live" in args
            assert args[args.index("--speed-mm-s")+1] == "30"
            assert args[args.index("--radius-mm")+1] == "200"
            assert args[args.index("--session-seconds")+1] == "600"
        app.process = None
        print("GUI_OFFLINE_PASS: default readonly, controls synced, restart latch, SDK stall warning/revoke")
    finally:
        root.destroy()


if __name__ == "__main__":
    main()
