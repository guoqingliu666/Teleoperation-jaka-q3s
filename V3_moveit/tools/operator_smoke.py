"""SDK-free GUI -> session -> ROS fake lifecycle check. Never selects hardware."""
import json,os,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if '--worker' in sys.argv:
    from types import SimpleNamespace
    from operator_session import Session
    session=Session(SimpleNamespace(hardware=False,task='teleop'))
    # No operator is needed to verify stationary fake feedback and lifecycle.
    # Only this test worker omits the physical Quest readiness wait.
    session.input_ready=lambda:None
    raise SystemExit(session.run())
os.environ['QT_QPA_PLATFORM']='xcb'
from PyQt5.QtWidgets import QApplication
from operator_panel import Panel
os.environ['V3_SCENE_CAPTURE']='1'
app=QApplication([]);panel=Panel(preview=True);panel.preview=False;panel.prefs.write_runtime();panel.show();panel.scene.start();start=time.monotonic();seen=False;stopped=False;timeout=False
panel.launch_command(['bash','tools/operator_ros.sh',sys.executable,'-B',__file__,'--worker'],'仿真界面启停自检')
child=panel.job
while panel.job is not None:
    app.processEvents();panel.tick();panel.studio_tick();now=time.monotonic()
    if panel.feedback_at and len(panel.plot.samples)>=4 and not seen:
        seen=True;panel.grab().save(str(ROOT/'artifacts/operator_live_fake.png'))
    if not stopped and ((seen and now-start>12) or now-start>45):
        timeout=not seen;panel.stop_session();stopped=True
    if now-start>90:
        raise SystemExit('Fake session did not exit after SIGINT; inspect retained process. No forced kill issued.')
    time.sleep(.05)
scene_report=json.loads((ROOT/'artifacts/operator_scene_view.json').read_text())
scene_ok=panel.scene.process.poll() is None and scene_report['pid']==panel.scene.process.pid and len(scene_report['joint_radians'])==6 and scene_report['offscreen'] and not panel.scene.image.isNull() and time.monotonic()-panel.scene.frame_at<1
panel.close();result=dict(passed=seen and stopped and not timeout and child.returncode==0 and scene_ok,embedded_qt_canvas_verified=scene_ok,mode='fake',hardware_selected=False,feedback_seen=seen,stop_requested=stopped,worker_exit_code=child.returncode,physical_acceptance=False)
result['cleanup_warning_seen']='process has died' in (ROOT/'.cache/operator/panel_session.log').read_text()
(ROOT/'artifacts/operator_smoke.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
