"""Capture real Qt layouts with the actual embedded Qt canvas. No robot session."""
import json,os,time,sys
from pathlib import Path
os.environ['QT_QPA_PLATFORM']='xcb';os.environ['V3_SCENE_CAPTURE']='1'
from PyQt5 import QtWidgets as W,QtCore,QtGui
from operator_panel import Panel,ROOT
app=W.QApplication([]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10));panel=Panel(preview=True);panel.setFixedSize(1640,1080);panel.show();panel.prefs.write_runtime();panel.scene.start();start=time.monotonic()
if '--no-sensors' not in sys.argv:panel.start_monitor()
def pump(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:app.processEvents();panel.studio_tick();time.sleep(.02)
def capture(name,scene=True):
    pump(.4);pix=panel.grab()
    if scene:
        assert not panel.scene.image.isNull() and time.monotonic()-panel.scene.frame_at<1,'Embedded scene not live'
    pix.save(str(ROOT/'artifacts'/name))
try:
    pump(5)
    before=panel.scene.distance;point=QtCore.QPointF(350,160)
    wheel=QtGui.QWheelEvent(point,point,QtCore.QPoint(),QtCore.QPoint(0,120),QtCore.Qt.NoButton,QtCore.Qt.NoModifier,QtCore.Qt.NoScrollPhase,False)
    app.sendEvent(panel.scene,wheel);pump(1);after=panel.scene.distance
    assert after<before,(before,after)
    scene_report=json.loads((ROOT/'artifacts/operator_scene_view.json').read_text());assert scene_report['offscreen']
    before_drag=dict(scene_report)
    for kind,point,buttons in [(QtCore.QEvent.MouseButtonPress,QtCore.QPointF(350,160),QtCore.Qt.LeftButton),(QtCore.QEvent.MouseMove,QtCore.QPointF(400,175),QtCore.Qt.LeftButton),(QtCore.QEvent.MouseButtonRelease,QtCore.QPointF(400,175),QtCore.Qt.NoButton)]:
        event=QtGui.QMouseEvent(kind,point,QtCore.Qt.LeftButton if kind!=QtCore.QEvent.MouseMove else QtCore.Qt.NoButton,buttons,QtCore.Qt.NoModifier);app.sendEvent(panel.scene,event);pump(.05)
    pump(1.1);after_drag=json.loads((ROOT/'artifacts/operator_scene_view.json').read_text())
    assert abs(after_drag['x']-before_drag['x'])+abs(after_drag['y']-before_drag['y'])>.01,'Orbit drag did not move view'
    for kind,point,buttons in [(QtCore.QEvent.MouseButtonPress,QtCore.QPointF(350,160),QtCore.Qt.MiddleButton),(QtCore.QEvent.MouseMove,QtCore.QPointF(380,180),QtCore.Qt.MiddleButton),(QtCore.QEvent.MouseButtonRelease,QtCore.QPointF(380,180),QtCore.Qt.NoButton)]:
        event=QtGui.QMouseEvent(kind,point,QtCore.Qt.MiddleButton if kind!=QtCore.QEvent.MouseMove else QtCore.Qt.NoButton,buttons,QtCore.Qt.NoModifier);app.sendEvent(panel.scene,event);pump(.05)
    pump(1.1);after_pan=json.loads((ROOT/'artifacts/operator_scene_view.json').read_text())
    assert abs(after_pan['x']-after_drag['x'])+abs(after_pan['y']-after_drag['y'])>.01,'Pan did not move view'
    frame_before=panel.scene.frame_seq;time_before=time.monotonic();pump(1.1);fps=(panel.scene.frame_seq-frame_before)/2/(time.monotonic()-time_before)
    panel.preview=False;panel.reset_scene();panel.preview=True;pump(1)
    assert abs(panel.scene.distance-1.25)<.001,panel.scene.distance
    (ROOT/'artifacts/operator_canvas_check.json').write_text(json.dumps(dict(qt_canvas=True,offscreen=scene_report['offscreen'],zoom_before=before,zoom_after=after,reset_distance=panel.scene.distance,frame=panel.scene.frame_seq,orbit=True,pan=True,observed_render_fps=round(fps,1)),indent=2))
    if '--require-cameras' in sys.argv:assert all(panel.media_states.get(name,{}).get('ok') for name in ('zed_left','zed_depth','c922')),panel.media_states
    panel.brightness.setValue(100);pump(.3);capture('operator_canvas_verified_light.png');panel.brightness.setValue(0);capture('operator_canvas_verified_dark.png')
    panel.set_page(1);capture('operator_devices_verified_dark.png',False);panel.brightness.setValue(100);capture('operator_devices_verified_light.png',False)
finally:panel.close();app.processEvents()
