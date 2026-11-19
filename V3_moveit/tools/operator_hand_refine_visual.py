"""Real Qt/OpenGL preview and synthetic display benchmark; zero hardware owners."""
import json,math,os,time
from pathlib import Path
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','xcb')
from PyQt5 import QtCore,QtGui,QtWidgets as W,QtTest
from operator_panel import Panel,ROOT
OUT=ROOT/'artifacts/operator_hand_refine_20261115';OUT.mkdir(exist_ok=True)
app=W.QApplication([]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10));panel=Panel(preview=True);panel.setWindowTitle('V3 · 离线界面验证（无硬件连接）');panel.set_page(4);panel.show()
def pump(seconds=.3):
    until=time.monotonic()+seconds
    while time.monotonic()<until:app.processEvents();time.sleep(.004)
def capture(name):
    pump();panel.grab().save(str(OUT/name))
try:
    pump(1);panel.brightness.setValue(100);pump();assert panel.hand_model.gl is not None
    capture('grasp_light.png');model=panel.hand_model
    model.gl.grabFramebuffer().save(str(OUT/'vendor_model_open.png'))
    model.set_values([6000,6000,8000,5500,9000,9000]);capture('grasp_pose.png');model.gl.grabFramebuffer().save(str(OUT/'vendor_model_pose.png'))
    # Actual GL surface selection, followed by a Qt mouse drag. Preview must sync sliders.
    model.set_values([0]*6);pump();gl=model.gl;gl.makeCurrent();picked=None
    for item in model.items.values():
        if item.axis_index not in (2,3,4,5):continue
        point=model.screen_point(item.part_name,item.local_center,model.values);d=gl.devicePixelRatioF()
        hits=gl.itemsAt((point[0]*d,point[1]*d,3*d,3*d))
        if hits and getattr(hits[0],'axis_index',None) in (2,3,4,5):picked=(hits[0],QtCore.QPoint(round(point[0]),round(point[1])));break
    assert picked,'No selectable finger surface'
    item,point=picked;QtTest.QTest.mousePress(gl,QtCore.Qt.LeftButton,pos=point);assert model.dragging is not None
    axis=model.dragging['axis'];t=model.dragging['tangent'];end=point+QtCore.QPoint(round(t[0]*2500),round(t[1]*2500))
    app.sendEvent(gl,QtGui.QMouseEvent(QtCore.QEvent.MouseMove,QtCore.QPointF(end),QtCore.Qt.NoButton,QtCore.Qt.LeftButton,QtCore.Qt.NoModifier));QtTest.QTest.mouseRelease(gl,QtCore.Qt.LeftButton,pos=end);pump()
    target=panel.manual_targets[axis][0].value();assert target>100,(axis,target)
    assert panel.manual_targets[axis][1].value()==target;capture('grasp_drag.png')
    panel.brightness.click();pump();capture('grasp_dark.png')
    panel.set_page(0);panel.monitor_buttons[1].click();pump();capture('tactile_dark_offline.png')
    coords=[[[.5,.18],[.41,.5],[.41,.62],[.58,.62],[.58,.5]],[[.5,.08],[.38,.3],[.38,.21],[.56,.21],[.56,.3],[.36,.82],[.36,.69],[.57,.69],[.57,.82]]]
    durations=[];begin=time.monotonic();seq=0
    while time.monotonic()-begin<3:
        now=time.monotonic();fingers=[]
        for i in range(5):
            xy=coords[0 if i in (0,4) else 1];nf=.6+.5*math.sin((now-begin)*4+i)
            fingers.append(dict(normal_n=[nf],tangent_n=[.2],direction_deg=[45+i*30],proximity=[],pressure_unit='normalized',pressure_xy=xy,pressure=[max(0,.8*math.sin((now-begin)*3-i-j*.4)) for j in range(len(xy))]))
        state=dict(sequence=seq,updated=now,fingers=fingers);seq+=1;before=time.perf_counter();panel.grasp_tactile.set_state(state);app.processEvents();durations.append(time.perf_counter()-before);time.sleep(.004)
    elapsed=time.monotonic()-begin;display_hz=sum(time.monotonic()-t<1 for t in panel.grasp_tactile.wave.draws)
    panel.grasp_tactile.caption.setText('离线显示验证 · 合成通道与力数据，不代表实机');capture('tactile_synthetic_dark.png')
    panel.brightness.click();pump();capture('tactile_synthetic_light.png');panel.grasp_tactile.grab().save(str(OUT/'pressure_panel_light.png'))
    panel.resize(1280,820);pump();panel.pages.widget(0).verticalScrollBar().setValue(panel.pages.widget(0).verticalScrollBar().maximum());capture('tactile_small.png')
    panel.resize(1640,1040);panel.grasp_tactile.set_state({});capture('tactile_light_offline.png')
    # Full 30 s / 200 Hz backlog, still synthetic and disconnected.
    wave=panel.grasp_tactile.wave;wave.samples.clear();wave.last_stamp=-1;wave.last_key=None;wave.arrivals.clear();wave.window.setCurrentIndex(2);now=time.monotonic()
    for i in range(6000):wave.ingest(dict(updated=now-30+i*.005,sequence=i,fingers=fingers))
    wave.draws.clear();event_loop=QtCore.QEventLoop();QtCore.QTimer.singleShot(3000,event_loop.quit);event_loop.exec();long_history_hz=sum(time.monotonic()-t<1 for t in wave.draws)
    assert panel.grasp.process is None and panel.hand.process is None and panel.job is None
    (OUT/'visual_result.json').write_text(json.dumps(dict(hardware_connections=0,motor_commands=0,gl_pick_axis=axis,drag_target=target,synthetic_samples=seq,ingest_draw_ms_p50=float(np.percentile(durations,50)*1000),ingest_draw_ms_p95=float(np.percentile(durations,95)*1000),source_hz=seq/elapsed,display_calls_last_second=display_hz,long_history_samples=6000,long_history_display_calls_last_second=long_history_hz,gl_valid=gl.isValid(),horizontal_overflow=panel.pages.widget(0).horizontalScrollBar().maximum()),indent=2)+'\n')
finally:panel.close();app.processEvents()
