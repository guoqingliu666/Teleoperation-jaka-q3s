"""Real GL input routing and credits screenshots; preview only, zero hardware."""
import json,os,time
import numpy as np
os.environ.setdefault('QT_QPA_PLATFORM','xcb')
from PyQt5 import QtCore,QtGui,QtWidgets as W,QtTest
from operator_panel import Panel,ROOT
from operator_hand_model import PUSH_PARTS
OUT=ROOT/'artifacts/operator_credits_drag_20261115';OUT.mkdir(exist_ok=True)
app=W.QApplication([]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10));p=Panel(preview=True);p.setWindowTitle('V3 · 离线交互检查');p.show();p.set_page(4)
def wait(ms=150):
    loop=QtCore.QEventLoop();QtCore.QTimer.singleShot(ms,loop.quit);loop.exec()
def move(gl,pos,button):
    app.sendEvent(gl,QtGui.QMouseEvent(QtCore.QEvent.MouseMove,QtCore.QPointF(pos),QtCore.Qt.NoButton,button,QtCore.Qt.NoModifier))
def press(gl,pos,button):
    app.sendEvent(gl,QtGui.QMouseEvent(QtCore.QEvent.MouseButtonPress,QtCore.QPointF(pos),button,button,QtCore.Qt.NoModifier))
def release(gl,pos,button):
    app.sendEvent(gl,QtGui.QMouseEvent(QtCore.QEvent.MouseButtonRelease,QtCore.QPointF(pos),button,QtCore.Qt.NoButton,QtCore.Qt.NoModifier))
def hit_point(model,names):
    gl=model.gl;gl.makeCurrent();d=gl.devicePixelRatioF()
    for name in names:
        item=model.items[name];point=model.screen_point(name,item.local_center,model.values)
        for x,y in ((0,0),(-3,0),(3,0),(0,3),(0,-3)):
            pt=QtCore.QPoint(round(point[0]+x),round(point[1]+y));hits=gl.itemsAt((pt.x()*d,pt.y()*d,2*d,2*d))
            if hits and hits[0].part_name in names:return pt,hits[0]
    raise AssertionError('No visible surface '+str(names))
try:
    p.brightness.setValue(100);wait(650);m=p.hand_model;gl=m.gl;assert gl and gl.isValid();commands=[];previews=[];frames=[]
    m.targetChanged.connect(lambda a,v:commands.append((time.monotonic(),a,v)));m.previewChanged.connect(lambda a,v:previews.append((time.monotonic(),a,v)));gl.frameSwapped.connect(lambda:frames.append(time.monotonic()))
    camera=(gl.opts['azimuth'],gl.opts['elevation']);empty=QtCore.QPoint(10,10)
    press(gl,empty,QtCore.Qt.LeftButton);move(gl,empty+QtCore.QPoint(45,25),QtCore.Qt.LeftButton);release(gl,empty+QtCore.QPoint(45,25),QtCore.Qt.LeftButton);assert (gl.opts['azimuth'],gl.opts['elevation'])==camera and not commands
    # Silver / black surfaces are not joint handles and do not orbit on left drag.
    pt,item=hit_point(m,['finger23_link','finger33_link','base_link'])
    press(gl,pt,QtCore.Qt.LeftButton);assert m.dragging is None;move(gl,pt+QtCore.QPoint(20,10),QtCore.Qt.LeftButton);release(gl,pt,QtCore.Qt.LeftButton);assert (gl.opts['azimuth'],gl.opts['elevation'])==camera and not commands
    pt,item=hit_point(m,PUSH_PARTS)
    press(gl,pt,QtCore.Qt.RightButton);move(gl,pt+QtCore.QPoint(35,16),QtCore.Qt.RightButton);release(gl,pt+QtCore.QPoint(35,16),QtCore.Qt.RightButton);assert (gl.opts['azimuth'],gl.opts['elevation'])!=camera and not commands
    m.reset_camera();wait();pt,item=hit_point(m,('finger21_link','finger31_link','finger41_link','finger51_link'))
    press(gl,pt,QtCore.Qt.LeftButton);assert m.dragging is not None
    axis=m.dragging['axis'];t=m.dragging['tangent'];gain=1/np.linalg.norm(t);started=time.monotonic();frames.clear()
    while time.monotonic()-started<.9:
        fraction=min(1,(time.monotonic()-started)/.9);end=pt+QtCore.QPoint(round(t[0]*4000*fraction),round(t[1]*4000*fraction));move(gl,end,QtCore.Qt.LeftButton);assert m.dragging is not None,('drag cancelled before wait',fraction);wait(8);assert m.dragging is not None,('drag cancelled after wait',fraction)
    end=pt+QtCore.QPoint(round(t[0]*4000),round(t[1]*4000));release(gl,end,QtCore.Qt.LeftButton);wait(100)
    target=p.manual_targets[axis][0].value();assert target==p.manual_targets[axis][1].value() and 3900<target<4100,(axis,target,m.values,commands[-3:],previews[-3:])
    assert len(previews)>len(commands) and len(commands)<35,(len(previews),len(commands))
    assert all(b[0]-a[0]>=1/30-.0002 for a,b in zip(commands[:-1],commands[1:-1]))
    frame_ms_p95=float(np.percentile(np.diff(frames)*1000,95));drag_updates=len(previews);command_updates=len(commands)
    assert m.dragging is None and not m.drag_timer.isActive();p.grab().save(str(OUT/'hand_drag_light.png'))
    pt,item=hit_point(m,PUSH_PARTS);press(gl,pt,QtCore.Qt.LeftButton);assert m.dragging is not None
    move(gl,pt+QtCore.QPoint(40,20),QtCore.Qt.LeftButton);app.sendEvent(gl,QtGui.QFocusEvent(QtCore.QEvent.FocusOut));count=len(commands);wait(90);assert m.dragging is None and not m.drag_timer.isActive() and len(commands)==count
    p.about_button.click();a=p.about_dialog;a.roll.set_playing(False);wait();a.grab().save(str(OUT/'credits_light.png'))
    a.roll.set_offset(900);wait();a.grab().save(str(OUT/'credits_roles_light.png'))
    p.brightness.setValue(0);wait(240);a.grab().save(str(OUT/'credits_dark.png'))
    a.roll.set_offset(a.roll.verticalScrollBar().maximum());wait();a.grab().save(str(OUT/'credits_ending.png'))
    a.roll.restart();a.roll.wait_until=0;wait(180);assert a.roll.offset>0;a.close();assert not a.roll.timer.isActive();p.grab().save(str(OUT/'hand_drag_dark.png'))
    assert p.grasp.process is None and p.hand.process is None and p.job is None
    result=dict(left_background_does_not_orbit=True,left_silver_does_not_drag=True,right_orbits_without_targets=True,focus_loss_cancels_without_extra_target=True,pink_joint_drag_axis=axis,target=target,ui_updates=drag_updates,command_updates=command_updates,eq_per_pixel=gain,render_frame_ms_p95=frame_ms_p95,hardware_connections=0,motor_commands=0,credits_nonmodal=a.windowModality()==QtCore.Qt.NonModal)
    (OUT/'visual_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(result)
finally:p.close();app.processEvents()
