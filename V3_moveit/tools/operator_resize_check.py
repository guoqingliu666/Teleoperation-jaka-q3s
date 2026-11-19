"""Actual Qt/RViz resize and camera-focus verification; no robot control session."""
import json,os,time
from pathlib import Path
os.environ['QT_QPA_PLATFORM']='xcb';os.environ['V3_SCENE_CAPTURE']='1'
import cv2,numpy as np
from PyQt5 import QtWidgets as W,QtCore,QtGui,QtTest
from operator_panel import Panel,ROOT
W.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling);W.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)
QtGui.QGuiApplication.setHighDpiScaleFactorRoundingPolicy(QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
app=W.QApplication([]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10));panel=Panel(preview=True);panel.setFixedSize(1640,1040);panel.show();panel.prefs.write_runtime();panel.scene.start();panel.start_monitor()
def pump(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:app.processEvents();panel.studio_tick();time.sleep(.02)
def pixels(image):
    image=image.convertToFormat(QtGui.QImage.Format_RGBA8888);bits=image.bits();bits.setsize(image.byteCount());return np.frombuffer(bits,np.uint8).reshape(image.height(),image.bytesPerLine()//4,4)[:,:image.width()].copy()
def blue_shape(image):
    a=pixels(image);mask=((a[:,:,2]>75)&(a[:,:,1]>45)&(a[:,:,0]<70)&(a[:,:,2]>a[:,:,0]*1.5)).astype(np.uint8)
    count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
    stats=stats[1:];stats=stats[stats[:,cv2.CC_STAT_AREA]>30];largest=stats[np.argmax(stats[:,cv2.CC_STAT_AREA])];return int(largest[2]),int(largest[3])
records=[]
try:
    pump(6)
    brightness=panel.prefs.data['brightness']/100
    expected='#'+''.join(f'{round(int("1d1f23"[i:i+2],16)*(1-brightness)+255*brightness):02x}' for i in (0,2,4))
    assert panel.scene.parentWidget().grab().toImage().pixelColor(50,8).name()==expected,'First-frame theme mismatch'
    for w,h in [(1240,800),(1640,1040),(2480,1400),(3440,1440),(1640,1040)]:
        panel.setFixedSize(w,h);pump(1.5);scene=panel.scene;img=scene.image;rect=scene.image_rect();assert not img.isNull() and time.monotonic()-scene.frame_at<1
        sx,sy=rect.width()/img.width(),rect.height()/img.height();assert abs(sx/sy-1)<1e-6
        assert abs((img.width()/img.height())/(scene.width()/scene.height())-1)<.004
        assert panel.pages.currentWidget().horizontalScrollBar().maximum()==0,'Horizontal clipping'
        assert panel.scene.rect().contains(scene.overlay.geometry())
        for tile in panel.camera_tiles.values():assert tile.width()*9==tile.height()*16
        bw,bh=blue_shape(img);records.append(dict(window=[w,h],canvas=[scene.width(),scene.height()],render=[img.width(),img.height()],paint_scale=[sx,sy],blue_mesh_bounds=[bw,bh],shape_ratio=bw/bh,camera_size=[tile.width(),tile.height()]))
        panel.brightness.setValue(0);pump(.2);panel.grab().save(str(ROOT/f'artifacts/operator_left_{w}x{h}_dark.png'))
    ratios=[r['shape_ratio'] for r in records];assert max(ratios)/min(ratios)<1.06,ratios
    assert all(panel.media_states.get(n,{}).get('ok') for n in ('zed_left','zed_depth','c922')),panel.media_states
    panel.brightness.setValue(100);pump(.4);panel.grab().save(str(ROOT/'artifacts/operator_left_final_light.png'))
    panel.brightness.setValue(0);pump(.4);panel.grab().save(str(ROOT/'artifacts/operator_left_final_dark.png'))
    QtTest.QTest.mouseClick(panel.camera_tiles['c922'],QtCore.Qt.LeftButton);pump(.3);assert scene.camera_focus.isVisible();assert panel.stop.isVisible()
    panel.grab().save(str(ROOT/'artifacts/operator_camera_focus.png'))
    QtTest.QTest.keyClick(scene.camera_focus,QtCore.Qt.Key_Escape);pump(.2);assert not scene.camera_focus.isVisible()
    panel.set_page(1);pump(.3);panel.grab().save(str(ROOT/'artifacts/operator_left_devices_dark.png'))
    (ROOT/'artifacts/operator_resize_check.json').write_text(json.dumps(dict(passed=True,display_only=True,hardware_control=False,resizes=records,shape_ratio_spread=max(ratios)/min(ratios),first_frame_theme=True,camera_focus=True,stop_visible_during_focus=True,real_cameras=True),indent=2)+'\n')
    print('PASS: proportional rendering, 5 resizes, real camera rail, focus/Esc, stop visible')
finally:panel.close();app.processEvents()
