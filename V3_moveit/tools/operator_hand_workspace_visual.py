"""Render real Qt widgets for inspection; hardware stays disconnected."""
import json,os,time
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5 import QtCore,QtGui,QtWidgets as W
from operator_panel import Panel,ROOT
app=W.QApplication([]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10));panel=Panel(preview=True);panel.set_page(4);panel.show()
def capture(name,width=1640,height=1040):
    panel.resize(width,height)
    for _ in range(8):app.processEvents();time.sleep(.01)
    panel.grab().save(str(ROOT/'artifacts'/name))
    scroll=panel.pages.widget(4)
    return dict(width=panel.width(),height=panel.height(),horizontal_overflow=scroll.horizontalScrollBar().maximum(),vertical_overflow=scroll.verticalScrollBar().maximum())
try:
    panel.brightness.setValue(100);sizes={'light':capture('hand_workspace_light.png')}
    panel.hand_bottom_tabs.setCurrentIndex(1);capture('hand_workspace_config.png')
    panel.hand_bottom_tabs.setCurrentIndex(2);panel.hand_log.append('warning','离线展示：测试日志筛选，不连接硬件');capture('hand_workspace_logs.png')
    panel.hand_bottom_tabs.setCurrentIndex(0);panel.grasp_tactile.tabs.setCurrentIndex(1);capture('hand_workspace_tactile_details.png')
    panel.grasp_tactile.tabs.setCurrentIndex(0);panel.hand_control_tabs.setCurrentIndex(1);capture('hand_workspace_presets.png')
    panel.hand_control_tabs.setCurrentIndex(0);panel.brightness.setValue(0);sizes['dark']=capture('hand_workspace_dark.png')
    sizes['small']=capture('hand_workspace_small.png',1280,820)
    assert all(d['horizontal_overflow']==0 for d in sizes.values()),sizes
    assert panel.grasp_all_stop.isVisible() and panel.mode.isVisible()
    assert panel.grasp_hand.image.width()==517 and panel.grasp_hand.image.height()==580
    assert panel.grasp.process is None and panel.hand.process is None and panel.job is None
    (ROOT/'artifacts/hand_workspace_visual_check.json').write_text(json.dumps(dict(layouts=sizes,hardware_connections=0,motor_commands=0,manufacturer_asset=True),indent=2)+'\n')
finally:panel.close();app.processEvents()
