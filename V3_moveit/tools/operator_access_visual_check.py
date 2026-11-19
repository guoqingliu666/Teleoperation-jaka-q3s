"""Real Qt access-page verification, without controller or SDK calls."""
import json,time,os
os.environ['QT_QPA_PLATFORM']='xcb'
from PyQt5 import QtCore,QtGui,QtWidgets as W,QtTest
from operator_panel import Panel,ROOT
W.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
app=W.QApplication([]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10));panel=Panel(preview=True);panel.preview=False;panel.show();panel.mode.setCurrentIndex(1);panel.request_access_probe()
try:
    deadline=time.monotonic()+20
    while not panel.access_state.get('loaded') and time.monotonic()<deadline:app.processEvents();time.sleep(.03)
    assert panel.access_state.get('loaded'), 'Readiness reader did not complete'
    assert panel.access_state['software']['ok'];assert panel.access_state['preflight']['ok']
    QtTest.QTest.mouseClick(panel.start,QtCore.Qt.LeftButton);app.processEvents()
    assert panel.pages.currentIndex()==2 and panel.job is None
    assert panel.access_action()[0]=='fake',panel.access_state
    assert not panel.site.isChecked()
    QtTest.QTest.mouseClick(panel.site,QtCore.Qt.LeftButton);app.processEvents();assert panel.site.isChecked()
    QtTest.QTest.mouseClick(panel.site,QtCore.Qt.LeftButton);app.processEvents();assert not panel.site.isChecked()
    assert panel.stop.isVisible();assert panel.pages.currentWidget().horizontalScrollBar().maximum()==0
    panel.grab().save(str(ROOT/'artifacts/operator_access_checks.png'))
    panel.set_page(0);app.processEvents();panel.grab().save(str(ROOT/'artifacts/operator_access_entry.png'))
    result=dict(passed=True,read_only=True,hardware_control_started=False,readiness=panel.access_state,main_button_opens_guidance=True,site_checkbox_clickable=True,stop_visible=True)
    (ROOT/'artifacts/operator_access_visual.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print('PASS: actual readiness worker, explicit next step, clickable checkbox, no control launch')
finally:panel.close();app.processEvents()
