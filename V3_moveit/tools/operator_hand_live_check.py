"""Real DH116 connect/disconnect button check; no arm or motor commands."""
import json,time
from PyQt5 import QtWidgets as W
from operator_panel import Panel,ROOT

app=W.QApplication([]);panel=Panel(preview=True);panel.show();panel.set_page(1)
def pump(predicate,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        app.processEvents();panel.studio_tick()
        if predicate():return True
        time.sleep(.02)
    return False
try:
    assert panel.hand_connect.isEnabled()
    panel.hand_connect.click()
    assert pump(lambda:panel.hand.sequence>=100 or panel.hand.process is None,60) and panel.hand.sequence>=100,panel.hand.state
    first=dict(panel.hand.state)
    assert first['feedback_verified'] and first['ok'] and any(first['raw_positions'])
    assert panel.job is None
    panel.grab().save(str(ROOT/'artifacts/dh116_connected_ui.png'))
    panel.hand_disconnect.click()
    assert pump(lambda:panel.hand.process is None,10),'Disconnect did not finish'
    assert panel.hand.closed and not panel.hand.state['ok'],panel.hand.state
    assert panel.hand_connect.isEnabled()
    result=dict(passed=True,real_feedback=first,disconnect_init_confirmed=panel.hand.closed,arm_session_started=False,motor_commands=0)
    (ROOT/'artifacts/dh116_gui_live_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False))
finally:
    if panel.hand.process is not None:
        panel.hand.disconnect_device();pump(lambda:panel.hand.process is None,10)
    panel.close();app.processEvents()
