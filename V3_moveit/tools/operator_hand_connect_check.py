"""Explicit DH116 GUI connect/enable/disable check, without position commands."""
import argparse,json,time
from PyQt5 import QtWidgets as W
from operator_panel import Panel,ROOT

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware',required=True,action='store_true')
    parser.add_argument('--interface',default='enp11s0')
    parser.add_argument('--output',required=True)
    parser.add_argument('--connect-only',action='store_true',help='Only exercise the read-only Connect and Disconnect buttons')
    args=parser.parse_args()
    from pathlib import Path
    folder=Path(args.output).resolve();folder.mkdir(parents=True,exist_ok=True)
    app=W.QApplication([]);panel=Panel(preview=True);panel.show()
    panel.preview=False  # Only the explicit hand controls run; arm/scene timers stay off.
    panel.grasp_mode.setCurrentIndex(1);panel.hand_interface.setCurrentText(args.interface)
    panel.grasp_timer.start(15)
    events=[];sessions=[];states=[];panel.grasp.activity.connect(lambda event:events.append(dict(event)))
    report=dict(hardware=True,passed=False,arm_session_started=False,position_commands=0)

    def pump(predicate,seconds=10):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            app.processEvents()
            if panel.grasp.state:states.append(dict(panel.grasp.state))
            if predicate():return
            time.sleep(.002)
        raise AssertionError(panel.grasp.detail)

    def armed_check(name):
        # Mode changes deliberately close one owner before a queued Qt callback
        # opens the next. A transient process=None is not a connection failure.
        pump(lambda:panel.grasp.armed,60)
        assert panel.grasp.armed,panel.grasp.detail
        assert all(s&1 for s in panel.grasp.state['axis_status'])
        assert panel.grasp.state['homed'] and panel.grasp.tactile
        deadline=time.monotonic()+3
        pump(lambda:time.monotonic()>=deadline)
        assert panel.grasp.armed and panel.grasp.process is not None,panel.grasp.detail
        panel.set_page(0);scene=panel.scene
        for index in (1,0,1):
            panel.monitor_buttons[index].click();app.processEvents()
            assert panel.pages.currentIndex()==0 and panel.scene is scene
            assert panel.monitor_stack.currentIndex()==index and panel.grasp.armed
        panel.grab().save(str(folder/(name+'.png')))
        sessions.append(dict(name=name,log=panel.grasp.log.name,state=dict(panel.grasp.state)))

    try:
        panel.set_page(4);panel.grasp_monitor.click()
        pump(lambda:panel.grasp.ready and panel.grasp.mode=='monitor' and panel.hand_feedback() and panel.grasp.tactile,15)
        assert not any(s&1 for s in panel.grasp.state['axis_status'])
        first=panel.grasp.sequence;first_tactile=panel.grasp.tactile_sequence
        deadline=time.monotonic()+3;pump(lambda:time.monotonic()>=deadline)
        assert panel.hand_feedback() and panel.grasp.sequence>first and panel.grasp.tactile_sequence>first_tactile,panel.grasp.detail
        sessions.append(dict(name='monitor',log=panel.grasp.log.name,state=dict(panel.grasp.state)))
        report.update(interface=args.interface,connection_badge=panel.grasp_connection_badge.text(),home_button_available=panel.grasp_home.isEnabled(),
                      enable_button_available=panel.grasp_manual_enable.isEnabled(),tactile_feedback_seen=len(panel.grasp.tactile['fingers'])==5)
        if args.connect_only:
            panel.grasp_stop.click();pump(lambda:panel.grasp.process is None)
            assert panel.grasp.closed and panel.job is None
            panel.set_page(1);panel.device_boxes['hand'].setChecked(True);panel.hand_connect.click()
            pump(lambda:panel.hand.state.get('feedback_verified') and panel.hand.sequence>0)
            report['device_page_feedback']=panel.hand.state['axis_status']
            panel.set_page(4);panel.grasp_monitor.click()
            pump(lambda:panel.hand.process is None and panel.grasp.process is not None and panel.grasp.ready and panel.hand_feedback() and panel.grasp.tactile,15)
            assert panel.hand.closed and panel.grasp.mode=='monitor'
            report['device_page_handoff_init_confirmed']=panel.hand.closed
            sessions.append(dict(name='monitor_after_handoff',log=panel.grasp.log.name,state=dict(panel.grasp.state)))
            panel.grasp_stop.click();pump(lambda:panel.grasp.process is None)
            assert panel.grasp.closed and panel.job is None
            assert not any(e.get('event') in ('armed','manual_target','homing') for e in events)
            report.update(passed=True,read_only=True,cleanup=panel.grasp.result)
            return
        panel.grasp_manual_enable.click();armed_check('connected')
        panel.grasp_manual_disable.click()
        pump(lambda:panel.grasp.mode=='monitor' and panel.grasp.ready and panel.grasp.process is not None,60)
        assert not any(s&1 for s in panel.grasp.state['axis_status'])
        deadline=time.monotonic()+1;pump(lambda:time.monotonic()>=deadline)
        panel.grasp_manual_enable.click();armed_check('reenabled')
        panel.stop_grasp();pump(lambda:panel.grasp.process is None)
        assert panel.grasp.closed and panel.grasp.result['disabled'],panel.grasp.detail
        assert panel.job is None and not any(e.get('event')=='manual_target' for e in events)
        positions=[s['raw_positions'] for s in states]
        ranges=[max(p[i] for p in positions)-min(p[i] for p in positions) for i in range(6)]
        report.update(passed=True,sessions=sessions,cleanup=panel.grasp.result,position_span_eq=ranges,
                      all_six_axes_enable_confirmed=True,tactile_feedback_seen=True,
                      local_panel_switch_verified=True,reenable_confirmed=True,
                      full_motion_acceptance=False)
    except Exception as exc:
        report.update(error=str(exc),detail=panel.grasp.detail)
        raise
    finally:
        report['sessions']=sessions
        if panel.grasp.process is not None:
            panel.stop_grasp()
            try:pump(lambda:panel.grasp.process is None)
            except AssertionError:report['cleanup_timeout']=True
        report['last_cleanup']=getattr(panel.grasp,'result',{})
        (folder/'hardware_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        panel.preview=True;panel.close();app.processEvents()
    print(json.dumps(report,ensure_ascii=False))

if __name__=='__main__':main()
