#!/usr/bin/env python3
"""Ubuntu desktop operator workbench. Starts no motion on opening."""
import argparse,fcntl,json,os,resource,signal,socket,subprocess,sys,time
from pathlib import Path
from operator_studio import StudioView
from operator_access import AccessWorkflow
from operator_quest_link import QuestLink
from operator_arm import ArmControl
from operator_tooling import mismatch,recovery,tool_values
from PyQt5 import QtCore,QtGui,QtWidgets as W
ROOT=Path(__file__).resolve().parents[1]

class Panel(AccessWorkflow,StudioView,W.QMainWindow):
    def __init__(self,preview=False):
        super().__init__();self.preview=preview;self.job=None;self.auth=None;self.start_after_auth=False;self.sock=None;self.position=0;self.pending_line="";self.status_input_at=0;self.stop_requested=False;self.tool_notices=set();self.tool_dialog=None
        self.quest_link=QuestLink(self);self.arm_control=ArmControl(self);self.build_ui();self.build_access_controls();self.apply_theme()
        self.arm_control.changed.connect(self.refresh_arm_control)
        self.quest_link.changed.connect(self.refresh_quest_link)
        if not preview:self.quest_link.start()
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick)
        if not preview:self.timer.start(150)
        self.refresh()
    def message(self,text):self.status.setText(text);self.latest_event.setText(text);self.logs.appendPlainText(text)
    def refresh(self):
        busy=self.job is not None or self.arm_control.busy;hardware=self.mode.currentIndex()==1;rt=resource.getrlimit(resource.RLIMIT_RTPRIO)[0]>=50
        if not (hardware and self.site.isChecked() and self.device_boxes['arm'].isChecked() and self.device_boxes['quest'].isChecked()):
            self.start_after_auth=False
        self.mode.setEnabled(not busy);self.quest.setEnabled(self.quest_link.reconnect is None);self.rt.setEnabled(not busy and not rt and self.auth is None)
        self.rt_label.setText('已启用 · 优先级上限 50' if rt else '真机启动前需系统认证')
        self.site.setVisible(hardware);self.site.setEnabled(not busy)
        allowed=not busy and (not hardware or (rt and self.site.isChecked()))
        self.start.setText('启动真机遥操作' if hardware else '启动仿真遥操作');self.start.setEnabled(not busy and self.auth is None)
        self.test.setText('运行 60 秒真机检查' if hardware else '运行 60 秒仿真检查');self.test.setEnabled(allowed)
        self.prepare.setEnabled(allowed and hardware);self.stop.setEnabled(self.job is not None and not self.stop_requested)
        self.accept.setEnabled(not busy and self.observed.isChecked())
        self.refresh_view(busy,hardware)
        self.refresh_access(busy,hardware)
        self.refresh_arm_buttons()
        self.refresh_tooling()
    def authorize(self):
        if self.auth is not None or self.job is not None or self.arm_control.busy:return
        if self.arm_control.after_poll(self.authorize):return
        self.auth=subprocess.Popen(['/usr/bin/pkexec','/usr/bin/prlimit','--pid',str(os.getpid()),'--rtprio=50:50'],stdin=subprocess.DEVNULL)
        self.message('认证后将继续启动，请看头显中的就绪提示。' if self.start_after_auth else '请在系统窗口完成认证，本次面板即可使用实时权限。');self.refresh()
    def close_input(self):
        if self.sock is not None:self.sock.close();self.sock=None
    def launch_command(self,argv,title):
        self.tool_notices.clear()
        self.session_instruction='';self.session_error='';self.session_terminal='';self.session_cleanup_warning=False
        self.close_input();self.observed.setChecked(False);self.stop_requested=False;self.job_started_at=time.monotonic();self.clear_feedback();self.plot.samples.clear();self.logs.clear()
        self.logpath=ROOT/'.cache/operator/panel_session.log';self.logfile=self.logpath.open('w');self.position=0;self.pending_line=""
        self.job=subprocess.Popen(argv,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=self.logfile,stderr=subprocess.STDOUT,start_new_session=True)
        self.message(title);self.refresh()
    def launch(self,task):
        if self.job is not None or self.auth is not None or self.arm_control.busy:return
        mode=self.mode.currentIndex();hardware=mode==1
        if hardware and (not self.site.isChecked() or resource.getrlimit(resource.RLIMIT_RTPRIO)[0]<50):return
        def resume():
            if self.mode.currentIndex()==mode:self.launch(task)
        if self.arm_control.after_poll(resume):return
        argv=['bash','tools/operator_ros.sh','python3','-B','tools/operator_session.py','--task',task]
        if hardware:argv.append('--hardware')
        self.launch_command(argv,'正在启动'+('真机' if hardware else '仿真')+'。')
        self.using_hardware_teleop=bool(hardware and task=='teleop' and self.job is not None)
        self.hand_teleop=bool(self.using_hardware_teleop and self.grasp.persistent and self.device_boxes['hand'].isChecked())
        wants_control=(self.grasp.initial_source!='disabled' if self.grasp.mode=='workspace' else self.grasp.mode!='monitor')
        self.hand_waiting_trigger=bool(self.hand_teleop and wants_control)
    def connect_quest(self):
        if not self.preview:self.quest_link.repair();self.refresh()
    def refresh_quest_link(self):
        self.quest_link_label.setText(self.quest_link.state.get('detail','正在检查 Quest USB / HTTPS'))
        self.quest.setEnabled(self.quest_link.reconnect is None)
    def arm_action(self,action):
        if self.preview or (self.mode.currentIndex()!=1 and action!='status') or self.job is not None or self.arm_control.busy or self.auth is not None or not self.device_boxes['arm'].isChecked():return
        if self.arm_control.after_poll(lambda:self.arm_action(action)):return
        if action!='status' and not self.site.isChecked():self.message('请先确认右侧现场准备，再操作机器人或刷新配置。');return
        self.tool_notices.clear()
        try:
            if action=='select_tool':self.arm_control.start(action,tool_id=self.tool_select.currentData())
            else:self.arm_control.start(action)
        except (OSError,ValueError) as error:
            self.arm_control.connection_state='error';self.arm_control.error=str(error)
            self.message(str(error));self.refresh()
    def refresh_arm_control(self):
        self.arm_power_status.setText(self.arm_control.detail);self.refresh()
        control=self.arm_control
        if control.process is None and not control.background:
            for item in getattr(control,'tooling',{}).get('tools',[]):
                index=self.tool_select.findData(item['tool_id'])
                if index>=0:self.tool_select.setItemData(index,'JAKA TCP（mm / rad）：'+str(item['tcp_mm_rad']),QtCore.Qt.ToolTipRole)
            kind,detail=recovery(getattr(control,'error',''))
            if not kind and control.result.get('ok') and getattr(control,'tooling',{}):
                try:kind,detail=mismatch(control.tooling,json.loads((ROOT/'src/v3_moveit/config/payload.json').read_text()))
                except ValueError as e:self.message(str(e))
            if kind:self.show_tool_notice(kind,detail)
            if control.action in ('select_tool','refresh_payload'):
                self.stop_access_probe();self.access_state={'loaded':False};self.request_access_probe()
                if control.result.get('ok'):
                    self.message(control.detail)
                    if not self.preview:self.scene.stop();self.scene.start()
    def refresh_arm_buttons(self):
        control=self.arm_control;available=self.job is None and not control.busy and self.auth is None and self.device_boxes['arm'].isChecked()
        hardware=self.mode.currentIndex()==1;state=control.state;fresh=bool(state) and 0<=time.monotonic()-state.get('updated',0)<6
        connection=control.connection_state
        if control.connection_requested and connection=='connected' and not fresh:connection='connecting'
        text={'disconnected':'连接机械臂','connecting':'正在连接…','connected':'● 已连接','occupied':'● 被占用','error':'连接失败 · 重试'}[connection]
        style={'connected':'on','occupied':'occupied','error':'error','connecting':'working'}.get(connection,'')
        if self.job is not None:
            text=('● 遥操作已连接' if self.feedback_at else '正在接入…') if hardware else '仿真运行中'
            style='on' if hardware and self.feedback_at else 'working'
        def button(widget,title,kind):
            widget.setText(title)
            if widget.property('armState')!=kind:widget.setProperty('armState',kind);widget.style().unpolish(widget);widget.style().polish(widget)
        button(self.arm_connect_button,text,style);self.arm_connect_button.setEnabled(available)
        self.arm_connect_button.setToolTip(control.error or ('机器人正被其他 Servo、程序、拖动或运动队列占用；结束占用后会自动更新。' if connection=='occupied' else '连接后自动更新状态；再次点击已连接按钮可断开。'))
        if connection=='error':self.arm_connect_button.setToolTip('请检查机器人地址和网络，或释放手机 / 其他上位机的控制占用后重试。\n'+control.error)
        self.tool_read.setEnabled(available);self.tool_select.setEnabled(available and hardware)
        writable=available and hardware and self.site.isChecked() and connection=='connected' and fresh
        self.tool_apply.setEnabled(writable);self.payload_refresh.setEnabled(writable)
        for widget,key,title,action in ((self.arm_power_button,'powered_on','本体电源','打开本体电源'),(self.arm_enable_button,'enabled','机器人使能','使能机器人')):
            value=state.get(key);known=fresh and connection in ('connected','occupied')
            label=title+' · '+(('已开启' if key=='powered_on' else '已使能') if value==1 else ('未开启' if key=='powered_on' else '未使能')) if known else title+' · '+('上次已开启' if value==1 and key=='powered_on' else '上次已使能' if value==1 else '待读取')
            working=control.busy and control.action==('power_on' if key=='powered_on' else 'enable')
            button(widget,'正在'+action+'…' if working else label,'working' if working else 'on' if known and value==1 else '')
            widget.setEnabled(writable and value!=1 and (key=='powered_on' or state.get('powered_on')==1))
            widget.setToolTip('请先连接机械臂，切换真机模式并确认现场准备。' if not writable else action+'；执行前核对状态、TCP、负载与模型。')
    def toggle_arm_connection(self):
        if self.preview or self.job is not None or self.arm_control.busy or self.auth is not None or not self.device_boxes['arm'].isChecked():return
        control=self.arm_control
        if control.connection_requested and control.connection_state in ('connected','connecting'):control.disconnect();return
        control.connection_requested=True;control.last_poll=0.;self.poll_arm_connection()
    def poll_arm_connection(self):
        control=self.arm_control
        if self.preview or self.job is not None or self.auth is not None or control.process is not None or control.deferred is not None or not control.connection_requested or not self.device_boxes['arm'].isChecked():return
        if time.monotonic()-control.last_poll<2:return
        try:control.start('poll',background=True)
        except (OSError,ValueError) as error:control.connection_state='error';control.error=str(error);control.last_poll=time.monotonic();self.refresh_arm_buttons()
    def open_tooling(self):
        self.refresh_tooling();self.tooling_popover.show_at(self.tool_read)
        if not self.preview:self.arm_action('status')
    def refresh_tooling(self):
        try:
            config=json.loads((ROOT/'src/v3_moveit/config/payload.json').read_text());tool,tcp=tool_values(config)
            def values(d):return f"{d['payload_kg']:.6f} kg · 质心 XYZ（mm）"+' / '.join(f'{v:.6f}' for v in d['cog_mm'])
            observed=getattr(self.arm_control,'tooling',{})
            current='待读取'
            if observed:
                age=time.monotonic()-observed['updated'];current=('法兰中心' if observed['tool_id']==0 else 'TCP'+str(observed['tool_id']))+('（上次读数）' if age>10 or self.job is not None else '（已回读）')
            self.tool_current.setText(f'JAKA 当前：{current} · V3：'+('法兰中心' if tool==0 else 'TCP'+str(tool)))
            self.payload_status.setText('V3：'+values(config)+('\nJAKA 上次回读：'+values(observed) if observed else ''))
            self.payload_status.setToolTip('V3 TCP（mm / rad）：'+str(tcp)+'\n保存配置使用完整精度；此处仅格式化显示。')
            detail='正在读取 JAKA…' if self.arm_control.process is not None and self.arm_control.action=='status' else self.arm_control.error or ('JAKA 上次回读 · '+f"{max(0,time.monotonic()-observed['updated']):.0f} 秒前" if observed else '点击读取后显示 JAKA 当前数据')
            self.tooling_popover.update_values(config,observed,detail)
        except (OSError,ValueError,KeyError) as e:self.payload_status.setText('工具配置读取失败：'+str(e))
    def show_tool_notice(self,kind,detail):
        if kind in self.tool_notices:return
        self.tool_notices.add(kind);self.message(detail)
        if self.tool_dialog is not None:self.tool_dialog.close()
        dialog=W.QMessageBox(W.QMessageBox.Warning,{'payload':'负载或质心与 V3 配置不一致','tool':'请核对 JAKA 工具坐标系','network':'机械臂网卡地址未就绪'}.get(kind,'启动检查提醒'),detail,W.QMessageBox.Ok,self)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose);dialog.setModal(False);self.tool_dialog=dialog
        dialog.finished.connect(lambda *_:setattr(self,'tool_dialog',None));dialog.show()
    def stop_session(self):
        stopped=[]
        self.start_after_auth=False
        self.pending_hand_mode=None;self.pending_grasp=None;self.pending_manual_target=None;self.grasp_transition+=1
        if self.grasp.process is not None:
            self.grasp.stop();stopped.append('DH116 抓握')
        if self.hand.process is not None:
            self.hand.disconnect_device();stopped.append('DH116 只读反馈')
        if self.device_monitors:
            self.stop_monitor();stopped.append('设备反馈模块')
        if self.job is not None and self.job.poll() is None:
            self.stop_requested=True;self.job.send_signal(signal.SIGINT);stopped.append('机械臂遥操作');self.stop.setEnabled(False)
        if self.arm_control.deferred is not None or (self.arm_control.connection_requested and self.arm_control.background):
            self.arm_control.disconnect();stopped.append('机械臂状态连接')
        detail='已请求停止 '+ '、'.join(stopped) if stopped else '当前没有运行中的灵巧手、机械臂或设备反馈模块。'
        self.message(detail);self.grasp_notice(detail)
    def accept_result(self):
        if not self.observed.isChecked():return
        try:
            from operator_session import accept_local_observation
            accept_local_observation()
            # Discard any probe begun before saving; it must not overwrite the
            # freshly validated acceptance with an older snapshot.
            self.stop_access_probe()
            self.access_state['accepted']=dict(ok=True,detail='已保存本版本的现场验收记录')
            self.message('已保存本次开发验收记录。')
        except Exception as e:self.message(str(e))
        self.request_access_probe();self.refresh()
    def consume_event(self,event):
        if event.get('ready') and getattr(self,'hand_waiting_trigger',False):
            self.hand_waiting_trigger=False
            if self.hand_teleop and self.grasp.persistent and not self.grasp.stopping:self.request_grasp_mode('control')
        self.event_view(event)
        if 'instruction' in event:self.session_instruction=event['instruction']
        if event.get('error'):self.session_error=event.get('message','检查未通过');self.session_instruction=''
        if event.get('recovery_kind'):self.show_tool_notice(event['recovery_kind'],event['message'])
        if event.get('canceled') or event.get('passed'):self.session_terminal=event.get('message','');self.session_instruction=''
        if event.get('cleanup_warning'):self.session_cleanup_warning=True
        if not self.stop_requested:self.status.setText(self.session_instruction or event['message'])
        if event.get('joint_degrees'):self.joint_label.setText('关节角（°）：  '+'  ·  '.join(str(v) for v in event['joint_degrees']))
        if 'input_reason' in event:self.input_label.setText({'RELEASED':'Grip 已松开','TRACKING_LOST':'手柄未跟踪','SOFTWARE_STOP':'摇杆软件停止','HELD':'Grip 握持中'}.get(event['input_reason'],event['input_reason']))
    def tick(self):
        self.tick_view();self.poll_access();self.poll_arm_connection()
        if self.auth is not None and self.auth.poll() is not None:
            granted=self.auth.returncode==0 and resource.getrlimit(resource.RLIMIT_RTPRIO)[0]>=50
            resume=self.start_after_auth and granted
            self.auth=None;self.start_after_auth=False
            self.message('本次实时权限已启用。' if granted else '系统认证未完成，可重新点击实时权限按钮。');self.refresh()
            if resume and self.mode.currentIndex()==1 and self.site.isChecked():self.primary_start()
        if self.job is not None:
            finished_code=self.job.poll()
            with self.logpath.open() as stream:stream.seek(self.position);data=stream.read();self.position=stream.tell()
            if data:
                self.logs.appendPlainText(data.rstrip())
                lines=(self.pending_line+data).split('\n');self.pending_line=lines.pop()
                for line in lines:
                    if line.startswith('OP '):
                        try:
                            self.consume_event(json.loads(line[3:]))
                        except (ValueError,KeyError):pass
            if finished_code is not None:
                code=finished_code;self.job=None;self.logfile.close()
                self.session_finished(code)
            return
        if self.preview:return
        if self.sock is None:
            try:self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('0.0.0.0',5006));self.sock.setblocking(False)
            except OSError:
                self.close_input();self.input_label.setText('输入端口由其他会话占用');return
        try:
            for _ in range(80):
                d=json.loads(self.sock.recvfrom(65535)[0]);r=d['right'];self.status_input_at=time.monotonic()
                self.input_label.setText(('追踪有效' if r.get('tracked') and r.get('pose_valid') else '未跟踪 · 唤醒右手柄')+' · Grip '+str(round(r.get('grip',0),2)))
        except BlockingIOError:pass
        except (ValueError,KeyError,TypeError):self.input_label.setText('收到无效输入')
        if time.monotonic()-self.status_input_at>2:self.input_label.setText('等待 Quest 网页进入透视模式')
    def closeEvent(self,event):
        if self.arm_control.busy:event.ignore();self.message('正在确认机器人电源 / 使能结果，请等待当前操作结束。');return
        if self.arm_control.process is not None:
            event.ignore();self.arm_control.connection_requested=False;self.arm_control.after_poll(self.close);return
        if self.grasp.process is not None:
            event.ignore();self.stop_grasp();self.message('正在停止 DH116 并确认去使能。');return
        if self.auth is not None:event.ignore();self.message('请先完成或取消系统认证窗口。');return
        if self.job is not None:event.ignore();self.stop_session();return
        if self.hand.process is not None:
            event.ignore();self.hand.disconnect_device();self.message('正在断开 DH116，请在断开完成后关闭窗口。');return
        if getattr(self,'about_dialog',None) is not None:self.about_dialog.close()
        self.quest_link.stop();self.close_grasp();self.stop_access_probe();self.close_input();self.stop_monitor();self.scene.stop();event.accept()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--snapshot');parser.add_argument('--snapshot-live');parser.add_argument('--monitor',action='store_true');parser.add_argument('--page',choices=['teleop','devices','checks','logs','grasp'],default='teleop');args=parser.parse_args();os.chdir(ROOT)
    lock=(ROOT/'.cache/operator/panel.lock').open('w')
    if not (args.snapshot or args.snapshot_live):
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('上位机已打开，请使用已有窗口。')
    W.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling);W.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps)
    QtGui.QGuiApplication.setHighDpiScaleFactorRoundingPolicy(QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app=W.QApplication(sys.argv[:1]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10));panel=Panel(preview=bool(args.snapshot or args.snapshot_live));panel.show();panel.set_page(['teleop','devices','checks','logs','grasp'].index(args.page));panel.refresh()
    if args.monitor and not panel.preview:panel.start_monitor()
    if args.snapshot_live:
        panel.snapshot_live=True
        os.environ["V3_SCENE_CAPTURE"]="1";panel.prefs.write_runtime();panel.scene.start();panel.start_monitor();panel.telemetry_timer.start(30)
        def capture_live():
            pix=panel.grab()
            pix.save(args.snapshot_live);panel.close();app.quit()
        QtCore.QTimer.singleShot(12000,capture_live)
    if args.snapshot:
        QtCore.QTimer.singleShot(300,lambda:(panel.grab().save(args.snapshot),app.quit()))
    return app.exec_()
if __name__=='__main__':raise SystemExit(main())
