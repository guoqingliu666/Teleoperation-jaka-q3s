"""Optional development checks and their saved results."""
import json,resource,time
from PyQt5 import QtCore,QtWidgets as W
from operator_view import frame,label
from operator_settings import ROOT

class AccessWorkflow:
    def build_access_controls(self):
        self.access_state={'loaded':False};self.access_probe=None;self.access_signature=None;self.access_polled=0.;self.access_checked_at=0.;self.session_instruction='';self.session_error='';self.session_terminal='';self.session_cleanup_warning=False;self.access_flow=False
        page=self.pages.widget(2).widget();card,box=frame();box.addWidget(label('开发验收流程','section'))
        box.addWidget(label('开发或调试时可在此验证跟随方向、松握与停止行为，并保存现场观察结果。','muted'))
        self.access_lines={}
        for name,title in [('software','软件检查'),('fake','Quest 仿真检查'),('permission','本次实时权限'),('site','现场准备'),('preflight','实机状态与反馈'),('hardware','真机检查'),('accepted','现场结果确认')]:
            item=label(title+'：读取中');self.access_lines[name]=item;box.addWidget(item)
        self.access_detail=label('','muted');box.addWidget(self.access_detail)
        self.access_next=W.QPushButton('读取接入状态…');self.access_next.setObjectName('primary');self.access_next.clicked.connect(self.advance_access);box.addWidget(self.access_next)
        box.addWidget(label('真机检查包含状态 / TCP / 负载与反馈核对、必要的 12 秒原位保持，以及 60 秒手柄跟随和两种停止检查。仍使用已确认的速度、碰撞保护及 30° / 1000 mm 观察退出范围。','muted'))
        self.access_cancel=W.QPushButton('返回遥操作');self.access_cancel.clicked.connect(lambda:self.set_page(0));box.addWidget(self.access_cancel)
        page.layout().insertWidget(2,card)
        self.access_hint=label('','muted');self.access_hint.setWordWrap(True)
        self.start.parentWidget().layout().insertWidget(self.start.parentWidget().layout().indexOf(self.start),self.access_hint)
    def request_access_probe(self):
        if self.preview or self.pages.currentIndex()!=2 or self.job is not None or self.access_probe is not None:return
        process=QtCore.QProcess(self);self.access_probe=process
        process.setWorkingDirectory(str(ROOT));process.setProcessChannelMode(QtCore.QProcess.SeparateChannels)
        process.finished.connect(self.finish_access_probe)
        process.errorOccurred.connect(lambda e:self.finish_access_probe(-1) if e==QtCore.QProcess.FailedToStart else None)
        process.start('bash',['tools/operator_ros.sh','python3','-B','tools/operator_readiness.py'])
    def finish_access_probe(self,code,*unused):
        process=self.access_probe
        if process is None:return
        try:
            if code:raise ValueError(bytes(process.readAllStandardError()).decode()[-1000:])
            self.access_state=json.loads(bytes(process.readAllStandardOutput()).decode())
        except (ValueError,TypeError) as e:self.access_state={'loaded':True,'software':{'ok':False,'detail':'读取检查记录失败：'+str(e)}}
        self.access_probe=None;self.access_checked_at=time.monotonic();process.deleteLater();self.refresh()
    def poll_access(self):
        if self.preview or self.pages.currentIndex()!=2 or self.job is not None or time.monotonic()-self.access_polled<3:return
        self.access_polled=time.monotonic()
        paths=['artifacts/hardware_readiness.json','artifacts/hardware_preflight_current.json','artifacts/quest_commissioning_fake/result.json','artifacts/quest_commissioning_hardware/result.json','artifacts/ros_smoke_pass.json','artifacts/smooth_controller_ros_test/result.json','artifacts/fixed_servo_ros_test/pure_servo/result.json','artifacts/fixed_servo_ros_test/auto_replan/result.json']
        signature=[]
        for name in paths:
            try:st=(ROOT/name).stat();signature.append((st.st_mtime_ns,st.st_size))
            except OSError:signature.append(None)
        if signature!=self.access_signature or time.monotonic()-self.access_checked_at>30:self.access_signature=signature;self.request_access_probe()
        self.refresh()
    def access_action(self):
        state=self.access_state
        if self.job is not None or self.auth is not None or self.arm_control.busy:return 'busy','等待当前操作结束'
        if not state.get('loaded'):return 'loading','读取接入状态…'
        if not self.device_boxes['arm'].isChecked() or not self.device_boxes['quest'].isChecked():return 'devices','选择机械臂和 Quest'
        if not state.get('software',{}).get('ok'):return 'software','软件检查未通过 · 查看原因'
        accepted=state.get('accepted',{}).get('ok')
        if not accepted and not state.get('fake',{}).get('ok'):return 'fake','下一步：运行 60 秒仿真检查'
        if resource.getrlimit(resource.RLIMIT_RTPRIO)[0]<50:return 'permission','下一步：启用本次实时权限'
        if not self.site.isChecked():return 'site','下一步：确认右侧现场准备'
        if accepted:return 'ready','返回遥操作'
        if not state.get('hardware',{}).get('ok'):return 'hardware','下一步：运行真机接入检查'
        if not state.get('accepted',{}).get('ok'):return 'accept','下一步：确认并保存实机结果'
        return 'ready','返回遥操作'
    def refresh_access(self,busy,hardware):
        if not hasattr(self,'access_next'):return
        rt=resource.getrlimit(resource.RLIMIT_RTPRIO)[0]>=50
        hint=''
        if hardware and not busy:
            if not rt:hint='启动前需启用本次实时权限。'
            elif not self.site.isChecked():hint='请确认现场准备，再启动遥操作。'
            else:hint='启动时自动核对工具、负载和机械臂反馈。'
        self.access_hint.setText(hint);self.access_hint.setVisible(bool(hint))
        if self.pages.currentIndex()!=2:return
        state=self.access_state;accepted=state.get('accepted',{}).get('ok')
        self.check_label.setText('本版本的开发验收已保存' if accepted else '本版本暂无已确认的开发验收')
        names={'software':'软件检查','fake':'Quest 仿真检查','preflight':'实机状态与反馈','hardware':'真机检查','accepted':'现场结果确认'}
        for name,title in names.items():
            item=state.get(name,{});self.access_lines[name].setText(('✓ ' if item.get('ok') else '○ ')+title+'：'+('通过' if item.get('ok') else '待完成' if state.get('loaded') else '读取中'))
            self.access_lines[name].setToolTip(item.get('detail',''))
            if accepted and name in ('fake','hardware'):
                self.access_lines[name].setText('✓ '+title+'：已保存验收，无需重复')
            if accepted and name=='preflight' and not item.get('ok'):
                self.access_lines[name].setText('○ '+title+'：运行开发检查时更新')
        self.access_lines['permission'].setText(('✓ 本次实时权限：已启用' if rt else '○ 本次实时权限：未启用，需本机系统认证'))
        self.access_lines['site'].setText('✓ 现场准备：已确认' if self.site.isChecked() else '○ 现场准备：请勾选右侧确认框')
        action,text=self.access_action();self.access_next.setText(text);self.access_next.setEnabled(action not in ('busy','loading'))
        details=[]
        if action in ('software','fake','hardware'):details.append(state.get(action,{}).get('detail',''))
        if action=='fake':details.append('这是带 Quest 手柄输入的仿真检查；刚才的普通仿真遥操作不替代本项检查。')
        if action=='permission':details.append('权限只对当前上位机进程生效，关闭或重开后需再次认证。')
        if action=='site':details.append('工作区清空、可立即按物理急停、右侧 Grip 完全松开。')
        if action=='accept':details.append('下方勾选你实际观察到的方向与两种停止结果，再保存。')
        self.access_detail.setText('\n'.join(details))
        if state.get('loaded'):
            self.check_summary.setText('当前仿真检查：'+('通过' if state.get('fake',{}).get('ok') else '待完成 / 需更新')+'\n当前真机检查：'+('通过' if state.get('hardware',{}).get('ok') else '待完成'))
            if accepted:self.check_summary.setText('本版本的现场跟随与停止记录已保存。')
        self.accept.setEnabled(not busy and self.auth is None and state.get('hardware',{}).get('ok',False) and self.observed.isChecked())
        self.access_cancel.setEnabled(not busy and self.auth is None)
    def primary_start(self):
        if self.job is not None or self.auth is not None or self.arm_control.busy:return
        if not (self.device_boxes['arm'].isChecked() and self.device_boxes['quest'].isChecked()):
            self.message('遥操作需要选用机械臂和 Quest。');return
        if self.mode.currentIndex()==1:
            if not self.site.isChecked():
                self.site.setFocus();self.message('请勾选“现场准备已完成”，再启动真机遥操作。');return
            if resource.getrlimit(resource.RLIMIT_RTPRIO)[0]<50:
                self.start_after_auth=True
                self.authorize();return
        self.launch('teleop')
    def advance_access(self):
        action,_=self.access_action()
        if action=='devices':self.set_page(1)
        elif action=='software':self.message(self.access_state.get('software',{}).get('detail','软件检查未通过'))
        elif action=='fake':self.access_flow=True;self.mode.setCurrentIndex(0);self.launch('commission');self.set_page(0)
        elif action=='permission':self.mode.setCurrentIndex(1);self.authorize()
        elif action=='site':self.mode.setCurrentIndex(1);self.site.setFocus();self.message('确认现场准备后，在检查页点击“下一步”继续开发验收。')
        elif action=='hardware':self.access_flow=True;self.mode.setCurrentIndex(1);self.launch('commission');self.set_page(0)
        elif action=='accept':
            if self.observed.isChecked():self.accept_result()
            else:self.observed.setFocus();self.pages.widget(2).ensureWidgetVisible(self.observed);self.message('请勾选你在现场实际观察到的跟随与停止结果，再保存。')
        elif action=='ready':self.set_page(0)
    def session_finished(self,code):
        if self.hand_teleop:self.stop_grasp(notify=False)
        self.hand_teleop=False;self.using_hardware_teleop=False;self.hand_waiting_trigger=False
        if self.session_error:message=self.session_error
        elif self.session_terminal:message=self.session_terminal
        elif code:message='会话未通过，具体原因见运行日志。'
        else:message='会话已结束。'
        if self.session_cleanup_warning and '退出异常' not in message:message+=' 部分组件退出异常，详见运行日志。'
        self.message(message)
        if self.access_flow:self.set_page(2);self.access_flow=False
        self.request_access_probe();self.refresh()
    def stop_access_probe(self):
        if self.access_probe is not None:
            process=self.access_probe;self.access_probe=None;process.finished.disconnect();process.terminate();process.waitForFinished(2000);process.deleteLater()
