"""Grasp editor, independent Quest input and exclusive native hand process."""
import copy,json,socket,time
from datetime import datetime
from PyQt5 import QtCore,QtWidgets as W
from grasp_profiles import Presets,TriggerInput,AXES,FINGERS,action_words,defaults,targets,validate
from operator_settings import ROOT,atomic
from operator_hand import decode_sample
from operator_hand_access import native_command,connection_error
from operator_tactile import decode_tactile,fresh_tactile,HandForces
from operator_view import frame,label

def hardware_grasp_readiness(root=ROOT):
    try:
        motion=json.loads((root/'artifacts/grasp_acceptance_live.json').read_text())
        tactile=json.loads((root/'artifacts/grasp_monitor_live.json').read_text())
        if motion.get('hardware') is not True or motion.get('motion_passed') is not True:
            return False,'逐指开合实机验收未通过'
        if tactile.get('hardware') is not True or tactile.get('tactile_response_observed') is not True:
            return False,'五指触觉受力响应尚未确认'
        return True,'动作与触觉前置检查已通过；力上限仍需接触验收'
    except (OSError,ValueError,TypeError,AttributeError):
        return False,'需先完成逐指开合和五指触觉实机检查'


class GraspConnection(QtCore.QObject):
    changed=QtCore.pyqtSignal()
    activity=QtCore.pyqtSignal(object)
    tactile_sample=QtCore.pyqtSignal(object)
    home_completed=QtCore.pyqtSignal()
    def __init__(self,parent=None):
        super().__init__(parent);self.process=None;self.state={};self.tactile={};self.detail='预览模式';self.mode='preview';self.buffer='';self.log=None;self.closed=False;self.stopping=False;self.sequence=0;self.tactile_sequence=0;self.manual_sequence=0;self.armed=False;self.ready=False;self.result={};self.last_publish=0.;self.failure=''
        self.persistent=False;self.profile=None;self.initial_source='manual';self.action_phase=0.
    def start(self,interface,mode,profile):
        if self.process is not None:raise ValueError('请先断开当前抓握会话')
        profile=copy.deepcopy(profile)
        if mode=='manual':
            profile['velocity']=max(profile.get('manual_velocity',[profile['velocity']]*6));profile['current_per_mille']=max(profile.get('manual_current',[profile['current_per_mille']]*6))
        elif mode=='open':
            # All-open uses one common setting; respect the lowest configured axis limit.
            profile['velocity']=min([profile['velocity']]+profile.get('manual_velocity',[]));profile['current_per_mille']=min([profile['current_per_mille']]+profile.get('manual_current',[]))
        if mode=='control':
            ready,reason=hardware_grasp_readiness()
            if not ready:raise ValueError(reason)
        if not profile['force_limit_enabled']:raise ValueError('实物模式必须启用触觉力上限')
        if (mode!='workspace' and profile['velocity']>2000) or profile['current_per_mille']>900:raise ValueError('独立诊断速度上限为 2000 当量/秒，实物电流上限为 900‰。')
        if any(a>b for a,b in zip(profile['open'],profile['closed'])):raise ValueError('闭合目标应不小于展开位置')
        binary=ROOT/'.cache/dh116/build/v3_dh116_grasp'
        if not binary.is_file():raise ValueError('请先构建 DH116 抓握组件')
        if not interface:raise ValueError('未找到有线网卡，请在设备连接页选择灵巧手所在网口。')
        folder=ROOT/'artifacts/grasp_sessions'/datetime.now().strftime('%Y%m%dT%H%M%S_%f');folder.mkdir(parents=True,exist_ok=False)
        self.log=(folder/'capture.jsonl').open('w');atomic(folder/'profile.json',dict(mode=mode,interface=interface,profile=profile))
        self.ready=False;self.armed=False;self.manual_sequence=0;self.result={};self.mode=mode;self.state={};self.tactile={};self.sequence=0;self.tactile_sequence=0;self.buffer='';self.closed=False;self.stopping=False;self.last_publish=0.;self.failure='';self.detail='正在连接并检查设备反馈'
        self.persistent=mode=='workspace';self.profile=profile
        self.process=QtCore.QProcess(self);self.process.setWorkingDirectory(str(ROOT));self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.consume);self.process.finished.connect(self.finished)
        self.process.errorOccurred.connect(lambda e:self.finished(-1) if e==QtCore.QProcess.FailedToStart else None)
        args=['--hardware',interface,mode,str(profile['velocity']),str(profile['current_per_mille']),str(profile['normal_force_limit_n'])]+[str(v) for v in profile['open']+profile['closed']]
        if self.persistent:args[3]='100'  # Inactive bootstrap; validated P supplies the actual profile before enable.
        if mode=='manual':args += [str(v) for v in profile.get('manual_velocity',[profile['velocity']]*6)+profile.get('manual_current',[profile['current_per_mille']]*6)]
        command,args=native_command(binary,args)
        if command=='/usr/bin/pkexec':self.detail='等待系统认证；免密码连接需安装 DH116 网口权限'
        self.process.start(command,args);self.changed.emit()
    def consume(self):
        if self.process is None:return
        text=bytes(self.process.readAllStandardOutput()).decode(errors='replace');self.log.write(text);self.log.flush();lines=(self.buffer+text).split('\n');self.buffer=lines.pop()
        old_detail=self.detail
        for line in lines:
            try:
                d=json.loads(line);event=d.get('event')
                if event in ('sample','tactile'):
                    # A GUI repaint can delay a queued frame. Only publish current readback.
                    # The native acquisition watchdog continues to supervise the bus.
                    if time.monotonic()-d['received_monotonic']>=.08:continue
                if event=='sample':self.state=decode_sample(d,time.monotonic(),self.sequence);self.sequence=d['sequence'];self.state.update({k:d[k] for k in ('currents','velocities','torque_reached','position_reached')})
                elif event=='tactile':self.tactile=decode_tactile(d,time.monotonic(),self.tactile_sequence);self.tactile_sequence=d['sequence'];self.tactile_sample.emit(self.tactile)
                elif event=='ready':
                    self.ready=True;self.detail={'monitor':'六轴和触觉反馈已连接','control':'检查通过 · 等待松开 Trigger','manual':'正在确认关节使能','home':'正在准备六轴归零','open':'五指展开恢复中'}.get(self.mode,'逐指握拳验收中')
                    if self.persistent:self.select_source(self.initial_source,self.profile)
                elif event=='action_progress':self.action_phase=d['phase']
                elif event=='workspace':
                    self.armed=d['armed'];self.mode={'disabled':'monitor','manual':'manual','trigger':'control','home':'home'}[d['source']]
                    if self.mode!='control':self.action_phase=0.
                    self.detail={'homing_required':'已连接 · 六轴尚未归零','force_limit':'已去使能 · 触及整手保护上限','torque_limit':'已去使能 · 达到关节力矩限制',
                                 'manual_paused':'已停止运动 · 拖动关节可继续','trigger_lost':'已停止跟随 · 请松开扳机后再握','nonzero_open_limit':'六轴最小位置均为 0 时才能完全张开','opening':'已提交六轴定位回零',
                                 'axis_disabled':'关节已去使能 · 请检查设备状态','homed':'六轴归零完成'}.get(d.get('reason'),{'monitor':'已连接 · 触觉在线','manual':'已连接 · 可拖动关节或扣右扳机','control':'已连接 · 右扳机控制','home':'六轴正在归零'}[self.mode])
                elif event=='homing':self.detail='六轴正在寻找零位，可以点击“停止 / 断开”中止'
                elif event=='armed':self.armed=True;self.detail='手动关节已使能' if self.mode=='manual' else '实物 trigger 控制中'
                elif event=='step':self.detail='验收：'+{-1:'先确认展开',0:'大拇指握拳',1:'食指握拳',2:'中指握拳',3:'无名指握拳',4:'小拇指握拳',5:'五指同时展开'}[d['step']]
                elif event=='error':
                    reason=d.get('detail','抓握失败')
                    if self.stopping and reason=='Stop or owner disappeared' and not self.failure:
                        self.detail='正在停止 DH116';d=dict(event='stopping')
                    else:
                        self.failure=self.failure or connection_error(reason);self.detail=self.failure;d=dict(d,detail=self.failure)
                elif event=='closed':
                    self.result=d
                    self.closed=d.get('init_confirmed') is True
                    if self.mode=='acceptance':self.detail=('逐指开合位置验收通过' if d.get('motion_passed') else '逐指握拳验收未通过 · '+self.detail)
                    elif self.mode=='acceptance_profile':self.detail=('指定目标逐指开合验收通过' if d.get('profile_motion_passed') else '指定目标验收未通过 · '+self.detail)
                    elif self.mode=='open':self.detail=('五指展开恢复通过' if d.get('open_passed') else '展开恢复未通过 · '+self.detail)
                    elif self.mode=='home':self.detail=self.failure or ('归零已中止' if self.stopping else '等待确认六轴归零结果')
                    elif self.mode=='monitor' and self.stopping:self.detail=self.failure or '已断开 DH116'
                    self.detail+=' · '+('INIT 已确认' if self.closed else 'INIT 未确认')
                if event not in ('sample','tactile','action_progress'):self.activity.emit(d)
            except (ValueError,TypeError,KeyError,IndexError) as e:
                if line.startswith('{'):
                    self.failure='反馈校验失败：'+str(e);self.detail=self.failure;self.tactile={};self.stop()
        now=time.monotonic()
        if now-self.last_publish>=.033 or old_detail!=self.detail:
            self.last_publish=now
            atomic(ROOT/'.cache/operator/grasp_state.json',dict(**self.state,tactile=self.tactile));self.changed.emit()
    def send_input(self,sequence,stamp,trigger,valid):
        if self.process is not None and (self.mode=='control' or (self.persistent and self.mode=='manual')) and self.ready and not self.stopping:
            if self.process.bytesToWrite()>1024:
                if not self.persistent:self.stop()
                return
            self.process.write(f'T {sequence} {stamp:.9f} {trigger:.6f} {int(valid)}\n'.encode())
    def select_source(self,source,profile=None):
        if not (self.persistent and self.ready and self.process is not None and not self.stopping):return False
        if source not in ('manual','trigger','disabled','home','open'):raise ValueError('无效控制来源')
        if source=='trigger' and any((profile or self.profile)['open']):
            raise ValueError('扳机松开时六轴需要定位到 0，请先核对方案中的展开下限。')
        line=''
        if profile is not None:
            line='P '+action_words(profile)+'\n'
        data=(line+'S '+source+'\n').encode()
        if self.process.write(data)!=len(data):return False
        if profile is not None:self.profile=copy.deepcopy(profile)
        self.armed=False
        # Suppress commands for the previous source until the native owner acknowledges.
        self.mode='switching'
        return True
    def manual_send(self,axis=None,target=None,velocity=None,current=None):
        if self.process is None or self.mode!='manual' or not self.ready or self.stopping:return False
        # Do not queue heartbeats while enable/parameter refresh is still running:
        # those packets could already exceed the 80 ms lease when the loop starts.
        if not self.armed and not (self.persistent and axis is not None):return False
        if self.process.bytesToWrite()>1024:
            if not self.persistent:self.stop()
            return False
        self.manual_sequence+=1;stamp=time.monotonic();line=f'H {self.manual_sequence} {stamp:.9f}' if axis is None else f'M {self.manual_sequence} {stamp:.9f} {axis} {target} {velocity} {current}'
        self.process.write((line+'\n').encode());return True
    def stop(self):
        if self.process is not None and not self.stopping:
            self.stopping=True;self.process.terminate();self.changed.emit()
    def finished(self,code,*_):
        if self.process is None:return
        self.consume();p=self.process;self.process=None;self.state={};self.tactile={};self.armed=False
        if not self.ready:self.detail+=' · '+(p.errorString() if code==-1 else f'连接退出码 {code}')
        p.deleteLater()
        if not self.closed:self.detail+=' · 进程结束，未确认 INIT'
        home_ok=(self.mode=='home' and code==0 and self.closed and not self.stopping and not self.failure and
                 self.result.get('homing_passed') is True and self.result.get('disabled') is True and
                 self.result.get('stop_rc')==0 and self.result.get('disable_rc')==0)
        if self.mode=='home':
            result='六轴已归零并去使能' if home_ok else self.failure or ('归零已中止' if self.stopping else '未确认六轴归零及去使能')
            self.detail=result+(' · INIT 已确认' if self.closed else ' · INIT 未确认')
        self.log.close();self.log=None;self.persistent=False;atomic(ROOT/'.cache/operator/grasp_state.json',{});self.changed.emit()
        if home_ok:self.home_completed.emit()

class GraspModule:
    def init_grasp(self):
        self.grasps=Presets();self.grasp=GraspConnection(self);self.grasp.changed.connect(self.refresh_grasp);self.trigger_policy=TriggerInput();self.grasp_socket=None;self.grasp_error='';self.grasp_loaded=0;self.grasp_loading=False;self.external_grasp=False;self.pending_grasp=None;self.pending_hand_mode=None;self.manual_synced=False;self.grasp_transition=0;self.pending_manual_target=None;self.last_manual_target=None;self.last_manual_target_at=0.;self.grasp_status_until=0.
        self.grasp.home_completed.connect(self.reconnect_after_home)
        self.grasp.activity.connect(self.grasp_transition_event)
        self.hand_teleop=False;self.using_hardware_teleop=False;self.preview_phase=0.;self.preview_phase_at=0.
    def grasp_transition_event(self,event):
        if event.get('event')=='workspace':
            self.pending_manual_target=None;self.last_manual_target=None;self.manual_synced=False
            if event.get('reason')=='opening':
                for spin,_ in self.manual_targets:spin.setValue(0)
                self.manual_synced=True
    def reconnect_after_home(self):
        generation=self.grasp_transition
        QtCore.QTimer.singleShot(0,lambda:self.request_grasp_mode('monitor') if generation==self.grasp_transition else None)
    def hand_feedback(self):
        state=self.grasp.state if self.grasp.process is not None else self.hand.state
        if state.get('feedback_verified') and 0<=time.monotonic()-state.get('updated',0)<.08:return state
        return {}
    def confirm_homing(self):
        if not self.grasp_home.isEnabled():return
        choice=W.QMessageBox.question(self,'DH116 六轴强制归零',
            '六个关节将按手内保存的厂家参数运动并重新寻找零位。请取下抓取物，让手指活动范围保持空闲。现在开始归零吗？',
            W.QMessageBox.Yes|W.QMessageBox.Cancel,W.QMessageBox.Cancel)
        if choice==W.QMessageBox.Yes:self.request_grasp_mode('home')
    def grasp_notice(self,message,seconds=4):
        self.grasp_status.setText(message);self.grasp_status_until=time.monotonic()+seconds
    def grasp_page(self):
        from operator_hand_workspace import build_workspace
        return build_workspace(self)
    def draft_grasp(self):
        p=self.grasps.profile;p.update(name=self.grasp_name.text().strip(),normal_force_limit_n=self.grasp_force.value(),force_limit_enabled=True,current_per_mille=self.grasp_current.value(),open=[r[0].value() for r in self.grasp_endpoints],closed=[r[1].value() for r in self.grasp_endpoints],manual_velocity=[r[0].value() for r in self.manual_parameters],manual_current=[r[1].value() for r in self.manual_parameters],**self.action_editor.values());return p
    def session_profile(self,mode):
        if mode in ('monitor','home'):
            profile=defaults()['profiles'][0]
            if mode=='home':profile['normal_force_limit_n']=self.grasp_force.value()
            return profile
        profile=self.draft_grasp();data=copy.deepcopy(self.grasps.data);data['profiles'][data['active']]=profile;validate(data)
        if any(a>b for a,b in zip(profile['open'],profile['closed'])):raise ValueError('最小位置不能大于最大位置')
        return profile
    def load_grasp(self):
        if self.grasp.process is not None and not self.grasp.persistent:return
        self.grasp_loading=True;self.grasps.data['active']=self.grasp_choice.currentIndex();p=self.grasps.profile;self.grasp_loaded=self.grasp_choice.currentIndex()
        self.grasp_name.setText(p['name']);self.grasp_force.setValue(p['normal_force_limit_n']);self.grasp_current.setValue(p['current_per_mille'])
        for (a,b,_),lo,hi in zip(self.grasp_endpoints,p['open'],p['closed']):a.setValue(lo);b.setValue(hi)
        for i,(speed,current) in enumerate(self.manual_parameters):speed.setValue(p.get('manual_velocity',[p['velocity']]*6)[i]);current.setValue(p.get('manual_current',[p['current_per_mille']]*6)[i])
        self.grasp_loading=False;self.action_editor.load(p);self.sync_joint_ranges()
        if hasattr(self,'grasp_status'):self.grasp_status.setText('已加载 '+p['name']+(' · 配置读取失败：'+self.grasps.error if self.grasps.error else ' · 点击保存并应用可更新连接中的动作' if self.grasp.persistent else ' · 尚未启动实物'))
    def save_grasp(self):
        if self.grasp.process is not None and not self.grasp.persistent:return
        before=copy.deepcopy(self.grasps.data)
        try:
            profile=self.draft_grasp();index=self.grasp_choice.currentIndex()
            self.grasps.data['profiles'][index]=profile;self.grasps.data['active']=index
            self.grasps.save()
        except (OSError,ValueError) as exc:
            self.grasps.data=before;self.grasp_notice('保存失败：'+str(exc));return
        self.grasp_loaded=index;self.hand_log.append('info','已保存抓握动作：'+profile['name'])
        if not self.grasp.persistent:self.grasp_notice('已保存 '+profile['name']);return
        source={'manual':'manual','control':'trigger'}.get(self.grasp.mode,'disabled')
        try:
            if source=='trigger':self.reset_grasp_input()
            if not self.grasp.select_source(source,profile):raise ValueError('连接正在切换或已经结束')
        except ValueError as exc:
            self.grasp_notice('方案已保存，当前设备未应用：'+str(exc));return
        self.last_manual_target=None;self.sync_joint_ranges();self.grasp_notice('已保存，正在应用 '+profile['name'])
    def reset_grasp_input(self):
        if self.grasp.process is not None and not self.grasp.persistent:return
        if self.grasp.persistent and self.grasp.mode=='control':self.grasp.process.write(b'X\n')
        if self.grasp_socket is not None:
            try:
                while True:self.grasp_socket.recvfrom(8192)
            except BlockingIOError:pass
        self.trigger_policy=TriggerInput();self.grasp_error='请先完全松开 Trigger'
    def sync_joint_ranges(self):
        if self.grasp_loading or not hasattr(self,'manual_targets'):return
        for axis,((lo,hi,_),(spin,slider)) in enumerate(zip(self.grasp_endpoints,self.manual_targets)):
            active=self.grasp.profile if self.grasp.persistent else None
            low,high=(active['open'][axis],active['closed'][axis]) if active else (lo.value(),hi.value())
            if low<=high:spin.setRange(low,high);slider.setRange(low,high)
        self.update_hand_preview()
        self.action_editor.select(self.action_editor.timeline.index)
    def preview_action(self,phase):
        values=targets(self.draft_grasp(),phase)
        if self.grasp_mode.currentIndex()==0:self.hand_model.set_values(values)
        for (lo,hi,view),v in zip(self.grasp_endpoints,values):view.setText(str(v))
    def manual_available(self):
        if self.grasp.mode!='manual' or self.grasp.stopping:return False
        if self.grasp.persistent:
            state=self.hand_feedback()
            return state.get('homed',False) and state.get('ok',False)
        return self.grasp.armed
    def manual_speed(self,axis):
        values=(self.grasp.profile if self.grasp.persistent else None) or self.draft_grasp()
        v=values.get('manual_velocity',[values['velocity']]*6)[axis]
        if self.grasp.persistent and v==values['velocity']:
            v=round(max(values.get('close_percent',v/200),values.get('open_percent',v/200))*200)
        return v,values.get('manual_current',[values['current_per_mille']]*6)[axis]
    def update_hand_preview(self,*unused):
        if self.grasp_loading or not hasattr(self,'hand_model') or len(self.manual_targets)!=6:return
        self.hand_model.low=[r[0].minimum() for r in self.manual_targets];self.hand_model.high=[r[0].maximum() for r in self.manual_targets]
        if self.grasp_mode.currentIndex()==0 and self.hand_control_tabs.currentWidget()!=self.action_editor:
            self.hand_model.set_values([r[0].value() for r in self.manual_targets])
    def preview_model_joint(self,axis,value):
        allowed=self.grasp_mode.currentIndex()==0 or (not self.preview and self.manual_available())
        if not allowed:return False
        self.manual_targets[axis][0].setValue(value);return True
    def apply_model_joint(self,axis,value):
        if self.preview_model_joint(axis,value):self.apply_manual_joint(axis)
    def apply_manual_joint(self,axis):
        if self.grasp_loading:return
        target=self.manual_targets[axis][0].value();speed,current=self.manual_speed(axis)
        if self.preview or self.grasp_mode.currentIndex()!=1:
            self.grasp_notice(f'预览 · {AXES[axis]} → {target} eq');return
        if self.grasp.process is None or not self.manual_available():
            self.grasp_notice('请先连接 DH116 并使能关节，当前目标尚未发送。');return
        self.pending_manual_target=None
        if (self.last_manual_target!=(axis,target) or not self.grasp.armed) and self.grasp.manual_send(axis+1,target,speed,current):
            self.last_manual_target=(axis,target);self.last_manual_target_at=time.monotonic()
        elif self.last_manual_target!=(axis,target):self.grasp_notice('目标未发送：请检查关节使能和实时反馈。')
    def queue_manual_joint(self,axis,target):
        if self.preview or self.grasp_mode.currentIndex()!=1:return
        if self.grasp.process is None or not self.manual_available():
            self.grasp_notice('请先连接 DH116 并使能关节，当前目标尚未发送。');return
        speed,current=self.manual_speed(axis)
        self.pending_manual_target=(axis,int(target),speed,current)
    def zero_joint(self,axis):
        if self.grasp_endpoints[axis][0].value()!=0:self.grasp_status.setText('该轴最小位置大于 0；请先核对关节配置。');return
        self.manual_targets[axis][0].setValue(0);self.apply_manual_joint(axis)
    def zero_all_joints(self):
        minima=self.grasp.profile['open'] if self.grasp.persistent else [row[0].value() for row in self.grasp_endpoints]
        if any(minima):self.grasp_notice('存在非零的最小位置，请核对关节配置。');return
        if self.grasp_mode.currentIndex()==0:
            for i in range(6):self.zero_joint(i)
        else:self.request_grasp_mode('open')
    def stop_grasp(self,*_,notify=True):
        self.hand_waiting_trigger=False;self.hand_teleop=False
        self.pending_grasp=None;self.pending_hand_mode=None;self.pending_manual_target=None;self.grasp_transition+=1
        if self.grasp.process is not None:
            self.grasp.stop()
            if notify:self.grasp_notice('正在停止 DH116 抓握并确认去使能 / INIT。')
        elif self.hand.process is not None:
            self.hand.disconnect_device()
            if notify:self.grasp_notice('正在安全断开 DH116 只读反馈连接。')
        elif notify:self.grasp_notice('当前没有运行中的 DH116 抓握或反馈会话。')
    def request_grasp_mode(self,mode):
        if self.preview or self.grasp_mode.currentIndex()!=1:self.grasp_notice('当前为预览模式；选择“hardware · DH116 实物”后再连接。');return
        if self.job is not None and not self.using_hardware_teleop:self.grasp_notice('当前机械臂会话未选择实物遥操作。');return
        if self.grasp.persistent and self.grasp.process is not None:
            if mode=='connect':
                if self.grasp.mode!='monitor':return
                mode='control' if self.using_hardware_teleop else 'manual'
            if mode=='workspace' or self.grasp.mode=='home':return
            if mode in ('control','manual','monitor','open','home'):
                self.hand_waiting_trigger=False
                try:
                    if mode=='control':
                        self.reset_grasp_input()
                        if not self.grasp.ready:
                            self.grasp.initial_source='trigger'
                            return
                    self.pending_manual_target=None;self.last_manual_target=None
                    source={'control':'trigger','monitor':'disabled'}.get(mode,mode)
                    self.grasp.select_source(source)
                    if self.using_hardware_teleop:self.hand_teleop=True
                except ValueError as exc:self.grasp_notice(str(exc))
                return
        if mode=='connect':
            self.grasp.initial_source='trigger' if self.using_hardware_teleop else 'manual';mode='workspace'
        elif mode=='monitor' and self.grasp.process is None:
            self.grasp.initial_source='disabled';mode='workspace'
        elif mode in ('manual','control') and self.grasp.process is None:
            self.grasp.initial_source='manual' if mode=='manual' else 'trigger';mode='workspace'
        if self.external_grasp:self.grasp_notice('另一项 DH116 实机检查正在运行，请等待其完成。');return
        state=self.hand_feedback()
        if mode!='monitor' and (self.hand.process is not None or self.grasp.process is not None) and not state.get('ok'):
            self.grasp_notice('正在等待有效的六轴反馈，当前连接会继续读取。');return
        if mode=='home' and not (state.get('ok') and all(s and not s&1 for s in state['axis_status'])):
            self.grasp_notice('请先连接 DH116，等六轴反馈在线且关节已去使能后再归零。');return
        if mode not in ('monitor','home','workspace') and state and not state['homed']:
            self.grasp_notice('六轴尚未全部归零。请先点击“强制归零”。');return
        try:
            p=self.session_profile(mode)
        except (ValueError,OSError) as e:self.grasp_notice(str(e));return
        prepared=(p,self.hand_interface.currentText())
        if self.hand.process is not None:
            if self.pending_hand_mode is not None:
                self.grasp_notice('正在安全断开只读 DH116 会话，请等待模式切换完成。');return
            self.grasp_transition+=1;self.pending_hand_mode=(mode,prepared,self.grasp_transition);self.hand.disconnect_device()
            self.grasp_notice('正在停止设备连接页的只读 DH116 会话并等待 EtherCAT INIT；完成后自动启动此模式。')
            return
        if self.grasp.process is not None:
            if self.grasp.mode==mode:
                if mode=='monitor':self.grasp_notice('DH116 只读连接已启动')
                return
            self.grasp_transition+=1
            self.pending_manual_target=None
            self.pending_grasp=(mode,prepared);self.grasp.stop();return
        self.grasp_transition+=1
        self.start_grasp(mode,prepared)
    def start_grasp(self,mode,prepared=None):
        if self.preview or self.grasp_mode.currentIndex()!=1 or self.hand.process is not None:return
        try:
            self.grasp_status_until=0.;self.pending_manual_target=None;self.last_manual_target=None;self.last_manual_target_at=0.
            p,interface=prepared or (self.session_profile(mode),self.hand_interface.currentText())
            self.reset_grasp_input();self.manual_synced=False;self.grasp.start(interface,mode,p)
            if mode=='workspace' and self.using_hardware_teleop:self.hand_teleop=True
        except (ValueError,OSError) as e:self.grasp_status.setText(str(e));self.hand_log.append('error',str(e))
        self.refresh_grasp()
    def refresh_grasp(self):
        if not hasattr(self,'grasp_status'):return
        busy=self.grasp.process is not None
        if self.pending_hand_mode is not None and self.hand.process is None:
            mode,prepared,generation=self.pending_hand_mode;self.pending_hand_mode=None
            if self.hand.closed:
                QtCore.QTimer.singleShot(0,lambda m=mode,p=prepared,g=generation:self.start_grasp(m,p) if g==self.grasp_transition else None)
            else:
                self.grasp_notice('只读 DH116 会话未确认返回 INIT；已取消启动实物抓握。')
                self.hand_log.append('error','反馈会话未确认 INIT，拒绝交接给抓握控制')
        if not busy and self.pending_grasp:
            request=self.pending_grasp;self.pending_grasp=None
            clean=self.grasp.closed and self.grasp.ready and (self.grasp.mode=='monitor' or (self.grasp.result.get('disabled') is True and self.grasp.result.get('stop_rc')==0 and self.grasp.result.get('disable_rc')==0))
            if clean and (request[0]=='monitor' or not self.grasp.failure):
                generation=self.grasp_transition
                QtCore.QTimer.singleShot(0,lambda r=request,g=generation:self.start_grasp(*r) if g==self.grasp_transition else None)
            else:self.hand_log.append('error','未确认安全断开，已取消模式切换')
        for w in self.grasp_editors:w.setEnabled(not busy or (self.grasp.persistent and self.grasp.ready and not self.grasp.stopping))
        self.grasp_mode.setEnabled(not busy)
        hardware=self.grasp_mode.currentIndex()==1 and not self.external_grasp and (self.job is None or self.using_hardware_teleop)
        allowed=not busy and hardware
        self.grasp_monitor.setEnabled(allowed or (hardware and self.grasp.persistent and self.grasp.mode=='monitor' and not self.grasp.stopping))
        self.grasp_monitor.setToolTip('连接后同步当前位置；已归零时可直接拖动关节。')
        if hasattr(self,'hand_interface'):
            self.hand_interface.setEnabled(not busy and self.hand.process is None and self.grasp.process is None and self.job is None)
            self.hand_connect.setEnabled(not busy and self.grasp.process is None and self.hand.process is None and self.job is None and self.device_boxes['hand'].isChecked() and self.hand_interface.count()>0)
            self.hand_disconnect.setEnabled(busy or self.hand.process is not None)
        self.grasp_stop.setEnabled(busy or self.hand.process is not None or self.pending_hand_mode is not None or self.pending_grasp is not None)
        monitor=busy and self.grasp.mode=='monitor' and self.grasp.ready and not self.grasp.stopping
        manual=busy and self.manual_available()
        state=self.hand_feedback()
        owner_online=busy or self.hand.process is not None
        workspace=busy and self.grasp.persistent and self.grasp.ready and not self.grasp.stopping and self.grasp.mode not in ('home','switching')
        can_move=(allowed or monitor or workspace) and (bool(state.get('ok') and state['homed']) if owner_online else True)
        self.grasp_home.setEnabled(hardware and (allowed or monitor or workspace) and bool(state.get('ok')) and (workspace or all(s and not s&1 for s in state['axis_status'])))
        self.grasp_manual_enable.setEnabled(can_move and not (manual and self.grasp.armed))
        self.grasp_manual_enable.setToolTip('切换为手动关节控制，保持触觉在线。')
        if state and not state['homed']:self.grasp_manual_enable.setToolTip('点击“强制归零”，确认六轴归零后即可使能。')
        self.grasp_enable.setEnabled(can_move and self.grasp.mode!='control');self.grasp_enable.setToolTip('右扳机松开时六轴展开，按住时跟随动作时间轴。')
        self.grasp_accept.setEnabled(can_move and not workspace and self.job is None)
        self.grasp_manual_disable.setEnabled(busy and self.grasp.mode!='monitor' and not self.grasp.stopping)
        self.tactile_start.setEnabled(allowed);self.tactile_stop.setEnabled(busy and not self.grasp.stopping)
        self.grasp_zero_all.setEnabled(self.grasp_mode.currentIndex()==0 or can_move or manual)
        self.grasp_zero_one.setEnabled(self.grasp_mode.currentIndex()==0 or manual)
        self.hand_model.set_allowed(self.grasp_mode.currentIndex()==0 or manual)
        self.update_hand_preview()
        for spin,slider in self.manual_targets:spin.setEnabled(self.grasp_mode.currentIndex()==0 or manual);slider.setEnabled(self.grasp_mode.currentIndex()==0 or manual)
        if manual and not self.manual_synced and self.grasp.state:
            self.grasp_loading=True
            for (spin,_),v in zip(self.manual_targets,self.grasp.state['raw_positions']):spin.setValue(v)
            self.grasp_loading=False;self.manual_synced=True
        if time.monotonic()>=self.grasp_status_until:
            if self.grasp.persistent:self.grasp_status.setText(self.grasp.detail)
            elif monitor and state:self.grasp_status.setText('可点击“使能关节”后拖动滑块' if state['homed'] else '模型暂保留预览姿态。请点击“强制归零”，完成后可使能关节。')
            elif hardware and not busy and state:self.grasp_status.setText('设备页已连接。点击“连接”可读取关节和触觉。' if state['homed'] else '设备页已连接，关节尚未全部归零。可点击“强制归零”。')
            elif self.grasp.detail!='预览模式':self.grasp_status.setText(self.grasp.detail)
    def tick_grasp(self):
        now=time.monotonic()
        self.grasp.manual_send()
        if self.pending_manual_target is not None and now-self.last_manual_target_at>=1/30:
            axis,target,speed,current=self.pending_manual_target;self.pending_manual_target=None
            if self.grasp.manual_send(axis+1,target,speed,current):self.last_manual_target=(axis,target);self.last_manual_target_at=now
            else:self.grasp_notice('目标未发送：请确认 DH116 已连接且关节使能。')
        if self.grasp_socket is None:
            try:self.grasp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.grasp_socket.bind(('127.0.0.1',5008));self.grasp_socket.setblocking(False)
            except OSError as e:
                if self.grasp_socket:self.grasp_socket.close()
                self.grasp_socket=None;self.grasp_input_label.setText('抓握输入不可用：'+str(e))
        try:
            for _ in range(64 if self.grasp_socket else 0):
                raw,_=self.grasp_socket.recvfrom(8192);stamp=time.monotonic()
                try:
                    packet=json.loads(raw);valid=self.trigger_policy.consume(packet,stamp);self.grasp_error='';self.grasp.send_input(packet['sequence'],stamp,self.trigger_policy.trigger,valid)
                except (ValueError,KeyError,TypeError,AttributeError) as e:
                    self.grasp_error=str(e)
                    if self.grasp.mode in ('control','manual') and self.grasp.persistent:self.grasp.process.write(b'X\n')
                    elif self.grasp.mode=='control':self.grasp.stop()
        except BlockingIOError:pass
        now=time.monotonic();valid=self.trigger_policy.valid(now);t=self.trigger_policy.trigger if valid else 0
        self.grasp_progress.setValue(round(t*100));self.grasp_input_label.setText('右 Trigger 已接通'+(' · 实物控制' if self.grasp.process and self.grasp.mode=='control' else ' · 预览目标') if valid else self.grasp_error or '等待新鲜输入和松开 Trigger；右摇杆停止后需重置')
        p=self.grasp.profile if self.grasp.persistent else self.draft_grasp()
        if valid and not self.action_editor.timer.isActive():
            values=targets(p,t)
            if self.grasp.mode=='control':
                self.action_editor.timeline.progress=self.grasp.action_phase;self.action_editor.timeline.update()
            if self.grasp_mode.currentIndex()==0 and self.hand_control_tabs.currentWidget()==self.action_editor and self.hand_model.dragging is None and self.action_editor.follow_input.isChecked():
                step=max(0.,now-self.preview_phase_at)*1000/p['duration_ms'] if self.preview_phase_at else 0.
                self.preview_phase=0 if t==0 else self.preview_phase+max(-step,min(t-self.preview_phase,step))
                self.preview_phase_at=now;self.action_editor.position(self.preview_phase)
            for row,value in zip(self.grasp_endpoints,values):row[2].setText(str(value))
        elif not valid and not self.action_editor.timer.isActive() and self.action_editor.follow_input.isChecked():
            self.preview_phase=0.;self.preview_phase_at=0.
            for row in self.grasp_endpoints:row[2].setText('—')
        state=self.grasp.state;tactile=self.grasp.tactile;external=False
        if self.grasp.process is None:
            state=self.hand.state
            try:
                cached=json.loads((ROOT/'.cache/operator/grasp_state.json').read_text()) if not self.preview or getattr(self,'snapshot_live',False) else {}
                if cached.get('ok') and 0<=now-cached.get('updated',0)<.08:
                    state=cached;tactile=cached.get('tactile',{});external=True
            except (OSError,ValueError,TypeError):pass
        if self.external_grasp!=external:
            self.external_grasp=external;self.refresh_grasp()
            if not external:
                qualified,reason=hardware_grasp_readiness();self.grasp_status.setText(reason)
        if external:self.grasp_status.setText('外部实机检查正在采集 · 本页显示真实反馈')
        if state and 0<=now-state.get('updated',0)<.08:self.media_states['hand']=state
        state=state if state and state.get('feedback_verified') and 0<=now-state.get('updated',0)<.08 else {}
        from operator_hand_workspace import refresh_monitor
        refresh_monitor(self,state)
        if self.grasp_mode.currentIndex()==1 and state and state['homed']:self.hand_model.set_values(state['sdk_positions'])
        tactile=fresh_tactile(tactile,now);self.grasp_tactile.set_state(tactile)
        for i,(name,w) in enumerate(zip(FINGERS,self.finger_force_labels)):
            force=f"{sum(tactile['fingers'][i]['normal_n']):.2f}" if tactile else '—'
            title=f'{name}\n{force} N'
            if w.text()!=title:w.setText(title)
    def close_grasp(self):
        self.grasp_timer.stop()
        if self.grasp_socket:self.grasp_socket.close();self.grasp_socket=None
