"""DH116 workspace widgets; no transport or motor operations on construction."""
import csv,time
from collections import deque
from PyQt5 import QtCore,QtGui,QtWidgets as W
from grasp_profiles import AXES, FINGERS
from operator_view import frame,label,CurrentSizeTabs
from operator_tactile import ASSETS
from operator_settings import ROOT

ICONS=('thumb_abd','thumb_flex','index_flex','middle_flex','ring_flex','pinky_flex')

def axis_label(index):
    widget=W.QWidget();widget.setObjectName('handAxis');row=W.QHBoxLayout(widget);row.setContentsMargins(3,0,3,0);row.setSpacing(7)
    icon=W.QLabel();icon.setPixmap(QtGui.QPixmap(str(ASSETS/(ICONS[index]+'.png'))).scaled(30,30,QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation));row.addWidget(icon)
    row.addWidget(label(AXES[index]));row.addStretch();return widget

def table(headers,rows=6):
    t=W.QTableWidget(rows,len(headers));t.setHorizontalHeaderLabels(headers);t.verticalHeader().hide();t.setAlternatingRowColors(False);t.setShowGrid(False)
    t.setSelectionBehavior(W.QAbstractItemView.SelectRows);t.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
    t.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch);t.horizontalHeader().setMinimumSectionSize(58)
    t.verticalHeader().setDefaultSectionSize(34);t.setMinimumHeight(242);return t

class HandLog(W.QWidget):
    def __init__(self,parent=None):
        super().__init__(parent);self.records=deque(maxlen=2000);self.filters={}
        box=W.QVBoxLayout(self);box.setContentsMargins(10,10,10,8);row=W.QHBoxLayout()
        for key,name in [('debug','调试'),('info','信息'),('warning','警告'),('error','错误')]:
            c=W.QCheckBox(name);c.setChecked(key!='debug');c.toggled.connect(self.render);self.filters[key]=c;row.addWidget(c)
        row.addStretch();self.export=W.QPushButton('导出日志');self.export.clicked.connect(self.export_log);row.addWidget(self.export)
        self.clear=W.QPushButton('清空显示');self.clear.clicked.connect(self.clear_view);row.addWidget(self.clear);box.addLayout(row)
        self.table=table(['时间','级别','内容'],0);self.table.setMinimumHeight(190);self.table.horizontalHeader().setSectionResizeMode(0,W.QHeaderView.ResizeToContents);self.table.horizontalHeader().setSectionResizeMode(1,W.QHeaderView.ResizeToContents);box.addWidget(self.table)
    def append(self,level,message):
        old=self.records[0] if len(self.records)==self.records.maxlen else None
        self.records.append((time.strftime('%H:%M:%S'),level,str(message)))
        if old and self.filters[old[1]].isChecked():self.table.removeRow(0)
        if self.filters[level].isChecked():
            self.add_row(self.records[-1]);self.table.scrollToBottom()
    def activity(self,event):
        kind=event.get('event');level='info';message=''
        if kind=='error':level='error';message=event.get('detail','设备返回错误')
        elif kind=='rejected_status':level='error';message=f"关节 {event.get('axis')} 状态异常：{event.get('status')}"
        elif kind=='ready':message='连接检查通过 · '+event.get('mode','')
        elif kind=='armed':message='关节已使能 · '+event.get('mode','Trigger')
        elif kind=='homing':message='已发送厂家六轴归零指令，等待关节反馈'
        elif kind=='finger_contact':
            finger=FINGERS[event['finger']];message=(f"{finger}达到 {event['limit_n']:g} N，已停止继续闭合" if event['held'] else f"{finger}开始张开")
        elif kind=='step':message='逐指验收 · 目标 '+str(event.get('targets'))
        elif kind=='manual_target':message=f"关节 {event.get('axis')} → {event.get('target')} eq；速度 {event.get('velocity')} eq/s，电流 {event.get('current')}‰"
        elif kind=='closed':
            level='info' if event.get('init_confirmed') else 'warning';message='会话结束 · '+('INIT 已确认' if event.get('init_confirmed') else 'INIT 未确认')
        elif kind=='rpdo_queued':level='debug';message='发送 RPDO '+event.get('raw_hex','')[:28]
        elif kind=='preparation_stop_pending':level='debug';message='等待准备阶段停止状态释放'
        if message:self.append(level,message)
    def render(self):
        if not hasattr(self,'table'):return
        self.table.setRowCount(0)
        for record in self.records:
            if self.filters[record[1]].isChecked():self.add_row(record)
        self.table.scrollToBottom()
    def add_row(self,record):
        stamp,level,message=record;i=self.table.rowCount();self.table.insertRow(i)
        names={'debug':'调试','info':'信息','warning':'警告','error':'错误'}
        for j,value in enumerate((stamp,names[level],message)):
            item=W.QTableWidgetItem(value)
            if level in ('warning','error'):item.setForeground(QtGui.QColor('#d17b11' if level=='warning' else '#e25353'))
            self.table.setItem(i,j,item)
    def clear_view(self):self.records.clear();self.render()
    def export_log(self):
        path,_=W.QFileDialog.getSaveFileName(self,'导出关节日志',str(ROOT/'artifacts'/('hand_log_'+time.strftime('%Y%m%d_%H%M%S')+'.csv')),'CSV (*.csv)')
        if not path:return
        try:
            with open(path,'w',encoding='utf-8-sig',newline='') as stream:writer=csv.writer(stream);writer.writerow(['时间','级别','内容']);writer.writerows(self.records)
            self.append('info','日志已导出：'+path)
        except OSError as e:self.append('error','导出失败：'+str(e))

def build_workspace(o):
    page,layout=o.page_base('灵巧手抓握','DH116 右手 · 连接后可拖动手指或使用右扳机，五指触觉持续显示');page.setObjectName('handWorkspace');layout.setSpacing(12)
    toolbar=W.QHBoxLayout();o.grasp_mode=W.QComboBox();o.grasp_mode.addItems(['预览 · 不驱动手指','hardware · DH116 实物']);toolbar.addWidget(o.grasp_mode)
    toolbar.addStretch()
    o.grasp_monitor=W.QPushButton('连接');o.grasp_monitor.setToolTip('读取六轴与触觉，已归零时同步当前位置并使能手动控制。');o.grasp_monitor.clicked.connect(lambda:o.request_grasp_mode('connect'));toolbar.addWidget(o.grasp_monitor)
    o.grasp_home=W.QPushButton('强制归零');o.grasp_home.setToolTip('确认后先停住手指，再使用手内保存的参数重找六轴零位。');o.grasp_home.clicked.connect(o.confirm_homing);toolbar.addWidget(o.grasp_home)
    o.grasp_zero_all=W.QPushButton('全部定位回零');o.grasp_zero_all.setToolTip('按现有零位将六轴定位到 0，沿用方案的速度与电流限制。');o.grasp_zero_all.clicked.connect(o.zero_all_joints);toolbar.addWidget(o.grasp_zero_all)
    o.grasp_manual_enable=W.QPushButton('手动关节');o.grasp_manual_enable.setObjectName('primary');o.grasp_manual_enable.clicked.connect(lambda:o.request_grasp_mode('manual'));toolbar.addWidget(o.grasp_manual_enable)
    o.grasp_manual_disable=W.QPushButton('去使能');o.grasp_manual_disable.clicked.connect(lambda:o.request_grasp_mode('monitor'));toolbar.addWidget(o.grasp_manual_disable)
    o.grasp_stop=W.QPushButton('断开连接');o.grasp_stop.setObjectName('stop');o.grasp_stop.clicked.connect(o.stop_grasp);toolbar.addWidget(o.grasp_stop)
    o.grasp_all_stop=W.QPushButton('全部停止');o.grasp_all_stop.setObjectName('stop');o.grasp_all_stop.setToolTip('停止灵巧手、机械臂遥操作、只读 DH116 会话和设备反馈模块');o.grasp_all_stop.clicked.connect(o.stop_session);toolbar.addWidget(o.grasp_all_stop);layout.addLayout(toolbar)
    status_row=W.QHBoxLayout();o.grasp_connection_badge=label('未连接','chip');status_row.addWidget(o.grasp_connection_badge)
    o.grasp_status=label('预览模式 · 尚未连接灵巧手','muted');o.grasp_status.setWordWrap(True);status_row.addWidget(o.grasp_status,1);layout.addLayout(status_row)
    from operator_hand_model import HandModel
    split=W.QSplitter(QtCore.Qt.Horizontal);split.setChildrenCollapsible(False);split.setHandleWidth(12);layout.addWidget(split,1)
    left,box=frame();box.setContentsMargins(12,12,12,12);o.hand_model=HandModel();box.addWidget(o.hand_model);split.addWidget(left)
    forces=W.QHBoxLayout();o.finger_force_labels=[]
    for name in FINGERS:
        item=label(name+'\n— N','muted');item.setAlignment(QtCore.Qt.AlignCenter);forces.addWidget(item,1);o.finger_force_labels.append(item)
    box.addLayout(forces)
    o.hand_model.targetChanged.connect(o.apply_model_joint)
    o.hand_model.previewChanged.connect(o.preview_model_joint)
    right,box=frame();box.setContentsMargins(14,12,14,12);o.hand_control_tabs=CurrentSizeTabs();o.hand_control_tabs.setObjectName('handTabs');box.addWidget(o.hand_control_tabs);split.addWidget(right);split.setStretchFactor(0,5);split.setStretchFactor(1,6)
    controls=W.QWidget();controls.setObjectName('handTabPage');c=W.QVBoxLayout(controls);c.setContentsMargins(4,9,4,0);c.setSpacing(4)
    o.manual_targets=[];o.manual_feedback=[]
    for i,name in enumerate(AXES):
        row=W.QHBoxLayout();name_label=axis_label(i);name_label.setFixedWidth(112);row.addWidget(name_label)
        value=W.QSpinBox();value.setRange(0,10000);value.setKeyboardTracking(False);value.setFixedWidth(83);row.addWidget(value)
        slider=W.QSlider(QtCore.Qt.Horizontal);slider.setRange(0,10000);slider.valueChanged.connect(value.setValue);slider.sliderMoved.connect(lambda target,axis=i:o.queue_manual_joint(axis,target));value.valueChanged.connect(slider.setValue);value.valueChanged.connect(o.update_hand_preview)
        slider.sliderReleased.connect(lambda axis=i:o.apply_manual_joint(axis));value.editingFinished.connect(lambda axis=i:o.apply_manual_joint(axis));row.addWidget(slider,1)
        actual=label('实际 —','muted');actual.setWordWrap(False);actual.setMinimumWidth(58);row.addWidget(actual);o.manual_feedback.append(actual);c.addLayout(row);c.addSpacing(4);o.manual_targets.append((value,slider))
    row=W.QHBoxLayout();o.manual_axis=W.QComboBox();o.manual_axis.addItems(AXES);row.addWidget(o.manual_axis,1);o.grasp_zero_one=W.QPushButton('单轴定位回零');o.grasp_zero_one.setToolTip('选中关节定位到 0，不改变零位标定。');o.grasp_zero_one.clicked.connect(lambda:o.zero_joint(o.manual_axis.currentIndex()));row.addWidget(o.grasp_zero_one);c.addLayout(row)
    c.addWidget(label('实物模式拖动滑块会限速发送目标；松开后提交最终位置。预览模式只改变显示目标。','muted'));c.addStretch();o.hand_control_tabs.addTab(controls,'关节控制')
    from operator_grasp_editor import ActionEditor
    o.action_editor=ActionEditor(o);o.action_editor.preview.connect(o.preview_action)
    o.hand_control_tabs.addTab(o.action_editor,'抓握方案')
    o.hand_bottom_tabs=CurrentSizeTabs();o.hand_bottom_tabs.setObjectName('handTabs');layout.addWidget(o.hand_bottom_tabs)
    o.joint_monitor=table(['关节','位置 / eq','速度 / eq/s','电流 / ‰','报警','使能','状态'])
    for i in range(6):o.joint_monitor.setCellWidget(i,0,axis_label(i))
    o.hand_bottom_tabs.addTab(o.joint_monitor,'关节监控')
    config=W.QWidget();cfg=W.QVBoxLayout(config);cfg.setContentsMargins(8,8,8,8);o.joint_config=table(['关节','手动速度 / eq/s','手动电流 / ‰','最小 / 展开','最大 / 握合','Trigger 目标']);o.grasp_endpoints=[];o.manual_parameters=[]
    for i in range(6):
        o.joint_config.setCellWidget(i,0,axis_label(i));speed=W.QSpinBox();speed.setRange(100,20000);current=W.QSpinBox();current.setRange(1,900);lo=W.QSpinBox();hi=W.QSpinBox();lo.setRange(0,10000);hi.setRange(0,10000);target=label('—','muted');target.setAlignment(QtCore.Qt.AlignCenter)
        for j,w in enumerate((speed,current,lo,hi,target),1):o.joint_config.setCellWidget(i,j,w)
        o.grasp_endpoints.append((lo,hi,target));o.manual_parameters.append((speed,current))
        lo.valueChanged.connect(o.sync_joint_ranges);hi.valueChanged.connect(o.sync_joint_ranges)
    cfg.addWidget(o.joint_config);row=W.QHBoxLayout();o.grasp_accept=W.QPushButton('逐指检查');o.grasp_accept.clicked.connect(lambda:o.request_grasp_mode('acceptance_profile'));row.addWidget(o.grasp_accept);row.addWidget(label('参数保存在当前抓握方案；不写入驱动器标定。','muted'),1);save=W.QPushButton('保存关节配置');save.clicked.connect(o.save_grasp);row.addWidget(save);o.joint_config_save=save;cfg.addLayout(row);o.hand_bottom_tabs.addTab(config,'关节配置')
    o.hand_log=HandLog();o.hand_bottom_tabs.addTab(o.hand_log,'日志窗口');o.grasp.activity.connect(o.hand_log.activity)
    o.grasp_editors=[o.grasp_choice,o.grasp_load,o.grasp_save,o.grasp_name,o.grasp_force,o.grasp_current,o.action_editor,o.joint_config_save]+[w for row in o.grasp_endpoints for w in row[:2]]+[w for row in o.manual_parameters for w in row]
    o.grasp_choice.setCurrentIndex(o.grasps.data['active']);o.load_grasp();o.grasp_mode.currentIndexChanged.connect(o.refresh_grasp)
    o.grasp_timer=QtCore.QTimer(o);o.grasp_timer.timeout.connect(o.tick_grasp)
    if not o.preview:o.grasp_timer.start(15)
    o.hand_log.append('info','工作台已就绪 · 预览模式，不发送运动指令')
    refresh_monitor(o,{})
    return page

def refresh_monitor(o,state):
    for i in range(6):
        if not state:values=['—']*6
        else:
            s=state['axis_status'][i];run_state=s>>5
            position='正在归零' if run_state==7 else '未归零' if not s&16 else '力矩到位' if s&8 else '位置到位' if s&2 else '运动中' if s&1 else '待机'
            values=[str(state['raw_positions'][i]),str(state.get('velocities',['—']*6)[i]),str(state.get('currents',['—']*6)[i]),f'代码 {run_state}' if 2<=run_state<=6 else '无','已使能' if s&1 else '停用',position]
        o.manual_feedback[i].setText('实际 '+values[0])
        for j,v in enumerate(values,1):
            item=o.joint_monitor.item(i,j)
            if item is None:item=W.QTableWidgetItem();item.setTextAlignment(QtCore.Qt.AlignCenter);o.joint_monitor.setItem(i,j,item)
            if item.text()!=v:item.setText(v)
            item.setToolTip(f"原始状态 0x{state['axis_status'][i]:02X}" if state else '等待真实反馈')
    if state:
        axes=state['axis_status'];enabled=sum(bool(s&1) for s in axes)
        if not state.get('ok'):badge='已连接 · 状态异常'
        elif o.grasp.mode=='home' and o.grasp.process is not None:badge='已连接 · 正在归零'
        elif not state['homed']:badge='已连接 · 未归零'
        elif enabled==6:badge='已连接 · 六轴已使能'
        elif enabled:badge=f'已连接 · {enabled}/6 轴已使能'
        else:badge='已连接 · 已归零 · 未使能'
    elif o.grasp.process is None:badge='未连接'
    elif o.grasp.stopping:badge='正在断开'
    else:badge='等待反馈' if o.grasp.ready else '连接中'
    o.grasp_connection_badge.setText(badge)
