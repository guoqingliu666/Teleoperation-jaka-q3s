"""Operator studio layout and read-only instrument orchestration."""
import copy,json,math,os,signal,subprocess,time
import warnings
from scipy.spatial.transform import Rotation
from pathlib import Path
from PyQt5 import QtCore,QtGui,QtWidgets as W,QtSvg
from operator_view import WorkbenchView,label,frame,rule,STYLE
from operator_instruments import Scope,SceneWidget,CameraTile,CameraOverlay,SIGNAL_COLORS
from operator_settings import ROOT,RUNTIME,RATES,CAMERA_PRESETS,CAMERA_DEFAULTS,camera_preset,Preferences,atomic
from operator_media import current_state
from operator_hand import HandConnection
from operator_grasp import GraspModule
from operator_tactile import HandForces,TactileMonitor
from operator_target import current_target

class StudioView(GraspModule,WorkbenchView):
    def build_ui(self):
        self.hand=HandConnection(self);self.hand.changed.connect(self.refresh)
        self.init_grasp()
        self.prefs=Preferences();self.device_monitors={};self.media_states={};self.last_media=0.;self.last_telemetry=0.;self.last_plot_at=0.;self.quest_feedback_at=0.;self.quest_feedback_ok=False;self.scene_reset=0.;self.target_read_at=0.;self.device_boxes={};self.device_labels={};self.device_frames={};self.camera_mode_combos={};self.device_module_buttons={};self.profile_dirty=False
        super().build_ui();self.resize(1640,1040);self.setMinimumSize(1240,800)
        from operator_theme import ThemeSwitch
        self.brightness=ThemeSwitch(self.prefs.data['brightness']);self.theme_switch=self.brightness
        self.mode.parentWidget().layout().insertWidget(2,self.theme_switch)
        from operator_credits import InfoButton
        self.about_button=InfoButton();self.about_dialog=None;self.about_button.clicked.connect(self.show_about);self.mode.parentWidget().layout().addWidget(self.about_button)
        self.prefs.data['brightness']=self.theme_switch.value()
        self.theme_animation=QtCore.QVariantAnimation(self);self.theme_animation.setDuration(180);self.theme_animation.setEasingCurve(QtCore.QEasingCurve.InOutCubic)
        self.theme_animation.valueChanged.connect(self.set_brightness);self.theme_animation.finished.connect(self.save_preferences)
        self.theme_switch.valueChanged.connect(self.change_theme);self.load_profile();self.apply_theme();self.telemetry_timer=QtCore.QTimer(self);self.telemetry_timer.timeout.connect(self.studio_tick)
        if not self.preview:
            self.prefs.write_runtime();atomic(ROOT/'.cache/operator/scene_state.json',dict(fresh=False));self.scene.start();self.telemetry_timer.start(16)
    def teleop_page(self):
        page,layout=self.page_base('遥操作','实时场景  /  多视角监看  /  信号分析');layout.setSpacing(9)
        subtitle=layout.takeAt(1).widget();subtitle.hide()
        tooling,tools_box=frame();tools_box.setContentsMargins(12,8,12,8);tools_box.setSpacing(4)
        row=W.QHBoxLayout();self.tool_current=label('JAKA 当前工具：待读取','muted');row.addWidget(self.tool_current,1)
        self.tool_select=W.QComboBox();self.tool_select.setMinimumWidth(95)
        for i in range(11):self.tool_select.addItem('末端法兰中心' if i==0 else 'TCP'+str(i),i)
        self.tool_select.setCurrentIndex(1);row.addWidget(self.tool_select)
        self.tool_read=W.QPushButton('读取 TCP / 负载');self.tool_read.clicked.connect(self.open_tooling);row.addWidget(self.tool_read)
        self.tool_apply=W.QPushButton('应用工具');self.tool_apply.clicked.connect(lambda:self.arm_action('select_tool'));row.addWidget(self.tool_apply)
        self.payload_refresh=W.QPushButton('已校准，刷新负载');self.payload_refresh.clicked.connect(lambda:self.arm_action('refresh_payload'));row.addWidget(self.payload_refresh);tools_box.addLayout(row)
        self.payload_status=label('','muted');self.payload_status.hide()
        from operator_arm_widgets import ToolingPopover
        self.tooling_popover=ToolingPopover(self)
        self.tool_apply.setToolTip('静止且无控制会话时，切换 JAKA 已保存的工具号，并同步 TCP 定义到 V3。默认 TCP1。')
        self.payload_refresh.setToolTip('先在 JAKA 上位机重新检测并保存负载与质心，再点击此处读取。只更新 V3，不向控制柜写入负载。')
        layout.addWidget(tooling)
        stage,box=frame();box.setContentsMargins(12,12,12,12);box.setSpacing(8)
        row=W.QHBoxLayout();row.addWidget(label('3D 场景','section'));self.scene_status=label('模型预览 · 等待关节反馈','muted');self.scene_status.setWordWrap(False);self.scene_status.setSizePolicy(W.QSizePolicy.Ignored,W.QSizePolicy.Preferred);row.addWidget(self.scene_status,1);reset=W.QPushButton('复位视角');reset.setMinimumHeight(28);reset.clicked.connect(self.reset_scene);row.addWidget(reset);box.addLayout(row)
        self.scene=SceneWidget();self.scene.setMinimumHeight(420);self.scene.setToolTip('左键旋转 · 中键平移 · 滚轮缩放');box.addWidget(self.scene,1)
        self.scene.overlay=CameraOverlay(self.scene);self.camera_tiles={}
        for name,title,sub in [('zed_depth','腕部深度','0.2–2.0 m'),('zed_left','腕部左目','ZED-M'),('c922','全局视角','C922')]:
            tile=CameraTile(title,sub);tile.setParent(self.scene.overlay);self.camera_tiles[name]=tile;self.scene.overlay.tiles.append(tile);tile.activated.connect(lambda t=tile:self.scene.camera_focus.reveal(t));tile.frame_updated.connect(self.scene.camera_focus.sync)
        self.scene.place_overlay();self.hand_feedback_label=label('DH116：等待实际反馈；预览姿态不代表实际手指位置。','muted');box.addWidget(self.hand_feedback_label)
        self.target_position=label('手柄目标 · 基坐标 / mm     X —    Y —    Z —');box.addWidget(self.target_position)
        self.target_rotation=label('目标姿态 / °     Rx —    Ry —    Rz —');box.addWidget(self.target_rotation)
        self.target_hint=label('目标轴：红 X · 绿 Y · 蓝 Z　轨迹 5 s：新青 → 蓝 → 旧紫渐隐','muted');box.addWidget(self.target_hint)
        self.target_position.setToolTip('来自 /v3/hand_target：映射后的手柄要求 TCP 到达的目标，坐标系 base_link，位置单位 mm。')
        self.target_rotation.setToolTip('姿态轴随目标旋转。Rx / Ry / Rz 为固定轴 XYZ 欧拉角，R = Rz × Ry × Rx；数值在 ±180° 附近可能跳变。')
        for item in (self.target_position,self.target_rotation):item.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(stage,5)
        card,box=frame();self.teleop_monitor_card=card;card.setObjectName('teleopMonitor');card.setMinimumHeight(290);card.setMaximumHeight(310);box.setContentsMargins(14,10,14,12);box.setSpacing(6);row=W.QHBoxLayout()
        self.monitor_select=W.QButtonGroup(self);self.monitor_buttons=[]
        for index,title in enumerate(('信号示波器','触觉监控')):
            button=W.QPushButton(title);button.setObjectName('nav');button.setCheckable(True);self.monitor_select.addButton(button,index);row.addWidget(button);self.monitor_buttons.append(button)
        self.monitor_buttons[0].setChecked(True);row.addStretch()
        self.scope_controls=W.QWidget();controls=W.QHBoxLayout(self.scope_controls);controls.setContentsMargins(0,0,0,0);row.addWidget(self.scope_controls);box.addLayout(row);header=row;row=controls
        self.quantity=W.QComboBox();self.quantity.addItems(['关节位置 / °','关节速度 / °/s']);self.quantity.setCurrentIndex(self.prefs.data['scope_quantity']=='velocity');self.quantity.setFixedWidth(136);self.quantity.currentIndexChanged.connect(self.scope_changed);row.addWidget(self.quantity)
        self.sample_rate=W.QComboBox();self.sample_rate.setFixedWidth(90);self.sample_rate.addItems([f'{r} Hz' for r in RATES]);self.sample_rate.setCurrentIndex(RATES.index(self.prefs.data['scope_hz']));self.sample_rate.setToolTip('显示采样范围 1–60 Hz；不改变 125 Hz 控制频率。实际显示率受反馈源限制。');self.sample_rate.currentIndexChanged.connect(self.scope_changed);row.addWidget(self.sample_rate)
        self.time_window=W.QComboBox();self.time_window.setFixedWidth(82);self.time_window.addItems(['5 s','10 s','30 s']);self.time_window.setCurrentIndex(1);self.time_window.currentIndexChanged.connect(self.scope_changed);row.addWidget(self.time_window)
        self.freeze=W.QPushButton('暂停');self.freeze.setCheckable(True);self.freeze.toggled.connect(self.freeze_scope);row.addWidget(self.freeze)
        self.monitor_stack=W.QStackedWidget();box.addWidget(self.monitor_stack,1);layout.addWidget(card,3)
        scope_page=W.QWidget();box=W.QVBoxLayout(scope_page);box.setContentsMargins(0,0,0,0);box.setSpacing(6);self.monitor_stack.addWidget(scope_page)
        row=W.QHBoxLayout();self.signal_checks=[];self.joint_values=[]
        for i in range(6):
            check=W.QCheckBox('J'+str(i+1));check.setChecked(i in self.prefs.data['scope_signals']);check.setStyleSheet('color:'+SIGNAL_COLORS[i]+';background:#152236;border-radius:3px;padding:3px;');check.toggled.connect(self.scope_changed);self.signal_checks.append(check);row.addWidget(check)
            value=label('—','muted');value.setMinimumWidth(48);row.addWidget(value);self.joint_values.append(value)
        box.addLayout(row);self.plot=Scope();self.plot.selected=self.prefs.data['scope_signals'];self.plot.max_gap=max(.25,2.5/self.prefs.data['scope_hz']);self.plot.unit='°/s' if self.quantity.currentIndex() else '°';box.addWidget(self.plot,1)
        row=W.QHBoxLayout();self.feedback_label=label('等待反馈','muted');row.addWidget(self.feedback_label);row.addStretch();row.addWidget(label('显示采样 1–60 Hz  ·  控制周期保持不变','muted'));box.addLayout(row)
        self.grasp_tactile=TactileMonitor(compact=True);self.grasp_hand=self.grasp_tactile.hand;self.grasp.tactile_sample.connect(self.grasp_tactile.wave.ingest);self.monitor_stack.addWidget(self.grasp_tactile)
        self.tactile_controls=W.QWidget();controls=W.QHBoxLayout(self.tactile_controls);controls.setContentsMargins(0,0,0,0);header.addWidget(self.tactile_controls)
        self.tactile_start=W.QPushButton('开始监控');self.tactile_start.clicked.connect(lambda:self.request_grasp_mode('monitor'));controls.addWidget(self.tactile_start)
        self.tactile_stop=W.QPushButton('停止');self.tactile_stop.clicked.connect(self.stop_grasp);controls.addWidget(self.tactile_stop)
        reset=W.QPushButton('重置视图');reset.clicked.connect(self.grasp_tactile.reset_view);controls.addWidget(reset)
        self.monitor_select.idClicked.connect(self.select_monitor);self.select_monitor(0)
        self.latest_event=label('请在“设备连接”选择设备与方案。','muted');layout.addWidget(self.latest_event);self.joint_label=W.QLabel();self.joint_label.hide();return page
    def select_monitor(self,index):
        self.monitor_stack.setCurrentIndex(index);self.scope_controls.setVisible(index==0);self.tactile_controls.setVisible(index==1)
        self.teleop_monitor_card.setMinimumHeight(350 if index else 290);self.teleop_monitor_card.setMaximumHeight(380 if index else 310);self.scene.setMinimumHeight(270 if index else 420)
    def device_page(self):
        page,layout=self.page_base('设备连接','选择参与本次操作的设备，保存为自己的设备方案。')
        row=W.QHBoxLayout();self.profile_combo=W.QComboBox();self.profile_combo.addItems([p['name'] for p in self.prefs.data['profiles']]);self.profile_combo.setCurrentIndex(self.prefs.data['active']);row.addWidget(self.profile_combo,1)
        self.profile_load=W.QPushButton('加载方案');self.profile_load.clicked.connect(self.apply_profile);row.addWidget(self.profile_load)
        self.profile_save=W.QPushButton('保存到此方案');self.profile_save.clicked.connect(self.save_profile);row.addWidget(self.profile_save);layout.addLayout(row)
        grid=W.QGridLayout();grid.setSpacing(12)
        descriptions=[('arm','JAKA S5','机械臂 · 6 轴 / Servo 跟随'),('quest','Quest 3S','操控输入 · 右手 Touch 手柄'),('hand','DH116','灵巧手 · 厂家实时反馈'),('zed','ZED-M','腕部双目 · 左目 RGB + 深度'),('c922','Logitech C922','全局相机 · RGB')]
        for i,(name,title,detail) in enumerate(descriptions):
            card,box=frame('deviceCard');box.setContentsMargins(18,16,18,16);box.setSpacing(8);top=W.QHBoxLayout();top.addWidget(label(title,'section'));top.addStretch();check=W.QCheckBox('使用');check.toggled.connect(self.mark_profile_dirty);top.addWidget(check);box.addLayout(top);box.addWidget(label(detail,'muted'))
            state=label('○ 尚未检测');box.addWidget(state);self.device_boxes[name]=check;self.device_labels[name]=state;self.device_frames[name]=card
            if name=='arm':
                state.hide();self.arm_power_status=label('','muted');self.arm_power_status.hide()
                row=W.QHBoxLayout()
                self.arm_connect_button=W.QPushButton('连接机械臂');self.arm_connect_button.clicked.connect(self.toggle_arm_connection);row.addWidget(self.arm_connect_button);self.arm_status_button=self.arm_connect_button
                self.arm_connect_button.setToolTip('主动连接后自动更新状态。再次点击已连接按钮可断开监测。')
                for attr,title,action in [('arm_power_button','本体电源 · 未读取','power_on'),('arm_enable_button','机器人使能 · 未读取','enable')]:
                    button=W.QPushButton(title);button.clicked.connect(lambda _,a=action:self.arm_action(a));setattr(self,attr,button);row.addWidget(button)
                for button in (self.arm_connect_button,self.arm_power_button,self.arm_enable_button):button.setMinimumHeight(42);button.setCursor(QtCore.Qt.PointingHandCursor)
                box.addLayout(row)
            if name=='quest':
                self.quest_link_label=label('USB / HTTPS：尚未检查','muted');box.addWidget(self.quest_link_label)
                self.quest=W.QPushButton('重连 USB');self.quest.clicked.connect(self.connect_quest);box.addWidget(self.quest)
            if name=='hand':
                box.addWidget(label('现有交换机 · 只读反馈连接；不使能、不回零、不驱动手指','muted'))
                self.hand_interface=W.QComboBox()
                self.hand_interface.addItems(sorted(p.name for p in Path('/sys/class/net').iterdir() if (p/'device').exists() and not (p/'wireless').exists()))
                box.addWidget(self.hand_interface)
                controls=W.QHBoxLayout();self.hand_connect=W.QPushButton('连接 DH116');self.hand_disconnect=W.QPushButton('断开 DH116')
                self.hand_connect.clicked.connect(self.connect_hand);self.hand_disconnect.clicked.connect(self.stop_grasp)
                controls.addWidget(self.hand_connect);controls.addWidget(self.hand_disconnect);box.addLayout(controls)
            if name in CAMERA_PRESETS:
                mode_row=W.QHBoxLayout();mode_row.addWidget(label('视频模式','muted'))
                mode=W.QComboBox();mode.setMinimumHeight(38);mode.setSizeAdjustPolicy(W.QComboBox.AdjustToMinimumContentsLengthWithIcon);mode.setMinimumContentsLength(28)
                for preset in CAMERA_PRESETS[name]:mode.addItem(preset['label'],preset['id'])
                current=self.prefs.data['camera_modes'].get(name,CAMERA_DEFAULTS[name]);index=mode.findData(current);mode.setCurrentIndex(max(0,index))
                mode.currentIndexChanged.connect(lambda _,camera=name:self.camera_mode_changed(camera));self.camera_mode_combos[name]=mode
                mode_row.addWidget(mode,1);box.addLayout(mode_row)
                module=W.QPushButton('启动模块');module.clicked.connect(lambda _,device=name:self.toggle_device_monitor(device));self.device_module_buttons[name]=module;box.addWidget(module)
                note=label('自动保存 · 未监看时点“连接所选监看设备”查看；监看中更改会自动重连。','muted');note.setObjectName('cameraModeHint');box.addWidget(note)
            grid.addWidget(card,i//2,i%2,1,2 if i==4 else 1)
        layout.addLayout(grid)
        row=W.QHBoxLayout();self.monitor_start=W.QPushButton('启动所有所选反馈模块');self.monitor_start.setObjectName('primary');self.monitor_start.clicked.connect(self.start_monitor);row.addWidget(self.monitor_start)
        self.monitor_stop=W.QPushButton('停止所有反馈模块');self.monitor_stop.clicked.connect(self.stop_monitor);row.addWidget(self.monitor_stop)
        self.device_settings=W.QPushButton('反馈来源设置');self.device_settings.clicked.connect(self.configure_sources);row.addWidget(self.device_settings);layout.addLayout(row)
        card,box=frame();row=W.QHBoxLayout();row.addWidget(label('实时控制权限','section'));self.rt_label=label('尚未启用','muted');row.addWidget(self.rt_label,1);self.rt=W.QPushButton('启用本次实时权限');self.rt.clicked.connect(self.authorize);row.addWidget(self.rt);box.addLayout(row);layout.addWidget(card)
        layout.addWidget(label('绿色：正在接收有效数据  ·  红色：缺流 / 设备异常  ·  灰色：未选用或尚未启动\n监看设备与控制会话独立；切换方案不会启动机械臂。','muted'));layout.addStretch();return page
    def scope_changed(self):
        if not hasattr(self,'plot'):return
        self.plot.selected=[i for i,c in enumerate(self.signal_checks) if c.isChecked()];self.plot.seconds=(5,10,30)[self.time_window.currentIndex()]
        unit='velocity' if self.quantity.currentIndex() else 'position'
        if self.prefs.data['scope_quantity']!=unit:self.plot.samples.clear()
        self.plot.unit='°/s' if unit=='velocity' else '°';self.plot.max_gap=max(.25,2.5/RATES[self.sample_rate.currentIndex()]);self.prefs.data.update(scope_hz=RATES[self.sample_rate.currentIndex()],scope_signals=self.plot.selected,scope_quantity=unit)
        if not self.preview:self.save_preferences()
        self.plot.update()
    def freeze_scope(self,yes):self.plot.freeze(yes);self.freeze.setText('继续' if yes else '暂停')
    def change_theme(self,value):
        self.theme_animation.stop();self.theme_animation.setStartValue(float(self.prefs.data['brightness']));self.theme_animation.setEndValue(float(value));self.theme_animation.start()
    def show_about(self):
        from operator_credits import AboutDialog
        if self.about_dialog is None:self.about_dialog=AboutDialog(self)
        self.about_dialog.set_theme(*self.about_colors);self.about_dialog.show();self.about_dialog.raise_();self.about_dialog.activateWindow()
    def set_brightness(self,value):self.prefs.data['brightness']=round(value);self.apply_theme()
    def apply_theme(self):
        t=self.prefs.data['brightness']/100
        def blend(a,b):return '#'+''.join(f'{round(int(a[i:i+2],16)*(1-t)+int(b[i:i+2],16)*t):02x}' for i in (0,2,4))
        bg=blend('111214','f5f5f7');card=blend('1d1f23','ffffff');fg=blend('f0f1f4','1d1d1f');muted=blend('a1a4ac','747880');border=blend('33363c','e6e6eb')
        sheet=STYLE+f'''QMainWindow,QWidget{{background:{bg};color:{fg};}} QFrame#card,QFrame#topbar,QFrame#topbar QLabel{{background:{card};border-color:{border};}} QLabel#heading,QLabel#section,QLabel#value{{color:{fg};}} QLabel#muted{{color:{muted};}} QComboBox,QPushButton{{background:{card};color:{fg};border-color:{border};}} QFrame#deviceCard{{background:{card};border:1px solid {border};border-radius:14px;}} QFrame#deviceCard QLabel,QFrame#deviceCard QCheckBox{{background:transparent;}} QFrame#deviceCard[health="ok"]{{background:{blend('17362d','e8f6ed')};border-color:#42836a;}} QFrame#deviceCard[health="error"]{{background:{blend('3b242b','fff0f0')};border-color:#ac5d65;}} QSlider::groove:horizontal{{height:4px;background:{border};border-radius:2px;}} QSlider::handle:horizontal{{background:#608ff2;width:13px;margin:-5px 0;border-radius:6px;}} QLabel#status{{color:{fg};}} QFrame#rule{{background:{border};}} QLabel#chip{{background:{blend('263650','edf3ff')};color:{blend('93b8ff','3b63a6')};border-color:{border};}} QPushButton:disabled{{background:{blend('222d3e','f5f6f8')};color:{blend('66758b','aeb7c5')};border-color:{border};}} QPushButton#stop:disabled{{background:{blend('302830','faf6f6')};color:{blend('8c6770','ccb2b2')};border-color:{blend('49303c','eadcdb')};}} QComboBox QAbstractItemView{{background:{card};color:{fg};selection-background-color:#3269d5;selection-color:white;}} QLineEdit,QSpinBox,QDoubleSpinBox{{background:{card};color:{fg};border:1px solid {border};padding:6px;}}QFrame#card{{border-radius:16px;}} QPushButton,QComboBox{{border-radius:9px;}} QComboBox{{min-width:0;}} QLabel#heading{{font-size:24px;}} QLabel#chip{{border-radius:10px;}} QFrame#sidebar,QFrame#sidebar QLabel{{background:{blend('18191d','ececf0')};color:{muted};}} QFrame#sidebar QLabel#brand{{color:{fg};}} QPushButton#nav{{color:{muted};border-radius:10px;}} QPushButton#nav:checked{{background:{blend('303238','dcdfe6')};color:{fg};border-left:0;}} QPushButton#nav:hover{{background:{blend('292b31','e1e3e8')};color:{fg};}} QPushButton#primary{{background:#0a78f0;border-color:#0a78f0;}} QPushButton#primary:disabled{{background:{blend('293343','e6edf7')};border-color:{border};color:{muted};}} QScrollBar::handle:vertical{{background:{blend('464951','c7c8cf')};}} QPushButton:hover,QComboBox:hover{{background:{blend('2a2d33','f2f4f7')};border-color:{blend('626772','aab3c1')};}} QPushButton:focus,QComboBox:focus{{border-color:#0a78f0;}} QCheckBox::indicator{{width:16px;height:16px;border:1px solid {muted};border-radius:4px;background:{card};}} QCheckBox::indicator:checked{{background:#0a78f0;border-color:#0a78f0;image:url({ROOT/'tools/operator_assets/check.svg'});}} QCheckBox::indicator:hover{{border-color:#0a78f0;}}'''
        for workspace in ('handWorkspace','teleopMonitor'):
            sheet+=f'''QWidget#{workspace} QTabWidget::pane{{border:0;background:{card};}} QWidget#{workspace} QTabBar::tab{{background:{blend('292b30','eeeef2')};color:{muted};padding:8px 14px;border:0;margin:2px;border-radius:8px;}} QWidget#{workspace} QTabBar::tab:selected{{background:{blend('3b3e45','ffffff')};color:{fg};font-weight:600;}} QWidget#{workspace} QTabBar::tab:hover{{color:#0a84ff;}} QWidget#{workspace} QTableWidget{{background:{card};alternate-background-color:{card};color:{fg};border:0;selection-background-color:{blend('233a54','eaf3ff')};selection-color:{fg};}} QWidget#{workspace} QTableWidget::item{{padding:4px;border-bottom:1px solid {border};}} QWidget#{workspace} QHeaderView::section{{background:{blend('25272c','f5f5f7')};color:{muted};border:0;border-bottom:1px solid {border};padding:8px 3px;font-size:11px;font-weight:500;}} QWidget#{workspace} QSpinBox,QWidget#{workspace} QDoubleSpinBox{{border-radius:7px;padding:4px 6px;}} QWidget#{workspace} QProgressBar{{border:0;background:{blend('2b2e35','edf0f5')};border-radius:6px;text-align:center;min-height:22px;}} QWidget#{workspace} QProgressBar::chunk{{background:{blend('285992','a3caff')};border-radius:6px;}} QWidget#{workspace} QSplitter::handle{{background:transparent;}}'''
        sheet+=f'''QFrame#teleopMonitor{{background:{card};border:1px solid {border};border-radius:16px;}} QFrame#teleopMonitor QWidget{{background:{card};}} QWidget#handCanvas,QWidget#handTabPage,QWidget#handAxis{{background:{card};}} QWidget#handAxis QLabel{{background:transparent;}}'''
        sheet+=f'''QPushButton[armState="on"],QPushButton[armState="on"]:disabled{{background:{blend('17382c','edf8f0')};color:{blend('77dda0','197842')};border:1px solid {blend('2e6047','b9dfc7')};}} QPushButton[armState="occupied"],QPushButton[armState="error"]{{background:{blend('422b2d','fff0f0')};color:{blend('ff8e89','c53030')};border:1px solid {blend('79403f','f1b8b5')};}} QPushButton[armState="working"]{{color:#0a84ff;border-color:#0a84ff;}}'''
        self.setStyleSheet(sheet)
        # QScrollArea can retain its child page's initial polished palette.
        # Theme the content root explicitly so the first frame matches the shell.
        for i in range(self.pages.count()):self.pages.widget(i).widget().setStyleSheet(sheet)
        self.session_controls.setStyleSheet(sheet)
        self.about_colors=(bg,card,fg,blend('9b9ea7','81818a'),border)
        if hasattr(self,'tooling_popover'):self.tooling_popover.set_theme(card,fg,muted,border)
        if getattr(self,'about_dialog',None) is not None:self.about_dialog.set_theme(*self.about_colors)
        if hasattr(self,'hand_model'):self.hand_model.set_theme(card,fg)
        if hasattr(self,'action_editor'):
            self.action_editor.setStyleSheet(f'QWidget#graspActionEditor {{background:{card};}} QLabel, QCheckBox {{background:transparent;}} '
                                            f'QSpinBox,QDoubleSpinBox,QLineEdit {{border:1px solid {border};border-radius:6px;background:{bg};padding:4px 6px;}}')
        if hasattr(self,'grasp_tactile'):self.grasp_tactile.set_theme(card,fg,border)
        for hand in self.findChildren(HandForces):hand.background=card;hand.foreground=fg;hand.update()
    def save_preferences(self):
        if not self.preview:self.prefs.save()
    def mark_profile_dirty(self):
        self.profile_dirty=True
        if hasattr(self,'profile_save'):self.profile_save.setText('保存到此方案 *')
        if hasattr(self,'start'):self.refresh()
    def load_profile(self):
        p=self.prefs.profile
        for name,check in self.device_boxes.items():check.blockSignals(True);check.setChecked(p['enabled'][name]);check.blockSignals(False)
        for camera,combo in self.camera_mode_combos.items():
            index=combo.findData(self.prefs.data['camera_modes'].get(camera,CAMERA_DEFAULTS[camera]));combo.blockSignals(True);combo.setCurrentIndex(max(0,index));combo.blockSignals(False)
        self.profile_dirty=False;self.profile_save.setText('保存到此方案')
        self.update_camera_controls()
    def selected_profile(self):
        p=copy.deepcopy(self.prefs.profile);p['enabled']={n:c.isChecked() for n,c in self.device_boxes.items()};return p
    def apply_profile(self):
        if self.job is not None or self.hand.process is not None:return
        self.stop_monitor();self.prefs.data['active']=self.profile_combo.currentIndex();self.load_profile();self.save_preferences();self.refresh();self.message('已加载设备方案。可逐个启动模块，也可启动所有所选反馈模块。')
    def save_profile(self):
        if self.job is not None:return
        index=self.profile_combo.currentIndex();p=self.selected_profile();p['name']=self.prefs.data['profiles'][index]['name'];self.prefs.data['profiles'][index]=p;self.prefs.data['active']=index;self.save_preferences();self.load_profile();self.message('设备方案已保存。')
    def camera_mode_changed(self,camera):
        combo=self.camera_mode_combos.get(camera)
        if combo is None or self.preview:return
        selected=combo.currentData();previous=self.prefs.data['camera_modes'].get(camera,CAMERA_DEFAULTS[camera])
        if selected==previous:return
        self.prefs.data['camera_modes'][camera]=selected
        try:self.save_preferences()
        except Exception as e:
            self.prefs.data['camera_modes'][camera]=previous;combo.blockSignals(True);combo.setCurrentIndex(max(0,combo.findData(previous)));combo.blockSignals(False);self.message('视频模式未保存：'+str(e));return
        preset=camera_preset(camera,selected)
        if camera in self.device_monitors and self.prefs.profile.get('camera_source')=='local' and self.device_boxes[camera].isChecked():
            self.message(f"{camera.upper()} 正在切换到 {preset['width']} × {preset['height']} · {preset['fps']} fps…")
            self.stop_device_monitor(camera)
            try:self.start_device_monitor(camera);self.message('视频模式已应用；画面恢复后可在遥操作的 3D 场景查看。')
            except OSError as e:self.message('视频模式已保存，但模块重启失败：'+str(e))
        else:self.message('视频模式已保存；下次连接本机监看相机时生效。')
    def update_camera_controls(self):
        local=self.prefs.profile.get('camera_source','local')=='local'
        busy=self.job is not None
        for camera,combo in self.camera_mode_combos.items():
            combo.setEnabled(local and not busy)
            preset=camera_preset(camera)
            combo.setToolTip(f"采集默认使用 {preset['width']} × {preset['height']}、{preset['fps']} fps，切换后仅重启此相机。" if local else '当前使用 ROS 图像话题，分辨率和帧率由图像发布端设置。')
            module=self.device_module_buttons.get(camera)
            if module is not None:
                active=camera in self.device_monitors
                module.setText('停止本模块' if active else '启动本模块')
                module.setEnabled(not busy and self.device_boxes[camera].isChecked())
                module.setToolTip(('停止此相机的独立监看进程。' if active else '只启动此相机模块，不影响其他设备。') if local else ('停止此相机的独立 ROS 图像订阅。' if active else '只订阅此相机 ROS 话题，不影响其他设备。'))
            card=self.device_frames[camera]
            for note in card.findChildren(W.QLabel,'cameraModeHint'):note.setText('自动保存 · 可单独启动此相机，也可启动全部所选反馈模块。' if local else '可单独启动此 ROS 图像订阅；分辨率和帧率由图像发布端设置。')
    def configure_sources(self):
        dialog=W.QDialog(self);dialog.setWindowTitle('反馈来源');layout=W.QFormLayout(dialog);fields={};p=self.prefs.profile
        name=W.QLineEdit(p['name']);name.setMaxLength(24);layout.addRow('方案名称',name)
        source=W.QComboBox();source.addItems(['本机相机（ZED SDK / USB）','ROS 图像话题']);source.setCurrentIndex(p['camera_source']=='ros');layout.addRow('相机来源',source)
        domain=W.QSpinBox();domain.setRange(0,101);domain.setValue(p['sensor_domain']);layout.addRow('传感器 ROS 域',domain)
        for key,text in [('hand_topic','DH116 厂家实时状态'),('zed_left_topic','ZED 左目 RGB'),('zed_depth_topic','ZED 深度图'),('c922_topic','C922 RGB')]:
            edit=W.QLineEdit(p[key]);edit.setMinimumWidth(350);fields[key]=edit;layout.addRow(text,edit)
        note=label('控制域 89/90 和显示域 91 保留。DH116 订阅厂家 HandJointState；相机订阅 sensor_msgs/Image。','muted');layout.addRow(note)
        buttons=W.QDialogButtonBox(W.QDialogButtonBox.Save|W.QDialogButtonBox.Cancel);buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject);layout.addRow(buttons)
        if dialog.exec_()==W.QDialog.Accepted:
            old=copy.deepcopy(self.prefs.data)
            try:
                p.update(name=name.text().strip(),camera_source='ros' if source.currentIndex() else 'local',sensor_domain=domain.value(),**{k:v.text().strip() for k,v in fields.items()});self.prefs.save();self.update_camera_controls()
                self.profile_combo.setItemText(self.prefs.data['active'],p['name']);self.message('来源设置已保存，重新连接监看设备后生效。')
            except Exception as e:self.prefs.data=old;self.message('设置未保存：'+str(e))
    def start_monitor(self):
        selected=[name for name in ('hand','zed','c922') if self.device_boxes[name].isChecked() and name not in self.device_monitors]
        if not selected:self.message('没有尚未启动的所选反馈模块。');return
        started=[];failed=[]
        for name in selected:
            try:self.start_device_monitor(name);started.append(name)
            except OSError as e:failed.append(name.upper()+': '+str(e))
        message='已独立启动：'+', '.join({'hand':'DH116','zed':'ZED-M','c922':'C922'}[n] for n in started) if started else '没有反馈模块启动。'
        if failed:message+=' 启动失败：'+'；'.join(failed)
        self.message(message)
    def start_device_monitor(self,name):
        if name not in ('hand','zed','c922') or name in self.device_monitors:return
        atomic(RUNTIME,dict(scope_hz=self.prefs.data['scope_hz'],profile=self.selected_profile(),camera_modes=self.prefs.data['camera_modes']))
        log=(ROOT/'artifacts'/f'operator_devices_{name}.log').open('w')
        try:
            process=subprocess.Popen(['bash','tools/operator_ros.sh','python3','-B','tools/operator_devices.py',name],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        except OSError:
            log.close();raise
        self.device_monitors[name]=dict(process=process,log=log);self.refresh()
    def toggle_device_monitor(self,name):
        if name in self.device_monitors:
            self.stop_device_monitor(name);self.message({'zed':'ZED-M','c922':'C922'}[name]+' 模块已停止。');return
        if not self.device_boxes[name].isChecked():self.message('请先勾选“使用”再启动该模块。');return
        try:self.start_device_monitor(name);self.message({'zed':'ZED-M','c922':'C922'}[name]+' 模块已单独启动。')
        except OSError as e:self.message(name.upper()+' 启动失败：'+str(e))
    def stop_device_monitor(self,name):
        entry=self.device_monitors.pop(name,None)
        if entry is None:return
        process=entry['process']
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=4)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        entry['log'].close();self.media_states.pop(name,None)
        if name=='zed':
            self.media_states.pop('zed_left',None);self.media_states.pop('zed_depth',None)
            for key in ('zed_left','zed_depth'):self.camera_tiles[key].update_frame('',False,'模块已停止')
        elif name=='c922':self.camera_tiles['c922'].update_frame('',False,'模块已停止')
        self.refresh()
    def connect_hand(self):
        if self.job is not None or self.grasp.process is not None:return
        try:self.hand.connect_device(self.hand_interface.currentText());self.message('DH116：正在连接；此入口仅采集反馈。连接并使能请使用“灵巧手抓握”。')
        except (OSError,ValueError) as e:self.message('DH116 连接未启动：'+str(e))
    def stop_monitor(self):
        for name in list(self.device_monitors):self.stop_device_monitor(name)
        self.media_states={}
        for tile in self.camera_tiles.values():tile.update_frame('',False,'监看已停止')
        if hasattr(self,'start'):self.refresh()
    def reset_scene(self):self.scene_reset=time.monotonic();self.write_scene(False)
    def write_scene(self,fresh,values=None):
        if not self.preview:atomic(ROOT/'.cache/operator/scene_state.json',dict(fresh=fresh,updated=time.monotonic(),joint_radians=values or [],reset_view=self.scene_reset,target_session_started=getattr(self,'job_started_at',0)))
    def refresh_view(self,busy,hardware):
        super().refresh_view(busy,hardware)
        if not hasattr(self,'monitor_start'):return
        hand_busy=self.hand.process is not None or self.grasp.process is not None
        self.hand_connect.setEnabled(not busy and not hand_busy and self.device_boxes['hand'].isChecked() and self.hand_interface.count()>0)
        self.hand_disconnect.setEnabled(hand_busy)
        self.hand_interface.setEnabled(not hand_busy and not busy)
        enabled={n:c.isChecked() for n,c in self.device_boxes.items()}
        any_monitor=bool(self.device_monitors)
        for widget in [self.profile_combo,self.profile_load,self.profile_save,self.device_settings]:widget.setEnabled(not busy and not any_monitor)
        self.monitor_start.setEnabled(not busy and any(c.isChecked() and n not in self.device_monitors for n,c in self.device_boxes.items() if n in ('hand','zed','c922')))
        for name,widget in self.device_boxes.items():
            widget.setEnabled(not busy and name not in self.device_monitors and not (name=='hand' and hand_busy))
        self.profile_load.setEnabled(not busy and not hand_busy)
        self.monitor_stop.setEnabled(any_monitor)
        self.update_camera_controls()
        if not enabled.get('arm') or not enabled.get('quest'):self.start.setEnabled(False);self.test.setEnabled(False);self.start.setToolTip('机械臂遥操作需要同时选用 JAKA S5 和 Quest 3S。')
        else:self.start.setToolTip('')
        self.refresh_grasp()
    def clear_feedback(self):
        super().clear_feedback()
        if hasattr(self,'scene_status'):self.scene_status.setText('模型预览 / 反馈已停止');self.write_scene(False)
    def event_view(self,event):
        # Slow textual events do not feed the high-rate oscilloscope.
        data=dict(event);data.pop('joint_feedback_fresh',None);data.pop('joint_degrees',None);super().event_view(data)
    def ingest_telemetry(self,d,now=None):
        now=time.monotonic() if now is None else now
        stamp=d['received_monotonic'];q=d['position'];v=d.get('velocity',[])
        if self.job is None or not self.job_started_at<=stamp<=now or now-stamp>=.5 or stamp<=self.last_telemetry or d['mode']!=('hardware' if self.mode.currentIndex() else 'fake'):return
        if len(q)!=6 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in q):return
        self.last_telemetry=stamp;self.feedback_at=stamp
        raw=v if self.quantity.currentIndex() else q
        values=[math.degrees(x) for x in raw] if len(raw)==6 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in raw) else []
        if stamp-self.last_plot_at>=.9/self.prefs.data['scope_hz']:
            self.plot.push(values,stamp);self.last_plot_at=stamp
        for i,item in enumerate(self.joint_values):item.setText(f'{values[i]:.1f}' if values else '—')
        self.feedback_label.setText(f'反馈在线 · 显示上限 {self.prefs.data["scope_hz"]} Hz' if values else '当前反馈源未提供关节速度')
        self.scene_status.setText(('实机' if self.mode.currentIndex() else '仿真')+'实时反馈');self.write_scene(True,q)
        self.quest_feedback_at=d.get('input_received_monotonic',0)
        self.quest_feedback_ok=d.get('input_reason') in ('RELEASED','FOLLOWING','HELD','SOFTWARE_STOP') and d.get('input_age_s',999)<.08
    def show_target(self,data,now):
        try:target=current_target(data,now,getattr(self,'job_started_at',0))
        except (TypeError,ValueError):target=None
        if target is None:
            self.target_position.setText('手柄目标 · 基坐标 / mm     X —    Y —    Z —')
            self.target_rotation.setText('目标姿态 / °     Rx —    Ry —    Rz —  · 等待有效目标')
            return
        p=[v*1000 for v in target['position_m']]
        with warnings.catch_warnings():
            warnings.simplefilter('ignore',UserWarning)
            angles=Rotation.from_quat(target['quaternion_xyzw']).as_euler('xyz',degrees=True)
        self.target_position.setText('手柄目标 · 基坐标 / mm     '+ '    '.join(f'{name} {v:+.1f}' for name,v in zip(('X','Y','Z'),p)))
        self.target_rotation.setText('目标姿态 / °     '+ '    '.join(f'{name} {v:+.1f}' for name,v in zip(('Rx','Ry','Rz'),angles)))
    def studio_tick(self):
        self.scene.poll();now=time.monotonic()
        if now-self.target_read_at>=1/30:
            self.target_read_at=now
            try:target_data=json.loads((ROOT/'.cache/operator/target_state.json').read_text())
            except (OSError,ValueError):target_data={}
            self.show_target(target_data,now)
        try:self.ingest_telemetry(json.loads((ROOT/'.cache/operator/telemetry.json').read_text()),now)
        except (OSError,ValueError,KeyError,TypeError):pass
        if self.feedback_at and now-self.feedback_at>.5:self.clear_feedback()
        if now-self.last_media<.05:return
        self.last_media=now
        for name,entry in list(self.device_monitors.items()):
            process=entry['process']
            if process.poll() is not None:
                entry['log'].close();self.device_monitors.pop(name,None);self.refresh()
        self.media_states={}
        for name,entry in self.device_monitors.items():
            process=entry['process']
            try:
                media=json.loads((ROOT/'.cache/operator/media'/f'monitor_{name}.json').read_text())
                if media['pid']==process.pid and 0<=now-media['updated']<1.5:
                    self.media_states.update({k:current_state(v,now) for k,v in media['devices'].items()})
            except (OSError,ValueError,KeyError,TypeError):pass
        if self.hand.state and (self.hand.process is not None or 'hand' not in self.media_states):self.media_states['hand']=current_state(self.hand.state,now)
        if self.grasp.state:self.media_states['hand']=current_state(self.grasp.state,now,.08)
        for name,tile in self.camera_tiles.items():
            d=self.media_states.get(name,{});ok=d.get('ok') and now-d.get('updated',0)<1.5
            if name in ('zed_left','c922'):
                camera='zed' if name=='zed_left' else 'c922';preset=camera_preset(camera,self.prefs.data['camera_modes'].get(camera,CAMERA_DEFAULTS[camera]))
                width=d.get('width',preset['width']);height=d.get('height',preset['height']);fps=d.get('fps')
                tile.subtitle=f"{width} × {height}"+(f" · {fps:.0f} fps" if isinstance(fps,(int,float)) and fps>0 else (f" · {preset['fps']} fps" if self.prefs.profile.get('camera_source')=='local' else ''))
            tile.update_frame(ROOT/'.cache/operator/media'/(name+'.jpg'),ok,d.get('detail','等待连接'))
        hand=self.media_states.get('hand',{});hand_fresh=hand.get('ok') and now-hand.get('updated',0)<1.5;self.hand_feedback_label.setText('DH116：'+(hand.get('detail','') if hand_fresh else '无当前手指反馈 · '+hand.get('detail','预览姿态')))
        for name,card in self.device_frames.items():
            enabled=self.device_boxes[name].isChecked();health='idle';text='未选用' if not enabled else '尚未启动'
            if enabled:
                if name=='arm':
                    # Status belongs on the buttons; keep a calm neutral card.
                    self.refresh_arm_buttons();text=''
                elif name=='quest':
                    valid=(now-self.quest_feedback_at<.5 and self.quest_feedback_ok) if self.job is not None else (now-self.status_input_at<1.5 and '追踪有效' in self.input_label.text());health='ok' if valid else 'error';text=self.input_label.text()
                elif name in ('hand','zed','c922') and (name in self.device_monitors or (name=='hand' and (self.hand.state or self.grasp.process is not None))):
                    entries=[self.media_states.get(n,{}) for n in (['zed_left','zed_depth'] if name=='zed' else [name])];health='ok' if all(d.get('ok') and d.get('mode_match',True) and now-d.get('updated',0)<1.5 for d in entries) else 'error';text=' / '.join(d.get('detail','等待数据') for d in entries)
            self.device_labels[name].setText(('● ' if health=='ok' else '○ ')+text)
            if card.property('health')!=health:card.setProperty('health',health);card.style().unpolish(card);card.style().polish(card)
