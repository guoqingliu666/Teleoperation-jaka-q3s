"""Presentation widgets only; no ROS, SDK or control side effects."""
import math,time
from collections import deque
from PyQt5 import QtCore,QtGui,QtWidgets as W

COLORS=['#3068d6','#15846f','#a87923','#8b65b5','#bf5f65','#437b98']

class RowCheckBox(W.QCheckBox):
    """The complete visible confirmation row is a click target."""
    def hitButton(self,point):return self.rect().contains(point)

class CurrentSizeTabs(W.QTabWidget):
    """Hidden pages must not enlarge a monitoring workspace's minimum height."""
    def __init__(self,parent=None):
        super().__init__(parent);self.currentChanged.connect(lambda _:self.updateGeometry())
    def hasHeightForWidth(self):return False
    def heightForWidth(self,width):return self.minimumSizeHint().height()
    def minimumSizeHint(self):
        widget=self.currentWidget()
        if widget is None:return super().minimumSizeHint()
        size=widget.minimumSizeHint().expandedTo(widget.minimumSize())
        return QtCore.QSize(max(size.width()+8,self.tabBar().minimumSizeHint().width()),max(0,size.height())+self.tabBar().sizeHint().height()+6)
    def sizeHint(self):
        widget=self.currentWidget()
        if widget is None:return super().sizeHint()
        size=widget.sizeHint().expandedTo(widget.minimumSize())
        return QtCore.QSize(max(size.width()+8,self.tabBar().sizeHint().width()),max(0,size.height())+self.tabBar().sizeHint().height()+6)

def label(text,kind=None):
    item=W.QLabel(text)
    if kind:item.setObjectName(kind)
    item.setWordWrap(True)
    return item

def frame(kind='card'):
    widget=W.QFrame();widget.setObjectName(kind)
    box=W.QVBoxLayout(widget);box.setContentsMargins(22,20,22,20);box.setSpacing(14)
    return widget,box

def rule():
    line=W.QFrame();line.setObjectName('rule');line.setFixedHeight(1);return line

class JointPlot(W.QWidget):
    """Thirty seconds of observed feedback, sampled for display only."""
    def __init__(self):
        super().__init__();self.samples=deque(maxlen=120);self.fresh=False;self.setMinimumHeight(180);self.setObjectName('chart')
    def push(self,values):
        if len(values)!=6 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):return
        self.samples.append((time.monotonic(),list(values)));self.fresh=True;self.update()
    def paintEvent(self,event):
        painter=QtGui.QPainter(self);painter.setRenderHint(QtGui.QPainter.Antialiasing)
        area=QtCore.QRectF(47,16,self.width()-62,self.height()-49)
        painter.setFont(QtGui.QFont('Noto Sans CJK SC',9))
        now=time.monotonic();points=[(t,q) for t,q in self.samples if t>=now-30]
        vals=[v for _,q in points for v in q];low=math.floor((min(vals)-5)/10)*10 if vals else -90;high=math.ceil((max(vals)+5)/10)*10 if vals else 90
        for i in range(5):
            y=area.top()+i*area.height()/4;painter.setPen(QtGui.QColor('#e7ebf0'));painter.drawLine(QtCore.QPointF(area.left(),y),QtCore.QPointF(area.right(),y))
            painter.setPen(QtGui.QColor('#8a94a5'));painter.drawText(QtCore.QRectF(0,y-10,38,20),QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter,f'{high-(high-low)*i/4:.0f}°')
        for i in range(4):
            x=area.left()+i*area.width()/3;painter.setPen(QtGui.QColor('#f0f2f5'));painter.drawLine(QtCore.QPointF(x,area.top()),QtCore.QPointF(x,area.bottom()))
            painter.setPen(QtGui.QColor('#8a94a5'));painter.drawText(QtCore.QRectF(x-24,area.bottom()+10,48,20),QtCore.Qt.AlignCenter,f'{-30+10*i} s' if i<3 else '现在')
        if not points:
            painter.setPen(QtGui.QColor('#8a94a5'));painter.drawText(area,QtCore.Qt.AlignCenter,'启动会话后显示关节反馈曲线');return
        for index,color in enumerate(COLORS):
            path=QtGui.QPainterPath();previous=None
            for t,q in points:
                point=QtCore.QPointF(area.right()-(now-t)/30*area.width(),area.bottom()-(q[index]-low)/(high-low)*area.height())
                if previous is None or t-previous>2:path.moveTo(point)
                else:path.lineTo(point)
                previous=t
            painter.setPen(QtGui.QPen(QtGui.QColor(color if self.fresh else '#b5bdc9'),1.7));painter.drawPath(path)

STYLE='''
QMainWindow,QWidget{background:#f4f5f7;color:#273243;font-family:"Noto Sans CJK SC";font-size:13px;}
QFrame#sidebar,QFrame#sidebar QLabel{background:#171d29;color:#8591a4;}
QFrame#sidebar QLabel#brand{color:white;font-size:24px;font-weight:700;letter-spacing:2px;}
QLabel#brandSub{color:#8b9bb3;font-size:10px;letter-spacing:2px;}
QPushButton#nav{background:transparent;color:#9da9ba;border:0;text-align:left;padding:15px 18px;border-radius:5px;}
QPushButton#nav:checked{background:#2b3548;color:white;border-left:3px solid #608ff2;}
QPushButton#nav:hover{background:#242e40;color:white;}
QFrame#topbar,QFrame#topbar QLabel{background:white;}
QFrame#card{background:white;border:1px solid #e2e6ec;border-radius:7px;}
QFrame#card QLabel{background:transparent;}
QWidget#chart{background:white;}
QLabel#heading{font-size:25px;font-weight:600;color:#1e2a3c;}
QLabel#section{font-size:15px;font-weight:600;color:#26334a;}
QLabel#muted{color:#8791a1;font-size:12px;}
QLabel#eyebrow{color:#8b97a9;font-size:10px;letter-spacing:1px;}
QLabel#value{font-family:"DejaVu Sans Mono";font-size:20px;color:#26334a;}
QLabel#chip{background:#edf3ff;color:#3b63a6;border:1px solid #dce7fc;border-radius:4px;padding:5px 10px;font-size:12px;}
QLabel#status{color:#344762;font-size:13px;}
QFrame#rule{background:#e8ecf1;border:0;}
QComboBox{background:white;border:1px solid #d9dfe8;border-radius:4px;padding:8px 12px;min-width:135px;}
QComboBox::drop-down{border:0;width:22px;}
QComboBox QAbstractItemView{background:white;selection-background-color:#edf3ff;selection-color:#2459b5;}
QPushButton{background:white;border:1px solid #d5dce6;border-radius:4px;padding:10px 15px;color:#41516b;}
QPushButton:hover{background:#f2f5fa;border-color:#9db3d6;}
QPushButton:disabled{background:#f5f6f8;color:#aeb7c5;border-color:#e7ebf0;}
QPushButton#primary{background:#3269d5;color:white;border:1px solid #3269d5;font-weight:600;}
QPushButton#primary:hover{background:#255bc1;}
QPushButton#primary:disabled{background:#e1e8f5;border-color:#e1e8f5;color:#99aacc;}
QPushButton#stop{background:#fff4f3;border:1px solid #e4b3b2;color:#bb3e3e;font-weight:600;}
QPushButton#stop:hover{background:#ffe6e3;border-color:#c95e5b;}
QPushButton#stop:disabled{background:#faf6f6;color:#ccb2b2;border-color:#eadcdb;}
QCheckBox{spacing:8px;background:transparent;}
QCheckBox::indicator{width:16px;height:16px;}
QPlainTextEdit{background:#192130;color:#bdc9db;border:0;border-radius:5px;font-family:"DejaVu Sans Mono";font-size:11px;padding:12px;}
QScrollArea{border:0;background:transparent;}
QScrollBar:vertical{width:6px;background:transparent;}
QScrollBar::handle:vertical{background:#c7ced9;border-radius:3px;min-height:30px;}
QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}
'''

class WorkbenchView:
    def build_ui(self):
        self.setWindowTitle('V3 Operator · JAKA S5 遥操作');self.resize(1440,900);self.setMinimumSize(1190,780)
        self.feedback_at=0;self.job_started_at=0;self.last_op_at=0
        host=W.QWidget();self.setCentralWidget(host);shell=W.QHBoxLayout(host);shell.setContentsMargins(0,0,0,0);shell.setSpacing(0)
        sidebar=W.QFrame();sidebar.setObjectName('sidebar');sidebar.setFixedWidth(170);side=W.QVBoxLayout(sidebar);side.setContentsMargins(16,30,16,22);side.setSpacing(8)
        side.addWidget(label('V3','brand'));side.addWidget(label('OPERATOR CONSOLE','brandSub'));side.addSpacing(40)
        self.nav=[None]*5
        for i,name in ((0,'遥操作'),(4,'灵巧手抓握'),(1,'设备连接'),(2,'检查与验收'),(3,'运行日志')):
            button=W.QPushButton(name);button.setObjectName('nav');button.setCheckable(True);button.setAutoExclusive(True);button.clicked.connect(lambda _,n=i:self.set_page(n));side.addWidget(button);self.nav[i]=button
        side.addStretch();side.addWidget(label('JAKA S5  ×  Quest 3S','muted'));side.addWidget(label('Ubuntu 24.04\nROS 2 Jazzy','muted'));shell.addWidget(sidebar)
        column=W.QVBoxLayout();column.setContentsMargins(0,0,0,0);column.setSpacing(0);shell.addLayout(column,1)
        top=W.QFrame();top.setObjectName('topbar');top.setFixedHeight(70);bar=W.QHBoxLayout(top);bar.setContentsMargins(28,0,28,0)
        bar.addWidget(label('工作站  /  JAKA S5','section'));bar.addStretch();self.mode_badge=label('仿真模式','chip');self.mode_badge.setFixedHeight(30);bar.addWidget(self.mode_badge);bar.addSpacing(14)
        self.mode=W.QComboBox();self.mode.addItems(['仿真 · 模拟硬件','真机 · JAKA S5']);self.mode.currentIndexChanged.connect(self.mode_changed);bar.addWidget(self.mode);column.addWidget(top)
        content=W.QHBoxLayout();content.setContentsMargins(28,26,28,20);content.setSpacing(22);column.addLayout(content,1)
        self.pages=W.QStackedWidget();content.addWidget(self.pages,1)
        for page in (self.teleop_page(),self.device_page(),self.check_page(),self.log_page(),self.grasp_page()):
            page.layout().setSizeConstraint(W.QLayout.SetMinimumSize)
            scroll=W.QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(page);self.pages.addWidget(scroll)
        controls,box=frame();self.session_controls=controls;controls.setFixedWidth(290);box.setSpacing(8);content.addWidget(controls)
        box.addWidget(label('会话控制','section'));row=W.QHBoxLayout();self.phase_label=label('待机','chip');self.elapsed=label('00:00','muted');row.addWidget(self.phase_label);row.addStretch();row.addWidget(self.elapsed);box.addLayout(row)
        self.status=label('选择模式后启动会话。','status');self.status.setMinimumHeight(64);box.addWidget(self.status);box.addWidget(rule())
        box.addWidget(label('QUEST 3S · 右手输入','eyebrow'));self.input_label=label('等待头显进入透视模式');box.addWidget(self.input_label);box.addWidget(rule())
        self.site=RowCheckBox('现场准备已完成');self.site.setToolTip('工作区已清空，可立即操作物理急停。');self.site.toggled.connect(self.refresh);box.addWidget(self.site)
        self.site_hint=label('工作区清空 · 可操作物理急停','muted');box.addWidget(self.site_hint)
        self.start=W.QPushButton('启动仿真遥操作');self.start.setObjectName('primary');self.start.setMinimumHeight(48);self.start.clicked.connect(self.primary_start);box.addWidget(self.start)
        self.stop=W.QPushButton('停止会话');self.stop.setObjectName('stop');self.stop.setMinimumHeight(48);self.stop.clicked.connect(self.stop_session);box.addWidget(self.stop)
        box.addWidget(label('停止会话会请求控制器退出。\n物理急停独立于软件工作。','muted'));box.addStretch();box.addWidget(rule())
        box.addWidget(label('手柄操作','section'))
        box.addWidget(label('看头显提示。握住右侧 Grip 跟随，松开暂停。'))
        box.addWidget(label('按右摇杆可软件停止。','muted'))
        bottom=W.QHBoxLayout();bottom.setContentsMargins(28,0,28,15);self.footer=label('工作台待机  ·  打开界面不会启动控制','muted');bottom.addWidget(self.footer);bottom.addStretch();bottom.addWidget(label('1:1 位姿映射  /  Servo','muted'));column.addLayout(bottom)
        for button in self.findChildren(W.QPushButton):button.setMinimumHeight(max(40,button.minimumHeight()))
        self.setStyleSheet(STYLE);self.set_page(0)
    def page_base(self,title,subtitle):
        page=W.QWidget();layout=W.QVBoxLayout(page);layout.setContentsMargins(0,0,0,0);layout.setSpacing(18);layout.addWidget(label(title,'heading'));layout.addWidget(label(subtitle,'muted'));return page,layout
    def teleop_page(self):
        page,layout=self.page_base('遥操作','观察反馈，掌握状态。三维机器人与场景在 RViz 窗口中同步显示。')
        card,box=frame();row=W.QHBoxLayout();row.addWidget(label('关节反馈','section'));row.addStretch();self.feedback_label=label('等待会话','muted');self.feedback_label.setWordWrap(False);self.feedback_label.setMinimumWidth(115);row.addWidget(self.feedback_label);box.addLayout(row)
        grid=W.QGridLayout();grid.setHorizontalSpacing(22);grid.setVerticalSpacing(14);self.joint_values=[]
        for i in range(6):
            cell=W.QVBoxLayout();name=label('●  J'+str(i+1),'muted');name.setStyleSheet('color:'+COLORS[i]);cell.addWidget(name);value=label('—','value');cell.addWidget(value);self.joint_values.append(value);grid.addLayout(cell,i//6,i%6)
        box.addLayout(grid);box.addWidget(rule());self.plot=JointPlot();box.addWidget(self.plot,1)
        row=W.QHBoxLayout();row.addWidget(label('最近 30 秒 · 单位 °','muted'));row.addStretch();row.addWidget(label('显示采样 1 Hz','muted'));box.addLayout(row);layout.addWidget(card,1)
        card,box=frame();box.addWidget(label('操作提示','section'));box.addWidget(label('头显显示「可以开始遥操作」时，握住右侧 Grip 跟随，松开暂停。'))
        self.latest_event=label('设备连接和权限设置可在左侧“设备连接”中完成。','muted');box.addWidget(self.latest_event);layout.addWidget(card)
        self.joint_label=W.QLabel();self.joint_label.hide()
        return page
    def device_page(self):
        page,layout=self.page_base('设备连接','连接头显，准备控制环境。设备状态由当前会话读取。')
        card,box=frame();box.addWidget(label('01  /  Quest 3S','section'));box.addWidget(label('通过 USB 传输右手柄位姿。头显浏览器地址：'))
        box.addWidget(label('https://localhost:8443','section'));box.addWidget(label('连接后在头显接受本机证书并进入透视模式。','muted'))
        self.quest=W.QPushButton('连接 / 重连 Quest USB');self.quest.clicked.connect(self.connect_quest);box.addWidget(self.quest);layout.addWidget(card)
        card,box=frame();box.addWidget(label('02  /  实时控制权限','section'));self.rt_label=label('尚未启用');box.addWidget(self.rt_label)
        box.addWidget(label('仅对本次工作台会话生效。关闭工作台后，下次使用真机时重新认证。','muted'))
        self.rt=W.QPushButton('启用本次实时权限');self.rt.clicked.connect(self.authorize);box.addWidget(self.rt);layout.addWidget(card)
        card,box=frame();box.addWidget(label('03  /  机械臂','section'));box.addWidget(label('JAKA S5    ·    192.168.110.224'))
        box.addWidget(label('真机启动时自动核对反馈、TCP、负载和机械臂状态。','muted'));layout.addWidget(card);layout.addStretch();return page
    def check_page(self):
        page,layout=self.page_base('检查与验收','开发调试时可运行这些检查，结果保存在本机。')
        card,box=frame();box.addWidget(label('会话检查','section'));self.check_label=label('读取记录中');box.addWidget(self.check_label)
        self.check_summary=label('','muted');box.addWidget(self.check_summary)
        row=W.QHBoxLayout();self.test=W.QPushButton('运行 60 秒仿真检查');self.test.clicked.connect(lambda:self.launch('commission'));row.addWidget(self.test)
        self.prepare=W.QPushButton('实机准备检查');self.prepare.clicked.connect(lambda:self.launch('prepare'));row.addWidget(self.prepare);box.addLayout(row);layout.addWidget(card)
        card,box=frame();box.addWidget(label('头显引导','section'))
        box.addWidget(label('头显会逐步提示轻移手柄、松开 Grip 和按下右摇杆。'))
        box.addWidget(rule());box.addWidget(label('真机检查窗口：60 秒 · 单关节 30° · 末端 1000 mm\n这些是本轮检查的退出阈值，速度与碰撞保护保持原配置。','muted'));layout.addWidget(card)
        card,box=frame();box.addWidget(label('实机结果确认','section'));box.addWidget(label('完整检查通过后，保存你在现场观察到的跟随和停止结果。','muted'))
        self.observed=RowCheckBox('方向正确，松 Grip 和按右摇杆后均停住');self.observed.toggled.connect(self.refresh);box.addWidget(self.observed)
        self.accept=W.QPushButton('保存开发验收记录');self.accept.clicked.connect(self.accept_result);box.addWidget(self.accept);layout.addWidget(card);layout.addStretch();return page
    def log_page(self):
        page,layout=self.page_base('运行日志','查看启动进度、输入状态与退出原因。')
        self.logs=W.QPlainTextEdit();self.logs.setReadOnly(True);self.logs.setMaximumBlockCount(2000);self.logs.setPlaceholderText('启动会话后显示运行日志。');layout.addWidget(self.logs,1);return page
    def set_page(self,index):
        self.pages.setCurrentIndex(index);self.nav[index].setChecked(True)
        if hasattr(self,'session_controls'):self.session_controls.setVisible(index!=4)
        if index==2 and hasattr(self,'access_next'):
            self.request_access_probe();self.refresh()
    def mode_changed(self):
        if not hasattr(self,'site'):return
        self.site.setChecked(False);self.refresh()
    def refresh_view(self,busy,hardware):
        self.mode_badge.setText('真机模式' if hardware else '仿真模式');self.mode_badge.setStyleSheet('color:#a56b1c;background:#fff4df;border-color:#efdbb9;' if hardware else '')
        self.site_hint.setVisible(hardware)
        self.phase_label.setText('停止中' if self.stop_requested and busy else '运行中' if busy else '待机')
        self.footer.setText(('真机' if hardware else '仿真')+'会话运行中' if busy else '工作台待机  ·  打开界面不会启动控制')
        if not busy:
            if self.pages.currentIndex()==2:self.check_summary.setText(self.report_summary())
            self.clear_feedback()
    def report_summary(self):
        import json
        from operator_panel import ROOT
        lines=[]
        for name,folder in [('仿真','fake'),('真机','hardware')]:
            try:
                d=json.loads((ROOT/'artifacts'/('quest_commissioning_'+folder)/'result.json').read_text());state='通过' if d.get('passed') else '未通过 / 未完成'
            except Exception:state='尚无记录'
            lines.append('最近'+name+'记录：'+state)
        return '\n'.join(lines)
    def clear_feedback(self):
        self.feedback_at=0;self.plot.fresh=False;self.plot.update();self.feedback_label.setText('无当前反馈')
        for value in self.joint_values:value.setText('—')
    def event_view(self,event):
        now=time.monotonic();self.last_op_at=now
        values=event.get('joint_degrees',[])
        if event.get('joint_feedback_fresh') is True and len(values)==6 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
            self.feedback_at=now;self.plot.push(values);self.feedback_label.setText('● 反馈接收中')
            for item,value in zip(self.joint_values,values):item.setText(f'{value:.1f}°')
        elif 'joint_feedback_fresh' in event:self.clear_feedback()
        mode=event.get('control_mode');modes={'BOOT':'启动中','IDLE':'待操作','FOLLOW':'跟随中','STOPPING':'减速停止','FAULT':'故障','HOLD':'保持中'}
        if mode and not self.stop_requested:self.phase_label.setText(modes.get(mode,mode))
        if event.get('error'):self.latest_event.setText(event.get('message',''))
    def tick_view(self):
        if self.job is not None:
            elapsed=max(0,int(time.monotonic()-self.job_started_at));self.elapsed.setText(f'{elapsed//60:02d}:{elapsed%60:02d}')
        if self.feedback_at and time.monotonic()-self.feedback_at>2:self.clear_feedback()
        self.plot.update()
