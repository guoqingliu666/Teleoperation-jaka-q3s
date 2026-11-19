"""A quiet, film-credit style about window; names describe roles, not test status."""
import time
from PyQt5 import QtCore,QtGui,QtWidgets as W

AUTHOR='刘国庆'
SECTIONS=(
    ('产品与策划',('产品构思','用户场景','需求分析','功能定义','版本规划','项目统筹')),
    ('系统与架构',('系统架构','技术选型','模块边界','接口设计','数据流设计','故障恢复设计')),
    ('评审与协作',('需求评审','技术方案评审','设计评审','代码评审','风险评估','里程碑管理')),
    ('交互与视觉',('信息架构','交互设计','视觉设计','设计规范','明暗主题','界面动效')),
    ('机械与模型',('装配梳理','模型整理','坐标系设计','运动学适配','碰撞模型','三维场景')),
    ('机器人控制',('MoveIt Servo 集成','运动规划集成','ros2_control 适配','厂家 SDK 接入','实时反馈','运动约束与停止保护')),
    ('灵巧手与触觉',('EtherCAT 接入','关节控制','抓握方案','三维关节交互','压力可视化','触觉波形')),
    ('空间交互',('Quest 接入','手柄输入映射','位姿跟随','透视工作台','桌面与头显同步','USB 链路维护')),
    ('感知与数据',('相机接入','深度数据处理','多视角监看','时间同步','数据有效性检查','信号可视化')),
    ('工程与实现',('C++ 开发','Python 开发','Qt 桌面开发','工具链搭建','依赖管理','版本管理','自动构建','持续集成')),
    ('安全与可靠性',('状态检查','异常处理','失鲜保护','权限设计','容错策略','安全边界')),
    ('质量与验证',('测试方案','单元测试','集成测试','协议验证','仿真回归','实机联调','性能分析')),
    ('发布与交付',('接入流程','现场验收流程','安装部署','发布检查','回退方案','技术文档','版本交接')),
    ('运行与维护',('运行日志','问题定位','故障修复','兼容性维护','体验打磨','持续迭代')),
)

class InfoButton(W.QAbstractButton):
    def __init__(self,parent=None):
        super().__init__(parent);self.setFixedSize(34,34);self.setCursor(QtCore.Qt.PointingHandCursor);self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setAccessibleName('关于 V3 与开发者名单');self.setToolTip('关于 V3 · 开发者名单')
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing);p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor('#3395ff' if self.underMouse() else '#007aff'));p.drawEllipse(QtCore.QRectF(5,5,24,24))
        font=QtGui.QFont('DejaVu Serif');font.setPixelSize(20);font.setItalic(True);font.setBold(True);p.setFont(font);p.setPen(QtGui.QColor('white'));p.drawText(QtCore.QRectF(5,3,24,26),QtCore.Qt.AlignCenter,'i')
        if self.hasFocus():p.setBrush(QtCore.Qt.NoBrush);p.setPen(QtGui.QPen(QtGui.QColor('#007aff'),1));p.drawEllipse(QtCore.QRectF(2,2,30,30))

class CreditsRoll(W.QAbstractScrollArea):
    playingChanged=QtCore.pyqtSignal(bool)
    def __init__(self,parent=None):
        super().__init__(parent);self.setFrameShape(W.QFrame.NoFrame);self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.offset=0.;self.playing=True;self.last_tick=time.monotonic();self.wait_until=self.last_tick+2;self.bg='#ffffff';self.fg='#1d1d1f';self.muted='#81818a'
        self.rows=[];y=310
        for number,(title,roles) in enumerate(SECTIONS,1):
            self.rows.append(('section',f'{number:02d}   {title}',y));y+=55
            for role in roles:self.rows.append(('role',role,y));y+=38
            y+=66
        self.ending_y=y;self.total_height=y+310
        self.verticalScrollBar().valueChanged.connect(self.seek)
        self.timer=QtCore.QTimer(self);self.timer.setTimerType(QtCore.Qt.PreciseTimer);self.timer.setInterval(16);self.timer.timeout.connect(self.tick)
        self.setAccessibleName('开发者制作名单，空格暂停，滚轮浏览，Home 返回开头')
    def showEvent(self,e):super().showEvent(e);self.last_tick=time.monotonic();self.timer.start()
    def hideEvent(self,e):self.timer.stop();super().hideEvent(e)
    def resizeEvent(self,e):
        super().resizeEvent(e);self.verticalScrollBar().setRange(0,max(0,self.total_height-self.viewport().height()));self.verticalScrollBar().setPageStep(self.viewport().height());self.set_offset(self.offset)
    def set_theme(self,bg,fg,muted):self.bg=bg;self.fg=fg;self.muted=muted;self.viewport().update()
    def set_playing(self,playing):self.playing=bool(playing);self.last_tick=time.monotonic();self.playingChanged.emit(self.playing)
    def seek(self,value):self.offset=float(value);self.set_playing(False);self.viewport().update()
    def set_offset(self,value):
        self.offset=max(0,min(float(value),self.verticalScrollBar().maximum()));bar=self.verticalScrollBar();block=QtCore.QSignalBlocker(bar);bar.setValue(round(self.offset));del block;self.viewport().update()
    def restart(self):self.set_offset(0);self.wait_until=time.monotonic()+2;self.set_playing(True)
    def tick(self):
        now=time.monotonic();dt=min(.08,max(0,now-self.last_tick));self.last_tick=now
        if not self.playing or now<self.wait_until:return
        self.set_offset(self.offset+dt*38)
        if self.offset>=self.verticalScrollBar().maximum():self.set_playing(False)
    def wheelEvent(self,e):self.set_playing(False);super().wheelEvent(e)
    def keyPressEvent(self,e):
        if e.key()==QtCore.Qt.Key_Space:self.set_playing(not self.playing);e.accept()
        elif e.key()==QtCore.Qt.Key_Home:self.restart();e.accept()
        else:super().keyPressEvent(e)
    def paintEvent(self,event):
        p=QtGui.QPainter(self.viewport());p.setRenderHints(QtGui.QPainter.Antialiasing|QtGui.QPainter.TextAntialiasing);rect=QtCore.QRectF(self.viewport().rect());path=QtGui.QPainterPath();path.addRoundedRect(rect,18,18);p.setClipPath(path);p.fillRect(rect,QtGui.QColor(self.bg));width=rect.width();height=rect.height()
        def text(value,y,size,color,align=QtCore.Qt.AlignCenter,x=24,w=None,weight=QtGui.QFont.Normal):
            y-=self.offset
            if y+60<0 or y>height:return
            font=QtGui.QFont('Noto Sans CJK SC');font.setPixelSize(size);font.setWeight(weight);p.setFont(font);p.setPen(QtGui.QColor(color));p.drawText(QtCore.QRectF(x,y,width-48 if w is None else w,48),align|QtCore.Qt.AlignVCenter,value)
        text('DESIGNED & BUILT BY',48,12,self.muted)
        text(AUTHOR,100,40,self.fg,weight=QtGui.QFont.Medium)
        text('一个人，一整支团队。',166,16,self.muted)
        for kind,value,y in self.rows:
            if kind=='section':text(value,y,15,self.muted,weight=QtGui.QFont.Medium)
            else:
                text(value,y,17,self.muted,QtCore.Qt.AlignRight,x=24,w=width/2-45)
                text(AUTHOR,y,18,self.fg,QtCore.Qt.AlignLeft,x=width/2+23,w=width/2-47,weight=QtGui.QFont.Medium)
        text('特别鸣谢',self.ending_y+15,15,self.muted)
        text('仍在认真打磨的自己',self.ending_y+60,22,self.fg)
        text('刘国庆',self.ending_y+108,18,self.fg)
        text('THE END',self.ending_y+198,12,self.muted)
        for start,end in ((0,38),(height,height-38)):
            fade=QtGui.QLinearGradient(0,start,0,end);opaque=QtGui.QColor(self.bg);clear=QtGui.QColor(self.bg);clear.setAlpha(0);fade.setColorAt(0,opaque);fade.setColorAt(1,clear);p.fillRect(QtCore.QRectF(0,min(start,end),width,38),fade)

class AboutDialog(W.QDialog):
    def __init__(self,parent=None):
        super().__init__(parent);self.setWindowTitle('关于 V3 · 开发者名单');self.setWindowModality(QtCore.Qt.NonModal);self.resize(900,780);self.setMinimumSize(700,560)
        box=W.QVBoxLayout(self);box.setContentsMargins(28,24,28,22);box.setSpacing(16);header=W.QHBoxLayout()
        logo=W.QLabel('V3');logo.setObjectName('creditsLogo');logo.setAlignment(QtCore.Qt.AlignCenter);logo.setFixedSize(62,62);header.addWidget(logo)
        titles=W.QVBoxLayout();titles.setSpacing(2);title=W.QLabel('V3 Operator');title.setObjectName('creditsTitle');titles.addWidget(title);subtitle=W.QLabel('机器人遥操作工作台');subtitle.setObjectName('creditsMuted');titles.addWidget(subtitle);header.addLayout(titles);header.addStretch()
        badge=W.QLabel('开发者名单');badge.setObjectName('creditsBadge');header.addWidget(badge);box.addLayout(header)
        self.roll=CreditsRoll();box.addWidget(self.roll,1)
        note=W.QLabel('基于 ROS 2 · MoveIt · Qt · PyQtGraph 及厂家 SDK');note.setObjectName('creditsMuted');note.setAlignment(QtCore.Qt.AlignCenter);box.addWidget(note)
        controls=W.QHBoxLayout();self.pause=W.QPushButton('暂停滚动');self.pause.clicked.connect(lambda:self.roll.set_playing(not self.roll.playing));controls.addWidget(self.pause)
        self.replay=W.QPushButton('从头播放');self.replay.clicked.connect(self.roll.restart);controls.addWidget(self.replay);controls.addStretch()
        hint=W.QLabel('滚轮浏览 · 空格暂停');hint.setObjectName('creditsMuted');controls.addWidget(hint);controls.addSpacing(12)
        self.done=W.QPushButton('完成');self.done.setObjectName('creditsDone');self.done.clicked.connect(self.close);controls.addWidget(self.done);box.addLayout(controls)
        self.roll.playingChanged.connect(lambda yes:self.pause.setText('暂停滚动' if yes else '继续滚动'));self.set_theme('#f5f5f7','#ffffff','#1d1d1f','#81818a','#e1e2e8')
    def set_theme(self,bg,card,fg,muted,border):
        self.setStyleSheet(f'''QDialog{{background:{bg};color:{fg};}} QLabel{{background:transparent;color:{fg};font:14px "Noto Sans CJK SC";}} QLabel#creditsTitle{{font-size:25px;font-weight:600;}} QLabel#creditsMuted{{color:{muted};font-size:12px;}} QLabel#creditsLogo{{background:{fg};color:{card};border-radius:16px;font-size:27px;font-weight:600;}} QLabel#creditsBadge{{color:{muted};background:{card};border:1px solid {border};border-radius:12px;padding:7px 12px;font-size:12px;}} QPushButton{{background:{card};color:{fg};border:1px solid {border};border-radius:10px;padding:8px 16px;font:13px "Noto Sans CJK SC";}} QPushButton:hover{{border-color:#007aff;}} QPushButton#creditsDone{{background:#007aff;color:white;border:1px solid #007aff;}} QAbstractScrollArea,QAbstractScrollArea>QWidget{{background:transparent;border:0;}} QScrollBar:vertical{{background:transparent;width:7px;margin:10px 0;}} QScrollBar::handle:vertical{{background:{border};min-height:32px;border-radius:3px;}} QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{{height:0;}} QScrollBar::add-page:vertical,QScrollBar::sub-page:vertical{{background:transparent;}}''')
        self.roll.set_theme(card,fg,muted)
