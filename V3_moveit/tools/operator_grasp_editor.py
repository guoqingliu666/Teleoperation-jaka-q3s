"""Five-finger animation timeline and inspector for saved DH116 actions."""
import copy
import time
from PyQt5 import QtCore, QtGui, QtWidgets as W
from grasp_profiles import FINGERS, AXIS_FINGER, MAX_SPEED, action_settings
from operator_view import label

class FingerTimeline(W.QWidget):
    selected = QtCore.pyqtSignal(int)
    endpoint = QtCore.pyqtSignal(int, str, int)
    scrubbed = QtCore.pyqtSignal(float)
    left, top, row = 60, 28, 27

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(250,self.top+5*self.row+5)
        self.setMouseTracking(True)
        self.duration=1000
        self.tracks=[dict(start_ms=0,end_ms=1000,force_n=3.) for _ in FINGERS]
        self.index=0;self.progress=0.;self.drag=None
        self.setAccessibleName('五指动作时间轴')
        self.setToolTip('拖动轨道两端调整起止时间，拖动上方标尺预览动作。')

    def x(self,ms):
        return self.left+(self.width()-self.left-16)*ms/self.duration

    def fraction(self,x):
        return max(0.,min(1.,(x-self.left)/max(1,self.width()-self.left-16)))

    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
        dark=self.palette().window().color().lightness()<120
        ink=QtGui.QColor('#ececf2' if dark else '#353641')
        grid=QtGui.QColor('#42434a' if dark else '#dfe3eb')
        accent=QtGui.QColor('#388eff' if dark else '#006ee6')
        p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor('#25262a' if dark else '#f5f6f9'))
        p.drawRoundedRect(self.rect(),9,9)
        for n in range(5):
            x=round(self.x(self.duration*n/4))
            p.setPen(grid);p.drawLine(x,self.top-4,x,self.height()-4)
            p.setPen(ink);p.drawText(QtCore.QRect(x-24,2,48,20),QtCore.Qt.AlignCenter,f'{self.duration*n/4000:g}s')
        for i,(name,t) in enumerate(zip(FINGERS,self.tracks)):
            y=self.top+i*self.row
            if i==self.index:
                p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor(70,135,245,26));p.drawRoundedRect(2,y,self.width()-4,self.row,5,5)
            p.setPen(ink);p.drawText(9,y+18,name)
            a,b=self.x(t['start_ms']),self.x(t['end_ms']);mid=y+self.row/2
            color=accent if i==self.index else QtGui.QColor('#8599b4')
            p.setPen(QtGui.QPen(color,4,QtCore.Qt.SolidLine,QtCore.Qt.RoundCap))
            p.drawLine(QtCore.QPointF(a,mid),QtCore.QPointF(b,mid));p.setBrush(color);p.setPen(QtCore.Qt.NoPen)
            for x in (a,b):
                p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x,mid-5),QtCore.QPointF(x+5,mid),QtCore.QPointF(x,mid+5),QtCore.QPointF(x-5,mid)]))
        x=int(self.x(self.progress*self.duration));p.setPen(QtGui.QPen(QtGui.QColor('#e76d44'),1.5));p.drawLine(x,self.top-4,x,self.height()-3)
        p.end()

    def mousePressEvent(self,e):
        if e.button()!=QtCore.Qt.LeftButton:return
        if e.y()<self.top:
            self.drag='cursor';self.scrubbed.emit(self.fraction(e.x()));return
        i=max(0,min(4,(e.y()-self.top)//self.row));self.selected.emit(i)
        track=self.tracks[i]
        key=min(('start_ms','end_ms'),key=lambda k:abs(self.x(track[k])-e.x()))
        if abs(self.x(track[key])-e.x())<=10:self.drag=key

    def mouseMoveEvent(self,e):
        if not self.drag or not (e.buttons()&QtCore.Qt.LeftButton):return
        f=self.fraction(e.x())
        if self.drag=='cursor':self.scrubbed.emit(f)
        else:self.endpoint.emit(self.index,self.drag,round(f*self.duration))

    def mouseReleaseEvent(self,e):
        self.drag=None


class ActionEditor(W.QWidget):
    preview = QtCore.pyqtSignal(float)
    def __init__(self,owner):
        super().__init__();self.owner=owner;self.loading=False
        self.setObjectName('graspActionEditor')
        box=W.QVBoxLayout(self);box.setContentsMargins(4,8,4,2);box.setSpacing(6)
        files=W.QHBoxLayout();owner.grasp_choice=W.QComboBox();owner.grasp_choice.addItems(['方案一','方案二','方案三'])
        owner.grasp_load=W.QPushButton('加载');owner.grasp_save=W.QPushButton('保存并应用')
        owner.grasp_load.clicked.connect(owner.load_grasp);owner.grasp_save.clicked.connect(owner.save_grasp)
        owner.grasp_name=W.QLineEdit();owner.grasp_name.setMaxLength(24);owner.grasp_name.setPlaceholderText('动作名称')
        for w in (owner.grasp_choice,owner.grasp_name,owner.grasp_load,owner.grasp_save):files.addWidget(w)
        box.addLayout(files)
        transport=W.QHBoxLayout();transport.addWidget(label('动作时间轴','section'));transport.addStretch()
        self.play=W.QPushButton('▶ 预览');self.play.setAccessibleName('播放动作预览');self.play.clicked.connect(self.toggle)
        transport.addWidget(self.play);self.duration=self.spin(.1,30,.05,' s',3);transport.addWidget(W.QLabel('时长'));transport.addWidget(self.duration);self.follow_input=W.QCheckBox('预览扳机');self.follow_input.setChecked(True);transport.addWidget(self.follow_input);box.addLayout(transport)
        self.timeline=FingerTimeline();box.addWidget(self.timeline)
        self.finger=W.QComboBox();self.finger.addItems(FINGERS)
        self.start=self.spin(0,30,.025,' s',3);self.end=self.spin(.001,30,.025,' s',3);self.force=self.spin(.1,20,.1,' N')
        self.target=W.QSpinBox();self.target.setRange(0,10000);self.target.setSuffix(' eq')
        self.side=W.QSpinBox();self.side.setRange(0,10000);self.side.setSuffix(' eq');self.side_title=W.QLabel('拇指侧摆')
        fields=W.QGridLayout();fields.setHorizontalSpacing(8)
        for i,(title,widget) in enumerate([('手指',self.finger),('接触阈值',self.force),('开始',self.start),('结束',self.end),('握合位置',self.target),(self.side_title,self.side)]):
            tag=title if isinstance(title,W.QWidget) else W.QLabel(title)
            fields.addWidget(tag,i//2,(i%2)*2);fields.addWidget(widget,i//2,(i%2)*2+1)
        box.addLayout(fields)
        settings=W.QGridLayout()
        self.closing,closing_field=self.speed_control();self.opening,opening_field=self.speed_control()
        owner.grasp_force=self.spin(.1,20,.1,' N');owner.grasp_current=W.QSpinBox();owner.grasp_current.setRange(1,900);owner.grasp_current.setSuffix(' ‰')
        for i,(title,w) in enumerate([('握合速度',closing_field),('张开速度',opening_field),('整手保护',owner.grasp_force),('电流上限',owner.grasp_current)]):
            settings.addWidget(W.QLabel(title),i//2,(i%2)*2);settings.addWidget(w,i//2,(i%2)*2+1)
        box.addLayout(settings)
        owner.grasp_progress=W.QProgressBar();owner.grasp_progress.setFormat('右 Trigger  %p%');owner.grasp_progress.setRange(0,100);box.addWidget(owner.grasp_progress)
        row=W.QHBoxLayout();owner.grasp_input_label=label('等待右手柄','muted');row.addWidget(owner.grasp_input_label,1)
        owner.grasp_enable=W.QPushButton('切到右扳机');owner.grasp_enable.clicked.connect(lambda:owner.request_grasp_mode('control'));row.addWidget(owner.grasp_enable)
        owner.grasp_reset=W.QPushButton('重置输入');owner.grasp_reset.clicked.connect(owner.reset_grasp_input);row.addWidget(owner.grasp_reset);box.addLayout(row)
        self.timer=QtCore.QTimer(self);self.timer.setInterval(24);self.timer.timeout.connect(self.advance)
        self.finger.currentIndexChanged.connect(self.select);self.timeline.selected.connect(self.finger.setCurrentIndex)
        self.timeline.endpoint.connect(self.edit_endpoint);self.timeline.scrubbed.connect(self.scrub)
        self.duration.valueChanged.connect(self.resize_tracks)
        for w in (self.start,self.end,self.force):w.valueChanged.connect(self.edit_track)
        self.target.valueChanged.connect(lambda v:self.write_target(self.timeline.index+1,v))
        self.side.valueChanged.connect(lambda v:self.write_target(0,v))
        owner.grasp_force.valueChanged.connect(self.limit_forces)
        self.closing.setToolTip('100% = 20000 当量/秒。速度限制运动快慢，扳机位置决定握到哪里。')
        self.opening.setToolTip(self.closing.toolTip())
        self.force.setToolTip('达到该指的法向力合计时停止继续闭合；反向目标允许张开。')
        self.play.setToolTip('预览时间轴，不向设备发送目标。')

    @staticmethod
    def spin(low,high,step,suffix,decimals=2):
        w=W.QDoubleSpinBox();w.setDecimals(decimals);w.setRange(low,high);w.setSingleStep(step);w.setSuffix(suffix);w.setKeyboardTracking(False);return w

    def speed_control(self):
        field=W.QWidget();row=W.QHBoxLayout(field);row.setContentsMargins(0,0,0,0);row.setSpacing(5)
        slider=W.QSlider(QtCore.Qt.Horizontal);slider.setRange(1,200);slider.setMinimumWidth(36)
        value=self.spin(.5,100,1,' %',3);value.setButtonSymbols(W.QAbstractSpinBox.NoButtons)
        value.setFixedWidth(82);value.setAlignment(QtCore.Qt.AlignRight)
        def sync_slider(v):
            with QtCore.QSignalBlocker(slider):slider.setValue(round(v*2))
        value.valueChanged.connect(sync_slider);slider.valueChanged.connect(lambda n:value.setValue(n/2))
        row.addWidget(slider,1);row.addWidget(value);slider.setAccessibleName('动作速度百分比')
        return value,field

    def load(self,p):
        self.timer.stop();self.play.setText('▶ 预览');self.loading=True
        a=action_settings(p);self.timeline.duration=a['duration_ms'];self.timeline.tracks=a['tracks'];self.timeline.progress=0
        self.duration.setValue(a['duration_ms']/1000);self.opening.setValue(a['open_percent']);self.closing.setValue(a['close_percent'])
        self.loading=False;self.select(self.finger.currentIndex())

    def values(self):
        return dict(duration_ms=self.timeline.duration,tracks=copy.deepcopy(self.timeline.tracks),
                    close_percent=self.closing.value(),open_percent=self.opening.value())

    def select(self,index):
        if self.loading or self.owner.grasp_loading:return
        self.loading=True;tl=self.timeline;tl.index=index;t=tl.tracks[index];self.end.setMaximum(tl.duration/1000)
        self.start.setMaximum((t['end_ms']-1)/1000);self.start.setValue(t['start_ms']/1000)
        self.end.setMinimum((t['start_ms']+1)/1000);self.end.setValue(t['end_ms']/1000)
        self.force.setMaximum(self.owner.grasp_force.value());self.force.setValue(t['force_n'])
        endpoints=self.owner.grasp_endpoints;self.target.setValue(endpoints[index+1][1].value());self.side.setValue(endpoints[0][1].value())
        self.side.setVisible(index==0);self.side_title.setVisible(index==0)
        self.loading=False;tl.update()

    def write_target(self,axis,v):
        if not self.loading and not self.owner.grasp_loading:self.owner.grasp_endpoints[axis][1].setValue(v)

    def edit_track(self):
        if self.loading or self.owner.grasp_loading:return
        t=self.timeline.tracks[self.timeline.index]
        t.update(start_ms=round(self.start.value()*1000),end_ms=round(self.end.value()*1000),force_n=self.force.value())
        self.select(self.timeline.index)

    def edit_endpoint(self,i,key,v):
        if self.loading:return
        t=self.timeline.tracks[i]
        t[key]=max(0,min(t['end_ms']-1,v)) if key=='start_ms' else min(self.timeline.duration,max(t['start_ms']+1,v))
        self.select(i)

    def limit_forces(self,value):
        if self.loading or self.owner.grasp_loading:return
        for t in self.timeline.tracks:t['force_n']=min(value,t['force_n'])
        self.select(self.timeline.index)

    def resize_tracks(self,value):
        if self.loading or self.owner.grasp_loading:return
        new=round(value*1000);tl=self.timeline;ratio=new/tl.duration
        for t in tl.tracks:
            t['start_ms']=min(new-1,round(t['start_ms']*ratio));t['end_ms']=min(new,max(t['start_ms']+1,round(t['end_ms']*ratio)))
        tl.duration=new;self.select(tl.index)

    def scrub(self,phase):
        self.follow_input.setChecked(False);self.timer.stop();self.play.setText('▶ 预览');self.position(phase)

    def position(self,phase):
        self.timeline.progress=phase;self.timeline.update();self.preview.emit(phase)

    def toggle(self):
        if self.timer.isActive():self.scrub(self.timeline.progress);return
        phase=0 if self.timeline.progress>=1 else self.timeline.progress
        self.follow_input.setChecked(False);self.began=time.monotonic()-phase*self.timeline.duration/1000;self.timer.start();self.play.setText('Ⅱ 暂停')

    def advance(self):
        phase=min(1.,(time.monotonic()-self.began)*1000/self.timeline.duration);self.position(phase)
        if phase>=1:self.timer.stop();self.play.setText('▶ 预览')
