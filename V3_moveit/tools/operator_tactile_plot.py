"""Bounded raw-sample history with PyQtGraph peak-preserving display reduction."""
import math,time
from collections import deque
import numpy as np
from operator_visual_backend import prepare_opengl
prepare_opengl()
import pyqtgraph as pg
from PyQt5 import QtCore,QtWidgets as W

COLORS=('#0a84ff','#30b995','#af77e9','#ed9c37','#ec6b86')
NAMES=('拇指','食指','中指','无名指','小指')

def resultant(f):
    normal=sum(f.get('normal_n',[]));tx=ty=0.;valid=True
    for force,angle in zip(f.get('tangent_n',[]),f.get('direction_deg',[])):
        if force and angle is None:valid=False;continue
        if angle is not None:tx+=force*math.cos(math.radians(angle));ty+=force*math.sin(math.radians(angle))
    return np.array([tx,ty,normal]),valid

class TactilePlot(W.QWidget):
    def __init__(self,parent=None):
        super().__init__(parent);self.samples=deque(maxlen=12000);self.last_stamp=-1.;self.last_key=None;self.last_draw=0;self.next_draw=0;self.paused=False;self.draws=deque(maxlen=240);self.arrivals=deque(maxlen=1000);self.display_quantity=None
        layout=W.QVBoxLayout(self);layout.setContentsMargins(0,0,0,0);layout.setSpacing(3);row=W.QHBoxLayout()
        self.quantity=W.QComboBox();self.quantity.addItems(['合力 · N','法向 · N','切向 · N','压力通道峰值 · %']);row.addWidget(self.quantity)
        self.window=W.QComboBox();self.window.addItems(['5 s','10 s','30 s']);self.window.setFixedWidth(73);row.addWidget(self.window)
        self.pause=W.QPushButton('暂停');self.pause.setCheckable(True);self.pause.toggled.connect(self.freeze);row.addWidget(self.pause);row.addStretch();layout.addLayout(row)
        self.plot=pg.PlotWidget();self.plot.setMinimumHeight(130);self.plot.setMenuEnabled(False);self.plot.hideButtons();self.plot.showGrid(x=True,y=True,alpha=.14);self.plot.setMouseEnabled(x=False,y=False)
        self.plot.setLabel('bottom','时间',units='s');self.plot.setLabel('left','合力',units='N');self.plot.addLegend(offset=(8,1),horSpacing=14,colCount=5,labelTextSize='9pt')
        self.curves=[self.plot.plot(name=n,pen=pg.mkPen(c,width=1.5),connect='finite',autoDownsample=True,clipToView=True,downsampleMethod='peak') for n,c in zip(NAMES,COLORS)]
        layout.addWidget(self.plot,1);self.timer=QtCore.QTimer(self);self.timer.setTimerType(QtCore.Qt.PreciseTimer);self.timer.timeout.connect(self.render);self.timer.start(8)
        self.quantity.currentIndexChanged.connect(lambda:self.render(force=True));self.window.currentIndexChanged.connect(lambda:self.render(force=True))
    def freeze(self,yes):self.paused=yes;self.pause.setText('继续' if yes else '暂停')
    def ingest(self,state):
        stamp=state.get('updated',-1);key=(state.get('sequence'),stamp)
        if not state or key==self.last_key or stamp<=self.last_stamp:return
        values=[]
        for f in state['fingers']:
            vector,valid=resultant(f);pressure=f.get('pressure',[])
            values.append([float(np.linalg.norm(vector)) if valid else math.nan,vector[2],float(np.hypot(vector[0],vector[1])) if valid else math.nan,max(pressure)*100 if pressure and f.get('pressure_unit')=='normalized' else math.nan])
        if self.last_stamp>=0 and stamp-self.last_stamp>.08:self.samples.append((stamp-1e-6,np.full((5,4),math.nan)))
        # Convert once on arrival; rebuilding thousands of nested lists every
        # frame would consume much of the 120 Hz display budget.
        self.samples.append((stamp,np.asarray(values,float)));self.last_stamp=stamp;self.last_key=key
        self.arrivals.append(stamp)
        while self.samples and stamp-self.samples[0][0]>30:self.samples.popleft()
    def render(self,force=False):
        if self.paused or (not force and not self.isVisible()):return
        now=time.monotonic()
        if not force and now<self.next_draw:return
        period=1/120
        self.next_draw=now+period if now-self.next_draw>period else self.next_draw+period
        self.last_draw=now;quantity=self.quantity.currentIndex();span=(5,10,30)[self.window.currentIndex()]
        if quantity!=self.display_quantity:
            title,unit=[('合力','N'),('法向','N'),('切向','N'),('通道强度','%')][quantity];self.plot.setLabel('left',title,units=unit);self.display_quantity=quantity
        self.plot.setXRange(-span,0,padding=0)
        if self.samples:
            x=np.fromiter((s[0]-now for s in self.samples),float);v=np.asarray([s[1] for s in self.samples],float)
            for i,curve in enumerate(self.curves):curve.setData(x,v[:,i,quantity])
            shown=v[x>=-span,:,quantity];finite=shown[np.isfinite(shown)];high=max(1,float(finite.max())*1.15) if finite.size else 1
            self.plot.setYRange(0,100 if quantity==3 else high,padding=0)
        self.draws.append(now)
    def set_theme(self,bg,fg,border):
        self.plot.setBackground(bg)
        for name in ('left','bottom'):
            a=self.plot.getAxis(name);a.setPen(border);a.setTextPen(fg)
        for sample,label in self.plot.plotItem.legend.items:label.setText(label.text,color=fg)
