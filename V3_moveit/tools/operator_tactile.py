"""Real DH116 tactile values and explicitly local sensor-coordinate arrows."""
import math,time
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets as W
from operator_view import label,CurrentSizeTabs

FINGERS = ('大拇指', '食指', '中指', '无名指', '小拇指')


def decode_tactile(d, now, previous=0):
    stamp, seq = d['received_monotonic'], d['sequence']
    if type(stamp) not in (int, float) or not math.isfinite(stamp) or not 0 <= now-stamp < .08:
        raise ValueError('触觉数据过期')
    if type(seq) is not int or seq <= previous:
        raise ValueError('触觉数据重复')
    raw = bytes.fromhex(d['raw_hex'])
    if len(raw) != 192 or raw[:2] != b'\x40\x06':
        raise ValueError('触觉帧布局不匹配')
    fingers = d['fingers']
    if not isinstance(fingers, list) or len(fingers) != 5:
        raise ValueError('缺少五指触觉反馈')
    result = []
    for f in fingers:
        n, t, a = (f[k] for k in ('normal_n', 'tangent_n', 'direction_deg'))
        if not isinstance(n, list) or not 1 <= len(n) <= 16 or len(n) != len(t) or len(n) != len(a):
            raise ValueError('触点数量不匹配')
        if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 655.35 for v in n+t):
            raise ValueError('触觉数值无效')
        # A missing/invalid direction never becomes zero degrees or a modulo angle.
        angles = [v if type(v) in (int, float) and math.isfinite(v) and 0 <= v <= 360 else None for v in a]
        optional={}
        for key,maximum in (('proximity',65534),('pressure',255)):
            values=f.get(key,[])
            optional[key]=[v if type(v) in (int,float) and math.isfinite(v) and 0<=v<=maximum else None for v in values] if isinstance(values,list) else []
        pressure=f.get('pressure',[]);xy=f.get('pressure_xy',[]);unit=f.get('pressure_unit')
        valid_pressure=(unit=='normalized' and isinstance(pressure,list) and 1<=len(pressure)<=128 and isinstance(xy,list) and len(xy)==len(pressure)
            and all(type(v) in (int,float) and math.isfinite(v) and 0<=v<=1 for v in pressure)
            and all(isinstance(pair,list) and len(pair)==2 and all(type(v) in (int,float) and math.isfinite(v) and 0<=v<=1 for v in pair) for pair in xy))
        optional.update(pressure=pressure if valid_pressure else [],pressure_xy=xy if valid_pressure else [],pressure_unit='normalized' if valid_pressure else None)
        result.append(dict(normal_n=n, tangent_n=t, direction_deg=angles,**optional))
    return dict(updated=stamp, sequence=seq, fingers=result, frame='sensor_local', calibrated=False)


def fresh_tactile(state, now):
    return state if state and 0 <= now-state.get('updated', 0) < .08 else {}


ASSETS=Path(__file__).resolve().parent/'operator_assets/dh116_vendor'

from operator_pressure_hand import PressureHand as HandForces
from operator_tactile_plot import TactilePlot

class TactileMonitor(W.QWidget):
    def __init__(self,parent=None,compact=False):
        super().__init__(parent);self.state={};self.last_seq=None;self.arrivals=[]
        box=W.QVBoxLayout(self);box.setContentsMargins(0,0,0,0);box.setSpacing(4 if compact else 10)
        self.fps=label('离线 · 0 Hz','muted')
        if not compact:
            row=W.QHBoxLayout();row.addWidget(label('触觉监控','section'));row.addStretch();row.addWidget(self.fps);box.addLayout(row)
        self.tabs=CurrentSizeTabs();self.tabs.setObjectName('handTabs');box.addWidget(self.tabs,1)
        heat=W.QWidget();row=W.QHBoxLayout(heat);row.setContentsMargins(0,0,0,0);row.setSpacing(12)
        self.wave=TactilePlot();row.addWidget(self.wave,3)
        self.hand=HandForces();self.hand.setMinimumHeight(190);self.hand.setMinimumWidth(215);self.hand.setMaximumWidth(300);row.addWidget(self.hand,1);self.tabs.addTab(heat,'热力图')
        self.table=W.QTableWidget(0,6);self.table.setHorizontalHeaderLabels(['手指','触点','法向 / N','切向 / N','方向 / °','接近']);self.table.verticalHeader().hide();self.table.setEditTriggers(W.QAbstractItemView.NoEditTriggers);self.table.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch);self.tabs.addTab(self.table,'详细数据')
        settings=W.QWidget();form=W.QFormLayout(settings);form.setContentsMargins(8,4,8,4) if compact else form.setContentsMargins(18,24,18,18)
        self.range=W.QDoubleSpinBox();self.range.setRange(.1,20);self.range.setValue(3);self.range.setSuffix(' N');self.range.valueChanged.connect(self.change_scale);form.addRow('合力箭头满量程',self.range)
        self.show_heat=W.QCheckBox('显示压力通道热力图');self.show_heat.setChecked(True);self.show_heat.toggled.connect(self.change_layers);form.addRow(self.show_heat)
        self.show_arrows=W.QCheckBox('显示法向 + 切向合力箭头');self.show_arrows.setChecked(True);self.show_arrows.toggled.connect(self.change_layers);form.addRow(self.show_arrows)
        form.addRow(label('颜色：手指归一化通道强度 0–100%；波形：真实力 / 通道强度。\n离散分区不是物理接触轮廓；图上位置尚未做毫米标定。\n合力箭头为传感器局部坐标斜投影，不是机器人基坐标方向。','muted'));self.tabs.addTab(settings,'设置')
        self.caption=label('等待真实触觉反馈 · 离线时不绘制受力','muted')
        row=W.QHBoxLayout();row.addWidget(self.caption,1)
        if compact:row.addWidget(self.fps)
        box.addLayout(row)
        self.set_state({})
    def set_theme(self,bg,fg,border):self.wave.set_theme(bg,fg,border)
    def change_scale(self,value):self.hand.scale_n=value;self.hand.update()
    def change_layers(self):self.hand.heat=self.show_heat.isChecked();self.hand.arrows=self.show_arrows.isChecked();self.hand.update()
    def reset_view(self):self.range.setValue(3);self.show_heat.setChecked(True);self.show_arrows.setChecked(True);self.tabs.setCurrentIndex(0)
    def set_state(self,state):
        self.state=state;self.hand.set_state(state);self.wave.ingest(state);now=time.monotonic()
        if state and state.get('sequence')!=self.last_seq:self.last_seq=state.get('sequence');self.arrivals.append(now)
        self.arrivals=[t for t in self.arrivals if now-t<1]
        sampled=sum(0<=now-t<1 for t in self.wave.arrivals);drawn=sum(0<=now-t<1 for t in self.wave.draws)
        self.fps.setText(f'采样 {sampled} Hz · 绘制 {drawn} Hz' if state else '离线 · 0 Hz')
        self.caption.setText('通道强度 0–100% · 合力箭头为局部投影' if state else '等待真实触觉反馈 · 离线时不绘制受力')
        rows=[]
        for name,f in zip(FINGERS,state.get('fingers',[])):
            for i,(n,t,a) in enumerate(zip(f['normal_n'],f['tangent_n'],f['direction_deg'])):
                prox=f.get('proximity',[]);v=prox[i] if i<len(prox) else None
                rows.append([name,str(i+1),f'{n:.2f}',f'{t:.2f}',f'{a:.0f}' if a is not None and t>0 else '—',f'{v:.0f}' if v is not None else '—'])
        if not rows:rows=[[name,'—','—','—','—','—'] for name in FINGERS]
        self.table.setRowCount(len(rows))
        for i,row in enumerate(rows):
            for j,value in enumerate(row):
                item=self.table.item(i,j)
                if item is None:item=W.QTableWidgetItem();item.setTextAlignment(QtCore.Qt.AlignCenter);self.table.setItem(i,j,item)
                if item.text()!=value:item.setText(value)


class TactileColumn(W.QWidget):
    def __init__(self):
        super().__init__();self.setMinimumWidth(215);self.setMaximumWidth(250)
        layout=W.QVBoxLayout(self);layout.setContentsMargins(8,0,0,0);layout.setSpacing(7)
        layout.addWidget(label('五指触觉','section'));self.status=label('等待真实反馈','muted');layout.addWidget(self.status)
        self.rows=[]
        for name in FINGERS:
            row=label(name+' · 无反馈\n法向 — N / 切向 — N','muted');layout.addWidget(row);self.rows.append(row)
        self.hand=HandForces();self.hand.setMinimumSize(180,140);layout.addWidget(self.hand,1)
        layout.addWidget(label('热力图：法向  橙色箭头：切向\n手图 / 3D 箭头均为局部示意\n空间方向待标定','muted'))
    def set_state(self,state):
        self.status.setText('真实触觉 · N' if state else '无新鲜触觉 · 不显示旧箭头')
        for name,row,f in zip(FINGERS,self.rows,state.get('fingers',[{}]*5)):
            if not f:row.setText(name+' · 无反馈\n法向 — N / 切向 — N');continue
            idx=max(range(len(f['tangent_n'])),key=f['tangent_n'].__getitem__);angle=f['direction_deg'][idx]
            row.setText(f"{name}   法向 Σ {sum(f['normal_n']):.2f} N\n切向峰值 {f['tangent_n'][idx]:.2f} N  · "+(f'{angle:.0f}°' if angle is not None else '方向无效'))
            row.setToolTip('\n'.join(f'触点 {i+1}: 法向 {n:.2f} N，切向 {t:.2f} N，方向 {a}' for i,(n,t,a) in enumerate(zip(f['normal_n'],f['tangent_n'],f['direction_deg']))))
        self.hand.set_state(state)
