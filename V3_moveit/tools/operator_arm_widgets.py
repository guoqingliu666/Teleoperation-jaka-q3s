"""Compact, anchored cabinet readback popover. No device operations."""
from PyQt5 import QtCore,QtGui,QtWidgets as W

class ToolingPopover(W.QFrame):
    def __init__(self,parent):
        super().__init__(parent,QtCore.Qt.Popup|QtCore.Qt.FramelessWindowHint)
        self.setObjectName('toolingPopover');self.setMinimumWidth(480);self.setMaximumWidth(560)
        box=W.QVBoxLayout(self);box.setContentsMargins(18,16,18,16);box.setSpacing(12)
        row=W.QHBoxLayout();title=W.QLabel('工具与负载');title.setStyleSheet('font-size:15px;font-weight:600;');row.addWidget(title);row.addStretch()
        close=W.QToolButton();close.setText('×');close.setAccessibleName('关闭工具与负载');close.clicked.connect(self.hide);row.addWidget(close);box.addLayout(row)
        self.status=W.QLabel('正在读取 JAKA…');self.status.setWordWrap(True);box.addWidget(self.status)
        grid=W.QGridLayout();grid.setHorizontalSpacing(18);grid.setVerticalSpacing(9);box.addLayout(grid)
        for col,text in enumerate(('参数','V3 配置','JAKA 回读')):grid.addWidget(W.QLabel(text),0,col)
        self.cells={}
        for i,(key,title) in enumerate([('tool','工具'),('tcp_xyz','TCP XYZ / mm'),('tcp_rpy','TCP Rx Ry Rz / rad'),('mass','质量 / kg'),('cog','质心 XYZ / mm')],1):
            grid.addWidget(W.QLabel(title),i,0)
            for col in (1,2):
                label=W.QLabel('—');label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse);label.setWordWrap(True);grid.addWidget(label,i,col);self.cells[key,col]=label
        self.note=W.QLabel('质心变化后，请先在 JAKA 上位机重新检测并保存，再刷新 V3 负载。');self.note.setWordWrap(True);box.addWidget(self.note)
    def set_theme(self,card,fg,muted,border):
        self.setStyleSheet(f'QFrame#toolingPopover{{background:{card};color:{fg};border:1px solid {border};border-radius:14px;}} QLabel,QToolButton{{background:transparent;color:{fg};border:0;}}')
        self.status.setStyleSheet(f'color:{muted};');self.note.setStyleSheet(f'color:{muted};font-size:11px;')
    def update_values(self,config,observed,detail=''):
        from operator_tooling import tool_values
        for col,data in ((1,config),(2,observed)):
            if not data:
                for key in ('tool','tcp_xyz','tcp_rpy','mass','cog'):self.cells[key,col].setText('—')
                continue
            tool,tcp=tool_values(data)
            values=dict(tool='法兰中心' if tool==0 else 'TCP'+str(tool),
                tcp_xyz='\n'.join(f'{axis}  {v:.3f}' for axis,v in zip(('X','Y','Z'),tcp[:3])),
                tcp_rpy='\n'.join(f'{axis}  {v:.5f}' for axis,v in zip(('Rx','Ry','Rz'),tcp[3:])),
                mass=f"{data['payload_kg']:.6f}",cog='\n'.join(f'{axis}  {v:.6f}' for axis,v in zip(('X','Y','Z'),data['cog_mm'])))
            for key,text in values.items():self.cells[key,col].setText(text);self.cells[key,col].setToolTip(str(data))
        self.status.setText(detail or ('已读取控制柜数据' if observed else '正在读取 JAKA…'));self.adjustSize()
    def show_at(self,anchor):
        self.adjustSize();point=anchor.mapToGlobal(QtCore.QPoint(0,anchor.height()+7));screen=anchor.screen().availableGeometry()
        point.setX(max(screen.left()+8,min(point.x(),screen.right()-self.width()-8)))
        if point.y()+self.height()>screen.bottom():point.setY(anchor.mapToGlobal(QtCore.QPoint(0,0)).y()-self.height()-7)
        self.move(point);self.show()
