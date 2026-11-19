"""One-click, keyboard-accessible light/dark switch with a short thumb animation."""
from PyQt5 import QtCore,QtGui,QtWidgets as W

class ThemeSwitch(W.QAbstractButton):
    valueChanged=QtCore.pyqtSignal(int)
    def __init__(self,value=100,parent=None):
        super().__init__(parent);self.setCheckable(True);self.setFixedSize(52,30)
        self.setCursor(QtCore.Qt.PointingHandCursor);self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setAccessibleName('深色模式');self.setToolTip('点击切换浅色 / 深色')
        self.setChecked(value<50);self._position=float(self.isChecked())
        self.animation=QtCore.QPropertyAnimation(self,b'position',self);self.animation.setDuration(180);self.animation.setEasingCurve(QtCore.QEasingCurve.InOutCubic)
        self.toggled.connect(self.animate)
    def animate(self,dark):
        self.animation.stop();self.animation.setStartValue(self._position);self.animation.setEndValue(float(dark));self.animation.start();self.valueChanged.emit(0 if dark else 100)
    @QtCore.pyqtProperty(float)
    def position(self):return self._position
    @position.setter
    def position(self,value):self._position=value;self.update()
    def value(self):return 0 if self.isChecked() else 100
    def setValue(self,value):self.setChecked(value<50)
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor('#3478f6' if self.isChecked() else '#c5c7ce'));p.drawRoundedRect(QtCore.QRectF(1,2,50,26),13,13)
        x=3+24*self._position;p.setBrush(QtGui.QColor('#ffffff'));p.drawEllipse(QtCore.QRectF(x,4,22,22))
        p.setPen(QtGui.QPen(QtGui.QColor('#697389'),1.4));p.setFont(QtGui.QFont('DejaVu Sans',11));p.drawText(QtCore.QRectF(x,3,22,22),QtCore.Qt.AlignCenter,'☾' if self.isChecked() else '☀')
        if self.hasFocus():p.setPen(QtGui.QPen(QtGui.QColor('#0a84ff'),1));p.setBrush(QtCore.Qt.NoBrush);p.drawRoundedRect(QtCore.QRectF(.5,.5,51,29),14,14)
