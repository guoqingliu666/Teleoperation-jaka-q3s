"""Qt background supervision for the independent Quest transport."""
import json
from PyQt5 import QtCore
from operator_settings import ROOT

class QuestLink(QtCore.QObject):
    changed=QtCore.pyqtSignal()
    def __init__(self,parent=None):
        super().__init__(parent);self.process=None;self.reconnect=None;self.state={};self.buffer='';self.stopping=False
    def start(self):
        if self.process is not None or self.stopping:return
        p=QtCore.QProcess(self);self.process=p;p.setWorkingDirectory(str(ROOT));p.readyReadStandardOutput.connect(self.consume);p.finished.connect(self.finished)
        p.errorOccurred.connect(lambda error:self.finished() if error==QtCore.QProcess.FailedToStart else None)
        p.start('/usr/bin/python3',['-B',str(ROOT/'tools/operator_quest.py'),'--watch'])
    def consume(self):
        if self.process is None:return
        lines=(self.buffer+bytes(self.process.readAllStandardOutput()).decode(errors='replace')).split('\n');self.buffer=lines.pop()
        for line in lines:
            try:self.state=json.loads(line)
            except ValueError:continue
            self.changed.emit()
    def finished(self,*_):
        if self.process is None:return
        self.consume();p=self.process;self.process=None;p.deleteLater()
        if not self.stopping:
            self.state=dict(ok=False,detail='Quest 连接检查已退出，正在重新检查');self.changed.emit();QtCore.QTimer.singleShot(3000,self.start)
    def repair(self):
        if self.reconnect is not None or self.stopping:return
        p=QtCore.QProcess(self);self.reconnect=p;p.setWorkingDirectory(str(ROOT))
        def done(*_):
            if self.reconnect is not p:return
            try:self.state=json.loads(bytes(p.readAllStandardOutput()).decode().strip().splitlines()[-1])
            except (ValueError,IndexError):self.state=dict(ok=False,detail='USB 重连未完成，请检查数据线和授权')
            self.reconnect=None;p.deleteLater();self.changed.emit()
        p.finished.connect(done);p.errorOccurred.connect(lambda e:done() if e==QtCore.QProcess.FailedToStart else None)
        p.start('/usr/bin/python3',['-B',str(ROOT/'tools/operator_quest.py'),'--force'])
    def stop(self):
        self.stopping=True
        for name in ('reconnect','process'):
            p=getattr(self,name)
            if p is not None:
                p.finished.disconnect();p.terminate();p.waitForFinished(16000);setattr(self,name,None);p.deleteLater()
