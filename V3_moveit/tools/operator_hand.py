"""DH116 feedback connection. The native owner has no motor-command path."""
import json,math,time
from pathlib import Path
from PyQt5 import QtCore
from operator_settings import ROOT,atomic
from operator_hand_access import native_command,connection_error

def decode_sample(d, now, previous=0):
    """Validate fresh transport, raw frame, and SDK clipping independently."""
    if d.get('event')!='sample' or d.get('valid') is not True or d.get('wkc')!=1 or d.get('decode_result')!=0:
        raise ValueError('未收到有效 TPDO')
    stamp=float(d['received_monotonic']);seq=d['sequence']
    if not math.isfinite(stamp) or not 0<=now-stamp<.5 or type(seq) is not int or seq<=previous:
        raise ValueError('反馈过期或序号未更新')
    raw=bytes.fromhex(d['raw_hex']);capacity={0:27,3:12,6:6}.get(raw[0] if raw else None)
    if len(raw)!=192 or not capacity or raw[1]!=capacity:raise ValueError('TPDO 布局不匹配')
    positions=[int.from_bytes(raw[2+6*i:4+6*i],'little',signed=True) for i in range(6)]
    states=list(raw[2+capacity*6:8+capacity*6])
    if positions!=d['raw_positions'] or states!=d['axis_status'] or d['positions']!=[max(0,min(10000,p)) for p in positions]:
        raise ValueError('原始反馈与 SDK 解码不一致')
    if not any(states):raise ValueError('尚无有效六轴状态')
    homed=all(s&16 for s in states);alarm=any((s>>5) in (2,3,4,5,6) for s in states)
    info=dict(ok=not alarm,feedback_verified=True,updated=stamp,sequence=seq,raw_positions=positions,
              sdk_positions=d['positions'],axis_status=states,homed=homed,
              currents=[int.from_bytes(raw[4+6*i:6+6*i],'little',signed=True) for i in range(6)],
              velocities=[10*int.from_bytes(raw[6+6*i:8+6*i],'little',signed=True) for i in range(6)],
              detail=('反馈在线 · '+('已回零' if homed else '未回零')+' · 原始位置 '+', '.join(map(str,positions))))
    if alarm:info['detail']='六轴反馈在线 · 状态异常 '+str(states)
    # Until homing is reported, leave the articulated model as a preview.
    if homed:
        names=['finger11','finger12','finger21','finger31','finger41','finger51']
        q={n:math.radians(p/10000*a) for n,p,a in zip(names,positions,[60,30,80,80,80,80])}
        for child,parent,k,b in [('finger13','finger12',1.2334,.004592),('finger22','finger21',1.1411,.011072),('finger32','finger31',1.1411,.011072),('finger42','finger41',1.1376,.011072),('finger52','finger51',1.1516,.011072)]:q[child]=q[parent]*k+b
        info.update(joint_names=list(q),position=list(q.values()))
    return info

class HandConnection(QtCore.QObject):
    changed=QtCore.pyqtSignal()
    def __init__(self,parent=None):
        super().__init__(parent);self.process=None;self.state={};self.buffer='';self.sequence=0;self.closed=False;self.error='';self.log=None;self.stopping=False
    def connect_device(self,interface):
        if self.process is not None:return
        device=Path('/sys/class/net')/interface
        if not (device/'device').exists() or (device/'wireless').exists():raise ValueError('请选择实际有线网口')
        binary=ROOT/'.cache/dh116/build/v3_dh116_feedback'
        if not binary.is_file():raise ValueError('DH116 反馈组件未构建，请运行 tools/build_dh116.sh')
        command,args=native_command(binary,[interface,'0'])
        folder=ROOT/'artifacts/dh116_sessions'/time.strftime('%Y%m%dT%H%M%S');folder.mkdir(parents=True,exist_ok=True)
        self.log=(folder/'capture.jsonl').open('w');self.sequence=0;self.closed=False;self.error='';self.stopping=False;self.buffer=''
        self.state=dict(ok=False,detail='正在连接并等待真实 TPDO',updated=time.monotonic())
        if command=='/usr/bin/pkexec':self.state['detail']='等待系统认证；免密码连接需安装 DH116 网口权限'
        p=QtCore.QProcess(self);self.process=p;p.setWorkingDirectory(str(ROOT));p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        p.readyReadStandardOutput.connect(self.consume);p.finished.connect(self.finished)
        p.errorOccurred.connect(lambda e:self.finished(-1) if e==QtCore.QProcess.FailedToStart else None)
        p.start(command,args);self.changed.emit()
    def consume(self):
        if self.process is None:return
        text=bytes(self.process.readAllStandardOutput()).decode(errors='replace');self.log.write(text);self.log.flush()
        lines=(self.buffer+text).split('\n');self.buffer=lines.pop()
        for line in lines:
            try:d=json.loads(line)
            except ValueError:continue
            if d.get('event')=='sample':
                try:self.state=decode_sample(d,time.monotonic(),self.sequence);self.sequence=d['sequence']
                except (ValueError,KeyError,TypeError) as e:self.state=dict(ok=False,detail=str(e),updated=time.monotonic())
            elif d.get('event')=='error':self.error=connection_error(d.get('detail','连接失败'))
            elif d.get('event')=='closed':self.closed=d.get('init_confirmed') is True
        atomic(ROOT/'.cache/operator/dh116_status.json',self.state);self.changed.emit()
    def disconnect_device(self):
        if self.process is not None:
            self.stopping=True;self.state=dict(ok=False,detail='正在断开反馈连接',updated=time.monotonic());self.process.terminate();self.changed.emit()
    def finished(self,code,*_):
        if self.process is None:return
        self.consume();p=self.process;self.process=None;p.deleteLater()
        detail=self.error or ('已断开 · EtherCAT INIT 已确认' if self.closed else '连接结束，未确认设备回到 INIT（或认证未完成）')
        self.state=dict(ok=False,detail=detail,updated=time.monotonic());self.log.close();self.log=None
        atomic(ROOT/'.cache/operator/dh116_status.json',self.state);self.changed.emit()
