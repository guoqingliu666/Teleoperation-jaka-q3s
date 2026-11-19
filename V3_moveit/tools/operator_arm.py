"""Explicit, asynchronous JAKA cabinet preparation; no teleoperation or targets."""
import argparse,hashlib,json,ipaddress,subprocess,time
from pathlib import Path
from PyQt5 import QtCore
from operator_settings import ROOT
from operator_tooling import tool_values, apply_tool, save_readback

def native_arguments(action,root=ROOT,tool_id=None):
    import yaml
    from scipy.spatial.transform import Rotation
    if action not in ('poll','status','check','power_on','enable','select_tool','refresh_payload'):raise ValueError('未知机器人操作')
    site=yaml.safe_load((root/'site_ubuntu.yaml').read_text());base=yaml.safe_load((root/'src/v3_moveit/config/runtime.yaml').read_text())
    if site['robot_ip']!=base['robot_ip']:raise ValueError('机器人地址与工程目标不符')
    if not isinstance(ipaddress.ip_address(site['robot_ip']),ipaddress.IPv4Address):raise ValueError('需要 IPv4 机器人地址')
    meta=json.loads((root/'src/v3_moveit/config/model_metadata.json').read_text());payload=json.loads((root/'src/v3_moveit/config/payload.json').read_text())
    import xml.etree.ElementTree as ET
    _,meta=apply_tool(ET.parse(root/'src/v3_moveit/description/s5.urdf').getroot(),meta,payload)
    selected,tcp=tool_values(payload)
    if action=='select_tool':
        if type(tool_id) is not int or not 0<=tool_id<=10:raise ValueError('请选择有效工具')
        selected=tool_id
    home=meta['tcp_home'];pose=[home[i][3]*1000 for i in range(3)]+Rotation.from_matrix([r[:3] for r in home[:3]]).as_euler('xyz').tolist()
    values=[payload['payload_kg'],*payload['cog_mm'],*meta['q_reference_rad'],*pose,*meta['joint_lower'],*meta['joint_upper'],selected,*tcp]
    binary=root/'.cache/operator/arm_build/v3_arm_power'
    if not binary.is_file():raise ValueError('机器人电源组件未构建，请运行 bash tools/build_operator_arm.sh')
    return str(binary),['--hardware',site['robot_ip'],action,*map(str,values)]

class ArmControl(QtCore.QObject):
    changed=QtCore.pyqtSignal()
    def __init__(self,parent=None):
        super().__init__(parent);self.process=None;self.state={};self.detail='连接机械臂后自动更新状态';self.buffer='';self.log=None;self.result={};self.action=None
        self.background=False;self.connection_requested=False;self.connection_state='disconnected';self.error='';self.tooling={};self.deferred=None;self.last_poll=0.;self.error_kind=''
    @property
    def busy(self):return self.process is not None and (not self.background or self.deferred is not None)
    def after_poll(self,callback):
        if self.process is None or not self.background:return False
        self.deferred=callback;self.changed.emit();self.process.terminate();return True
    def disconnect(self):
        self.deferred=None
        self.connection_requested=False;self.connection_state='disconnected'
        if self.process is not None and self.background:self.process.terminate()
        self.changed.emit()
    def start(self,action,tool_id=None,background=False):
        if self.process is not None:return
        command,args=native_arguments(action,tool_id=tool_id);self.action=action;self.result={};self.buffer='';self.error='';self.error_kind='';self.background=background
        self.last_poll=time.monotonic()
        if not background:self.connection_requested=True
        if not self.state:self.connection_state='connecting'
        self.before_sha=hashlib.sha256((ROOT/'src/v3_moveit/config/payload.json').read_bytes()).hexdigest()
        if background:self.capture=ROOT/'.cache/operator/arm_monitor.jsonl'
        else:
            folder=ROOT/'artifacts/arm_power_sessions'/(time.strftime('%Y%m%dT%H%M%S')+f'_{time.time_ns()%1000000000:09d}');folder.mkdir(parents=True,exist_ok=True);self.capture=folder/'capture.jsonl'
        self.log=self.capture.open('w')
        self.detail={'poll':'正在核对机械臂连接','status':'正在读取控制柜状态、TCP 和负载','check':'正在检查 TCP、负载与模型','power_on':'检查通过后打开本体电源','enable':'检查通过后使能机器人','select_tool':'正在切换并回读 JAKA 工具坐标系','refresh_payload':'正在读取校准后的负载与质心'}[action]
        p=QtCore.QProcess(self);self.process=p;p.setWorkingDirectory(str(ROOT));p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        p.readyReadStandardOutput.connect(self.consume);p.finished.connect(self.finished);p.errorOccurred.connect(lambda e:self.finished(-1) if e==QtCore.QProcess.FailedToStart else None)
        p.start(command,args);self.changed.emit()
    def consume(self):
        if self.process is None:return
        text=bytes(self.process.readAllStandardOutput()).decode(errors='replace');self.log.write(text);self.log.flush();lines=(self.buffer+text).split('\n');self.buffer=lines.pop()
        for line in lines:
            try:d=json.loads(line)
            except ValueError:continue
            if d.get('event')=='status':self.state=dict(d,updated=time.monotonic());self.detail=('本体电源已打开' if d['powered_on']==1 else '本体电源未打开')+' · '+('机器人已使能' if d['enabled']==1 else '机器人未使能')+(f" · 报警 {d['errcode']}" if d['errcode'] else '')
            elif d.get('event')=='tooling':
                if not d.get('tools'):d['tools']=self.tooling.get('tools',[])
                self.tooling=dict(d,updated=time.monotonic())
            elif d.get('event')=='error':self.detail=d.get('detail','机器人操作失败');self.error=self.detail;self.error_kind=d.get('kind','error')
            elif d.get('event')=='result':self.result=d
        self.changed.emit()
    def finished(self,code,*_):
        if self.process is None:return
        self.consume();p=self.process;self.process=None
        if self.result.get('ok') and code==0:
            s=self.state
            occupied=any(s.get(k,0) for k in ('servo','drag','program','queue','active_queue')) or (s.get('enabled')==1 and s.get('inpos',1)!=1)
            self.connection_state='occupied' if occupied else 'connected'
        elif not (self.background and self.deferred):self.connection_state='occupied' if self.error_kind=='occupied' else 'error'
        if not self.connection_requested:self.connection_state='disconnected'
        if code==-1:self.detail='启动失败：'+p.errorString()
        elif not self.result.get('ok'):self.detail+=' · 操作未完成'
        else:
            if self.action in ('select_tool','refresh_payload'):
                try:
                    changed=save_readback(self.action,self.tooling,self.capture,self.before_sha)
                    self.detail=('工具坐标系已应用' if self.action=='select_tool' else '负载与质心已刷新')+('；下次启动将使用新配置' if changed else '；与现有配置一致')
                    if changed:
                        from operator_model import generate
                        generate()
                except Exception as e:self.error=str(e);self.detail='控制柜已回读，但 V3 保存/显示更新失败：'+str(e);self.result['ok']=False
            self.detail+=' · '+time.strftime('%H:%M:%S')+' 读取确认'
        p.deleteLater();self.log.close();self.log=None;self.changed.emit()
        if self.deferred is not None:QtCore.QTimer.singleShot(0,self.run_deferred)
    def run_deferred(self):
        if self.process is not None:return
        callback=self.deferred;self.deferred=None
        if callback is not None:callback()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--hardware',action='store_true',required=True);p.add_argument('action',choices=['status','check','power_on','enable']);a=p.parse_args()
    command,args=native_arguments(a.action);raise SystemExit(subprocess.run([command,*args],cwd=ROOT).returncode)
