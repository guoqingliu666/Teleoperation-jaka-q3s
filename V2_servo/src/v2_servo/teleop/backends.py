"""同一周期运行器的仿真和真实 JAKA 驱动。真实驱动仅在子进程中导入 SDK。"""
import datetime,hashlib,json,multiprocessing as mp,queue,socket,subprocess,time,os
from pathlib import Path
import numpy as np
from ..configuration import ROOT,vector
from ..safety import Feedback
from ..jaka import load_sdk,checked,checked_bool,JakaServo,decode_status
from ..kinematics import ArmKinematics,CalibratedKinematics
from .feedback import feedback_process
from .edg import EdgMode1Feedback

def tcp_model(arm,q):
    t,_,_=arm.forward(q); r=t[:3,:3]
    return np.r_[t[:3,3]*1000,[np.arctan2(r[2,1],r[2,2]),np.arcsin(np.clip(-r[2,0],-1,1)),np.arctan2(r[1,0],r[0,0])]]

class SimBackend:
    kind='sim'
    def __init__(self,cfg,options):
        self.cfg=cfg; self.connected=False; self.powered=False; self.enabled=False; self.active=False
        self.q=np.array([-1.1159,2.2865,-2.3207,3.0411,2.4042,2.5245]); self.target=self.q.copy(); self.seq=0
        self.arm=ArmKinematics(ROOT/'models/assembly_reference.urdf'); self.fault=False; self.inject_stale=False
        self.fb=None; self.tcp=None; self.sent=0
    def connect(self): self.connected=True; self.sample(time.perf_counter())
    def power(self,on):
        if not self.connected: raise RuntimeError('请先连接')
        if not on: self.stop(); self.enabled=False
        self.powered=on
    def enable(self,on):
        if not self.connected or not self.powered: raise RuntimeError('请先连接、上电')
        if not on: self.stop()
        self.enabled=on
    def sample(self,now):
        if not self.connected: return None,None
        if self.inject_stale and self.fb is not None: return self.fb,self.tcp
        if self.active: self.q+=(self.target-self.q)*(-np.expm1(-self.cfg['period_s']/.018))
        self.seq+=1; self.fb=Feedback(self.q.copy(),now,self.seq,enabled=self.enabled,powered=self.powered,collision=self.fault)
        self.tcp=tcp_model(self.arm,self.q); return self.fb,self.tcp
    def start(self,space='joint',source='keyboard'):
        if not self.connected or not self.enabled or self.fault: raise RuntimeError('not enabled / fault')
        self.target=self.q.copy(); self.active=True
    def send(self,motion): self.target=vector(motion.q); self.sent+=1; return 0.
    def stop(self):
        self.active=False; self.target=self.q.copy()
        return {'servo_exited':True,'physical_stop_confirmed':True,'scope':'simulation_only'}
    def reset(self): self.fault=False; self.inject_stale=False
    def disconnect(self): self.stop(); self.connected=False; self.enabled=False; self.powered=False

def controller_owners(host):
    if os.name!='nt': return []
    # netstat只读系统连接表，避免CIM/PowerShell启动阻塞；失败时拒绝接管。
    import ipaddress
    address=str(ipaddress.ip_address(host))
    r=subprocess.run(['netstat.exe','-ano','-p','tcp'],capture_output=True,timeout=3,creationflags=0x08000000)
    if r.returncode!=0:raise RuntimeError('could not inspect robot SDK ownership')
    return tcp_owners_from_netstat(r.stdout.decode('ascii',errors='replace'),address,os.getpid())

def tcp_owners_from_netstat(output,host,current_pid):
    import ipaddress
    address=ipaddress.ip_address(host);owners=set()
    for line in output.splitlines():
        fields=line.split()
        if len(fields)!=5 or fields[0].upper()!='TCP':continue
        remote,port=fields[2].rsplit(':',1)
        if port!='10001' or not fields[-1].isdigit():continue
        try:remote=ipaddress.ip_address(remote.strip('[]'))
        except ValueError:continue
        pid=int(fields[-1])
        # FIN_WAIT 表示本端已关闭发送方向；Windows可能仍保留退出进程PID。
        # 此类TCP收尾不代表SDK所有者。ESTABLISHED/CLOSE_WAIT/连接中仍拒绝接管。
        if fields[3].upper() in ('FIN_WAIT_1','FIN_WAIT_2','TIME_WAIT','CLOSING','LAST_ACK','CLOSED'):
            continue
        if remote==address and pid not in (0,current_pid):owners.add(pid)
    return sorted(owners)

class DeploymentReview:
    """需要真实现场报告。界面没有用一个勾选框绕过缺项的入口。"""
    def __init__(self,path): self.path=Path(path); self.profile={}; self.deadline=0.
    def verify(self,space,source):
        p=json.loads(self.path.read_text(encoding='utf-8')); failures=[]
        if p.get('status')!='validated': failures.append('现场部署报告未验收')
        if p.get('robot_serial')!='S5160217': failures.append('机器人编号不匹配')
        transport=p.get('feedback_transport','port10000')
        if transport not in ('port10000','edg_mode1'): failures.append('未知反馈传输模式')
        feedback_evidence=('feedback_edg_age_and_units','lightweight_status_age_and_fields') if transport=='edg_mode1' else ('feedback_10000_age_and_units',)
        for key in ('incident_hardware_inspection','physical_estop_and_network_watchdog','software_filter_ruckig_validation',
                    'full_arm_workspace_and_cables',*feedback_evidence,'end_to_end_servo_timing'):
            entry=p.get('evidence',{}).get(key)
            if not isinstance(entry,dict): failures.append(key); continue
            file=(ROOT/entry.get('path','')).resolve()
            if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest()!=entry.get('sha256'): failures.append(key+':证据缺失或哈希不符')
        try:
            expires=datetime.datetime.fromisoformat(p['valid_until_utc'].replace('Z','+00:00'))
            remaining=(expires-datetime.datetime.now(datetime.timezone.utc)).total_seconds()
            if remaining<=0 or remaining>86400: raise ValueError()
        except Exception: remaining=0; failures.append('部署有效期需在24小时内')
        age=p.get('controller_feedback_age_bound_s')
        if type(age) not in (int,float) or not 0<=age<.016: failures.append('反馈源年龄上界未验收')
        timeout=p.get('network_abort_timeout_ms')
        if type(timeout) is not int or not 8<=timeout<=100: failures.append('断流终止时限未验收')
        if not p.get('reviewed_by'): failures.append('缺现场审查记录人')
        if space=='cartesian' and not p.get('cartesian_kinematics_validated'): failures.append('笛卡尔模型/TCP未校验')
        if source=='vr' and not p.get('vr_mapping_validated'): failures.append('VR坐标映射未标定')
        if failures: raise RuntimeError('真机伺服未就绪：'+'；'.join(failures))
        self.profile=p; self.deadline=time.perf_counter()+remaining
        return p
    def authorize(self,action):
        if time.perf_counter()>=self.deadline: raise RuntimeError('部署审查已失效')

class LiveBackend:
    kind='live'
    def __init__(self,cfg,options):
        self.cfg=cfg; self.options=options; self.connected=False; self.rc=None; self.driver=None
        self.fb=None; self.tcp=None; self.seq=0; self.last_poll=0; self.receiver=None; self.active=False
        self.review=DeploymentReview(ROOT/options['deployment_review']); self.lease=None; self.sent=0
        self.feedback_error='waiting'; self.fp_stop=None; self.fp_queue=None
        self.interrupted=lambda:False
        self.stream_fb=None; self.stream_tcp=None
        self.cartesian_kinematics=None
        self.controller_servo=None
        self.edg=None
    def authorize(self,action):
        self.review.authorize(action)
        if self.interrupted(): raise RuntimeError('停止请求优先于伺服使能/发送')
    def connect(self):
        owners=controller_owners(self.options['host'])
        if owners: raise RuntimeError('控制器10001已被进程占用：'+','.join(map(str,owners))+'；请在原窗口断开')
        try:
            self.lease=socket.socket(); self.lease.bind(('127.0.0.1',36151)); self.lease.listen(1)
            sdk=load_sdk(self.options['sdk_directory']); self.rc=sdk.RC(self.options['host']); checked(self.rc.login(),'login'); self.connected=True
            self.sdk_version=checked(self.rc.get_sdk_version(),'get_sdk_version')
            if checked_bool(self.rc.is_in_servomove(),'is_in_servomove'): raise RuntimeError('已有伺服会话，拒绝接管')
            self._poll()
            if self.options.get('feedback_transport','port10000')=='port10000':
                ctx=mp.get_context('spawn'); self.fp_stop=ctx.Event(); self.fp_queue=ctx.Queue(4)
                self.receiver=ctx.Process(target=feedback_process,args=(self.options['host'],self.fp_queue,self.fp_stop),daemon=True); self.receiver.start()
        except Exception:
            self.disconnect(); raise
    def _poll(self):
        start=time.perf_counter(); result=self.rc.get_robot_status(); self.seq+=1
        self.fb=decode_status(result,start,self.seq); self.tcp=vector(checked(self.rc.get_actual_tcp_position(),'get_actual_tcp_position'))
        self.controller_servo=checked_bool(self.rc.is_in_servomove(),'is_in_servomove')
        self.last_poll=time.perf_counter()
    def power(self,on):
        if not self.connected: raise RuntimeError('请先连接')
        if self.active: raise RuntimeError('请先停止伺服')
        checked(getattr(self.rc,'power_on' if on else 'power_off')(),'power'); self._poll()
    def enable(self,on):
        if not self.connected: raise RuntimeError('请先连接')
        if self.active: self.stop()
        checked(getattr(self.rc,'enable_robot' if on else 'disable_robot')(),'enable'); self._poll()
    def sample(self,now):
        if not self.connected: return None,None
        if self.edg is not None and self.edg.entered:
            self.fb,self.tcp=self.edg.sample(now)
            self.feedback_error=None
            return self.fb,self.tcp
        if self.fp_queue is not None:
            while True:
                try: r=self.fp_queue.get_nowait()
                except queue.Empty: break
                if 'error' in r: self.feedback_error=r['error']; continue
                self.stream_fb=r['feedback']; self.stream_tcp=r['tcp']; self.feedback_error=None
        if self.stream_fb is not None:
            age=self.review.profile.get('controller_feedback_age_bound_s',0)
            self.fb=Feedback(**{**self.stream_fb.__dict__,'sampled_at':self.stream_fb.sampled_at-age}); self.tcp=self.stream_tcp
        if self.active and self.feedback_error is not None: raise RuntimeError('独立反馈接收失败：'+str(self.feedback_error))
        # 伺服期间绝不做同步状态查询；独立反馈超时由安全层锁停。
        if not self.active and now-self.last_poll>.5 and (self.fb is None or now-self.fb.sampled_at>.1): self._poll()
        return self.fb,self.tcp
    def start(self,space='joint',source='keyboard'):
        if not self.connected: raise RuntimeError('请先连接')
        profile=self.review.verify(space,source)
        transport=self.options.get('feedback_transport','port10000')
        if profile.get('feedback_transport','port10000')!=transport:
            raise RuntimeError('部署审查的反馈模式与运行配置不匹配')
        if space=='cartesian':
            entry=profile.get('kinematics_calibration') or {}; file=(ROOT/entry.get('path','')).resolve()
            if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest()!=entry.get('sha256'): raise RuntimeError('缺经验收的笛卡尔变换文件及哈希，不能使用预览模型TCP')
            calibration=json.loads(file.read_text(encoding='utf-8'))
            if calibration.get('status')!='validated' or calibration.get('robot_serial')!='S5160217': raise RuntimeError('笛卡尔变换未验收或机器人不匹配')
            if calibration.get('model_sha256')!=hashlib.sha256((ROOT/'models/assembly_reference.urdf').read_bytes()).hexdigest(): raise RuntimeError('标定使用的运动学模型哈希不匹配')
            self.cartesian_kinematics=CalibratedKinematics(ArmKinematics(ROOT/'models/assembly_reference.urdf'),
                calibration['model_to_robot_m'],calibration['link06_to_tcp_m'],calibration['joint_offsets_rad'])
        # 仅在未进入伺服时核对当前工具/负载，不能拿历史截图当当前状态。
        expected=json.loads((ROOT/'config/site_observed_20260928.json').read_text(encoding='utf-8'))
        tool_id=checked(self.rc.get_tool_id(),'get_tool_id')
        if tool_id!=expected['tool_id']: raise RuntimeError('当前 Tool ID 与现场归档不符')
        tool=self.rc.get_tool_data(tool_id); checked(tool,'get_tool_data')
        if len(tool)!=3 or tool[1]!=tool_id or not np.allclose(vector(tool[2]),expected['tcp_mm_rad'],rtol=0,atol=1e-3): raise RuntimeError('当前 TCP 与现场归档不符')
        payload=checked(self.rc.get_payload(),'get_payload')
        if len(payload)!=2 or abs(float(payload[0])-expected['payload_kg'])>1e-6 or not np.allclose(payload[1],expected['cog_mm'],rtol=0,atol=1e-3): raise RuntimeError('当前负载/重心与现场归档不符')
        if transport=='edg_mode1':
            import ipaddress
            client_ip=str(ipaddress.ip_address(self.options['edg_client_ip']))
            port=int(self.options['edg_port'])
            if not 1024<=port<=65535: raise RuntimeError('EDG port invalid')
            self.edg=EdgMode1Feedback(self.rc,client_ip,port,profile['controller_feedback_age_bound_s'])
            self.edg.start()
            self.fb,self.tcp=self.edg.sample(time.perf_counter())
        else:
            self.sample(time.perf_counter())
            if self.feedback_error is not None or self.stream_fb is None: raise RuntimeError('独立10000反馈未建立：'+str(self.feedback_error))
            source_age=time.perf_counter()-self.stream_fb.sampled_at+profile['controller_feedback_age_bound_s']
            if not 0<=source_age<=self.cfg['safety']['feedback_timeout_s']: raise RuntimeError('独立反馈不新鲜')
        # 单位/数据一致性静态交叉核对，不让同步状态轮询冒充独立反馈。
        actual=vector(checked(self.rc.get_actual_joint_position(),'get_actual_joint_position'))
        independent=self.fb if transport=='edg_mode1' else self.stream_fb
        if np.max(np.abs(actual-independent.q))>np.deg2rad(.5): raise RuntimeError('SDK与独立关节反馈不一致')
        if space=='cartesian':
            model,_jac,_points=self.cartesian_kinematics.forward(actual)
            pose=vector(checked(self.rc.get_actual_tcp_position(),'get_actual_tcp_position'))
            from ..kinematics import rotate
            rotation=rotate([0,0,1],pose[5])@rotate([0,1,0],pose[4])@rotate([1,0,0],pose[3])
            angular=np.arccos(np.clip((np.trace(model[:3,:3].T@rotation)-1)/2,-1,1))
            if np.linalg.norm(model[:3,3]*1000-pose[:3])>2 or angular>np.deg2rad(1): raise RuntimeError('当前模型TCP与官方反馈偏差超过2mm/1°，拒绝笛卡尔模式')
        if not self.fb.enabled or not self.fb.powered: raise RuntimeError('请先上电、使能')
        checked(self.rc.set_network_exception_handle(int(profile['network_abort_timeout_ms']),2),'set_network_exception_handle')
        self.driver=JakaServo(self.rc,self.cfg,self.authorize)
        self.driver.start(); self.active=True; self.controller_servo=True
    def send(self,motion):
        elapsed=self.driver.send(motion); self.sent+=1; return elapsed
    def stop(self):
        result={'servo_exited':None,'physical_stop_confirmed':False}
        if self.driver is not None: result=self.driver.stop()
        self.active=False
        if self.driver is not None: self.controller_servo=False if result.get('servo_exited') is True else None
        if result.get('servo_exited') is True: self.driver=None

        if self.edg is not None:
            try:self.edg.close();self.edg=None
            except Exception as e: result.setdefault('errors',[]).append('EDG exit: '+str(e))
        return result
    def reset(self):
        if self.active: raise RuntimeError('stop before reset')
        if not self.connected: return
        self._poll()
        if self.fb.estop or self.fb.collision or self.fb.error: raise RuntimeError('请在现场确认并通过JAKA App复位硬件报警；V2不自动清故障')
    def disconnect(self):
        try:
            if self.driver is not None or self.edg is not None: self.stop()
            if self.connected: checked(self.rc.logout(),'logout')
        finally:
            self.connected=False
            self.controller_servo=None
            if self.fp_stop is not None: self.fp_stop.set()
            if self.receiver is not None:
                self.receiver.join(.5)
                if self.receiver.is_alive(): self.receiver.terminate(); self.receiver.join(.5) # 仅终止无命令的接收器。
            if self.lease is not None: self.lease.close(); self.lease=None
            self.receiver=None; self.fp_queue=None; self.fp_stop=None; self.stream_fb=None; self.stream_tcp=None; self.feedback_error='waiting'
