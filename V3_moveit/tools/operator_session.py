"""Local session owner. Reuses verified launch/check tools; never calls the SDK."""
import argparse,fcntl,hashlib,json,os,resource,signal,socket,subprocess,sys,time
from datetime import datetime,timezone
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def emit(message,**values):
    print('OP '+json.dumps(dict(message=message,**values),ensure_ascii=False),flush=True)

def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def failure_detail(text):
    from operator_tooling import recovery
    kind,detail=recovery(text)
    if kind:return detail
    for line in reversed(text.splitlines()):
        try:record=json.loads(line)
        except ValueError:continue
        if isinstance(record,dict) and record.get('passed') is False and record.get('failure'):
            detail=str(record['failure'])
            for source,target in {
                'Grip held before released-input and stationary READY checkpoint':'开始时 Grip 已握住。下次看到头显显示就绪再握住。',
                'Observed TCP excursion exceeded commissioning exit threshold':'末端移动超过本轮观察退出范围，请缩小动作完成停止检查',
                'Observed joint excursion exceeded commissioning exit threshold':'关节移动超过本轮 30° 观察退出范围，请缩小动作',
                'Production/observer fault: ':'',
            }.items():detail=detail.replace(source,target)
            return detail
    for line in reversed(text.splitlines()):
        if line.startswith(('RuntimeError:','ValueError:')):return line.split(':',1)[1].strip()
        try:record=json.loads(line)
        except ValueError:continue
        if record.get('event')=='error':return record.get('detail','检查未通过')
    return '未通过，具体原因见运行日志。'

def stopped_event(report,reason):
    episodes=report.get('motion_episodes',[])
    for e in report.get('stop_observations',[]):
        i=e.get('motion_episode_index')
        if (e.get('reason')==reason and e.get('confirmed_stationary') is True and
            not e.get('invalidated_after_confirmation') and type(i) is int and 0<=i<len(episodes)):
            motion=episodes[i]
            if motion.get('follow_seen') and motion.get('maximum_measured_motion_rad',0)>1e-4 and motion.get('maximum_command_motion_rad',0)>1e-4:return True
    return False

def validated_hardware_report(p):
    from verify_offline import source_digest,calibration_digest
    d=read(p)
    if not (d.get('passed') is True and not d.get('failure') and d.get('mode')=='hardware' and
            d.get('package_sha256')==source_digest() and d.get('script_sha256')==sha(ROOT/'tools/hardware_quest_commissioning.py') and
            d.get('servo_exit_confirmed') is True and d.get('launch_exit_code')==0 and not d.get('cleanup_unconfirmed_live_pid') and
            stopped_event(d,'RELEASED') and stopped_event(d,'SOFTWARE_STOP')):
        raise RuntimeError('需要当前版本的完整实机检查通过，且包含运动后的松 Grip、摇杆停止及 Servo 退出。')
    capture=d.get('capture',{});raw=Path(capture.get('path','')).resolve()
    if not raw.is_relative_to(ROOT) or not raw.is_file() or sha(raw)!=capture.get('sha256'):
        raise RuntimeError('实机原始记录缺失或改变。')
    if d.get('site',{}).get('sha256')!=sha(ROOT/'site_ubuntu.yaml'):
        raise RuntimeError('实机检查对应的站点配置已改变。')
    if d.get('calibration_sha256')!=calibration_digest(ROOT):
        raise RuntimeError('负载或 TCP 配置已更新，需按当前配置完成实机检查；仿真仍可直接使用。')
    return p,d

def acceptance_report():
    return validated_hardware_report(ROOT/'artifacts/quest_commissioning_hardware/result.json')

def accept_local_observation():
    from operator_settings import atomic
    p,report=acceptance_report()
    archive=ROOT/'artifacts/operator_acceptance'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ');archive.mkdir(parents=True)
    raw=Path(report['capture']['path']);(archive/raw.name).write_bytes(raw.read_bytes())
    report['capture']['path']=str(archive/raw.name)
    p=archive/'result.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    record_path=ROOT/'artifacts/hardware_readiness.json'
    record=read(record_path) if record_path.exists() else {}
    record.update(package_sha256=report['package_sha256'],workspace=str(ROOT),
        teleoperation_ready=True,physical_acceptance=False,
        basic_teleoperation_user_accepted=True,physical_direction_user_confirmed=True,
        physical_grip_stop_user_confirmed=True,physical_joystick_stop_user_confirmed=True,
        accepted_utc=datetime.now(timezone.utc).isoformat(),accepted_via='Ubuntu local operator panel',
        accepted_hardware_report=dict(path=str(p),sha256=sha(p)),
        accepted_profile=dict(auto_replan=False,site_sha256=sha(ROOT/'site_ubuntu.yaml')),
        blockers=[],acceptance_scope='Supervised low-speed Servo-only follow and two operator stops. Not worst-case braking or environmental clearance certification.')
    atomic(ROOT/'artifacts/hardware_readiness.json',record)
    saved_acceptance()

def saved_acceptance():
    """Validate immutable, user-confirmed evidence independently of session caches."""
    from verify_offline import source_digest
    record=read(ROOT/'artifacts/hardware_readiness.json')
    if record.get('teleoperation_ready') is not True or record.get('package_sha256')!=source_digest():
        raise RuntimeError('当前版本尚无已确认的现场跟随与停止验收记录。')
    evidence=record.get('accepted_hardware_report',{});p=Path(evidence.get('path','')).resolve()
    if not p.is_relative_to(ROOT) or not p.is_file() or sha(p)!=evidence.get('sha256'):
        raise RuntimeError('实机验收证据缺失或改变。')
    validated_hardware_report(p)
    profile=record.get('accepted_profile',{})
    if profile!={'auto_replan':False,'site_sha256':sha(ROOT/'site_ubuntu.yaml')}:
        raise RuntimeError('当前站点与已验收的 Servo 跟随配置不符。')
    return record

def validate_teleop_runtime(runtime):
    import yaml
    base=yaml.safe_load((ROOT/'src/v3_moveit/config/runtime.yaml').read_text())
    cfg={**base, **yaml.safe_load(Path(runtime).read_text())}
    site=yaml.safe_load((ROOT/'site_ubuntu.yaml').read_text())
    if cfg.get('auto_replan') is not False or any(cfg.get(k)!=v for k,v in site.items()):
        raise RuntimeError('真机遥操作需要使用当前站点和 auto_replan=false 的 Servo 跟随配置。')
    if cfg!={**base, **site, 'auto_replan':False}:raise RuntimeError('运行参数与本站点的 Servo 配置不一致。')

def validate_live_fake(digest=None):
    from verify_offline import source_digest
    from check_ros_pass import SIMULATION_REPORTS
    digest=digest or source_digest()
    fake=read(ROOT/'artifacts/quest_commissioning_fake/result.json')
    if not fake.get('passed'):raise RuntimeError('尚未完成 Quest 60 秒仿真检查。')
    if fake.get('package_sha256')!=digest or fake.get('script_sha256')!=sha(ROOT/'tools/hardware_quest_commissioning.py'):
        raise RuntimeError('Quest 仿真记录对应旧版本，请重新运行 60 秒仿真检查。')
    if fake.get('site',{}).get('sha256')!=sha(ROOT/'site_ubuntu.yaml'):
        raise RuntimeError('站点配置已更新，请按当前站点重新运行 Quest 60 秒仿真检查。')
    current={name:sha(ROOT/name) for name,_ in SIMULATION_REPORTS}
    if fake.get('prerequisite_reports')!=current:
        raise RuntimeError('软件回归记录已更新，请重新运行 Quest 60 秒仿真检查。')
    return fake

def assert_domain_empty(check_cancel):
    """Read-only discovery before a continuous launch; owns no motion topic."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('v3_operator_prelaunch',start_parameter_services=False,enable_rosout=False)
    try:
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            check_cancel();rclpy.spin_once(node,timeout_sec=.05)
        other=[(n,ns) for n,ns in node.get_node_names_and_namespaces() if (n,ns)!=(node.get_name(),node.get_namespace())]
        if other:raise RuntimeError('控制域已有会话运行，请先结束原会话：'+str(other))
        for topic in ('/v3/motion_lease','/arm_controller/joint_trajectory','/joint_states'):
            if node.count_publishers(topic):raise RuntimeError('控制或反馈端口已占用：'+topic)
    finally:
        node.destroy_node();rclpy.shutdown()

class SessionStopped(RuntimeError):
    pass

class Session:
    def __init__(self,args):
        self.args=args;self.cancel=False;self.child=None;self.cue=None;self.owner=os.getppid()
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.out=ROOT/'artifacts/operator_sessions'/stamp;self.out.mkdir(parents=True)
        self.events=[]
        self.started=time.monotonic();self.guide_stage='starting';self.guide_written=None
        try:self.tool_id=read(ROOT/'src/v3_moveit/config/payload.json').get('tool_id',1)
        except (ValueError,OSError,AttributeError):self.tool_id=None
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,self.request_stop)
    def update_guidance(self,stage=None):
        from operator_settings import atomic
        now=time.monotonic()
        if stage is None:stage=self.guide_stage
        if stage==self.guide_stage and self.guide_written is not None and now-self.guide_written<.4:return
        self.guide_stage=stage;self.guide_written=now
        try:
            atomic(ROOT/'.cache/operator/teleop_session.json',dict(started=self.started,updated=now,
                   stage=stage,task=self.args.task,mode='hardware' if self.args.hardware else 'fake',tool_id=self.tool_id))
        except OSError:
            # The headset expires its display if this optional status channel is unavailable.
            pass
    def request_stop(self,*unused):self.cancel=True
    def check_cancel(self):
        if os.getppid()!=self.owner:self.cancel=True
        if self.cancel:raise SessionStopped('操作者已结束会话。')
        self.update_guidance()
    def run_child(self,argv,label,interruptible=True,cues=False):
        self.check_cancel()
        self.update_guidance('running' if cues else 'checking')
        emit(label,phase=label)
        log=self.out/(str(len(self.events))+'.log');self.events.append(dict(label=label,argv=argv,log=str(log)))
        signaled=False;announced=False;position=0
        with log.open('w') as stream:
            self.child=subprocess.Popen(argv,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            while self.child.poll() is None:
                if os.getppid()!=self.owner:self.cancel=True
                self.update_guidance('stopping' if self.cancel else None)
                if self.cancel and interruptible and not signaled:
                    self.child.send_signal(signal.SIGINT);signaled=True;emit('正在结束会话。')
                with log.open() as reader:
                    reader.seek(position);chunk=reader.read();position=reader.tell()
                if chunk:print(chunk,end='',flush=True)
                if cues and not self.cancel and not announced and (cues=='continuous' or 'READY: Grip' in log.read_text()):
                    announced=True;env=dict(os.environ,ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID','90'),ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_STATIC_PEERS='')
                    self.cue=subprocess.Popen([sys.executable,'-B','tools/operator_audio.py','--session-started',str(self.started),'--mode','hardware' if self.args.hardware else 'fake']+(['--continuous'] if cues=='continuous' else []),cwd=ROOT,env=env,stdin=subprocess.DEVNULL)
                    emit('正在连接控制器。' if cues=='continuous' else '开发检查已就绪，请看头显提示。',ready=cues!='continuous')
                time.sleep(.05)
            code=self.child.returncode;self.child=None
        if self.cue is not None:
            if self.cue.poll() is None:self.cue.send_signal(signal.SIGINT);self.cue.wait(timeout=5)
            self.cue=None
        print(log.read_text()[position:],end='',flush=True)
        from operator_tooling import recovery
        kind,detail=recovery(log.read_text())
        if kind:emit(detail,error=True,recovery_kind=kind)
        self.events[-1]['exit_code']=code
        warnings=[line for line in log.read_text().splitlines() if 'process has died' in line]
        self.events[-1]['cleanup_warnings']=warnings
        if warnings:emit('部分组件退出异常，请查看运行日志。',cleanup_warning=True)
        if self.args.hardware:
            report_path=None
            if 'tools/hardware_hold_check.py' in argv:report_path=ROOT/'artifacts/hardware_hold_check.json'
            elif 'tools/hardware_quest_commissioning.py' in argv:report_path=ROOT/'artifacts/quest_commissioning_hardware/result.json'
            retained=read(report_path).get('cleanup_unconfirmed_live_pid') if report_path and report_path.exists() else None
            if type(retained) is int and retained>1:
                emit('Servo 退出尚未确认，保留控制进程和会话锁；请使用物理急停并查看日志。',error=True)
                while Path('/proc',str(retained)).exists():time.sleep(.2)
                raise RuntimeError('保留的进程已结束，但尚未确认 Servo 退出；本次不算通过。')
        if self.args.hardware and 'ros2' in argv and 'mode:=hardware' in argv:
            confirmed='Servo exit confirmed' in log.read_text()
            self.events[-1]['servo_exit_confirmed']=confirmed
            if not confirmed:
                raise RuntimeError((detail+' ' if kind else '')+'会话已退出，未收到 Servo 退出确认，请检查控制柜状态。')
            emit('已收到 Servo 退出确认。',servo_exit_confirmed=True)
            if kind:raise RuntimeError(detail)
        self.check_cancel()
        if code:raise RuntimeError(label+'：'+failure_detail(log.read_text()))
    def input_ready(self):
        from operator_quest import ensure_connection
        import yaml
        runtime=yaml.safe_load((ROOT/'.cache/operator/runtime.yaml').read_text())
        port=int(runtime.get('quest_port',5006))
        self.update_guidance('input')
        ensure_connection()
        emit('正在识别右手柄。')
        deadline=time.monotonic()+90;good=None;link_check=time.monotonic()
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
            s.bind(('0.0.0.0',port));s.settimeout(.1)
            while time.monotonic()<deadline:
                self.check_cancel()
                if time.monotonic()-link_check>3:
                    ensure_connection();link_check=time.monotonic()
                try:r=json.loads(s.recvfrom(65535)[0])['right']
                except socket.timeout:good=None;self.update_guidance('input');continue
                valid=(r.get('connected') and r.get('tracked') and r.get('pose_valid') and
                       type(r.get('grip')) in (float,int) and r['grip']<.1 and r.get('thumbstick_click') is False)
                if valid:
                    self.update_guidance('starting')
                    good=good or time.monotonic()
                    if time.monotonic()-good>=1:return
                else:
                    good=None
                    tracked=all(r.get(k) is True for k in ('connected','tracked','pose_valid'))
                    held=type(r.get('grip')) in (int,float) and r['grip']>=.1
                    self.update_guidance('release' if tracked and held else 'input')
        raise RuntimeError('右手柄未就绪，请查看头显中的连接状态。')
    def prepare(self):
        from verify_offline import source_digest
        from check_ros_pass import validate_reports
        from hardware_current_preflight import validate
        from hardware_hold_check import validate_site
        from hardware_quest_commissioning import commissioning_prerequisites
        digest=source_digest()
        validate_site(ROOT)
        validate_reports(ROOT,digest)
        validate_live_fake(digest)
        try:validate(ROOT,digest)
        except Exception:
            self.run_child([sys.executable,'-B','tools/hardware_current_preflight.py','--hardware'],'读取实机状态、TCP、负载和反馈（不进入伺服）',interruptible=False)
        try:commissioning_prerequisites(ROOT,digest,True)
        except Exception:
            for name in ('hardware_hold_check.json','hardware_hold_check.log','hardware_hold_check_feedback.jsonl'):
                p=ROOT/'artifacts'/name
                if p.exists():(self.out/('previous_'+name)).write_bytes(p.read_bytes())
            self.run_child([sys.executable,'-B','tools/hardware_hold_check.py','--hardware'],'实机原位保持检查（12 秒）')
        commissioning_prerequisites(ROOT,digest,True)
    def execute(self):
        import yaml
        cfg=yaml.safe_load((ROOT/'site_ubuntu.yaml').read_text());cfg['auto_replan']=False
        runtime=ROOT/'.cache/operator/runtime.yaml';runtime.write_text(yaml.safe_dump(cfg))
        if self.args.hardware:
            if resource.getrlimit(resource.RLIMIT_RTPRIO)[0]<50:raise RuntimeError('请先点击“启用本次实时权限”，完成系统认证。')
            if self.args.task=='teleop':
                from hardware_hold_check import validate_site
                validate_site(ROOT)
                validate_teleop_runtime(runtime)
            else:self.prepare()
        self.check_cancel()
        if self.args.task=='prepare':return
        self.input_ready()
        if self.args.task=='commission':
            argv=[sys.executable,'-B','tools/hardware_quest_commissioning.py','--duration','60']
            if self.args.hardware:argv.append('--hardware')
            self.run_child(argv,('真机' if self.args.hardware else '仿真')+' 60 秒检查',cues=True)
            result=ROOT/'artifacts'/('quest_commissioning_hardware' if self.args.hardware else 'quest_commissioning_fake')/'result.json'
            d=read(result)
            emit('检查通过。',passed=True,report=str(result),release_stop=stopped_event(d,'RELEASED'),joystick_stop=stopped_event(d,'SOFTWARE_STOP'))
        else:
            assert_domain_empty(self.check_cancel)
            argv=['ros2','launch','--noninteractive','v3_moveit','bringup.launch.py','mode:='+('hardware' if self.args.hardware else 'fake'),'runtime_config:='+str(runtime),'rviz:=false','sigterm_timeout:=86400','sigkill_timeout:=86400']
            if self.args.hardware:
                # Recheck after waiting for the operator. The driver validates live
                # state, FK, tool, payload and fresh stationary feedback before Servo.
                validate_site(ROOT)
                validate_teleop_runtime(runtime)
                argv.append('payload_config:='+str(ROOT/'src/v3_moveit/config/payload.json'))
            self.run_child(argv,'正在启动'+('真机' if self.args.hardware else '仿真')+'遥操作。',cues='continuous')
    def run(self):
        code=0
        self.update_guidance()
        try:self.execute()
        except SessionStopped:
            emit('会话已结束。'+('部分组件退出异常，详见运行日志。' if any(e.get('cleanup_warnings') for e in self.events) else ''),canceled=True)
        except Exception as e:code=1;emit(str(e),error=True)
        finally:
            self.update_guidance('stopping')
            # Never force-kill an SDK owner. Keep this process alive while its
            # child completes graceful cleanup, even if the UI has disappeared.
            if self.child is not None and self.child.poll() is None:
                self.child.send_signal(signal.SIGINT);self.child.wait()
            if self.cue is not None and self.cue.poll() is None:
                self.cue.send_signal(signal.SIGINT);self.cue.wait()
            self.update_guidance('failed' if code else 'ended')
            (self.out/'session.json').write_text(json.dumps(dict(mode='hardware' if self.args.hardware else 'fake',task=self.args.task,events=self.events,exit_code=code,canceled=self.cancel),ensure_ascii=False,indent=2)+'\n')
        return code

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--hardware',action='store_true');parser.add_argument('--task',choices=['commission','teleop','prepare'],default='teleop');args=parser.parse_args()
    os.chdir(ROOT)
    lock=(ROOT/'.cache/operator/control.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:emit('已有本机控制会话运行。',error=True);return 2
    return Session(args).run()
if __name__=='__main__':raise SystemExit(main())
