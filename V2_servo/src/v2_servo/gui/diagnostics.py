"""诊断页不拥有机器人SDK；相机在独立外部进程运行。"""
import json,os,subprocess,sys,time,ipaddress,uuid
from tkinter import ttk
import tkinter as tk
from ..configuration import ROOT

class DiagnosticPanel:
    def __init__(self,app,parent):
        self.app=app; self.camera=None; self.camera_report={}; self.camera_out=None
        self.ip=tk.StringVar(value=app.options['host']); self.text=tk.StringVar(); self.cam_text=tk.StringVar(value='ZED-M / SN12474260 / 期望固件1523 / eye-in-hand；尚未扫描')
        row=ttk.Frame(parent,padding=8); row.pack(fill='x'); ttk.Label(row,text='机器人 IP').pack(side='left'); ttk.Entry(row,textvariable=self.ip,width=22).pack(side='left')
        ttk.Label(row,text='控制10001 / 候选反馈EDG模式1（见配置）').pack(side='left',padx=10)
        ttk.Button(row,text='只读诊断',command=lambda:app.send('diagnose')).pack(side='left',padx=5)
        ttk.Button(row,text='详细点击说明',command=lambda:os.startfile(str(ROOT/'docs/V2_servo_usage_click_by_click.md'))).pack(side='left')
        ttk.Label(parent,textvariable=self.text,justify='left',font=('Microsoft YaHei UI',11),wraplength=1080,padding=12).pack(anchor='w',fill='x')
        commissioning=ttk.LabelFrame(parent,text='最近真机分级试验：历史证据，非当前机器人状态',padding=8)
        commissioning.pack(fill='x',padx=8,pady=3)
        self.commissioning_text=tk.StringVar()
        ttk.Label(commissioning,textvariable=self.commissioning_text,wraplength=1060).pack(anchor='w')
        ttk.Button(commissioning,text='刷新H1/H2记录（不发送命令）',command=self.refresh_commissioning).pack(side='left')
        ttk.Button(commissioning,text='本轮步骤 / 故障报告',command=lambda:os.startfile(str(ROOT/'docs/commissioning_20261002.md'))).pack(side='left',padx=6)
        ttk.Button(commissioning,text='Quest位移J1现场入口',command=self.start_quest_j1).pack(side='left',padx=6)
        self.refresh_commissioning()
        camera=ttk.LabelFrame(parent,text='独立 ZED 进程：不会成为手动 jog 的依赖',padding=12); camera.pack(fill='x',padx=8,pady=8)
        ttk.Label(camera,textvariable=self.cam_text,wraplength=1060).pack(anchor='w')
        self.camera_buttons=[]
        for label,preview in [('扫描/标定核对',False),('打开双目预览（30秒）',True)]:
            b=ttk.Button(camera,text=label,command=lambda p=preview:self.camera_start(p)); b.pack(side='left',padx=5,pady=8); self.camera_buttons.append(b)
        ttk.Button(camera,text='打开相机日志',command=lambda:os.startfile(str(self.camera_out or ROOT/'data'))).pack(side='left')
        ttk.Label(parent,text='软件急停不是物理急停；安全门未知不会显示为正常。滤波总开关强制开启。\n手眼外参已确认：相机 → TCP tool0，平移 mm，矩阵左乘。现场配准未验收，运动转换未启用。',wraplength=1050,padding=12).pack(anchor='w')
    def host(self): return str(ipaddress.ip_address(self.ip.get().strip()))
    def start_quest_j1(self):
        if self.app.snapshot.get('connected') or self.app.snapshot.get('active'):
            self.app.alarm_text.set('请先点“断开”，确认未连接；H3需要唯一SDK所有者。');return
        # 关闭此GUI及其UDP发布器，防止DISCONNECTED帧覆盖H3真实反馈。
        self.app.close()
        subprocess.Popen(['cmd.exe','/d','/c','call',str(ROOT/'START_QUEST_J1_POSE_LIVE.cmd')],
                         cwd=ROOT,creationflags=subprocess.CREATE_NEW_CONSOLE if os.name=='nt' else 0)
    def refresh_commissioning(self):
        lines=[]
        for prefix in ('h1','h2','h3'):
            paths=sorted((ROOT/'data').glob(prefix+'_*/report.json'))
            if not paths:
                lines.append(prefix.upper()+': 未实测');continue
            path=paths[-1]
            try:
                r=json.loads(path.read_text(encoding='utf-8'))
                passed=r.get({'h1':'passed_static_only','h2':'passed_single_axis_only','h3':'passed_quest_j1_session_only'}[prefix])
                scope='EDG本地返回成功，不等同控制器逐条确认' if r.get('edg_mode')==0 else 'SDK返回成功'
                lines.append(f"{path.parent.name}: {'仅本阶段通过' if passed else '未通过'}；发送{r.get('servo_commands',0)}条（{scope}）；退出伺服={r.get('servo_exited')}；错误={r.get('failure') or '无'}")
            except (OSError,ValueError) as exc:lines.append(prefix.upper()+': 报告不可读 '+str(exc))
        lines.append('H1静态或H2微动均不放行Quest连续6DoF；故障后不会自动返程。')
        self.commissioning_text.set('\n'.join(lines))
    def camera_start(self,preview):
        if self.camera and self.camera.poll() is None: self.cam_text.set('相机任务仍运行，请等待或在预览窗口按Esc'); return
        self.camera_out=ROOT/'data'/('zed_gui_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]); self.camera_out.mkdir()
        args=[sys.executable,str(ROOT/'scripts/zed_check.py'),'--output',str(self.camera_out),'--seconds','30' if preview else '4']
        if preview: args.append('--preview')
        with (self.camera_out/'process.log').open('w',encoding='utf-8') as log:
            self.camera=subprocess.Popen(args,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,creationflags=0x08000000 if os.name=='nt' else 0)
        self.cam_text.set('ZED 独立进程正在按SN12474260打开；eye-in-hand；相机失败不改变机器人状态')
    def update(self,s):
        def setting(var):
            try: return str(var.get())
            except (ValueError,tk.TclError): return '输入无效（已请求停止）'
        state=lambda key:'未知' if s.get(key) is None else str(s[key])
        reasons=s.get('deployment_blockers') or ['未执行只读诊断；点击“只读诊断”列出现场缺项']
        self.text.set('\n'.join([
            f"运行后端：{'SIM 仿真：本窗口不会控制实机' if self.app.mode=='sim' else 'LIVE 真实 JAKA'}   状态：{s.get('state','启动中')}   PID：{s.get('worker_pid','—')}",
            f"连接：{state('connected')}   SDK：{s.get('sdk_version','未加载')}   控制器SERVO：{state('controller_servo')}   使能：{state('enabled')}   上电：{state('powered')}",
            f"物理急停原始位：{state('estop')}   碰撞/保护停原始位：{state('collision')}   软限位：{state('soft_limit')}   安全门：未知",
            f"操作队列约：{s.get('queue_depth','—')}/24   jog输入槽容量：1   已发送：{s.get('samples',0)}   反馈源：{s.get('feedback_source','未连接')}",
            f"进程周期：{s.get('period_ms')} ms   抖动RMS：{s.get('jitter_rms_ms')} ms   反馈年龄：{s.get('feedback_age_ms')} ms（未伺服时含同步状态查询）",
            f"独立反馈接收器错误：{s.get('feedback_receiver_error') or '无/待收包'}",
            f"输入看门狗80ms / 反馈24ms / 调度迟到4ms / SDK调用12ms；状态：{s.get('state')}；外部硬件看门狗：未验收",
            f"运动链滤波：强制开启；指令{s.get('filter_hz',6)}Hz＋Ruckig速度/加速度/jerk约束；均值{self.app.cfg['filters']['moving_average']}点；控制柜LPF策略{self.app.cfg['filters'].get('controller_lpf_policy','preserve')}",
            f"辅助滤波参数v/a/力矩={self.app.cfg['filters']['velocity_hz']}/{self.app.cfg['filters']['acceleration_hz']}/{self.app.cfg['filters']['torque_hz']}Hz（非运动链默认叠加）；力矩接口/单位尚未验收；安全位不滤波",
            f"缩放：速度{setting(self.app.speed)}% / 加速度{setting(self.app.accel)}%；坐标：{setting(self.app.space)}；碰撞检测策略：待现场验收",
            f"最后错误：{s.get('alarm') or '无'}",
            '真机进入条件：'+'；'.join(reasons)]))
        if self.camera and self.camera.poll() is not None:
            report=self.camera_out/'report.json'
            if report.is_file():
                self.camera_report=json.loads(report.read_text(encoding='utf-8')); r=self.camera_report
                self.cam_text.set(f"eye-in-hand | SN请求12474260 / 实测{r.get('serial_actual','—')} | 固件{r.get('firmware_actual','—')} / 期望1523\n打开{r.get('opened')}，图像{r.get('frames',0)}帧；错误：{r.get('error') or '无'}\n标定：in_zedR1_parameters.txt / in_zedR2_parameters.txt / out_zed_parameters.txt；out：相机→TCP tool0，左乘，平移mm；现场配准待验收\nD盘副本：{ROOT/'data/zed/settings'}；报告：{report}")
            else: self.cam_text.set('相机进程失败，查看 '+str(self.camera_out/'process.log'))
            self.camera=None
