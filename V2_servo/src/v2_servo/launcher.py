"""明确区分运行/只读/开发试验的单一启动窗口；默认零SDK。"""
import argparse,datetime,json,os,subprocess,sys
from pathlib import Path
from .configuration import ROOT
from .runtime_paths import load_runtime_options,sdk_artifacts

PLAYER=ROOT/'digital_twin/unity/Build/V2Twin.exe'

def check_runtime():
    result=dict(root=str(ROOT),python=sys.executable,SDK_connection_attempts=0,robot_commands=0,
        continuous_live_released=False,blockers=['连续新控制器尚无真机时序/跟踪/停止验收','全臂/工具/线缆碰撞边界尚未现场验收'])
    try:
        import numpy,ruckig
        result['numpy']=numpy.__version__;result['ruckig_import']=True
    except ImportError as exc:result.setdefault('setup_errors',[]).append(str(exc))
    try:
        options=load_runtime_options();result['SDK_directory']=options['sdk_directory']
        result['SDK_inside_project']=Path(options['sdk_directory']).resolve().is_relative_to(ROOT)
        result['SDK_artifacts']=sdk_artifacts(options['sdk_directory'])
    except (OSError,ValueError,KeyError) as exc:result['SDK_not_bundled_public_shadow_only']=str(exc)
    result['Player_present']=PLAYER.is_file()
    try:
        from .teleop.continuous_shadow import load_model
        model,_,path=load_model();result['global_model_evidence']=str(path)
        result['shadow_tool']=getattr(model,'tool_provenance',None)
    except (OSError,ValueError,KeyError) as exc:result['shadow_model_error']=str(exc)
    return result

class Launcher:
    def __init__(self,root):
        import tkinter as tk
        from tkinter import ttk
        self.root=root;self.shadow=None;self.session=None;self.live=None;self.owned_player=None;self.log_handle=None
        root.title('V2 伺服遥操作 · 统一入口');root.geometry('1050x760');root.minsize(960,700)
        ttk.Style().configure('.',font=('Microsoft YaHei UI',10))
        panel=ttk.Frame(root,padding=18);panel.pack(fill='both',expand=True)
        ttk.Label(panel,text='V2 伺服遥操作',font=('Microsoft YaHei UI',20,'bold')).pack(anchor='w')
        ttk.Label(panel,text='连续大范围：影子可运行，真机未验收。H9仅代表已有局部真机记录。',foreground='#9a4e00').pack(anchor='w',pady=(4,12))
        tabs=ttk.Notebook(panel);tabs.pack(fill='x')
        regular=ttk.Frame(tabs,padding=12);advanced=ttk.Frame(tabs,padding=12)
        tabs.add(regular,text='日常入口');tabs.add(advanced,text='开发 / 受监督试验')
        ttk.Label(regular,text='影子姿态映射（只在停止后切换；一比一候选仍保留全部保护）').pack(anchor='w')
        self.orientation=ttk.Combobox(regular,state='readonly',values=('半倍率（原映射对照）','一比一姿态（影子候选）'))
        self.orientation.current(0);self.orientation.pack(fill='x')
        ttk.Button(regular,text='连续遥操作影子：Quest → 候选规划 → 模拟孪生（不动真机）',command=self.start_shadow).pack(fill='x',pady=4)
        ttk.Button(regular,text='Quest 追踪 / 按键只读诊断（不连接机器人）',command=self.quest_diagnostics).pack(fill='x',pady=4)
        ttk.Button(regular,text='原有仿真 GUI：键盘 jog / 录制 / 回放',command=self.sim_gui).pack(fill='x',pady=4)
        ttk.Button(regular,text='连续大范围真机伺服：等待真机和安全验收',state='disabled').pack(fill='x',pady=4)
        ttk.Label(advanced,text='这是工程试验入口；不代表正式发布或连续全空间放行。').pack(anchor='w')
        ttk.Button(advanced,text='真机入口未随公开包提供',state='disabled').pack(fill='x',pady=7)
        ttk.Button(advanced,text='打开开发 / 测试入口说明',command=lambda:self.open_file('development/README.md')).pack(fill='x')
        bar=ttk.Frame(panel);bar.pack(fill='x',pady=12)
        self.check_button=ttk.Button(bar,text='文件 / 依赖诊断（零SDK）',command=self.diagnose);self.check_button.pack(side='left')
        ttk.Button(bar,text='停止影子任务',command=self.stop_shadow).pack(side='left',padx=8)
        ttk.Button(bar,text='使用说明',command=lambda:self.open_file('docs/runtime_user_guide.md')).pack(side='left')
        self.message=tk.StringVar(value='请选择明确标注的模式。软件停止与物理急停不同。')
        ttk.Label(panel,textvariable=self.message,wraplength=980).pack(anchor='w')
        self.text=tk.Text(panel,height=14,wrap='word',font=('Consolas',10));self.text.pack(fill='both',expand=True,pady=8)
        self.diagnose();root.protocol('WM_DELETE_WINDOW',self.close);root.after(300,self.poll)
    def open_file(self,relative):os.startfile(str(ROOT/relative))
    def diagnose(self):
        self.text.delete('1.0','end');self.text.insert('end',json.dumps(check_runtime(),ensure_ascii=False,indent=2))
    def busy(self):
        if self.shadow is not None and self.shadow.poll() is None:return True
        if self.live is not None and self.live.poll() is None:return True
        return False
    def environment(self):
        env=os.environ.copy();env['V2_PYTHON']=sys.executable
        env['PYTHONPATH']=os.pathsep.join([str(ROOT/'src'),str(ROOT/'.deps'),str(ROOT)])
        env['TEMP']='D:/ChatGPT/Temp';env['TMP']=env['TEMP'];return env
    def player(self,args,env,folder):
        # 不结束、不复用其他会话的Player；不同模式不得隐式抢占输入。
        # tasklist 的中文“没有任务”输出可能是 OEM/GBK，不能假定跟 Python 一样为 UTF-8。
        active=subprocess.run(['tasklist.exe','/FI','IMAGENAME eq V2Twin.exe','/FO','CSV','/NH'],capture_output=True,creationflags=0x08000000)
        if active.returncode:raise RuntimeError('无法检查已有Player，请检查tasklist权限。')
        if b'V2Twin.exe' in active.stdout:raise RuntimeError('请先正常关闭已有V2Twin，再从统一入口选择模式。')
        if not PLAYER.is_file():raise RuntimeError('缺V2Twin.exe；按说明构建现有Unity工程。')
        folder.mkdir(parents=True,exist_ok=True)
        self.owned_player=subprocess.Popen([str(PLAYER),*args,'-screen-fullscreen','0','-screen-width','1280','-screen-height','800','-logFile',str(folder/'player.log')],cwd=ROOT,env=env)
    def start_shadow(self):
        try:
            if self.busy():raise RuntimeError('已有任务运行，请先停止。')
            from .teleop.continuous_shadow import load_model
            load_model()
            name='continuous_quest_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.session=ROOT/'data'/name;display=ROOT/'logs'/name
            env=self.environment();env['V2_TWIN_DATA_DIR']=str(self.session)
            self.player(['--quest-j1-input'],env,display)
            self.log_handle=(display/'shadow_console.log').open('w',encoding='utf-8')
            orientation='one-to-one' if self.orientation.current()==1 else 'frozen'
            self.shadow=subprocess.Popen([sys.executable,'-m','v2_servo.teleop.continuous_shadow','--duration','3600','--orientation',orientation,'--session',str(self.session)],cwd=ROOT,env=env,
                stdout=self.log_handle,stderr=subprocess.STDOUT,creationflags=0x08000000)
            self.orientation.configure(state='disabled')
            self.message.set('影子运行：不连接机器人。握Grip自然移动；松手减速；最长1小时为本次观察时长。')
        except Exception as exc:self.message.set(str(exc))
    def quest_diagnostics(self):
        try:
            if self.busy():raise RuntimeError('先停止当前任务。')
            folder=ROOT/'data'/('quest_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            env=self.environment();env['V2_QUEST_DIAGNOSTICS_DIR']=str(folder);env['V2_TWIN_DATA_DIR']=str(ROOT/'digital_twin/data')
            self.player(['--quest-diagnostics'],env,folder);self.message.set('只读Quest诊断已打开，没有机器人SDK。')
        except Exception as exc:self.message.set(str(exc))
    def sim_gui(self):
        if self.busy():self.message.set('先停止当前任务。');return
        subprocess.Popen([sys.executable,str(ROOT/'gui/main.py')],cwd=ROOT,env=self.environment(),creationflags=0x08000000)
    def workspace_test(self):
        self.message.set('公开V2.0.0只允许影子运行；不包含SDK，不授予真机权限。');return
        if self.busy():self.message.set('先停止当前任务。');return
        self.live=subprocess.Popen(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'scripts/start_quest_pose_workspace.ps1')],
            cwd=ROOT,env=self.environment(),creationflags=0x00000010)
        self.message.set('局部试验由单独黑窗控制。只有黑窗现场确认后才连接机器人；停止用手柄点击/黑窗Ctrl+C，异常物理急停。')
    def stop_shadow(self):
        if self.shadow is not None and self.shadow.poll() is None and self.session.is_dir():
            (self.session/'STOP.request').write_text('operator stop',encoding='utf-8');self.message.set('已请求影子减速停止；不是机器人急停。')
    def poll(self):
        if self.session is not None and (self.session/'status.json').exists():
            try:
                self.text.delete('1.0','end');self.text.insert('end',(self.session/'status.json').read_text(encoding='utf-8'))
            except OSError:pass
        if self.shadow is not None and self.shadow.poll() is not None:
            self.message.set('影子任务结束，日志：'+str(self.session))
            self.shadow=None
            self.orientation.configure(state='readonly')
            if self.log_handle:self.log_handle.close();self.log_handle=None
        self.root.after(300,self.poll)
    def close(self):
        if self.live is not None and self.live.poll() is None:
            self.message.set('局部真机黑窗仍在运行，请先在黑窗停止并退出；此窗口不会擅自杀死控制进程。');return
        self.stop_shadow();self.root.destroy()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');p.add_argument('--ui-probe',type=Path)
    args=p.parse_args()
    if args.check:
        r=check_runtime();print(json.dumps(r,ensure_ascii=False,indent=2));return 1 if r.get('setup_errors') else 0
    import tkinter as tk
    root=tk.Tk();app=Launcher(root)
    if args.ui_probe:
        out=args.ui_probe.resolve()
        if not out.is_relative_to(ROOT/'data'):raise ValueError('UI probe output must be in V2/data')
        out.mkdir(parents=True,exist_ok=True)
        def capture():
            try:
                import ctypes
                from PIL import ImageGrab
                app.check_button.invoke();root.update()
                # Capture this window itself, not another foreground app covering its rectangle.
                user=ctypes.windll.user32;user.GetAncestor.argtypes=[ctypes.c_void_p,ctypes.c_uint];user.GetAncestor.restype=ctypes.c_void_p
                hwnd=user.GetAncestor(root.winfo_id(),2)
                ImageGrab.grab(window=hwnd).save(out/'launcher.png')
                (out/'report.json').write_text(json.dumps(dict(scope='actual_Tk_launcher_ui_only',file_check_button_invoked=True,robot_commands=0),indent=2),encoding='utf-8')
            finally:root.after(100,root.destroy)
        root.after(1600,capture)
    root.mainloop();return 0
