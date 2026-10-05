"""实际统一入口启动/停止影子子进程，不发送机器人命令。"""
import argparse,ctypes,datetime,json,os,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'.deps'),str(ROOT)];sys.dont_write_bytecode=True
for key in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[key]='1'
os.environ['TEMP']='D:/ChatGPT/Temp';os.environ['TMP']=os.environ['TEMP'];os.environ['PYTHONDONTWRITEBYTECODE']='1'
import tkinter as tk
from v2_servo.launcher import Launcher

def main():
    p=argparse.ArgumentParser();p.add_argument('--one-to-one',action='store_true');args=p.parse_args()
    root=tk.Tk();app=Launcher(root);report={'scope':'actual_launcher_shadow_process_smoke_no_robot','robot_commands':0}
    out=ROOT/'data'/('launcher_shadow_smoke_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    process=None
    try:
        root.update()
        if args.one_to_one:app.orientation.current(1)
        app.start_shadow();process=app.owned_player
        if app.shadow is None:raise RuntimeError(app.message.get())
        report['mode_switch_disabled_while_running']=str(app.orientation.cget('state'))=='disabled'
        report['session']=str(app.session);start=time.perf_counter()
        while time.perf_counter()-start<10:
            root.update();time.sleep(.03)
            if app.shadow is None:raise RuntimeError('Shadow child exited early')
        from PIL import ImageGrab
        root.lift();root.update()
        ImageGrab.grab(bbox=(root.winfo_rootx(),root.winfo_rooty(),root.winfo_rootx()+root.winfo_width(),root.winfo_rooty()+root.winfo_height())).save(out/'launcher.png')
        app.stop_shadow();start=time.perf_counter()
        while app.shadow is not None and time.perf_counter()-start<6:root.update();time.sleep(.03)
        r=json.loads((app.session/'report.json').read_text(encoding='utf-8'))
        expected='one-to-one' if args.one_to_one else 'frozen'
        report['mode_switch_enabled_after_stop']=str(app.orientation.cget('state'))=='readonly'
        report.update(child_report=r,passed=r['passed_process_only'] and app.shadow is None and r['orientation_mode']==expected and
            report['mode_switch_disabled_while_running'] and report['mode_switch_enabled_after_stop'],
            controller_follow_verified=r['input_follow_observed'])
    except Exception as exc:report.update(passed=False,failure=str(exc))
    finally:
        app.stop_shadow();root.destroy()
        if process is not None:
            callback_type=ctypes.WINFUNCTYPE(ctypes.c_bool,ctypes.c_void_p,ctypes.c_void_p)
            def close(hwnd,_):
                pid=ctypes.c_ulong();ctypes.windll.user32.GetWindowThreadProcessId(ctypes.c_void_p(hwnd),ctypes.byref(pid))
                if pid.value==process.pid:ctypes.windll.user32.PostMessageW(ctypes.c_void_p(hwnd),0x0010,0,0)
                return True
            callback=callback_type(close);ctypes.windll.user32.EnumWindows(callback,0)
            try:report['player_exit_code']=process.wait(12)
            except Exception as exc:report.update(passed=False,cleanup_error=str(exc))
        (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(out=str(out),passed=report.get('passed'),failure=report.get('failure'),session=report.get('session')),ensure_ascii=False))
    return 0 if report.get('passed') else 2

if __name__=='__main__':raise SystemExit(main())
