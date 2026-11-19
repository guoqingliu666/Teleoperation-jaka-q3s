"""Maintain Quest USB forwarding and the local HTTPS relay; no robot access."""
import argparse,fcntl,json,os,signal,ssl,subprocess,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def run(args,timeout=12):
    result=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
    if result.returncode:raise RuntimeError((result.stderr or result.stdout).strip() or '命令执行失败：'+args[0])
    return result.stdout

def https_ready():
    try:
        with urllib.request.urlopen('https://127.0.0.1:8443/xr/health',context=ssl._create_unverified_context(),timeout=2) as response:
            data=json.loads(response.read(4096))
            return response.status==200 and data.get('service')=='v3-quest-workbench' and data.get('version')==1
    except (OSError,ValueError):return False

def ensure_server():
    if https_ready():return
    folder=ROOT/'.cache/quest-tls';folder.mkdir(parents=True,exist_ok=True)
    cert=folder/'localhost-usb-20261112.crt';key=folder/'localhost-usb-20261112.key'
    if not cert.exists() or not key.exists():
        run(['openssl','req','-x509','-newkey','rsa:2048','-nodes','-days','365','-keyout',str(key),'-out',str(cert),'-subj','/CN=localhost','-addext','subjectAltName=DNS:localhost,IP:127.0.0.1']);key.chmod(0o600)
    with (ROOT/'artifacts/operator_quest_server.log').open('a') as log:
        child=subprocess.Popen(['/usr/bin/python3','-B',str(ROOT/'tools/operator_xr/server.py'),'--cert',str(cert),'--key',str(key)],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    for _ in range(30):
        if child.poll() is not None:raise RuntimeError('Quest HTTPS 服务未启动，请检查 8443 端口占用及 operator_quest_server.log。')
        if https_ready():return
        time.sleep(.1)
    raise RuntimeError('Quest HTTPS 服务尚未就绪，请重试连接。')

def authorized_device(text):
    rows=[r.split() for r in text.splitlines()[1:] if r.strip()]
    devices=[r[0] for r in rows if len(r)>=2 and r[1]=='device']
    if len(devices)>1:raise RuntimeError('检测到多台 USB 调试设备，请只连接本次使用的 Quest。')
    if not devices:
        if any(len(r)>1 and r[1]=='unauthorized' for r in rows):raise RuntimeError('请在 Quest 中允许这台电脑进行 USB 调试。')
        raise RuntimeError('Quest USB 未连接或离线，请检查数据线并唤醒头显。')
    return devices[0]

def has_reverse(text):return any(row.split()[-2:]==['tcp:8443','tcp:8443'] for row in text.splitlines())

def ensure_connection(force=False):
    (ROOT/'.cache/operator').mkdir(parents=True,exist_ok=True);(ROOT/'artifacts').mkdir(exist_ok=True)
    with (ROOT/'.cache/operator/quest_link.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        ensure_server();serial=authorized_device(run(['adb','devices']))
        repaired=force or not has_reverse(run(['adb','-s',serial,'reverse','--list']))
        if repaired:run(['adb','-s',serial,'reverse','tcp:8443','tcp:8443'])
        if not has_reverse(run(['adb','-s',serial,'reverse','--list'])):raise RuntimeError('Quest 8443 USB 转发未建立，请重连数据线。')
    return dict(ok=True,serial=serial,reverse_repaired=repaired,detail='USB / HTTPS 已就绪 · 在 Quest 打开 https://localhost:8443/')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--watch',action='store_true');parser.add_argument('--force',action='store_true');args=parser.parse_args()
    running=True
    def stop(*_):
        nonlocal running
        running=False
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,stop)
    owner=os.getppid();previous=None
    while running and os.getppid()==owner:
        try:state=ensure_connection(force=args.force)
        except (OSError,RuntimeError,subprocess.SubprocessError) as exc:state=dict(ok=False,detail=str(exc))
        signature=(state.get('ok'),state.get('serial'),state.get('detail'))
        if signature!=previous or state.get('reverse_repaired') or not args.watch:
            print(json.dumps(dict(event='quest_link',**state),ensure_ascii=False),flush=True);previous=signature
        if not args.watch:return 0 if state['ok'] else 1
        for _ in range(30):
            if not running or os.getppid()!=owner:break
            time.sleep(.1)
    return 0

if __name__=='__main__':raise SystemExit(main())
