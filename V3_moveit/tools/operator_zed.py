"""Official ZED SDK camera worker in an isolated Python environment; no robot access."""
import os,signal,threading,time,json
from pathlib import Path
import cv2,numpy as np,pyzed.sl as sl
from operator_settings import ROOT,RUNTIME,CAMERA_DEFAULTS,camera_preset,atomic
from operator_media import depth_preview
OUT=ROOT/'.cache/operator/media';OUT.mkdir(parents=True,exist_ok=True)
runtime=json.loads(RUNTIME.read_text());preset=camera_preset('zed',runtime.get('camera_modes',CAMERA_DEFAULTS).get('zed',CAMERA_DEFAULTS['zed']))
running=True;owner=os.getppid();states={};lock=threading.Lock()
def status(name,ok,detail,**extra):
    with lock:states[name]=dict(ok=ok,detail=detail,updated=time.monotonic(),**extra)
def heartbeat():
    while running:
        if os.getppid()!=owner:os._exit(0)
        with lock:atomic(OUT/'zed_status.json',dict(pid=os.getpid(),updated=time.monotonic(),devices=dict(states)))
        time.sleep(.2)
for n in ('zed_left','zed_depth'):status(n,False,'ZED SDK 初始化 / 深度模型加载中')
atomic(OUT/'zed_status.json',dict(pid=os.getpid(),updated=time.monotonic(),devices=states))
threading.Thread(target=heartbeat,daemon=True).start()
camera=sl.Camera();params=sl.InitParameters();params.camera_resolution=getattr(sl.RESOLUTION,preset['resolution']);params.camera_fps=preset['fps'];params.coordinate_units=sl.UNIT.METER;params.depth_mode=sl.DEPTH_MODE.NEURAL_LIGHT
params.depth_minimum_distance=.15;params.depth_maximum_distance=5.;params.optional_settings_path=str(ROOT/'.cache/operator/zed_sdk_unpacked/settings')
error=camera.open(params)
if error!=sl.ERROR_CODE.SUCCESS:
    for n in ('zed_left','zed_depth'):status(n,False,'ZED SDK: '+str(error))
    time.sleep(.3);running=False;raise SystemExit(2)
def stop(*args):
    global running
    running=False
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
left=sl.Mat();depth=sl.Mat();last=0.;failures=0
def save(name,img,**extra):
    h,w=img.shape[:2];small=cv2.resize(img,(min(1280,w),round(h*min(1280,w)/w)));ok,data=cv2.imencode('.jpg',small,[cv2.IMWRITE_JPEG_QUALITY,82])
    if not ok:raise RuntimeError('JPEG encode failed')
    tmp=OUT/(name+'.zed.tmp');tmp.write_bytes(data.tobytes());os.replace(tmp,OUT/(name+'.jpg'))
    fps=extra.get('fps');detail=f"官方 SDK 画面在线 · {w} × {h}"+(f" · {fps:.1f} fps" if isinstance(fps,(int,float)) and fps>0 else '')
    status(name,True,detail,width=w,height=h,**extra)
try:
    while running:
        error=camera.grab()
        if error!=sl.ERROR_CODE.SUCCESS:
            for n in ('zed_left','zed_depth'):status(n,False,'ZED SDK: '+str(error))
            failures+=1
            if failures>=5:break
            time.sleep(.05);continue
        failures=0
        now=time.monotonic()
        if now-last<.05:continue
        last=now;camera.retrieve_image(left,sl.VIEW.LEFT);camera.retrieve_measure(depth,sl.MEASURE.DEPTH)
        actual_fps=float(camera.get_current_fps());h,w=left.get_data().shape[:2]
        mode_match=(w==preset['width'] and h==preset['height'] and abs(actual_fps-preset['fps'])<=max(2.,preset['fps']*.15))
        a=depth.get_data();valid=np.isfinite(a)&(a>0)
        save('zed_left',cv2.cvtColor(left.get_data(),cv2.COLOR_BGRA2BGR),fps=actual_fps,mode_match=mode_match,requested_fps=preset['fps'])
        if valid.any():save('zed_depth',depth_preview(a),fps=actual_fps,mode_match=mode_match,requested_fps=preset['fps'],valid_fraction=float(valid.mean()),mode='NEURAL_LIGHT',units='m',sdk=sl.Camera.get_sdk_version())
        else:status('zed_depth',False,'当前帧没有有效深度')
finally:
    running=False;camera.close()
