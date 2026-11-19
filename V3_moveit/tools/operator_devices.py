"""Independent sensor monitor: camera reads and ROS subscriptions only; no arm/hand SDK."""
import json,os,signal,threading,time,subprocess,sys
from pathlib import Path
import cv2
from operator_settings import ROOT,RUNTIME,CAMERA_DEFAULTS,camera_preset,atomic
from operator_media import image_from_ros,depth_preview,current_state
OUT=ROOT/'.cache/operator/media';OUT.mkdir(exist_ok=True,parents=True)
runtime=json.loads(RUNTIME.read_text());profile=runtime['profile'];camera_modes=runtime.get('camera_modes',CAMERA_DEFAULTS);running=True;owner=os.getppid();lock=threading.Lock();states={};last_frame={}
device=sys.argv[1] if len(sys.argv)>1 else 'all'
if device not in ('all','hand','zed','c922'):raise SystemExit('device must be all, hand, zed, or c922')
if device!='all':profile['enabled']={name:(name==device) for name in profile['enabled']}
STATUS=OUT/('status.json' if device=='all' else f'monitor_{device}.json')
def stop(*args):
    global running
    running=False
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
def status(name,ok,detail,**extra):
    with lock:states[name]=dict(ok=ok,detail=detail,updated=time.monotonic(),**extra)
def frame(name,img,detail='画面在线',**extra):
    now=time.monotonic()
    if now-last_frame.get(name,0)<.05:return
    last_frame[name]=now;h,w=img.shape[:2];preview=cv2.resize(img,(min(1280,w),max(1,round(h*min(1280,w)/w))))
    ok,data=cv2.imencode('.jpg',preview,[cv2.IMWRITE_JPEG_QUALITY,82])
    if ok:
        tmp=OUT/(name+'.tmp');tmp.write_bytes(data.tobytes());os.replace(tmp,OUT/(name+'.jpg'))
        status(name,True,detail,width=w,height=h,**extra)
def device_path(token):
    paths=sorted(Path('/dev/v4l/by-id').glob('*'+token+'*video-index0'))
    return str(paths[0]) if len(paths)==1 else None

def uvc(name,token,stereo=False):
    path=device_path(token)
    if not path:status(name,False,'未检测到相机 / 同型号设备不唯一');return
    camera='zed' if stereo else 'c922';preset=camera_preset(camera,camera_modes.get(camera,CAMERA_DEFAULTS[camera]));expected_width=preset['width'];expected_height=preset['height'];expected_capture_width=expected_width*(2 if stereo else 1)
    cap=cv2.VideoCapture(path,cv2.CAP_V4L2)
    try:
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*('YUYV' if stereo else 'MJPG')))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,expected_capture_width);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,expected_height);cap.set(cv2.CAP_PROP_FPS,preset['fps']);cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        if not cap.isOpened():status(name,False,'相机无法打开');return
        misses=0;window_started=time.monotonic();window_frames=0;actual_fps=float(cap.get(cv2.CAP_PROP_FPS) or 0.);mode_match=True
        while running:
            ok,img=cap.read()
            if not ok:
                misses+=1;status(name,False,'相机读帧失败');time.sleep(.1)
                if misses>20:return
                continue
            misses=0
            window_frames+=1;now=time.monotonic()
            if stereo:
                if img.shape[1]/img.shape[0]<2.8:status(name,False,'未取得双目左右拼接帧');continue
                img=img[:,:img.shape[1]//2]
            height,width=img.shape[:2];dimensions_match=(width==expected_width and height==expected_height)
            if now-window_started>=.5:
                actual_fps=window_frames/(now-window_started);window_frames=0;window_started=now
            mode_match=dimensions_match and abs(actual_fps-preset['fps'])<=max(2.,preset['fps']*.15)
            detail=f"{width} × {height} · {actual_fps:.1f} fps"
            if not mode_match:detail+=f"（所选 {expected_width} × {expected_height} · {preset['fps']} fps 未完全达到）"
            frame(name,img,detail,fps=actual_fps,mode_match=mode_match,requested_fps=preset['fps'])
    finally:cap.release()
def zed():
    if not (ROOT/'.venv-zed/bin/python').exists() or not (ROOT/'.cache/operator/zed_sdk_unpacked/lib/libsl_zed.so').exists():
        status('zed_depth',False,'缺少 Linux ZED SDK，深度不可用');uvc('zed_left','ZED-M',True);return
    log=(ROOT/'artifacts/operator_zed.log').open('w');child=subprocess.Popen(['bash','tools/operator_zed.sh','tools/operator_zed.py'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    failure='ZED SDK 未提供深度'
    try:
        while running:
            try:
                data=json.loads((OUT/'zed_status.json').read_text())
                if data['pid']==child.pid and time.monotonic()-data['updated']<1.5:
                    with lock:states.update(data['devices'])
                    failure=data['devices'].get('zed_depth',{}).get('detail',failure)
            except (OSError,ValueError,KeyError):pass
            if child.poll() is not None:break
            time.sleep(.1)
    finally:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        log.close()
    if running:
        status('zed_depth',False,failure);uvc('zed_left','ZED-M',True)
def ros_monitor():
    import rclpy,math
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState,Image
    os.environ['ROS_DOMAIN_ID']=str(profile['sensor_domain']);rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('v3_operator_sensor_monitor',start_parameter_services=False,enable_rosout=False)
    def hand(msg):
        age=(node.get_clock().now().nanoseconds-msg.header.stamp.sec*1000000000-msg.header.stamp.nanosec)/1e9
        if not 0<=age<1.5:status('hand',False,'DH116 反馈时间戳失效');return
        ids=dict(zip(msg.joint_ids,msg.angles))
        if not all(i in ids and math.isfinite(ids[i]) for i in range(1,7)):
            status('hand',False,'DH116 六轴反馈不完整');return
        names=['finger11','finger12','finger21','finger31','finger41','finger51'];q={name:math.radians(ids[i+1]) for i,name in enumerate(names)}
        # Vendor URDF linear mimic relationships; follower joints are modeled.
        for child,parent,k,b in [('finger13','finger12',1.2334,.004592),('finger22','finger21',1.1411,.011072),('finger32','finger31',1.1411,.011072),('finger42','finger41',1.1376,.011072),('finger52','finger51',1.1516,.011072)]:q[child]=q[parent]*k+b
        alarm=any(msg.alarms)
        status('hand',not alarm,'DH116 报警' if alarm else '六轴反馈在线 · 连杆按厂家模型显示',joint_names=list(q),position=list(q.values()),alarms=list(msg.alarms),currents=list(msg.currents))
    if profile['enabled']['hand']:
        try:
            from hand_control_interfaces.msg import HandJointState
            node.create_subscription(HandJointState,profile['hand_topic'],hand,qos_profile_sensor_data)
        except ImportError:status('hand',False,'厂家 ROS 消息接口未构建')
    def image_cb(name,msg):
        try:
            a=image_from_ros(msg)
            if name=='zed_depth':
                if msg.encoding not in ('16UC1','32FC1'):raise ValueError('Depth requires 16UC1 or 32FC1')
                a=depth_preview(a,'mm' if msg.encoding=='16UC1' else 'm')
            frame(name,a)
        except Exception as e:status(name,False,'图像格式错误：'+str(e))
    if profile['camera_source']=='ros':
        for name,key,enabled in [('zed_left','zed_left_topic','zed'),('zed_depth','zed_depth_topic','zed'),('c922','c922_topic','c922')]:
            if profile['enabled'][enabled]:node.create_subscription(Image,profile[key],lambda msg,n=name:image_cb(n,msg),qos_profile_sensor_data)
    try:
        while running:rclpy.spin_once(node,timeout_sec=.05)
    finally:node.destroy_node();rclpy.shutdown()
threads=[]
def launch(fn,*args):
    def safe():
        while running:
            try:fn(*args)
            except Exception as e:status(args[0] if args else 'monitor',False,str(e))
            if fn not in (uvc,zed):return
            # Read-only USB hotplug recovery; never touches the arm/hand bus.
            for _ in range(30):
                if not running:return
                time.sleep(.1)
    t=threading.Thread(target=safe,daemon=True);t.start();threads.append(t)
if profile['enabled']['hand']:status('hand',False,'等待 DH116 厂家驱动反馈')
if profile['camera_source']=='local':
    if profile['enabled']['c922']:launch(uvc,'c922','C922')
    if profile['enabled']['zed']:launch(zed)
if profile['enabled']['hand'] or profile['camera_source']=='ros':launch(ros_monitor)
try:
    while running and os.getppid()==owner:
        with lock:data={k:current_state(v,time.monotonic()) for k,v in states.items()}
        atomic(STATUS,dict(pid=os.getpid(),updated=time.monotonic(),devices=data));time.sleep(.2)
finally:
    running=False
    for t in threads:t.join(timeout=4)
