"""阶段 B 只读进程。仅 login/get/logout；不使能、不设置 EtherCAT 状态。"""
import json, os, socket, sys, time
from pathlib import Path
from .episode import event
ROOT=Path(__file__).resolve().parents[3]

def emit(q, value):
    # 这是独立采集进程，可以背压；不在机器人伺服线程调用。
    q.put(value,timeout=2)

def s5_readonly(q,stop):
    from ..jaka import load_sdk,checked,decode_status
    from ..teleop.backends import controller_owners
    cfg=json.loads((ROOT/'config/teleop.json').read_text());rc=None;logged=False
    try:
        owners=controller_owners(cfg['host'])
        if owners:raise RuntimeError(f'controller already owned by PID {owners}; user must disconnect; no process terminated')
        with socket.create_connection((cfg['host'],10001),timeout=2):pass
        sdk=load_sdk(cfg['sdk_directory']);rc=sdk.RC(cfg['host']);checked(rc.login(),'login');logged=True
        emit(q,{'info':{'s5_sdk':checked(rc.get_sdk_version(),'get_sdk_version'),'s5_host':cfg['host']}})
        seq=0
        while not stop.is_set():
            start=time.perf_counter_ns();raw=rc.get_robot_status();end=time.perf_counter_ns()
            f=decode_status(raw,start/1e9,seq)
            emit(q,event('s5_state',seq,{'q_rad':f.q.tolist(),'velocity_rad_s':None,'torque_Nm':None,
                 'tcp_mm_rad':None,'enabled':f.enabled,'estop':f.estop,'collision':f.collision,
                 'error':f.error,'raw_status':raw,'source':'SDK get_robot_status; controller sampling time unavailable'},
                 host_ns=end,start_ns=start,end_ns=end,clock='host_receive_only',uncertainty_ns=None))
            seq+=1;stop.wait(.008)
    except Exception as exc:emit(q,{'error':'s5_readonly: '+repr(exc)})
    finally:
        if logged:
            try:checked(rc.logout(),'logout')
            except Exception as exc:emit(q,{'error':'s5 logout: '+repr(exc)})

def zed_readonly(q,stop):
    import numpy as np
    sys.path.insert(0,str(ROOT/'.deps'))
    cfg=json.loads((ROOT/'config/zed.json').read_text());handles=[];camera=None
    try:
        for d in ('bin','dependencies'):
            handles.append(os.add_dll_directory(str(Path(cfg['sdk_directory'])/d)))
        import pyzed.sl as sl
        camera=sl.Camera();init=sl.InitParameters();init.set_from_serial_number(12474260)
        init.camera_resolution=sl.RESOLUTION.HD720;init.camera_fps=30
        init.coordinate_units=sl.UNIT.METER;init.depth_mode=sl.DEPTH_MODE.PERFORMANCE
        init.optional_settings_path=str(ROOT/cfg['staged_settings_directory'])
        init.sdk_verbose_log_file=str(ROOT/'logs/act_zed_sdk.log')
        code=camera.open(init)
        if code!=sl.ERROR_CODE.SUCCESS:raise RuntimeError(str(code))
        info=camera.get_camera_information();cc=info.camera_configuration
        if info.serial_number!=12474260 or int(cc.firmware_version)!=1523:raise RuntimeError('ZED identity mismatch')
        def intrinsics(c):return {'fx':c.fx,'fy':c.fy,'cx':c.cx,'cy':c.cy,'disto':np.asarray(c.disto).tolist()}
        emit(q,{'info':{'zed_serial':info.serial_number,'zed_firmware':int(cc.firmware_version),
          'zed_sdk':sl.Camera.get_sdk_version(),'zed_resolution':[1280,720],'zed_fps':30,'mount':'eye-in-hand',
          'left_intrinsics':intrinsics(cc.calibration_parameters.left_cam),
          'right_intrinsics':intrinsics(cc.calibration_parameters.right_cam)}})
        mats=[sl.Mat(),sl.Mat(),sl.Mat()];seq=0;runtime=sl.RuntimeParameters()
        while not stop.is_set():
            start=time.perf_counter_ns();code=camera.grab(runtime);end=time.perf_counter_ns()
            if code!=sl.ERROR_CODE.SUCCESS:raise RuntimeError('grab '+str(code))
            source=camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            # 括住 CURRENT 查询，只能估计设备 UNIX -> QPC 的时钟映射。
            a=time.perf_counter_ns();current=camera.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_nanoseconds();b=time.perf_counter_ns()
            mapped=int(source-current+(a+b)//2)
            for name,mat,kind in zip(('zed_left','zed_right','zed_depth'),mats,(sl.VIEW.LEFT,sl.VIEW.RIGHT,sl.MEASURE.DEPTH)):
                code=camera.retrieve_measure(mat,kind) if name=='zed_depth' else camera.retrieve_image(mat,kind)
                if code!=sl.ERROR_CODE.SUCCESS:raise RuntimeError(name+' '+str(code))
                pixels=mat.get_data().copy()
                if name!='zed_depth':pixels=pixels[:,:,[2,1,0]].copy()
                e=event(name,seq,pixels,host_ns=mapped,start_ns=start,end_ns=end,source_ns=int(source),
                  clock='ZED_IMAGE_UNIX_ns mapped via bracketed CURRENT; physical error unvalidated',uncertainty_ns=None)
                e['mapping_bracket_ns']=b-a;e['received_ns']=end
                emit(q,e)
            seq+=1
    except Exception as exc:emit(q,{'error':'zed_readonly: '+repr(exc)})
    finally:
        if camera is not None:camera.close()

def scene_readonly(q,stop,device):
    import cv2
    cap=None
    try:
        cap=cv2.VideoCapture(device,cv2.CAP_DSHOW);seq=0
        if not cap.isOpened():raise RuntimeError('scene camera open failed')
        emit(q,{'info':{'scene_device':device,'scene_width':cap.get(cv2.CAP_PROP_FRAME_WIDTH),
             'scene_height':cap.get(cv2.CAP_PROP_FRAME_HEIGHT),'scene_fps_reported':cap.get(cv2.CAP_PROP_FPS)}})
        while not stop.is_set():
            a=time.perf_counter_ns();ok,image=cap.read();b=time.perf_counter_ns()
            if not ok:raise RuntimeError('scene read failed')
            emit(q,event('scene',seq,cv2.cvtColor(image,cv2.COLOR_BGR2RGB),host_ns=b,start_ns=a,end_ns=b));seq+=1
    except Exception as exc:emit(q,{'error':'scene_readonly: '+repr(exc)})
    finally:
        if cap is not None:cap.release()
