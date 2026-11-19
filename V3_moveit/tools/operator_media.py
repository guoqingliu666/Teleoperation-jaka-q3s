"""Read-only camera conversion helpers. Invalid depth stays black, never fabricated."""
import numpy as np,cv2

def image_from_ros(msg):
    dtype,channels={'rgb8':('u1',3),'bgr8':('u1',3),'rgba8':('u1',4),'bgra8':('u1',4),'mono8':('u1',1),'16UC1':('u2',1),'32FC1':('f4',1)}[msg.encoding]
    dt=np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<');rowbytes=msg.width*channels*dt.itemsize
    if msg.step<rowbytes or len(msg.data)<msg.step*msg.height:raise ValueError('Short image frame')
    rows=np.frombuffer(bytes(msg.data),dtype='u1',count=msg.step*msg.height).reshape(msg.height,msg.step)[:,:rowbytes].copy()
    a=rows.view(dt).reshape((msg.height,msg.width,channels) if channels>1 else (msg.height,msg.width))
    if msg.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
    elif msg.encoding=='rgba8':a=cv2.cvtColor(a,cv2.COLOR_RGBA2BGR)
    elif msg.encoding=='bgra8':a=cv2.cvtColor(a,cv2.COLOR_BGRA2BGR)
    elif msg.encoding=='mono8':a=cv2.cvtColor(a,cv2.COLOR_GRAY2BGR)
    return a

def depth_preview(depth,units='m',near=.2,far=2.):
    d=depth.astype('f4')/(1000 if units=='mm' else 1)
    valid=np.isfinite(d)&(d>0);normalized=np.clip((d-near)/(far-near),0,1);normalized[~valid]=0
    color=cv2.applyColorMap((255*(1-normalized)).astype('u1'),cv2.COLORMAP_TURBO);color[~valid]=0
    return color

def current_state(state,now,timeout=1.5):
    result=dict(state)
    age=now-result.get('updated',0)
    if result.get('ok') and not 0<=age<timeout:result.update(ok=False,detail='反馈已中断，等待新数据')
    return result
