"""QuestPoseBridge v1/v2 UDP 输入适配，沿用纯解码模块；不拥有 SDK。"""
import json,socket,threading,time,sys
import numpy as np
from ..configuration import ROOT,vector
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from vendor.quest_vr_input import QuestUdpReceiver,quaternion_matrix

class QuestVelocity:
    def __init__(self,basis):
        self.basis=np.array(basis,float)
        if self.basis.shape!=(3,3) or not np.allclose(self.basis@self.basis.T,np.eye(3)): raise ValueError('VR basis invalid')
        self.source=None; self.seq=-1; self.source_time=-1.; self.previous=None; self.release=False; self.gripped=False
        self.click_held=False
    def decode(self,payload,address,stamp):
        frame=QuestUdpReceiver._frame(payload,str(address)); seq=payload.get('sequence'); ts=payload.get('unity_time_s')
        if type(seq) is not int or not isinstance(ts,(int,float)) or not np.isfinite(ts): raise ValueError('VR源时间/序号缺失')
        for k in ('connected','tracked','pose_valid'):
            if type(payload['right'].get(k)) is not bool: raise ValueError('VR tracking flag must be bool')
        if self.source is not None and address!=self.source: raise ValueError('VR source locked')
        if seq<=self.seq or ts<=self.source_time: raise ValueError('VR packet reordered/replayed')
        self.source=address; self.seq=seq; self.source_time=ts
        axes=np.zeros(6); deadman=False
        # 摇杆点击已由同一 Quest 3S 上的旧 XR 和 Input System 双路只读日志实测。
        # 点击仅是软件停止，绝不代替控制柜的物理急停。停止后须先松开 Grip 再握持。
        right=payload['right']
        available=right.get('stick_click_available',False)
        clicked=right.get('thumbstick_click',False)
        if type(available) is not bool or type(clicked) is not bool:
            raise ValueError('VR stick click flag must be bool')
        clicked=available and clicked
        stop_clicked=clicked and not self.click_held
        self.click_held=clicked
        if clicked:
            self.previous=None; self.release=False; self.gripped=False
            return {'axes':axes,'deadman':False,'stamp':stamp,'sequence':seq,
                    'state':'右摇杆软件停止；先松开 Grip','software_stop':stop_clicked}
        if not (frame.connected and frame.tracked and frame.valid and frame.rotation_valid):
            self.previous=None; self.release=False; self.gripped=False
            return {'axes':axes,'deadman':False,'stamp':stamp,'sequence':seq,'state':'追踪无效，松开 Grip 重建','software_stop':False}
        if frame.grip<.3:
            self.release=True; self.gripped=False; self.previous=None
        elif frame.grip>.7 and self.release:
            p=np.array(frame.position_m); r=np.array(quaternion_matrix(frame.rotation_xyzw))
            if self.previous is not None:
                p0,r0,t0=self.previous; dt=ts-t0
                if not 0<dt<=.08 or np.linalg.norm(p-p0)>.08:
                    self.release=False; self.gripped=False; self.previous=None
                    raise ValueError('VR jump/stale; release Grip')
                linear=self.basis@(p-p0)/dt
                dr=r@r0.T; angle=np.arccos(np.clip((np.trace(dr)-1)/2,-1,1))
                if angle>.5:
                    self.release=False; self.gripped=False; self.previous=None
                    raise ValueError('VR orientation jump; release Grip')
                angular=np.zeros(3) if angle<1e-7 else np.array([dr[2,1]-dr[1,2],dr[0,2]-dr[2,0],dr[1,0]-dr[0,1]])*angle/(2*np.sin(angle)*dt)
                angular=np.linalg.det(self.basis)*(self.basis@angular)
                axes=np.clip(np.r_[linear/.25,angular/.5],-1,1)
            self.previous=(p,r,ts); self.gripped=True; deadman=True
        return {'axes':axes,'deadman':deadman,'stamp':stamp,'sequence':seq,
                'state':'Grip跟随' if deadman else '松开/等待Grip','software_stop':False}

class QuestInput:
    def __init__(self,port,basis,mapper=None,audit=None):
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.socket.bind(('127.0.0.1',port)); self.socket.settimeout(.1)
        self.mapper=mapper or QuestVelocity(basis); self.latest=None; self.error='等待 Unity 数据'; self.stop=threading.Event()
        self.received=0;self.decoded=0;self.rejected=0;self.last_receive_stamp=None
        self.audit=audit
        self.software_stop=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True); self.thread.start()
    def run(self):
        while not self.stop.is_set():
            try:
                raw,address=self.socket.recvfrom(65536); stamp=time.perf_counter()
                self.received+=1;self.last_receive_stamp=stamp
                payload=json.loads(raw)
                if self.audit is not None:self.audit(host_receive_s=stamp,peer=address,raw=payload)
                decoded=self.mapper.decode(payload,address,stamp)
                self.decoded+=1
                if decoded.get('software_stop'): self.software_stop.set()
                self.latest=decoded; self.error=''
            except socket.timeout: continue
            except OSError: break
            except Exception as e: self.rejected+=1;self.latest=None; self.error=str(e)
    def close(self): self.stop.set(); self.socket.close(); self.thread.join(.3)
    def take_software_stop(self):
        if not self.software_stop.is_set(): return False
        self.software_stop.clear()
        return True
