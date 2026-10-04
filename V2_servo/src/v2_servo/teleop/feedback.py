"""官方 1.7.2 的 10000 JSON 反馈：跨进程被动接收，处理拆包/粘包，禁止重打时标。"""
import codecs,json,socket,time,queue
import numpy as np
from ..configuration import vector
from ..safety import Feedback

class JsonFrames:
    def __init__(self): self.buffer=''; self.decoder=json.JSONDecoder(); self.utf8=codecs.getincrementaldecoder('utf-8')()
    def feed(self,data):
        self.buffer+=self.utf8.decode(data)
        if len(self.buffer)>262144: raise ValueError('feedback buffer overflow')
        frames=[]
        while self.buffer.strip():
            self.buffer=self.buffer.lstrip('\r\n \0\t')
            if not self.buffer: break
            if self.buffer[0]!='{': raise ValueError('unknown feedback framing')
            try: value,end=self.decoder.raw_decode(self.buffer)
            except json.JSONDecodeError: break
            if not isinstance(value,dict): raise ValueError('feedback not object')
            frames.append(value); self.buffer=self.buffer[end:]
        return frames

def strict_flag(data,key):
    v=data[key]
    if type(v) not in (int,bool) or v not in (0,1): raise ValueError('invalid '+key)
    return bool(v)

def decode_frame(data,received,seq):
    q=np.deg2rad(vector(data['joint_actual_position']))
    error=data['errcode']; error=int(error,0) if isinstance(error,str) else error
    if type(error) is not int: raise ValueError('unknown error schema')
    fb=Feedback(q,received,seq,enabled=strict_flag(data,'enabled'),powered=strict_flag(data,'powered_on'),
                estop=strict_flag(data,'emergency_stop'),collision=strict_flag(data,'protective_stop'),
                soft_limit=strict_flag(data,'on_soft_limit'),error=error,torque=None)
    tcp=vector(data['actual_position']); tcp[3:]=np.deg2rad(tcp[3:])
    return fb,tcp

def feedback_process(host,out,stop):
    seq=0
    try:
        with socket.create_connection((host,10000),timeout=2) as sock:
            sock.settimeout(.2); parser=JsonFrames()
            while not stop.is_set():
                try: raw=sock.recv(65536)
                except socket.timeout: continue
                if not raw: raise ConnectionError('feedback disconnected')
                stamp=time.perf_counter(); frames=parser.feed(raw)
                if not frames: continue
                seq+=len(frames); fb,tcp=decode_frame(frames[-1],stamp,seq)
                record={'feedback':fb,'tcp':tcp,'received':stamp,'scope':'host_receive_timestamp_not_controller_sample_time'}
                try: out.put_nowait(record)
                except queue.Full:
                    try: out.get_nowait()
                    except queue.Empty: pass
                    try: out.put_nowait(record)
                    except queue.Full: pass
    except Exception as e:
        try: out.put_nowait({'error':str(e)})
        except queue.Full: pass
