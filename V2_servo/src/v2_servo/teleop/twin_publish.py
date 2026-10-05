"""GUI进程向数字孪生发布只读状态；有界最新槽、独立线程，不参与8ms伺服。"""
import json,queue,socket,threading,time

class TwinPublisher:
    def __init__(self,ports=(5007,5008)):
        self.ports=ports; self.latest=queue.Queue(maxsize=1); self.stop=threading.Event(); self.dropped=0
        self.thread=threading.Thread(target=self._run,name='V2-twin-status',daemon=True); self.thread.start()
    def offer(self,s):
        age=s.get('feedback_age_ms'); valid=(bool(s.get('connected')) and s.get('q') is not None and
            isinstance(age,(int,float)) and 0<=age<=250)
        packet={'schema':1,'mode':s.get('mode'),'state':s.get('state'),'connected':bool(s.get('connected')),
            'active':bool(s.get('active')),'q_rad':s.get('q'),'tcp_mm_rad':s.get('tcp'),
            'feedback_age_ms':age,'feedback_valid':valid,'alarm':s.get('alarm') or '',
            'gripper_state':'unknown','zed_mount':'eye-in-hand','zed_transform_validated':False,
            'source_session':s.get('session'),'sent_wall_unix_s':time.time(),
            'feedback_age_basis':s.get('feedback_age_basis','backend_reported_age'),
            'controller_sample_age_validated':bool(s.get('sample_age_validated',False)),
            'tracking':s.get('tracking')}
        try: self.latest.put_nowait(packet)
        except queue.Full:
            try: self.latest.get_nowait()
            except queue.Empty: pass
            self.dropped+=1
            try: self.latest.put_nowait(packet)
            except queue.Full: self.dropped+=1
    def _run(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
            while not self.stop.is_set():
                try: packet=self.latest.get(timeout=.1)
                except queue.Empty: continue
                try: raw=json.dumps(packet,ensure_ascii=False,allow_nan=False).encode('utf-8')
                except (TypeError,ValueError): self.dropped+=1;continue
                for port in self.ports:
                    try: sock.sendto(raw,('127.0.0.1',port))
                    except OSError: self.dropped+=1
    def close(self): self.stop.set(); self.thread.join(.3)
