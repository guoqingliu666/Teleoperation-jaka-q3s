"""可选控制进程旁路；有界队列异步发本机 UDP，绝不写图像或等待采集器。"""
import json,queue,socket,threading,time

class CommandTap:
    def __init__(self,port=5010):
        self.port=port;self.q=queue.Queue(256);self.dropped=0;self.sent=0;self.stop=threading.Event()
        self.thread=threading.Thread(target=self._run,name='ACT-command-tap',daemon=True);self.thread.start()
    def offer(self,sequence,mode,motion,feedback,issued_ns,returned_ns):
        p={'schema':'v2_act_command_v1','sequence':sequence,'mode':mode,'issued_ns':issued_ns,
           'returned_ns':returned_ns,'q_target_rad':motion.q.tolist(),'q_rad':feedback.q.tolist(),
           'feedback_sequence':feedback.sequence,'feedback_host_ns':round(feedback.sampled_at*1e9),
           'controller_execution_ns':None,'controller_sample_timestamp':feedback.controller_timestamp,
           'dropped':self.dropped}
        try:self.q.put_nowait(p)
        except queue.Full:self.dropped+=1
    def _run(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
            sock.settimeout(.01)
            while not self.stop.is_set():
                try:p=self.q.get(timeout=.05)
                except queue.Empty:continue
                try:sock.sendto(json.dumps(p,allow_nan=False).encode(),('127.0.0.1',self.port));self.sent+=1
                except (ValueError,OSError):self.dropped+=1
    def close(self):self.stop.set();self.thread.join(.15)
