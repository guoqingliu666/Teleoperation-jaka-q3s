"""GUI 只能写最新输入槽和有界操作队列；停止走独立 Event。"""
import multiprocessing as mp
import queue,time
import numpy as np

class Channels:
    def __init__(self,ctx=None):
        ctx=ctx or mp.get_context('spawn')
        self.lock=ctx.Lock(); self.latest=ctx.RawArray('d',16)
        self.operations=ctx.Queue(24); self.status=ctx.Queue(8)
        self.stop=ctx.Event(); self.estop=ctx.Event(); self.shutdown=ctx.Event()
    def publish(self,seq,axes,space,deadman,speed,acceleration,hz,stamp=None,source='keyboard'):
        values=[seq,time.perf_counter() if stamp is None else stamp,int(space=='cartesian'),int(deadman),*axes,speed,acceleration,hz,1,int(source=='vr'),0]
        if not np.isfinite(values).all(): raise ValueError('nonfinite input')
        if not self.lock.acquire(timeout=.002): return False
        try: self.latest[:]=values
        finally: self.lock.release()
        return True
    def read(self):
        if not self.lock.acquire(False): return None
        try: v=list(self.latest)
        finally: self.lock.release()
        return {'sequence':int(v[0]),'stamp':v[1],'space':'cartesian' if v[2] else 'joint','deadman':bool(v[3]),
                'axes':v[4:10],'speed':v[10],'acceleration':v[11],'hz':v[12],'source':'vr' if v[14] else 'keyboard'}
    def operation(self,name,**values):
        try: self.operations.put_nowait({'name':name,'stamp':time.perf_counter(),**values})
        except queue.Full: self.stop.set(); raise RuntimeError('operation queue full; stop requested')
    def emit(self,value):
        try: self.status.put_nowait(value)
        except queue.Full: pass # 遥测显示可丢旧；审计日志在独立可靠有界队列。
    def drain_status(self):
        result=None
        while True:
            try: result=self.status.get_nowait()
            except queue.Empty: return result
