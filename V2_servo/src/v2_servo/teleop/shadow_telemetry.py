"""影子模式低频文字/文件输出；与125Hz状态发送、控制循环分离。"""
import json,queue,threading
from .commissioning_guidance import GuidanceWriter

class ShadowTelemetry:
    def __init__(self,folder):
        self.folder=folder;self.writer=GuidanceWriter(folder/'quest_live_guidance.json')
        self.latest=queue.Queue(maxsize=1);self.stop=threading.Event();self.error=None
        self.offered=0;self.written=0;self.dropped=0
        self.thread=threading.Thread(target=self._run,name='shadow-status-file-writer',daemon=True);self.thread.start()
    def offer(self,status,detail):
        self.offered+=1
        try:self.latest.put_nowait((status,detail))
        except queue.Full:
            try:self.latest.get_nowait();self.dropped+=1
            except queue.Empty:pass
            try:self.latest.put_nowait((status,detail))
            except queue.Full:self.dropped+=1
    def _run(self):
        while not self.stop.is_set() or not self.latest.empty():
            try:status,detail=self.latest.get(timeout=.1)
            except queue.Empty:continue
            try:
                self.writer.update('POSE_SHADOW','pose6dof_shadow',detail=detail,session=str(self.folder),control_scope='CONTINUOUS_SHADOW')
                # 原子替换防止GUI读到半个JSON；文件I/O不进入规划周期。
                temp=self.folder/'status.next.json';temp.write_text(json.dumps(status,ensure_ascii=False),encoding='utf-8')
                temp.replace(self.folder/'status.json');self.written+=1
            except Exception as exc:self.error=str(exc)
    def close(self):
        self.stop.set();self.thread.join(2)
        if self.thread.is_alive():self.error='shadow telemetry close timeout'
