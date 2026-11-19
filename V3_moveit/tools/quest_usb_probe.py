"""HTTPS readback through the headset's localhost; never starts WebXR input."""
import json,select,ssl,subprocess,time
from operator_quest import authorized_device,run

def probe(path='/',marker=None):
    if not path.startswith('/') or any(c in path for c in '\r\n '):raise ValueError('Invalid probe path')
    marker=marker or '<h1>三维工作台</h1>'.encode()
    serial=authorized_device(run(['adb','devices']))
    p=subprocess.Popen(['adb','-s',serial,'shell','-T','nc','-w','6','localhost','8443'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    incoming,outgoing=ssl.MemoryBIO(),ssl.MemoryBIO();tls=ssl._create_unverified_context().wrap_bio(incoming,outgoing,server_hostname='localhost');deadline=time.monotonic()+10
    def flush():
        while outgoing.pending:p.stdin.write(outgoing.read());p.stdin.flush()
    def receive():
        flush()
        if time.monotonic()>=deadline or not select.select([p.stdout],[],[],max(0,deadline-time.monotonic()))[0]:raise TimeoutError('Quest localhost HTTPS read timed out')
        data=p.stdout.read1(16384)
        if not data:raise RuntimeError('Quest localhost connection closed before HTTP response')
        incoming.write(data)
    try:
        while True:
            try:tls.do_handshake();flush();break
            except ssl.SSLWantReadError:receive()
        tls.write(('GET '+path+' HTTP/1.1\r\nHost: localhost:8443\r\nConnection: close\r\n\r\n').encode());flush();body=b''
        while marker not in body:
            try:
                chunk=tls.read(16384)
                if not chunk:raise RuntimeError('No V3 page returned')
                body+=chunk
            except ssl.SSLWantReadError:receive()
        assert body.startswith(b'HTTP/1.1 200'),body[:100]
        return dict(passed=True,serial=serial,url='https://localhost:8443'+path,path='Host TLS client -> adb shell nc on Quest -> Android localhost reverse -> host V3 HTTPS',http_status=200,page_identity_verified=True,webxr_started=False,robot_commands=0)
    finally:
        p.terminate()
        try:p.wait(timeout=3)
        except subprocess.TimeoutExpired:p.kill();p.wait()

if __name__=='__main__':print(json.dumps(probe(),ensure_ascii=False))
