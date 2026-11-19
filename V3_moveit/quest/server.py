"""HTTPS/WebSocket 输入转发，只发送到本机 Quest 输入端口，不加载机器人 SDK。"""
import argparse
import json
from pathlib import Path
import socket
import ssl
from aiohttp import web


def application(port=5006, hand_port=5008):
    app=web.Application(client_max_size=8192)
    app['active']=None
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    async def cleanup(application):udp.close()
    app.on_cleanup.append(cleanup)
    async def index(request):return web.FileResponse(Path(__file__).with_name('index.html'))
    async def input_view(request):return web.FileResponse(Path(__file__).with_name('input-view.js'))
    async def receive(request):
        if app['active'] is not None:return web.Response(status=409,text='已有手柄连接')
        ws=web.WebSocketResponse(max_msg_size=8192,heartbeat=2)
        await ws.prepare(request);app['active']=ws
        try:
            async for message in ws:
                if message.type!=web.WSMsgType.TEXT:break
                packet=json.loads(message.data)
                if not isinstance(packet,dict) or packet.get('source')!='quest-webxr':break
                udp.sendto(message.data.encode('utf8'),('127.0.0.1',port))
                # Independent hand subscriber; arm sessions keep exclusive use of 5006.
                if hand_port is not None:
                    udp.sendto(message.data.encode('utf8'),('127.0.0.1',hand_port))
        except (ValueError,OSError):pass
        finally:
            app['active']=None;await ws.close()
        return ws
    app.router.add_get('/',index);app.router.add_get('/input-view.js',input_view);app.router.add_get('/input',receive)
    return app


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--cert',required=True);p.add_argument('--key',required=True)
    p.add_argument('--port',type=int,default=8443);p.add_argument('--udp-port',type=int,default=5006)
    a=p.parse_args();tls=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);tls.load_cert_chain(a.cert,a.key)
    web.run_app(application(a.udp_port),host='0.0.0.0',port=a.port,ssl_context=tls)
