"""真实 WebSocket -> 本机 UDP 转发回读；不涉及 WebXR 硬件或机器人。"""
import asyncio
import importlib.util
import json
from pathlib import Path
import socket
from aiohttp import WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('quest_relay',ROOT/'quest/server.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


async def verify():
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind(('127.0.0.1',0));udp.setblocking(False)
    hand=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);hand.bind(('127.0.0.1',0));hand.setblocking(False)
    async with TestClient(TestServer(module.application(udp.getsockname()[1],hand.getsockname()[1]))) as client:
        page=await client.get('/');assert page.status==200 and '进入透视模式' in await page.text()
        ws=await client.ws_connect('/input')
        try:await client.ws_connect('/input');raise AssertionError('second input source accepted')
        except WSServerHandshakeError as exc:assert exc.status==409
        loop=asyncio.get_running_loop()
        for i in range(150):
            raw=json.dumps(dict(source='quest-webxr',sequence=i,unity_time_s=i*.01,right={'grip':0.}))
            await ws.send_str(raw);data,source=await asyncio.wait_for(loop.sock_recvfrom(udp,8192),1.)
            assert data.decode()==raw
            hand_data,_=await asyncio.wait_for(loop.sock_recvfrom(hand,8192),1.);assert hand_data==data
            if i==0:first_source=source
        await ws.close();await asyncio.sleep(.01)
        ws=await client.ws_connect('/input');await ws.send_json(dict(source='quest-webxr',sequence=151))
        _,source=await asyncio.wait_for(loop.sock_recvfrom(udp,8192),1.);assert source==first_source
        await ws.close()
    udp.close();hand.close()
    result=dict(passed=True,packets=151,exclusive_source=True,hand_fanout_verified=True,reconnect_udp_source_stable=True,
                tls_tested=False,webxr_headset_tested=False,robot_commands=0)
    (ROOT/'artifacts/web_relay_verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result))


if __name__=='__main__':asyncio.run(verify())
