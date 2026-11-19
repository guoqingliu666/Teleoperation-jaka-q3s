"""Isolated real HTTP/UDP/browser exercise. Never uses production input ports."""
import asyncio
import argparse
import json
from pathlib import Path
import socket
import shutil
import ssl
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from aiohttp.test_utils import TestServer

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from operator_xr.server import application
from verify_offline import fk


async def main(output):
    output.mkdir(parents=True,exist_ok=True)
    sockets=[]
    for _ in range(2):
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0));sock.setblocking(False);sockets.append(sock)
    packets=[[],[]]
    async def receive(i):
        while True:
            data,_=await asyncio.get_running_loop().sock_recvfrom(sockets[i],8192)
            packets[i].append(json.loads(data))
    listeners=[asyncio.create_task(receive(i)) for i in range(2)]
    q=json.loads((ROOT/'src/v3_moveit/config/model_metadata.json').read_text())['q_reference_rad']
    model=ET.parse(ROOT/'src/v3_moveit/description/s5.urdf').getroot()
    from operator_tooling import apply_tool
    model,_=apply_tool(model,json.loads((ROOT/'src/v3_moveit/config/model_metadata.json').read_text()),json.loads((ROOT/'src/v3_moveit/config/payload.json').read_text()))
    cases=[dict(q=q,expected=fk(model,q).tolist()),dict(q=(np.array(q)+[.1,-.05,.1,0,.08,-.1]).tolist())]
    cases[1]['expected']=fk(model,cases[1]['q']).tolist()
    fixture=output/'browser_fk.json';fixture.write_text(json.dumps(cases))
    tls=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    with tempfile.TemporaryDirectory() as directory:
        cert=Path(directory)/'localhost.crt';key=Path(directory)/'localhost.key'
        subprocess.run(['openssl','req','-x509','-newkey','rsa:2048','-nodes','-days','1',
                        '-subj','/CN=localhost','-keyout',str(key),'-out',str(cert)],check=True,
                       stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        tls.load_cert_chain(cert,key)
    try:
        server=TestServer(application(sockets[0].getsockname()[1],sockets[1].getsockname()[1]))
        await server.start_server(ssl=tls)
        async with server:
            node=shutil.which('node')
            if not node:raise RuntimeError('The optional browser check requires Node.js and Playwright.')
            proc=await asyncio.create_subprocess_exec(str(node),str(ROOT/'tools/operator_xr/browser_check.cjs'),str(server.make_url('/')),str(output),str(fixture),cwd=ROOT)
            code=await proc.wait()
            if code:raise RuntimeError('Browser verification failed: '+str(code))
        assert len(packets[0])>=3,packets
        assert packets[0]==packets[1],'Arm/hand fanout differ'
        assert packets[0][0]['right']['grip']==0 and any(p['right']['grip']==1 for p in packets[0])
        assert any(p['right']['thumbstick_click'] for p in packets[0])
        assert any(not p['right']['pose_valid'] for p in packets[0])
        assert all(p['right']['position_m']==dict(x=.2,y=1.1,z=.3) for p in packets[0] if p['right']['pose_valid'])
        result=dict(passed=True,packets=len(packets[0]),hand_fanout=True,synthetic_webxr=True,
                    physical_headset=False,robot_connections=0,production_ports_used=False)
        (output/'browser_relay.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    finally:
        for task in listeners:task.cancel()
        await asyncio.gather(*listeners,return_exceptions=True)
        for sock in sockets:sock.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'artifacts/operator_xr_20261116')
    asyncio.run(main(parser.parse_args().output))
