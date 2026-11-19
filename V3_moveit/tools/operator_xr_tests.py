"""Model/state/relay regression for the read-only Quest workbench."""
import asyncio
import hashlib
import json
import math
from pathlib import Path
import socket
import tempfile
import unittest
from aiohttp import WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer
from operator_xr import server
from operator_xr.model import export_model
from operator_xr.state import snapshot
from operator_guidance import live_phase
from operator_target import TargetHistory


class DisplayState(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.cache = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def write(self, name, data):
        path = self.cache / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data))

    def test_no_feedback_is_explicit_preview(self):
        data = snapshot(self.cache, 10)
        self.assertFalse(data['arm_fresh']);self.assertEqual(data['joints'], {})
        self.assertIsNone(data['target']);self.assertEqual(data['mode'], 'preview')

    def test_joint_age_and_session_fence(self):
        self.write('telemetry.json', dict(position=[.1]*6, received_monotonic=10, mode='fake'))
        self.assertEqual(snapshot(self.cache, 10.1)['joints']['joint_1'], .1)
        self.assertFalse(snapshot(self.cache, 10.51)['arm_fresh'])
        self.assertFalse(snapshot(self.cache, 9.99)['arm_fresh'])
        self.write('scene_state.json', dict(target_session_started=11))
        self.assertFalse(snapshot(self.cache, 10.1)['arm_fresh'])

    def test_neutral_grip_is_ready_and_follow_requires_controller_confirmation(self):
        ctrl=dict(mode='IDLE',scene_ready=True,held=False)
        inputs=dict(held=False,reason='RELEASED',age_s=0.)
        def current(followed=False):
            return live_phase(ctrl,inputs,10.01,10.,10.,10.,followed)
        self.assertEqual(current(),'ready')
        inputs.update(held=True,reason='FOLLOWING')
        self.assertEqual(current(),'engaging')
        ctrl.update(mode='FOLLOW',held=True)
        self.assertEqual(current(),'following')
        inputs.update(held=False,reason='RELEASED')
        self.assertEqual(current(True),'braking')
        ctrl.update(mode='IDLE',held=False)
        self.assertEqual(current(True),'paused')
        inputs['age_s']=.08
        self.assertEqual(current(True),'tracking')
        inputs['software_stop']=True
        self.assertEqual(current(True),'software_stop')
        ctrl['mode']='FAULT'
        self.assertEqual(current(True),'fault')

    def test_headset_drops_stale_wrong_session_and_stopping_cues(self):
        session=dict(started=9.,updated=10.,stage='running',mode='fake',task='teleop',tool_id=1)
        signals=dict(session_started=9.,mode='fake',guide_phase='ready',guide_updated=10.)
        self.write('teleop_session.json',session);self.write('telemetry.json',signals)
        self.assertEqual(snapshot(self.cache,10.05)['guidance']['phase'],'ready')
        self.assertEqual(snapshot(self.cache,10.41)['guidance']['phase'],'waiting_feedback')
        self.assertEqual(snapshot(self.cache,11.6)['guidance']['phase'],'unavailable')
        for changed in (dict(mode='hardware'),dict(session_started=8.)):
            self.write('telemetry.json',dict(signals,**changed))
            self.assertEqual(snapshot(self.cache,10.05)['guidance']['phase'],'starting')
        self.write('telemetry.json',signals)
        for stage in ('stopping','ended','failed'):
            self.write('teleop_session.json',dict(session,stage=stage))
            self.assertEqual(snapshot(self.cache,10.05)['guidance']['phase'],stage)
        self.assertEqual(snapshot(self.cache,25.)['guidance']['phase'],'idle')

    def test_target_release_fade_and_new_session(self):
        history=TargetHistory(10)
        for t in (10.,10.03):
            history.inputs(dict(held=True,age_s=0),t)
            history.pose([.2,-.1,.8],[0,0,0,1],'base_link',t,0,t)
        self.write('target_state.json',history.snapshot(10.04))
        self.assertEqual(snapshot(self.cache,10.05)['target']['position_m'],[.2,-.1,.8])
        self.assertIsNone(snapshot(self.cache,10.3)['target'])
        history.inputs(dict(held=False,age_s=0),10.05)
        self.write('target_state.json',history.snapshot(10.06))
        self.assertIsNone(snapshot(self.cache,10.07)['target'])
        self.assertEqual(len(snapshot(self.cache,14.9)['trail']),2)
        self.assertEqual(snapshot(self.cache,15.1)['trail'],[])
        self.write('scene_state.json',dict(target_session_started=11))
        self.assertEqual(snapshot(self.cache,10.07)['trail'],[])

    def test_hand_priority_and_expiry(self):
        self.write('media/status.json',dict(updated=10,devices={'hand':dict(ok=True,updated=10,joint_names=['finger11'],position=[.1])}))
        self.write('dh116_status.json',dict(ok=True,updated=10,joint_names=['finger11'],position=[.2]))
        self.write('grasp_state.json',dict(ok=True,updated=10,joint_names=['finger11'],position=[.3]))
        self.assertEqual(snapshot(self.cache,10.01)['joints']['DH_finger11'],.3)
        self.assertEqual(snapshot(self.cache,10.1)['joints']['DH_finger11'],.1)
        self.assertFalse(snapshot(self.cache,11.6)['hand_fresh'])

    def test_malformed_files_do_not_keep_live_targets(self):
        self.write('target_state.json',dict(frame='base_link',source='/v3/hand_target',session_started='bad',updated=None,active=True))
        self.write('telemetry.json',dict(position=[math.nan]*6,received_monotonic=10))
        data=snapshot(self.cache,10.01)
        self.assertFalse(data['arm_fresh']);self.assertIsNone(data['target'])
        self.assertEqual(data['trail'],[])


class Model(unittest.TestCase):
    def test_exact_asset_allowlist_and_local_vendor_integrity(self):
        model,assets=export_model()
        self.assertEqual(len(model['links']),27);self.assertEqual(len(assets),46)
        self.assertNotIn('file://',json.dumps(model));self.assertNotIn(str(server.ROOT),json.dumps(model))
        for key,path in assets.items():self.assertTrue(key.startswith(hashlib.sha256(path.read_bytes()).hexdigest()))
        vendor=server.HERE/'vendor';manifest=json.loads((vendor/'manifest.json').read_text())
        for name,entry in manifest['files'].items():self.assertEqual(hashlib.sha256((vendor/name).read_bytes()).hexdigest(),entry['sha256'])

    def test_input_program_preserved_except_declared_render_hooks(self):
        canonical=(server.ROOT/'quest/index.html').read_text().split('<script type="module">')[1].split('</script>')[0]
        actual=server.page().split('<script type="module">')[1].split('</script>')[0]
        for before,after in reversed(server.DRAW_HOOKS):
            self.assertEqual(actual.count(after),1);actual=actual.replace(after,before,1)
        self.assertEqual(actual,canonical)


class Relay(unittest.IsolatedAsyncioTestCase):
    async def test_real_http_resources_and_exclusive_input_fanout(self):
        arm=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);arm.bind(('127.0.0.1',0));arm.setblocking(False)
        hand=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);hand.bind(('127.0.0.1',0));hand.setblocking(False)
        self.addCleanup(arm.close);self.addCleanup(hand.close)
        async with TestClient(TestServer(server.application(arm.getsockname()[1],hand.getsockname()[1]))) as client:
            page=await client.get('/');self.assertIn('三维工作台',await page.text())
            health=await client.get('/xr/health');self.assertFalse((await health.json())['input_active'])
            desc=await (await client.get('/xr/model')).json()
            mesh=next(v['geometry']['url'] for link in desc['links'] for v in link['visuals'] if v['kind']=='mesh')
            self.assertEqual((await client.get(mesh)).status,200)
            for path in ('/xr/mesh/unknown.stl','/xr/vendor/manifest.json','/xr/.cache/quest-tls/private.key'):
                self.assertEqual((await client.get(path)).status,404)
            ws=await client.ws_connect('/input')
            with self.assertRaises(WSServerHandshakeError):await client.ws_connect('/input')
            loop=asyncio.get_running_loop()
            for i in range(150):
                packet=json.dumps(dict(source='quest-webxr',sequence=i,right=dict(grip=0)))
                await ws.send_str(packet)
                for sock in (arm,hand):self.assertEqual((await asyncio.wait_for(loop.sock_recvfrom(sock,8192),1))[0].decode(),packet)
            self.assertTrue((await (await client.get('/xr/health')).json())['input_active'])
            await ws.close();await asyncio.sleep(.01)
            ws=await client.ws_connect('/input');await ws.close()


if __name__=='__main__':unittest.main()
