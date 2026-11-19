"""Compose the original Quest relay with read-only workbench rendering/resources.

The accepted input implementation stays in quest/index.html and quest/server.py.
Only the display adapter import is substituted, with an exact-count check.
Packet generation, mapping, throttle, visibility, stop and relay code are
served from that source verbatim. An upstream change fails closed at startup.
"""
import argparse
import importlib.util
import ssl
import sys
from pathlib import Path
from aiohttp import web

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from operator_xr.model import export_model
from operator_xr.state import snapshot

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('v3_original_quest_relay', ROOT / 'quest/server.py')
relay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay)

# Deliberately narrow: no transformation may change a control/input expression.
DRAW_HOOKS = (
    ("import {display} from '/input-view.js';",
     "import {workbench as display} from '/xr/workbench.js';"),
)


def page():
    source = (ROOT / 'quest/index.html').read_text()
    script = source.split('<script type="module">', 1)[1].split('</script>', 1)[0]
    for before, after in DRAW_HOOKS:
        if script.count(before) != 1:
            raise ValueError('Quest rendering hook no longer matches canonical input: ' + before)
        script = script.replace(before, after, 1)
    return (HERE / 'index.html').read_text().replace('/* CANONICAL_QUEST_INPUT */', script)


def application(port=5006, hand_port=5008, root=ROOT):
    html = page()
    model, assets = export_model(root)
    model_stamp=(root/'.cache/operator/display.urdf').stat().st_mtime_ns
    app = relay.application(port, hand_port)

    @web.middleware
    async def workbench_index(request, handler):
        if request.path == '/':
            return web.Response(text=html, content_type='text/html', headers={'Cache-Control': 'no-store'})
        response = await handler(request)
        if request.path.startswith('/xr/'):
            response.headers['Cache-Control'] = 'no-cache'
        return response

    app.middlewares.append(workbench_index)

    async def health(request):
        return web.json_response(dict(service='v3-quest-workbench', version=1, visual_guidance=True,
                                      input_active=app['active'] is not None))

    async def description(request):
        nonlocal model,assets,model_stamp
        stamp=(root/'.cache/operator/display.urdf').stat().st_mtime_ns
        if stamp!=model_stamp:
            model,assets=export_model(root);model_stamp=stamp
        return web.json_response(model)

    async def state(request):
        return web.json_response(snapshot(root / '.cache/operator'))

    async def mesh(request):
        path = assets.get(request.match_info['key'])
        if path is None:
            raise web.HTTPNotFound()
        return web.FileResponse(path)

    # Explicit assets only; never expose project directories, snapshots or TLS keys.
    static = {name: HERE / name for name in ('workbench.js', 'robot.js', 'view.js', 'guidance.js', 'style.css')}
    static.update({'vendor/' + p.name: p for p in (HERE / 'vendor').iterdir() if p.suffix == '.js' or p.name == 'LICENSE'})

    async def resource(request):
        path = static.get(request.match_info['name'])
        if path is None:
            raise web.HTTPNotFound()
        return web.FileResponse(path)

    app.router.add_get('/xr/health', health)
    app.router.add_get('/xr/model', description)
    app.router.add_get('/xr/state', state)
    app.router.add_get('/xr/mesh/{key}', mesh)
    app.router.add_get('/xr/{name:.*}', resource)
    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cert', required=True)
    parser.add_argument('--key', required=True)
    parser.add_argument('--port', type=int, default=8443)
    parser.add_argument('--udp-port', type=int, default=5006)
    args = parser.parse_args()
    tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    tls.load_cert_chain(args.cert, args.key)
    web.run_app(application(args.udp_port), host='0.0.0.0', port=args.port, ssl_context=tls)
