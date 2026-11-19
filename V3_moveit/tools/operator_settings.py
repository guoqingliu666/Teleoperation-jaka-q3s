"""Display/device preferences; never modify control limits or hardware approval."""
import copy,json,os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
STORE=ROOT/'.config/operator_profiles.json'
RUNTIME=ROOT/'.cache/operator/display_settings.json'
RATES=(1,5,10,20,30,60)
DEVICES=('arm','quest','hand','zed','c922')
CAMERA_PRESETS={
    'zed':(
        dict(id='hd1080_30',resolution='HD1080',width=1920,height=1080,fps=30,label='采集推荐 · 1920 × 1080 · 30 fps'),
        dict(id='hd720_30',resolution='HD720',width=1280,height=720,fps=30,label='均衡 · 1280 × 720 · 30 fps'),
        dict(id='hd720_60',resolution='HD720',width=1280,height=720,fps=60,label='高速 · 1280 × 720 · 60 fps'),
        dict(id='hd2k_15',resolution='HD2K',width=2208,height=1242,fps=15,label='最高分辨率 · 2208 × 1242 · 15 fps'),
        dict(id='vga_100',resolution='VGA',width=672,height=376,fps=100,label='最高帧率 · 672 × 376 · 100 fps'),
    ),
    'c922':(
        dict(id='720p30',width=1280,height=720,fps=30,label='推荐 · 1280 × 720 · 30 fps'),
        dict(id='720p60',width=1280,height=720,fps=60,label='高速 · 1280 × 720 · 60 fps'),
        dict(id='1080p30',width=1920,height=1080,fps=30,label='最高分辨率 · 1920 × 1080 · 30 fps'),
        dict(id='480p30',width=640,height=480,fps=30,label='低带宽 · 640 × 480 · 30 fps'),
    ),
}
CAMERA_DEFAULTS={'zed':'hd1080_30','c922':'720p30'}
def camera_preset(camera,preset_id=None):
    preset_id=preset_id or CAMERA_DEFAULTS[camera]
    return next((p for p in CAMERA_PRESETS[camera] if p['id']==preset_id),next(p for p in CAMERA_PRESETS[camera] if p['id']==CAMERA_DEFAULTS[camera]))
DEFAULT_PROFILE=dict(enabled={name:True for name in DEVICES},sensor_domain=0,hand_topic='/hand_control/realtime_state',zed_left_topic='/zed/zed_node/left/color/rect/image',zed_depth_topic='/zed/zed_node/depth/depth_registered',c922_topic='/c922/image_raw',camera_source='local')
def atomic(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding="utf-8");os.replace(tmp,path)
def defaults():return dict(version=1,active=0,brightness=100,scope_hz=20,scope_signals=list(range(6)),scope_quantity='position',camera_modes=copy.deepcopy(CAMERA_DEFAULTS),profiles=[dict(name='方案'+name,**copy.deepcopy(DEFAULT_PROFILE)) for name in ('一','二','三')])
def validate(data):
    if data.get('version')!=1 or type(data.get('active')) is not int or not 0<=data['active']<3:raise ValueError('Invalid profile selection')
    if data.get('scope_hz') not in RATES:raise ValueError('Display rate must be 1..60 Hz')
    if type(data.get('brightness')) is not int or not 0<=data['brightness']<=100:raise ValueError('Invalid brightness')
    signals=data.get('scope_signals')
    if not isinstance(signals,list) or any(type(i) is not int or i not in range(6) for i in signals):raise ValueError('Invalid signal selection')
    if data.get('scope_quantity') not in ('position','velocity'):raise ValueError('Invalid signal quantity')
    modes=data.get('camera_modes')
    if not isinstance(modes,dict):modes={}
    data['camera_modes']={camera:(modes.get(camera) if isinstance(modes.get(camera),str) and modes.get(camera) in {p['id'] for p in CAMERA_PRESETS[camera]} else CAMERA_DEFAULTS[camera]) for camera in CAMERA_PRESETS}
    profiles=data.get('profiles')
    if not isinstance(profiles,list) or len(profiles)!=3:raise ValueError('Three profiles required')
    for p in profiles:
        if not isinstance(p.get('name'),str) or not 1<=len(p['name'])<=24:raise ValueError('Invalid profile name')
        if set(p.get('enabled',{}))!=set(DEVICES) or any(type(v) is not bool for v in p['enabled'].values()):raise ValueError('Invalid devices')
        if type(p.get('sensor_domain')) is not int or not 0<=p['sensor_domain']<=101 or p['sensor_domain'] in (89,90,91):raise ValueError('Sensor domain must be separate from control/display domains 89/90/91')
        for key in ('hand_topic','zed_left_topic','zed_depth_topic','c922_topic'):
            if not isinstance(p.get(key),str) or not p[key].startswith('/') or len(p[key])>180 or any(c.isspace() for c in p[key]):raise ValueError('Invalid topic')
        if p.get('camera_source') not in ('local','ros'):raise ValueError('Invalid camera source')
    return data
class Preferences:
    def __init__(self,path=STORE):
        self.path=path;self.error=None
        try:self.data=validate(json.loads(path.read_text()))
        except FileNotFoundError:self.data=defaults()
        except (ValueError,TypeError,KeyError) as e:self.error=str(e);self.data=defaults()
    @property
    def profile(self):return self.data['profiles'][self.data['active']]
    def save(self):validate(self.data);atomic(self.path,self.data);self.write_runtime()
    def write_runtime(self):atomic(RUNTIME,dict(scope_hz=self.data['scope_hz'],profile=self.profile,camera_modes=self.data['camera_modes']))
