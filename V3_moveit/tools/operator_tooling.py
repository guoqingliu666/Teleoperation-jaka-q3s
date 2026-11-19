"""Cabinet readback validation, recovery messages and atomic local configuration."""
import fcntl
import hashlib
import json
import math
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from operator_settings import ROOT, atomic
sys.path.insert(0, str(ROOT/'src/v3_moveit'))
from v3_moveit.tool_config import tool_values, apply_tool

PAYLOAD_HELP = '请在“读取工具/TCP”核对控制柜值与 V3 配置。控制柜已保存正确的标定时，点击“已校准，刷新负载”即可同步。'
TOOL_HELP = 'JAKA 当前工具坐标系或 TCP 定义与 V3 不一致。请在遥操作窗口读取当前工具，选择需要的工具（默认 TCP1），然后点击“应用工具”。'

def recovery(text):
    for line in reversed(text.splitlines()):
        if any(key in line for key in ('的有线网卡 ', '的有线网卡，请检查', '未就绪，无法确认到 ')):
            return 'network',line.removeprefix('RuntimeError: ').strip()
    if any(key in text for key in ('PAYLOAD_', 'payload mismatch', 'payload centroid mismatch', 'Historical payload', '实际负载', '实际重心')):
        reading=re.search(r'PAYLOAD_(?:CENTROID_)?MISMATCH: (控制柜[^。\n]+。)',text)
        return 'payload',(reading.group(1) if reading else '负载或质心与 V3 配置不一致。')+PAYLOAD_HELP
    if any(key in text for key in ('TOOL_MISMATCH', 'TCP_MISMATCH', 'TCP1 required', 'TCP1 mismatch', 'Historical tool', 'Historical TCP')):
        return 'tool', TOOL_HELP
    if 'site_ubuntu EDG client IP is not assigned' in text:
        import yaml
        site=yaml.safe_load((ROOT/'site_ubuntu.yaml').read_text())
        return 'network', f"本机未配置当前站点要求的 EDG 地址 {site['edg_client_ip']}。请检查机械臂网线及 Ubuntu 有线网卡，使 site_ubuntu.yaml 中的 edg_client_ip 与本机实际连接机械臂的网卡地址一致，然后重新检查。"
    return '', ''

def validate_snapshot(snapshot):
    tool_values(snapshot)
    mass=snapshot.get('payload_kg'); cog=snapshot.get('cog_mm')
    if type(mass) not in (int,float) or not math.isfinite(mass) or not 0<=mass<=5:
        raise ValueError('控制柜负载必须是 0–5 kg 的有限读数')
    if not isinstance(cog,list) or len(cog)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in cog):
        raise ValueError('控制柜质心读数无效')
    return snapshot

def mismatch(snapshot, config):
    validate_snapshot(snapshot); tool,tcp=tool_values(config)
    if snapshot['tool_id']!=tool or any(abs(a-b)>.001 for a,b in zip(snapshot['tcp_mm_rad'],tcp)):
        return 'tool', TOOL_HELP
    if abs(snapshot['payload_kg']-config['payload_kg'])>1e-6 or any(abs(a-b)>.001 for a,b in zip(snapshot['cog_mm'],config['cog_mm'])):
        return 'payload', PAYLOAD_HELP
    return '', ''

def save_readback(action, snapshot, capture, before_sha, root=ROOT):
    """Only a completed, fresh native action may call this. Never writes cabinet load."""
    if action not in ('select_tool','refresh_payload'):raise ValueError('不支持的配置刷新')
    validate_snapshot(snapshot)
    path=root/'src/v3_moveit/config/payload.json'
    with (root/'.cache/operator/control.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        before=path.read_bytes()
        if hashlib.sha256(before).hexdigest()!=before_sha:raise ValueError('配置在读取期间发生变化，请重新读取')
        config=json.loads(before); updated=dict(config)
        if action=='select_tool':
            updated.update(tool_id=snapshot['tool_id'],tcp_mm_rad=snapshot['tcp_mm_rad'])
            keys=('tool_id','tcp_mm_rad')
        else:
            if mismatch(dict(snapshot,payload_kg=config['payload_kg'],cog_mm=config['cog_mm']),config)[0]:
                raise ValueError('TOOL_MISMATCH: 请先应用工具坐标系，再刷新负载')
            updated.update(payload_kg=snapshot['payload_kg'],cog_mm=snapshot['cog_mm'])
            keys=('payload_kg','cog_mm')
        if all(updated.get(k)==config.get(k, tool_values(config)[1] if k=='tcp_mm_rad' else None) for k in keys):return False
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        out=root/'artifacts/tooling_updates'/stamp;out.mkdir(parents=True)
        (out/'before_payload.json').write_bytes(before)
        raw=Path(capture).read_bytes();(out/'capture.jsonl').write_bytes(raw)
        if action=='refresh_payload':
            for k in ('source','source_sha256','confirmed_by_user','controller_written','controller_write_scope','displayed_rounded_values'):
                updated.pop(k,None)
        updated[action+'_readback']=dict(utc=datetime.now(timezone.utc).isoformat(),
            source=str((out/'capture.jsonl').relative_to(root)),sha256=hashlib.sha256(raw).hexdigest(),controller_payload_written=False)
        atomic(path,updated);atomic(out/'after_payload.json',updated)
    return True
