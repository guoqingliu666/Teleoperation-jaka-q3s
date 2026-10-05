"""Open the actual desktop Player; record camera/HUD checks and screenshots, no SDK."""
import json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from process_util import close_own_player

def main():
    active=subprocess.run(['tasklist','/FI','IMAGENAME eq V2Twin.exe','/FO','CSV','/NH'],capture_output=True)
    if active.returncode or b'V2Twin.exe' in active.stdout:raise RuntimeError('Existing Player: no takeover')
    out=ROOT/'data/tcp_view_fix_20261004/player';out.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env.update(TEMP='D:/ChatGPT/Temp',TMP='D:/ChatGPT/Temp',V2_VIEW_PROBE=str(out),V2_TWIN_DATA_DIR=str(out))
    p=subprocess.Popen([str(ROOT/'digital_twin/unity/Build/V2Twin.exe'),'--quest-j1-input','-screen-fullscreen','0',
        '-screen-width','1280','-screen-height','800','-logFile',str(out/'player.log')],env=env,cwd=ROOT)
    try:code=p.wait(timeout=40)
    except subprocess.TimeoutExpired:
        close_own_player(p);raise
    r=json.loads((out/'view_report.json').read_text())
    checks={k:r[k] for k in ('default_details_hidden','panel_fixed_in_world','desktop_mouse_functions','details_can_toggle','free_mode_preserves_xr_anchor')}
    checks.update(level_horizon=r['default_roll_sine']<1e-6,aligned_table=r['default_table_yaw_sine']<1e-6,
        gravity_preserved=r['anchor_up_error']<1e-6,screenshot=(out/'default_unobstructed.png').is_file(),exit_ok=code==0)
    r.update(checks=checks,passed=all(checks.values()))
    (out/'report.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r))
    return 0 if r['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
