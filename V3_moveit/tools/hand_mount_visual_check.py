"""Render the current operator canvas without starting devices or control."""
import hashlib,json,os,sys,time
from pathlib import Path
os.environ['QT_QPA_PLATFORM']='xcb'
os.environ['V3_SCENE_CAPTURE']='1'
from PyQt5 import QtWidgets as W,QtGui
from operator_panel import Panel,ROOT
from operator_model import generate

def main():
    generate()
    app=W.QApplication([]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10))
    panel=Panel(preview=True);panel.resize(1640,1040);panel.show();panel.scene.start()
    try:
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            app.processEvents();panel.studio_tick();time.sleep(.02)
        if panel.scene.image.isNull() or time.monotonic()-panel.scene.frame_at>1:
            raise RuntimeError('Updated model did not produce a fresh embedded scene')
        path=ROOT/'artifacts/hand_mount_operator_20261113.png'
        if not panel.grab().save(str(path)):raise RuntimeError('Failed to save operator view')
        panel.scene.image.save(str(ROOT/'artifacts/hand_mount_scene_20261113.png'))
        report=dict(passed=True,preview_only=True,hardware_started=False,sensors_started=False,
                    control_urdf_sha256=hashlib.sha256((ROOT/'src/v3_moveit/description/s5.urdf').read_bytes()).hexdigest(),
                    display_urdf_sha256=hashlib.sha256((ROOT/'.cache/operator/display.urdf').read_bytes()).hexdigest(),
                    frame=panel.scene.frame_seq,screenshot=str(path))
        (ROOT/'artifacts/hand_mount_visual_20261113.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)
    finally:
        panel.close();app.processEvents()

if __name__=='__main__':main()
