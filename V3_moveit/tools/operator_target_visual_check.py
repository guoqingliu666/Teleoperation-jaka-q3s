"""Isolated ROS 96 synthetic input -> real subscriber -> RViz/Qt display.

No SDK, hardware launch, or publications on the control domain. Requires the
normal operator window to be closed because the display renderer uses ROS 91.
"""
import fcntl,json,math,os,signal,subprocess,sys,tempfile,time
from unittest.mock import patch
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
os.environ['ROS_DOMAIN_ID']='96'
os.environ['QT_QPA_PLATFORM']='xcb'
from PyQt5 import QtCore,QtGui,QtWidgets as W
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from operator_settings import atomic,Preferences
import operator_instruments,operator_studio
from operator_panel import Panel

out=ROOT/'artifacts/operator_target_axes_20261115'
lock=(ROOT/'.cache/operator/panel.lock').open('r')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
temp=tempfile.TemporaryDirectory(prefix='target_display_',dir=ROOT/'.tmp');fixture=Path(temp.name)
for name in ('tools','src','install'):(fixture/name).symlink_to(ROOT/name,target_is_directory=True)
cache=fixture/'.cache/operator';cache.mkdir(parents=True);(fixture/'artifacts').mkdir()
(cache/'scene_build').symlink_to(ROOT/'.cache/operator/scene_build',target_is_directory=True)
(cache/'display.urdf').write_bytes((ROOT/'.cache/operator/display.urdf').read_bytes())
operator_instruments.ROOT=fixture;operator_studio.ROOT=fixture
W.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
app=W.QApplication([]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10))
prefs=Preferences();prefs.data['brightness']=0
with patch('operator_studio.Preferences',return_value=prefs):panel=Panel(preview=True)
panel.resize(1640,1080);panel.show()
panel.scene_status.setText('显示验证 · 合成目标');panel.message('显示验证：ROS 96 合成目标，无机器人连接或运动命令。')
panel.scene.start()
rclpy.init();node=rclpy.create_node('target_display_fixture',start_parameter_services=False,enable_rosout=False)
inputs=node.create_publisher(String,'/v3/input_state',1);poses=node.create_publisher(PoseStamped,'/v3/hand_target',1)
log=(out/'subscriber.log').open('w')
child=subprocess.Popen([sys.executable,'-B',str(ROOT/'tools/operator_audio.py'),'--continuous','--mode','fake'],cwd=fixture,stdout=log,stderr=subprocess.STDOUT)
started=time.monotonic();latest={};last_position=None
def pump(seconds,held=False):
    global latest,last_position
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        now=time.monotonic();t=now-started
        inputs.publish(String(data=json.dumps(dict(held=held,reason='FOLLOWING' if held else 'RELEASED',age_s=0,software_stop=False))))
        if held:
            m=PoseStamped();m.header.frame_id='base_link';m.header.stamp=node.get_clock().now().to_msg()
            last_position=[.55+.1*math.cos(t),-.05+.1*math.sin(t),.48+.035*math.sin(t*2)]
            m.pose.position.x,m.pose.position.y,m.pose.position.z=last_position
            m.pose.orientation.z=math.sin(t*.2);m.pose.orientation.w=math.cos(t*.2);poses.publish(m)
        rclpy.spin_once(node,timeout_sec=0);app.processEvents();panel.scene.poll()
        try:latest=json.loads((cache/'target_state.json').read_text());panel.show_target(latest,now)
        except (OSError,ValueError):pass
        assert child.poll() is None,'subscriber exited'
        time.sleep(.025)
try:
    pump(3.);pump(6.,True)
    assert latest['active'] and len(latest['trail'])>100,latest
    assert latest['frame']=='base_link' and latest['source']=='/v3/hand_target'
    assert not panel.scene.image.isNull() and time.monotonic()-panel.scene.frame_at<1
    assert 'X —' not in panel.target_position.text()
    live=dict(target=latest['target'],trail_points=len(latest['trail']),readout=panel.target_position.text(),orientation=panel.target_rotation.text())
    panel.grab().save(str(out/'target_live.png'))
    pump(.25);assert not latest['active'];assert latest['trail'];assert 'X —' in panel.target_position.text()
    panel.grab().save(str(out/'target_released.png'))
    pump(5.1);assert latest['trail']==[]
    panel.grab().save(str(out/'target_expired.png'))
    atomic(out/'visual_check.json',dict(passed=True,ros_domain=96,synthetic=True,robot_connections=0,hardware_commands=0,
        live=live,release_hides_axes=True,expired_after_five_seconds=True,actual_qt_rviz_render=True))
    print(json.dumps(live,ensure_ascii=False))
finally:
    if child.poll() is None:child.send_signal(signal.SIGINT);child.wait(timeout=5)
    log.close();panel.close();app.processEvents();node.destroy_node();rclpy.shutdown();temp.cleanup()
