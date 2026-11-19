"""Synthetic sensor transport check on isolated DDS domain 97; no SDK or hardware."""
import json,os,subprocess,time
from operator_settings import ROOT,RUNTIME,defaults,atomic
os.environ['ROS_DOMAIN_ID']='97'
import rclpy
from hand_control_interfaces.msg import HandJointState
from sensor_msgs.msg import Image
backup=RUNTIME.read_bytes() if RUNTIME.exists() else None
p=defaults()['profiles'][0];p['camera_source']='ros';p['sensor_domain']=97
atomic(RUNTIME,dict(scope_hz=20,profile=p))
log=(ROOT/'artifacts/operator_sensor_smoke.log').open('w');worker=None
rclpy.init();node=rclpy.create_node('v3_operator_sensor_test',start_parameter_services=False)
hand=node.create_publisher(HandJointState,p['hand_topic'],10)
images=[node.create_publisher(Image,p[key],10) for key in ('zed_left_topic','zed_depth_topic','c922_topic')]
def send(alarm=0,old=False):
    h=HandJointState();h.header.stamp=node.get_clock().now().to_msg();h.hand_name='test_only';h.active_dof=6;h.joint_ids=list(range(1,7));h.angles=[10.,20.,30.,40.,50.,60.];h.currents=[0.]*6;h.alarms=[alarm]*6
    if old:h.header.stamp.sec-=10
    hand.publish(h)
    for i,pub in enumerate(images):
        msg=Image();msg.header.stamp=node.get_clock().now().to_msg();msg.width=2;msg.height=1;msg.encoding='16UC1' if i==1 else 'rgb8';msg.step=4 if i==1 else 6;msg.data=bytes([232,3,0,0]) if i==1 else bytes([255,0,0,0,255,0]);pub.publish(msg)
    rclpy.spin_once(node,timeout_sec=.02)
def status():
    try:
        data=json.loads((ROOT/'.cache/operator/media/status.json').read_text());return data['devices'] if data['pid']==worker.pid else {}
    except (ValueError,OSError):return {}
def wait_for(predicate,send_data=None,seconds=8):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if send_data:send_data()
        if predicate(status()):return
        time.sleep(.05)
    raise AssertionError(status())
try:
    worker=subprocess.Popen(['bash','tools/operator_ros.sh','python3','-B','tools/operator_devices.py'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    wait_for(lambda d:all(d.get(k,{}).get('ok') for k in ('hand','zed_left','zed_depth','c922')),send)
    d=status()['hand'];q=dict(zip(d['joint_names'],d['position']));assert abs(q['finger11']-0.174532925)<1e-6;assert abs(q['finger13']-(0.34906585*1.2334+.004592))<1e-6
    wait_for(lambda d:d.get('hand',{}).get('detail')=='DH116 报警',lambda:send(alarm=3))
    wait_for(lambda d:d.get('hand',{}).get('detail')=='DH116 反馈时间戳失效',lambda:send(old=True))
    wait_for(lambda d:all(d.get(k,{}).get('ok') for k in ('hand','zed_left','zed_depth','c922')),send)
    wait_for(lambda d:len(d)==4 and all(not d[k]['ok'] for k in d),seconds=4)
    result=dict(passed=True,domain=97,synthetic_data=True,hardware_access=False,checks=['ROS RGB/depth conversion','DH116 six axes and URDF mimic','hand alarm','old hand timestamp','stale streams'])
    atomic(ROOT/'artifacts/operator_sensor_smoke.json',result);print(json.dumps(result))
finally:
    if worker is not None:worker.terminate();worker.wait(timeout=5)
    log.close();node.destroy_node();rclpy.shutdown()
    if backup is not None:RUNTIME.write_bytes(backup)
    else:RUNTIME.unlink(missing_ok=True)
