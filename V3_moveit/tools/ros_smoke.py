"""真实启动 ROS+MoveIt+mock hardware 的集成测试，自动清理仿真进程。"""
import yaml
import json
from collections import Counter
import math
import re
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from rclpy.parameter_client import AsyncParameterClient
from verify_offline import source_digest

ROOT=Path(__file__).resolve().parents[1]


class Probe(Node):
    def __init__(self, *, mode):
        if mode!='fake':raise ValueError('Smoke handoff capture is fake-only')
        super().__init__('v3_smoke_probe');self.state={};self.samples=[];self.q=None;self.codes=[];self.max_velocity=0.
        self.mode_counts=Counter();self.mode_history=[]
        self.handoff_frames=[];self.handoff_overflow=0;self.handoff_limit=20000
        # Keep raw messages in memory. No serialization or filesystem I/O runs
        # in these callbacks, and diagnostic observations never modify commands.
        for topic in ('/v3/servo_trajectory','/arm_controller/joint_trajectory'):
            self.create_subscription(JointTrajectory,topic,
                lambda msg,topic=topic:self.capture_handoff(topic,msg),1000)
        self.create_subscription(JointTrajectoryControllerState,'/arm_controller/controller_state',
            lambda msg:self.capture_handoff('/arm_controller/controller_state',msg),1000)
        self.create_subscription(String,'/v3/state',self.status,1)
        self.create_subscription(JointState,'/joint_states',self.joints,1)
        self.scene=self.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
    def capture_handoff(self,topic,msg):
        if len(self.handoff_frames)>=self.handoff_limit:
            self.handoff_overflow+=1;return
        self.handoff_frames.append((topic,time.monotonic_ns(),self.get_clock().now().nanoseconds,
                                    self.state.get('mode'),msg))
    def status(self,m):
        self.state=json.loads(m.data);self.codes.append(self.state['servo_code'])
        mode=self.state['mode'];self.mode_counts[mode]+=1
        if not self.mode_history or self.mode_history[-1]['mode']!=mode:
            self.mode_history.append(dict(mode=mode,host_monotonic_s=time.monotonic()))
    def joints(self,m):
        try:
            self.q=np.array([m.position[m.name.index(f'joint_{i}')] for i in range(1,7)]);self.samples.append((time.monotonic(),self.q.copy()))
            self.max_velocity=max(self.max_velocity,max(abs(m.velocity[m.name.index(f'joint_{i}')]) for i in range(1,7)))
        except (ValueError,IndexError):pass


def guard_audit_summary(path):
    """Check final-write evidence only after the fake process has closed its log."""
    rows=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    starts=[i for i,row in enumerate(rows) if row.get('event')=='session_start']
    if not starts:raise RuntimeError('Guard audit has no session_start')
    current=rows[starts[-1]:]
    if current[-1].get('event')!='session_end':
        raise RuntimeError('Guard audit is incomplete: final session_end missing')
    start,end=current[0],current[-1]
    writes=[row for row in current if row.get('event')=='write']
    # A previous configure cycle in this same run cannot hide a rejection.
    rejected=[row for row in rows if row.get('event')=='write' and
              (row.get('result')=='rejected' or row.get('first_rejection'))]
    if rejected:
        first=rejected[0]
        raise RuntimeError(f"Final-write Guard rejected index {first.get('index')}: {first.get('reason')}")
    if not start.get('strict') or any(row.get('strict') is not True for row in writes):
        raise RuntimeError('Guard audit did not run in strict mode')
    if start.get('sdk_calls')!=0:raise RuntimeError('Guard audit does not identify the SDK-free backend')
    accepted=[row for row in writes if row.get('result')=='accepted']
    startup=[row for row in writes if row.get('result')=='startup_no_command']
    if not accepted:raise RuntimeError('Guard audit has no accepted real command writes')
    if len(writes)!=len(accepted)+len(startup):raise RuntimeError('Guard audit has an unknown write result')
    if [row.get('index') for row in writes]!=list(range(len(writes))):
        raise RuntimeError('Guard audit write sequence is incomplete')
    if (end.get('writes')!=len(writes) or end.get('rejections')!=0 or
            end.get('startup_writes')!=len(startup) or end.get('first_rejection')!=''):
        raise RuntimeError('Guard audit session_end counters do not match its writes')
    if any(len(row.get(key,[]))!=6 or any(x is None or not math.isfinite(x) for x in row[key])
           for row in accepted for key in ('command','state')):
        raise RuntimeError('Guard audit accepted a nonfinite or malformed sample')
    periods=[row['period_s'] for row in accepted]
    if any(not math.isfinite(period) or not .004<=period<=.012 for period in periods):
        raise RuntimeError('Guard audit accepted a period outside the unchanged 4..12 ms guard')
    return dict(passed=True,path=str(path),strict=True,sessions=len(starts),
                writes=len(writes),accepted_writes=len(accepted),startup_writes=len(startup),
                rejections=0,session_end_seen=True,minimum_period_s=min(periods),
                maximum_period_s=max(periods),physical_acceptance=False)


def run(smooth_follow=False):
    artifacts=ROOT/'artifacts'/('smooth_follow_smoke' if smooth_follow else '.');artifacts.mkdir(parents=True,exist_ok=True)
    (artifacts/'ros_smoke_pass.json').write_text(json.dumps(dict(passed=False,reason='verification started; no current pass')))
    capture_path=artifacts/'servo_handoff_capture.jsonl'
    capture_path.unlink(missing_ok=True)
    audit=artifacts/'guarded_smoke_output.jsonl'
    audit.unlink(missing_ok=True)  # This test's fake-only output; never carry a prior pass forward.
    log=(artifacts/'ros_smoke.log').open('w',encoding='utf8')
    # Keep synthetic verification separate from an already connected Quest relay.
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
        reserve.bind(('127.0.0.1',0));input_port=reserve.getsockname()[1]
    runtime=ROOT/'.tmp/ros_smoke_runtime.yaml'
    runtime.write_text(f'quest_bind: 127.0.0.1\nquest_port: {input_port}\n')
    configured_servo=yaml.safe_load((ROOT/'src/v3_moveit/config/servo.yaml').read_text())
    configured_runtime=yaml.safe_load((ROOT/'src/v3_moveit/config/runtime.yaml').read_text())
    threshold=configured_servo['self_collision_proximity_threshold']
    process=subprocess.Popen(['ros2','launch','v3_moveit','bringup.launch.py','mode:=fake','rviz:=false',f'runtime_config:={runtime}','guarded_fake:=true',f'guard_audit_path:={audit}','guard_strict:=true',
        f'fake_velocity_tracking:={str(smooth_follow).lower()}',
        f'fake_self_collision_proximity_threshold:={threshold}'],
        cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Probe(mode='fake');sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sequence=0
    def step(duration,grip=None,distance=0.):
        nonlocal sequence
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('ROS launch exited; inspect artifacts/ros_smoke.log')
            if grip is not None:
                now=time.monotonic();data=dict(sequence=sequence,unity_time_s=now,right=dict(connected=True,tracked=True,pose_valid=True,
                    position_m=[0,0,distance],rotation_xyzw=[0,0,0,1],grip=grip,thumbstick_click=False))
                sock.sendto(json.dumps(data).encode(),('127.0.0.1',input_port));sequence+=1
            rclpy.spin_once(node,timeout_sec=.01)
            if node.handoff_overflow:raise RuntimeError('Fake handoff diagnostic exceeded 20000 frames')
            if node.state.get('mode')=='FAULT':raise RuntimeError('Supervisor latched FAULT; inspect ROS log')
    report=dict(passed=False,hardware_backend='v3_moveit/GuardedGenericSystem',robot_connections=0,robot_commands=0,isolated_input_port=input_port,
        fake_velocity_tracking=smooth_follow,velocity_tracking=configured_runtime["velocity_tracking"] or smooth_follow,
        self_collision_proximity_threshold_m=threshold,
        tested_scope=['servo_motion','input_loss_stop','collision_response','strict_final_write_guard'],physical_acceptance=False)
    failure=None
    def wait_future(future,grip=None):
        deadline=time.monotonic()+5
        while not future.done() and time.monotonic()<deadline:step(.02,grip)
        if not future.done():raise RuntimeError('ROS service timeout')
    try:
        until=time.monotonic()+60
        while node.state.get('mode')!='IDLE' and time.monotonic()<until:step(.1)
        if node.state.get('mode')!='IDLE':raise RuntimeError('MoveIt/scene/controllers did not become ready')
        params=AsyncParameterClient(node,'/arm_controller')
        if not params.wait_for_services(timeout_sec=5):raise RuntimeError('JTC parameter service missing')
        future=params.get_parameters(['constraints.decelerate_on_cancel'])
        wait_future(future)
        if not future.result().values[0].bool_value:raise RuntimeError('Installed JTC lacks enabled decelerate_on_cancel support')
        step(.5,0.);initial=node.q.copy();step(.3,1.,0.)
        for i in range(1,101):step(.02,1.,i*.0002)
        step(.5,1.,.02);moved=float(np.max(np.abs(node.q-initial)))
        if moved<1e-4:raise RuntimeError('Servo produced no measurable joint motion')
        # 不再发送输入，验证失鲜后静止，不依赖客户端主动发送停止包。
        step(1.2);qstop=node.q.copy();step(.3)
        if np.max(np.abs(node.q-qstop))>1e-4:raise RuntimeError('Input loss did not stop motion')
        if node.state.get('mode')!='IDLE':raise RuntimeError('Input loss did not reach IDLE')
        # 只在 mock 场景里把障碍放到当前 TCP，验证 MoveIt 碰撞状态和停止行为。
        step(.3,0.);tf=node.tf.lookup_transform('world','tcp',rclpy.time.Time())
        co=CollisionObject();co.header.frame_id='world';co.id='smoke_injected_obstacle';co.operation=co.ADD
        primitive=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.08,.08,.08]);pose=Pose();pose.position.x=tf.transform.translation.x;pose.position.y=tf.transform.translation.y;pose.position.z=tf.transform.translation.z;pose.orientation.w=1.
        co.primitives=[primitive];co.primitive_poses=[pose]
        request=ApplyPlanningScene.Request();request.scene=PlanningScene(is_diff=True);request.scene.world.collision_objects=[co]
        future=node.scene.call_async(request)
        wait_future(future,0.)
        if not future.result().success:raise RuntimeError('Obstacle injection failed')
        before=len(node.codes);step(.3,1.);step(.6,1.,.003)
        if not any(code in (4,5) for code in node.codes[before:]):raise RuntimeError('Collision monitor did not react')
        if node.max_velocity>.192:raise RuntimeError('Simulation exceeded the configured low-speed velocity envelope')
        report.update(measured_joint_motion_rad=moved,input_loss_stops=True,injected_obstacle_detected=True,
            package_sha256=source_digest(),samples=len(node.samples),maximum_velocity_rad_s=node.max_velocity)
    except Exception as exc:
        failure=exc
    finally:
        report['supervisor_mode_message_counts']=dict(node.mode_counts)
        report['supervisor_mode_history']=node.mode_history
        try:
            sock.close();node.destroy_node();rclpy.shutdown()
        except Exception as exc:
            failure=failure or exc
        # Signal launch once; it forwards SIGINT to children. Group signalling here
        # also hits the children directly and can interrupt their cleanup twice.
        try:process.send_signal(signal.SIGINT)
        except ProcessLookupError:pass
        try:
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                # 本测试只启动 mock；不能把这个清理逻辑用于真实机器人。
                report['forced_fake_cleanup']=True
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        except Exception as exc:
            failure=failure or exc
        finally:
            log.close()
        # Serialize only after fake teardown. Header stamps and complete p/v/a/t
        # fields remain verbatim in message; receipt clocks are separate metadata.
        try:
            counts=Counter(frame[0] for frame in node.handoff_frames)
            with capture_path.open('w',encoding='utf8') as stream:
                stream.write(json.dumps(dict(kind='session_start',mode='fake',
                    hardware_backend='v3_moveit/GuardedGenericSystem',frame_limit=node.handoff_limit,
                    ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),physical_acceptance=False))+'\n')
                for topic,host_ns,ros_ns,mode,message in node.handoff_frames:
                    stream.write(json.dumps(dict(kind='message',topic=topic,host_monotonic_ns=host_ns,
                        receipt_ros_ns=ros_ns,supervisor_mode=mode,message=message_to_ordereddict(message)),
                        separators=(',',':'))+'\n')
                stream.write(json.dumps(dict(kind='session_end',frames=len(node.handoff_frames),
                    topic_counts=dict(counts),overflow_frames=node.handoff_overflow,
                    complete=node.handoff_overflow==0))+'\n')
            report['servo_handoff_capture']=dict(path=str(capture_path),frames=len(node.handoff_frames),
                topic_counts=dict(counts),overflow_frames=node.handoff_overflow,complete=node.handoff_overflow==0)
            if node.handoff_overflow:
                failure=failure or RuntimeError('Fake handoff diagnostic exceeded 20000 frames')
        except Exception as exc:
            failure=failure or exc
            report['servo_handoff_capture']=dict(path=str(capture_path),complete=False,error=str(exc))
    # Final evidence is read after teardown flushes the test-only hardware audit.
    text=(artifacts/'ros_smoke.log').read_text(errors='replace')
    adapter_calls=len(re.findall(r'Retimed \d+ geometric points to \d+ stationary vertices / \d+ curve samples',text))
    executed_messages=node.mode_counts.get('EXECUTING',0)
    report['planning_execution_coverage']=dict(
        covered=adapter_calls>0 and executed_messages>0,passed=None,
        retiming_adapter_successful_calls=adapter_calls,
        retiming_adapter_rejections=text.count('Rejected path retiming:'),
        planning_messages=node.mode_counts.get('PLANNING',0),executing_messages=executed_messages,
        execution_entries=sum(item['mode']=='EXECUTING' for item in node.mode_history),
        action_cancel_stop_verified=False,
        note='Observed modes and adapter calls only; successful plan execution/cancel requires its separate integration test.')
    try:report['final_write_guard']=guard_audit_summary(audit)
    except Exception as exc:
        report['final_write_guard']=dict(passed=False,path=str(audit),failure=str(exc))
        failure=failure or exc
    if failure is not None:
        report['failure']=str(failure)
        (artifacts/'ros_smoke_pass.json').write_text(json.dumps(report,indent=2))
        (artifacts/'ros_smoke_failed.json').write_text(json.dumps(report,indent=2))
        raise failure
    report['passed']=True
    (artifacts/'ros_smoke_pass.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--smooth-follow',action='store_true')
    run(parser.parse_args().smooth_follow)
