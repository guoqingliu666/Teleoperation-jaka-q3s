"""Exercise the ordinary operator Session with real fake ROS and isolated input.

USB discovery alone is substituted; released-input waiting, launch, FK/Servo,
feedback and graceful session stop run normally. Never selects hardware.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import socket
import tempfile
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import yaml
import operator_session as operator
from operator_xr.state import snapshot

ROOT=Path(__file__).resolve().parents[1]

def main(output=None):
    if os.environ.get('ROS_DOMAIN_ID') not in ('74','75'):raise RuntimeError('Use isolated test domain 74 or 75')
    if output is None:
        output=ROOT/'artifacts'/('operator_startup_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    output.mkdir(parents=True,exist_ok=True)
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reserve:
        reserve.bind(('127.0.0.1',0));port=reserve.getsockname()[1]
    assert port not in (5006,5008)
    with tempfile.TemporaryDirectory(dir=ROOT/'.tmp') as folder:
        root=Path(folder);(root/'.cache/operator').mkdir(parents=True);(root/'tools').symlink_to(ROOT/'tools',target_is_directory=True)
        site=yaml.safe_load((ROOT/'site_ubuntu.yaml').read_text());site.update(quest_port=port,quest_bind='127.0.0.1');(root/'site_ubuntu.yaml').write_text(yaml.safe_dump(site))
        cfg=root/'src/v3_moveit/config';cfg.mkdir(parents=True);(cfg/'payload.json').write_bytes((ROOT/'src/v3_moveit/config/payload.json').read_bytes())
        started=threading.Event();done=threading.Event();samples=[];failures=[];mode_verified=False;phases=[]
        ready_at=None;first_press=None;released_at=None;initial_input=[];released_input=[]
        with patch.object(operator,'ROOT',root),patch('operator_quest.ensure_connection',return_value={'ok':True}):
            session=operator.Session(SimpleNamespace(task='teleop',hardware=False))
            original=session.run_child
            def launch(argv,label,**kwargs):
                assert 'mode:=fake' in argv and not any('hardware' in x for x in argv)
                started.set();return original(argv,label,**kwargs)
            session.run_child=launch
            def input_and_observer():
                nonlocal mode_verified,ready_at,first_press,released_at
                import rclpy
                from rclpy.signals import SignalHandlerOptions
                from rcl_interfaces.srv import GetParameters
                from sensor_msgs.msg import JointState
                node=None;client=None;future=None;requested_at=0.;follow_start=None;sequence=0;deadline=time.monotonic()+45;next_read=0.
                try:
                    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
                        while not done.is_set() and time.monotonic()<deadline:
                            now=time.monotonic()
                            if started.is_set() and node is None:
                                rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('operator_startup_check')
                                def joints(msg):
                                    if all(f'joint_{i}' in msg.name for i in range(1,7)):samples.append((time.monotonic(),np.array([msg.position[msg.name.index(f'joint_{i}')] for i in range(1,7)])))
                                node.create_subscription(JointState,'/joint_states',joints,1)
                                client=node.create_client(GetParameters,'/v3_supervisor/get_parameters')
                            if node is not None:
                                rclpy.spin_once(node,timeout_sec=.001)
                                for _ in range(4):rclpy.spin_once(node,timeout_sec=0)
                                if future is not None and future.done() and not mode_verified:
                                    assert future.result().values[0].string_value=='fake';mode_verified=True
                                # DDS may discover the service before its return path and lose the first reply.
                                if not mode_verified and client.service_is_ready() and (future is None or now-requested_at>=1.):
                                    if future is not None:client.remove_pending_request(future)
                                    future=client.call_async(GetParameters.Request(names=['mode']));requested_at=now
                            if now>=next_read:
                                cue=snapshot(root/'.cache/operator')['guidance'];next_read=now+.05
                                if not phases or phases[-1]!=cue['phase']:phases.append(cue['phase'])
                                if mode_verified and samples and cue['phase']=='ready' and follow_start is None:
                                    ready_at=now;follow_start=now+1.1
                            elapsed=now-follow_start if follow_start is not None else -1
                            held=0<elapsed<5
                            # Move the released controller too. Only the held segment may move the robot.
                            if held:
                                if first_press is None:first_press=now
                                distance=elapsed*.004
                            elif elapsed>=5:
                                if released_at is None:released_at=now
                                distance=.02+.006*np.sin((elapsed-5)*5)
                                released_input.append(distance)
                            else:
                                distance=.006*np.sin(now*5)
                                if ready_at is not None:initial_input.append(distance)
                            packet=dict(sequence=sequence,unity_time_s=now,right=dict(connected=True,tracked=True,pose_valid=True,position_m=[0,0,distance],rotation_xyzw=[0,0,0,1],grip=1. if held else 0.,thumbstick_click=False))
                            sock.sendto(json.dumps(packet).encode(),('127.0.0.1',port));sequence+=1
                            if elapsed>7:session.request_stop();break
                            time.sleep(.01)
                        else:raise RuntimeError('Operator fake startup timed out')
                except Exception as e:failures.append(str(e));session.request_stop()
                finally:
                    if node is not None:node.destroy_node();rclpy.shutdown()
            worker=threading.Thread(target=input_and_observer);worker.start()
            try:code=session.run()
            finally:done.set();worker.join(timeout=5)
            for p in session.out.iterdir():
                if p.is_file():(output/p.name).write_bytes(p.read_bytes())
        def joint_travel(start,end):
            if start is None or end is None:return None
            positions=[q for t,q in samples if start<=t<=end]
            return float(np.ptp(positions,axis=0).max()) if len(positions)>=5 else None
        initial=joint_travel(ready_at,first_press)
        motion=joint_travel(first_press,released_at)
        end=samples[-1][0] if samples else 0.
        quiet=joint_travel(end-1,end) if released_at is not None and end>released_at+1 else None
        moved_initial=float(np.ptp(initial_input)) if initial_input else 0.
        moved_released=float(np.ptp(released_input)) if released_input else 0.
        unheld_moving=min(moved_initial,moved_released)>.005
        phases.append(snapshot(root/'.cache/operator')['guidance']['phase'])
        order=[p for i,p in enumerate(phases) if p in ('ready','following','paused','ended') and p not in phases[:i]]
        passed=(code==0 and mode_verified and unheld_moving and initial is not None and initial<.001
                and motion is not None and motion>1e-4 and quiet is not None and quiet<.001
                and not failures and order==['ready','following','paused','ended'])
        report=dict(passed=passed,ordinary_operator_session=True,actual_fake_ros=True,usb_discovery_substituted=True,real_released_input_wait=True,
                    production_input_ports_used=False,domain_id=os.environ['ROS_DOMAIN_ID'],udp_port=port,measured_joint_motion_rad=motion,final_second_drift_rad=quiet,
                    headset_phases=phases,mode_verified=mode_verified,joint_samples=len(samples),initial_unheld_drift_rad=initial,
                    released_controller_moving=unheld_moving,initial_controller_travel_m=moved_initial,after_release_controller_travel_m=moved_released,
                    exit_code=code,failures=failures,robot_connections=0,physical_acceptance=False)
        (output/'startup_check.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
        if not passed:raise RuntimeError('Ordinary operator fake startup check failed')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    main(parser.parse_args().output)
