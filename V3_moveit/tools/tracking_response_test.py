#!/usr/bin/env python3
"""Bounded, strictly fake-only 5 cm tracking diagnostic. Never imports an SDK.

Reuse the independently clocked loopback sender and original Probe/Guard audit.
Default: +base X 5 cm cosine ramp in 4 s, hold 5 s, release Grip, verify rest.
Tracking accuracy is measured, not declared acceptable merely because this runs.
"""
import argparse
from collections import Counter
import gc
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_ROOT = Path('/media/gq/software/ChatGPT/Projects/VR/V3_moveit').resolve()
if ROOT != EXPECTED_ROOT or not (ROOT/'tools/fixed_servo_ros_test.py').is_file():
    raise RuntimeError('This diagnostic must remain in the D-drive project')
sys.path.insert(0, str(ROOT/'tools'))
from fixed_servo_ros_test import IndependentSender, NAMES
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=93)
    parser.add_argument('--ramp-seconds', type=float, default=4.)
    parser.add_argument('--auto-replan', action='store_true',
                        help='Use production automatic planning; input must remain continuously held')
    parser.add_argument('--self-collision-threshold', type=float)
    parser.add_argument('--velocity-tracking', action='store_true')
    parser.add_argument('--axis', choices=['x','y','z'], default='x', help='Robot-base translation axis')
    args = parser.parse_args()
    import fixed_servo_ros_test as sender_module
    direction={'x':'[0., 0., position]', 'y':'[-position, 0., 0.]', 'z':'[0., position, 0.]'}[args.axis]
    sender_module.SENDER_PROGRAM=sender_module.SENDER_PROGRAM.replace('position_m=[0., 0., position]', 'position_m='+direction)
    if args.self_collision_threshold is not None and (not np.isfinite(args.self_collision_threshold) or args.self_collision_threshold<=0):
        parser.error('self collision threshold must be finite and positive')
    if not 1 <= args.domain_id <= 230 or not 1. <= args.ramp_seconds <= 8.:
        parser.error('domain must be 1..230; ramp seconds must be 1..8')
    for key, value in {'ROS_DOMAIN_ID': str(args.domain_id), 'ROS_LOCALHOST_ONLY': '1',
                       'ROS_LOG_DIR': str(ROOT/'log/ros'), 'ROS_HOME': str(ROOT/'.ros'),
                       'TMPDIR': str(ROOT/'.tmp'), 'XDG_CACHE_HOME': str(ROOT/'.cache'),
                       'PYTHONDONTWRITEBYTECODE': '1'}.items():
        os.environ[key] = value
    import rclpy
    from std_msgs.msg import String
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import PoseStamped
    from rosidl_runtime_py.convert import message_to_ordereddict
    from tf2_ros import TransformException
    from ros_smoke import Probe, guard_audit_summary
    from verify_offline import source_digest

    output = ROOT/'artifacts'/f'strict_fake_5cm_{time.strftime("%Y%m%dT%H%M%S")}_{os.getpid()}'
    output.mkdir(parents=True, exist_ok=False)
    runtime = ROOT/'.tmp'/f'strict_fake_5cm_runtime_{os.getpid()}.yaml'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
        reserve.bind(('127.0.0.1', 0))
        port = reserve.getsockname()[1]
    runtime.write_text(f'quest_bind: 127.0.0.1\nquest_port: {port}\nauto_replan: {str(args.auto_replan).lower()}\n')
    audit = output/'guard.jsonl'
    log_path = output/'ros.log'
    report_path = output/'result.json'
    report = dict(passed=False, diagnostic_completed=False, fully_clean_passed=False,
                  scope='continuous loopback input tracking measurement, not tracking accuracy acceptance',
                  hardware_backend='v3_moveit/GuardedGenericSystem', mode='fake', strict=True,
                  physical_acceptance=False, robot_connections=0, sdk_calls=0,
                  package_sha256=source_digest(), script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  domain_id=args.domain_id, isolated_input_port=port, auto_replan=args.auto_replan,
                  self_collision_proximity_threshold_m=args.self_collision_threshold or .03,
                  fake_velocity_tracking=args.velocity_tracking,
                  scene='unchanged default', input_direction='base +'+args.axis.upper(), input_axis=args.axis,
                  input_displacement_m=.05, ramp_seconds=args.ramp_seconds, hold_seconds=5.,
                  output=str(output), phases=[], checks=[], capture_limit_ros=40000,
                  capture_limit_sender=5000, capture_limit_total=45000)
    report_path.write_text(json.dumps(report, indent=2)+'\n')
    print('OUTPUT='+str(output), flush=True)
    log = log_path.open('w')
    process, sender, failure = None, None, None
    rclpy.init()

    class TrackingProbe(Probe):
        def __init__(self):
            self.velocity = None
            self.latest_target = None
            self.latest_input = {}
            self.latest_joint_received_ns = None
            self.latest_joint_stamp_ns = None
            super().__init__(mode='fake')
            self.handoff_limit = 40000
            self.create_subscription(String, '/v3/input_state', self.input_state, 1000)
            self.create_subscription(PoseStamped, '/v3/hand_target', self.target, 1000)
            self.create_subscription(GoalStatusArray,
                '/arm_controller/follow_joint_trajectory/_action/status',
                lambda msg: self.capture_handoff('/arm_controller/follow_joint_trajectory/_action/status', msg),1000)
            self.create_timer(.02, self.tcp_sample)

        def status(self, msg):
            super().status(msg)
            self.capture_handoff('/v3/state', msg)

        def joints(self, msg):
            super().joints(msg)
            self.capture_handoff('/joint_states', msg)
            self.latest_joint_received_ns=time.monotonic_ns()
            self.latest_joint_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            try:
                self.velocity = np.array([msg.velocity[msg.name.index(name)] for name in NAMES])
            except (ValueError, IndexError):
                pass

        def input_state(self, msg):
            self.latest_input = json.loads(msg.data)
            self.capture_handoff('/v3/input_state', msg)

        def target(self, msg):
            self.latest_target = msg
            self.capture_handoff('/v3/hand_target', msg)

        def tcp_sample(self):
            try:
                transform = self.tf.lookup_transform('base_link', 'tcp', rclpy.time.Time())
                self.capture_handoff('/diagnostic/tcp_tf', transform)
            except TransformException:
                pass

    node = TrackingProbe()
    gc_was_enabled = gc.isenabled()
    gc.disable()

    def pump(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            if process is not None and process.poll() is not None:
                raise RuntimeError('Fake launch exited before cleanup')
            if sender is not None and sender.process.poll() is not None:
                raise RuntimeError('Independent sender exited unexpectedly')
            rclpy.spin_once(node, timeout_sec=.001)
            if node.handoff_overflow:
                raise RuntimeError('Bounded ROS capture overflow')
            if node.state.get('mode') == 'FAULT':
                raise RuntimeError('Supervisor latched FAULT; diagnostic preserved')

    def wait_for(predicate, seconds, label):
        until = time.monotonic()+seconds
        while not predicate() and time.monotonic() < until:
            pump(.01)
        if not predicate():
            raise RuntimeError('Timed out: '+label+'; state='+str(node.state))

    def fresh_feedback():
        if node.latest_joint_received_ns is None or node.latest_joint_stamp_ns is None:
            return False
        stamp_age=(node.get_clock().now().nanoseconds-node.latest_joint_stamp_ns)*1e-9
        receipt_age=(time.monotonic_ns()-node.latest_joint_received_ns)*1e-9
        return 0.<=stamp_age<=.024 and 0.<=receipt_age<=.024

    def ready_tcp():
        if not fresh_feedback():return False
        try:
            transform=node.tf.lookup_transform('base_link','tcp',rclpy.time.Time())
            stamp=transform.header.stamp.sec*10**9+transform.header.stamp.nanosec
            return 0.<=(node.get_clock().now().nanoseconds-stamp)*1e-9<=.024
        except TransformException:
            return False

    def snapshot():
        transform = node.tf.lookup_transform('base_link', 'tcp', rclpy.time.Time())
        p = transform.transform.translation
        stamp = transform.header.stamp.sec*10**9+transform.header.stamp.nanosec
        tf_age=(node.get_clock().now().nanoseconds-stamp)*1e-9
        if not 0.<=tf_age<=.024 or not fresh_feedback():
            raise RuntimeError('Diagnostic snapshot has stale joint feedback or TCP TF')
        target = node.latest_target
        target_p = None if target is None else [target.pose.position.x, target.pose.position.y, target.pose.position.z]
        return dict(host_monotonic_s=time.monotonic(), tcp_m=[p.x, p.y, p.z],
                    tcp_tf_stamp_ns=stamp, tcp_tf_age_s=tf_age,
                    joint_stamp_ns=node.latest_joint_stamp_ns,
                    joint_receipt_age_s=(time.monotonic_ns()-node.latest_joint_received_ns)*1e-9,
                    joint_q_rad=None if node.q is None else node.q.tolist(),
                    joint_v_rad_s=None if node.velocity is None else node.velocity.tolist(),
                    target_m=target_p, state=node.state.copy(), input=node.latest_input.copy())

    def phase(name, duration, command=None):
        item=dict(name=name, start_host_s=time.monotonic(), before=snapshot())
        report['phases'].append(item)
        if command is not None:
            sender.command(command)
        try:
            pump(duration)
        finally:
            item['end_host_s']=time.monotonic()
            try:item['after']=snapshot()
            except Exception as exc:item['snapshot_failure']=str(exc)
        return item

    try:
        sender = IndependentSender(port)
        sender.command(dict(op='stop'))
        pump(.5)
        others=[(name, namespace) for name,namespace in node.get_node_names_and_namespaces()
                if name != node.get_name()]
        if others:
            raise RuntimeError(f'Domain {args.domain_id} is occupied: {others}')
        process=subprocess.Popen(['ros2','launch','v3_moveit','bringup.launch.py',
            'mode:=fake','rviz:=false','guarded_fake:=true','guard_strict:=true',
            f'fake_velocity_tracking:={str(args.velocity_tracking).lower()}',
            f'guard_audit_path:={audit}',f'runtime_config:={runtime}',
            f'fake_self_collision_proximity_threshold:={args.self_collision_threshold}' if args.self_collision_threshold is not None else 'fake_self_collision_proximity_threshold:=0.03'],
            cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        wait_for(lambda: node.state.get('mode')=='IDLE' and node.q is not None,45.,'ready fake stack')
        wait_for(ready_tcp,5.,'fresh joint feedback and TCP TF')
        if node.state.get('auto_replan') is not args.auto_replan:
            raise RuntimeError('Requested auto_replan setting did not reach Supervisor')
        phase('release_initial',.2,dict(op='hold',grip=0.,distance=0.))
        phase('clutch',.05,dict(op='hold',grip=1.,distance=0.))
        if node.latest_target is None or not node.latest_input.get('held'):
            raise RuntimeError('Clutch did not establish held target')
        report['anchor']=snapshot()
        phase('ramp_5cm',args.ramp_seconds,
              dict(op='ramp',grip=1.,start=0.,end=.05,duration=args.ramp_seconds))
        # The child continues sending the ramp endpoint without a new command;
        # it never depends on recorder scheduling to keep this target alive.
        phase('hold_5s',5.)
        release=dict(name='release_to_idle',start_host_s=time.monotonic(),before=snapshot())
        report['phases'].append(release)
        sender.command(dict(op='hold',grip=0.,distance=.05))
        try:
            wait_for(lambda: node.state.get('mode')=='IDLE' and node.velocity is not None
                     and float(np.max(np.abs(node.velocity)))<1e-4 and fresh_feedback(),3.,'release stop')
        finally:
            release['end_host_s']=time.monotonic()
            release['after']=snapshot()
        terminal=phase('terminal_hold',.3)
        drift=float(np.max(np.abs(np.array(terminal['after']['joint_q_rad'])-
                                    np.array(terminal['before']['joint_q_rad']))))
        report['terminal_feedback_drift_rad']=drift
        samples=[f[4] for f in node.handoff_frames if f[0]=='/joint_states'
                 and terminal['start_host_s']<=f[1]*1e-9<=terminal['end_host_s']]
        stamps={m.header.stamp.sec*10**9+m.header.stamp.nanosec for m in samples}
        if len(stamps)<10:
            raise RuntimeError('Terminal rest lacks 10 distinct fresh joint feedback stamps')
        q0=np.array(terminal['before']['joint_q_rad'])
        every_sample_drift=max(float(np.max(np.abs(np.array([m.position[m.name.index(n)] for n in NAMES])-q0))) for m in samples)
        report['terminal_joint_messages']=len(samples)
        report['terminal_distinct_joint_stamps']=len(stamps)
        report['terminal_feedback_maximum_sample_drift_rad']=every_sample_drift
        if drift>1e-8 or every_sample_drift>1e-8:
            raise RuntimeError('Released terminal feedback drifted')
        report['checks'].append('grip_release_feedback_stationary')
        report['diagnostic_completed']=True
    except Exception as exc:
        failure=exc
    finally:
        try:
            try:
                if sender is not None:
                    timing=sender.close()
                    frames=timing.pop('frames')
                    report['independent_sender']=timing
                    for frame in frames:
                        node.handoff_frames.append(('/test/quest_udp_tx',frame['host_monotonic_ns'],None,None,frame['packet']))
                    if timing['error'] or timing['maximum_active_send_gap_s']>=.08:
                        raise RuntimeError('Continuous sender failed/deadline exceeded: '+str(timing))
            except Exception as exc:
                failure=failure or exc
            cleanup_start=log_path.stat().st_size
            if process is not None:
                try:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=15)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    report['forced_fake_cleanup']=True
                    try:
                        try:os.killpg(process.pid,signal.SIGTERM)
                        except ProcessLookupError:pass
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        try:os.killpg(process.pid,signal.SIGKILL)
                        except ProcessLookupError:pass
                        process.wait(timeout=5)
                    failure=failure or RuntimeError('Forced fake cleanup')
            # Check this launch's dedicated process group on every path, even
            # when its parent already exited. Never inspect or signal other groups.
            def group_exists():
                if process is None:return False
                try:os.killpg(process.pid,0);return True
                except ProcessLookupError:return False
            if group_exists():
                report['residual_fake_process_group_after_parent_exit']=True
                report['forced_fake_cleanup']=True
                for sig in (signal.SIGTERM,signal.SIGKILL):
                    try:os.killpg(process.pid,sig)
                    except ProcessLookupError:break
                    until=time.monotonic()+2.
                    while group_exists() and time.monotonic()<until:time.sleep(.02)
                    if not group_exists():break
                failure=failure or RuntimeError('Residual fake launch process group required cleanup')
            report['fake_launch_group_remaining']=group_exists()
            node.destroy_node()
            rclpy.shutdown()
            log.close()
            text=log_path.read_text(errors='replace')
            cleanup_text=log_path.read_bytes()[cleanup_start:].decode(errors='replace')
            deaths=[line for line in text.splitlines() if 'process has died' in line]
            cleanup_deaths=[line for line in cleanup_text.splitlines() if 'process has died' in line]
            known=[line for line in cleanup_deaths if '[move_group-' in line and 'exit code -11' in line]
            unexpected=deaths.copy()
            for line in known:unexpected.remove(line)
            cleanup_errors=[line for line in cleanup_text.splitlines() if '[ERROR]' in line or 'Traceback' in line]
            report['cleanup']=dict(launch_returncode=process.returncode if process else None,
                known_move_group_shutdown_segfault=known,unexpected_process_deaths=unexpected,
                cleanup_error_lines=cleanup_errors,
                clean=not deaths and not cleanup_errors and not report.get('forced_fake_cleanup') and process is not None and process.returncode==0)
            if unexpected:
                failure=failure or RuntimeError('Unexpected process death in runtime or cleanup')
            frames=sorted(node.handoff_frames,key=lambda item:item[1])
            counts=Counter(item[0] for item in frames)
            if len(frames)>45000 or node.handoff_overflow:
                failure=failure or RuntimeError('Bounded capture overflow')
            capture=output/'capture.jsonl'
            with capture.open('w') as stream:
                stream.write(json.dumps(dict(kind='session_start',mode='fake',strict=True,
                    package_sha256=report['package_sha256'],domain_id=args.domain_id,
                    frame_limit=45000,physical_acceptance=False))+'\n')
                for topic,host_ns,ros_ns,mode,msg in frames:
                    stream.write(json.dumps(dict(kind='message',topic=topic,host_monotonic_ns=host_ns,
                        receipt_ros_ns=ros_ns,supervisor_mode=mode,
                        message=msg if isinstance(msg,dict) else message_to_ordereddict(msg)),separators=(',',':'))+'\n')
                stream.write(json.dumps(dict(kind='session_end',frames=len(frames),topic_counts=dict(counts),
                    overflow_frames=node.handoff_overflow,complete=node.handoff_overflow==0 and len(frames)<=45000))+'\n')
            report['capture']=dict(path=str(capture),frames=len(frames),topic_counts=dict(counts),overflow_frames=node.handoff_overflow)
            report['supervisor_mode_history']=node.mode_history
            report['maximum_velocity_rad_s']=node.max_velocity
            report['joint_samples']=len(node.samples)
            for item in report['phases']:
                start,end=item['start_host_s'],item.get('end_host_s',float('inf'))
                selected=[f for f in frames if start<=f[1]*1e-9<=end]
                states=[json.loads(f[4].data) for f in selected if f[0]=='/v3/state']
                inputs=[json.loads(f[4].data) for f in selected if f[0]=='/v3/input_state']
                item['servo_code_message_counts']=dict(Counter(str(s.get('servo_code')) for s in states))
                item['mode_message_counts']=dict(Counter(s.get('mode') for s in states))
                item['input_reason_message_counts']=dict(Counter(s.get('reason') for s in inputs))
                item['code4_message_fraction']=(sum(s.get('servo_code')==4 for s in states)/len(states)) if states else None
                item['maximum_observed_input_age_s']=max((s.get('age_s',0.) for s in inputs),default=None)
                if item['name'] in ('ramp_5cm','hold_5s'):
                    item['continuous_held_input']=bool(inputs) and all(s.get('held') is True for s in inputs)
                    item['servo_follow_only']=bool(states) and all(s.get('mode')=='FOLLOW' for s in states)
                    item['continuous_held_follow']=item['continuous_held_input'] and item['servo_follow_only']
                before,after=item.get('before'),item.get('after')
                if before and after:
                    delta=np.array(after['tcp_m'])-np.array(before['tcp_m'])
                    item['tcp_displacement_m']=delta.tolist()
                    if after['target_m'] is not None:
                        error=np.array(after['target_m'])-np.array(after['tcp_m'])
                        item['end_target_error_m']=error.tolist()
                        item['end_target_error_norm_m']=float(np.linalg.norm(error))
            hold=next((p for p in report['phases'] if p['name']=='hold_5s' and 'after' in p),None)
            if hold and 'anchor' in report:
                anchor=np.array(report['anchor']['tcp_m'])
                final=np.array(hold['after']['tcp_m'])
                target=np.array(hold['after']['target_m'])
                report['tracking_measurement']=dict(tcp_displacement_m=(final-anchor).tolist(),
                    requested_displacement_m=(target-np.array(report['anchor']['target_m'])).tolist(),
                    achieved_requested_axis_fraction_of_5cm=float((final-anchor)['xyz'.index(args.axis)]/.05),
                    remaining_error_m=(target-final).tolist(),remaining_error_norm_m=float(np.linalg.norm(target-final)),
                    claim='Measurement only; no tracking-accuracy or physical-safety acceptance threshold')
            report['input_continuity_verified']=all(p.get('continuous_held_input',False) for p in report['phases']
                if p['name'] in ('ramp_5cm','hold_5s')) and hold is not None
            if not report['input_continuity_verified']:
                failure=failure or RuntimeError('Held input was interrupted; do not attribute tracking error solely to collision/planning')
            try:
                report['final_write_guard']=guard_audit_summary(audit)
                terminal=next((p for p in report['phases'] if p['name']=='terminal_hold'),None)
                if terminal is None:
                    raise RuntimeError('No terminal hold checkpoint')
                rows=[json.loads(line) for line in audit.read_text().splitlines()]
                writes=[r for r in rows if r.get('event')=='write' and r.get('result')=='accepted'
                        and terminal['start_host_s']<=r['host_monotonic_ns']*1e-9<=terminal['end_host_s']]
                if len(writes)<10 or max(float(np.max(np.abs(np.array(r['command'])-np.array(writes[0]['command'])))) for r in writes)>1e-8:
                    raise RuntimeError('Terminal final-write commands did not remain stationary')
                report['terminal_guard_accepted_writes']=len(writes)
                report['checks'].append('grip_release_strict_final_command_hold')
            except Exception as exc:
                report.setdefault('final_write_guard',dict(passed=False,path=str(audit)))
                failure=failure or exc
            report['planning_mode_entries']=[s for s in node.mode_history if s['mode'] in ('PLANNING','EXECUTING','CANCELING','RESUMING')]
            if not args.auto_replan and report['planning_mode_entries']:
                failure=failure or RuntimeError('Unexpected OMPL action in pure Servo diagnostic')
            if not args.auto_replan and any(not p.get('servo_follow_only',False) for p in report['phases'] if p['name'] in ('ramp_5cm','hold_5s')):
                failure=failure or RuntimeError('Pure Servo diagnostic left FOLLOW while input was held')
            report['source_sha256_after']=source_digest()
            if report['source_sha256_after']!=report['package_sha256']:
                failure=failure or RuntimeError('Source digest changed while diagnostic ran')
        except Exception as exc:
            failure=failure or exc
        finally:
            if not log.closed:log.close()
            if gc_was_enabled:gc.enable()
            report['passed']=failure is None and report['diagnostic_completed']
            report['fully_clean_passed']=report['passed'] and report.get('cleanup',{}).get('clean',False)
            if failure is not None:report['failure']=str(failure)
            report_path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':
    main()
