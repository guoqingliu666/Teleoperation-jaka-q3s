#!/usr/bin/env python3
"""Fake-only Servo handoff integration cases, with strict final-write evidence.

No hardware mode is selectable. Two fresh launches use dedicated ROS domains and
UDP ports: pure Servo with auto_replan=false, then the default automatic planner.
The unchanged default scene remains active. The caller must source Jazzy/install.
"""
import argparse
from collections import Counter
import gc
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
import yaml

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NAMES = [f'joint_{i}' for i in range(1, 7)]


# Runs in a separate isolated Python interpreter with only the standard library.
# stdin EOF also terminates it, so a dead recorder cannot leave a command source.
SENDER_PROGRAM = r"""
import json, math, os, select, socket, sys, time
port = int(sys.argv[1])
if not 1 <= port <= 65535:
    raise ValueError('invalid fake loopback destination')
source = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
active = None
frames, controls, long_gaps = [], [], []
sequence, previous_ns, maximum_gap, next_send = 0, None, 0., 0.
buffer = b''
error = None
quit_requested = False
try:
    while not quit_requested:
        now = time.monotonic()
        wait = max(0., next_send-now) if active is not None else .1
        readable, _, _ = select.select([sys.stdin.fileno()], [], [], min(wait, .1))
        if readable:
            chunk = os.read(sys.stdin.fileno(), 65536)
            if not chunk:
                break
            buffer += chunk
            if len(buffer) > 65536:
                raise ValueError('fake sender control backlog')
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                command = json.loads(line)
                operation = command['op']
                now = time.monotonic()
                controls.append(dict(host_monotonic_ns=time.monotonic_ns(), **command))
                if len(controls) > 1000:
                    raise ValueError('fake sender control budget exceeded')
                if operation == 'shutdown':
                    quit_requested = True
                    active = None
                    break
                if operation == 'stop':
                    active, previous_ns = None, None
                elif operation in ('hold', 'ramp'):
                    if not math.isfinite(command['grip']) or not 0 <= command['grip'] <= 1:
                        raise ValueError('invalid fake sender grip')
                    fields = ('distance',) if operation == 'hold' else ('start', 'end', 'duration')
                    if any(not math.isfinite(command[key]) for key in fields):
                        raise ValueError('invalid fake sender point')
                    if operation == 'ramp' and command['duration'] <= 0:
                        raise ValueError('invalid fake sender segment duration')
                    if active is None:
                        next_send, previous_ns = now, None
                    active = dict(command, begun=now)
                else:
                    raise ValueError('unsupported fake sender operation')
        if quit_requested:
            break
        now = time.monotonic()
        if active is not None and now >= next_send:
            if len(frames) >= 5000:
                raise ValueError('fake sender exceeded 5000-frame budget')
            if active['op'] == 'hold':
                position = active['distance']
            else:
                phase = min(1., max(0., (now-active['begun'])/active['duration']))
                position = active['start']+(active['end']-active['start'])*(1-math.cos(math.pi*phase))/2
            packet = dict(sequence=sequence, unity_time_s=now,
                right=dict(connected=True, tracked=True, pose_valid=True,
                    position_m=[0., 0., position], rotation_xyzw=[0., 0., 0., 1.],
                    grip=active['grip'], thumbstick_click=False))
            source.sendto(json.dumps(packet).encode(), ('127.0.0.1', port))
            sent_ns = time.monotonic_ns()
            if previous_ns is not None:
                gap = (sent_ns-previous_ns)*1e-9
                maximum_gap = max(maximum_gap, gap)
                if gap > .02:
                    long_gaps.append(dict(start_host_ns=previous_ns, end_host_ns=sent_ns, duration_s=gap))
            previous_ns = sent_ns
            frames.append(dict(host_monotonic_ns=sent_ns, packet=packet))
            sequence += 1
            # No catch-up bursts or relabeling stale samples after a delay.
            next_send = time.monotonic()+.008
except Exception as exc:
    error = str(exc)
finally:
    source.close()
    print(json.dumps(dict(mode='fake', loopback_only=True, frames=frames, controls=controls,
        frame_limit=5000, maximum_active_send_gap_s=maximum_gap,
        active_send_gaps_over_20ms=long_gaps, error=error)), flush=True)
"""


class IndependentSender:
    """A fake loopback-only source; continuous segments are owned by the child."""
    def __init__(self, port):
        self.process = subprocess.Popen([sys.executable, '-I', '-S', '-u', '-c', SENDER_PROGRAM, str(port)],
            cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, start_new_session=True)
        self.current = None

    def command(self, value):
        if self.process.poll() is not None:
            raise RuntimeError('Independent fake sender exited unexpectedly')
        if value == self.current:
            return
        self.process.stdin.write(json.dumps(value)+'\n')
        self.process.stdin.flush()
        self.current = value.copy()

    def close(self):
        try:
            if self.process.poll() is None:
                self.command(dict(op='shutdown'))
            stdout, stderr = self.process.communicate(timeout=3)
        except Exception:
            self.process.kill()
            self.process.communicate(timeout=3)
            raise
        if self.process.returncode != 0:
            raise RuntimeError('Fake sender failed: '+stderr)
        return json.loads(stdout)


def execute_session(case, domain, output, smooth_follow=False):
    for key, value in {'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
                       'ROS_LOG_DIR': str(ROOT/'log/ros'), 'ROS_HOME': str(ROOT/'.ros'),
                       'TMPDIR': str(ROOT/'.tmp'), 'XDG_CACHE_HOME': str(ROOT/'.cache'),
                       'PYTHONDONTWRITEBYTECODE': '1'}.items():
        os.environ[key] = value
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from std_msgs.msg import String
    from rosidl_runtime_py.convert import message_to_ordereddict
    from ros_smoke import Probe, guard_audit_summary
    from verify_offline import source_digest

    output.mkdir(parents=True, exist_ok=True)
    audit = output/'guard.jsonl'
    report_path = output/'result.json'
    # Archive old evidence rather than allowing an old passing audit to survive.
    if audit.exists():
        audit.rename(output/f'guard.previous.{time.time_ns()}.jsonl')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    runtime = ROOT/f'.tmp/fixed_servo_{case}_runtime.yaml'
    runtime.parent.mkdir(parents=True, exist_ok=True)
    runtime.write_text(f'quest_bind: 127.0.0.1\nquest_port: {port}\n' +
                       ('auto_replan: false\n' if case == 'pure_servo' else ''))
    configured_servo=yaml.safe_load((ROOT/"src/v3_moveit/config/servo.yaml").read_text())
    configured_runtime=yaml.safe_load((ROOT/"src/v3_moveit/config/runtime.yaml").read_text())
    threshold=.03 if case=="auto_replan" else configured_servo["self_collision_proximity_threshold"]
    report = dict(passed=False, fully_clean_passed=False, case=case,
                  package_sha256=source_digest(), hardware_backend='v3_moveit/GuardedGenericSystem',
                  strict=True, physical_acceptance=False, robot_connections=0, sdk_calls=0,
                  domain_id=domain, isolated_input_port=port,
                  auto_replan=case != 'pure_servo', scene='unchanged default', checks=[], phases=[],
                  fake_velocity_tracking=smooth_follow,
                  self_collision_proximity_threshold_m=threshold,
                  velocity_tracking=configured_runtime['velocity_tracking'] or smooth_follow,
                  collision_fixture='stricter 30 mm proximity for actual collision-status-triggered OMPL' if case=='auto_replan' else 'configured profile')
    report_path.write_text(json.dumps(report, indent=2))
    failure, process = None, None
    log_path = output/'ros.log'
    log = log_path.open('w')
    sender = None
    rclpy.init()

    class ExtendedProbe(Probe):
        def __init__(self):
            self.velocity = None
            self.action_history = []
            super().__init__(mode='fake')
            self.handoff_limit = 15000  # Plus 5000 child sender frames: 20000 total.
            self.create_subscription(String, '/v3/input_state',
                lambda msg: self.capture_handoff('/v3/input_state', msg), 1000)
            self.create_subscription(GoalStatusArray,
                '/arm_controller/follow_joint_trajectory/_action/status', self.action_status, 10)

        def joints(self, msg):
            super().joints(msg)
            try:
                self.velocity = np.array([msg.velocity[msg.name.index(name)] for name in NAMES])
            except (ValueError, IndexError):
                pass

        def action_status(self, msg):
            now = time.monotonic()
            for status in msg.status_list:
                value = dict(uuid=bytes(status.goal_info.goal_id.uuid).hex(), status=status.status,
                             host_monotonic_s=now)
                if not any(old['uuid'] == value['uuid'] and old['status'] == value['status']
                           for old in self.action_history):
                    self.action_history.append(value)

    node = ExtendedProbe()
    gc_was_enabled = gc.isenabled()
    # The bounded recorder retains <=15000 ROS messages, and the independent
    # sender retains <=5000 packets. GC is restored only after inputs stop.
    gc.disable()

    def pump(seconds, grip=None, distance=0., preserve_sender=False):
        if not preserve_sender:
            sender.command(dict(op='stop') if grip is None else
                           dict(op='hold', grip=float(grip), distance=float(distance)))
        start = time.monotonic()
        while time.monotonic()-start < seconds:
            if process is not None and process.poll() is not None:
                raise RuntimeError('Fake launch exited before requested cleanup')
            if sender.process.poll() is not None:
                raise RuntimeError('Independent fake sender exited unexpectedly')
            rclpy.spin_once(node, timeout_sec=.001)
            if node.handoff_overflow:
                raise RuntimeError('Fake diagnostic exceeded 15000 ROS / 20000 total frames')
            if node.state.get('mode') == 'FAULT':
                raise RuntimeError('Supervisor latched FAULT')

    def wait_for(predicate, timeout, grip=None, distance=0., label='condition'):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            pump(.01, grip, distance)
        if not predicate():
            raise RuntimeError(f'Timed out waiting for {label}; state={node.state}')

    def tcp():
        transform = node.tf.lookup_transform('base_link', 'tcp', rclpy.time.Time())
        p = transform.transform.translation
        return np.array([p.x, p.y, p.z])

    def segment(name, seconds, start, end, grip=1.):
        begin = time.monotonic()
        q0, p0 = node.q.copy(), tcp()
        first_mode = len(node.mode_history)
        sender.command(dict(op='ramp', grip=float(grip), start=float(start),
                            end=float(end), duration=float(seconds)))
        pump(seconds, preserve_sender=True)
        q1, p1 = node.q.copy(), tcp()
        item = dict(name=name, start_host_s=begin, end_host_s=time.monotonic(),
                    input_start_m=start, input_end_m=end, tcp_start_m=p0.tolist(), tcp_end_m=p1.tolist(),
                    max_joint_motion_rad=float(np.max(abs(q1-q0))),
                    tcp_x_motion_m=float(p1[0]-p0[0]), modes=node.mode_history[first_mode:])
        report['phases'].append(item)
        return item

    def require_motion(item, sign):
        if item['max_joint_motion_rad'] < 1e-4 or sign*item['tcp_x_motion_m'] < 5e-5:
            raise RuntimeError(f"No measured motion in requested direction: {item['name']}")
        report['checks'].append(item['name']+'_measured_motion')

    def stopped(name, grip=None, distance=0.):
        wait_for(lambda: node.state.get('mode') == 'IDLE' and node.velocity is not None
                 and float(np.max(abs(node.velocity))) < 1e-4,
                 2., grip, distance, label=name+' IDLE and stationary feedback')
        begin = time.monotonic()
        q0 = node.q.copy()
        pump(.2, grip, distance)
        if float(np.max(abs(node.q-q0))) > 1e-8:
            raise RuntimeError(name+' terminal feedback drifted')
        report['phases'].append(dict(name=name, start_host_s=begin, end_host_s=time.monotonic(),
                                     stationary_checkpoint=True))
        report['checks'].append(name+'_feedback_stationary')

    try:
        sender = IndependentSender(port)
        # Check isolation before launch; a test never attaches to an existing stack.
        pump(.5)
        others = [(name, namespace) for name, namespace in node.get_node_names_and_namespaces()
                  if name != node.get_name()]
        if others:
            raise RuntimeError(f'Domain {domain} already has nodes: {others}')
        process = subprocess.Popen(['ros2', 'launch', 'v3_moveit', 'bringup.launch.py',
            'mode:=fake', 'rviz:=false', 'guarded_fake:=true', 'guard_strict:=true',
            f'guard_audit_path:={audit}', f'runtime_config:={runtime}',
            f'fake_velocity_tracking:={str(smooth_follow).lower()}',
            f'fake_self_collision_proximity_threshold:={threshold}'],
            cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        wait_for(lambda: node.state.get('mode') == 'IDLE' and node.q is not None,
                 60., label='ready fake stack')
        pump(.2, 0.)
        if bool(node.state.get('auto_replan')) != (case != 'pure_servo'):
            raise RuntimeError('Runtime auto_replan setting did not reach supervisor')
        if case == 'pure_servo':
            pump(.05, 1., 0.)
            first_follow = time.monotonic()
            forward = segment('forward', 1.2, 0., .004)
            require_motion(forward, 1)
            reverse = segment('reverse', 1.6, .004, -.004)
            require_motion(reverse, -1)
            if node.state.get('mode') != 'FOLLOW':
                raise RuntimeError('Continuous input did not remain in FOLLOW')
            unexpected = [entry for entry in node.mode_history if entry['host_monotonic_s'] >= first_follow
                          and entry['mode'] in ('PAUSING', 'PLANNING', 'EXECUTING', 'CANCELING')]
            if unexpected:
                raise RuntimeError('Pure Servo motion included a planning/action transition')
            report['checks'].append('continuous_follow_without_ompl')
            pump(.02, 0., -.004)
            stopped('grip_release_stop', 0., -.004)
            # Move the released hand far away, then clutch at that same pose.
            # The robot must remain at rest until a relative hand displacement.
            pump(.1, 0., .05)
            clutch_q = node.q.copy()
            pump(.05, 1., .05)
            if float(np.max(abs(node.q-clutch_q))) > 1e-7:
                raise RuntimeError('Reclutch introduced a position jump')
            report['checks'].append('reclutch_without_jump')
            require_motion(segment('reclutch_follow', 1.2, .05, .054), 1)
            # Stop UDP entirely, without transmitting a release packet.
            if node.state.get('mode') != 'FOLLOW' or node.state.get('held') is not True:
                raise RuntimeError('Input-loss test did not start from active held FOLLOW')
            stopped('input_loss_stop')
        else:
            pump(.05, 1., 0.)
            # At the unchanged near-collision start, a held stable target causes
            # real Servo status -> supervisor planning, rather than forged status.
            segment('stable_plan_target', .35, 0., .012)
            wait_for(lambda: node.state.get('mode') == 'EXECUTING', 8., 1., .012,
                     label='automatic OMPL execution')
            initial_action_q = node.q.copy()
            wait_for(lambda: float(np.max(abs(node.q-initial_action_q))) > 1e-4,
                     2., 1., .012, label='actual OMPL action motion')
            report['checks'].append('automatic_ompl_actual_motion')
            cancel_at = time.monotonic()
            pump(.02, 0., .012)
            stopped('ompl_cancel_stop', 0., .012)
            # Require action status evidence in addition to supervisor mode names.
            wait_for(lambda: any(item['status'] == 5 and item['host_monotonic_s'] >= cancel_at
                                 for item in node.action_history), 1., 0., .012,
                     label='FollowJointTrajectory CANCELED status')
            canceled = {item['uuid'] for item in node.action_history
                        if item['status'] == 5 and item['host_monotonic_s'] >= cancel_at}
            if not any(item['uuid'] in canceled and item['status'] == 2 for item in node.action_history):
                raise RuntimeError('Canceled action has no prior EXECUTING status evidence')
            modes = [item['mode'] for item in node.mode_history if item['host_monotonic_s'] >= cancel_at]
            if 'CANCELING' not in modes or 'RESUMING' not in modes:
                raise RuntimeError('Cancel-to-resume transition not observed')
            report['checks'].append('ompl_cancel_result_then_resume')
            pump(.05, 1., .012)
            # Keep the target changing across the 2 mm / .25 s stability gate
            # so this observation measures Servo, not a second automatic plan.
            resume = segment('follow_after_ompl_cancel', .45, .012, .020)
            require_motion(resume, 1)
            if node.state.get('mode') != 'FOLLOW' or any(
                item['mode'] in ('PLANNING', 'EXECUTING')
                and resume['start_host_s'] <= item['host_monotonic_s'] <= resume['end_host_s']
                for item in node.mode_history):
                raise RuntimeError('Post-cancel movement was not independent Servo FOLLOW')
            report['checks'].append('actual_follow_after_ompl_cancel')
            pump(.02, 0., .020)
            stopped('final_release_stop', 0., .020)
    except Exception as error:
        failure = error
    finally:
        try:
            # Stop the independent source before any potentially slow ROS teardown.
            # Child stdout is drained by communicate(), so full timing evidence
            # cannot block shutdown on a full pipe.
            try:
                if sender is None:
                    raise RuntimeError('Independent sender did not initialize')
                timing = sender.close()
                frames = timing.pop('frames')
                report['independent_sender'] = timing
                for frame in frames:
                    node.handoff_frames.append(('/test/quest_udp_tx', frame['host_monotonic_ns'],
                                                None, None, frame['packet']))
                node.handoff_frames.sort(key=lambda item: item[1])
                if len(node.handoff_frames) > 20000:
                    failure = failure or RuntimeError('Combined capture exceeded 20000-frame budget')
                if timing['error'] is not None:
                    failure = failure or RuntimeError('Independent sender: '+timing['error'])
                if timing['maximum_active_send_gap_s'] >= .08:
                    failure = failure or RuntimeError('Independent sender itself exceeded the 80 ms input deadline')
            except Exception as error:
                failure = failure or error
                report['independent_sender_failure'] = str(error)
            report['supervisor_mode_history'] = node.mode_history
            report['supervisor_mode_message_counts'] = dict(node.mode_counts)
            report['action_status_history'] = node.action_history
            report['samples'] = len(node.samples)
            report['maximum_velocity_rad_s'] = node.max_velocity
            cleanup_start = log_path.stat().st_size
            try:
                if process is not None:
                    try:
                        process.send_signal(signal.SIGINT)
                        process.wait(timeout=15)
                    except ProcessLookupError:
                        pass
                    except subprocess.TimeoutExpired:
                        report['forced_fake_cleanup'] = True
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=10)
            except Exception as error:
                failure = failure or error
            node.destroy_node()
            rclpy.shutdown()
            log.close()
            cleanup_text = log_path.read_bytes()[cleanup_start:].decode(errors='replace')
            death_lines = [line for line in cleanup_text.splitlines() if 'process has died' in line]
            known_death = [line for line in death_lines if '[move_group-' in line and 'exit code -11' in line]
            unexpected_death = [line for line in death_lines if line not in known_death]
            report['cleanup'] = dict(clean=not death_lines and not report.get('forced_fake_cleanup', False)
                and process is not None and process.returncode == 0,
                launch_returncode=process.returncode if process is not None else None,
                known_move_group_shutdown_segfault=known_death, unexpected_process_deaths=unexpected_death,
                note='Known MoveGroup shutdown crash is reported separately; a functional pass is not a clean-exit claim.')
            if unexpected_death:
                failure = failure or RuntimeError('Unexpected process death during fake cleanup')
            try:
                capture_path = output/'capture.jsonl'
                counts = Counter(frame[0] for frame in node.handoff_frames)
                with capture_path.open('w') as stream:
                    stream.write(json.dumps(dict(kind='session_start', mode='fake',
                        hardware_backend='v3_moveit/GuardedGenericSystem', physical_acceptance=False))+'\n')
                    for topic, host_ns, ros_ns, mode, msg in node.handoff_frames:
                        stream.write(json.dumps(dict(kind='message', topic=topic, host_monotonic_ns=host_ns,
                            receipt_ros_ns=ros_ns, supervisor_mode=mode,
                            message=msg if isinstance(msg, dict) else message_to_ordereddict(msg)),
                            separators=(',', ':'))+'\n')
                    stream.write(json.dumps(dict(kind='session_end', frames=len(node.handoff_frames),
                        topic_counts=dict(counts), overflow_frames=node.handoff_overflow,
                        complete=node.handoff_overflow == 0))+'\n')
                if node.handoff_overflow:
                    raise RuntimeError('Capture overflow')
                input_transitions = []
                sender_gaps = []
                previous_input, previous_send = None, None
                for topic, host_ns, ros_ns, mode, message in node.handoff_frames:
                    if topic == '/v3/input_state':
                        state = json.loads(message.data)
                        identity = (state.get('held'), state.get('reason'))
                        if identity != previous_input:
                            input_transitions.append(dict(host_monotonic_ns=host_ns, **state))
                            previous_input = identity
                    elif topic == '/test/quest_udp_tx':
                        if previous_send is not None:
                            before_ns, before_message = previous_send
                            gap = (host_ns-before_ns)*1e-9
                            if gap > .02:
                                sender_gaps.append(dict(start_host_ns=before_ns, end_host_ns=host_ns,
                                    duration_s=gap, previous_grip=before_message['right']['grip'],
                                    next_grip=message['right']['grip'], previous_sequence=before_message['sequence'],
                                    next_sequence=message['sequence']))
                        previous_send = (host_ns, message)
                report['input_state_transitions'] = input_transitions
                report['all_sender_gaps_over_20ms_including_explicit_stops'] = sender_gaps
                report['capture'] = dict(path=str(capture_path), frames=len(node.handoff_frames),
                                          topic_counts=dict(counts), overflow_frames=node.handoff_overflow)
                report['final_write_guard'] = guard_audit_summary(audit)
                rows = [json.loads(line) for line in audit.read_text().splitlines() if line.strip()]
                writes = [row for row in rows if row.get('event') == 'write' and row.get('result') == 'accepted']
                for phase in report['phases']:
                    if not phase.get('stationary_checkpoint'):
                        continue
                    # Require exact final-command rest for >=80 ms, independent of
                    # joint-state subscriber arrival intervals or measured dynamics.
                    selected = [row for row in writes if phase['start_host_s']+.08 <=
                                row['host_monotonic_ns']/1e9 <= phase['end_host_s']]
                    if len(selected) < 11 or any(
                        max(abs(x-y) for x, y in zip(row['command'], selected[0]['command'])) > 1e-10
                        or max(abs(x) for x in row['guard_previous_velocity']) > 1e-8
                        or max(abs(x) for x in row['guard_previous_acceleration']) > 1e-7 for row in selected):
                        raise RuntimeError(phase['name']+' lacks stationary final-write evidence')
                    phase['stationary_final_writes'] = len(selected)
                    report['checks'].append(phase['name']+'_strict_command_hold')
                text = log_path.read_text(errors='replace')
                report['retiming_adapter_calls'] = len(re.findall(r'Retimed \d+ geometric points to', text))
                if case == 'pure_servo' and (report['retiming_adapter_calls'] or node.mode_counts.get('EXECUTING', 0)):
                    raise RuntimeError('Pure Servo case unexpectedly executed a plan')
                if case != 'pure_servo' and not report['retiming_adapter_calls']:
                    raise RuntimeError('No real path-retiming adapter invocation observed')
            except Exception as error:
                failure = failure or error
                report['audit_or_capture_failure'] = str(error)
            if failure is not None:
                report['failure'] = str(failure)
            report['passed'] = failure is None
            report['fully_clean_passed'] = report['passed'] and report['cleanup']['clean']
            report_path.write_text(json.dumps(report, indent=2)+'\n')
        finally:
            if gc_was_enabled:
                gc.enable()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=89,
                        help='Two fresh, otherwise unused domains: this ID and ID+1')
    parser.add_argument('--case', choices=('all', 'pure_servo', 'auto_replan'), default='all')
    parser.add_argument('--smooth-follow',action='store_true',help='Fake-only candidate; evidence stored separately from hardware baseline')
    args = parser.parse_args()
    if not 1 <= args.domain_id <= 230:
        parser.error('domain-id must be in 1..230')
    output = ROOT/'artifacts'/('smooth_follow_ros_test' if args.smooth_follow else 'fixed_servo_ros_test')
    output.mkdir(parents=True, exist_ok=True)
    cases = ('pure_servo', 'auto_replan') if args.case == 'all' else (args.case,)
    reports = []
    for case in cases:
        result = execute_session(case, args.domain_id+(case == 'auto_replan'), output/case, args.smooth_follow)
        reports.append(result)
        if not result['passed']:
            break
    summary = dict(passed=len(reports) == len(cases) and all(item['passed'] for item in reports),
        fully_clean_passed=len(reports) == len(cases) and all(item['fully_clean_passed'] for item in reports),
        physical_acceptance=False, results=reports,
        note='Functional and strict final-write results are separate from process shutdown cleanliness.')
    (output/'result.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    if not summary['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
