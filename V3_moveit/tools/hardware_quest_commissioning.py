#!/usr/bin/env python3
"""Short supervised live-Quest observation; defaults to strict SDK-free fake.

Source Jazzy and this D-drive install first. An already running Quest HTTPS relay
must deliver the live Touch input to the configured UDP port. Start with Grip
released, wait for READY, make a small motion, then release Grip and observe rest.
READY is an observer checkpoint, not a motion interlock: an early Grip may reach
the production stack before this observer detects it and requests shutdown.
No synthetic input, motion lease, trajectory, action goal or stop command is
published by this observer. The production stack owns all robot control.

--hardware requires the six current software/read-only evidence reports, a
current passed real hold with Servo exit confirmed, and a current successful
fake run of THIS tool. Ordinary hardware_readiness.json is never changed.
The launch window defaults to 30 seconds. Strict fake may request up to 90
seconds; hardware is capped at the user-authorized 60 seconds. It starts when launch is created
and triggers SIGINT. Neither that timer nor observed excursion thresholds
bound physical travel/stop time. Capture remains bounded: 75,000 frames for fake,
50,000 for hardware.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import gc
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
NAMES = [f'joint_{i}' for i in range(1, 7)]
MAX_SESSION_S = 60.  # User-authorized observation window; independent of fake allowance.
MAX_FAKE_SESSION_S = 90.
OBSERVER_RECEIPT_DEADLINE_S = .05  # Independent observer only; driver retains 24 ms.
# User explicitly authorized this observation range on 2026-11-13.
# These SIGINT thresholds are not hard travel/stop bounds. Production joint
# limits, velocity/acceleration/jerk, input freshness and collision guards remain.
JOINT_EXCURSION_RAD = math.radians(30.)
TCP_EXCURSION_M = 1.0
# Fake observation uses the same user-authorized observation range as hardware.
# These are observer exit thresholds, not production limits or motion commands.
FAKE_JOINT_EXCURSION_RAD = JOINT_EXCURSION_RAD
FAKE_TCP_EXCURSION_M = TCP_EXCURSION_M
QUIET_WINDOW_S = .08
MAX_FRAMES = 50000
MAX_FAKE_FRAMES = 75000


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def sha256(path):
    import hashlib
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_vector(values, size=6):
    out = [float(x) for x in values]
    require(len(out) == size and all(math.isfinite(x) for x in out), 'Incomplete/nonfinite observation')
    return out


def excursion(a, b):
    return max(abs(x-y) for x, y in zip(a, b))


def json_safe(value):
    # Before the first packet Clutch reports age_s=Infinity. Preserve that fact
    # as text in diagnostics; never let serialization interrupt ROS cleanup.
    if isinstance(value, float) and not math.isfinite(value):return repr(value)
    if isinstance(value, dict):return {key:json_safe(item) for key,item in value.items()}
    if isinstance(value, (list, tuple)):return [json_safe(item) for item in value]
    return value


def commissioning_prerequisites(root, digest, hardware):
    from check_ros_pass import validate_reports, validate_guard, SIMULATION_REPORTS
    from hardware_hold_check import hardware_prerequisites, validate_site
    reports = hardware_prerequisites(root, digest) if hardware else {
        name: sha256(root/name) for name in validate_reports(root, digest)}
    site = validate_site(root) if hardware else dict(path=str(root/'site_ubuntu.yaml'),
                                                    sha256=sha256(root/'site_ubuntu.yaml'))
    if hardware:
        relative = 'artifacts/hardware_hold_check.json'
        record = json.loads((root/relative).read_text())
        require(record.get('passed') is True and not record.get('failure') and
                record.get('mode') == 'hardware' and record.get('package_sha256') == digest and
                record.get('hardware_backend') == 'v3_moveit/JakaSystem' and
                record.get('script_sha256') == sha256(root/'tools/hardware_hold_check.py'),
                'Current real hardware hold is not passed for this driver/tool')
        require(record.get('servo_exit_confirmed') is True and record.get('launch_exit_code') == 0 and
                not record.get('cleanup_unconfirmed_live_pid') and record.get('observation_elapsed_s', 0) >= 12.,
                'Real hold duration/Servo exit/cleanup incomplete')
        require(record.get('prerequisite_reports') == reports and record.get('site', {}).get('sha256') == site['sha256'],
                'Real hold prerequisites/site no longer match')
        phases = record.get('log_phases', {})
        require(phases.get('runtime_faults') == [] and phases.get('unexpected_cleanup_errors') == [],
                'Real hold logged a runtime or unexpected cleanup fault')
        raw = record.get('feedback_log', {})
        path = Path(raw.get('path', '')).resolve()
        require(path.is_relative_to(root.resolve()) and path.is_file() and sha256(path) == raw.get('sha256'),
                'Real hold raw feedback identity missing/changed')
        require('Servo exit confirmed' in (root/'artifacts/hardware_hold_check.log').read_text(),
                'Real hold Servo exit log missing')
        reports[relative] = sha256(root/relative)
        relative = 'artifacts/quest_commissioning_fake/result.json'
        fake = json.loads((root/relative).read_text())
        require(fake.get('passed') is True and fake.get('mode') == 'fake' and
                fake.get('package_sha256') == digest and fake.get('script_sha256') == sha256(Path(__file__)) and
                fake.get('physical_acceptance') is False and fake.get('auto_replan') is False,
                'This commissioning tool lacks a current passed live-Quest fake run')
        require(fake.get('hardware_backend') == 'v3_moveit/GuardedGenericSystem' and
                fake.get('supervised_motion_observed') is True and fake.get('release_stop_observed') is True and
                fake.get('observed_faults') == [] and not fake.get('failure') and fake.get('launch_exit_code') == 0 and
                not fake.get('cleanup_unconfirmed_live_pid') and
                all(fake.get(key) == 0 for key in ('control_publishers_created','udp_packets_sent','action_goals_sent')),
                'Commissioning fake behavior/ownership/cleanup evidence incomplete')
        require(fake.get('site', {}).get('sha256') == site['sha256'],
                'Commissioning fake used a different site file')
        import yaml
        expected_override = yaml.safe_load((root/'site_ubuntu.yaml').read_text())
        expected_override['auto_replan'] = False
        override = fake.get('runtime_override', {})
        require(override.get('values') == expected_override, 'Commissioning fake runtime/input mapping differs')
        require(fake.get('prerequisite_reports') == {name:reports[name] for name,_ in SIMULATION_REPORTS},
                'Commissioning fake software evidence changed')
        for label, record in (('capture', fake.get('capture', {})), ('runtime override', override)):
            path=Path(record.get('path', '')).resolve()
            require(path.is_relative_to(root.resolve()) and path.is_file() and sha256(path)==record.get('sha256'),
                    'Commissioning fake '+label+' identity missing/changed')
        require(type(fake['capture'].get('frames')) is int and 0 < fake['capture']['frames'] <= MAX_FAKE_FRAMES and
                fake.get('capture_frame_limit') == MAX_FAKE_FRAMES,
                'Commissioning fake capture missing/overflowed')
        require(fake.get('observation_thresholds') == dict(joint_excursion_rad=FAKE_JOINT_EXCURSION_RAD,
                tcp_excursion_m=FAKE_TCP_EXCURSION_M, receipt_s=OBSERVER_RECEIPT_DEADLINE_S),
                'Commissioning fake observer limits differ from current mode policy')
        validate_guard(fake.get('final_write_guard'))
        reports[relative] = sha256(root/relative)
    return reports, site


def launch_source(root, hardware, runtime, audit, rviz):
    # Execute the complete production bringup, preserving its own core-exit
    # handlers. Large launch escalation timeouts prevent forced SDK-owner death.
    config = dict(mode='hardware' if hardware else 'fake', guarded_fake='false' if hardware else 'true',
                  guard_audit_path=str(audit), guard_strict='true', runtime_config=str(runtime),
                  payload_config=str(root/'src/v3_moveit/config/payload.json'),
                  rviz=str(rviz).lower(), sigterm_timeout='86400', sigkill_timeout='86400')
    return '''import importlib.util
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions

def full(context):
    context.launch_configurations.update(%r)
    spec=importlib.util.spec_from_file_location('commissioning_bringup',%r)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    actions=module.setup(context)
    def resolve(value):
        return perform_substitutions(context,normalize_to_list_of_substitutions(value))
    actual=[(resolve(n.node_package),resolve(n.node_executable)) for n in actions if isinstance(n,Node)]
    expected=[('robot_state_publisher','robot_state_publisher'),('controller_manager','ros2_control_node'),
        ('controller_manager','spawner'),('controller_manager','spawner'),('moveit_ros_move_group','move_group'),
        ('v3_moveit','v3_fixed_servo_node'),('v3_moveit','supervisor'),('v3_moveit','scene_loader'),('v3_moveit','quest_bridge')]
    if %r:expected.append(('rviz2','rviz2'))
    if actual!=expected:raise RuntimeError('Production bringup process contract changed')
    return actions

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=full)])
''' % (config, str(root/'src/v3_moveit/launch/bringup.launch.py'), rviz)


class Observation:
    """Bounded subscriber state; never used to generate or modify a command."""
    def __init__(self, lower, upper, velocity_limit, frame_limit=MAX_FRAMES, hardware=True):
        require(type(frame_limit) is int and 1 <= frame_limit <= MAX_FAKE_FRAMES, 'Invalid bounded capture limit')
        require(type(hardware) is bool, 'Observer hardware mode must be an explicit bool')
        self.frame_limit = frame_limit
        self.joint_excursion_limit = JOINT_EXCURSION_RAD if hardware else FAKE_JOINT_EXCURSION_RAD
        self.tcp_excursion_limit = TCP_EXCURSION_M if hardware else FAKE_TCP_EXCURSION_M
        self.lower, self.upper, self.velocity_limit = lower, upper, velocity_limit
        self.frames = []; self.counts = Counter(); self.faults = []
        self.q = self.v = self.q0 = self.tcp0 = self.command_q = None
        self.joint_at = self.command_at = self.tcp_at = None
        self.physical_quiet_since = self.command_quiet_since = None
        self.physical_anchor = self.command_anchor = None
        self.last_joint_stamp = self.last_command_stamp = None
        self.state = {}; self.input = {}; self.input_at = None
        self.first_released = False; self.ready = False; self.held_seen = False
        self.max_joint_excursion = self.max_tcp_excursion = self.max_velocity = 0.
        self.input_transitions = []; self.stop_events = []; self.active_stop = None
        self.mode_transitions = []; self.lease_counts = Counter()
        self.motion_episodes = []; self.active_motion = None

    def motion_observed(self):
        return any(self.episode_moved(episode) for episode in self.motion_episodes)

    @staticmethod
    def episode_moved(episode):
        return (episode['follow_seen'] and episode['maximum_command_motion_rad'] > 1e-4 and
                episode['maximum_measured_motion_rad'] > 1e-4)

    def released_stop_observed(self):
        return any(event['confirmed_stationary'] and not event.get('invalidated_after_confirmation', False) and
                   event['reason'] == 'RELEASED' and event.get('motion_episode_index') is not None and
                   self.episode_moved(self.motion_episodes[event['motion_episode_index']])
                   for event in self.stop_events)

    def invalidate_stop(self, reason, now):
        event = self.active_stop
        if event is not None and event['confirmed_stationary']:
            event.update(confirmed_stationary=False, invalidated_after_confirmation=True,
                         invalidation_host_s=now, invalidation_reason=reason)
            self.faults.append('Released stop no longer stationary: '+reason)

    def capture(self, topic, now, ros_ns, data):
        require(len(self.frames) < self.frame_limit, 'Observation capture capacity reached')
        self.frames.append(dict(topic=topic, host_monotonic_s=now, receipt_ros_ns=ros_ns, data=data))
        self.counts[topic] += 1

    def joints(self, names, q, v, stamp, ros_ns, now):
        require(len(names) == 6 and set(names) == set(NAMES), 'Unexpected joint feedback names')
        indices = [names.index(name) for name in NAMES]
        q, v = finite_vector(q), finite_vector(v)
        q, v = [q[i] for i in indices], [v[i] for i in indices]
        require(stamp > 0 and (self.last_joint_stamp is None or stamp > self.last_joint_stamp),
                'JointState source header stalled/regressed')
        age=(ros_ns-stamp)*1e-9
        require(0 <= age < OBSERVER_RECEIPT_DEADLINE_S, 'Observed JointState header stale or future')
        self.capture('/joint_states', now, ros_ns, dict(stamp_ns=stamp, header_age_s=age, q=q, velocity=v))
        self.last_joint_stamp = stamp
        if self.q0 is None: self.q0 = q.copy()
        delta, speed = excursion(q, self.q0), max(abs(x) for x in v)
        self.max_joint_excursion = max(self.max_joint_excursion, delta); self.max_velocity = max(self.max_velocity, speed)
        require(all(lo <= x <= hi for lo, x, hi in zip(self.lower, q, self.upper)), 'Observed physical joint position outside model limits')
        require(delta <= self.joint_excursion_limit, 'Observed joint excursion exceeded commissioning exit threshold')
        require(speed <= self.velocity_limit+.005, 'Observed reported joint velocity exceeded commissioning exit threshold')
        uninterrupted = self.joint_at is not None and 0 <= now-self.joint_at < OBSERVER_RECEIPT_DEADLINE_S
        if speed <= .005:
            if not uninterrupted or self.physical_anchor is None or excursion(q, self.physical_anchor) > .0005:
                self.physical_quiet_since = now; self.physical_anchor = q.copy()
        else: self.physical_quiet_since = None; self.physical_anchor = None
        self.q, self.v, self.joint_at = q, v, now
        if self.active_motion is not None and self.input.get('held'):
            self.active_motion['maximum_measured_motion_rad'] = max(
                self.active_motion['maximum_measured_motion_rad'], excursion(q, self.active_motion['q_at_grip']))
        if self.active_stop is not None:
            self.active_stop['maximum_joint_motion_after_edge_rad'] = max(
                self.active_stop['maximum_joint_motion_after_edge_rad'], excursion(q, self.active_stop['q_at_edge']))
            if self.active_stop['confirmed_stationary'] and (speed > .005 or
                    excursion(q, self.active_stop['q_at_confirmation']) > .0005):
                self.invalidate_stop('measured velocity or displacement after confirmation', now)

    def command(self, names, output, reference, velocity, acceleration, stamp, ros_ns, now):
        require(len(names) == 6 and set(names) == set(NAMES), 'Unexpected command-state names')
        indices = [names.index(name) for name in NAMES]
        fields = [finite_vector(values) for values in (output, reference, velocity, acceleration)]
        q, ref, v, a = [[values[i] for i in indices] for values in fields]
        require(stamp > 0 and (self.last_command_stamp is None or stamp > self.last_command_stamp), 'Command target header stalled/regressed')
        require(-.024 <= (ros_ns-stamp)*1e-9 < OBSERVER_RECEIPT_DEADLINE_S, 'Observed command target header stale/future')
        self.capture('/arm_controller/controller_state', now, ros_ns,
                     dict(target_stamp_ns=stamp, target_header_age_s=(ros_ns-stamp)*1e-9, output_q=q, reference_q=ref, reference_v=v, reference_a=a))
        self.last_command_stamp = stamp
        quiet = excursion(q, ref) <= 1e-10 and max(abs(x) for x in v+a) <= 1e-8
        uninterrupted = self.command_at is not None and 0 <= now-self.command_at < OBSERVER_RECEIPT_DEADLINE_S
        if quiet:
            if not uninterrupted or self.command_anchor is None or excursion(q, self.command_anchor) > 1e-10:
                self.command_quiet_since = now; self.command_anchor = q.copy()
        else: self.command_quiet_since = None; self.command_anchor = None
        self.command_q, self.command_at = q, now
        if self.active_motion is not None and self.input.get('held'):
            self.active_motion['maximum_command_motion_rad'] = max(
                self.active_motion['maximum_command_motion_rad'], excursion(q, self.active_motion['command_at_grip']))
        if self.active_stop is not None and self.active_stop['confirmed_stationary'] and (
                not quiet or excursion(q, self.active_stop['command_at_confirmation']) > 1e-10):
            self.invalidate_stop('command moved after confirmation', now)

    def inputs(self, data, ros_ns, now):
        require(type(data.get('held')) is bool and type(data.get('software_stop')) is bool, 'Malformed input state')
        old_held = self.input.get('held') is True
        identity = (data['held'], data.get('reason'), data['software_stop'])
        if not self.input_transitions or identity != self.input_transitions[-1]['identity']:
            self.input_transitions.append(dict(host_monotonic_s=now, identity=identity, state=data.copy()))
        self.capture('/v3/input_state', now, ros_ns, data.copy())
        self.input, self.input_at = data, now
        if data.get('reason') == 'RELEASED' and data['held'] is False and data.get('age_s', 999) <= .08:
            self.first_released = True
        if data['held']:
            require(self.first_released and self.ready, 'Grip held before released-input and stationary READY checkpoint')
            self.held_seen = True
            if not old_held:
                require(self.q is not None and self.command_q is not None, 'Grip episode lacks measured/command baselines')
                if self.active_stop is not None:
                    self.active_stop.update(observation_end_host_s=now, observation_end_reason='next_grip')
                    self.active_stop = None
                episode = dict(index=len(self.motion_episodes), grip_host_s=now,
                    q_at_grip=self.q.copy(), command_at_grip=self.command_q.copy(),
                    maximum_measured_motion_rad=0., maximum_command_motion_rad=0., follow_seen=False)
                self.motion_episodes.append(episode); self.active_motion = episode
        if old_held and not data['held'] and self.q is not None:
            if self.active_stop is not None:self.active_stop['superseded_by_new_stop_edge'] = True
            episode_index = self.active_motion['index'] if self.active_motion is not None else None
            if self.active_motion is not None:
                self.active_motion.update(release_host_s=now, motion_observed=self.episode_moved(self.active_motion))
                self.active_motion = None
            event = dict(reason=data.get('reason'), software_stop=data['software_stop'], edge_host_s=now,
                         motion_episode_index=episode_index,
                         q_at_edge=self.q.copy(), maximum_joint_motion_after_edge_rad=0., confirmed_stationary=False)
            self.stop_events.append(event); self.active_stop = event
            # Quiet time before release cannot establish rest after release: the
            # production trajectory may still contain its explicit 32 ms horizon.
            self.physical_quiet_since = self.command_quiet_since = None
            self.physical_anchor = self.command_anchor = None

    def status(self, data, ros_ns, now):
        self.capture('/v3/state', now, ros_ns, data.copy()); self.state = data
        mode = data.get('mode')
        if mode == 'FOLLOW' and self.active_motion is not None and self.input.get('held'):
            self.active_motion['follow_seen'] = True
        if not self.mode_transitions or self.mode_transitions[-1]['mode'] != mode:
            self.mode_transitions.append(dict(mode=mode, host_monotonic_s=now))
        require(mode != 'FAULT', 'Production supervisor entered FAULT')
        require(data.get('auto_replan') is False, 'Automatic planning is enabled during first commissioning')
        require(mode not in ('PLANNING', 'EXECUTING', 'PAUSING', 'CANCELING'), 'Unexpected automatic plan/action during Servo-only commissioning')

    def tcp(self, xyz, xyzw, stamp, ros_ns, now):
        xyz, xyzw = finite_vector(xyz, 3), finite_vector(xyzw, 4)
        age=(ros_ns-stamp)*1e-9
        require(stamp > 0 and -.024 <= age < OBSERVER_RECEIPT_DEADLINE_S, 'Observed TCP transform is stale')
        self.capture('/observed_tcp_tf', now, ros_ns, dict(stamp_ns=stamp, header_age_s=age, xyz_m=xyz, xyzw=xyzw))
        if self.tcp0 is None:self.tcp0 = xyz.copy()
        delta = math.sqrt(sum((a-b)**2 for a,b in zip(xyz, self.tcp0)))
        self.max_tcp_excursion = max(self.max_tcp_excursion, delta); self.tcp_at = now
        require(delta <= self.tcp_excursion_limit, 'Observed TCP excursion exceeded commissioning exit threshold')

    def tick(self, now):
        require(not self.faults, 'Production/observer fault: '+str(self.faults[:3]))
        for name, at in (('joint feedback', self.joint_at), ('command state', self.command_at)):
            if at is not None and not 0 <= now-at < OBSERVER_RECEIPT_DEADLINE_S:
                self.invalidate_stop(name+' receipt expired after confirmation', now)
                raise RuntimeError(name+' receipt expired in observer')
        quiet = (self.physical_quiet_since is not None and self.command_quiet_since is not None and
                 now-self.physical_quiet_since >= QUIET_WINDOW_S and now-self.command_quiet_since >= QUIET_WINDOW_S)
        if not self.ready and quiet and self.first_released and self.input.get('held') is False and self.input.get('reason') == 'RELEASED' and self.input.get('age_s', 999) <= .08 and self.input_at is not None and now-self.input_at < .08 and self.state.get('mode') == 'IDLE' and self.state.get('scene_ready') is True and self.tcp_at is not None:
            self.ready = True
            return 'ready'
        if (self.active_stop is not None and not self.input.get('held') and quiet and
                not self.active_stop['confirmed_stationary'] and not self.active_stop.get('invalidated_after_confirmation', False)):
            self.active_stop.update(confirmed_stationary=True, confirmation_host_s=now,
                q_at_confirmation=self.q.copy(), command_at_confirmation=self.command_q.copy(),
                observed_confirmation_delay_s=now-self.active_stop['edge_host_s'],
                note='Includes 80ms quiet confirmation, DDS/observer delay and production queued trajectory; not a physical stop bound.')
        return None


def log_findings(text, cleanup=False):
    errors = [line for line in text.splitlines() if any(marker in line for marker in
        ('[ERROR]', '[FATAL]', 'Latched stop:', 'STOP NOT CONFIRMED', 'Hardware shutdown unconfirmed', 'process has died'))]
    known = []
    for line in errors:
        if not cleanup and line.startswith('[move_group-') and (
            '[move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates' in line or
            '[move_group.moveit.moveit.plugins.simple_controller_manager]: No controller_names specified.' in line): known.append(line)
        elif cleanup and (('[move_group-' in line and 'process has died' in line and 'exit code -11' in line) or
            (line.startswith('[ros2_control_node-') and '[controller_manager.pal_statistics]:' in line and
             line.endswith("Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!"))): known.append(line)
    return dict(all_error_lines=errors, known_configuration_or_cleanup_lines=known, unexpected=[line for line in errors if line not in known])


def validate_session_window(hardware=False, domain_id=90, duration=30.):
    """Pure gate; reject invalid windows before imports, files, or ROS startup."""
    maximum = MAX_SESSION_S if hardware else MAX_FAKE_SESSION_S
    require(1 <= domain_id <= 101 and math.isfinite(duration) and 1 <= duration <= maximum,
            f'Domain must be 1..101 and {"hardware" if hardware else "fake"} duration 1..{maximum:g} seconds')
    return dict(maximum_session_s=maximum, capture_frame_limit=MAX_FRAMES if hardware else MAX_FAKE_FRAMES)


def run(hardware=False, domain_id=90, duration=30., rviz=False):
    session_policy = validate_session_window(hardware, domain_id, duration)
    from verify_offline import source_digest
    import yaml
    digest = source_digest(); mode = 'hardware' if hardware else 'fake'
    output = ROOT/'artifacts'/('quest_commissioning_'+mode); output.mkdir(parents=True, exist_ok=True)
    if (output/'result.json').exists():
        output = output/datetime.now(timezone.utc).strftime('previous_%Y%m%dT%H%M%S_%fZ')
        output.mkdir()
        for old in output.parent.iterdir():
            if old.is_file():old.rename(output/old.name)
        output = output.parent
    work = ROOT/'.tmp'/('quest_commissioning_'+mode); work.mkdir(parents=True, exist_ok=True)
    report = dict(schema_version=1, passed=False, mode=mode, package_sha256=digest,
        script_sha256=sha256(Path(__file__)), generated_at_utc=datetime.now(timezone.utc).isoformat(),
        hardware_backend='v3_moveit/JakaSystem' if hardware else 'v3_moveit/GuardedGenericSystem',
        auto_replan=False, duration_limit_s=duration, maximum_session_s=session_policy['maximum_session_s'],
        capture_frame_limit=session_policy['capture_frame_limit'],
        session_window_caps_s=dict(fake=MAX_FAKE_SESSION_S, hardware=MAX_SESSION_S), domain_id=domain_id,
        physical_acceptance=False, teleoperation_ready=False, ordinary_hardware_readiness_modified=False,
        control_publishers_created=0, udp_packets_sent=0, action_goals_sent=0, hardware_launch_started=False,
        cleanup_signals=[], observation_thresholds=dict(joint_excursion_rad=JOINT_EXCURSION_RAD if hardware else FAKE_JOINT_EXCURSION_RAD,
            tcp_excursion_m=TCP_EXCURSION_M if hardware else FAKE_TCP_EXCURSION_M, receipt_s=OBSERVER_RECEIPT_DEADLINE_S),
        limitations=['READY is an observer checkpoint, not a motion interlock; early Grip can reach production before observer-triggered shutdown.',
            'Observer timer/excursion thresholds request SIGINT; they are not hard travel or physical stop bounds.',
            'Stop observations include input/DDS/observer/quiet-window delay; no deterministic stop limit is established.',
            'JTC output is target data, separate from measured JointState feedback.',
            'Only the configured static scene is available; this does not add a live obstacle sensor.'])
    if hardware:
        from verify_offline import calibration_digest
        report['calibration_sha256']=calibration_digest(ROOT)
    result = output/'result.json'; result.write_text(json.dumps(report, indent=2))
    node = process = log = reader = ros = None; failure = None; launched_at = None; cleanup_offset = 0
    gc_enabled = gc.isenabled(); prior_handlers = {}; signal_requested = [False]
    try:
        report['prerequisite_reports'], report['site'] = commissioning_prerequisites(ROOT, digest, hardware)
        cfg = yaml.safe_load((ROOT/'src/v3_moveit/config/runtime.yaml').read_text())
        local = yaml.safe_load((ROOT/'site_ubuntu.yaml').read_text()); cfg.update(local)
        # Retain all production limits. Only the already supported auto_replan switch changes.
        local['auto_replan'] = False
        runtime = work/'runtime.yaml'; runtime.write_text(yaml.safe_dump(local))
        report['runtime_override'] = dict(path=str(runtime), sha256=sha256(runtime), values=local)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:reserve.bind((cfg['quest_bind'], int(cfg['quest_port'])))
        for key, relative in {'ROS_LOG_DIR':'log/ros', 'ROS_HOME':'.ros', 'COLCON_HOME':'.colcon',
                              'TMPDIR':'.tmp', 'XDG_CACHE_HOME':'.cache'}.items():
            path = ROOT/relative; path.mkdir(exist_ok=True); os.environ[key] = str(path)
        os.environ.update(ROS_DOMAIN_ID=str(domain_id), ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                          ROS_STATIC_PEERS='', PYTHONDONTWRITEBYTECODE='1', V3_EDG_AUDIT_PATH=str(output/'edg_audit.jsonl'))
        import rclpy
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import JointState
        from control_msgs.msg import JointTrajectoryControllerState
        from std_msgs.msg import String, Bool
        from tf2_ros import Buffer, TransformListener, TransformException
        from ament_index_python.packages import get_package_prefix
        require(Path(get_package_prefix('v3_moveit')).resolve().is_relative_to((ROOT/'install').resolve()), 'Wrong ROS package install')
        meta = json.loads((ROOT/'src/v3_moveit/config/model_metadata.json').read_text())
        observation = Observation(meta['joint_lower'], meta['joint_upper'], float(cfg['joint_velocity_rad_s']),
                                  frame_limit=session_policy['capture_frame_limit'], hardware=hardware)
        ros = rclpy
        class Probe(Node):
            def __init__(self):
                super().__init__('v3_quest_commissioning_observer'); self.obs = observation
                self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
                self.create_subscription(JointState, '/joint_states', self.joints, 10)
                self.create_subscription(JointTrajectoryControllerState, '/arm_controller/controller_state', self.command, 10)
                self.create_subscription(String, '/v3/input_state', lambda m:self.safe(self.obs.inputs, json.loads(m.data)), 10)
                self.create_subscription(String, '/v3/state', lambda m:self.safe(self.obs.status, json.loads(m.data)), 10)
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(String, '/v3/controller_fault', lambda m:self.obs.faults.append(m.data), qos)
                self.create_subscription(Bool, '/v3/motion_lease', lambda m:self.obs.lease_counts.update([str(m.data)]), 10)
            def safe(self, callback, *args):
                try:callback(*args, self.get_clock().now().nanoseconds, time.monotonic())
                except Exception as error:self.obs.faults.append(str(error))
            def joints(self, m):self.safe(self.obs.joints, list(m.name), list(m.position), list(m.velocity), m.header.stamp.sec*10**9+m.header.stamp.nanosec)
            def command(self, m):self.safe(self.obs.command, list(m.joint_names), list(m.output.positions), list(m.reference.positions), list(m.reference.velocities), list(m.reference.accelerations), m.header.stamp.sec*10**9+m.header.stamp.nanosec)
        ros.init(signal_handler_options=SignalHandlerOptions.NO); node = Probe(); gc.disable()
        for sig in (signal.SIGINT, signal.SIGTERM):
            prior_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, lambda signum, frame:signal_requested.__setitem__(0, True))
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:ros.spin_once(node, timeout_sec=.02)
        other = [(n, ns) for n, ns in node.get_node_names_and_namespaces() if (n, ns) != (node.get_name(), node.get_namespace())]
        require(not other, f'DDS domain {domain_id} already occupied: {other}')
        for topic in ('/v3/motion_lease', '/arm_controller/joint_trajectory', '/joint_states'):
            require(node.count_publishers(topic) == 0, 'Preexisting control/feedback publisher: '+topic)
        require(not signal_requested[0], 'Interrupted before launch')
        audit = output/'guard.jsonl'; launch = work/'full.launch.py'
        launch.write_text(launch_source(ROOT, hardware, runtime, audit, rviz))
        log_path = output/'ros.log'; log = log_path.open('w'); reader = log_path.open('r')
        process = subprocess.Popen(['ros2', 'launch', str(launch)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        launched_at = time.monotonic(); deadline = launched_at+duration
        report.update(launch_pid=process.pid, hardware_launch_started=hardware, launch_host_monotonic_s=launched_at)
        result.write_text(json.dumps(report, indent=2))
        print(f'{mode.upper()} Quest commissioning: release Grip; total launch window {duration:g}s; wait for READY.', flush=True)
        last_tf = last_log = last_graph = 0.; pending_log = ''
        while time.monotonic() < deadline and not signal_requested[0]:
            require(process.poll() is None, 'Production launch exited during observation')
            ros.spin_once(node, timeout_sec=.001); now = time.monotonic()
            if now-last_tf >= .02:
                last_tf = now
                try:
                    tf = node.tf.lookup_transform('base_link', 'tcp', ros.time.Time())
                    p, q = tf.transform.translation, tf.transform.rotation
                    stamp = tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
                    observation.tcp([p.x,p.y,p.z], [q.x,q.y,q.z,q.w], stamp, node.get_clock().now().nanoseconds, now)
                except TransformException:
                    if observation.ready:raise RuntimeError('TCP transform unavailable after READY')
            if observation.tick(now) == 'ready':print('READY: Grip has been released and commanded/measured rest observed. Small supervised motion only; release Grip to stop.', flush=True)
            if now-last_graph >= .25:
                last_graph = now
                for topic in ('/v3/motion_lease', '/arm_controller/joint_trajectory', '/v3/input_state', '/joint_states'):
                    publishers=node.count_publishers(topic)
                    require(publishers <= 1 and (not observation.ready or publishers == 1), 'Unexpected publisher count on '+topic)
            if now-last_log >= .05:
                last_log = now; pending_log += reader.read()
                complete, separator, pending_log = pending_log.rpartition('\n')
                if not separator:complete=''
                findings = log_findings(complete)
                require(not findings['unexpected'], 'Runtime fault: '+str(findings['unexpected'][:2]))
        report['session_elapsed_before_cleanup_s'] = time.monotonic()-launched_at
        report['exit_trigger'] = 'operator_signal' if signal_requested[0] else 'duration_limit'
    except (Exception, KeyboardInterrupt) as error:failure = str(error) or 'Interrupted'
    finally:
        if process is not None:
            cleanup_offset = (output/'ros.log').stat().st_size
            report['cleanup_requested_host_s'] = time.monotonic()
            try:
                if process.poll() is None:process.send_signal(signal.SIGINT); report['cleanup_signals'].append('SIGINT')
                process.wait(timeout=30.)
                report['launch_exit_code'] = process.returncode
                if process.returncode != 0:failure = failure or 'Launch returned nonzero during cleanup'
            except subprocess.TimeoutExpired:
                report['cleanup_unconfirmed_live_pid'] = process.pid
                failure = failure or 'Cleanup exceeded 30s; SDK-owner process retained; Servo exit unconfirmed'
                print('STOP UNCONFIRMED: retained launch PID '+str(process.pid), flush=True)
            except Exception as error:failure = failure or str(error)
        if reader is not None:reader.close()
        if log is not None:log.close()
        if node is not None:
            obs = node.obs
            if obs.active_stop is not None:
                obs.active_stop.update(observation_end_host_s=report.get('cleanup_requested_host_s', time.monotonic()),
                    observation_end_reason='observation_end', monitoring_continues_during_cleanup=False)
            report.update(ready_seen=obs.ready, grip_held_seen=obs.held_seen, input_transitions=obs.input_transitions,
                stop_observations=obs.stop_events, motion_episodes=obs.motion_episodes,
                supervisor_modes=obs.mode_transitions, lease_counts=dict(obs.lease_counts),
                maximum_joint_excursion_rad=obs.max_joint_excursion, maximum_tcp_excursion_m=obs.max_tcp_excursion,
                maximum_reported_joint_velocity_rad_s=obs.max_velocity, observed_faults=obs.faults)
            raw = output/'capture.jsonl'
            try:
                raw.write_text(''.join(json.dumps(json_safe(row), allow_nan=False)+'\n' for row in obs.frames))
                report['capture'] = dict(path=str(raw), sha256=sha256(raw), frames=len(obs.frames), frame_limit=obs.frame_limit, topic_counts=dict(obs.counts))
            except Exception as error:failure = failure or 'Capture serialization failed: '+str(error)
            report['supervised_motion_observed'] = obs.motion_observed()
            report['release_stop_observed'] = obs.released_stop_observed()
            if not report['supervised_motion_observed'] or not report['release_stop_observed']:
                failure = failure or 'Required small motion and released-input stationary observation incomplete'
            node.destroy_node()
        if ros is not None and ros.ok():ros.shutdown()
        for sig, handler in prior_handlers.items():signal.signal(sig, handler)
        gc.enable() if gc_enabled else gc.disable()
        text = (output/'ros.log').read_text(errors='replace') if process is not None else ''
        data = (output/'ros.log').read_bytes() if process is not None else b''
        report['runtime_log'] = log_findings(data[:cleanup_offset].decode(errors='replace'))
        report['cleanup_log'] = log_findings(data[cleanup_offset:].decode(errors='replace'), cleanup=True)
        if report['runtime_log']['unexpected'] or report['cleanup_log']['unexpected']:failure = failure or 'Unexpected runtime/cleanup log failure'
        report['servo_exit_confirmed'] = 'Servo exit confirmed' in text if hardware else None
        if hardware and process is not None and not report['servo_exit_confirmed']:failure = failure or 'Hardware Servo exit confirmation missing'
        if not hardware and process is not None:
            try:
                from ros_smoke import guard_audit_summary
                report['final_write_guard'] = guard_audit_summary(output/'guard.jsonl')
            except Exception as error:failure = failure or str(error)
        if source_digest() != digest:failure = failure or 'Project source changed during observation'
        if hardware:
            try:
                if calibration_digest(ROOT)!=report['calibration_sha256']:failure=failure or 'Load/TCP calibration changed during observation'
            except Exception as error:failure=failure or str(error)
        report['passed'] = failure is None
        report['fully_clean_passed'] = report['passed'] and not report['runtime_log']['all_error_lines'] and not report['cleanup_log']['all_error_lines']
        if failure:report['failure'] = failure
        result.write_text(json.dumps(json_safe(report), indent=2, allow_nan=False)); print(json.dumps(dict(passed=report['passed'], failure=failure, report=str(result)), ensure_ascii=False), flush=True)
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--domain-id', type=int, default=90)
    parser.add_argument('--duration', type=float, default=30., help='Total launch window including startup; default 30s, fake maximum 90s, hardware maximum 60s')
    parser.add_argument('--rviz', action='store_true')
    args = parser.parse_args()
    return run(args.hardware, args.domain_id, args.duration, args.rviz)


if __name__ == '__main__':raise SystemExit(main())
