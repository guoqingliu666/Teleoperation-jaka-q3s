#!/usr/bin/env python3
"""Prepare/run a bounded hold-only commissioning observation; default is fake.

Source /opt/ros/jazzy/setup.bash and this D-drive project's install/setup.bash.
This is a separate commissioning check, not the RUN_HARDWARE teleoperation gate.
--hardware enters servo mode and sends EDG absolute joint HOLD targets through
JakaSystem's existing never-leased activation-initial_ path. It is NOT a read-only
SDK check or a no-motion-API check. No lease/trajectory publisher or action client
is created here. Passing cannot accept physical braking or teleoperation.
"""
import argparse
from collections import deque
import gc
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
NAMES = [f'joint_{i}' for i in range(1, 7)]
OBSERVATION_S = 12.0
FEEDBACK_DEADLINE_S = .024
MAX_DRIFT_RAD = .0005
MAX_REPORTED_VELOCITY_RAD_S = .005
FAULT_MARKERS = ('[ERROR]', '[FATAL]', 'Latched stop:', 'STOP NOT CONFIRMED',
                 'Hardware shutdown unconfirmed', 'process has died', 'HOLD_CORE_PROCESS_EXIT')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def validate_simulation_record(record, digest, *, action_test=False, fixed_case=None):
    # One shared filesystem-only contract for RUN_HARDWARE and commissioning.
    from check_ros_pass import validate_simulation_record as validate
    validate(record, digest, fixed_case or ('action' if action_test else 'smoke'))


def verified_evidence(root, record):
    evidence = record.get('evidence', [])
    require(bool(evidence), 'Historical validation lacks evidence hashes')
    for entry in evidence:
        path = (root / entry['path']).resolve()
        require(path.is_relative_to(root.resolve()) and path.is_file(), 'Historical evidence missing/outside project')
        require(sha256(path) == entry.get('sha256'), f'Historical evidence changed: {entry["path"]}')
    return {entry['path'] for entry in evidence}


def hardware_prerequisites(root, digest):
    """Filesystem-only gate; never import or invoke the SDK or initialize ROS."""
    reports = {}
    from check_ros_pass import validate_reports
    for relative in validate_reports(root, digest):
        reports[relative] = sha256(root / relative)
    if (root/'artifacts/hardware_preflight_current.json').exists():
        from hardware_current_preflight import validate
        validate(root,digest)
        reports['artifacts/hardware_preflight_current.json']=sha256(root/'artifacts/hardware_preflight_current.json')
        return reports
    for relative, mode0 in (('artifacts/hardware_readonly_reanalysis.json', False),
                            ('artifacts/edg_mode0_reanalysis.json', True)):
        path = root / relative
        record = json.loads(path.read_text())
        checks = record.get('checks', {})
        require(record.get('measurement_checks_passed') is True and checks and all(v is True for v in checks.values()),
                f'Historical measurement checks not passed: {relative}')
        evidence = verified_evidence(root, record)
        provenance = record.get('provenance', {})
        require(record.get('reanalysis') is True and record.get('package_sha256') == digest and
                record.get('analysis_connected_hardware') is False and record.get('analysis_started_ros') is False and
                provenance.get('acquisition_repeated') is False and provenance.get('new_robot_connections') == 0 and
                provenance.get('new_sdk_calls') == 0 and record.get('physical_acceptance') is False,
                'Current offline reanalysis provenance is incomplete')
        original = provenance.get('original_report', {})
        expected_original = 'artifacts/edg_mode0_validation.json' if mode0 else 'artifacts/hardware_readonly_validation.json'
        require(original.get('path') == expected_original and expected_original in evidence,
                'Reanalysis is not tied to the preserved historical report')
        preserved = provenance.get('preserved_backup', {})
        require(preserved.get('sha256') == original.get('sha256'), 'Historical report backup identity changed')
        verified_evidence(root, {'evidence': [original, preserved, record.get('analyzer', {})]})
        require({'tools/reanalyze_hardware_readonly.py', 'tools/verify_offline.py'}.issubset(evidence),
                'Current reanalysis implementation identities are missing')
        if mode0:
            require(record.get('mode0_feedback_measurement_passed') is True and
                    record.get('execution', {}).get('mode') == 0 and
                    record.get('execution', {}).get('actually_executed') is True,
                    'Mode 0 feedback measurement was not completed')
        else:
            for key in ('tcp1_exact_float_value_match', 'payload_exact_float_value_match',
                        'all_fk_comparisons_within_existing_guard', 'initial_robot_ready_and_stationary_status'):
                require(checks.get(key) is True, f'Missing historical model/TCP/state check: {key}')
            require({'src/v3_moveit/config/payload.json', 'src/v3_moveit/config/model_metadata.json',
                     'src/v3_moveit/description/s5.urdf'}.issubset(evidence), 'Model/payload identity evidence incomplete')
        cleanup = record.get('cleanup', {})
        require(cleanup.get('edg_off_rc') == 0 and cleanup.get('servo_query_rc') == 0 and
                cleanup.get('servo_active') == 0 and cleanup.get('disconnect_rc') == 0,
                'Historical SDK session cleanup was not confirmed')
        reports[relative] = sha256(path)
    return reports


def validate_site(root):
    import yaml
    path = root / 'site_ubuntu.yaml'
    site = yaml.safe_load(path.read_text())
    base = yaml.safe_load((root / 'src/v3_moveit/config/runtime.yaml').read_text())
    for key in ('robot_ip', 'edg_client_ip'):
        require(isinstance(ipaddress.ip_address(site[key]), ipaddress.IPv4Address), f'{key} must be IPv4')
    require(site['robot_ip'] == base['robot_ip'], 'site_ubuntu robot_ip differs from the validated project target')
    require(site['edg_port'] == base['edg_port'] and 1024 <= site['edg_port'] <= 65535, 'Unexpected EDG port')
    try:
        routes = json.loads(subprocess.check_output(['ip', '-j', '-4', 'route', 'get', site['robot_ip']], text=True, stderr=subprocess.PIPE))
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"未找到直连 {site['robot_ip']} 的有线网卡，请检查机械臂网线和有线路由") from exc
    route = routes[0] if routes else {}
    interface = route.get('dev', '')
    device = Path('/sys/class/net') / interface
    require(interface and (device/'device').exists() and not (device/'wireless').exists() and
            not route.get('gateway'), f"未找到直连 {site['robot_ip']} 的有线网卡，请检查机械臂网线和有线路由")
    addresses = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr', 'show', 'dev', interface], text=True))
    source = route.get('prefsrc')
    require(any('LOWER_UP' in item.get('flags', []) and any(
        a.get('local') == source and a.get('scope') == 'global' for a in item.get('addr_info', [])
    ) for item in addresses), f"{interface} 未就绪，无法确认到 {site['robot_ip']} 的本机 IPv4 地址")
    require(site['edg_client_ip'] == source,
            f"到 {site['robot_ip']} 的有线网卡 {interface} 当前使用 {source}，请将 site_ubuntu.yaml 的 edg_client_ip 从 {site['edg_client_ip']} 改为 {source} 后重新检查")
    return dict(path=str(path), sha256=sha256(path), interface=interface, **site)


class FeedbackWindow:
    """Pure feedback validation. ROS header freshness is not SDK source latency."""
    def __init__(self):
        self.samples = []
        self.rejected_samples = []
        self.rejection_overflow = 0
        self.initial = None
        self.last_host = None
        self.last_stamp = None
        self.max_drift = 0.
        self.max_velocity = 0.
        self.max_age = 0.
        self.max_host_gap = 0.
        self.max_stamp_gap = 0.

    def ingest(self, names, positions, velocities, stamp_ns, ros_now_ns, host_now):
        require(len(names) == 6 and set(names) == set(NAMES), 'Unexpected/duplicate joint names')
        require(len(positions) == 6 and len(velocities) == 6, 'Incomplete joint position/velocity feedback')
        indices = [names.index(name) for name in NAMES]
        q = [float(positions[i]) for i in indices]
        v = [float(velocities[i]) for i in indices]
        require(all(math.isfinite(x) for x in q + v), 'Nonfinite joint feedback')
        age = (ros_now_ns - stamp_ns) / 1e9
        require(stamp_ns > 0 and 0 <= age < FEEDBACK_DEADLINE_S,
                f'JointState header is stale or in the future: age_s={age:.12g}, stamp_ns={stamp_ns}, ros_now_ns={ros_now_ns}, receipt_host_s={host_now:.12g}')
        if self.last_stamp is not None:
            gap = (stamp_ns - self.last_stamp) / 1e9
            require(0 < gap < FEEDBACK_DEADLINE_S,
                    f'JointState stamp stalled/regressed or exceeded 24 ms: gap_s={gap:.12g}, previous_stamp_ns={self.last_stamp}, stamp_ns={stamp_ns}, age_s={age:.12g}')
            host_gap = host_now - self.last_host
            require(0 <= host_gap < FEEDBACK_DEADLINE_S,
                    f'New JointState receipt exceeded 24 ms: gap_s={host_gap:.12g}, previous_host_s={self.last_host:.12g}, receipt_host_s={host_now:.12g}, age_s={age:.12g}')
            self.max_stamp_gap = max(self.max_stamp_gap, gap)
            self.max_host_gap = max(self.max_host_gap, host_gap)
        if self.initial is None:
            self.initial = q.copy()
        drift = max(abs(a-b) for a, b in zip(q, self.initial))
        velocity = max(abs(x) for x in v)
        self.max_drift = max(self.max_drift, drift)
        self.max_velocity = max(self.max_velocity, velocity)
        self.max_age = max(self.max_age, age)
        require(drift <= MAX_DRIFT_RAD, 'Hold feedback drift exceeded 0.0005 rad')
        require(velocity <= MAX_REPORTED_VELOCITY_RAD_S, 'Hold reported velocity exceeded 0.005 rad/s')
        require(len(self.samples)+len(self.rejected_samples) < 20000, 'Feedback audit capacity exceeded')
        self.samples.append(dict(accepted=True, host_monotonic_s=host_now, stamp_ns=stamp_ns,
                                 receipt_ros_ns=ros_now_ns, age_s=age, q=q, velocity=v))
        self.last_host = host_now
        self.last_stamp = stamp_ns

    def reject(self, names, positions, velocities, stamp_ns, ros_now_ns, host_ns, error, context):
        if len(self.samples)+len(self.rejected_samples) >= 20000:
            self.rejection_overflow += 1
            return
        clean = lambda values: [value if not isinstance(value, float) or math.isfinite(value) else repr(value)
                                for value in values]
        self.rejected_samples.append(dict(accepted=False, names=names, position=clean(positions),
            velocity=clean(velocities), stamp_ns=stamp_ns, receipt_ros_ns=ros_now_ns,
            receipt_host_monotonic_ns=host_ns, host_monotonic_s=host_ns*1e-9,
            age_s=(ros_now_ns-stamp_ns)*1e-9, last_accepted_stamp_ns=self.last_stamp,
            last_accepted_host_s=self.last_host, reason=str(error), step_context=dict(context)))

    def assert_fresh(self, host_now):
        require(self.last_host is not None and host_now-self.last_host < FEEDBACK_DEADLINE_S,
                f'Joint feedback receipt expired at 24 ms: age_s={None if self.last_host is None else host_now-self.last_host}, last_host_s={self.last_host}, now_host_s={host_now}')

    def summary(self):
        return dict(samples=len(self.samples), rejected_samples=len(self.rejected_samples),
                    rejected_sample_overflow=self.rejection_overflow, maximum_drift_from_first_rad=self.max_drift,
                    maximum_reported_velocity_rad_s=self.max_velocity,
                    maximum_message_age_s=self.max_age, maximum_receipt_gap_s=self.max_host_gap,
                    maximum_header_stamp_gap_s=self.max_stamp_gap,
                    freshness_limit_s=FEEDBACK_DEADLINE_S, drift_limit_rad=MAX_DRIFT_RAD,
                    source_note='ROS JointState stamps/receipts; driver independently enforces fresh EDG source samples')


def allowed_nodes(hardware=False):
    # Installed spawner.py creates Node('spawner_'+controller); the longer
    # ros2_control_controller_spawner_* string is only its pre-node logger.
    # hardware_interface creates 'jakas5' from this URDF's system name JakaS5.
    names = {'v3_hold_check', 'robot_state_publisher', 'controller_manager', 'jakas5',
             'joint_state_broadcaster', 'arm_controller',
             'spawner_joint_state_broadcaster', 'spawner_arm_controller'}
    if hardware:
        names.add('jaka_hardware_watchdog')
    return {(name, '/') for name in names}


def launch_source(root, hardware, audit):
    # Construct all actions through current bringup, but execute only the exact
    # four verified state publisher/manager/spawner actions. No control publisher.
    context = dict(mode='hardware' if hardware else 'fake', guarded_fake='false' if hardware else 'true',
                   guard_audit_path=str(audit), guard_strict='true', runtime_config=str(root/'site_ubuntu.yaml'),
                   payload_config=str(root/'src/v3_moveit/config/payload.json'),
                   rviz='false', sigterm_timeout='86400', sigkill_timeout='86400')
    return '''import importlib.util
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions

def minimal(context):
    context.launch_configurations.update(%r)
    spec=importlib.util.spec_from_file_location('hold_bringup',%r)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nodes=module.setup(context)[:4]
    expected=[('robot_state_publisher','robot_state_publisher'),('controller_manager','ros2_control_node'),
              ('controller_manager','spawner'),('controller_manager','spawner')]
    def resolve(value):
        return perform_substitutions(context,normalize_to_list_of_substitutions(value))
    actual=[(resolve(n.node_package),resolve(n.node_executable)) for n in nodes]
    if actual!=expected:raise RuntimeError('Bringup first-four process contract changed; refusing commissioning')
    events=[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='HOLD_CORE_PROCESS_EXIT'))])) for n in nodes[:2]]
    return nodes+events

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=minimal)])
''' % (context, str(root/'src/v3_moveit/launch/bringup.launch.py'))


def classify_log_phases(data, cleanup_offset, sigint_sent, launch_exit_code):
    """Keep teardown diagnostics without excusing runtime or hardware faults."""
    runtime = data[:cleanup_offset].decode(errors='replace')
    cleanup = data[cleanup_offset:].decode(errors='replace')
    fault_lines = lambda text: [line for line in text.splitlines() if any(marker in line for marker in FAULT_MARKERS)]
    runtime_faults, cleanup_faults = fault_lines(runtime), fault_lines(cleanup)
    def known_statistics(line):
        return (sigint_sent and launch_exit_code == 0 and line.startswith('[ros2_control_node-') and
                '[ERROR]' in line and '[FATAL]' not in line and '[controller_manager.pal_statistics]:' in line and
                line.endswith("Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!"))
    known = [line for line in cleanup_faults if known_statistics(line)]
    unexpected = [line for line in cleanup_faults if line not in known and 'HOLD_CORE_PROCESS_EXIT' not in line]
    return dict(runtime_faults=runtime_faults, cleanup_errors=cleanup_faults,
                known_statistics_shutdown_errors=known, unexpected_cleanup_errors=unexpected,
                clean=not runtime_faults and not cleanup_faults)


def run(hardware=False, domain_id=89):
    gc_was_enabled = gc.isenabled()
    try:
        return _run(hardware, domain_id)
    finally:
        # Restore even if setup, observation, serialization, or teardown raises.
        gc.enable() if gc_was_enabled else gc.disable()


def _run(hardware=False, domain_id=89):
    require(1 <= domain_id <= 101, 'Use a dedicated ROS domain in 1..101')
    from verify_offline import source_digest
    digest = source_digest()
    basename = 'hardware_hold_check' if hardware else 'fake_hold_check'
    work = ROOT / '.tmp' / basename
    artifacts = ROOT / 'artifacts'
    work.mkdir(parents=True, exist_ok=True)
    artifacts.mkdir(exist_ok=True)
    path = artifacts / (basename+'.json')
    report = dict(schema_version=1, passed=False, mode='hardware' if hardware else 'fake',
                  hardware_backend='v3_moveit/JakaSystem' if hardware else 'v3_moveit/GuardedGenericSystem',
                  scope='12-second commissioning hold observation; no trajectory/lease command source',
                  generated_at_utc=datetime.now(timezone.utc).isoformat(), package_sha256=digest,
                  script_sha256=sha256(Path(__file__)), domain_id=domain_id, observation_requested_s=OBSERVATION_S,
                  physical_stopping_accepted=False, teleoperation_accepted=False, physical_acceptance=False,
                  lease_publishers_created=0, trajectories_sent=0, action_goals_sent=0,
                  hardware_launch_started=False, ordinary_hardware_readiness_modified=False,
                  api_hold_is_not_readonly=hardware, cleanup_signals=[],
                  limitations=['Holding an API target is a motion API operation, not a no-motion-API test.',
                               'Observed small feedback drift cannot prove exactly zero physical movement.',
                               'No physical stop time/distance, collision braking, or Quest teleoperation is accepted.'])
    path.write_text(json.dumps(report, indent=2))
    process = None
    node = None
    log = None
    reader = None
    failure = None
    rclpy = None
    cleanup_offset = 0
    step_records = deque(maxlen=256)
    maximum_stage_ns = {}
    step_count = 0
    audit = artifacts / (basename+'_guard.jsonl')
    log_path = artifacts / (basename+'.log')
    try:
        if hardware:
            print('HARDWARE HOLD COMMISSIONING: will call servo_move_enable(TRUE), then send repeated '
                  'edg_servo_j_extend absolute activation-initial_ HOLD targets; cleanup calls motion_abort '
                  'and servo_move_enable(FALSE). This is NOT read-only and NOT physical braking/teleoperation acceptance.', flush=True)
            report['prerequisite_reports'] = hardware_prerequisites(ROOT, digest)
            report['site'] = validate_site(ROOT)
            saved = artifacts/'hardware_readiness.json'
            # Preserve the historical report field for tools that display archived checks.
            report['normal_hardware_gate'] = json.loads(saved.read_text()).get('teleoperation_ready') if saved.is_file() else None
        else:
            print('FAKE HOLD CHECK: no robot SDK connection; strict final-write audit enabled.', flush=True)
            require((ROOT/'site_ubuntu.yaml').is_file(), 'site_ubuntu.yaml missing')
        for key, relative in {'ROS_LOG_DIR':'log/ros', 'ROS_HOME':'.ros', 'COLCON_HOME':'.colcon',
                              'TMPDIR':'.tmp', 'XDG_CACHE_HOME':'.cache'}.items():
            directory=ROOT/relative;directory.mkdir(parents=True,exist_ok=True);os.environ[key]=str(directory)
        os.environ.update(ROS_DOMAIN_ID=str(domain_id), ROS_LOCALHOST_ONLY='1',
                          ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='', PYTHONDONTWRITEBYTECODE='1')
        import rclpy as ros
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, String
        from controller_manager_msgs.srv import ListControllers
        from ament_index_python.packages import get_package_prefix
        rclpy=ros
        prefix=Path(get_package_prefix('v3_moveit')).resolve()
        require(prefix.is_relative_to((ROOT/'install').resolve()), 'Active ROS package is not this project install')

        class Probe(Node):
            def __init__(self):
                super().__init__('v3_hold_check')
                self.feedback=FeedbackWindow();self.faults=[];self.lease_messages=0;self.step_context={"stage":"startup"}
                self.create_subscription(JointState,'/joint_states',self.joints,10)
                qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(String,'/v3/controller_fault',lambda m:self.faults.append(m.data),qos)
                self.create_subscription(String,'/v3/state',self.state,1)
                self.create_subscription(Bool,'/v3/motion_lease',self.lease,1)
                self.controllers=self.create_client(ListControllers,'/controller_manager/list_controllers')
            def joints(self,msg):
                host_ns=time.monotonic_ns();ros_ns=self.get_clock().now().nanoseconds
                names=list(msg.name);positions=list(msg.position);velocities=list(msg.velocity)
                stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                try:self.feedback.ingest(names,positions,velocities,stamp,ros_ns,host_ns*1e-9)
                except Exception as error:
                    self.feedback.reject(names,positions,velocities,stamp,ros_ns,host_ns,error,self.step_context)
                    self.faults.append(str(error))
            def state(self,msg):
                try:
                    if json.loads(msg.data).get('mode')=='FAULT':self.faults.append('Supervisor FAULT observed')
                except ValueError:self.faults.append('Malformed supervisor state')
            def lease(self,msg):
                self.lease_messages+=1;self.faults.append('Unexpected motion lease message; never-leased premise violated')

        rclpy.init();node=Probe()
        gc.disable();report["gc_disabled_during_bounded_capture"]=True
        # No ros2 daemon: inspect this freshly isolated DDS context before launch.
        deadline=time.monotonic()+2.
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        other=[(name,ns) for name,ns in node.get_node_names_and_namespaces() if (name,ns)!=(node.get_name(),node.get_namespace())]
        require(not other,f'Domain {domain_id} is occupied: {other}')
        for topic in ('/v3/motion_lease','/arm_controller/joint_trajectory','/joint_states'):
            require(node.count_publishers(topic)==0,f'Domain has an existing publisher on {topic}')
        report['domain_empty_before_launch']=True
        audit.unlink(missing_ok=True)
        launch=work/'minimal.launch.py';launch.write_text(launch_source(ROOT,hardware,audit))
        log=log_path.open('w');reader=log_path.open('r')
        process=subprocess.Popen(['ros2', 'launch', '--noninteractive', str(launch)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        report.update(launch_pid=process.pid,hardware_launch_started=hardware,
                      servo_enable_may_have_been_called=hardware,edg_hold_commands_may_have_been_sent=hardware)
        path.write_text(json.dumps(report,indent=2))
        graph_at=0.;log_at=0.
        allowed=allowed_nodes(hardware)
        report['allowed_node_identities']=sorted(allowed)
        def step():
            nonlocal graph_at,log_at,step_count
            step_count+=1
            trace=dict(sequence=step_count,start_host_monotonic_ns=time.monotonic_ns(),stages_ns={})
            phase_start=trace['start_host_monotonic_ns'];phase='process_poll'
            node.step_context=dict(sequence=step_count,stage=phase,stage_start_host_monotonic_ns=phase_start)
            def stage(name):
                nonlocal phase_start,phase
                now_ns=time.monotonic_ns();elapsed=now_ns-phase_start
                trace['stages_ns'][phase]=elapsed
                maximum_stage_ns[phase]=max(maximum_stage_ns.get(phase,0),elapsed)
                phase_start=now_ns;phase=name
                node.step_context=dict(sequence=step_count,stage=phase,stage_start_host_monotonic_ns=now_ns)
            try:
                require(process.poll() is None,'Commissioning launch exited before observation completed')
                stage('spin_once');rclpy.spin_once(node,timeout_sec=.002)
                stage('feedback_checks');require(not node.faults,'Feedback/controller fault: '+str(node.faults))
                now=time.monotonic()
                if node.feedback.last_host is not None:node.feedback.assert_fresh(now)
                if now-graph_at>.1:
                    stage('graph_checks');graph_at=now
                    unexpected=[(name,ns) for name,ns in node.get_node_names_and_namespaces() if (name,ns) not in allowed]
                    require(not unexpected,f'Unexpected nodes entered commissioning domain: {unexpected}')
                    for topic in ('/v3/motion_lease','/arm_controller/joint_trajectory'):
                        require(node.count_publishers(topic)==0,f'Forbidden command publisher appeared: {topic}')
                if now-log_at>.05:
                    stage('log_read');log_at=now;new=reader.read()
                    require(not any(marker in new for marker in FAULT_MARKERS),'Runtime fault/exit in commissioning log: '+new[-1000:])
            finally:
                stage('done');trace['end_host_monotonic_ns']=phase_start
                trace['total_ns']=phase_start-trace['start_host_monotonic_ns'];step_records.append(trace)
        ready_deadline=time.monotonic()+60.;listing=None;controllers_ready=False
        while time.monotonic()<ready_deadline:
            step()
            if listing is None and node.controllers.service_is_ready():listing=node.controllers.call_async(ListControllers.Request())
            if listing is not None and listing.done():
                controllers={item.name:item for item in listing.result().controller}
                controllers_ready=all(name in controllers and controllers[name].state=='active' for name in ('arm_controller','joint_state_broadcaster'))
                if controllers_ready:
                    require(controllers['arm_controller'].type=='v3_moveit/SmoothTrajectoryController','Unexpected arm controller type')
                    if node.feedback.last_host is not None:break
                listing=None
        require(controllers_ready and node.feedback.last_host is not None,'Fresh feedback and active controllers not ready')
        start=time.monotonic();first_sample=len(node.feedback.samples)
        while time.monotonic()-start<OBSERVATION_S:step()
        report.update(observation_elapsed_s=time.monotonic()-start,observation_samples=len(node.feedback.samples)-first_sample,
                      controller_type='v3_moveit/SmoothTrajectoryController',lease_messages_observed=node.lease_messages)
    except (Exception,KeyboardInterrupt) as error:
        failure=str(error) or 'Interrupted by user'
    finally:
        report['step_timing']=dict(total_steps=step_count,maximum_stage_ns=maximum_stage_ns,
                                   recent_steps=list(step_records),recent_step_limit=256)
        cleanup_offset=log_path.stat().st_size if process is not None and log_path.exists() else 0
        report['cleanup_started_host_monotonic_ns']=time.monotonic_ns()
        report['cleanup_log_offset_bytes']=cleanup_offset
        if process is not None:
            try:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT);report['cleanup_signals'].append('SIGINT')
                process.wait(timeout=30.)
                report['launch_exit_code']=process.returncode
                if process.returncode!=0:failure=failure or f'Launch exit code {process.returncode}'
            except subprocess.TimeoutExpired:
                # Preserve SDK ownership for its stop retry; do not SIGKILL a
                # blocked hardware owner and mistake process death for servo-off.
                failure=failure or 'Cleanup exceeded 30 s; servo exit unconfirmed; process may still own SDK'
                report['cleanup_unconfirmed_live_pid']=process.pid
                print('STOP UNCONFIRMED: inspect cabinet/physical stop and retained launch PID '+str(process.pid),flush=True)
            except Exception as error:failure=failure or str(error)
        if reader is not None:reader.close()
        if log is not None:log.close()
        if node is not None:
            report['feedback']=node.feedback.summary();report['observed_faults']=node.faults
            raw=artifacts/(basename+'_feedback.jsonl')
            rows=sorted(node.feedback.samples+node.feedback.rejected_samples,key=lambda item:item['host_monotonic_s'])
            raw.write_text(''.join(json.dumps(item,allow_nan=False)+'\n' for item in rows))
            report['rejected_feedback_samples']=node.feedback.rejected_samples
            report['feedback_log']=dict(path=str(raw),sha256=sha256(raw))
            node.destroy_node()
        if rclpy is not None and rclpy.ok():rclpy.shutdown()
        data=log_path.read_bytes() if process is not None and log_path.exists() else b''
        text=data.decode(errors='replace')
        report['log_phases']=classify_log_phases(data,cleanup_offset,'SIGINT' in report['cleanup_signals'],report.get('launch_exit_code'))
        report['servo_exit_confirmed']='Servo exit confirmed' in text if hardware else None
        if hardware and process is not None and not report['servo_exit_confirmed']:
            failure=failure or 'Hardware Servo exit confirmed log is missing'
        if report['log_phases']['runtime_faults']:
            failure=failure or 'Runtime fault or process failure was logged'
        if report['log_phases']['unexpected_cleanup_errors']:
            failure=failure or 'Unexpected cleanup error was logged'
        if not hardware and process is not None:
            try:
                from ros_smoke import guard_audit_summary
                report['final_write_guard']=guard_audit_summary(audit)
            except Exception as error:failure=failure or str(error)
        if source_digest()!=digest:failure=failure or 'Project source changed during the check'
        report['passed']=failure is None and report.get('observation_elapsed_s',0)>=OBSERVATION_S
        report['fully_clean_passed']=report['passed'] and report['log_phases']['clean']
        if failure:report['failure']=failure
        path.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware',action='store_true',help='Explicitly enable the real servo/EDG hold commissioning path')
    parser.add_argument('--domain-id',type=int,default=89,help='Unused local DDS domain, 1..101 (default 89)')
    args=parser.parse_args()
    return run(args.hardware,args.domain_id)


if __name__=='__main__':
    raise SystemExit(main())
