#!/usr/bin/env python3
"""Fake-only action semantics test; no MoveGroup, Servo, Quest, or SDK connection.

Run after a successful project build with /opt/ros/jazzy and install/setup.bash
sourced. The test hardcodes mode=fake + GuardedGenericSystem strict=true and uses
its own ROS domain. It never starts the normal teleoperation stack.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
NAMES = [f'joint_{i}' for i in range(1, 7)]
NATIVE = r'''
#include "v3_moveit/path_retime.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
int main(int argc,char** argv){
  if(argc!=14)return 1;
  v3_moveit::PathVector start(6),target(6);
  for(int i=0;i<6;++i){start[i]=std::stod(argv[1+i]);target[i]=start[i]+std::stod(argv[7+i]);}
  const double scale=std::stod(argv[13]);if(scale<=0.||scale>1.)return 2;
  v3_moveit::PathLimits limits{v3_moveit::PathVector(6,.1745329252*scale),
    v3_moveit::PathVector(6,.6981317008*scale),v3_moveit::PathVector(6,2.7925268032*scale)};
  const auto path=v3_moveit::retime_path({start,target},limits);
  std::cout<<std::setprecision(17)<<"[";
  for(std::size_t k=0;k<path.samples.size();++k){
    const auto& p=path.samples[k];if(k)std::cout<<",";
    std::cout<<"{\"time_ns\":"<<std::llround(p.time*1e9)<<",\"positions\":[";
    for(int j=0;j<6;++j){if(j)std::cout<<",";std::cout<<p.position[j];}std::cout<<"],\"velocities\":[";
    for(int j=0;j<6;++j){if(j)std::cout<<",";std::cout<<p.velocity[j];}std::cout<<"],\"accelerations\":[";
    for(int j=0;j<6;++j){if(j)std::cout<<",";std::cout<<p.acceleration[j];}std::cout<<"]}";
  }
  std::cout<<"]\n";
}
'''


def run(domain):
    if not 1 <= domain <= 231:
        raise ValueError('Use a dedicated ROS_DOMAIN_ID in 1..231')
    work = ROOT / '.tmp/smooth_controller_ros_test'
    output = ROOT / 'artifacts/smooth_controller_ros_test'
    work.mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    for key, value in {'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
                       'ROS_LOG_DIR': str(ROOT / 'log/ros'), 'ROS_HOME': str(ROOT / '.ros'),
                       'TMPDIR': str(ROOT / '.tmp'), 'XDG_CACHE_HOME': str(ROOT / '.cache'),
                       'PYTHONDONTWRITEBYTECODE': '1'}.items():
        os.environ[key] = value
    # Generate the incoming test trajectory with the installed upstream Ruckig.
    source = work / 'trajectory.cpp'
    binary = work / 'trajectory'
    source.write_text(NATIVE)
    subprocess.run(['c++', '-std=c++17', '-O2', '-I/opt/ros/jazzy/include', '-I' + str(ROOT / 'src/v3_moveit/include'), str(source),
                    '-L/opt/ros/jazzy/lib/x86_64-linux-gnu',
                    '-Wl,-rpath,/opt/ros/jazzy/lib/x86_64-linux-gnu', '-lruckig', '-o', str(binary)],
                   check=True, cwd=ROOT)
    audit = output / 'guard.jsonl'
    if audit.exists():
        audit.rename(output / f'guard.previous.{time.time_ns()}.jsonl')
    # Reuse current bringup's exact model/control interfaces. Its first four
    # processes are state publisher, manager, and the two controller spawners.
    # The hardcoded context cannot select hardware or start the remaining nodes.
    launch = work / 'minimal.launch.py'
    launch.write_text('''import importlib.util
from launch import LaunchDescription
from launch.actions import OpaqueFunction

def minimal(context):
    context.launch_configurations.update(%r)
    spec=importlib.util.spec_from_file_location('v3_source_bringup',%r)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nodes=module.setup(context)
    return nodes[:4]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=minimal)])
''' % ({'mode': 'fake', 'guarded_fake': 'true', 'guard_audit_path': str(audit),
        'guard_strict': 'true', 'runtime_config': '', 'rviz': 'false'},
        str(ROOT / 'src/v3_moveit/launch/bringup.launch.py')))
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class Probe(Node):
        def __init__(self):
            super().__init__('v3_smooth_action_probe')
            self.q = None
            self.v = None
            self.samples = []
            self.faults = []
            self.create_subscription(JointState, '/joint_states', self.joints, 10)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/v3/controller_fault', lambda m: self.faults.append(m.data), qos)
            self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
            self.topic = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 1)

        def joints(self, msg):
            try:
                self.q = [msg.position[msg.name.index(name)] for name in NAMES]
                self.v = [msg.velocity[msg.name.index(name)] for name in NAMES]
                self.samples.append((time.monotonic(), self.q.copy(), self.v.copy()))
            except (ValueError, IndexError):
                pass

    from verify_offline import source_digest
    from ros_smoke import guard_audit_summary
    report = dict(package_sha256=source_digest(),passed=False, hardware_backend='v3_moveit/GuardedGenericSystem',
                  strict=True, domain_id=domain, sdk_calls=0, physical_acceptance=False,
                  test_trajectory_limits=dict(velocity=.1745329252, acceleration=.6981317008, jerk=2.7925268032),
                  test_limit_scales=[.5, 1.], cases=[], checks=[])
    report_path = output / 'result.json'
    report_path.write_text(json.dumps(report, indent=2))
    process = None
    log = (output / 'ros.log').open('w')
    rclpy.init()
    node = Probe()

    def step(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if process and process.poll() is not None:
                raise RuntimeError('Minimal fake launch exited')
            rclpy.spin_once(node, timeout_sec=.005)
            if node.faults:
                raise RuntimeError(f'Controller fault: {node.faults[-1]}')

    def wait(future, timeout=8.):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            step(.01)
        if not future.done():
            raise RuntimeError('Action future timed out')
        return future.result()

    def goal(delta, scale):
        data = json.loads(subprocess.check_output([str(binary)] + [str(x) for x in node.q + delta + [scale]], text=True))
        result = FollowJointTrajectory.Goal()
        result.trajectory.joint_names = NAMES
        for row in data:
            point = JointTrajectoryPoint()
            point.positions = row['positions']
            point.velocities = row['velocities']
            point.accelerations = row['accelerations']
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(row['time_ns'], 1000000000)
            result.trajectory.points.append(point)
        return result

    def send(value, accepted=True):
        handle = wait(node.client.send_goal_async(value))
        if handle.accepted != accepted:
            raise RuntimeError(f'Goal acceptance {handle.accepted}, expected {accepted}')
        return handle

    def successful(handle):
        value = wait(handle.get_result_async())
        if value.status != GoalStatus.STATUS_SUCCEEDED or value.result.error_code != 0:
            raise RuntimeError(f'Expected SUCCEEDED: {value.status}, {value.result}')

    try:
        step(.5)
        other = [(name, namespace) for name, namespace in node.get_node_names_and_namespaces()
                 if name != node.get_name()]
        if other:
            raise RuntimeError(f'Domain {domain} already has nodes; refusing mixed-session test: {other}')
        process = subprocess.Popen(['ros2', 'launch', str(launch)], cwd=ROOT,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        ready = time.monotonic() + 60
        while (node.q is None or not node.client.server_is_ready()) and time.monotonic() < ready:
            step(.05)
        if node.q is None or not node.client.server_is_ready():
            raise RuntimeError('Minimal controllers did not become ready')
        step(.3)
        for scale in (.5, 1.):
            report['current_limit_scale'] = scale
            first_check = len(report['checks'])
            successful(send(goal([.015, 0., 0., 0., 0., 0.], scale)))
            report['checks'].append('normal_goal_succeeded')
            step(.1)
            moving = send(goal([.12, 0., 0., 0., 0., 0.], scale))
            moving_result = moving.get_result_async()
            deadline = time.monotonic() + 3
            while max(abs(x) for x in node.v) < .025 and time.monotonic() < deadline:
                step(.008)
            if max(abs(x) for x in node.v) < .025:
                raise RuntimeError('Cancellation test did not reach moving state')
            # Prebuild before cancel so compilation/JSON creation cannot consume the
            # braking interval in which rejection must be tested.
            next_goal = goal([-.015, 0., 0., 0., 0., 0.], scale)
            start = time.monotonic()
            response = wait(moving.cancel_goal_async())
            if len(response.goals_canceling) != 1:
                raise RuntimeError('Active cancellation was not accepted')
            send(next_goal, accepted=False)
            report['checks'].append('new_action_rejected_while_stopping')
            marker = JointTrajectory()
            marker.joint_names = NAMES
            point = JointTrajectoryPoint()
            point.positions = [x + .5 for x in node.q]
            point.velocities = [0.] * 6
            point.accelerations = [0.] * 6
            point.time_from_start.nanosec = 8000000
            marker.points = [point]
            node.topic.publish(marker)
            canceled = wait(moving_result, timeout=3.)
            elapsed = time.monotonic() - start
            if canceled.status != GoalStatus.STATUS_CANCELED or canceled.result.error_code != 0:
                raise RuntimeError(f'Expected CANCELED: {canceled.status}, {canceled.result}')
            if not .05 < elapsed < 2.5:
                raise RuntimeError(f'Cancellation result elapsed time implausible: {elapsed}')
            report['checks'].append('cancellation_result_after_braking')
            report['cancel_result_elapsed_s'] = elapsed
            step(.08)
            first = node.q.copy()
            start_sample = len(node.samples)
            step(.3)
            held = node.samples[start_sample:]
            drift = max((abs(q[i] - first[i]) for _, q, _ in held for i in range(6)), default=float('inf'))
            speed = max((abs(v) for _, _, velocities in held for v in velocities), default=float('inf'))
            if drift > 1e-8 or speed > 1e-7:
                raise RuntimeError(f'Terminal hold changed: drift={drift}, speed={speed}')
            report.update(hold_drift_rad=drift, hold_velocity_rad_s=speed)
            report['checks'].extend(['terminal_position_held', 'topic_ignored_while_stopping'])
            successful(send(goal([-.015, 0., 0., 0., 0., 0.], scale)))
            report['checks'].append('new_goal_succeeded_after_cancel_result')
            step(.1)
            report['cases'].append(dict(limit_scale=scale, checks=report['checks'][first_check:],
                                        cancel_result_elapsed_s=elapsed, hold_drift_rad=drift,
                                        hold_velocity_rad_s=speed))
        report['passed'] = True
    except Exception as error:
        report['failure'] = str(error)
        raise
    finally:
        if process:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=15)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        try:
            report['final_write_guard'] = guard_audit_summary(audit)
        except Exception as error:
            report.update(passed=False, audit_failure=str(error))
        if audit.exists():
            records = [json.loads(line) for line in audit.read_text().splitlines() if line.strip()]
            rejected = [r for r in records if r.get('result') == 'rejected']
            report['guard_rejections'] = len(rejected)
            starts = [r for r in records if r.get('event') == 'session_start']
            if not starts or any(not r.get('strict') or r.get('sdk_calls') != 0 for r in starts):
                report.update(passed=False, guard_session_invalid=True)
            if rejected:
                report.update(passed=False, guard_first_rejection=rejected[0])
        else:
            report.update(passed=False, guard_missing=True)
        report_path.write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    if not report['passed']:
        raise RuntimeError('Final strict Guard audit did not pass')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=87)
    args = parser.parse_args()
    run(args.domain_id)
