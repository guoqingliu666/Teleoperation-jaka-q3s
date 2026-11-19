"""Read-only JTC state capture and diagnostic replay through the native Guard.

Never launches ROS, publishes commands, connects hardware, or invokes the SDK.
Controller header timestamps are used for dt; receive times only end capture.
Topic telemetry lacks the hardware write(period) values and a cycle counter, so
this is diagnostic evidence, never a definitive hardware-path acceptance test.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
JOINTS = [f'joint_{i}' for i in range(1, 7)]
CPP = r'''
#include "v3_moveit/guard.hpp"
#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
using namespace v3_moveit;
int main() {
  Guard guard;
  for(auto& value:guard.lower)std::cin>>value;
  for(auto& value:guard.upper)std::cin>>value;
  Joints q{},actual{},previous{},velocity{},acceleration{};
  std::int64_t stamp=0,last_stamp=0;
  std::size_t index=0,gaps=0,valid_intervals=0,first_failure=0,contiguous=0,guard_steps=0;
  double max_jerk=0.,max_velocity=0.,max_acceleration=0.;
  double min_dt=std::numeric_limits<double>::infinity(),max_dt=0.;
  std::string reason;bool guard_failed=false,guard_eligible=true;
  while(std::cin>>stamp) {
    for(auto& value:q)std::cin>>value;
    for(auto& value:actual)std::cin>>value;
    if(!std::cin) return 2;
    ++index;
    if(index==1) {guard.reset(q);previous=q;last_stamp=stamp;continue;}
    const double dt=double(stamp-last_stamp)*1e-9;
    min_dt=std::min(min_dt,dt);max_dt=std::max(max_dt,dt);
    if(dt<.004 || dt>.012) {
      ++gaps;guard_eligible=false;contiguous=0;previous=q;velocity.fill(0.);acceleration.fill(0.);
      last_stamp=stamp;continue;
    }
    ++valid_intervals;++contiguous;
    Joints next_velocity{},next_acceleration{};
    for(std::size_t joint=0;joint<6;++joint) {
      next_velocity[joint]=(q[joint]-previous[joint])/dt;
      next_acceleration[joint]=(next_velocity[joint]-velocity[joint])/dt;
      const double jerk=(next_acceleration[joint]-acceleration[joint])/dt;
      max_velocity=std::max(max_velocity,std::abs(next_velocity[joint]));
      if(contiguous>=2)max_acceleration=std::max(max_acceleration,std::abs(next_acceleration[joint]));
      if(contiguous>=3)max_jerk=std::max(max_jerk,std::abs(jerk));
    }
    if(!guard_failed && guard_eligible) {
      ++guard_steps;
      try {guard.check(q,actual,dt);}
      catch(const std::exception& e) {guard_failed=true;first_failure=index;reason=e.what();}
    }
    previous=q;velocity=next_velocity;acceleration=next_acceleration;last_stamp=stamp;
  }
  std::cout<<std::setprecision(17)<<"{\"rows\":"<<index<<",\"valid_intervals\":"<<valid_intervals
    <<",\"guard_replay_steps_before_gap_or_failure\":"<<guard_steps
    <<",\"gaps_or_invalid_intervals\":"<<gaps<<",\"first_failure_row\":"<<first_failure
    <<",\"first_failure_reason\":\""<<reason<<"\",\"peak_discrete_jerk_rad_s3\":"<<max_jerk
    <<",\"peak_discrete_velocity_rad_s\":"<<max_velocity
    <<",\"peak_discrete_acceleration_rad_s2\":"<<max_acceleration
    <<",\"min_header_dt_s\":"<<(index>1?min_dt:0.)<<",\"max_header_dt_s\":"<<max_dt<<"}\n";
}
'''


class Capture(Node):
    def __init__(self):
        super().__init__('v3_guard_replay_observer')
        self.samples = []
        self.last_receive = None
        self.first_receive = None
        self.supervisor_state = None
        self.state_subscription = self.create_subscription(String, "/v3/state", self.receive_state, 100)
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState, '/arm_controller/controller_state',
            self.receive, QoSProfile(depth=10000))

    def receive_state(self, msg):
        try:
            self.supervisor_state = json.loads(msg.data)
        except ValueError:
            pass

    def receive(self, msg):
        received = time.monotonic()
        if self.first_receive is None:
            self.first_receive = received
        self.last_receive = received
        self.samples.append(dict(
            stamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
            joint_names=list(msg.joint_names),
            reference=list(msg.reference.positions),
            output=list(msg.output.positions),
            feedback=list(msg.feedback.positions),
            reference_velocities=list(msg.reference.velocities),
            output_velocities=list(msg.output.velocities),
            reference_accelerations=list(msg.reference.accelerations),
            output_accelerations=list(msg.output.accelerations),
            supervisor_state=self.supervisor_state))


def replay(samples, prefix):
    executable = ROOT / 'artifacts' / 'guard_replay_native'
    environment = dict(os.environ, TMPDIR=str(ROOT / '.tmp'))
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT / 'src/v3_moveit/include'), '-x', 'c++', '-',
                    '-o', str(executable)], input=CPP, text=True, check=True, env=environment)
    robot = ET.parse(ROOT / 'src/v3_moveit/description/s5.urdf').getroot()
    limits = {j.get('name'): j.find('limit') for j in robot.findall('joint') if j.find('limit') is not None}
    lines = [' '.join(limits[j].get('lower') for j in JOINTS),
             ' '.join(limits[j].get('upper') for j in JOINTS)]
    selected = []
    reference_fallbacks = 0
    missing_feedback = 0
    skipped = 0
    for sample in samples:
        try:
            indices = [sample['joint_names'].index(j) for j in JOINTS]
            field = 'output' if len(sample['output']) == 6 else 'reference'
            if field == 'reference':
                reference_fallbacks += 1
            q = [sample[field][i] for i in indices]
            if len(sample['feedback']) == 6:
                actual = [sample['feedback'][i] for i in indices]
            else:
                actual = q
                missing_feedback += 1
            selected.append(q)
            lines.append(' '.join(map(str, [sample['stamp_ns'], *q, *actual])))
        except (ValueError, IndexError):
            skipped += 1
    (prefix.with_suffix('.tsv')).write_text('\n'.join(lines) + '\n')
    result = subprocess.run([str(executable)], input='\n'.join(lines) + '\n',
                            text=True, capture_output=True, check=True, env=environment)
    report = json.loads(result.stdout)
    initial_stationary = len(selected) >= 10 and all(
        max(abs(a-b) for a, b in zip(selected[0], q)) < 1e-9 for q in selected[1:10])
    report.update(
        status='inconclusive', captured_messages=len(samples),
        replay_source='controller_state.output.positions' if not reference_fallbacks else 'reference fallback used',
        reference_fallbacks=reference_fallbacks, missing_feedback=missing_feedback,
        skipped_messages=skipped, initial_ten_samples_stationary=initial_stationary,
        dt_source='consecutive controller_state.header.stamp; never receive time',
        physical_acceptance=False, robot_connections=0, robot_commands=0,
        limitations=[
            'Topic headers do not expose the exact hardware write(period) or a cycle sequence counter.',
            'Dropped/non-125Hz state publications prevent exact per-write reconstruction.',
            'Guard replay stops at the first gap; jerk peaks require four contiguous position samples.',
            'A diagnostic failure does not prove physical hardware motion occurred.'],
    )
    prefix.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--duration', type=float, default=45.)
    parser.add_argument('--idle', type=float, default=3.)
    parser.add_argument('--prefix', default='controller_guard_replay')
    args = parser.parse_args()
    if Path(args.prefix).name != args.prefix:
        parser.error('--prefix must be a plain artifact basename')
    prefix = ROOT / 'artifacts' / args.prefix
    rclpy.init()
    node = Capture()
    started = time.monotonic()
    print('READY: read-only JTC controller_state subscriber; no commands or launch', flush=True)
    try:
        while time.monotonic()-started < args.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if node.last_receive is not None and now-node.last_receive > args.idle:
                break
            if node.first_receive is not None and now-node.first_receive > args.duration:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    prefix.with_suffix('.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in node.samples))
    replay(node.samples, prefix)


if __name__ == '__main__':
    main()
