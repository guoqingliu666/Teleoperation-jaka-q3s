#!/usr/bin/env python3
"""Bounded real DH116 feedback commissioning; no motion publishers/services."""
import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--duration', type=float, default=20.0)
    args = parser.parse_args()
    if not args.hardware or not 5 <= args.duration <= 60:
        parser.error('--hardware and a duration between 5 and 60 seconds are required')
    os.environ['ROS_DOMAIN_ID'] = '0'
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from hand_control_interfaces.msg import HandJointState
    from rosidl_runtime_py.convert import message_to_ordereddict

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('v3_dh116_feedback_check', start_parameter_services=False, enable_rosout=False)
    frames = []
    invalid = []
    canceled = False

    def cancel(*unused):
        nonlocal canceled
        canceled = True

    signal.signal(signal.SIGINT, cancel)
    signal.signal(signal.SIGTERM, cancel)

    def on_state(msg):
        now = time.monotonic()
        age = (node.get_clock().now().nanoseconds - msg.header.stamp.sec * 10**9 - msg.header.stamp.nanosec) / 1e9
        valid = (msg.active_dof == 6 and list(msg.joint_ids) == list(range(1, 7))
                 and all(len(getattr(msg, name)) == 6 for name in ['angles', 'currents', 'statuses', 'alarms'])
                 and all(math.isfinite(x) for x in list(msg.angles) + list(msg.currents))
                 and all(x >= 0 for x in msg.statuses) and 0 <= age < 0.2)
        row = dict(received_monotonic=now, age_s=age, message=message_to_ordereddict(msg))
        (frames if valid else invalid).append(row)

    node.create_subscription(HandJointState, '/hand_control/realtime_state', on_state, qos_profile_sensor_data)
    report = dict(started_at_utc=datetime.now(timezone.utc).isoformat(), passed=False,
                  hardware='DH116', duration_s=args.duration, motor_commands=0,
                  motion_services_expected=False, source_mac='02:01:01:01:01:01',
                  interface='enp11s0', ros_domain_id=0)
    logfile = (ROOT / 'artifacts/dh116_feedback_driver.log').open('w')
    child = subprocess.Popen(['bash', 'START_DH116_FEEDBACK.sh', '--hardware'], cwd=ROOT,
                             stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
    deadline = time.monotonic() + 180
    first = None
    try:
        while not canceled:
            rclpy.spin_once(node, timeout_sec=0.05)
            if child.poll() is not None:
                raise RuntimeError(f'Driver exited with {child.returncode}; inspect driver log')
            now = time.monotonic()
            if frames and first is None:
                first = frames[0]['received_monotonic']
                print('DH116 feedback received; collecting bounded observation', flush=True)
            if first is not None and now - first >= args.duration:
                break
            if first is not None and now - frames[-1]['received_monotonic'] > 0.5:
                raise RuntimeError('DH116 feedback became stale')
            if now > deadline:
                raise RuntimeError('Authentication or first valid DH116 feedback timed out')
        services = [name for name, types in node.get_service_names_and_types() if name.startswith('/hand_control/')]
        motion_services = [name for name in services if any(x in name for x in ['set_enable', 'home_motors', 'move_motors', 'control_motors', 'play_gesture'])]
        intervals = [b['received_monotonic'] - a['received_monotonic'] for a, b in zip(frames, frames[1:])]
        report.update(canceled=canceled, valid_frames=len(frames), invalid_frames=len(invalid),
                      duration_observed_s=frames[-1]['received_monotonic'] - frames[0]['received_monotonic'] if frames else 0,
                      maximum_gap_s=max(intervals, default=None), motion_services_found=motion_services,
                      first_feedback=frames[0]['message'] if frames else None,
                      last_feedback=frames[-1]['message'] if frames else None,
                      all_alarm_free=bool(frames) and all(not any(f['message']['alarms']) for f in frames))
        report['passed'] = (not canceled and len(frames) >= 100 and not invalid
                            and not motion_services and report['all_alarm_free']
                            and report['duration_observed_s'] >= args.duration - 0.2)
    except Exception as exc:
        report['failure'] = str(exc)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                report['cleanup_unconfirmed_live_pid'] = child.pid
                report['passed'] = False
        report['driver_exit_code'] = child.poll()
        if child.poll() != 0:
            report['passed'] = False
        logfile.close()
        log_text = (ROOT / 'artifacts/dh116_feedback_driver.log').read_text(errors='replace')
        report['device_identity_read'] = 'DH116_SII_IDENTITY_OK' in log_text
        report['identity_source'] = 'EtherCAT SII vendor/product/revision; exact DH116 match'
        report['firmware_and_serial_read'] = 'DH116_DEVICE_ID firmware=' in log_text
        report['tpdo_decode_seen'] = 'DH116_TPDO_DECODE_OK' in log_text
        if not report['device_identity_read'] or not report['tpdo_decode_seen']:
            report['passed'] = False
        node.destroy_node()
        rclpy.shutdown()
        capture = ROOT / 'artifacts/dh116_feedback_capture.jsonl'
        capture.write_text(''.join(json.dumps(row, ensure_ascii=False) + '\n' for row in frames + invalid))
        report['capture'] = dict(path=str(capture), sha256=hashlib.sha256(capture.read_bytes()).hexdigest())
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        (ROOT / 'artifacts/dh116_feedback_check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
