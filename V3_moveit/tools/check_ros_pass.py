"""Validate current strict fake evidence before a hardware entry; no ROS/SDK import."""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FAKE_BACKEND = 'v3_moveit/GuardedGenericSystem'
SIMULATION_REPORTS = (
    ('artifacts/ros_smoke_pass.json', 'smoke'),
    ('artifacts/smooth_controller_ros_test/result.json', 'action'),
    ('artifacts/fixed_servo_ros_test/pure_servo/result.json', 'pure_servo'),
    ('artifacts/fixed_servo_ros_test/auto_replan/result.json', 'auto_replan'),
)
REQUIRED_ACTION_CHECKS = {
    'normal_goal_succeeded', 'new_action_rejected_while_stopping',
    'cancellation_result_after_braking', 'terminal_position_held',
    'topic_ignored_while_stopping', 'new_goal_succeeded_after_cancel_result',
}
FIXED_SERVO_CHECKS = {
    'pure_servo': {
        'forward_measured_motion', 'reverse_measured_motion', 'continuous_follow_without_ompl',
        'grip_release_stop_feedback_stationary', 'reclutch_without_jump',
        'reclutch_follow_measured_motion', 'input_loss_stop_feedback_stationary',
        'grip_release_stop_strict_command_hold', 'input_loss_stop_strict_command_hold',
    },
    'auto_replan': {
        'automatic_ompl_actual_motion', 'ompl_cancel_stop_feedback_stationary',
        'ompl_cancel_result_then_resume', 'follow_after_ompl_cancel_measured_motion',
        'actual_follow_after_ompl_cancel', 'final_release_stop_feedback_stationary',
        'ompl_cancel_stop_strict_command_hold', 'final_release_stop_strict_command_hold',
    },
}


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_guard(guard):
    require(isinstance(guard, dict), 'Final-write Guard summary is malformed')
    require(guard.get('passed') is True and guard.get('strict') is True and
            guard.get('session_end_seen') is True and guard.get('complete', True) is True,
            'Complete strict final-write Guard evidence is required')
    for key in ('sessions', 'writes', 'accepted_writes', 'startup_writes', 'rejections'):
        require(type(guard.get(key)) is int, f'Guard counter missing or not an integer: {key}')
    require(guard['sessions'] >= 1 and guard['accepted_writes'] > 0 and guard['startup_writes'] >= 0 and
            guard['writes'] == guard['accepted_writes']+guard['startup_writes'] and guard['rejections'] == 0,
            'Guard writes are incomplete, empty, inconsistent, or include rejections')
    low, high = guard.get('minimum_period_s'), guard.get('maximum_period_s')
    require(finite_number(low) and finite_number(high) and .004 <= low <= high <= .012,
            'Guard final-write periods violate the unchanged 4..12 ms bounds')
    require(guard.get('physical_acceptance') is False, 'Fake Guard evidence cannot claim physical acceptance')


def validate_simulation_record(record, digest, kind):
    require(isinstance(record, dict), 'Simulation report is malformed')
    require(record.get('passed') is True and not record.get('failure'), 'Required simulation report is not passed')
    require(record.get('package_sha256') == digest, 'Simulation report does not match current source_digest')
    require(record.get('hardware_backend') == FAKE_BACKEND, 'Strict SDK-free fake backend is required')
    require(record.get('physical_acceptance') is False, 'Simulation report cannot claim physical acceptance')
    require(record.get('fake_velocity_tracking',False) is False,
            'Fake-only follow override cannot substitute for the configured hardware profile')
    validate_guard(record.get('final_write_guard'))
    if kind == 'smoke':
        motion = record.get('measured_joint_motion_rad')
        require(record.get('input_loss_stops') is True and record.get('injected_obstacle_detected') is True and
                finite_number(motion) and motion > 0, 'Full Servo/input-loss/collision smoke evidence is required')
    elif kind == 'action':
        require(record.get('strict') is True, 'Action test must explicitly select strict mode')
        require(REQUIRED_ACTION_CHECKS.issubset(record.get('checks', [])), 'Action cancellation/restart evidence incomplete')
        cases = {case.get('limit_scale'): case for case in record.get('cases', [])}
        for scale in (.5, 1.):
            require(scale in cases and REQUIRED_ACTION_CHECKS.issubset(cases[scale].get('checks', [])),
                    f'Action checks at limit scale {scale} are incomplete')
    elif kind in FIXED_SERVO_CHECKS:
        require(record.get('case') == kind and record.get('auto_replan') is (kind == 'auto_replan'),
                'Fixed Servo case identity/auto_replan setting does not match required report')
        require(record.get('strict') is True and type(record.get('robot_connections')) is int and
                record['robot_connections'] == 0 and type(record.get('sdk_calls')) is int and record['sdk_calls'] == 0,
                'Fixed Servo evidence must identify strict fake execution with zero robot/SDK calls')
        require(FIXED_SERVO_CHECKS[kind].issubset(record.get('checks', [])),
                f'Fixed Servo behavioral evidence is incomplete: {kind}')
        capture = record.get('capture', {})
        require(type(capture.get('frames')) is int and capture['frames'] > 0 and
                type(capture.get('overflow_frames')) is int and capture['overflow_frames'] == 0 and
                capture.get('complete', True) is True, 'Fixed Servo capture is empty or incomplete')
        calls = record.get('retiming_adapter_calls')
        executing = record.get('supervisor_mode_message_counts', {}).get('EXECUTING', 0)
        require(type(calls) is int and type(executing) is int,
                'Fixed Servo planning coverage counters are missing/malformed')
        require((calls == 0 and executing == 0) if kind == 'pure_servo' else (calls > 0 and executing > 0),
                'Fixed Servo actual planning coverage does not match case')
        cleanup = record.get('cleanup', {})
        require(cleanup.get('launch_returncode') == 0 and cleanup.get('unexpected_process_deaths') == [] and
                record.get('forced_fake_cleanup', False) is False, 'Fixed Servo cleanup is incomplete/unexpected')
        # The separately recorded known MoveGroup SIGINT destructor crash is not
        # silently promoted to a clean-exit claim and does not erase functional evidence.
    else:
        raise RuntimeError(f'Unknown simulation evidence kind: {kind}')


def validate_reports(root, digest):
    records = {}
    for relative, kind in SIMULATION_REPORTS:
        try:
            record = json.loads((root/relative).read_text())
            validate_simulation_record(record, digest, kind)
        except (OSError, ValueError, TypeError, KeyError, AttributeError, RuntimeError) as exc:
            raise RuntimeError(f'{relative}: {exc}') from exc
        records[relative] = record
    return records


def main():
    from verify_offline import source_digest
    try:
        validate_reports(ROOT, source_digest())
    except RuntimeError as exc:
        raise SystemExit('实机证据门禁未通过：'+str(exc)+
                         '。请运行 tools/verify_ros.sh 和 tools/fixed_servo_ros_test.py 更新全部当前代码验收。') from exc


if __name__ == '__main__':
    main()
