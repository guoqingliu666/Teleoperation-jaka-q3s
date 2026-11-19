#!/usr/bin/env python3
"""Offline fake-capture analysis. Reads files only; never imports ROS or the SDK."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MATRIX = np.array([[1, 1, 1], [3, 4, 5], [6, 12, 20]], dtype=float)
DT = .008


def stamp_ns(stamp):
    return stamp['sec'] * 10**9 + stamp['nanosec']


def read_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def interpolate(left, right, elapsed, duration):
    """Endpoint p/v/a quintic Hermite, in normalized time to avoid ill conditioning."""
    q0, q1 = np.array(left['positions']), np.array(right['positions'])
    v0, v1 = np.array(left['velocities']), np.array(right['velocities'])
    a0, a1 = np.array(left['accelerations']), np.array(right['accelerations'])
    c1, c2 = v0 * duration, a0 * duration**2 / 2
    c3, c4, c5 = np.linalg.solve(MATRIX, np.array([
        q1 - q0 - c1 - c2, v1 * duration - c1 - 2 * c2,
        a1 * duration**2 - 2 * c2]))
    u = elapsed / duration
    return (
        q0 + c1*u + c2*u**2 + c3*u**3 + c4*u**4 + c5*u**5,
        (c1 + 2*c2*u + 3*c3*u**2 + 4*c4*u**3 + 5*c5*u**4) / duration,
        (2*c2 + 6*c3*u + 12*c4*u**2 + 20*c5*u**3) / duration**2,
        (6*c3 + 24*c4*u + 60*c5*u**2) / duration**3)


def reconstruct(states, messages, writes):
    output = []
    for state in states:
        msg = state['message']
        target_ns = stamp_ns(msg['header']['stamp']) + 8000000
        candidates = []
        for received in messages:
            age = state['host_monotonic_ns'] - received['host_monotonic_ns']
            if not 0 <= age <= 100_000_000:
                continue
            trajectory = received['message']
            points = trajectory['points']
            relative_ns = target_ns - stamp_ns(trajectory['header']['stamp'])
            knots = [stamp_ns(point['time_from_start']) for point in points]
            segment = int(np.searchsorted(knots, relative_ns, side='right') - 1)
            if not 0 <= segment < len(points)-1:
                continue  # No assumed point-before bridge or extrapolation.
            left, right = knots[segment:segment+2]
            if right <= left:
                continue
            if any(len(points[i].get(key, [])) != 6
                   for i in (segment, segment+1)
                   for key in ('positions', 'velocities', 'accelerations')):
                continue
            q, v, a, j = interpolate(points[segment], points[segment+1],
                                    (relative_ns-left)*1e-9, (right-left)*1e-9)
            error = float(np.max(np.abs(q-msg['output']['positions'])))
            candidates.append((error, received, relative_ns, segment, left, right, q, v, a, j))
        if not candidates:
            continue
        best = min(candidates, key=lambda candidate: candidate[0])
        error, received, relative_ns, segment, left, right, q, v, a, j = best
        near = [row for row in writes if abs(row['host_monotonic_ns']-state['host_monotonic_ns']) < 2_000_000]
        exact = [row for row in near if row['command'] == msg['output']['positions']]
        output.append(dict(
            target_ros_ns=target_ns, trajectory_header_ns=stamp_ns(received['message']['header']['stamp']),
            trajectory_receipt_host_ns=received['host_monotonic_ns'],
            relative_target_ns=relative_ns, segment=segment,
            segment_start_ns=left, segment_end_ns=right,
            max_output_error_rad=error, final_write_index=exact[0]['index'] if exact else None,
            output=msg['output']['positions'], interpolated_velocity=v.tolist(),
            interpolated_acceleration=a.tolist(), interpolated_continuous_jerk=j.tolist()))
    return output


def startup_counterfactual(messages, reconstructions):
    # This experiment is valid only when q/a establish a saturated constant-jerk
    # startup after two stationary points; never infer it for arbitrary motion.
    selected = next((row for row in messages if len(row['message']['points']) >= 6), None)
    if selected is None:
        return dict(available=False, reason='No six-point startup message captured')
    msg = selected['message']
    points = msg['points'][:6]
    if any(len(point.get(key, [])) != 6 for point in points
           for key in ('positions', 'velocities', 'accelerations')):
        return dict(available=False, reason='Missing endpoint derivatives')
    q_all = np.array([point['positions'] for point in points])
    a_all = np.array([point['accelerations'] for point in points])
    v_all = np.array([point['velocities'] for point in points])
    axis = int(np.argmax(abs(a_all[2])))
    if np.max(abs(q_all[1]-q_all[0])) > 1e-13 or np.max(abs(a_all[:2])) > 1e-12 or np.max(abs(v_all[:2])) > 1e-12:
        return dict(available=False, reason='Prefix is not two stationary points')
    q = q_all[:, axis]-q_all[0, axis]
    a, v = a_all[:, axis], v_all[:, axis]
    ts = np.array([stamp_ns(point['time_from_start']) for point in points])*1e-9
    jerk = a[2]/DT
    true_t = np.maximum(np.arange(6)-1, 0)*DT
    expected_q = jerk*true_t**3/6
    if abs(jerk) < 1e-8 or np.max(abs(q-expected_q)) > 1e-12 or np.max(abs(a-jerk*true_t)) > 1e-10:
        return dict(available=False, reason='Captured q/a do not establish four constant-jerk steps')
    true_v = jerk*true_t**2/2
    header_ns = stamp_ns(msg['header']['stamp'])
    sample_ns = sorted({row['target_ros_ns']-header_ns for row in reconstructions
                        if row['max_output_error_rad'] < 1e-12
                        and ts[1]*1e9 < row['target_ros_ns']-header_ns < ts[4]*1e9})
    if len(sample_ns) < 3 or any(y-x != 8000000 for x, y in zip(sample_ns[:2], sample_ns[1:3])):
        return dict(available=False, reason='Three consecutive startup output samples were not reconstructed')

    def sample(t, times, velocities):
        if t <= times[1]:
            return 0.
        i = min(int(np.searchsorted(times, t, side='right')-1), len(times)-2)
        duration = times[i+1]-times[i]
        c1, c2 = velocities[i]*duration, a[i]*duration**2/2
        c3, c4, c5 = np.linalg.solve(MATRIX, [
            q[i+1]-q[i]-c1-c2, velocities[i+1]*duration-c1-2*c2,
            a[i+1]*duration**2-2*c2])
        u = (t-times[i])/duration
        return q[i]+c1*u+c2*u*u+c3*u**3+c4*u**4+c5*u**5

    experiments = {}
    for label, times, velocities in (
            ('raw', ts, v), ('fixed_knots_only', np.arange(6)*DT, v),
            ('true_velocity_only', ts, true_v),
            ('fixed_knots_and_true_velocity', np.arange(6)*DT, true_v)):
        samples = np.r_[0., 0., 0., [sample(t*1e-9, times, velocities) for t in sample_ns[:3]]]
        discrete_jerk = np.diff(samples, 3)/DT**3
        phase = []
        for offset in np.linspace(0., DT-1e-7, 1001):
            samples = np.r_[0., 0., 0., [sample(t, times, velocities)
                                       for t in np.arange(DT, .03201, DT)+offset]]
            phase.append(float(np.max(abs(np.diff(samples, 3)/DT**3))))
        experiments[label] = dict(first_3_discrete_jerk_rad_s3=discrete_jerk.tolist(),
            phase_scan_max_abs_jerk_rad_s3=max(phase),
            worst_phase_s=float(np.argmax(phase)*(DT-1e-7)/1000))
    return dict(available=True, joint=msg['joint_names'][axis],
        constant_jerk_rad_s3=float(jerk), observed_sample_relative_ns=sample_ns[:3],
        knots=dict(relative_time_s=ts.tolist(), delta_q=q.tolist(), published_velocity=v.tolist(),
                   constant_jerk_expected_velocity=true_v.tolist(), acceleration=a.tolist()),
        experiments=experiments,
        scope='Captured first four constant-jerk startup steps only; 1001 phase offsets, not a global motion proof')


def analyze(capture_path, audit_path, state_path):
    capture, audit = read_lines(capture_path), read_lines(audit_path)
    if not capture or capture[0].get('mode') != 'fake':
        raise ValueError('Only explicit fake capture evidence is supported')
    if capture[-1].get('kind') != 'session_end' or capture[-1].get('overflow_frames'):
        raise ValueError('Capture incomplete or overflowed')
    report = json.loads(state_path.read_text())
    writes = [row for row in audit if row.get('event') == 'write']
    rejected = next((row for row in writes if row['result'] == 'rejected'), None)
    cutoff = rejected['host_monotonic_ns']/1e9 if rejected else float('inf')
    follows = [row['host_monotonic_s'] for row in report['supervisor_mode_history']
               if row['mode'] == 'FOLLOW' and row['host_monotonic_s'] <= cutoff]
    if not follows:
        raise ValueError('No FOLLOW entry preceding the first rejection')
    follow_ns = int(follows[-1]*1e9)
    raw = [row['message'] for row in capture if row.get('topic') == '/v3/servo_trajectory']
    forwarded = [row['message'] for row in capture if row.get('topic') == '/arm_controller/joint_trajectory']
    messages = [row for row in capture if row.get('topic') == '/arm_controller/joint_trajectory'
                and row['host_monotonic_ns'] >= follow_ns]
    states = [row for row in capture if row.get('topic') == '/arm_controller/controller_state'
              and row['host_monotonic_ns'] >= follow_ns]
    reconstruction = reconstruct(states, messages, writes)
    recent = []
    for row in writes:
        if row['host_monotonic_ns'] < follow_ns:
            continue
        velocity = (np.array(row['command'])-row['guard_previous'])/DT
        acceleration = (velocity-row['guard_previous_velocity'])/DT
        jerk = (acceleration-row['guard_previous_acceleration'])/DT
        recent.append(dict(index=row['index'], result=row['result'], q=row['command'],
                           v=velocity.tolist(), a=acceleration.tolist(), j=jerk.tolist()))
    return dict(physical_acceptance=False, analysis_scope='Offline fake capture; no ROS or SDK calls',
        inputs={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (capture_path, audit_path, state_path)},
        capture_frames=capture[-1]['frames'], capture_overflow=capture[-1]['overflow_frames'],
        raw_forwarded_identical=raw == forwarded, raw_count=len(raw), forwarded_count=len(forwarded),
        first_rejection_index=rejected['index'] if rejected else None,
        follow_start_host_ns=follow_ns, final_write_after_follow=recent,
        jtc_reconstruction=reconstruction,
        startup_counterfactual=startup_counterfactual(messages, reconstruction),
        limitations=[
            'Topic receipt time does not prove controller callback order; identical overlapping knots can match multiple messages.',
            'Output reconstruction uses controller header + 8 ms, never subscriber arrival time as the commanded interval.',
            'Counterfactual velocities are derived only when the captured position and acceleration prefix proves constant jerk.',
            'Instantaneous quintic jerk and the final Guard discrete third difference are distinct quantities.',
            'A phase scan of this prefix does not establish global jerk or collision safety, or physical robot behavior.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, default=ROOT/'artifacts/servo_handoff_capture.jsonl')
    parser.add_argument('--audit', type=Path, default=ROOT/'artifacts/guarded_smoke_output.jsonl')
    parser.add_argument('--state-report', type=Path, default=ROOT/'artifacts/ros_smoke_pass.json')
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/servo_handoff_analysis.json')
    args = parser.parse_args()
    result = analyze(args.capture, args.audit, args.state_report)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    errors = [row['max_output_error_rad'] for row in result['jtc_reconstruction']]
    print(json.dumps(dict(output=str(args.output), first_rejection=result['first_rejection_index'],
        reconstructed_outputs=len(errors), max_reconstruction_error_rad=max(errors, default=None),
        counterfactual=result['startup_counterfactual']), indent=2))


if __name__ == '__main__':
    main()
