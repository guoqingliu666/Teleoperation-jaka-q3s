"""Opt-in fake-mode observation of supervisor action calls; no goal mutations.

For an isolated mock run only, prepend this directory to PYTHONPATH and set
V3_CAPTURE_FJT to an absolute JSONL path under this checkout's artifacts/.
The normal environment does not load this module. The hook forwards every call
unchanged and records only v3_supervisor nodes whose ROS mode parameter is fake.
"""
import os


def install_capture(destination):
    import json
    from pathlib import Path
    import sys
    import time

    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from rosidl_runtime_py.convert import message_to_ordereddict

    root = Path(__file__).resolve().parents[2]
    path = Path(destination).resolve()
    if not path.is_relative_to(root / 'artifacts'):
        raise ValueError('V3_CAPTURE_FJT must be inside this checkout artifacts/')
    original_send = ActionClient.send_goal_async
    original_cancel = ActionClient._cancel_goal_async

    def eligible(client):
        try:
            node = client._node
            return (node.get_name() == 'v3_supervisor'
                    and node.get_parameter('mode').value == 'fake')
        except Exception:
            return False

    def emit(client, kind, **payload):
        try:
            node = client._node
            data = dict(kind=kind, pid=os.getpid(), wall_ns=time.time_ns(),
                        ros_ns=node.get_clock().now().nanoseconds,
                        action=client._action_name, supervisor_mode=getattr(node, 'mode', None),
                        **payload)
            with path.open('a', encoding='utf8') as stream:
                stream.write(json.dumps(data, separators=(',', ':')) + '\n')
        except Exception as exc:
            print(f'V3 diagnostic capture failed: {exc}', file=sys.stderr)

    def send(self, goal, feedback_callback=None, goal_uuid=None):
        observe = isinstance(goal, FollowJointTrajectory.Goal) and eligible(self)
        before = self._node.get_clock().now().nanoseconds if observe else None
        # Original invocation, goal object, callback, UUID and returned Future
        # remain unchanged. Diagnostic serialization happens after dispatch.
        future = original_send(self, goal, feedback_callback=feedback_callback, goal_uuid=goal_uuid)
        if observe:
            try:
                after = self._node.get_clock().now().nanoseconds
                node = self._node
                emit(self, 'goal_send', send_before_ros_ns=before, send_after_ros_ns=after,
                     goal=message_to_ordereddict(goal),
                     measured_q=list(node.q) if getattr(node, 'q', None) is not None else None,
                     measured_v=list(node.v) if getattr(node, 'v', None) is not None else None,
                     generation=getattr(getattr(node, 'ticket', None), 'generation', None))

                def accepted(done):
                    try:
                        handle = done.result()
                        emit(self, 'goal_ack', accepted=handle.accepted,
                             uuid=[int(value) for value in handle.goal_id.uuid], send_before_ros_ns=before)
                    except Exception as exc:
                        emit(self, 'goal_ack_error', error=str(exc), send_before_ros_ns=before)
                future.add_done_callback(accepted)
            except Exception as exc:
                print(f'V3 diagnostic capture failed: {exc}', file=sys.stderr)
        return future

    def cancel(self, goal_handle):
        observe = eligible(self)
        before = self._node.get_clock().now().nanoseconds if observe else None
        future = original_cancel(self, goal_handle)
        if observe:
            emit(self, 'goal_cancel', send_before_ros_ns=before,
                 uuid=[int(value) for value in goal_handle.goal_id.uuid])
        return future

    ActionClient.send_goal_async = send
    ActionClient._cancel_goal_async = cancel


if os.environ.get('V3_CAPTURE_FJT'):
    install_capture(os.environ['V3_CAPTURE_FJT'])
