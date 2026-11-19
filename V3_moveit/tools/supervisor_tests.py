"""Supervisor callback regressions using ROS messages, without Node/DDS or hardware.

Run after sourcing ROS Jazzy: python3 -B tools/supervisor_tests.py.
These are mocked callback tests, not ROS integration or physical acceptance.
"""
import copy
import json
from concurrent.futures import Future
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from moveit_msgs.msg import (CollisionObject, PlanningScene, AttachedCollisionObject,
                             AllowedCollisionEntry, LinkPadding, LinkScale, ObjectColor)
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetMotionPlan
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

# Exercise this checkout even when the installed package has not been rebuilt.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/v3_moveit'))
from v3_moveit.coordinator import Epoch
from v3_moveit.supervisor import Supervisor, SceneEnvironment


JOINTS = [f'joint_{i}' for i in range(1, 7)]
MONOTONIC = 100.0


class ShutdownTests(unittest.TestCase):
    def test_conversion_error_only_suppressed_after_context_shutdown(self):
        from v3_moveit import supervisor as module
        expected="Unable to convert call argument '0' to Python object (#define PYBIND11_DETAILED_ERROR_MESSAGES or compile in debug mode for details)"
        for context_ok,error,raises in [(False,expected,False),(True,expected,True),(False,'different runtime failure',True),(True,'different runtime failure',True)]:
            with self.subTest(context_ok=context_ok,error=error):
                node=Mock();node.context.ok.return_value=context_ok
                with patch.object(module,'Supervisor',return_value=node),patch.object(module.rclpy,'init'),patch.object(module.rclpy,'try_shutdown'),patch.object(module.rclpy,'spin',side_effect=RuntimeError(error)):
                    if raises:
                        with self.assertRaises(RuntimeError):module.main()
                    else:module.main()
                node.destroy_node.assert_called_once()
                self.assertEqual(node.lease.publish.call_count,1 if context_ok else 0)


def setUpModule():
    global _clock_patch
    _clock_patch=patch('v3_moveit.supervisor.time.monotonic', return_value=MONOTONIC)
    _clock_patch.start()


def tearDownModule():
    _clock_patch.stop()


def pending_future():
    future = Future()
    future.cancel = Mock(wraps=future.cancel)
    return future


def supervisor_harness():
    """Bind production callbacks without initializing a ROS node or executor."""
    goal = PoseStamped()
    goal.header.frame_id = 'base_link'
    goal.header.stamp = Time(sec=42)
    goal.pose.orientation.w = 1.0
    now = SimpleNamespace(nanoseconds=42_000_000_000, to_msg=lambda: Time(sec=42))
    node = SimpleNamespace(
        mode='BRAKING', epoch=Epoch(), q=np.zeros(6), v=np.zeros(6), held=True,
        goal=goal, reference_goal=copy.deepcopy(goal), plan_goal=copy.deepcopy(goal),
        goal_at=MONOTONIC, stable_at=MONOTONIC, input_at=MONOTONIC, input_seen=True,
        scene_at=MONOTONIC, joint_at=MONOTONIC, servo_at=MONOTONIC,
        scene_ready=True, quiet_since=MONOTONIC - 0.2, code=0,
        command_q=np.zeros(6), command_at=MONOTONIC, command_stamp_ns=42_000_000_000,
        command_quiet_since=MONOTONIC-.2, command_quiet_q=np.zeros(6), plan_start_command=np.zeros(6),
        plan_future=None, execute_handle=None, cancel_pending=None,
        execute_future=None, execute_result_future=None,
        service_future=None, brake_started=0.0, resume_started=MONOTONIC, last_replan=-float('inf'),
        scene_environment=SceneEnvironment(), scene_version='', plan=Mock(spec=['service_is_ready', 'call_async']),
        execute=Mock(spec=['server_is_ready', 'send_goal_async']),
        pause=Mock(spec=['service_is_ready', 'call_async']),
        pose_pub=Mock(), lease=Mock(), state=Mock(), get_logger=Mock(return_value=Mock()),
        get_clock=Mock(return_value=SimpleNamespace(now=lambda: now)),
        get_parameter=Mock(return_value=SimpleNamespace(value=0.3)),
    )
    node.plan.service_is_ready.return_value = True
    node.execute.server_is_ready.return_value = True
    node.pause.service_is_ready.return_value = True
    node.pause.call_async.return_value = pending_future()
    node.execute.send_goal_async.return_value = pending_future()
    node.ticket = node.epoch.ticket(node.q)
    for name in ('input', 'hand', 'scene', 'scene_update', 'cancel', 'resume', 'begin_plan',
                 'send_plan', 'planned', 'executed', 'controller_fault', 'fault', 'tick',
                 'command_state', 'command_ready', 'physical_stopped', 'start_resume_if_ready'):
        setattr(node, name, MethodType(getattr(Supervisor, name), node))
    return node


def successful_response():
    response = GetMotionPlan.Response()
    result = response.motion_plan_response
    result.error_code.val = 1
    result.trajectory.joint_trajectory.joint_names = JOINTS
    first = JointTrajectoryPoint(positions=[0.0] * 6, velocities=[0.0] * 6, accelerations=[0.0] * 6)
    last = JointTrajectoryPoint(positions=[0.02] * 6, velocities=[0.0] * 6, accelerations=[0.0] * 6)
    last.time_from_start.sec = 1
    result.trajectory.joint_trajectory.points = [first, last]
    return response


def action_handle():
    terminal, cancellation = pending_future(), pending_future()
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = terminal
    handle.cancel_goal_async.return_value = cancellation
    return handle, terminal, cancellation


def action_outcome(status=4, error_code=0):
    result = FollowJointTrajectory.Result(error_code=error_code)
    return SimpleNamespace(status=status, result=result)


def tick_now(node):
    # Deterministic healthy feedback and quiet state, without an executor or sleeps.
    with patch('v3_moveit.supervisor.time.monotonic', return_value=MONOTONIC):
        node.tick()


class SupervisorCommandStartTests(unittest.TestCase):
    @staticmethod
    def state(stamp_ns=42_000_000_000, q=None):
        msg=JointTrajectoryControllerState();msg.joint_names=list(JOINTS)
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp_ns,10**9)
        values=[0.]*6 if q is None else list(q)
        msg.output.positions=values  # Position-only hardware has no output v/a.
        msg.reference.positions=values
        msg.reference.velocities=[0.]*6;msg.reference.accelerations=[0.]*6
        return msg

    @staticmethod
    def deliver(node,msg,now=MONOTONIC,ros_ns=None):
        if ros_ns is None:ros_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        node.get_clock.return_value=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=ros_ns))
        with patch('v3_moveit.supervisor.time.monotonic',return_value=now):node.command_state(msg)

    def ready_from_callbacks(self,q=None):
        node=supervisor_harness();node.command_q=None;node.command_stamp_ns=None
        node.command_at=-float('inf');node.command_quiet_since=None;node.command_quiet_q=None
        for i in range(13):
            offset=(i-12)*8_000_000
            self.deliver(node,self.state(42_000_000_000+offset,q),MONOTONIC+offset/1e9)
        self.assertTrue(node.command_ready())
        return node

    def test_position_only_output_with_continuous_stationary_reference_is_ready(self):
        node=self.ready_from_callbacks([.1,-.2,.3,-.4,.5,-.6])
        np.testing.assert_array_equal(node.command_q,[.1,-.2,.3,-.4,.5,-.6])

    def test_feedback_noise_does_not_change_frozen_command_planning_start(self):
        node=self.ready_from_callbacks()
        node.q=np.array([8.7e-5,-8.7e-5,0.,0.,0.,0.])
        future=pending_future();node.plan.call_async.return_value=future
        node.begin_plan();self.assertEqual(node.mode,'PAUSING')
        np.testing.assert_array_equal(node.plan_start_command,np.zeros(6))
        self.assertEqual(node.ticket.start,tuple(node.q))  # Measured-state epoch stays independent.
        node.send_plan();request=node.plan.call_async.call_args.args[0].motion_plan_request
        self.assertEqual(list(request.start_state.joint_state.position),[0.]*6)
        self.assertEqual(list(request.start_state.joint_state.velocity),[0.]*6)
        self.assertTrue(request.start_state.is_diff)
        future.set_result(successful_response())
        node.execute.send_goal_async.assert_called_once()

    def test_missing_stale_moving_or_unsettled_command_cannot_begin_plan(self):
        for issue in ('missing','stale','moving','unsettled','physical_stale','physical_moving'):
            with self.subTest(issue=issue):
                node=supervisor_harness()
                if issue=='missing':node.command_q=None
                elif issue=='stale':node.command_at=MONOTONIC-.025
                elif issue=='moving':node.command_quiet_since=None
                elif issue=='unsettled':node.command_quiet_since=MONOTONIC-.079
                elif issue=='physical_stale':node.joint_at=MONOTONIC-.025
                else:node.v[0]=.003
                node.begin_plan();self.assertEqual(node.mode,'BRAKING')
                node.pause.call_async.assert_not_called();node.plan.call_async.assert_not_called()

    def test_moving_reference_or_mismatch_invalidates_stationary_window(self):
        for field in ('positions','velocities','accelerations'):
            with self.subTest(field=field):
                node=self.ready_from_callbacks();msg=self.state(42_008_000_000)
                getattr(msg.reference,field)[0]=1e-6
                self.deliver(node,msg,MONOTONIC+.008)
                self.assertIsNone(node.command_quiet_since)
                self.assertFalse(node.command_ready())

    def test_malformed_nonfinite_replayed_or_bad_stamp_invalidates_command(self):
        for issue in ('joint_names','missing_acceleration','nonfinite','replay','past','future'):
            with self.subTest(issue=issue):
                node=self.ready_from_callbacks();msg=self.state(42_008_000_000);ros_ns=42_008_000_000
                if issue=='joint_names':msg.joint_names[0]=msg.joint_names[1]
                elif issue=='missing_acceleration':msg.reference.accelerations=[]
                elif issue=='nonfinite':msg.output.positions[0]=float('nan')
                elif issue=='replay':msg.header.stamp.nanosec=0
                elif issue=='past':ros_ns+=24_000_001
                else:ros_ns-=24_000_001
                self.deliver(node,msg,MONOTONIC+.008,ros_ns)
                self.assertIsNone(node.command_q);self.assertFalse(node.command_ready())

    def test_command_timestamp_tolerance_does_not_override_receipt_timeout(self):
        node=self.ready_from_callbacks();msg=self.state(42_008_000_000)
        self.deliver(node,msg,MONOTONIC+.008,42_000_000_000)
        with patch('v3_moveit.supervisor.time.monotonic',return_value=MONOTONIC+.008):
            self.assertTrue(node.command_ready())
        with patch('v3_moveit.supervisor.time.monotonic',return_value=MONOTONIC+.033):
            self.assertFalse(node.command_ready())

    def test_receipt_gap_and_output_drift_reset_full_eighty_ms_window(self):
        for gap in (False,True):
            with self.subTest(gap=gap):
                node=self.ready_from_callbacks();offset=.025 if gap else .008
                msg=self.state(42_000_000_000+round(offset*1e9))
                if not gap:
                    msg.output.positions[0]=2e-10;msg.reference.positions[0]=2e-10
                self.deliver(node,msg,MONOTONIC+offset)
                self.assertAlmostEqual(node.command_quiet_since,MONOTONIC+offset)
                with patch('v3_moveit.supervisor.time.monotonic',return_value=MONOTONIC+offset):
                    self.assertFalse(node.command_ready())

    def test_joint_order_is_normalized_to_command_order(self):
        node=supervisor_harness();node.command_stamp_ns=None
        msg=self.state(q=[1.,2.,3.,4.,5.,6.]);msg.joint_names=list(reversed(JOINTS))
        self.deliver(node,msg)
        np.testing.assert_array_equal(node.command_q,[6.,5.,4.,3.,2.,1.])

    def test_command_change_during_pause_discards_request_and_keeps_frozen_copy(self):
        node=supervisor_harness();node.begin_plan();node.command_q[0]=8.7e-5
        self.assertEqual(node.plan_start_command[0],0.)
        node.send_plan();node.plan.call_async.assert_not_called()
        self.assertEqual(node.mode,'RESUMING')

    def test_command_change_or_staleness_before_plan_reply_discards_result(self):
        for issue in ('changed','stale','moving'):
            with self.subTest(issue=issue):
                node=supervisor_harness();future=pending_future();node.plan.call_async.return_value=future
                node.send_plan()
                if issue=='changed':node.command_q[0]=8.7e-5
                elif issue=='stale':node.command_at=MONOTONIC-.025
                else:node.command_quiet_since=None
                future.set_result(successful_response())
                node.execute.send_goal_async.assert_not_called();self.assertEqual(node.mode,'RESUMING')

    def test_plan_first_point_must_match_exact_command_not_measured_noise(self):
        node=supervisor_harness();node.q[0]=8.7e-5;node.ticket=node.epoch.ticket(node.q)
        future=pending_future();node.plan.call_async.return_value=future;node.send_plan()
        response=successful_response();response.motion_plan_response.trajectory.joint_trajectory.points[0].positions[0]=8.7e-5
        future.set_result(response)
        node.execute.send_goal_async.assert_not_called();self.assertEqual(node.mode,'RESUMING')


class SupervisorResumeTests(unittest.TestCase):
    def test_terminal_result_waits_for_command_and_actual_rest(self):
        for reason in ('command_moving','command_unsettled','physical_moving','physical_unsettled'):
            with self.subTest(reason=reason):
                node=supervisor_harness();node.mode='EXECUTING'
                terminal=pending_future();node.execute_result_future=terminal
                if reason=='command_moving':node.command_quiet_since=None
                elif reason=='command_unsettled':node.command_quiet_since=MONOTONIC-.079
                elif reason=='physical_moving':node.v[0]=.003
                else:node.quiet_since=MONOTONIC-.079
                terminal.set_result(action_outcome());node.executed(terminal)
                self.assertEqual(node.mode,'RESUMING');node.pause.call_async.assert_not_called()
                node.v[:]=0.;node.quiet_since=MONOTONIC-.2;node.command_quiet_since=MONOTONIC-.2
                tick_now(node);node.pause.call_async.assert_called_once()
                self.assertFalse(node.pause.call_async.call_args.args[0].data)
                tick_now(node);node.pause.call_async.assert_called_once()

    def test_resume_deadline_faults_without_service_or_rest(self):
        node=supervisor_harness();node.command_quiet_since=None;node.resume()
        node.resume_started=MONOTONIC-1.501;tick_now(node)
        self.assertEqual(node.mode,'FAULT');node.pause.call_async.assert_not_called()
        self.assertFalse(node.lease.publish.call_args.args[0].data)

    def test_resume_exception_revokes_lease(self):
        node=supervisor_harness();node.pause.call_async.side_effect=RuntimeError('service unavailable')
        node.resume();self.assertEqual(node.mode,'FAULT')
        self.assertFalse(node.lease.publish.call_args.args[0].data)


class SupervisorPlanningTests(unittest.TestCase):
    def inflight(self):
        node = supervisor_harness()
        future = pending_future()
        node.plan.call_async.return_value = future
        node.send_plan()
        self.assertIs(node.plan_future, future)
        self.assertEqual(node.mode, 'PLANNING')
        return node, future

    def assert_discarded(self, node, future):
        future.set_result(successful_response())
        self.assertIsNone(node.plan_future)
        node.execute.send_goal_async.assert_not_called()
        future.cancel.assert_not_called()

    def test_grip_release_discards_delayed_plan_without_canceling_planner(self):
        node, future = self.inflight()
        node.input(String(data=json.dumps({'held': False, 'age_s': 0.0})))
        self.assertFalse(node.held)
        self.assertEqual(node.mode, 'CANCELING')
        self.assert_discarded(node, future)
        self.assertEqual(node.mode, 'CANCELING')

    def test_changed_hand_target_discards_delayed_plan(self):
        node, future = self.inflight()
        changed_goal = copy.deepcopy(node.goal)
        changed_goal.pose.position.x += 0.01
        node.hand(changed_goal)
        self.assertGreater(node.epoch.generation, node.ticket.generation)
        self.assertEqual(node.mode, 'CANCELING')
        self.assert_discarded(node, future)

    def test_scene_change_discards_delayed_plan(self):
        node, future = self.inflight()
        scene = PlanningScene()
        scene.world.collision_objects = [CollisionObject(id='new_obstacle')]
        node.scene_update(scene)
        self.assertGreater(node.epoch.scene, node.ticket.scene)
        self.assertEqual(node.mode, 'CANCELING')
        self.assert_discarded(node, future)

    def test_invalid_ticket_in_planning_resumes_without_motion(self):
        for invalidation in ('target', 'scene', 'released', 'moved_start'):
            with self.subTest(invalidation=invalidation):
                node, future = self.inflight()
                if invalidation == 'target':
                    node.epoch.invalidate()
                elif invalidation == 'scene':
                    node.epoch.scene_changed()
                elif invalidation == 'released':
                    node.held = False
                else:
                    node.q[0] = 0.02
                self.assert_discarded(node, future)
                self.assertEqual(node.mode, 'RESUMING')
                self.assertFalse(node.pause.call_async.call_args.args[0].data)

    def test_late_reply_after_resuming_does_not_restart_execution(self):
        node, future = self.inflight()
        node.cancel('target canceled')
        node.resume()
        # The Servo resume may finish before the older planning service responds.
        node.mode = 'FOLLOW'
        self.assert_discarded(node, future)
        self.assertEqual(node.mode, 'FOLLOW')

    def test_unfinished_plan_blocks_another_request_after_cancel(self):
        node, future = self.inflight()
        node.cancel('target changed')
        node.mode = 'BRAKING'
        old_ticket = node.ticket
        node.begin_plan()
        self.assertIs(node.plan_future, future)
        self.assertIs(node.ticket, old_ticket)
        self.assertEqual(node.mode, 'BRAKING')
        node.pause.call_async.assert_not_called()
        self.assertEqual(node.plan.call_async.call_count, 1)
        future.cancel.assert_not_called()

    def test_discarded_reply_releases_slot_for_next_plan(self):
        node, future = self.inflight()
        node.cancel('target changed')
        self.assert_discarded(node, future)
        node.mode = 'BRAKING'
        node.begin_plan()
        self.assertEqual(node.mode, 'PAUSING')
        self.assertEqual(node.ticket.generation, node.epoch.generation)
        self.assertTrue(node.pause.call_async.call_args.args[0].data)
        self.assertEqual(node.plan.call_async.call_count, 1)

    def test_unrelated_old_callback_cannot_clear_pending_request(self):
        node, current = self.inflight()
        old = pending_future()
        old.set_result(successful_response())
        node.planned(old, node.ticket)
        self.assertIs(node.plan_future, current)
        self.assertEqual(node.mode, 'PLANNING')
        node.execute.send_goal_async.assert_not_called()
        node.pause.call_async.assert_not_called()

    def test_valid_service_plan_is_submitted_to_joint_trajectory_controller(self):
        node, future = self.inflight()
        request = node.plan.call_async.call_args.args[0]
        self.assertIsInstance(request, GetMotionPlan.Request)
        self.assertEqual(request.motion_plan_request.group_name, 'arm')
        self.assertTrue(request.motion_plan_request.start_state.is_diff)
        self.assertEqual(request.motion_plan_request.start_state.joint_state.name, JOINTS)
        self.assertEqual(list(request.motion_plan_request.start_state.joint_state.velocity), [0.0] * 6)
        future.set_result(successful_response())
        self.assertIsNone(node.plan_future)
        self.assertEqual(node.mode, 'EXECUTING')
        node.execute.send_goal_async.assert_called_once()
        goal = node.execute.send_goal_async.call_args.args[0]
        self.assertIsInstance(goal, FollowJointTrajectory.Goal)
        self.assertEqual(goal.trajectory.joint_names, JOINTS)
        self.assertEqual(list(goal.trajectory.points[-1].positions), [0.02] * 6)
        self.assertEqual(goal.trajectory.header.stamp.sec, 0)
        self.assertEqual(goal.trajectory.header.stamp.nanosec, 0)
        future.cancel.assert_not_called()

    def test_executing_trajectory_still_sends_action_cancel(self):
        node = supervisor_harness()
        node.mode = 'EXECUTING'
        cancellation = pending_future()
        handle = Mock()
        handle.cancel_goal_async.return_value = cancellation
        node.execute_handle = handle
        generation = node.epoch.generation
        node.cancel('grip released')
        handle.cancel_goal_async.assert_called_once_with()
        self.assertIs(node.cancel_pending, cancellation)
        self.assertIsNone(node.execute_handle)
        self.assertGreater(node.epoch.generation, generation)
        self.assertEqual(node.mode, 'CANCELING')

    def test_trajectory_accepted_after_cancel_is_immediately_canceled(self):
        node, future = self.inflight()
        future.set_result(successful_response())
        accepted = node.execute.send_goal_async.return_value
        node.cancel('grip released before controller acceptance')
        handle, terminal, cancellation = action_handle()
        accepted.set_result(handle)
        handle.cancel_goal_async.assert_called_once_with()
        handle.get_result_async.assert_called_once_with()
        self.assertIs(node.execute_result_future, terminal)
        self.assertIs(node.cancel_pending, cancellation)
        self.assertIsNone(node.execute_handle)
        self.assertEqual(node.mode, 'CANCELING')

    def test_planning_service_exception_resumes_without_motion(self):
        node, future = self.inflight()
        future.set_exception(RuntimeError('planner service unavailable'))
        self.assertIsNone(node.plan_future)
        self.assertEqual(node.mode, 'RESUMING')
        node.execute.send_goal_async.assert_not_called()
        self.assertFalse(node.pause.call_async.call_args.args[0].data)
        node.get_logger.return_value.error.assert_called_once()

    def test_unsuccessful_or_empty_plan_resumes_without_motion(self):
        for failure in ('error', 'empty'):
            with self.subTest(failure=failure):
                node, future = self.inflight()
                response = successful_response()
                if failure == 'error':
                    response.motion_plan_response.error_code.val = -1
                else:
                    response.motion_plan_response.trajectory.joint_trajectory.points = []
                future.set_result(response)
                self.assertEqual(node.mode, 'RESUMING')
                node.execute.send_goal_async.assert_not_called()

    def test_nonfinite_plan_faults_without_motion(self):
        node, future = self.inflight()
        response = successful_response()
        response.motion_plan_response.trajectory.joint_trajectory.points[-1].positions[0] = float('nan')
        future.set_result(response)
        self.assertEqual(node.mode, 'FAULT')
        node.execute.send_goal_async.assert_not_called()
        self.assertFalse(node.lease.publish.call_args.args[0].data)


class SupervisorActionRaceTests(unittest.TestCase):
    def submitted(self):
        node = supervisor_harness()
        plan = pending_future()
        node.plan.call_async.return_value = plan
        node.send_plan()
        plan.set_result(successful_response())
        self.assertEqual(node.mode, 'EXECUTING')
        return node, node.execute.send_goal_async.return_value

    def accepted(self):
        node, acceptance = self.submitted()
        handle, terminal, cancellation = action_handle()
        acceptance.set_result(handle)
        self.assertIs(node.execute_handle, handle)
        self.assertIs(node.execute_result_future, terminal)
        return node, handle, terminal, cancellation

    def test_controller_fault_revokes_lease_before_fallible_cancel(self):
        node, handle, terminal, _ = self.accepted()
        generation = node.epoch.generation
        def failing_cancel():
            self.assertEqual(node.mode, 'FAULT')
            self.assertFalse(node.lease.publish.call_args.args[0].data)
            self.assertGreater(node.epoch.generation, generation)
            raise RuntimeError('cancel transport failed')
        handle.cancel_goal_async.side_effect = failing_cancel
        node.controller_fault(String(data='Ruckig output rejected'))
        self.assertEqual(node.mode, 'FAULT')
        self.assertIsNone(node.execute_handle)
        self.assertIs(node.execute_result_future, terminal)
        handle.cancel_goal_async.assert_called_once_with()
        terminal.set_result(action_outcome(status=5))
        self.assertEqual(node.mode, 'FAULT')
        node.pause.call_async.assert_not_called()
        self.assertFalse(node.lease.publish.call_args.args[0].data)

    def test_fault_cannot_resume_replan_or_tick_back_to_motion(self):
        node = supervisor_harness()
        node.controller_fault(String(data='command generation failed'))
        node.resume()
        node.begin_plan()
        node.send_plan()
        tick_now(node)
        self.assertEqual(node.mode, 'FAULT')
        node.plan.call_async.assert_not_called()
        node.pause.call_async.assert_not_called()
        node.pose_pub.publish.assert_not_called()
        self.assertTrue(node.lease.publish.called)
        self.assertTrue(all(not call.args[0].data for call in node.lease.publish.call_args_list))

    def test_late_plan_after_controller_fault_is_discarded(self):
        node = supervisor_harness()
        planning = pending_future()
        node.plan.call_async.return_value = planning
        node.send_plan()
        node.controller_fault(String(data='controller error'))
        planning.set_result(successful_response())
        self.assertEqual(node.mode, 'FAULT')
        self.assertIsNone(node.plan_future)
        node.execute.send_goal_async.assert_not_called()
        node.pause.call_async.assert_not_called()

    def test_late_rejected_acceptance_cannot_resume_fault(self):
        node, acceptance = self.submitted()
        node.controller_fault(String(data='controller error'))
        acceptance.set_result(Mock(accepted=False))
        self.assertEqual(node.mode, 'FAULT')
        self.assertIsNone(node.execute_future)
        node.pause.call_async.assert_not_called()

    def test_late_accepted_goal_after_fault_is_tracked_and_canceled(self):
        node, acceptance = self.submitted()
        node.controller_fault(String(data='controller error'))
        handle, terminal, cancellation = action_handle()
        acceptance.set_result(handle)
        self.assertEqual(node.mode, 'FAULT')
        self.assertIsNone(node.execute_handle)
        self.assertIs(node.execute_result_future, terminal)
        self.assertIs(node.cancel_pending, cancellation)
        handle.cancel_goal_async.assert_called_once_with()
        terminal.set_result(action_outcome(status=5))
        cancellation.set_result(SimpleNamespace(goals_canceling=[]))
        tick_now(node)
        self.assertEqual(node.mode, 'FAULT')
        node.pause.call_async.assert_not_called()
        self.assertFalse(node.lease.publish.call_args.args[0].data)

    def test_completed_result_during_acceptance_does_not_resurrect_or_cancel_handle(self):
        node, acceptance = self.submitted()
        handle, terminal, _ = action_handle()
        terminal.set_result(action_outcome())
        # Future.add_done_callback runs inline for an already completed result.
        acceptance.set_result(handle)
        self.assertIsNone(node.execute_future)
        self.assertIsNone(node.execute_result_future)
        self.assertIsNone(node.execute_handle)
        self.assertEqual(node.mode, 'RESUMING')
        node.pause.call_async.assert_called_once()
        handle.cancel_goal_async.assert_not_called()

    def test_unrelated_terminal_callback_cannot_clear_current_execution(self):
        for stale_failure in (False, True):
            with self.subTest(stale_failure=stale_failure):
                node, handle, current, _ = self.accepted()
                old = pending_future()
                if stale_failure:
                    old.set_exception(RuntimeError('old failed goal'))
                else:
                    old.set_result(action_outcome())
                node.executed(old)
                self.assertEqual(node.mode, 'EXECUTING')
                self.assertIs(node.execute_handle, handle)
                self.assertIs(node.execute_result_future, current)
                node.pause.call_async.assert_not_called()
                node.lease.publish.assert_not_called()

    def test_result_registration_failure_revokes_lease_and_cancels_accepted_goal(self):
        node, acceptance = self.submitted()
        handle, _, _ = action_handle()
        handle.get_result_async.side_effect = RuntimeError('result transport unavailable')
        acceptance.set_result(handle)
        self.assertEqual(node.mode, 'FAULT')
        self.assertFalse(node.lease.publish.call_args.args[0].data)
        self.assertIsNone(node.execute_handle)
        handle.cancel_goal_async.assert_called_once_with()
        node.pause.call_async.assert_not_called()

    def test_acceptance_or_terminal_future_exception_latches_fault(self):
        for phase in ('acceptance', 'terminal'):
            with self.subTest(phase=phase):
                if phase == 'acceptance':
                    node, future = self.submitted()
                else:
                    node, _, future, _ = self.accepted()
                future.set_exception(RuntimeError('action transport unavailable'))
                self.assertEqual(node.mode, 'FAULT')
                self.assertFalse(node.lease.publish.call_args.args[0].data)
                node.pause.call_async.assert_not_called()

    def test_current_terminal_success_resumes_exactly_once(self):
        node, _, terminal, _ = self.accepted()
        terminal.set_result(action_outcome())
        self.assertIsNone(node.execute_handle)
        self.assertIsNone(node.execute_result_future)
        self.assertEqual(node.mode, 'RESUMING')
        node.executed(terminal)  # Duplicate delivery must be a no-op.
        node.pause.call_async.assert_called_once()

    def test_failed_current_result_revokes_lease_without_resuming(self):
        for status, error in ((6, 0), (4, -1), (5, -1)):
            with self.subTest(status=status, error=error):
                node, _, terminal, _ = self.accepted()
                terminal.set_result(action_outcome(status, error))
                self.assertEqual(node.mode, 'FAULT')
                self.assertFalse(node.lease.publish.call_args.args[0].data)
                node.pause.call_async.assert_not_called()

    def test_cancel_waits_for_late_acceptance_response_terminal_and_quiet(self):
        node, acceptance = self.submitted()
        with patch('v3_moveit.supervisor.time.monotonic', return_value=MONOTONIC):
            node.cancel('released before accepted')
        generation = node.epoch.generation
        tick_now(node)
        self.assertEqual(node.mode, 'CANCELING')
        node.pause.call_async.assert_not_called()
        handle, terminal, cancellation = action_handle()
        acceptance.set_result(handle)
        self.assertGreater(node.epoch.generation, node.ticket.generation)
        self.assertEqual(node.epoch.generation, generation)
        tick_now(node)
        self.assertEqual(node.mode, 'CANCELING')
        cancellation.set_result(SimpleNamespace(goals_canceling=[]))
        tick_now(node)  # Cancel acknowledgment alone is not a completed stop.
        self.assertEqual(node.mode, 'CANCELING')
        node.quiet_since = None
        terminal.set_result(action_outcome(status=5))
        tick_now(node)  # Terminal action state alone is not measured quiet.
        self.assertEqual(node.mode, 'CANCELING')
        node.quiet_since = MONOTONIC - 0.2
        tick_now(node)
        self.assertEqual(node.mode, 'RESUMING')
        node.pause.call_async.assert_called_once()
        self.assertIsNone(node.cancel_pending)

    def test_cancel_waits_for_ack_even_if_terminal_arrives_first(self):
        node, _, terminal, cancellation = self.accepted()
        with patch('v3_moveit.supervisor.time.monotonic', return_value=MONOTONIC):
            node.cancel('release')
        terminal.set_result(action_outcome(status=5))
        tick_now(node)
        self.assertEqual(node.mode, 'CANCELING')
        node.pause.call_async.assert_not_called()
        cancellation.set_result(SimpleNamespace(goals_canceling=[]))
        tick_now(node)
        self.assertEqual(node.mode, 'RESUMING')

    def test_repeated_cancel_preserves_canceling_and_original_stop_deadline(self):
        node, _, _, _ = self.accepted()
        with patch('v3_moveit.supervisor.time.monotonic', return_value=MONOTONIC):
            node.cancel('release')
        original = node.brake_started
        with patch('v3_moveit.supervisor.time.monotonic', return_value=MONOTONIC + 0.5):
            node.cancel('input expired again')
        self.assertEqual(node.mode, 'CANCELING')
        self.assertEqual(node.brake_started, original)

    def test_pending_action_lifecycle_blocks_new_plan_and_direct_resume(self):
        for field in ('execute_future', 'execute_result_future'):
            with self.subTest(field=field):
                node = supervisor_harness()
                setattr(node, field, pending_future())
                node.begin_plan()
                node.resume()
                self.assertEqual(node.mode, 'BRAKING')
                node.pause.call_async.assert_not_called()
                node.plan.call_async.assert_not_called()


class SupervisorTrajectoryContractTests(unittest.TestCase):
    def deliver(self, response):
        node = supervisor_harness()
        future = pending_future()
        node.plan.call_async.return_value = future
        node.send_plan()
        future.set_result(response)
        return node

    def assert_faulted(self, response):
        node = self.deliver(response)
        self.assertEqual(node.mode, 'FAULT')
        node.execute.send_goal_async.assert_not_called()
        self.assertFalse(node.lease.publish.call_args.args[0].data)

    def test_nonzero_planner_timestamp_is_cleared_for_controller_start(self):
        response = successful_response()
        response.motion_plan_response.trajectory.joint_trajectory.header.stamp = Time(sec=123, nanosec=456)
        node = self.deliver(response)
        stamp = node.execute.send_goal_async.call_args.args[0].trajectory.header.stamp
        self.assertEqual((stamp.sec, stamp.nanosec), (0, 0))

    def test_invalid_position_velocity_or_acceleration_is_rejected(self):
        for field in ('positions', 'velocities', 'accelerations'):
            for value in ('missing', 'nan', 'inf'):
                with self.subTest(field=field, value=value):
                    response = successful_response()
                    point = response.motion_plan_response.trajectory.joint_trajectory.points[-1]
                    values = [] if value == 'missing' else [float(value)] + [0.0] * 5
                    setattr(point, field, values)
                    self.assert_faulted(response)

    def test_nonstationary_endpoint_is_rejected(self):
        for point_index in (0, -1):
            for field in ('velocities', 'accelerations'):
                with self.subTest(point_index=point_index, field=field):
                    response = successful_response()
                    point = response.motion_plan_response.trajectory.joint_trajectory.points[point_index]
                    getattr(point, field)[0] = 0.001
                    self.assert_faulted(response)

    def test_times_must_start_at_zero_and_strictly_increase(self):
        for times in ((1, 100), (0, 0), (0, -1)):
            with self.subTest(times=times):
                response = successful_response()
                points = response.motion_plan_response.trajectory.joint_trajectory.points
                for point, sec in zip(points, times):
                    point.time_from_start.sec = sec
                self.assert_faulted(response)

    def test_duplicate_joint_name_is_rejected(self):
        response = successful_response()
        response.motion_plan_response.trajectory.joint_trajectory.joint_names = JOINTS + [JOINTS[0]]
        self.assert_faulted(response)


class SupervisorSceneEnvironmentTests(unittest.TestCase):
    @staticmethod
    def box(name, frame='world'):
        obj=CollisionObject(id=name)
        obj.header.frame_id=frame;obj.header.stamp.sec=1
        obj.pose.orientation.w=1.
        pose=Pose();pose.orientation.w=1.
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.1,.1,.1])]
        obj.primitive_poses=[pose]
        return obj

    def full_scene(self):
        msg=PlanningScene()
        msg.robot_model_name='s5'
        msg.world.collision_objects=[self.box('table'),self.box('wall')]
        attached=AttachedCollisionObject(link_name='link_6',object=self.box('tool','link_6'),touch_links=['link_6','tcp'])
        msg.robot_state.attached_collision_objects=[attached]
        acm=msg.allowed_collision_matrix;acm.entry_names=['link_1','link_2']
        acm.entry_values=[AllowedCollisionEntry(enabled=[True,False]),AllowedCollisionEntry(enabled=[False,True])]
        acm.default_entry_names=['tool'];acm.default_entry_values=[False]
        msg.link_padding=[LinkPadding(link_name='link_1',padding=.005)]
        msg.link_scale=[LinkScale(link_name='link_2',scale=1.01)]
        transform=TransformStamped(child_frame_id='world');transform.header.frame_id='camera'
        transform.transform.rotation.w=1.;transform.header.stamp.sec=1
        msg.fixed_frame_transforms=[transform]
        msg.world.octomap.header.frame_id='world';msg.world.octomap.header.stamp.sec=1
        msg.world.octomap.origin.orientation.w=1.
        msg.world.octomap.octomap.id='OcTree';msg.world.octomap.octomap.resolution=.02
        msg.world.octomap.octomap.data=[1,2,3]
        return msg

    def baseline(self,mode='EXECUTING'):
        node=supervisor_harness();node.mode='IDLE'
        msg=self.full_scene();node.scene_update(msg)
        node.mode=mode;node.lease.publish.reset_mock()
        return node,msg

    def assert_fault(self,node,msg):
        generation=node.epoch.generation
        node.scene_update(msg)
        self.assertEqual(node.mode,'FAULT')
        self.assertGreater(node.epoch.generation,generation)
        self.assertFalse(node.lease.publish.call_args.args[0].data)
        node.pause.call_async.assert_not_called()

    def test_static_channels_revoke_execution_and_canceling_lease(self):
        changes={
            'world_pose':lambda m:setattr(m.world.collision_objects[0].pose.position,'x',.2),
            'attached_pose':lambda m:setattr(m.robot_state.attached_collision_objects[0].object.pose.position,'z',.2),
            'touch_links':lambda m:m.robot_state.attached_collision_objects[0].touch_links.append('link_5'),
            'acm':lambda m:m.allowed_collision_matrix.entry_values[0].enabled.__setitem__(1,True),
            'acm_default':lambda m:m.allowed_collision_matrix.default_entry_values.__setitem__(0,True),
            'padding':lambda m:setattr(m.link_padding[0],'padding',.01),
            'scale':lambda m:setattr(m.link_scale[0],'scale',1.02),
            'fixed_transform':lambda m:setattr(m.fixed_frame_transforms[0].transform.translation,'x',.1),
            'octomap_data':lambda m:m.world.octomap.octomap.data.__setitem__(0,4),
            'octomap_origin':lambda m:setattr(m.world.octomap.origin.position,'x',.1),
            'model':lambda m:setattr(m,'robot_model_name','different_model'),
        }
        for mode in ('FOLLOW','BRAKING','EXECUTING','CANCELING'):
            for name,change in changes.items():
                with self.subTest(mode=mode,change=name):
                    node,msg=self.baseline(mode);change(msg);self.assert_fault(node,msg)

    def test_scene_fault_revokes_lease_before_canceling_active_handle(self):
        node,msg=self.baseline();handle,_,_=action_handle();node.execute_handle=handle
        def cancel():
            self.assertEqual(node.mode,'FAULT')
            self.assertFalse(node.lease.publish.call_args.args[0].data)
            return pending_future()
        handle.cancel_goal_async.side_effect=cancel
        msg.world.collision_objects=[]
        self.assert_fault(node,msg)
        handle.cancel_goal_async.assert_called_once_with()

    def test_timestamp_joint_feedback_colors_and_reordering_do_not_invalidate(self):
        node,msg=self.baseline();generation=node.epoch.generation;signature=node.scene_environment.signature
        msg.name='display name only';msg.object_colors=[ObjectColor(id='wall')]
        msg.robot_state.joint_state.name=JOINTS;msg.robot_state.joint_state.position=[.01]*6
        msg.robot_state.joint_state.header.stamp.sec=999
        msg.robot_state.multi_dof_joint_state.header.stamp.sec=999
        for obj in msg.world.collision_objects:obj.header.stamp.sec=999
        msg.world.collision_objects.reverse()
        msg.robot_state.attached_collision_objects[0].object.header.stamp.sec=999
        msg.robot_state.attached_collision_objects[0].detach_posture.header.stamp.sec=999
        msg.robot_state.attached_collision_objects[0].touch_links.reverse()
        msg.fixed_frame_transforms[0].header.stamp.sec=999
        msg.world.octomap.header.stamp.sec=999;msg.world.octomap.octomap.header.stamp.sec=999
        # Reorder both matrix axes consistently: the collision permission is unchanged.
        msg.allowed_collision_matrix.entry_names.reverse()
        msg.allowed_collision_matrix.entry_values.reverse()
        for row in msg.allowed_collision_matrix.entry_values:row.enabled.reverse()
        node.scene_update(msg)
        self.assertEqual(node.mode,'EXECUTING');self.assertEqual(node.epoch.generation,generation)
        self.assertEqual(node.scene_environment.signature,signature)
        node.lease.publish.assert_not_called()

    def test_joint_only_robot_state_diff_does_not_clear_attachments(self):
        node,_=self.baseline();generation=node.epoch.generation
        msg=PlanningScene(is_diff=True);msg.robot_state.is_diff=True
        msg.robot_state.joint_state.name=JOINTS;msg.robot_state.joint_state.position=[.1]*6
        node.scene_update(msg)
        self.assertEqual(node.mode,'EXECUTING');self.assertEqual(node.epoch.generation,generation)
        self.assertIn('tool',node.scene_environment.parts['attached'])

    def test_full_robot_state_clears_missing_attachments_even_in_scene_diff(self):
        node,_=self.baseline();msg=PlanningScene(is_diff=True)
        msg.robot_state.joint_state.name=JOINTS;msg.robot_state.joint_state.position=[0.]*6
        self.assert_fault(node,msg)
        self.assertEqual(node.scene_environment.parts['attached'],{})

    def test_empty_scene_diff_keeps_static_content(self):
        node,_=self.baseline();signature=node.scene_environment.signature
        node.scene_update(PlanningScene(is_diff=True))
        self.assertEqual(node.mode,'EXECUTING');self.assertEqual(node.scene_environment.signature,signature)

    def test_full_empty_scene_replaces_world_acm_attachments_and_octomap(self):
        node,_=self.baseline();self.assert_fault(node,PlanningScene())
        parts=node.scene_environment.parts
        self.assertEqual(parts['world'],{});self.assertEqual(parts['attached'],{})
        self.assertEqual(parts['acm'],dict(entries={},defaults={}));self.assertIsNone(parts['octomap'])
        # These vectors are keyed updates in MoveIt 2.12.4, including full messages.
        self.assertIn('camera',parts['transforms']);self.assertIn('link_1',parts['padding'])

    def test_remove_empty_world_id_removes_all_objects_but_keeps_octomap(self):
        node,_=self.baseline();msg=PlanningScene(is_diff=True)
        msg.world.collision_objects=[CollisionObject(operation=CollisionObject.REMOVE)]
        self.assert_fault(node,msg)
        self.assertEqual(node.scene_environment.parts['world'],{})
        self.assertIsNotNone(node.scene_environment.parts['octomap'])

    def test_octomap_diff_with_id_and_empty_data_clears_map(self):
        node,_=self.baseline();msg=PlanningScene(is_diff=True)
        msg.world.octomap.octomap.id='OcTree'
        self.assert_fault(node,msg)
        self.assertIsNone(node.scene_environment.parts['octomap'])

    def test_attached_remove_empty_id_is_scoped_to_link(self):
        node,msg=self.baseline('IDLE')
        extra=AttachedCollisionObject(link_name='link_5',object=self.box('other','link_5'))
        msg.robot_state.attached_collision_objects.append(extra);node.scene_update(msg)
        node.mode='CANCELING'
        remove=PlanningScene(is_diff=True);remove.robot_state.is_diff=True
        remove.robot_state.attached_collision_objects=[AttachedCollisionObject(link_name='link_6',object=CollisionObject(operation=CollisionObject.REMOVE))]
        self.assert_fault(node,remove)
        self.assertEqual(set(node.scene_environment.parts['attached']),{'other'})

    def test_same_world_move_is_stable_but_changed_pose_invalidates(self):
        node,_=self.baseline();generation=node.epoch.generation
        msg=PlanningScene(is_diff=True);obj=CollisionObject(id='table',operation=CollisionObject.MOVE)
        obj.header.frame_id='world';obj.pose.orientation.w=1.;msg.world.collision_objects=[obj]
        node.scene_update(msg)
        self.assertEqual(node.epoch.generation,generation);self.assertEqual(node.mode,'EXECUTING')
        obj.pose.position.x=.01;self.assert_fault(node,msg)
        self.assertEqual(len(node.scene_environment.parts['world']['table']['primitives']),1)

    def test_append_geometry_is_not_treated_as_repeated_add(self):
        node,_=self.baseline();msg=PlanningScene(is_diff=True);obj=self.box('table')
        obj.operation=CollisionObject.APPEND;msg.world.collision_objects=[obj]
        self.assert_fault(node,msg)
        self.assertEqual(len(node.scene_environment.parts['world']['table']['primitives']),2)

    def test_planning_and_pausing_invalidate_epoch_then_cancel(self):
        for mode in ('PLANNING','PAUSING'):
            with self.subTest(mode=mode):
                node,_=self.baseline(mode);generation=node.epoch.generation
                node.scene_update(PlanningScene())
                self.assertEqual(node.mode,'CANCELING');self.assertGreater(node.epoch.generation,generation)
                node.lease.publish.assert_not_called()

    def test_malformed_or_nonfinite_static_data_fails_closed(self):
        for kind in ('acm','nonfinite'):
            with self.subTest(kind=kind):
                node,msg=self.baseline()
                if kind=='acm':msg.allowed_collision_matrix.entry_values=[]
                else:msg.link_padding[0].padding=float('nan')
                self.assert_fault(node,msg)

    def test_scene_ready_false_revokes_lease_in_canceling(self):
        node,_=self.baseline('CANCELING')
        node.scene(String(data=json.dumps(dict(ready=False))))
        self.assertEqual(node.mode,'FAULT');self.assertFalse(node.lease.publish.call_args.args[0].data)


if __name__ == '__main__':
    unittest.main(verbosity=2)
