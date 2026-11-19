#!/usr/bin/env python3
"""Pure observer regressions; never start ROS or load the vendor SDK."""
import ast
import json
import math
from pathlib import Path
import sys
import unittest
from unittest import mock
import hardware_quest_commissioning as m


class ObserverTests(unittest.TestCase):
    def setUp(self):
        self.obs=m.Observation([-6.]*6,[6.]*6,.1745329252)

    def frame(self, now, q=0., v=0., a=0.):
        stamp=int(now*1e9)
        self.obs.joints(m.NAMES,[q]*6,[v]*6,stamp,stamp,now)
        self.obs.command(m.NAMES,[q]*6,[q]*6,[v]*6,[a]*6,stamp,stamp,now)
        self.obs.tcp([q,0.,0.],[0.,0.,0.,1.],stamp,stamp,now)

    def released(self, now):
        self.obs.inputs(dict(held=False,reason='RELEASED',software_stop=False,age_s=.001),int(now*1e9),now)

    def ready(self):
        self.obs.status(dict(mode='IDLE',scene_ready=True,auto_replan=False),10**9,1.)
        for i in range(12):
            now=1.+i*.008;self.frame(now);self.released(now);self.obs.tick(now)
        self.assertTrue(self.obs.ready)

    def test_default_does_not_import_ros_or_sdk(self):
        self.assertNotIn('rclpy',sys.modules)
        tree=ast.parse(Path(m.__file__).read_text())
        calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertFalse(calls & {'create_publisher','sendto','kill','killpg'})
        self.assertFalse(any(isinstance(n,ast.Name) and n.id=='ActionClient' for n in ast.walk(tree)))

    def test_no_ready_without_released_input(self):
        self.obs.status(dict(mode='IDLE',scene_ready=True,auto_replan=False),10**9,1.)
        for i in range(12):self.frame(1.+i*.008);self.obs.tick(1.+i*.008)
        self.assertFalse(self.obs.ready)

    def test_held_before_ready_rejected(self):
        with self.assertRaises(RuntimeError):
            self.obs.inputs(dict(held=True,reason='FOLLOWING',software_stop=False,age_s=.001),10**9,1.)

    def test_release_requires_new_quiet_window(self):
        self.ready()
        self.obs.inputs(dict(held=True,reason='FOLLOWING',software_stop=False,age_s=.001),1100000000,1.1)
        self.released(1.101);self.obs.tick(1.102)
        self.assertFalse(self.obs.stop_events[-1]['confirmed_stationary'])
        for i in range(12):self.frame(1.104+i*.008);self.obs.tick(1.104+i*.008)
        event=self.obs.stop_events[-1]
        self.assertTrue(event['confirmed_stationary'])
        self.assertGreaterEqual(event['observed_confirmation_delay_s'],.08)

    def test_nonzero_command_derivatives_prevent_ready(self):
        self.obs.status(dict(mode='IDLE',scene_ready=True,auto_replan=False),10**9,1.)
        for i in range(12):
            now=1.+i*.008;self.frame(now,a=.01);self.released(now);self.obs.tick(now)
        self.assertFalse(self.obs.ready)

    def test_stale_receipt_exits(self):
        self.frame(1.)
        with self.assertRaises(RuntimeError):self.obs.tick(1.051)

    def test_increasing_but_stale_joint_headers_rejected(self):
        self.obs.joints(m.NAMES,[0.]*6,[0.]*6,10**9,10**9,1.)
        with self.assertRaises(RuntimeError):
            self.obs.joints(m.NAMES,[0.]*6,[0.]*6,1008000000,1108000000,1.108)

    def test_increasing_but_stale_command_headers_rejected(self):
        self.obs.command(m.NAMES,[0.]*6,[0.]*6,[0.]*6,[0.]*6,10**9,10**9,1.)
        with self.assertRaises(RuntimeError):
            self.obs.command(m.NAMES,[0.]*6,[0.]*6,[0.]*6,[0.]*6,1008000000,1108000000,1.108)

    def test_joint_excursion_exits(self):
        self.frame(1.)
        with self.assertRaises(RuntimeError):
            self.obs.joints(m.NAMES,[math.radians(30.)+1e-6]*6,[0.]*6,1008000000,1008000000,1.008)

    def test_joint_rotation_inside_range_does_not_change_tcp_bound(self):
        self.frame(1.)
        self.obs.joints(m.NAMES,[.075]*6,[0.]*6,1008000000,1008000000,1.008)
        self.assertEqual(self.obs.tcp_excursion_limit,1.)
        with self.assertRaises(RuntimeError):
            self.obs.tcp([1.000001,0.,0.],[0.,0.,0.,1.],1008000000,1008000000,1.008)

    def test_tcp_excursion_exits(self):
        self.frame(1.)
        with self.assertRaises(RuntimeError):self.obs.tcp([1.000001,0.,0.],[0.,0.,0.,1.],1008000000,1008000000,1.008)

    def test_fake_observation_range_allows_release_after_normal_hand_motion(self):
        self.obs=m.Observation([-6.]*6,[6.]*6,.1745329252,hardware=False)
        self.frame(1.)
        self.frame(1.008,q=.04)
        self.obs.tcp([.20,0.,0.],[0.,0.,0.,1.],1016000000,1016000000,1.016)
        self.assertEqual(self.obs.joint_excursion_limit,math.radians(30.))
        self.assertEqual(self.obs.tcp_excursion_limit,1.)

    def test_fake_observation_still_has_finite_bounds(self):
        self.obs=m.Observation([-6.]*6,[6.]*6,.1745329252,hardware=False)
        self.frame(1.)
        with self.assertRaises(RuntimeError):
            self.obs.joints(m.NAMES,[math.radians(30.)+.000001]*6,[0.]*6,1008000000,1008000000,1.008)
        with self.assertRaises(RuntimeError):
            self.obs.tcp([1.000001,0.,0.],[0.,0.,0.,1.],1008000000,1008000000,1.008)

    def test_hardware_observer_uses_explicitly_authorized_finite_limits(self):
        for obs in (self.obs,m.Observation([-6.]*6,[6.]*6,.1745329252,hardware=True)):
            self.assertEqual(obs.joint_excursion_limit,math.radians(30.))
            self.assertEqual(obs.tcp_excursion_limit,1.)
        with self.assertRaises(RuntimeError):
            m.Observation([-6.]*6,[6.]*6,.1745329252,hardware='false')

    def test_stale_tcp_exits(self):
        with self.assertRaises(RuntimeError):self.obs.tcp([0.,0.,0.],[0.,0.,0.,1.],10**9,1100000000,1.1)

    def test_planning_and_fault_rejected(self):
        for mode in ('PLANNING','EXECUTING','FAULT'):
            with self.assertRaises(RuntimeError):self.obs.status(dict(mode=mode,auto_replan=False),1,1.)
        with self.assertRaises(RuntimeError):self.obs.status(dict(mode='IDLE',auto_replan=True),1,1.)

    def test_initial_input_infinity_serializes(self):
        self.obs.inputs(dict(held=False,reason='INPUT_STALE',software_stop=False,age_s=float('inf')),1,1.)
        text=json.dumps(m.json_safe(self.obs.frames),allow_nan=False)
        self.assertIn('"inf"',text);self.assertFalse(self.obs.first_released)

    def test_capture_bounded(self):
        self.obs.frames=[None]*m.MAX_FRAMES
        with self.assertRaises(RuntimeError):self.obs.capture('/test',1.,1,{})

    def test_capture_defaults_to_hardware_bound_and_fake_is_explicit(self):
        self.assertEqual(self.obs.frame_limit,50000)
        policy=m.validate_session_window(duration=90.)
        fake=m.Observation([-6.]*6,[6.]*6,.1745329252,frame_limit=policy['capture_frame_limit'])
        fake.frames=[None]*m.MAX_FRAMES
        fake.capture('/test',1.,1,{})
        self.assertEqual(len(fake.frames),50001)
        fake.frames=[None]*m.MAX_FAKE_FRAMES
        with self.assertRaises(RuntimeError):fake.capture('/test',1.,1,{})
        with self.assertRaises(RuntimeError):m.Observation([-6.]*6,[6.]*6,.2,frame_limit=75001)

    def test_log_phases_distinguish_runtime_fault_and_known_cleanup(self):
        runtime='[servo_node-6] [ERROR] failure'
        death='[ERROR] [move_group-5]: process has died [pid 1, exit code -11]'
        self.assertTrue(m.log_findings(runtime)['unexpected'])
        self.assertTrue(m.log_findings(death)['unexpected'])
        self.assertFalse(m.log_findings(death,True)['unexpected'])

    def test_fake_window_up_to_90_seconds_and_authorized_hardware_up_to_60(self):
        self.assertEqual(m.validate_session_window(duration=90.),dict(maximum_session_s=90.,capture_frame_limit=75000))
        self.assertEqual(m.validate_session_window(duration=30.01)['maximum_session_s'],90.)
        self.assertEqual(m.validate_session_window(hardware=True,duration=60.),dict(maximum_session_s=60.,capture_frame_limit=50000))
        with self.assertRaises(RuntimeError):m.validate_session_window(hardware=True,duration=60.01)
        with self.assertRaises(RuntimeError):m.validate_session_window(duration=90.01)

    def test_duration_cap_before_ros_or_filesystem_work(self):
        for hardware,duration in ((False,0.),(False,90.01),(False,float('nan')),
                                  (True,0.),(True,60.01),(True,90.),(True,float('inf'))):
            with mock.patch.object(m.Path,'mkdir') as mkdir, mock.patch('builtins.__import__') as imports:
                with self.assertRaises(RuntimeError):m.run(hardware=hardware,duration=duration)
                mkdir.assert_not_called();imports.assert_not_called()
        self.assertNotIn('rclpy',sys.modules)

    def test_default_window_stays_30_seconds(self):
        import inspect
        self.assertEqual(inspect.signature(m.run).parameters['duration'].default,30.)
        self.assertEqual(inspect.signature(m.validate_session_window).parameters['duration'].default,30.)

    def test_full_launch_source_compiles_without_execution(self):
        code=m.launch_source(Path('/example'),False,Path('/runtime'),Path('/audit'),False)
        compile(code,'generated_launch','exec')
        self.assertIn("'mode': 'fake'",code)
        self.assertIn("'sigkill_timeout': '86400'",code)
        self.assertIn('module.setup(context)',code)


    def held(self, now, follow=True):
        self.obs.inputs(dict(held=True,reason='FOLLOWING',software_stop=False,age_s=.001),int(now*1e9),now)
        if follow:self.obs.status(dict(mode='FOLLOW',auto_replan=False),int(now*1e9),now)

    def confirmed_motion_stop(self):
        self.ready();self.held(1.096);self.frame(1.104,q=.002,v=.001)
        self.released(1.112)
        for i in range(12):self.frame(1.120+i*.008,q=.002);self.obs.tick(1.120+i*.008)
        self.assertTrue(self.obs.released_stop_observed())
        return self.obs.stop_events[-1]

    def test_pre_ready_motion_does_not_count_as_grip_motion(self):
        self.frame(.9,q=.01);self.ready();self.held(1.096)
        self.frame(1.104);self.released(1.112)
        self.assertGreater(self.obs.max_joint_excursion,1e-4)
        self.assertFalse(self.obs.motion_observed())

    def test_measured_noise_without_command_motion_cannot_pass(self):
        self.ready();self.held(1.096)
        self.obs.joints(m.NAMES,[.0002]*6,[0.]*6,1104000000,1104000000,1.104)
        self.assertFalse(self.obs.motion_observed())

    def test_command_motion_without_measured_motion_cannot_pass(self):
        self.ready();self.held(1.096)
        self.obs.command(m.NAMES,[.002]*6,[.002]*6,[0.]*6,[0.]*6,1104000000,1104000000,1.104)
        self.assertFalse(self.obs.motion_observed())

    def test_episode_needs_actual_command_and_follow(self):
        self.ready();self.held(1.096,follow=False);self.frame(1.104,q=.002)
        self.assertFalse(self.obs.motion_observed())
        self.obs.status(dict(mode='FOLLOW',auto_replan=False),1112000000,1.112)
        self.assertTrue(self.obs.motion_observed())

    def test_motion_only_after_release_does_not_count_as_held_episode(self):
        self.ready();self.held(1.096);self.released(1.100);self.frame(1.104,q=.002)
        self.assertFalse(self.obs.motion_observed())

    def test_confirmed_stop_loses_proof_if_measured_motion_resumes(self):
        event=self.confirmed_motion_stop();self.frame(1.216,q=.003)
        self.assertFalse(event['confirmed_stationary'])
        self.assertTrue(event['invalidated_after_confirmation'])
        self.assertGreaterEqual(event['maximum_joint_motion_after_edge_rad'],.001)
        self.assertFalse(self.obs.released_stop_observed())
        with self.assertRaises(RuntimeError):self.obs.tick(1.216)

    def test_confirmed_stop_loses_proof_if_command_moves(self):
        event=self.confirmed_motion_stop()
        self.obs.command(m.NAMES,[.003]*6,[.003]*6,[0.]*6,[0.]*6,1216000000,1216000000,1.216)
        self.assertTrue(event['invalidated_after_confirmation'])
        self.assertFalse(self.obs.released_stop_observed())

    def test_confirmed_stop_loses_proof_if_feedback_expires(self):
        event=self.confirmed_motion_stop()
        with self.assertRaises(RuntimeError):self.obs.tick(1.260)
        self.assertTrue(event['invalidated_after_confirmation'])
        self.assertFalse(self.obs.released_stop_observed())

    def test_new_grip_ends_previous_stop_monitoring(self):
        event=self.confirmed_motion_stop();self.held(1.216);self.frame(1.224,q=.003)
        self.assertTrue(event['confirmed_stationary'])
        self.assertEqual(event['observation_end_reason'],'next_grip')
        self.assertTrue(self.obs.released_stop_observed())


if __name__=='__main__':unittest.main()
