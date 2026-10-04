import ctypes as C
import unittest
from v2_servo.teleop.native_sdk import NativeRC,Vector6,MotionStatus,SimpleStatus,abi_layout
from v2_servo.teleop.edg import decode_status_pair
from v2_servo.teleop.edg import StaggeredSafetyPoll
from v2_servo.safety import SafetyFault
from unittest.mock import Mock


class NativeSDKTests(unittest.TestCase):
    def test_edg_sender_keeps_absolute_target_step_and_robot_index(self):
        rc=NativeRC.__new__(NativeRC);rc.logged=True;rc.handle=C.c_int(7);rc.edg_servo=True
        def send(handle,joint,mode,step,index):
            self.assertEqual(list(C.cast(joint,C.POINTER(Vector6)).contents.values),[.1]*6)
            self.assertEqual((mode,step,index),(0,1,0));return 0
        rc.functions={'edg_servo_j_extend':send}
        self.assertEqual(rc.servo_j([.1]*6,0,1),(0,))
    def test_staggered_raw_fault_is_rejected_without_waiting_for_other_query(self):
        rc=Mock();rc.get_motion_status.return_value=(0,[0]*9+[1,0])
        poll=StaggeredSafetyPoll(rc,lambda **row:None)
        with self.assertRaisesRegex(SafetyFault,'raw motion safety fault'):poll.query_one()
        rc.get_robot_status_simple.assert_not_called()
    def test_staggered_status_keeps_oldest_actual_query_time(self):
        rc=Mock();rc.get_motion_status.return_value=(0,[0]*11)
        rc.get_robot_status_simple.return_value=(0,[0,'',1,1])
        poll=StaggeredSafetyPoll(rc,lambda **row:None)
        flags,stamp,_=poll.query_one();self.assertIsNone(flags);self.assertIsNone(stamp)
        flags,stamp,_=poll.query_one();self.assertTrue(flags['enabled'])
        self.assertEqual(stamp,poll.motion_at)
        self.assertLessEqual(stamp,poll.simple_at)
    def test_official_queue_api_preserves_target_and_has_no_do_action(self):
        rc=NativeRC.__new__(NativeRC);rc.logged=True;rc.handle=C.c_int(7);rc.queue_api=True
        def send(handle,joint,mode,step,depth,do_info,robot_id):
            self.assertEqual(list(C.cast(joint,C.POINTER(Vector6)).contents.values),[.1]*6)
            self.assertEqual((mode,step,do_info,robot_id),(0,1,None,0))
            C.cast(depth,C.POINTER(C.c_int)).contents.value=1
            return 0
        rc.functions={'servo_j_extend_queue':send}
        self.assertEqual(rc.servo_j([.1]*6,0,1),(0,1))
    def test_absolute_joint_target_passes_in_radians_without_scaling(self):
        rc=NativeRC.__new__(NativeRC);rc.logged=True;rc.handle=C.c_int(7)
        captured=[]
        def send(handle,joint,mode,step):
            captured.append((C.cast(handle,C.POINTER(C.c_int)).contents.value,
                             list(C.cast(joint,C.POINTER(Vector6)).contents.values),mode,step))
            return 0
        rc.functions={'servo_j_extend':send}
        q=[.1,-.2,.3,-.4,.5,-.6]
        self.assertEqual(rc.servo_j(q,0,1),(0,))
        self.assertEqual(captured,[(7,q,0,1)])
        for mode,step in ((1,1),(0,2)):
            with self.assertRaises(ValueError):rc.servo_j(q,mode,step)
        self.assertEqual(len(captured),1)

    def test_native_fault_feedback_is_raw_and_blocks_invalid_fields(self):
        rc=NativeRC.__new__(NativeRC);rc.logged=True;rc.handle=C.c_int(7)
        def motion(handle,p):
            value=C.cast(p,C.POINTER(MotionStatus)).contents
            value.values[9]=1;return 0
        def simple(handle,p):
            value=C.cast(p,C.POINTER(SimpleStatus)).contents
            value.errcode=42;value.powered=1;value.enabled=1;return 0
        rc.functions={'get_motion_status':motion,'get_robot_status_simple':simple}
        raw=rc.read_safety();flags=decode_status_pair(raw['motion'][1],raw['simple'][1])
        self.assertTrue(flags['estop']);self.assertEqual(flags['error'],42)
        rc.logged=False
        with self.assertRaises(RuntimeError):rc.read_safety()

    def test_header_layout_uses_int_bool_and_nested_double_fields(self):
        layout=abi_layout()
        self.assertEqual(layout['SimpleStatus']['offsets']['powered'],204)
        self.assertEqual(layout['EdgState']['offsets']['io'],240)
        self.assertEqual(layout['EdgState']['size'],432)
        self.assertEqual(layout['Payload']['size'],40)
