"""假SDK故障注入，不连接硬件。验证迟到结果不能让后续运动继续。"""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1] / "src"))
from vla_lab.sdk_trace import TracedSdk


class Trace:
    def __init__(self): self.events=[]
    def write(self,event): self.events.append(event)


class Robot:
    def __init__(self): self.moves=0; self.stops=0
    def get_actual_joint_position(self): return (0,[0]*6)
    def linear_move_extend(self): self.moves+=1; return (0,)
    def motion_abort(self): self.stops+=1; return (0,)
    def logout(self): return (0,)
    def kine_inverse(self): return (-4,)


class Tests(unittest.TestCase):
    def test_start_written_before_native_call(self):
        raw,log=Robot(),Trace()
        raw.get_actual_joint_position=lambda:(0,len(log.events))
        sdk=TracedSdk(raw,log)
        self.assertEqual(sdk.get_actual_joint_position(),(0,1))
        self.assertEqual([x['state'] for x in log.events],['sdk_begin','sdk_end'])

    def test_late_result_latches_future_movement_but_allows_stop(self):
        raw,log=Robot(),Trace()
        times=iter([0,25,25,25.001])
        sdk=TracedSdk(raw,log,clock=lambda:next(times))
        with self.assertRaises(TimeoutError): sdk.get_actual_joint_position()
        with self.assertRaises(RuntimeError): sdk.linear_move_extend()
        self.assertEqual(raw.moves,0)
        self.assertEqual(sdk.motion_abort(),(0,))
        self.assertEqual(raw.stops,1)

    def test_communication_failure_latches(self):
        raw,log=Robot(),Trace()
        raw.get_actual_joint_position=lambda:(-3,)
        sdk=TracedSdk(raw,log)
        self.assertEqual(sdk.get_actual_joint_position(),(-3,))
        with self.assertRaises(RuntimeError): sdk.linear_move_extend()
        sdk.logout()

    def test_ik_unreachable_is_not_communication_failure(self):
        sdk=TracedSdk(Robot(),Trace())
        self.assertEqual(sdk.kine_inverse(),(-4,))
        self.assertIsNone(sdk.fault)


if __name__=='__main__': unittest.main()
