"""运行入口集成测试：替换SDK、UDP和界面管道；没有真实网络或运动。

仅日志写入项目Validation，验证实际入口的shadow/live分支与finally注销路径。
"""
import importlib.util
import io
import math
from contextlib import redirect_stdout
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from verify_sampled_follow import LIMITS, Robot, frame

spec = importlib.util.spec_from_file_location("sampled_runner", Path(__file__).resolve().parents[1] / "连续采样位置遥操作.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class FakeRobot(Robot):
    def __init__(self):
        super().__init__()
        self.logouts = 0
    def login(self): return (0,)
    def logout(self): self.logouts += 1; return (0,)


class AutoCompleteRobot(FakeRobot):
    def is_in_pos(self):
        if self.moves and not self.in_pos:
            self.complete()
        return super().is_in_pos()


class OrientationAutoCompleteRobot(FakeRobot):
    def __init__(self):
        super().__init__()
        self.orientation_moves=[]
    def linear_move_extend_ori(self,target,*args):
        self.orientation_moves.append((target,args)); self.in_pos=False
        return (0,)
    def is_in_pos(self):
        if self.orientation_moves and not self.in_pos:
            self.tcp=self.orientation_moves[-1][0]; self.in_pos=True
        return super().is_in_pos()


class Receiver:
    error = None
    def __init__(self, *_): self.calls=0
    def latest(self):
        self.calls += 1
        return frame(time.monotonic, grip=0 if self.calls <= 3 else 1,
                     y=1 + max(0,min(self.calls-4,30))*.001)
    def close(self): pass


class FiveSegmentReceiver(Receiver):
    def latest(self):
        self.calls += 1
        return frame(time.monotonic, grip=0 if self.calls <= 3 else 1,
                     y=1 + max(0,min(self.calls-4,65))*.001)


class TenSegmentReceiver(Receiver):
    def latest(self):
        self.calls += 1
        return frame(time.monotonic, grip=0 if self.calls <= 3 else 1,
                     y=1 + max(0,min(self.calls-4,120))*.001)


class TwentySegmentReceiver(Receiver):
    def latest(self):
        self.calls += 1
        return frame(time.monotonic, grip=0 if self.calls <= 3 else 1,
                     y=1 + max(0,min(self.calls-4,240))*.001)


class OrientationReceiver(Receiver):
    def latest(self):
        self.calls += 1
        angle = 0.0 if self.calls <= 6 else math.radians(.6)/2
        return frame(time.monotonic, grip=0 if self.calls <= 3 else 1,
                     rotation_xyzw=(0.0,math.sin(angle),0.0,math.cos(angle)))


class ThreeOrientationReceiver(Receiver):
    def latest(self):
        self.calls += 1
        if self.calls <= 6:
            degrees = 0.0
        elif self.calls <= 12:
            degrees = .6
        elif self.calls <= 18:
            degrees = 1.6
        else:
            degrees = 2.6
        half = math.radians(degrees)/2
        return frame(time.monotonic, grip=0 if self.calls <= 3 else 1,
                     rotation_xyzw=(0.0,math.sin(half),0.0,math.cos(half)))


class TenOrientationReceiver(Receiver):
    def latest(self):
        self.calls += 1
        degrees = 0.0 if self.calls <= 6 else 9.6
        half = math.radians(degrees)/2
        return frame(time.monotonic, grip=0 if self.calls <= 3 else 1,
                     rotation_xyzw=(0.0,math.sin(half),0.0,math.cos(half)))


class SixDofReceiver(Receiver):
    def latest(self):
        self.calls += 1
        degrees = 0.0 if self.calls <= 6 else 9.6
        half = math.radians(degrees)/2
        return frame(time.monotonic,grip=0 if self.calls<=3 else 1,
                     y=1.0 if self.calls<=6 else 1.03,
                     rotation_xyzw=(0.0,math.sin(half),0.0,math.cos(half)))


class ExpandedSixDofReceiver(Receiver):
    def latest(self):
        self.calls+=1
        degrees=0.0 if self.calls<=6 else 25.0
        half=math.radians(degrees)/2
        return frame(time.monotonic,grip=0 if self.calls<=3 else 1,
                     y=1.0 if self.calls<=6 else 1.08,
                     rotation_xyzw=(0.0,math.sin(half),0.0,math.cos(half)))


class Permit:
    # 假心跳有效期要长于测试的1秒会话，否则慢速CI会把性能波动误判为产品失效。
    def __init__(self): self.until=time.monotonic()+5.0
    def valid(self): return time.monotonic()<self.until


class Socket:
    def sendto(self,*_): pass
    def close(self): pass


class RunnerTests(unittest.TestCase):
    def run_fake(self, mode, *, display=False, one_segment=False, two_segment=False,
                 five_segment=False, ten_segment=False, twenty_segment=False, bounded_continuous=False,
                 orientation_acceptance=False, three_orientation_acceptance=False,
                 ten_orientation_acceptance=False,
                 bounded_six_dof=False,
                 expanded_six_dof=False,
                 production_six_dof=False,
                 robot_class=FakeRobot,
                 receiver_class=Receiver, expected_code=0):
        robot=robot_class()
        output=io.StringIO()
        with patch.object(runner,"load_sdk",return_value=SimpleNamespace(RC=lambda _:robot)), \
             patch.object(runner,"SESSION_LOG_ROOT",runner.ROOT / "Validation" / "offline_fake_sdk_follow"), \
             patch.object(runner,"read_limits",return_value=(LIMITS,"fake-limits-sha256")), \
             patch.object(runner,"QuestUdpReceiver",receiver_class), \
             patch.object(runner,"UiPermit",Permit), \
             patch.object(runner,"request_readonly_handoff",return_value=False), \
             patch.object(runner,"reject_legacy_bridge"), \
             patch.object(runner,"SdkOwnerLease",return_value=SimpleNamespace(close=lambda:None)), \
             patch.object(runner.socket,"socket",return_value=Socket()), redirect_stdout(output):
            flags=[mode,"--ui-heartbeat","--ui-protocol","2","--session-seconds",
                   "3" if twenty_segment else "1",
                   "--host","192.168.1.10","--limits-file","unused.ini"]
            if display: flags.append("--single-owner-display")
            if one_segment: flags.append("--one-segment-acceptance")
            if two_segment: flags.append("--two-segment-acceptance")
            if five_segment: flags.append("--five-segment-acceptance")
            if ten_segment: flags.append("--ten-segment-acceptance")
            if twenty_segment: flags.append("--twenty-segment-acceptance")
            if bounded_continuous: flags.append("--bounded-continuous")
            if orientation_acceptance: flags.append("--one-degree-orientation-acceptance")
            if three_orientation_acceptance: flags.append("--three-degree-orientation-acceptance")
            if ten_orientation_acceptance: flags.append("--ten-degree-orientation-acceptance")
            if bounded_six_dof: flags.append("--bounded-six-dof")
            if expanded_six_dof: flags.append("--expanded-six-dof")
            if production_six_dof:
                flags.extend(["--production-six-dof", "--radius-mm", "1000",
                              "--speed-mm-s", "45", "--acceleration-mm-s2", "90",
                              "--rotation-radius-deg", "25",
                              "--orientation-speed-deg-s", "8"])
            code=runner.main(flags)
        self.assertEqual(code,expected_code,output.getvalue())
        self.assertEqual(robot.logouts,1)
        self.assertIn('"stop_confirmed": true',output.getvalue())
        return robot,output.getvalue()

    def test_shadow_never_moves(self):
        robot,output=self.run_fake("--shadow")
        self.assertIn('"state": "shadow"',output)
        self.assertEqual(robot.moves,[])
        self.assertEqual(robot.abort_count,0)

    def test_single_owner_display_uses_same_fake_sdk_without_motion(self):
        robot,output=self.run_fake("--shadow",display=True)
        self.assertIn('"state": "display"',output)
        self.assertEqual(robot.moves,[])

    def test_rotation_shadow_uses_vendor_ik_but_never_moves(self):
        robot,output=self.run_fake("--rotation-shadow",display=True)
        self.assertEqual(robot.moves,[])
        self.assertEqual(robot.abort_count,0)
        self.assertIn('"state": "rotation_shadow_complete"',output)
        self.assertIn('"movement_commands_sent": 0',output)

    def test_one_degree_orientation_acceptance_sends_exactly_one_vendor_orientation_move(self):
        robot,output=self.run_fake(
            "--live",display=True,orientation_acceptance=True,
            robot_class=OrientationAutoCompleteRobot,receiver_class=OrientationReceiver)
        self.assertEqual(len(robot.orientation_moves),1)
        self.assertEqual(robot.moves,[])
        target,args=robot.orientation_moves[0]
        self.assertEqual(args[:5],(0,False,5.0,10.0,0.0))
        self.assertAlmostEqual(math.degrees(args[5]),1.0)
        self.assertAlmostEqual(math.degrees(args[6]),2.0)
        self.assertIn('"state": "orientation_acceptance_complete"',output)

    def test_three_degree_orientation_acceptance_sends_three_bounded_vendor_moves(self):
        robot,output=self.run_fake(
            "--live",display=True,three_orientation_acceptance=True,
            robot_class=OrientationAutoCompleteRobot,
            receiver_class=ThreeOrientationReceiver)
        self.assertEqual(len(robot.orientation_moves),3)
        self.assertEqual(robot.moves,[])
        for target,args in robot.orientation_moves:
            self.assertEqual(target[:3],(400.0,100.0,300.0))
            self.assertAlmostEqual(math.degrees(args[5]),2.0)
            self.assertAlmostEqual(math.degrees(args[6]),4.0)
        self.assertIn('"state": "orientation_acceptance_complete"',output)
        self.assertIn('"movement_commands_sent": 3',output)

    def test_ten_degree_orientation_acceptance_sends_ten_one_degree_vendor_moves(self):
        robot,output=self.run_fake(
            "--live",display=True,ten_orientation_acceptance=True,
            robot_class=OrientationAutoCompleteRobot,
            receiver_class=TenOrientationReceiver)
        self.assertEqual(len(robot.orientation_moves),10)
        self.assertEqual(robot.moves,[])
        for target,args in robot.orientation_moves:
            self.assertEqual(target[:3],(400.0,100.0,300.0))
            self.assertAlmostEqual(math.degrees(args[5]),5.0)
            self.assertAlmostEqual(math.degrees(args[6]),10.0)
        self.assertIn('"movement_commands_sent": 10',output)

    def test_bounded_six_dof_combines_position_and_orientation_without_servo(self):
        robot,output=self.run_fake(
            "--live",display=True,bounded_six_dof=True,
            robot_class=OrientationAutoCompleteRobot,receiver_class=SixDofReceiver)
        self.assertGreaterEqual(len(robot.orientation_moves),9)
        self.assertEqual(robot.moves,[])
        for target,args in robot.orientation_moves:
            self.assertLessEqual(math.dist(target[:3],(400,100,300)),200.000001)
            self.assertEqual(args[:5],(0,False,30.,60.,0.0))
            self.assertAlmostEqual(math.degrees(args[5]),5.0)
            self.assertAlmostEqual(math.degrees(args[6]),10.0)
        self.assertIn('"bounded_six_dof": true',output)

    def test_expanded_six_dof_uses_larger_but_still_bounded_segments(self):
        robot,output=self.run_fake(
            "--live",display=True,expanded_six_dof=True,
            robot_class=OrientationAutoCompleteRobot,
            receiver_class=ExpandedSixDofReceiver)
        self.assertGreaterEqual(len(robot.orientation_moves),12)
        self.assertEqual(robot.moves,[])
        for target,args in robot.orientation_moves:
            self.assertLessEqual(math.dist(target[:3],(400,100,300)),500.000001)
            self.assertEqual(args[:5],(0,False,50.,100.,0.0))
            self.assertAlmostEqual(math.degrees(args[5]),10.0)
            self.assertAlmostEqual(math.degrees(args[6]),20.0)
        self.assertIn('"expanded_six_dof": true',output)

    def test_production_six_dof_uses_adjustable_limits_but_same_short_segments(self):
        robot,output=self.run_fake(
            "--live",display=True,production_six_dof=True,
            robot_class=OrientationAutoCompleteRobot,
            receiver_class=ExpandedSixDofReceiver)
        self.assertGreaterEqual(len(robot.orientation_moves),12)
        self.assertEqual(robot.moves,[])
        for target,args in robot.orientation_moves:
            self.assertLessEqual(math.dist(target[:3],(400,100,300)),1000.000001)
            self.assertEqual(args[:5],(0,False,45.,90.,0.0))
            self.assertAlmostEqual(math.degrees(args[5]),8.0)
            self.assertAlmostEqual(math.degrees(args[6]),32.0)
        self.assertIn('"production_six_dof": true',output)

    def test_continuous_live_is_locked_before_any_sdk_connection(self):
        with self.assertRaises(SystemExit) as caught:
            runner.main(["--live","--ui-heartbeat","--ui-protocol","2"])
        self.assertEqual(caught.exception.code,2)

    def test_orientation_acceptance_cannot_run_without_live_gui_gate(self):
        with self.assertRaises(SystemExit) as caught:
            runner.main(["--one-degree-orientation-acceptance"])
        self.assertEqual(caught.exception.code,2)

    def test_bounded_continuous_live_uses_latest_targets_with_fixed_short_segments(self):
        robot,output=self.run_fake("--live",display=True,bounded_continuous=True,
                                   robot_class=AutoCompleteRobot)
        self.assertGreaterEqual(len(robot.moves),2)
        self.assertIn('"bounded_continuous": true',output)
        self.assertNotIn('"state": "acceptance_start"',output)
        for target, options in robot.moves:
            self.assertEqual(options,(0,False,10.,50.,0.))
            self.assertLessEqual(math.dist(target[:3],(400.,100.,300.)),100.000001)

    def test_bounded_continuous_hard_caps_reject_cli_bypass_before_connection(self):
        base = ["--live", "--ui-heartbeat", "--ui-protocol", "2",
                "--single-owner-display", "--bounded-continuous"]
        for extra in (["--radius-mm", "201"], ["--speed-mm-s", "31"],
                      ["--acceleration-mm-s2", "61"]):
            with self.assertRaises(SystemExit) as caught:
                runner.main(base + extra)
            self.assertEqual(caught.exception.code, 2)

    def test_one_segment_acceptance_auto_ends_after_exactly_one(self):
        robot,output=self.run_fake("--live",display=True,one_segment=True,
                                   robot_class=AutoCompleteRobot)
        self.assertEqual(len(robot.moves),1)
        self.assertEqual(robot.abort_count,0)
        self.assertIn('"state": "acceptance_complete"',output)
        self.assertIn('"measured_displacement_mm"',output)
        self.assertIn('"measured_tcp"',output)
        target, options = robot.moves[0]
        self.assertLessEqual(math.dist(target[:3],(400.,100.,300.)),5.000001)
        self.assertEqual(target[3:],(0.,0.,0.))
        self.assertEqual(options,(0,False,5.,10.,0.))

    def test_one_segment_release_is_incomplete_and_never_retries(self):
        robot,output=self.run_fake("--live",display=True,one_segment=True,
                                   expected_code=2)
        self.assertEqual(len(robot.moves),1)
        self.assertEqual(robot.abort_count,1)
        self.assertIn('"state": "acceptance_incomplete"',output)

    def test_two_segment_acceptance_has_hard_command_limit_and_measured_endpoint(self):
        robot,output=self.run_fake("--live",display=True,two_segment=True,
                                   robot_class=AutoCompleteRobot)
        self.assertEqual(len(robot.moves),2)
        self.assertEqual(robot.abort_count,0)
        self.assertIn('"state": "acceptance_complete"',output)
        for target, options in robot.moves:
            self.assertEqual(options,(0,False,10.,20.,0.))
        self.assertLessEqual(math.dist(robot.tcp[:3],(400.,100.,300.)),30.000001)

    def test_five_segment_acceptance_stops_at_five_and_records_measured_path(self):
        robot,output=self.run_fake("--live",display=True,five_segment=True,
                                   robot_class=AutoCompleteRobot,receiver_class=FiveSegmentReceiver)
        self.assertEqual(len(robot.moves),5)
        self.assertEqual(robot.abort_count,0)
        self.assertIn('"state": "acceptance_complete"',output)
        self.assertIn('"measured_path_mm"',output)
        for target, options in robot.moves:
            self.assertEqual(options,(0,False,15.,30.,0.))
        self.assertLessEqual(math.dist(robot.tcp[:3],(400.,100.,300.)),60.000001)

    def test_ten_segment_acceptance_stops_at_ten_and_uses_fixed_profile(self):
        robot,output=self.run_fake("--live",display=True,ten_segment=True,
                                   robot_class=AutoCompleteRobot,receiver_class=TenSegmentReceiver)
        self.assertEqual(len(robot.moves),10)
        self.assertEqual(robot.abort_count,0)
        self.assertIn('"state": "acceptance_complete"',output)
        self.assertIn('"measured_path_mm"',output)
        for target, options in robot.moves:
            self.assertEqual(options,(0,False,20.,40.,0.))
        self.assertLessEqual(math.dist(robot.tcp[:3],(400.,100.,300.)),100.000001)

    def test_twenty_segment_acceptance_stops_at_twenty_and_uses_fixed_profile(self):
        robot,output=self.run_fake("--live",display=True,twenty_segment=True,
                                   robot_class=AutoCompleteRobot,receiver_class=TwentySegmentReceiver)
        self.assertEqual(len(robot.moves),20)
        self.assertEqual(robot.abort_count,0)
        self.assertIn('"state": "acceptance_complete"',output)
        self.assertIn('"path_limit_mm": 200',output)
        for target, options in robot.moves:
            self.assertEqual(options,(0,False,30.,60.,0.))
        self.assertLessEqual(math.dist(robot.tcp[:3],(400.,100.,300.)),200.000001)

    def test_communication_check_never_moves_or_uses_quest(self):
        with patch.object(Receiver,"__init__",side_effect=AssertionError("通信检查不需要Quest")):
            robot,output=self.run_fake("--comm-check")
        self.assertEqual(robot.moves,[])
        self.assertEqual(robot.abort_count,0)
        self.assertIn('"state": "comm_result"',output)

    def test_communication_check_with_unity_feedback_still_never_moves(self):
        robot,output=self.run_fake("--comm-check",display=True)
        self.assertIn('"state": "comm_result"',output)
        self.assertEqual(robot.moves,[])


if __name__=="__main__": unittest.main()
