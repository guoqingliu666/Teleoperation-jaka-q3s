"""真实入口的离线契约测试：不能连接、运动或改滤波；只读影子不伪造实测。"""
from pathlib import Path
import importlib.util
import io
import math
import sys
from contextlib import redirect_stdout
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from vla_lab.trajectory_reference import rpy_quaternion
from vla_lab.trajectory_shadow import TrajectoryShadow

spec=importlib.util.spec_from_file_location("trajectory_readonly_entry",
    Path(__file__).resolve().parents[1]/"只读检查新轨迹.py")
entry=importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)

MAPPING={"right":"X+","left":"X-","up":"Y+","down":"Y-","forward":"Z+","backward":"Z-"}
LIMITS=((-360.,360.),)*6


class Clock:
    def __init__(self): self.t=10.
    def __call__(self): return self.t


class Robot:
    def __init__(self): self.ik_calls=0; self.changed=False
    def get_actual_joint_position(self):
        return (0, (.1 if self.changed else 0.,0.,0.,0.,0.,-math.pi))
    def get_actual_tcp_position(self): return (0,(400.,100.,300.,0.,0.,-math.pi))
    def kine_inverse(self,ref,tcp):
        self.ik_calls+=1
        # 测试替身按种子保持同一连续圈；不代表实际JAKA一定返回这一路径。
        angle=tcp[5]+round((ref[5]-tcp[5])/(2*math.pi))*2*math.pi
        return (0,((tcp[0]-400)*.001,(tcp[1]-100)*.001,(tcp[2]-300)*.001,0.,0.,angle))


def frame(clock,grip,*,x=0.,deg=0.,source="test"):
    return SimpleNamespace(connected=True,tracked=True,valid=True,rotation_valid=True,
        head_rotation_valid=True,head_rotation_xyzw=(0.,0.,0.,1.),received_s=clock(),
        position_m=(x,0.,0.),rotation_xyzw=rpy_quaternion((0.,0.,math.radians(deg))),
        grip=grip,trigger=0.,udp_source=source,button_b=False,button_a=False,
        button_x=False,button_y=False,thumbstick_click=False)


class ShadowTests(unittest.TestCase):
    def test_dynamic_mode_uses_raw_pose_without_unused_quintic_fitting(self):
        shadow, robot, clock, events = self.new()
        with patch.object(shadow.reference, "push", side_effect=AssertionError("多余的曲线拟合")):
            shadow.process(frame(clock, 0.))
            clock.t += .02
            shadow.process(frame(clock, 1.))
            clock.t += .02
            shadow.process(frame(clock, 1., x=.001, deg=.1))
            self.assertEqual(len(shadow.horizon.samples), 1)
            sample, received = shadow.horizon.samples[0]
            self.assertEqual(sample.t, clock())
            self.assertEqual(received, clock())
            # 新调度允许小幅静止尾段合并等待至160ms；最终仍必须交付。
            for _ in range(100):
                clock.t += .002
                shadow.process(frame(clock, 1., x=.001, deg=.1))
        self.assertEqual(shadow.blocks, 0)
        self.assertGreater(shadow.candidates, 0)
        self.assertFalse(any(e["state"] == "trajectory_reference_span" for e in events))
        self.assertTrue(all(e.get("movement_commands_sent", 0) == 0 for e in events))

    def test_pending_ik_yields_without_extra_delay(self):
        shadow,_,_,_=self.new()
        self.assertEqual(entry.readonly_loop_delay(shadow),.002)
        shadow.horizon.job=object()
        self.assertEqual(entry.readonly_loop_delay(shadow),0.)
        shadow.horizon.job=None
        shadow.release("test")
        self.assertEqual(entry.readonly_loop_delay(shadow),.002)

    def test_sdk_timing_preserves_result_and_exception(self):
        clock=Clock()
        class TimedRobot:
            def get_actual_joint_position(self):
                clock.t+=.003
                return (0,(1,2,3,4,5,6))
            def kine_inverse(self,*args):
                clock.t+=.005
                raise RuntimeError("SDK error")
        sdk=entry.ReadOnlySDK(TimedRobot(),clock)
        self.assertEqual(sdk.get_actual_joint_position(),(0,(1,2,3,4,5,6)))
        with self.assertRaisesRegex(RuntimeError,"SDK error"):
            sdk.kine_inverse(None,None)
        report=sdk.timing_report()
        self.assertEqual(report["get_actual_joint_position"]["count"],1)
        self.assertAlmostEqual(report["get_actual_joint_position"]["total_ms"],3.)
        self.assertAlmostEqual(report["kine_inverse"]["max_ms"],5.)
        report["kine_inverse"]["count"]=999
        self.assertEqual(sdk.timing_report()["kine_inverse"]["count"],1)

    def test_ui_stop_and_timeout_cannot_authorize_motion(self):
        clock=Clock()
        c=entry.ReadonlyControl(clock)
        self.assertIsNone(c.reason())
        clock.t+=2; c.accept("HEARTBEAT")
        clock.t+=2; self.assertIsNone(c.reason())
        clock.t+=2; self.assertIn("心跳",c.reason())
        c.accept("STOP")
        c.accept("HEARTBEAT")
        self.assertTrue(c.stopped)
        c=entry.ReadonlyControl(clock)
        c.accept("ENABLE_MOTION")
        self.assertTrue(c.stopped)

    def new(self):
        robot,clock,events=Robot(),Clock(),[]
        shadow=TrajectoryShadow(robot,MAPPING,LIMITS,emit=lambda **e:events.append(e),clock=clock)
        return shadow,robot,clock,events

    def test_default_script_never_loads_sdk(self):
        with patch.object(entry,"load_sdk",side_effect=AssertionError("不能连接")):
            with redirect_stdout(io.StringIO()): self.assertEqual(entry.main([]),0)

    def test_input_status_distinguishes_missing_tracking_and_release(self):
        shadow,robot,clock,events=self.new()
        self.assertIn("UDP",entry.input_status(None,clock(),shadow))
        self.assertIn("已收到UDP",entry.input_status(None,clock(),shadow,untracked_before_lock=108))
        self.assertIn("未有效TRACKED",entry.input_status(None,clock(),shadow,untracked_before_lock=108))
        f=frame(clock,1.); f.tracked=False
        self.assertIn("TRACKED",entry.input_status(f,clock(),shadow))
        f=frame(clock,1.)
        self.assertIn("松Grip",entry.input_status(f,clock(),shadow))
        shadow.process(frame(clock,0.))
        self.assertIn("已见松Grip",entry.input_status(f,clock(),shadow))
        clock.t+=.2
        self.assertIn("过期",entry.input_status(f,clock(),shadow))

    def test_readonly_whitelist_refuses_all_writes(self):
        sdk=entry.ReadOnlySDK(Robot())
        for method in ("linear_move_extend_ori","servo_j","servo_p","motion_abort",
                       "servo_move_use_joint_LPF","set_motion_planner","power_on","enable_robot","clear_error"):
            with self.assertRaises(PermissionError): getattr(sdk,method)

    def test_requires_release_before_first_grip(self):
        shadow,robot,clock,events=self.new()
        shadow.process(frame(clock,1.))
        self.assertIsNone(shadow.anchor)
        self.assertEqual(robot.ik_calls,0)

    def test_three_grips_and_raw_records(self):
        shadow,robot,clock,events=self.new()
        for grip in range(3):
            shadow.process(frame(clock,0.))
            for i in range(20):
                clock.t+=.02
                f=frame(clock,1.,x=i*.0002,deg=i*.1)
                for _ in range(8):
                    shadow.process(f)
                    clock.t+=.001
            shadow.process(frame(clock,0.))
        self.assertEqual(shadow.strokes,3)
        self.assertGreater(robot.ik_calls,0)
        self.assertGreater(shadow.candidates,0)
        self.assertTrue(any(e["state"]=="trajectory_raw_sample" for e in events))
        self.assertTrue(all(e.get("movement_commands_sent",0)==0 for e in events))

    def test_source_change_requires_regrip(self):
        shadow,robot,clock,events=self.new()
        shadow.process(frame(clock,0.)); clock.t+=.02
        shadow.process(frame(clock,1.)); clock.t+=.02
        shadow.process(frame(clock,1.,source="other"))
        self.assertIsNone(shadow.anchor)
        clock.t+=.02; shadow.process(frame(clock,1.,source="other"))
        self.assertIsNone(shadow.anchor)

    def test_full_turn_is_retained_in_real_input_pipeline_without_moving_robot(self):
        shadow,robot,clock,events=self.new()
        shadow.process(frame(clock,0.))
        for i in range(361):
            clock.t+=.01
            f=frame(clock,1.,deg=i)
            for _ in range(10):
                shadow.process(f)
                clock.t+=.001
        self.assertEqual(shadow.blocks,0)
        self.assertAlmostEqual(shadow.max_twist_deg,360.,places=6)
        plans=[e for e in events if e["state"]=="trajectory_vendor_preview"]
        self.assertGreater(len(plans),10)
        self.assertGreater(math.degrees(plans[-1]["solutions"][-1][5]),160.)
        self.assertEqual(robot.get_actual_joint_position()[1][-1],-math.pi)

    def test_physical_motion_invalidates_static_shadow(self):
        shadow,robot,clock,events=self.new()
        shadow.process(frame(clock,0.)); clock.t+=.02
        shadow.process(frame(clock,1.))
        robot.changed=True
        clock.t+=.11
        with self.assertRaisesRegex(RuntimeError,"实机发生运动"):
            shadow.process(frame(clock,1.))

    def test_tracking_loss_clears_future_and_cannot_resume_held(self):
        shadow,robot,clock,events=self.new()
        shadow.process(frame(clock,0.)); clock.t+=.02
        shadow.process(frame(clock,1.))
        shadow.process(None)
        self.assertIsNone(shadow.horizon.ready)
        self.assertIsNone(shadow.anchor)
        clock.t+=.02; shadow.process(frame(clock,1.))
        self.assertIsNone(shadow.anchor)


if __name__=="__main__": unittest.main()
