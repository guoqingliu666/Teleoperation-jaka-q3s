"""DH116/Grip 集成离线测试；只启动 DemoHand，绝不连接任何网卡或机器人。"""
import json
import math
from pathlib import Path
import socket
import sys
import time
import tkinter as tk
from types import SimpleNamespace
import unittest
from dataclasses import replace
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from vla_lab.dh116_control import TriggerPolicy, validate_limits, HandController
from vla_lab.engineering_teleop_live import grip_deadman_pressed, LevelALiveWindow
from vla_lab.vr_robot_visualization import RobotVrBroadcaster
from vla_lab.jaka_jog_controller import RobotSnapshot
from vla_lab.quest_vr_input import QuestUdpReceiver

ROOT=Path(__file__).resolve().parents[2]


def complete_end_effector_folder():
    """优先检查 Unity 源工程；发布包中则检查随 Player 交付的同一套模型。"""
    candidates = (
        ROOT/'QuestPoseBridge/Assets/StreamingAssets/CompleteEndEffector',
        ROOT/'Player_通信修复/QuestPosePreview_Data/StreamingAssets/CompleteEndEffector',
    )
    for folder in candidates:
        if (folder/'assembly.json').is_file():
            return folder
    raise FileNotFoundError('未找到 CompleteEndEffector 发布资源')


class HandTests(unittest.TestCase):
    def test_grip_does_not_depend_on_trigger(self):
        for trigger in (0.,0.5,1.):
            self.assertTrue(grip_deadman_pressed(SimpleNamespace(grip=1.,trigger=trigger),already_active=False))
            self.assertFalse(grip_deadman_pressed(SimpleNamespace(grip=0.,trigger=trigger),already_active=True))
        self.assertFalse(grip_deadman_pressed(SimpleNamespace(grip=math.nan),already_active=True))
        self.assertTrue(grip_deadman_pressed(SimpleNamespace(grip=0.6),already_active=True))
        self.assertFalse(grip_deadman_pressed(SimpleNamespace(grip=0.6),already_active=False))

    def test_trigger_release_interlock_and_rate(self):
        policy=TriggerPolicy(); p=[1200.]+[0.]*5
        def step(trigger,permit=True,dt=.02):
            return policy.step(permit=permit,trigger=trigger,positions=p,dt=dt,max_percent=30,speed_percent_s=10)
        self.assertIsNone(step(1.))
        self.assertEqual(step(0.),p)
        result=step(1.)
        self.assertEqual(result[0],1200.)
        self.assertEqual(result[1:],[20.]*5)
        self.assertEqual(step(1.,dt=5.)[1:],[50.]*5,'卡顿不能产生超大一步')
        self.assertIsNone(step(1.,permit=False))
        self.assertIsNone(step(1.),'重新 Grip 不允许沿用未释放的 Trigger')

    def test_invalid_input_and_limits(self):
        for limits in ((100,10,100),(30,99,100),(30,10,1000),(math.nan,10,100)):
            with self.assertRaises(ValueError): validate_limits(*limits)
        self.assertEqual(validate_limits(30,10,150),(30.,10.,150.))
        for t in (math.nan,math.inf,-1,2):
            with self.assertRaises(ValueError):
                TriggerPolicy().step(permit=True,trigger=t,positions=[0.]*6,dt=.02,max_percent=30,speed_percent_s=10)

    def test_complete_urdf_references_tree_and_fit(self):
        folder=complete_end_effector_folder()
        desc=json.loads((folder/'assembly.json').read_text(encoding='utf-8'))
        self.assertEqual(len(desc['links']),17); self.assertEqual(len(desc['joints']),16)
        self.assertEqual(len(desc['rigid']),3)
        self.assertLess(desc['registration']['hand_fit_mean_trimmed_mm'],1.)
        robot=ET.parse(folder/'jaka_s5_complete_display.urdf').getroot()
        links=[e.get('name') for e in robot.findall('link')]; self.assertEqual(len(links),len(set(links)))
        child_links=[]
        for joint in robot.findall('joint'):
            self.assertIn(joint.find('parent').get('link'),links)
            self.assertIn(joint.find('child').get('link'),links)
            child_links.append(joint.find('child').get('link'))
        self.assertEqual(len(child_links),len(set(child_links)))
        self.assertEqual(len(links)-len(child_links),1)
        for mesh in robot.findall('.//mesh'): self.assertTrue((folder/mesh.get('filename')).is_file())

    def test_feedback_contract_distinguishes_simulation(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1',0)); receiver.settimeout(1.)
            broadcaster=RobotVrBroadcaster(port=receiver.getsockname()[1])
            try:
                broadcaster.publish(RobotSnapshot(),armed=False,starting=False,active=False,target_tcp_mm_rad=None,
                                    hand_state={'connected':True,'feedback_valid':True,'simulated':True,'angles_deg':[0,3,8,8,8,8]})
                data=json.loads(receiver.recv(65535))
                self.assertTrue(data['hand_simulated']); self.assertEqual(data['hand_angles_deg'][1],3.)
            finally: broadcaster.close()

    def test_worker_connect_prepare_grip_stop_watchdog_reconnect(self):
        hand=HandController(live=False)
        def wait(predicate, seconds=3., permit=False, trigger=0., heartbeat=True):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                if heartbeat: hand.update(permit=permit,trigger=trigger)
                state=hand.snapshot()
                if predicate(state): return state
                time.sleep(.015)
            self.fail('超时，最后状态='+str(hand.snapshot()))
        try:
            hand.connect(); state=wait(lambda s:s.get('feedback_valid'))
            self.assertFalse(state.get('authorized')); self.assertEqual(state['positions'],[0.]*6)
            hand.prepare((30,10,150)); wait(lambda s:s.get('authorized'))
            wait(lambda s:s.get('moving'),permit=True,trigger=0.)
            state=wait(lambda s:s.get('positions',[0]*6)[2]>80,permit=True,trigger=1.)
            self.assertEqual(state['positions'][0],0.)
            state=wait(lambda s:not s.get('moving'),permit=False)
            frozen=state['positions'][2]
            # 失去心跳撤销授权，即使之前有有效 Trigger 也不会继续闭合。
            state=wait(lambda s:not s.get('authorized'),seconds=1.,heartbeat=False)
            self.assertEqual(state['positions'][2],frozen)
            self.assertIn('超时',state['status'])
            hand.prepare((30,10,150)); wait(lambda s:s.get('authorized'))
            hand.stop(); wait(lambda s:not s.get('authorized'))
            hand.close(); hand.connect(); wait(lambda s:s.get('feedback_valid'))
        finally: hand.close()

    def test_gui_panel_opens_without_hardware(self):
        root=tk.Tk(); root.withdraw()
        app=LevelALiveWindow(root,live=False,host='192.0.2.10',quest_port=0)
        try:
            app.open_hand_panel(); root.update()
            self.assertTrue(app.hand_panel.window.winfo_exists())
            self.assertFalse(app.hand_allowed.get()); self.assertIsNone(app.hand.process)
        finally: app.close()

    def test_arm_motion_trigger_independent_and_grip_resume(self):
        root=tk.Tk(); root.withdraw()
        app=LevelALiveWindow(root,live=False,host='192.0.2.10',quest_port=0)
        fixture=json.loads((ROOT/'Validation/unity_v2_fixture.json').read_text())
        frame=QuestUdpReceiver._frame(fixture)
        measured=RobotSnapshot(connected=True,powered_on=True,enabled=True,tool_id=1,
                              tcp_pose=(400.,100.,500.,0.,0.,0.),joints_rad=(0.,)*6)
        servo=[False]
        def snapshot(): return replace(measured,timestamp_ns=time.time_ns(),engineering_servo_active=servo[0])
        def tick(grip,trigger):
            app.latest=replace(frame,grip=grip,trigger=trigger,received_time_ns=time.time_ns())
            root.after_cancel(app.timer); app.tick()
        try:
            with patch.object(app.controller,'get_snapshot',side_effect=snapshot), \
                 patch.object(app.controller,'start_engineering_cartesian_servo',side_effect=lambda **kw:servo.__setitem__(0,True)), \
                 patch.object(app.controller,'stop_engineering_cartesian_servo',side_effect=lambda:servo.__setitem__(0,False)), \
                 patch.object(app.controller,'set_engineering_cartesian_target') as send:
                tick(0,0); app.lock_heading(); app.arm(); tick(0,0)
                tick(1,0); self.assertTrue(app.starting)
                tick(1,0); self.assertTrue(app.active)
                tick(1,1); self.assertTrue(app.active)
                tick(1,0); self.assertTrue(app.active)
                self.assertGreater(send.call_count,0)
                tick(0,1); self.assertFalse(app.active); self.assertTrue(app.armed)
                tick(1,1); tick(1,1); self.assertTrue(app.active)
                tick(0,0); self.assertFalse(app.active)
        finally: app.close()


if __name__=='__main__': unittest.main(verbosity=2)
