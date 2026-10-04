"""离线验收：不导入DLL、不占用网卡、没有实机运动。"""
import json
import tempfile
import unittest
from pathlib import Path
from v2_servo.hand import GraspLibrary, HandController, SimHandBackend, TriggerSample, SdkHandBackend
from v2_servo.hand.profiles import MOTOR_IDS

ROOT = Path(__file__).resolve().parents[1]


def fixture_config():
    # 所有数字仅是Mock契约参数，不是DH116可用的现场参数。
    return dict(hardware_model='DH116EC-R1-M', motor_ids=list(MOTOR_IDS), motion_approved=True,
                calibration_validated=True, transport_validated=True, vendor_return_contract_validated=True,
                motion_evidence='mock', calibration_evidence='mock', transport_evidence='mock',
                current_unit='mock', current_unit_evidence='mock', position_unit='mock', position_unit_evidence='mock',
                velocity_unit='mock', velocity_unit_evidence='mock', open_raw=[1000]*6,
                closed_raw=[9000,8000,7000,6000,5000,4000], position_lower_raw=[0]*6,
                position_upper_raw=[10000]*6, velocity_limit_raw=[200]*6, current_limit_raw=[10]*6,
                feedback_timeout_s=.15, approved_profiles=['cylinder'])


class FakeSdk:
    def __init__(self):
        self.calls = []
        self.fail = None
        self.alarm = 0
        self.current = 0
        self.status = 0

    def __getattr__(self, name):
        def call(motor, *args):
            if motor not in MOTOR_IDS:
                raise AssertionError('广播ID或错误电机编号')
            self.calls.append((name, motor, args))
            if self.fail == name:
                raise RuntimeError('mock SDK failure')
            return {'get_now_position':1000, 'get_now_current':self.current,
                    'get_enable':True, 'get_now_alarm':self.alarm, 'get_now_status':self.status}.get(name)
        return call


class GraspTests(unittest.TestCase):
    def setUp(self):
        self.library = GraspLibrary(ROOT/'config/dh116_grasps.json')
        self.backend = SimHandBackend()
        self.controller = HandController(self.library, self.backend)
        self.sequence = 0

    def feed(self, t, value, grip=True, **kwargs):
        self.sequence += 1
        self.controller.submit(TriggerSample(value, t, self.sequence, grip, **kwargs))
        return self.controller.tick(t)

    def test_all_grasp_endpoints_pregrasp_and_channels(self):
        self.assertTrue({'open','cylinder','pinch','tripod','lateral','hook','fist'} <= set(self.library.profiles))
        for key, p in self.library.profiles.items():
            for trigger, stage in [(0, 'open'), (.25, 'pregrasp'), (1, 'closed')]:
                self.assertEqual(self.library.target(key, trigger), tuple(p[stage]))
        self.assertEqual(self.library.target('pinch', 1)[3:], (0,0,0))
        self.assertEqual(self.library.target('hook', 1)[:2], (0,0))

    def test_bad_trigger_stops_instead_of_clipping(self):
        for value in (float('nan'), float('inf'), -1, 2, True):
            with self.subTest(value=value):
                with self.assertRaises(ValueError): self.feed(.1, value)
                self.assertIsNotNone(self.controller.fault)
        self.assertFalse(self.backend.commands)

    def test_boot_held_trigger_cannot_move(self):
        for i in range(10): self.feed(i*.02, 1)
        self.assertFalse(self.backend.commands)

    def test_rate_bounded_close_release_and_stationary_no_write(self):
        self.feed(0, 0, False)
        for i in range(1, 121): self.feed(i*.02, 1)
        closed = self.library.target('cylinder', 1)
        for a,b in zip(self.controller.q, closed): self.assertAlmostEqual(a,b)
        commands = list(self.backend.commands)
        for before, after in zip(commands, commands[1:]):
            dt = after[0]-before[0]
            self.assertLessEqual(max(abs(a-b) for a,b in zip(before[1],after[1])), self.controller.slew*dt+1e-9)
        count = len(commands)
        for i in range(121,131):self.feed(i*.02, 1)
        self.assertEqual(len(self.backend.commands),count)
        for i in range(131,251): self.feed(i*.02, 0)
        self.assertEqual(self.controller.q,(0.,)*6)

    def test_grip_release_stops_holds_and_regrip_requires_trigger_release(self):
        self.feed(0,0,False)
        self.feed(.02,1)
        self.feed(.04,1,False)
        held = self.controller.q
        count = len(self.backend.commands)
        self.assertGreater(self.backend.stops,0)
        self.feed(.06,1,True)
        self.assertEqual(self.controller.q,held)
        self.assertEqual(len(self.backend.commands),count)
        self.feed(.08,0,False)
        self.feed(.1,1,True)
        self.assertGreater(len(self.backend.commands),count)

    def test_timeout_latches_and_reset_requires_release(self):
        self.feed(0,0,False)
        self.feed(.02,1)
        self.controller.tick(.4)
        self.assertIsNotNone(self.controller.fault)
        with self.assertRaises(RuntimeError):self.controller.reset(.4)
        self.feed(.42,0,False)
        self.controller.reset(.42)
        self.assertIsNone(self.controller.fault)

    def test_replay_tracking_stop_and_profile_switch(self):
        self.feed(0,0,False)
        self.feed(.02,1)
        with self.assertRaises(RuntimeError):self.controller.select('pinch')
        with self.assertRaises(ValueError):self.controller.submit(TriggerSample(1,.02,2,True))
        self.assertIsNotNone(self.controller.fault)
        self.feed(.04,0,False)
        self.controller.reset(.04)
        self.feed(.06,0,False,tracking_valid=False)
        self.assertIsNotNone(self.controller.fault)
        self.feed(.08,0,False)
        self.controller.reset(.08)
        self.feed(.1,0,False,software_stop=True)
        self.assertIsNotNone(self.controller.fault)

    def test_feedback_stale_or_fault_stops(self):
        for mutate in (lambda f:f.update(stamp=-1), lambda f:f.update(wkc_ok=False),
                       lambda f:f.update(alarm=[1]*6), lambda f:f.update(status=[3]*6),
                       lambda f:f.update(position=[float('nan')]*6)):
            backend=SimHandBackend()
            original=backend.snapshot
            def snapshot(now):
                f=original(now);mutate(f);return f
            backend.snapshot=snapshot
            controller=HandController(self.library,backend)
            controller.tick(0)
            self.assertIsNotNone(controller.fault)
            self.assertEqual(backend.stops,1)
            self.assertFalse(backend.commands)

    def test_custom_save_readback(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'data') as directory:
            path=Path(directory)/'grasps.json'
            path.write_bytes(self.library.path.read_bytes())
            library=GraspLibrary(path)
            library.save_custom('测试动作',[.1,.2,.3,.4,.5,.6])
            loaded=GraspLibrary(path)
            self.assertEqual(loaded.target('custom',1),(.1,.2,.3,.4,.5,.6))
            self.assertEqual(loaded.profiles['custom']['validation'],'simulation_draft')

    def test_quest_trigger_shares_existing_arm_deadman_and_stop(self):
        from v2_servo.hand.quest_bridge import trigger_sample_from_quest
        payload=dict(sequence=12,right=dict(trigger=.7,connected=True,tracked=True,pose_valid=True))
        intent=dict(sequence=12,stamp=1.,deadman=False,software_stop=False)
        sample=trigger_sample_from_quest(payload,intent)
        self.assertEqual(sample.value,.7)
        self.assertFalse(sample.deadman)
        intent.update(deadman=True,software_stop=True)
        sample=trigger_sample_from_quest(payload,intent)
        self.assertTrue(sample.software_stop)
        payload['sequence']=11
        with self.assertRaises(ValueError):trigger_sample_from_quest(payload,intent)


class SdkContractTests(unittest.TestCase):
    def setUp(self):
        self.sdk=FakeSdk()
        self.meta=dict(stamp=1.,sequence=1,wkc_ok=True)
        self.cfg=fixture_config()
        self.backend=SdkHandBackend(self.sdk,self.cfg,lambda:self.meta,lambda _:None,clock=lambda:1.)

    def writes(self):
        return [call for call in self.sdk.calls if call[0].startswith('set_') or call[0]=='move_motors']

    def test_live_template_refuses_construction_without_sdk_calls(self):
        cfg=json.loads((ROOT/'config/dh116_hand_live.template.json').read_text(encoding='utf-8'))
        with self.assertRaises(RuntimeError):SdkHandBackend(self.sdk,cfg,lambda:self.meta,lambda _:None)
        self.assertFalse(self.sdk.calls)

    def test_explicit_one_based_ids_raw_mapping_velocity_and_current(self):
        self.backend.command([.5]*6,1.,'cylinder')
        positions=[(motor,args[0]) for name,motor,args in self.sdk.calls if name=='set_target_position']
        self.assertEqual(positions,[(i,v) for i,v in enumerate([5000,4500,4000,3500,3000,2500],1)])
        self.assertEqual([motor for name,motor,args in self.sdk.calls if name=='move_motors'],list(MOTOR_IDS))
        self.assertEqual(len(self.writes()),30)
        self.assertFalse(any(name in ('home_motors','set_enable','set_move_no_home','initial') for name,_,_ in self.sdk.calls))

    def test_reverse_direction_calibration(self):
        self.cfg['open_raw']=[9000]*6
        self.cfg['closed_raw']=[1000]*6
        self.backend.command([.25]*6,1.,'cylinder')
        self.assertEqual([args[0] for name,_,args in self.sdk.calls if name=='set_target_position'],[7000]*6)

    def test_partial_sdk_failure_requests_stop_all_six(self):
        self.sdk.fail='set_target_position'
        with self.assertRaisesRegex(RuntimeError,'mock SDK'):self.backend.command([.5]*6,1.,'cylinder')
        self.assertEqual([motor for name,motor,_ in self.sdk.calls if name=='stop_motors'],list(MOTOR_IDS))
        self.assertFalse(any(name=='move_motors' for name,_,_ in self.sdk.calls))

    def test_stale_alarm_limit_current_and_unapproved_profile_block_writes(self):
        for case in ('stale','wkc','alarm','current','limit','profile','gate'):
            self.setUp()
            profile='cylinder'
            if case=='stale':self.meta['stamp']=0
            if case=='wkc':self.meta['wkc_ok']=False
            if case=='alarm':self.sdk.alarm=1
            if case=='current':self.sdk.current=11
            if case=='limit':self.sdk.status=3
            if case=='profile':profile='pinch'
            if case=='gate':self.cfg['motion_approved']=False
            with self.subTest(case=case),self.assertRaises(RuntimeError):self.backend.command([.5]*6,1.,profile)
            self.assertFalse(self.writes())

    def test_safety_revocation_mid_transaction_stops_without_next_write(self):
        count=0
        def authorize(_):
            nonlocal count
            count+=1
            if count==3:raise RuntimeError('revoked')
        self.backend.authorize=authorize
        with self.assertRaisesRegex(RuntimeError,'revoked'):self.backend.command([.5]*6,1.,'cylinder')
        self.assertEqual(len(self.writes()),2)
        self.assertEqual([motor for name,motor,_ in self.sdk.calls if name=='stop_motors'],list(MOTOR_IDS))

    def test_same_pdo_cannot_be_restamped(self):
        self.backend.snapshot(1.)
        self.meta['stamp']=1.01
        with self.assertRaisesRegex(RuntimeError,'未收帧'):self.backend.snapshot(1.01)


if __name__=='__main__':unittest.main()
