import json,tempfile,unittest
from pathlib import Path
from v2_servo.configuration import ROOT
from v2_servo.teleop.commissioning_guidance import GuidanceWriter,guidance_packet


class CommissioningGuidanceTests(unittest.TestCase):
    def test_XY_is_explicit_single_axis_display(self):
        for axis in ('X','Y'):
            live=guidance_packet('LIVE','pose',control_scope='TCP_'+axis)
            self.assertEqual(live['title'],'正式真机跟随 · Tool1基坐标'+axis)
            self.assertIn('当前只控制'+axis+'轴',live['instruction'])
            self.assertIn('Tool1基坐标'+axis,live['control_scope'])
    def test_TCP_stage_displays_Tool1_not_J1(self):
        live=guidance_packet('LIVE','pose',control_scope='TCP_Z')
        self.assertIn('Tool1',live['title']);self.assertIn('0..+0.5mm',live['control_scope'])
        self.assertNotIn('仅J1',live['control_scope']);self.assertIn('1秒',live['instruction'])
        self.assertEqual(live['stage'],'LIVE')
    def test_rehearsal_and_live_have_distinct_operator_instructions(self):
        rehearsal=guidance_packet('INPUT_GESTURE','pose')
        live=guidance_packet('LIVE','pose')
        self.assertIn('不动机械臂',rehearsal['title'])
        self.assertIn('正式真机',live['title'])
        self.assertIn('完成移动并松手后',live['instruction'])
        self.assertEqual(live['scope'],'operator_display_only_no_motion_permission')
        self.assertNotIn('q_rad',live) # 阶段提示不得冒充机器人反馈。
        self.assertNotIn('servo_enabled',live)

    def test_atomic_latest_display_does_not_leave_partial_file(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'data',prefix='guidance_test_') as folder:
            path=Path(folder)/'guidance.json';writer=GuidanceWriter(path)
            self.assertTrue(writer.update('INPUT_RELEASE','pose'))
            self.assertTrue(writer.update('LIVE','pose',detail='test only'))
            self.assertFalse(writer.update('LIVE','pose',detail='throttled'))
            actual=json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(actual['stage'],'LIVE');self.assertEqual(actual['detail'],'test only')
            self.assertTrue(writer.update('STOPPING','pose'))
            self.assertEqual(json.loads(path.read_text(encoding='utf-8'))['stage'],'STOPPING')
            self.assertEqual(list(Path(folder).glob('*.tmp')),[])

    def test_failed_display_write_is_not_motion_permission_or_supervisor_crash(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'data',prefix='guidance_error_') as folder:
            parent=Path(folder)/'not_directory';parent.write_text('x')
            writer=GuidanceWriter(parent/'guide.json')
            self.assertFalse(writer.update('LIVE','pose'));self.assertEqual(writer.errors,1)
            self.assertFalse((parent/'guide.json').exists())
