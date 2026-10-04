import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from v2_servo.configuration import ROOT
from v2_servo.teleop.pose_evidence import verify_pose_shadow


class PoseEvidenceTests(unittest.TestCase):
    def test_passed_synthetic_and_failed_actual_never_authorize_H8(self):
        for scope,passed,reason in (('synthetic_UDP_combined_6dof_shadow_not_hardware',True,'synthetic'),
                                   ('real_Quest_combined_6dof_shadow_no_robot_SDK',False,'actual passed')):
            report=dict(scope=scope,passed_shadow_only=passed,failure=None,robot_commands=0,active_rejected=0)
            with patch('v2_servo.teleop.pose_evidence.report_path',return_value=(
                    ROOT/'data/quest_pose_shadow_20261003_164220_269953/report.json',report)), \
                 patch('v2_servo.teleop.pose_evidence.time.time',return_value=1791016940.):
                with self.assertRaisesRegex(ValueError,reason):verify_pose_shadow('unused',{})

    def test_live_intent_roundtrips_async_logger_without_array_error(self):
        from v2_servo.teleop.quest_pose_live import QuestPoseLiveMapping
        from v2_servo.runtime import JsonLogger
        with (ROOT/'data/h6_20261003_130933_206345/events.jsonl').open(encoding='utf-8') as lines:
            raw=next(json.loads(line)['raw'] for line in lines if json.loads(line).get('event')=='Quest_raw_input')
        raw.update(sequence=1,unity_time_s=1.);raw['right']['grip']=0.
        mapping=QuestPoseLiveMapping([[0,0,1],[-1,0,0],[0,1,0]])
        frame=mapping.decode(raw,('127.0.0.1',1234),1.)
        with tempfile.TemporaryDirectory(dir='D:/ChatGPT/Temp') as folder:
            path=Path(folder)/'audit.jsonl';logger=JsonLogger(path)
            logger.emit(event='test_intent',quest_intent=frame);logger.close()
            r=json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(r['quest_intent']['input_mode'],'pose6dof_live')
            self.assertEqual(len(r['quest_intent']['axes']),6)
