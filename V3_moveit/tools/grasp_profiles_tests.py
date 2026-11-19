"""No hardware calls: settings integrity and loss/replay/stop behaviour."""
import copy
import tempfile
import unittest
from pathlib import Path
from grasp_profiles import Presets, TriggerInput, action_settings,action_words,defaults, targets, validate


def packet(seq, stamp, trigger=0, stop=False):
    return dict(source='quest-webxr', sequence=seq, unity_time_s=stamp,
                right=dict(connected=True, tracked=True, pose_valid=True,
                           trigger=trigger, thumbstick_click=stop))


class Profiles(unittest.TestCase):
    def test_three_independent_saved_profiles(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Presets(Path(folder)/'profiles.json')
            p.data['profiles'][1]['closed'][2]=2200;p.data['active']=1;p.save()
            q=Presets(p.path)
            self.assertEqual(q.profile['closed'][2],2200)
            self.assertEqual(q.data['profiles'][0]['closed'][2],1500)

    def test_invalid_hardware_parameters_are_rejected(self):
        for key,value in [('current_per_mille',1001),('current_per_mille',True),
                          ('velocity',-1),('normal_force_limit_n',float('nan')),
                          ('closed',[10001]*6),('open',[0]*5)]:
            d=defaults();d['profiles'][0][key]=value
            with self.assertRaises(ValueError):validate(d)

    def test_corrupt_file_is_not_silently_reported_as_loaded(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'p.json';path.write_text('{}')
            self.assertIsNotNone(Presets(path).error)

    def test_trigger_scales_each_endpoint(self):
        p=defaults()['profiles'][1]
        self.assertEqual(targets(p,.5),[750,750,750,0,0,0])
        for value in [float('nan'),float('inf'),-1,1.01,True]:
            with self.assertRaises(ValueError):targets(p,value)

    def test_legacy_speed_migrates_and_five_tracks_round_trip(self):
        data=defaults();p=data['profiles'][0];p['closed']=[8000,6000,10000,10000,10000,10000]
        a=action_settings(p);self.assertEqual((a['close_percent'],a['open_percent']),(10,10))
        a.update(close_percent=70,open_percent=30)
        for track,(start,end,force) in zip(a['tracks'],[(0,1000,2),(250,750,1.5),(500,1000,2),(0,500,2),(0,1000,2)]):
            track.update(start_ms=start,end_ms=end,force_n=force)
        p.update(a)
        self.assertEqual(targets(p,0),[0]*6)
        self.assertEqual(targets(p,.5),[4000,3000,5000,0,10000,5000])
        self.assertEqual(targets(p,1),p['closed'])
        self.assertEqual(list(map(int,action_words(p).split()[:4])),[1000,14000,6000,200])
        with tempfile.TemporaryDirectory() as folder:
            store=Presets(Path(folder)/'p.json');store.data=data;store.save()
            self.assertEqual(Presets(store.path).profile,p)
        for key,value in [('start_ms',1000),('end_ms',0),('force_n',float('nan')),('force_n',4.)]:
            invalid=copy.deepcopy(data);invalid['profiles'][0]['tracks'][0][key]=value
            with self.assertRaises(ValueError):validate(invalid)

    def test_start_and_resume_require_release(self):
        s=TriggerInput();self.assertFalse(s.consume(packet(1,0,.6),10))
        self.assertTrue(s.consume(packet(2,.01,0),10.01))
        self.assertTrue(s.consume(packet(3,.02,.7),10.02))
        self.assertFalse(s.consume(packet(4,.2,.8),10.2))
        self.assertTrue(s.consume(packet(5,.21,0),10.21))

    def test_replay_invalidates_held_lease(self):
        s=TriggerInput();s.consume(packet(1,0,0),10)
        with self.assertRaises(ValueError):s.consume(packet(1,.01,.5),10.01)
        self.assertFalse(s.valid(10.01))

    def test_loss_stop_and_tracking(self):
        s=TriggerInput();s.consume(packet(1,0,0),10)
        self.assertFalse(s.valid(10.081))
        s.consume(packet(2,.1,0),10.1)
        self.assertFalse(s.consume(packet(3,.11,.5,True),10.11))
        self.assertFalse(s.consume(packet(4,.12,0),10.12))
        p=packet(5,.13,0);p['right']['tracked']=False
        with self.assertRaises(ValueError):s.consume(p,10.13)

    def test_gradual_network_delay_cannot_refresh_input(self):
        s=TriggerInput();s.consume(packet(1,0,0),10)
        for i in range(2,5):s.consume(packet(i,(i-1)*.01,0),10+(i-1)*.03)
        with self.assertRaises(ValueError):s.consume(packet(5,.04,0),10.13)


if __name__=='__main__':unittest.main()
