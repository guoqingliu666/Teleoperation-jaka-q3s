import json,tempfile,unittest
from pathlib import Path
import numpy as np
from unittest.mock import patch
from v2_servo.data_collection.episode import Episode,OnlineMatcher,event,STREAMS,action_vector,decode_action
from v2_servo.data_collection.dh116 import ForcePositionAdapter

class Tests(unittest.TestCase):
    def test_action_order_and_invalid(self):
        v=action_vector(np.arange(6),np.arange(6)*1000)
        self.assertEqual(decode_action(v)['position_target_eq'],[0,1000,2000,3000,4000,5000])
        for a,h in [([float('nan')]*6,[0]*6),([0]*6,[-1]*6),([0]*5,[0]*6)]:
            with self.assertRaises(ValueError):action_vector(a,h)
    def test_online_no_interpolation_or_stale_reuse(self):
        with tempfile.TemporaryDirectory() as d:
            m=OnlineMatcher(Path(d)/'align.jsonl')
            for s in STREAMS:m.push(event(s,0,{},host_ns=100000000,uncertainty_ns=0))
            self.assertTrue(m.match(100000000)['accepted'])
            self.assertFalse(m.match(100000000)['accepted'])
            for s in STREAMS:m.push(event(s,1,{},host_ns=200000000,uncertainty_ns=0))
            self.assertFalse(m.match(206000000)['accepted'])
            self.assertFalse(m.push(event('s5_state',0,{},host_ns=300000000)))
    def test_unknown_sample_clock_never_passes(self):
        with tempfile.TemporaryDirectory() as d:
            m=OnlineMatcher(Path(d)/'align.jsonl')
            for s in STREAMS:m.push(event(s,0,{},host_ns=100000000))
            self.assertFalse(m.match(100000000)['accepted'])
    def test_pairwise_span_rejects_two_opposite_offsets(self):
        with tempfile.TemporaryDirectory() as d:
            m=OnlineMatcher(Path(d)/'align.jsonl')
            for i,s in enumerate(STREAMS):m.push(event(s,0,{},host_ns=100000000+(4000000 if i%2 else -4000000),uncertainty_ns=0))
            r=m.match(100000000)
            self.assertEqual(r['pairwise_span_ns'],8000000);self.assertFalse(r['accepted'])
    def test_missing_stream_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            m=OnlineMatcher(Path(d)/'align.jsonl');m.push(event('s5_state',1,{}))
            self.assertIn('hand_state',m.match(1)['missing'])
    def test_bad_disk_payload_invalidates_episode(self):
        with tempfile.TemporaryDirectory() as d:
            e=Episode(d,{'stage':'A'});e.offer(event('s5_state',0,{'q_rad':[float('nan')]*6}));e.close()
            self.assertFalse(json.loads((e.path/'metadata.json').read_text())['complete'])
    def test_cannot_open_timestamps_exits_cleanly(self):
        original=Path.open
        def fail(path,*args,**kwargs):
            if path.name=='timestamps.csv':raise OSError('test disk unavailable')
            return original(path,*args,**kwargs)
        with tempfile.TemporaryDirectory() as d,patch.object(Path,'open',fail):
            e=Episode(d,{'stage':'A'});e.offer(event('s5_state',0,{'q_rad':[0]*6}));e.close()
            self.assertFalse(e.meta['complete']);self.assertIn('startup/fatal',e.errors[0])
    def test_overflow_never_silent(self):
        with tempfile.TemporaryDirectory() as d:
            e=Episode(d,{'stage':'A'},queue_size=1)
            # 明确锁住 writer 的取队列方法不可取；大量数据压测真正有界队列。
            for i in range(10000):e.offer(event('s5_state',i,{'q_rad':[0]*6}))
            e.close();self.assertGreater(e.dropped,0);self.assertFalse(e.meta['complete'])
    def test_force_write_blocked_without_units(self):
        class Vendor:
            def __getattr__(self,name):raise AssertionError('must not call hardware: '+name)
        a=ForcePositionAdapter(Vendor(),{'motion_approved':True,'motion_evidence':'x'},lambda _:None)
        with self.assertRaisesRegex(RuntimeError,'units'):a.command([100]*6,[1]*6)
    def test_vendor_wrapper_contract_and_safety_before_each_motor(self):
        calls=[]
        class Vendor:
            def __getattr__(self,name):return lambda *args:calls.append((name,args))
        cfg={'motion_approved':True,'motion_evidence':'test-only','current_unit':'fixture',
             'current_unit_evidence':'test-only','current_limit_raw':[10]*6,'vendor_return_contract_validated':True}
        a=ForcePositionAdapter(Vendor(),cfg,lambda _:calls.append(('safety',())))
        a.command([10]*6,[1]*6)
        self.assertEqual([v for n,v in calls if n=='move_motors'],[(i,) for i in range(1,7)])
        self.assertEqual([v for n,v in calls if n=='set_target_position'],[(i,10) for i in range(1,7)])
        self.assertEqual(len([n for n,v in calls if n=='safety']),13)
    def test_partial_hand_command_failure_stops_all(self):
        stopped=[]
        class Vendor:
            def __getattr__(self,name):
                if name=='set_target_position':return lambda *args:(_ for _ in ()).throw(RuntimeError('injected vendor failure'))
                if name=='stop_motors':return lambda i:stopped.append(i)
                return lambda *args:None
        cfg={'motion_approved':True,'motion_evidence':'test-only','current_unit':'fixture',
             'current_unit_evidence':'test-only','current_limit_raw':[10]*6,'vendor_return_contract_validated':True}
        a=ForcePositionAdapter(Vendor(),cfg,lambda _:None)
        with self.assertRaisesRegex(RuntimeError,'injected'):a.command([10]*6,[1]*6)
        self.assertEqual(stopped,list(range(1,7)))
if __name__=='__main__':unittest.main()
