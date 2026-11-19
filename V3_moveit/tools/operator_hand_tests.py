import copy,time,unittest
from operator_hand import decode_sample
from operator_media import current_state

def sample():
    raw=bytearray(192);raw[:2]=bytes([0,27]);positions=[-11,-88,-8,-12,-57,-5]
    for i,p in enumerate(positions):raw[2+6*i:4+6*i]=p.to_bytes(2,'little',signed=True)
    raw[164:170]=bytes([6]*6)
    return dict(event='sample',valid=True,wkc=1,decode_result=0,received_monotonic=100.,sequence=2,
                raw_hex=raw.hex(),raw_positions=positions,positions=[0]*6,axis_status=[6]*6)

class Feedback(unittest.TestCase):
    def test_sdk_zero_preserves_actual_negative_feedback(self):
        d=decode_sample(sample(),100.1,1)
        self.assertTrue(d['feedback_verified']);self.assertEqual(d['raw_positions'],[-11,-88,-8,-12,-57,-5]);self.assertFalse(d['homed']);self.assertNotIn('position',d)
    def test_default_buffer_is_not_feedback(self):
        d=sample();d.update(raw_hex=bytes(192).hex(),raw_positions=[0]*6,axis_status=[0]*6)
        with self.assertRaises(ValueError):decode_sample(d,100.1)
    def test_missing_transport_or_decode_cannot_refresh_feedback(self):
        for change in [dict(wkc=0),dict(valid=False),dict(decode_result=8),dict(positions=[1]*6),dict(raw_positions=[0]*6),dict(axis_status=[0]*6)]:
            d=sample();d.update(change)
            with self.assertRaises(ValueError):decode_sample(d,100.1)
    def test_old_future_repeated_and_expired_state(self):
        for now,previous in [(101.,1),(99.,1),(100.1,2)]:
            with self.assertRaises(ValueError):decode_sample(sample(),now,previous)
        self.assertFalse(current_state(decode_sample(sample(),100.1),102.)['ok'])

if __name__=='__main__':unittest.main()
