import unittest
from v2_servo.runtime import WindowsTimerResolution,WindowsThreadPriority,Deadline,CommissioningPythonScheduling
from v2_servo.safety import SafetyFault


class WindowsTimingTests(unittest.TestCase):
    def test_finite_child_gc_and_switch_restored_after_exception(self):
        import gc,sys
        before=(gc.isenabled(),sys.getswitchinterval())
        policy=CommissioningPythonScheduling()
        try:
            policy.start()
            self.assertFalse(gc.isenabled());self.assertAlmostEqual(sys.getswitchinterval(),.001)
            with self.assertRaises(RuntimeError):policy.start()
            raise ValueError('simulated child error')
        except ValueError:pass
        finally:report=policy.close()
        self.assertEqual((gc.isenabled(),sys.getswitchinterval()),before)
        self.assertTrue(report['gc_restored']);self.assertTrue(report['switch_interval_restored'])
        self.assertIsNone(policy.close())

    def test_child_policy_preserves_already_disabled_gc(self):
        import gc,sys
        before=(gc.isenabled(),sys.getswitchinterval());policy=CommissioningPythonScheduling()
        try:
            gc.disable();policy.start();policy.close()
            self.assertFalse(gc.isenabled());self.assertEqual(sys.getswitchinterval(),before[1])
        finally:
            if before[0]:gc.enable()
            else:gc.disable()
    def test_begin_and_cleanup_match_once(self):
        calls=[]
        class API:
            def timeBeginPeriod(self,value):calls.append(('begin',value));return 0
            def timeEndPeriod(self,value):calls.append(('end',value));return 0
        timer=WindowsTimerResolution(API())
        self.assertTrue(timer.start())
        with self.assertRaises(RuntimeError):timer.start()
        timer.close();timer.close()
        self.assertEqual(calls,[('begin',1),('end',1)])

    def test_failed_begin_cannot_be_claimed_active(self):
        class API:
            def timeBeginPeriod(self,value):return 97
            def timeEndPeriod(self,value):raise AssertionError('not active')
        timer=WindowsTimerResolution(API())
        with self.assertRaises(SafetyFault):timer.start()
        self.assertFalse(timer.active);timer.close()

    def test_thread_priority_is_read_back_and_restored(self):
        calls=[]
        class API:
            priority=0
            def GetCurrentThread(self):return 7
            def GetThreadPriority(self,handle):return self.priority
            def SetThreadPriority(self,handle,value):self.priority=value;calls.append((handle,value));return 1
        api=API();priority=WindowsThreadPriority(api)
        self.assertTrue(priority.start());self.assertEqual(api.priority,1)
        priority.close();priority.close();self.assertEqual(api.priority,0)
        self.assertEqual(calls,[(7,1),(7,0)])
    def test_thread_priority_failure_is_not_active(self):
        class API:
            def GetCurrentThread(self):return 7
            def GetThreadPriority(self,handle):return 0
            def SetThreadPriority(self,handle,value):return 0
        priority=WindowsThreadPriority(API())
        with self.assertRaises(SafetyFault):priority.start()
        self.assertFalse(priority.active)
    def test_bounded_fine_wait_does_not_catch_up_after_a_miss(self):
        current=[0.];sleeps=[]
        def clock():current[0]+=.00001;return current[0]
        def sleep(dt):sleeps.append(dt);current[0]+=dt
        timer=Deadline(.008,clock,sleep,spin_s=.0005)
        timer.wait();first,_=timer.wait()
        self.assertTrue(sleeps);self.assertLess(sleeps[0],.008)
        current[0]+=.020;late_stamp,late=timer.wait();self.assertGreater(late,.011)
        following,_=timer.wait();self.assertGreaterEqual(following-late_stamp,.008)
        with self.assertRaises(ValueError):Deadline(spin_s=.002)
