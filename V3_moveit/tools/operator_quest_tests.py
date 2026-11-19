"""Quest transport recovery tests; mock ADB, HTTPS and server process."""
import tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import operator_quest as q

class QuestTransport(unittest.TestCase):
    def test_device_authorization_and_ambiguity(self):
        self.assertEqual(q.authorized_device('List of devices attached\nabc\tdevice\n'),'abc')
        for text,why in [('abc\tunauthorized','允许'),('abc\toffline','离线'),('abc\tdevice\ndef\tdevice','多台')]:
            with self.assertRaisesRegex(RuntimeError,why):q.authorized_device('List of devices attached\n'+text)
    def test_wrong_port_is_not_accepted(self):
        self.assertFalse(q.has_reverse('UsbFfs tcp:8443 tcp:8444'));self.assertTrue(q.has_reverse('UsbFfs tcp:8443 tcp:8443\n'))
    def test_missing_reverse_is_repaired_and_confirmed(self):
        with tempfile.TemporaryDirectory() as temp,patch.object(q,'ROOT',Path(temp)),patch.object(q,'ensure_server') as server,patch.object(q,'run',side_effect=['List of devices attached\nabc device\n','','','UsbFfs tcp:8443 tcp:8443']) as run:
            result=q.ensure_connection();self.assertTrue(result['ok']);self.assertTrue(result['reverse_repaired']);server.assert_called_once();self.assertIn(['adb','-s','abc','reverse','tcp:8443','tcp:8443'],[c.args[0] for c in run.call_args_list])
    def test_healthy_link_is_not_rebound_on_periodic_check(self):
        with tempfile.TemporaryDirectory() as temp,patch.object(q,'ROOT',Path(temp)),patch.object(q,'ensure_server'),patch.object(q,'run',side_effect=['List of devices attached\nabc device\n','UsbFfs tcp:8443 tcp:8443','UsbFfs tcp:8443 tcp:8443']) as run:
            self.assertFalse(q.ensure_connection()['reverse_repaired']);self.assertEqual(run.call_count,3)
    def test_failed_reverse_readback_is_an_error(self):
        with tempfile.TemporaryDirectory() as temp,patch.object(q,'ROOT',Path(temp)),patch.object(q,'ensure_server'),patch.object(q,'run',side_effect=['List of devices attached\nabc device\n','','','']):
            with self.assertRaisesRegex(RuntimeError,'未建立'):q.ensure_connection()
    def test_existing_https_server_is_reused(self):
        with patch.object(q,'https_ready',return_value=True),patch.object(q.subprocess,'Popen') as start:q.ensure_server();start.assert_not_called()

if __name__=='__main__':unittest.main()
