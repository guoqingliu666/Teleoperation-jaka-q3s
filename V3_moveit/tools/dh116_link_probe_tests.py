"""Reject local echoes, zero WKC, stale indices and malformed frames."""
import socket
import struct
import unittest
from dh116_link_probe import request, response, REGISTERS

class Frames(unittest.TestCase):
    def test_no_echo_or_unprocessed_packet_is_feedback(self):
        frame=request(27,0)
        self.assertIsNone(response(frame,socket.PACKET_OUTGOING,27,0))
        self.assertIsNone(response(frame,socket.PACKET_BROADCAST,27,0))
    def test_processed_reply_and_request_identity(self):
        frame=bytearray(request(27,0));frame[26:38]=bytes.fromhex('110100000003000000000000')
        struct.pack_into('<H',frame,38,1)
        self.assertEqual(response(frame,socket.PACKET_BROADCAST,27,0)['wkc'],1)
        self.assertIsNone(response(frame,socket.PACKET_BROADCAST,28,0))
        self.assertIsNone(response(frame,socket.PACKET_BROADCAST,27,0x0130))
        self.assertIsNone(response(frame[:30],socket.PACKET_BROADCAST,27,0))
    def test_only_read_commands_and_whitelisted_registers(self):
        for register,length in REGISTERS.items():
            frame=request(1,register)
            self.assertEqual(frame[16],7)
            self.assertEqual(frame[26:28+length],bytes(length+2))
        with self.assertRaises(KeyError):request(1,0x0120)

if __name__=='__main__':unittest.main()
