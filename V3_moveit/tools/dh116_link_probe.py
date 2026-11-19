"""Bounded EtherCAT BRD diagnostic; no state, PDO or motor writes.

Frame layout/BRD/register constants follow the manufacturer's bundled SOEM
src/ec_base.c and include/soem/ec_type.h. This is a link diagnostic, not a
replacement master or a hand-feedback/acceptance test.
"""
import argparse
import json
import os
from pathlib import Path
import secrets
import socket
import struct
import time
from datetime import datetime, timezone

ETHERTYPE = 0x88A4
SOURCE = bytes.fromhex('010101010101')  # Manufacturer SOEM primary MAC.
LOCAL_SOURCE = bytes.fromhex('02ec00000116')  # Diagnostic-only local unicast MAC.
REGISTERS = {0x0000: 12, 0x0130: 2, 0x0134: 2}


def request(index, register, source=SOURCE):
    length = REGISTERS[register]
    header = b'\xff' * 6 + source + struct.pack('!H', ETHERTYPE)
    datagram = struct.pack('<BBHHHH', 7, index, 0, register, length, 0)
    return (header + struct.pack('<H', 0x1000 | (12 + length)) +
            datagram + bytes(length + 2)).ljust(60, b'\x00')


def response(frame, packet_type, index, register, source=SOURCE):
    length = REGISTERS[register]
    if packet_type == socket.PACKET_OUTGOING or len(frame) < 28 + length:
        return None
    if frame[:14] != b'\xff' * 6 + source + struct.pack('!H', ETHERTYPE):
        return None
    if struct.unpack_from('<H', frame, 14)[0] != 0x1000 | (12 + length):
        return None
    command, found_index, adp, ado, flags, irq = struct.unpack_from('<BBHHHH', frame, 16)
    if (command, found_index, ado, flags) != (7, index, register, length):
        return None
    wkc = struct.unpack_from('<H', frame, 26 + length)[0]
    if wkc == 0:
        return None
    return dict(wkc=wkc, address_position=adp, data_hex=frame[26:26+length].hex(), raw_hex=frame.hex())


def probe(interface, source=SOURCE):
    info = Path('/sys/class/net') / interface
    if not info.is_dir() or not (info/'device').exists() or (info/'wireless').exists():
        raise ValueError('Select a physical wired interface')
    if (info/'operstate').read_text().strip() != 'up':
        raise ValueError('Interface is not up; this diagnostic never changes link settings')
    rows = []; received = []
    # No SIOCSIFFLAGS, promiscuous mode, IP, routes, DHCP or NetworkManager edits.
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETHERTYPE)) as sock:
        sock.bind((interface, 0))
        if os.geteuid() == 0 and 'PKEXEC_UID' in os.environ:
            import pwd
            uid = int(os.environ['PKEXEC_UID']); account = pwd.getpwuid(uid)
            os.setgroups([]); os.setgid(account.pw_gid); os.setuid(uid)
        index = secrets.randbelow(200)
        for _ in range(3):
            for register in REGISTERS:
                index += 1
                tx = request(index, register, source); sent = time.monotonic()
                sock.send(tx)
                result = None
                deadline = sent + .3
                while time.monotonic() < deadline:
                    sock.settimeout(max(.001, deadline-time.monotonic()))
                    try: frame, address = sock.recvfrom(2048)
                    except socket.timeout: break
                    if len(received)<100:received.append(dict(packet_type=address[2],raw_hex=frame.hex()))
                    result = response(frame, address[2], index, register, source)
                    if result:
                        result['roundtrip_s'] = time.monotonic()-sent
                        break
                rows.append(dict(register=hex(register), index=index, tx_hex=tx.hex(), response=result))
                time.sleep(.1)
    return dict(generated_at_utc=datetime.now(timezone.utc).isoformat(), interface=interface,
                ethercat_response_seen=any(r['response'] for r in rows),
                read_requests=len(rows), state_writes=0, pdo_writes=0, motor_commands=0,
                network_configuration_changed=False, hand_feedback_verified=False,
                source_mac=source.hex(':'), received=received, reads=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--local-source', action='store_true')
    args = parser.parse_args()
    try:
        result = probe(args.interface, LOCAL_SOURCE if args.local_source else SOURCE)
    except Exception as error:
        result = dict(ethercat_response_seen=False, hand_feedback_verified=False, error=str(error))
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    raise SystemExit(0 if result.get('ethercat_response_seen') else 1)
