"""Pinned, reversible SDK fix for DH116 EtherCAT re-enable in LHandPro 1.4.5.

After set_enable's successful neutral-command phase, mark cached velocity and
current dirty for EtherCAT only. The next requested movement resends the exact
cached parameters. No target, limit, enable command, error check or firmware is
changed. Other transport modes take the original tail call unchanged.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import tempfile

LIB = Path('/opt/lhandpro/libLHandProLib.so')
BACKUP = LIB.with_name('libLHandProLib.so.vendor-1.4.5.original')
ORIGINAL_SHA = '9314f185db1c51223d075fd7296aa65b8b45e13a94897117f0184fe346224b97'
OUT = Path(__file__).resolve().parents[1] / 'artifacts/lhandpro_motion_tactile_20261115'
CAVE = 0x37b00
TAIL = 0x18487
SEND_CTRL_PLT = 0x15480


def sha(data):
    return hashlib.sha256(data).hexdigest()


def patched(original):
    if sha(original) != ORIGINAL_SHA:
        raise ValueError('Unknown SDK build; refusing patch')
    data = bytearray(original)
    assert data[TAIL:TAIL+5] == bytes.fromhex('e9f4cfffff')
    # rdi already holds LHandProLibPrivate*. set_enable's normal return paths
    # converge here after restoring all callee-saved registers.
    code = bytes.fromhex('83bf0401000000750966c787a41500000101')
    code += b'\xe9' + struct.pack('<i', SEND_CTRL_PLT - (CAVE + len(code) + 5))
    assert len(code) == 23 and data[CAVE:CAVE+len(code)] == bytes(len(code))
    phoff = struct.unpack_from('<Q', data, 32)[0]
    phsize, phnum = struct.unpack_from('<HH', data, 54)
    assert (phoff, phsize, phnum) == (64, 56, 12)
    ph = phoff + 3 * phsize
    assert struct.unpack_from('<IIQQQQQQ', data, ph) == (
        1, 5, 0x13000, 0x13000, 0x13000, 0x24ab5, 0x24ab5, 0x1000)
    # Extend the existing RX segment into verified empty padding, within its
    # existing last page. The next read-only segment starts at 0x38000.
    size = CAVE + len(code) - 0x13000
    struct.pack_into('<QQ', data, ph + 32, size, size)
    data[CAVE:CAVE+len(code)] = code
    data[TAIL:TAIL+5] = b'\xe9' + struct.pack('<i', CAVE - (TAIL + 5))
    return bytes(data)


def require_stopped():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if (proc / 'comm').read_text().strip() == 'LHandPro':
                raise RuntimeError('Disconnect and close LHandPro first')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass


def replace(data):
    fd, name = tempfile.mkstemp(prefix='.lhp-motion-fix-', dir=LIB.parent)
    try:
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
            os.fchown(f.fileno(), 0, 0)
            os.fchmod(f.fileno(), 0o644)
        os.replace(name, LIB)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'check', 'install', 'restore'])
    action = parser.parse_args().action
    if LIB.is_symlink() or not LIB.is_file():
        raise ValueError('Unexpected SDK path')
    current = LIB.read_bytes()
    original = current if sha(current) == ORIGINAL_SHA else BACKUP.read_bytes()
    replacement = patched(original)
    if sha(current) not in (ORIGINAL_SHA, sha(replacement)):
        raise ValueError('Installed SDK is not the pinned original or this patch')
    if action == 'prepare':
        for folder, data in [('original_sdk', original), ('patched_sdk', replacement)]:
            target = OUT / folder
            target.mkdir(parents=True, exist_ok=True)
            (target / LIB.name).write_bytes(data)
    elif action in ('install', 'restore'):
        if os.geteuid() != 0:
            raise PermissionError('System authentication required')
        require_stopped()
        if BACKUP.exists():
            if BACKUP.is_symlink() or sha(BACKUP.read_bytes()) != ORIGINAL_SHA:
                raise ValueError('Unexpected backup; refusing overwrite')
        elif action == 'install':
            with BACKUP.open('xb') as f:
                f.write(original)
                f.flush()
                os.fsync(f.fileno())
            BACKUP.chmod(0o644)
        replace(replacement if action == 'install' else original)
    report = dict(action=action, original_sha256=ORIGINAL_SHA,
                  patched_sha256=sha(replacement), installed_sha256=sha(LIB.read_bytes()),
                  backup=str(BACKUP), tail_offset=hex(TAIL), code_offset=hex(CAVE),
                  dirty_fields=['velocity', 'current'], transport='EtherCAT only')
    if action == 'prepare':
        (OUT / 'motion_patch_manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
