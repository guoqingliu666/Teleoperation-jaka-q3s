"""Version-pinned, reversible EtherCAT compatibility fix for installed LHandPro.

The vendor 1.4.5 executable calls SIOCSIFFLAGS despite having only NET_RAW,
and emits a multicast source MAC rejected by the user's Ethernet switch.
Only the unnecessary flags ioctl and the source MAC's first byte are changed.
No hand command, state machine, SDK, joint parameter or firmware is patched.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile

APP = Path('/opt/lhandpro/LHandPro')
BACKUP = APP.with_name('LHandPro.vendor-1.4.5.original')
SHA256 = '5bcc05cc1f5fba5f138e029018118b22b7fb1fe6da81d401f13327a93fc9735f'
OUT = Path(__file__).resolve().parents[1] / 'artifacts/lhandpro_connection_fix_20261115'
# ELF virtual addresses equal file offsets in these two LOAD segments.
PATCHES = (
    # EAX was zeroed immediately before this call. Retain that success value;
    # all socket creation, interface lookup and bind failures remain checked.
    (0x33a08d, bytes.fromhex('e88e55d3ff'), b'\x90' * 5),
    # ec_setupheader loads this broadcast destination + first source word.
    # Retain source word 1 (01:01), which SOEM uses for RX route matching.
    (0x1f70a30, bytes.fromhex('ffffffffffff0101'), bytes.fromhex('ffffffffffff0201')),
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def patched(original):
    if digest(original) != SHA256:
        raise ValueError('Unknown vendor executable; refusing version-dependent patch')
    result = bytearray(original)
    for offset, before, after in PATCHES:
        if result[offset:offset + len(before)] != before or len(before) != len(after):
            raise ValueError('Unexpected instruction/data bytes')
        result[offset:offset + len(before)] = after
    return bytes(result)


def require_stopped():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            name = (proc / 'comm').read_text().strip()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if name == 'LHandPro':
            raise RuntimeError('Close LHandPro before installing/restoring')


def replace_installed(data):
    fd, name = tempfile.mkstemp(prefix='.LHandPro-fix-', dir=APP.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
            os.fchown(stream.fileno(), 0, 0)
            os.fchmod(stream.fileno(), 0o755)
        subprocess.run(['/usr/sbin/setcap', 'cap_net_raw=ep', name], check=True)
        os.replace(name, APP)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'install', 'restore', 'check'))
    args = parser.parse_args()
    if APP.is_symlink() or not stat.S_ISREG(APP.stat().st_mode):
        raise ValueError('Unexpected application path')
    current = APP.read_bytes()
    original = current if digest(current) == SHA256 else BACKUP.read_bytes()
    replacement = patched(original)
    if digest(current) not in (SHA256, digest(replacement)):
        raise ValueError('Installed executable is neither pinned original nor our patch')
    report = dict(original_sha256=SHA256, patched_sha256=digest(replacement),
                  installed_sha256=digest(current), source_mac='02:01:01:01:01:01',
                  changed_bytes=sum(a != b for a, b in zip(original, replacement)),
                  backup=str(BACKUP), patches=[dict(offset=hex(o), before=a.hex(),
                  after=b.hex()) for o, a, b in PATCHES])
    if args.action == 'prepare':
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / 'LHandPro.original').write_bytes(original)
        (OUT / 'LHandPro.patched').write_bytes(replacement)
        (OUT / 'patch_manifest.json').write_text(json.dumps(report, indent=2) + '\n')
    elif args.action in ('install', 'restore'):
        if os.geteuid() != 0:
            raise PermissionError('Installation requires system authentication')
        require_stopped()
        if BACKUP.exists():
            if BACKUP.is_symlink() or digest(BACKUP.read_bytes()) != SHA256:
                raise ValueError('Unexpected backup; refusing overwrite')
        elif args.action == 'install':
            with BACKUP.open('xb') as stream:
                stream.write(original)
                stream.flush()
                os.fsync(stream.fileno())
            BACKUP.chmod(0o644)
        replace_installed(replacement if args.action == 'install' else original)
        report['installed_sha256'] = digest(APP.read_bytes())
    report['action'] = args.action
    report['installed_patch'] = report['installed_sha256'] == digest(replacement)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
