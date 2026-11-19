#!/usr/bin/env python3
"""Compile current driver method bodies against native fake SDK functions only.

This deliberately does not import ROS, load the vendor shared library, create a
socket, or run a robot session. Generated sources/binary/report stay in project
directories. Run: python3 -B tools/hardware_lifecycle_tests.py
"""
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / 'src/v3_moveit/cpp/jaka_system.cpp'
METHODS = ('checked', 'preflight', 'on_activate', 'on_deactivate', 'on_cleanup',
           'on_error', 'on_shutdown', 'fail', 'stop_servo', 'poll', 'worker',
           'flush_edg_audit', 'shutdown')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def method_text(source, name):
    """Extract a whole definition, balancing braces outside strings/comments."""
    matches = list(re.finditer(
        rf'^  (?:static )?(?:void|bool|CB) {re.escape(name)}\([^\n]*',
        source, re.MULTILINE))
    if len(matches) != 1:
        raise ValueError(f'Expected exactly one current definition of {name}')
    start = matches[0].start()
    opening = source.find('{', matches[0].start())
    depth, state, escaped, i = 0, 'code', False, opening
    while i < len(source):
        c, nxt = source[i], source[i:i + 2]
        if state == 'line':
            if c == '\n':
                state = 'code'
        elif state == 'block':
            if nxt == '*/':
                state = 'code'
                i += 1
        elif state in ('"', "'"):
            if escaped:
                escaped = False
            elif c == '\\':
                escaped = True
            elif c == state:
                state = 'code'
        elif nxt in ('//', '/*'):
            state = 'line' if nxt == '//' else 'block'
            i += 1
        elif c in ('"', "'"):
            state = c
        elif c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                method = source[start:i + 1]
                # Only remove the ROS interface override specifier; preserve body.
                signature, body = method.split('{', 1)
                return signature.replace(' override', '') + '{' + body
        i += 1
    raise ValueError(f'Unclosed definition of {name}')


def run():
    out = ROOT / '.tmp/hardware_lifecycle_tests'
    artifacts = ROOT / 'artifacts'
    out.mkdir(parents=True, exist_ok=True)
    artifacts.mkdir(exist_ok=True)
    report_path = artifacts / 'hardware_lifecycle_tests.json'
    if report_path.exists():
        backup = artifacts / 'backups'
        backup.mkdir(exist_ok=True)
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        (backup / f'hardware_lifecycle_tests.{stamp}.json').write_bytes(report_path.read_bytes())
    source_bytes = DRIVER.read_bytes()
    source = source_bytes.decode()
    test_source = ROOT / 'tools/hardware_lifecycle_tests.cpp'
    dependencies = [DRIVER, test_source, Path(__file__),
                    ROOT / 'src/v3_moveit/include/v3_moveit/guard.hpp',
                    ROOT / 'src/v3_moveit/include/v3_moveit/edg_feedback.hpp',
                    ROOT / 'src/v3_moveit/include/v3_moveit/jaka_sdk.hpp']
    dependencies += sorted((ROOT / 'vendor/jaka/include').glob('*.h'))
    evidence = {str(p.relative_to(ROOT)): sha(p.read_bytes()) for p in dependencies}
    report = dict(passed=False, scope='Current extracted C++ driver methods with fake C SDK functions',
                  driver_sha256=sha(source_bytes), evidence=evidence,
                  robot_connections=0, robot_commands=0, vendor_sdk_linked=False,
                  ros_session_started=False, physical_acceptance=False,
                  stop_time_guarantee=False, framework_lifecycle_concurrency_tested=False,
                  started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    try:
        methods = {name: method_text(source, name) for name in METHODS}
        members = re.search(r'^  JKHD handle_.*?(?=^  rclcpp::Node::SharedPtr)',
                            source, re.MULTILINE | re.DOTALL)
        if not members:
            raise ValueError('Current driver member declaration block not found')
        (out / 'hardware_lifecycle_methods.inc').write_text('\n'.join(methods.values()) + '\n')
        (out / 'hardware_lifecycle_members.inc').write_text(members.group())
        report['method_sha256'] = {name: sha(text.encode()) for name, text in methods.items()}
        binary = out / 'hardware_lifecycle_tests'
        command = ['g++', '-std=c++17', '-O0', '-g', '-Wall', '-Wextra', '-pthread',
                   '-Wl,--no-undefined', '-I', str(ROOT / 'src/v3_moveit/include'),
                   '-I', str(ROOT / 'vendor/jaka/include'), '-I', str(out),
                   str(test_source), '-o', str(binary)]
        env = dict(os.environ, TMPDIR=str(out), PYTHONDONTWRITEBYTECODE='1')
        report['compile_command'] = command
        compiled = subprocess.run(command, text=True, capture_output=True, env=env, timeout=90)
        report['compile_output'] = compiled.stdout + compiled.stderr
        if compiled.returncode:
            raise RuntimeError('Native fake-SDK compile failed: ' + report['compile_output'])
        # readelf inspects metadata without loading or running any shared library.
        dynamic = subprocess.run(['readelf', '-d', str(binary)], text=True,
                                 capture_output=True, check=True, timeout=10)
        needed = re.findall(r'Shared library: \[(.*?)\]', dynamic.stdout)
        allowed = {'libstdc++.so.6', 'libm.so.6', 'libgcc_s.so.1', 'libc.so.6',
                   'libpthread.so.0', 'libatomic.so.1'}
        report['elf_needed'] = needed
        if not needed or set(needed) - allowed:
            raise RuntimeError(f'Unexpected native test shared dependency: {needed}')
        executed = subprocess.run([str(binary)], text=True, capture_output=True,
                                  env=env, timeout=30)
        report['output'] = executed.stdout + executed.stderr
        report['cases'] = len(re.findall(r'^PASS ', executed.stdout, re.MULTILINE))
        report['binary_sha256'] = sha(binary.read_bytes())
        if executed.returncode:
            raise RuntimeError('Native fake-SDK test failed: ' + report['output'])
        if report['cases'] < 28 or f"TOTAL {report['cases']} cases" not in executed.stdout:
            raise RuntimeError('Native fake-SDK test output incomplete')
        changed = [path for path, digest in evidence.items()
                   if sha((ROOT / path).read_bytes()) != digest]
        if changed:
            raise RuntimeError(f'Sources changed during test: {changed}')
        report['passed'] = True
        print(report['output'], end='')
    except Exception as error:
        report['error'] = str(error)
        print(report['error'], file=sys.stderr)
    finally:
        report['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print(f'Report: {report_path}')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(run())
