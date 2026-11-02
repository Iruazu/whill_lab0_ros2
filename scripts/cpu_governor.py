#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Install/check/remove #136's boot-time performance governor service."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import os
import time

POLICIES = Path('/sys/devices/system/cpu/cpufreq')
STATE = Path('/var/lib/whill-cpu-performance')
HELPER = Path('/usr/local/lib/whill-cpu-performance.py')
UNIT = Path('/etc/systemd/system/whill-cpu-performance.service')


def run(*args):
    subprocess.run(args, check=True)


def governors():
    values = {p.parent.name: p.read_text().strip()
              for p in sorted(POLICIES.glob('policy*/scaling_governor'))}
    if not values:
        raise RuntimeError('No CPUFreq policies found; cannot verify governor')
    return values


def check():
    values = governors()
    for policy, value in values.items():
        print(f'{policy}: {value}', flush=True)
    if any(value != 'performance' for value in values.values()):
        raise RuntimeError('Some CPU policies are not performance')
    print(f'PASS: all {len(values)} CPU policies are performance')


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action', choices=['install', 'apply', 'check', 'verify-reboot', 'remove'])
    action = ap.parse_args().action
    if action in ('check', 'verify-reboot'):
        check()
        if action == 'verify-reboot':
            state = json.loads((STATE / 'before.json').read_text())
            if state['boot_id'] == boot_id():
                raise RuntimeError('Boot ID unchanged since installation; reboot verification pending')
            run('systemctl', 'is-enabled', UNIT.name)
            run('systemctl', 'is-active', UNIT.name)
            print('PASS: service enabled and active; performance verified after reboot')
        return
    if os.geteuid() != 0:
        raise RuntimeError(f'{action} requires sudo on the host')
    if action == 'apply':
        run('/usr/bin/cpupower', '-c', 'all', 'frequency-set', '-g', 'performance')
        check()
        return
    # Refuse changes when systemd is inaccessible (including the agent sandbox).
    run('systemctl', 'show', '--property=Version', '--value')
    before = STATE / 'before.json'
    if action == 'remove':
        state = json.loads(before.read_text())
        if UNIT.read_text() != (STATE / 'unit.txt').read_text():
            raise RuntimeError('Unit changed externally; refusing to remove it')
        if HELPER.read_bytes() != (STATE / 'helper.py').read_bytes():
            raise RuntimeError('Installed helper changed externally; refusing to remove it')
        run('systemctl', 'disable', '--now', UNIT.name)
        # Restore each policy's original value, not an assumed powersave default.
        for policy, value in state['governors'].items():
            if not re.fullmatch(r'policy\d+', policy) or not re.fullmatch(r'[a-z_]+', value):
                raise RuntimeError('Invalid governor backup')
            path = POLICIES / policy / 'scaling_governor'
            if path.exists():
                path.write_text(value + '\n')
                if path.read_text().strip() != value:
                    raise RuntimeError(f'Could not restore {policy}')
        UNIT.unlink()
        HELPER.unlink()
        run('systemctl', 'daemon-reload')
        archive = STATE.with_name(STATE.name + f'-removed-{time.time_ns()}')
        STATE.rename(archive)
        print(f'Removed service and restored governors. Backup retained at {archive}')
        return
    if not Path('/usr/bin/cpupower').is_file():
        raise RuntimeError('cpupower is not installed')
    values = governors()
    for policy in values:
        available = (POLICIES / policy / 'scaling_available_governors').read_text().split()
        if 'performance' not in available:
            raise RuntimeError(f'{policy} does not support performance')
    # Start the installed power-profile manager first, then apply our governor.
    # Do not disable desktop power management or change its stored profile.
    ppd = any((Path(d) / 'power-profiles-daemon.service').exists()
              for d in ['/usr/lib/systemd/system', '/lib/systemd/system'])
    wants = 'Wants=power-profiles-daemon.service\n' if ppd else ''
    unit = (
        '[Unit]\nDescription=WHILL CPU performance governor (#136)\n'
        'After=power-profiles-daemon.service\n' + wants +
        '\n[Service]\nType=oneshot\nRemainAfterExit=yes\n'
        f'ExecStart=/usr/bin/python3 {HELPER} apply\nTimeoutStartSec=30\n'
        '\n[Install]\nWantedBy=multi-user.target\n')
    source = Path(__file__).resolve().read_bytes()
    for path, expected in [(UNIT, unit.encode()), (HELPER, source)]:
        if path.exists() and (not before.exists() or path.read_bytes() != expected):
            raise RuntimeError(f'Refusing to overwrite unmanaged/different file: {path}')
    STATE.mkdir(mode=0o755, parents=True, exist_ok=True)
    if not before.exists():
        before.write_text(json.dumps({'boot_id': boot_id(), 'governors': values}, indent=2) + '\n')
    (STATE / 'unit.txt').write_text(unit)
    (STATE / 'helper.py').write_bytes(source)
    HELPER.parent.mkdir(parents=True, exist_ok=True)
    HELPER.write_bytes(source)
    HELPER.chmod(0o644)
    UNIT.write_text(unit)
    UNIT.chmod(0o644)
    run('systemctl', 'daemon-reload')
    run('systemctl', 'enable', UNIT.name)
    run('systemctl', 'restart', UNIT.name)
    check()
    print('Installed. Reboot verification remains pending; no reboot was requested.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError, ValueError) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        sys.exit(1)
