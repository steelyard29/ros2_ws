#!/usr/bin/env python3
"""Keep the serial Micro XRCE-DDS Agent running.

PX4 1.16 uxrce_dds_client does not recover if this Agent is killed and restarted.
Start it once, leave it up, and power-cycle the flight controller only if the
client is already stuck.
"""
from __future__ import annotations

import argparse
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / 'evidence' / 'xrce_serial_agent.log'
PID_FILE = Path('/tmp/robocup_xrce_serial_agent.pid')
ALLOWED = ('/dev/ttyTHS1', '/dev/ttyUSB0')


def agent_command(device: str, baud: int) -> list[str]:
    return ['MicroXRCEAgent', 'serial', '-D', device, '-b', str(baud), '-v', '4']


def _cmdline(pid: int) -> str:
    try:
        return Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\x00', b' ').decode()
    except OSError:
        return ''


def running_pid(device: str, baud: int) -> int | None:
    needle_dev = f'-D {device}'
    needle_baud = f'-b {baud}'
    try:
        out = subprocess.check_output(['pgrep', '-af', 'MicroXRCEAgent'], text=True)
    except subprocess.CalledProcessError:
        return None
    for line in out.splitlines():
        if needle_dev in line and needle_baud in line and 'pgrep' not in line:
            try:
                return int(line.split(None, 1)[0])
            except ValueError:
                continue
    return None


def occupants(device: str) -> list[int]:
    check = subprocess.run(['fuser', device], capture_output=True, text=True)
    if check.returncode == 1:
        return []
    if check.returncode != 0:
        raise RuntimeError('Serial occupancy check failed')
    text = (check.stdout or '') + ' ' + (check.stderr or '')
    pids = []
    for token in text.replace(':', ' ').split():
        if token.isdigit():
            pids.append(int(token))
    return pids


def configure_port(device: str, baud: int) -> None:
    subprocess.run(
        ['stty', '-F', device, str(baud), 'cs8', '-cstopb', '-parenb',
         '-ixon', '-ixoff', '-crtscts', '-hupcl', 'clocal'],
        check=False, capture_output=True)


def ensure(device: str = '/dev/ttyTHS1', baud: int = 921600) -> dict:
    if device not in ALLOWED:
        raise RuntimeError('unsupported serial device')
    if not Path(device).exists():
        raise RuntimeError('Serial device absent')
    pid = running_pid(device, baud)
    if pid is not None:
        held = occupants(device)
        if held and pid not in held:
            raise RuntimeError('XRCE Agent exists but does not own '+device)
        return {'pid': pid, 'started': False, 'log': str(LOG), 'device': device, 'baud': baud}

    held = occupants(device)
    if held:
        names = ', '.join(f'{p}:{_cmdline(p).strip()[:80]}' for p in held)
        raise RuntimeError('Serial port occupied: '+names)

    LOG.parent.mkdir(parents=True, exist_ok=True)
    configure_port(device, baud)
    log = LOG.open('a')
    log.write('\n# ensure %s %s %s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), device, baud))
    log.flush()
    proc = subprocess.Popen(
        agent_command(device, baud), stdout=log, stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True)
    PID_FILE.write_text(str(proc.pid)+'\n')
    time.sleep(0.4)
    if proc.poll() is not None:
        raise RuntimeError('MicroXRCEAgent exited immediately; see '+str(LOG))
    live = running_pid(device, baud)
    if live is None:
        raise RuntimeError('MicroXRCEAgent did not stay running; see '+str(LOG))
    return {'pid': live, 'started': True, 'log': str(LOG), 'device': device, 'baud': baud}


def log_offset() -> int:
    try:
        return LOG.stat().st_size
    except OSError:
        return 0


def session_established(log_path: Path | None = None, since: int = 0) -> bool:
    path = log_path or LOG
    if not path.exists():
        return False
    try:
        data = path.read_bytes()[since:].decode('utf-8', 'replace')
    except OSError:
        return False
    return 'client_key' in data or 'session established' in data.lower()


def wait_for_session(timeout: float, extra_check=None, log_path: Path | None = None) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if session_established(log_path):
            return True
        if extra_check is not None and extra_check():
            return True
        time.sleep(0.2)
    return session_established(log_path) or (extra_check() if extra_check else False)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('action', nargs='?', default='ensure', choices=('ensure', 'status'))
    parser.add_argument('--device', default='/dev/ttyTHS1')
    parser.add_argument('--baud', type=int, default=921600)
    args = parser.parse_args()
    if args.action == 'status':
        pid = running_pid(args.device, args.baud)
        print({'pid': pid, 'session': session_established(since=max(0, log_offset()-8000)), 'log': str(LOG)})
        return 0 if pid else 1
    info = ensure(args.device, args.baud)
    print(info)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print('Error:', exc)
        raise SystemExit(1)
