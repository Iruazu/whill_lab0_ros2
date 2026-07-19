#!/usr/bin/env python3
"""rosbridge websocket E2E smoke for whill_dispatch — the CLI proxy for AC6.

Exercises the exact path the tablet UI uses (roslibjs -> rosbridge ws ->
dispatch_node), but from a headless script so the dispatch boundary can be
regression-checked without a browser. It:

  1. opens ws://<host>:9090 (rosbridge)
  2. subscribes /dispatch/waypoints and asserts the named-waypoint list
     arrives over the socket
  3. advertises + publishes /dispatch/submit for a chosen waypoint
  4. watches /dispatch/state and asserts the phase walks
     QUEUED/ACTIVE -> SUCCEEDED with progress climbing to ~1.0
  5. (optional, --cancel) submits again and calls the /dispatch/cancel
     service over ws, asserting the phase reaches CANCELED

Dependency-free on purpose: no websocket-client / websockets / roslibpy on
the demo host, and the repo policy is "no pip installs, vendor instead"
(roslib.min.js is vendored the same way). This implements the minimal
RFC6455 client subset rosbridge needs (text frames, client masking, ping
reply) directly on a socket.

Prereq: dispatch_launch.py already running with a NavigateToPose server
(use_mock:=true for the no-robot case). This script does NOT start anything.

Usage:
    python3 scripts/m7_ws_smoke.py
    python3 scripts/m7_ws_smoke.py --host localhost --waypoint gate --cancel
"""

import argparse
import base64
import json
import os
import socket
import struct
import sys
import time


def _handshake(sock, host, port):
    key = base64.b64encode(os.urandom(16)).decode()
    req = (
        f'GET / HTTP/1.1\r\n'
        f'Host: {host}:{port}\r\n'
        f'Upgrade: websocket\r\n'
        f'Connection: Upgrade\r\n'
        f'Sec-WebSocket-Key: {key}\r\n'
        f'Sec-WebSocket-Version: 13\r\n\r\n'
    )
    sock.sendall(req.encode())
    resp = b''
    while b'\r\n\r\n' not in resp:
        chunk = sock.recv(1024)
        if not chunk:
            raise RuntimeError('server closed during handshake')
        resp += chunk
    if b'101' not in resp.split(b'\r\n', 1)[0]:
        raise RuntimeError(f'handshake not upgraded: {resp[:80]!r}')


def _send_text(sock, obj):
    payload = json.dumps(obj).encode()
    # Client frames MUST be masked (RFC6455 §5.3). FIN=1, opcode=0x1 (text).
    header = bytearray([0x81])
    n = len(payload)
    if n < 126:
        header.append(0x80 | n)
    elif n < 65536:
        header.append(0x80 | 126)
        header += struct.pack('>H', n)
    else:
        header.append(0x80 | 127)
        header += struct.pack('>Q', n)
    mask = os.urandom(4)
    header += mask
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    sock.sendall(bytes(header) + masked)


class _Reader:
    """Buffered RFC6455 text-frame reader (server frames are unmasked)."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = b''

    def _recv_exact(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError('server closed')
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def next_message(self):
        """Return the next full text message as a dict, replying to pings."""
        frags = []
        while True:
            b0, b1 = self._recv_exact(2)
            fin = b0 & 0x80
            opcode = b0 & 0x0F
            masked = b1 & 0x80
            length = b1 & 0x7F
            if length == 126:
                length = struct.unpack('>H', self._recv_exact(2))[0]
            elif length == 127:
                length = struct.unpack('>Q', self._recv_exact(8))[0]
            mask = self._recv_exact(4) if masked else b''
            data = self._recv_exact(length)
            if masked:
                data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))

            if opcode == 0x9:  # ping -> pong (echo payload)
                self._send_pong(data)
                continue
            if opcode == 0x8:  # close
                raise RuntimeError('server sent close')
            frags.append(data)
            if fin:
                return json.loads(b''.join(frags).decode())

    def _send_pong(self, data):
        header = bytearray([0x8A, 0x80 | len(data)])
        mask = os.urandom(4)
        header += mask
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        self.sock.sendall(bytes(header) + masked)


def _fail(msg):
    print(f'FAIL: {msg}')
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--waypoint', default='gate')
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--cancel', action='store_true',
                    help='also run a submit+cancel and assert CANCELED')
    args = ap.parse_args()

    sock = socket.create_connection((args.host, args.port), timeout=5.0)
    sock.settimeout(args.timeout)
    _handshake(sock, args.host, args.port)
    reader = _Reader(sock)

    _send_text(sock, {'op': 'subscribe', 'topic': '/dispatch/waypoints',
                      'type': 'std_msgs/String'})
    _send_text(sock, {'op': 'subscribe', 'topic': '/dispatch/state',
                      'type': 'std_msgs/String'})
    _send_text(sock, {'op': 'advertise', 'topic': '/dispatch/submit',
                      'type': 'std_msgs/String'})

    # 1) waypoints over ws
    deadline = time.time() + args.timeout
    names = None
    while time.time() < deadline:
        m = reader.next_message()
        if m.get('op') == 'publish' and m['topic'] == '/dispatch/waypoints':
            wps = json.loads(m['msg']['data'])
            names = [w['name'] for w in wps]
            break
    if not names:
        _fail('no /dispatch/waypoints received over ws')
    if args.waypoint not in names:
        _fail(f'waypoint {args.waypoint!r} not in {names}')
    print(f'PASS ws waypoints: {names}')

    # 2) submit -> phase walk to SUCCEEDED
    _send_text(sock, {'op': 'publish', 'topic': '/dispatch/submit',
                      'msg': {'data': json.dumps(
                          {'waypoint': args.waypoint, 'type': 'goto'})}})
    # /dispatch/state は 5 Hz で「最後の phase」を再送し続けるため、直前ジョブの
    # 終端 phase (SUCCEEDED 等) が submit 直後に届き得る。新ジョブの開始証拠
    # (非終端 phase = QUEUED/ACTIVE) を見るまで終端 phase を無視しないと、
    # stale な SUCCEEDED を新ジョブの完了と誤認する (2026-07-19 再現済み)。
    # 前提: このスクリプト起動時に走行中ジョブが無いこと。
    phases = []
    max_progress = 0.0
    started = False
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        m = reader.next_message()
        if m.get('op') == 'publish' and m['topic'] == '/dispatch/state':
            s = json.loads(m['msg']['data'])
            if not started:
                if s['phase'] not in ('QUEUED', 'ACTIVE'):
                    continue
                started = True
            if not phases or phases[-1] != s['phase']:
                phases.append(s['phase'])
            max_progress = max(max_progress, s.get('progress') or 0.0)
            if s['phase'] == 'SUCCEEDED':
                break
    if 'SUCCEEDED' not in phases:
        _fail(f'goto did not reach SUCCEEDED; phases={phases}')
    if 'ACTIVE' not in phases:
        _fail(f'goto never went ACTIVE; phases={phases}')
    if max_progress < 0.99:
        _fail(f'progress never reached ~1.0 (max {max_progress})')
    print(f'PASS ws goto: phases={phases} max_progress={max_progress:.2f}')

    # 3) optional cancel
    if args.cancel:
        _send_text(sock, {'op': 'publish', 'topic': '/dispatch/submit',
                          'msg': {'data': json.dumps(
                              {'waypoint': args.waypoint, 'type': 'goto'})}})
        # wait for ACTIVE, then call cancel over ws
        deadline = time.time() + args.timeout
        went_active = False
        while time.time() < deadline:
            m = reader.next_message()
            if m.get('op') == 'publish' and m['topic'] == '/dispatch/state':
                s = json.loads(m['msg']['data'])
                if s['phase'] == 'ACTIVE':
                    went_active = True
                    break
        if not went_active:
            _fail('cancel test: job never went ACTIVE')
        _send_text(sock, {'op': 'call_service',
                          'service': '/dispatch/cancel',
                          'type': 'std_srvs/Trigger', 'args': {}})
        got_cancel = False
        deadline = time.time() + args.timeout
        while time.time() < deadline:
            m = reader.next_message()
            if m.get('op') == 'publish' and m['topic'] == '/dispatch/state':
                s = json.loads(m['msg']['data'])
                if s['phase'] == 'CANCELED':
                    got_cancel = True
                    break
        if not got_cancel:
            _fail('cancel over ws did not reach CANCELED')
        print('PASS ws cancel: reached CANCELED')

    print('ALL WS SMOKE CHECKS PASSED')


if __name__ == '__main__':
    main()
