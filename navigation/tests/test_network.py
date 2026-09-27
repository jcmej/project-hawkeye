"""Real loopback UDP + WebSocket. Never connects to hardware."""
import asyncio
import importlib.util
import json
import math
import socket
import sys
import time
import unittest
from pathlib import Path
from test_navigation import observation


@unittest.skipUnless(importlib.util.find_spec('websockets'), 'Install requirements-autonomy.txt')
class NetworkTest(unittest.IsolatedAsyncioTestCase):
    async def test_phone_controls_and_status(self):
        from websockets.asyncio.server import serve
        commands = []
        ack = True
        async def car(ws):
            async for raw in ws:
                m = json.loads(raw)
                commands.append(m)
                if ack:
                    await ws.send(json.dumps({'ack': m['D']}))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
            reserve.bind(('127.0.0.1', 0))
            port = reserve.getsockname()[1]
        phone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        phone.bind(('127.0.0.1', 0))
        phone.setblocking(False)
        status, seq = {}, 0
        async def feed(seconds, generation=0, action='stop'):
            nonlocal seq, status
            until = time.monotonic()+seconds
            while time.monotonic() < until:
                m = observation(seq=seq, generation=generation, request=action, hub=status.get('hubId', ''),
                                heading=math.atan2(90, 140))   # facing the goal, so it drives rather than turns
                m['sentAt'] = time.time()
                seq += 1
                phone.sendto(json.dumps(m).encode(), ('127.0.0.1', port))
                await asyncio.sleep(.04)
                while True:
                    try:
                        raw, _ = phone.recvfrom(65535)
                        status = json.loads(raw)
                    except BlockingIOError:
                        break
        async with serve(car, '127.0.0.1', 0) as server:
            car_port = server.sockets[0].getsockname()[1]
            process = await asyncio.create_subprocess_exec(
                sys.executable, str(Path(__file__).resolve().parents[1]/'hub.py'),
                '--car', '127.0.0.1', '--car-port', str(car_port), '--bind', '127.0.0.1',
                '--port', str(port), '--no-discovery', stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            try:
                await feed(.7)
                self.assertEqual(status.get('type'), 'navStatus')
                self.assertTrue(commands)
                self.assertTrue(all(m['A'] == m['B'] == m['C'] == 0 for m in commands))
                await feed(.35, 1, 'start')
                self.assertEqual(status['state'], 'driving')
                self.assertGreater(len(status['path']), 1)
                self.assertTrue(any(m['A'] or m['B'] for m in commands))
                # Explicit Stop must override later-arriving frames with old Start.
                phone.sendto(json.dumps({'type': 'stop', 'sessionId': 'phone', 'hubId': status['hubId'],
                                         'generation': 2, 'requestId': '2'}).encode(), ('127.0.0.1', port))
                await feed(.25, 1, 'start')
                self.assertEqual([commands[-1][k] for k in 'ABC'], [0, 0, 0])
                self.assertEqual(status['state'], 'stopped')
                await feed(.25, 3, 'start')
                self.assertEqual(status['state'], 'driving')
                # Lost observations stop; fresh observations don't auto-rearm.
                await asyncio.sleep(.6)
                self.assertEqual([commands[-1][k] for k in 'ABC'], [0, 0, 0])
                await feed(.25, 3, 'start')
                self.assertEqual(status['state'], 'stopped')
                await feed(.25, 4, 'start')
                self.assertEqual(status['state'], 'driving')
                ack = False
                await feed(.65, 4, 'start')
                self.assertEqual([commands[-1][k] for k in 'ABC'], [0, 0, 0])
                self.assertEqual(status['state'], 'blocked')
                self.assertEqual(status['message'], 'Car not responding')
            finally:
                phone.close()
                process.send_signal(2)
                stdout, stderr = await asyncio.wait_for(process.communicate(), 4)
                self.assertEqual(process.returncode, 0, stdout.decode()+stderr.decode())
