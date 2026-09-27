"""Actual UDP -> controller subprocess -> WebSocket fake firmware; no hardware."""
import asyncio
import importlib.util
import json
import socket
import sys
import tempfile
import time
import unittest
from pathlib import Path
from hawkeye import Scene
from simulate import observation


@unittest.skipUnless(importlib.util.find_spec('websockets'), 'Install requirements-autonomy.txt for network test')
class NetworkTests(unittest.IsolatedAsyncioTestCase):
    async def test_udp_websocket_stops_and_rearms(self):
        from websockets.asyncio.server import serve
        received = []
        acknowledge = True
        async def fake_car(ws):
            async for raw in ws:
                m = json.loads(raw)
                received.append(m)
                if acknowledge:
                    await ws.send(json.dumps({'ack': m['D']}))
        s = Scene()
        s.known = set(range(s.cols * s.rows))
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.bind(('127.0.0.1', 0))
        port = udp.getsockname()[1]
        udp.close()
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        proc = None
        seq = 0
        async def feed(seconds, car=None):
            nonlocal seq
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                udp.sendto(json.dumps(observation(s, seq, car)).encode(), ('127.0.0.1', port))
                seq += 1
                await asyncio.sleep(.04)
        async def command(text):
            proc.stdin.write((text + '\n').encode())
            await proc.stdin.drain()
        with tempfile.TemporaryDirectory() as tmp:
            async with serve(fake_car, '127.0.0.1', 0) as server:
                wsport = server.sockets[0].getsockname()[1]
                try:
                    proc = await asyncio.create_subprocess_exec(
                        sys.executable, str(Path(__file__).resolve().parents[1] / 'autonomous.py'),
                        '--camera', 'sim', '--live', '--car', '127.0.0.1', '--car-port', str(wsport),
                        '--bind', '127.0.0.1', '--port', str(port), '--preview', tmp + '/map.svg',
                        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                    await asyncio.sleep(.3)
                    await command('scan')
                    await feed(.5)
                    await command('freeze')
                    await feed(.2)
                    await command('goal 170 120')
                    await feed(.2, (30, 30))
                    self.assertTrue(received)
                    self.assertTrue(all(m['A'] == m['B'] == m['C'] == 0 for m in received))
                    await command('arm')
                    await feed(.4, (30, 30))
                    self.assertTrue(any(m['A'] or m['B'] for m in received))
                    await asyncio.sleep(.65)
                    self.assertEqual([received[-1][k] for k in 'ABC'], [0, 0, 0])
                    mark = len(received)
                    await feed(.25, (30, 30))
                    self.assertTrue(all(m['A'] == m['B'] == m['C'] == 0 for m in received[mark:]))
                    await command('arm')
                    await feed(.25, (30, 30))
                    self.assertNotEqual([received[-1][k] for k in 'AB'], [0, 0])
                    acknowledge = False
                    await feed(.65, (30, 30))
                    self.assertEqual([received[-1][k] for k in 'ABC'], [0, 0, 0])
                    self.assertTrue(Path(tmp + '/map.svg').exists())
                    await command('quit')
                    stdout, stderr = await asyncio.wait_for(proc.communicate(), 3)
                    self.assertEqual(proc.returncode, 0, stderr.decode() + stdout.decode())
                finally:
                    udp.close()
                    if proc and proc.returncode is None:
                        proc.kill()
                        await proc.wait()
