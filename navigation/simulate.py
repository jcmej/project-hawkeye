#!/usr/bin/env python3
"""Synthetic iPhone UDP feed plus a WebSocket fake car. Never connects to hardware."""
import argparse
import asyncio
import base64
import json
import math
import socket
import time
import uuid
from hawkeye import Scene


def observation(scene, seq, position=None, heading=0, session='simulation'):
    n = scene.cols * scene.rows
    def pack(indices):
        b = bytearray((n + 7) // 8)
        for i in indices:
            b[i // 8] |= 1 << (i % 8)
        return base64.b64encode(b).decode()
    return {'type': 'obs', 'cameraId': 'sim', 'sessionId': session, 'seq': seq,
            'sentAt': time.time(), 'calibrated': True, 'veto': False, 'fps': 20,
            'car': None if position is None else {'position': {'x': position[0], 'y': position[1]}, 'heading': heading},
            'carSource': None if position is None else 'marker', 'carConfidence': 1,
            'obstacles': [], 'arena': {'width': scene.width, 'height': scene.height,
             'carRadius': scene.radius, 'safetyMargin': scene.margin, 'obstacleRadius': scene.obstacle_radius},
            'grid': {'cols': scene.cols, 'rows': scene.rows,
                     'visible': pack(scene.known), 'occupied': pack(scene.occupied)}}


async def main(args):
    from websockets.asyncio.server import serve
    scene = Scene()
    scene.known = set(range(scene.cols * scene.rows))
    scene.occupied = {i for i in scene.known if 90 <= scene.center(i)[0] <= 110 and 50 <= scene.center(i)[1] <= 85}
    car = [30., 30., 0.]
    drive = [0., 0., 0.]
    last_command = 0.
    async def handler(ws):
        nonlocal last_command
        async for raw in ws:
            m = json.loads(raw)
            drive[:] = [m[k] / 1000 for k in ('A', 'B', 'C')]
            last_command = time.monotonic()
            await ws.send(json.dumps({'ack': m['D']}))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start, seq, session = time.monotonic(), 0, str(uuid.uuid4())
    print('Fake car: ws://127.0.0.1:8765. Synthetic phone: sim. First 20 seconds: empty-of-car scan. Then car at (30,30).', flush=True)
    try:
        async with serve(handler, '127.0.0.1', 8765):
            last = time.monotonic()
            while True:
                now = time.monotonic()
                dt, last = min(.1, now - last), now
                if now - last_command > .3:
                    drive[:] = [0., 0., 0.]
                x, y, omega = drive
                car[0] += (x * math.cos(car[2]) - y * math.sin(car[2])) * 30 * dt
                car[1] += (x * math.sin(car[2]) + y * math.cos(car[2])) * 30 * dt
                car[2] += omega * 1.8 * dt
                msg = observation(scene, seq, car[:2] if now - start > 20 else None, car[2], session)
                sock.sendto(json.dumps(msg).encode(), ('127.0.0.1', args.port))
                seq += 1
                await asyncio.sleep(.05)
    finally:
        sock.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', type=int, default=47800)
    try:
        asyncio.run(main(p.parse_args()))
    except KeyboardInterrupt:
        pass
