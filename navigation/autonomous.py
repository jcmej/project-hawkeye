#!/usr/bin/env python3
"""Single-camera static-scene hub. Dry-run by default; --live enables WebSocket output."""
import argparse
import asyncio
import html
import json
import queue
import socket
import threading
import time
from pathlib import Path

from hawkeye import Controller, Scene, MAX_AGE, payload, point, number


def preview(scene, controller, destination):
    def dot(p, color, radius):
        return f'<circle cx="{p[0]}" cy="{scene.height-p[1]}" r="{radius}" fill="{color}"/>'
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {scene.width} {scene.height}" width="800" height="600">',
             '<rect width="100%" height="100%" fill="#d4d4d4"/>']
    for i in scene.known:
        x, y = scene.center(i)
        color = '#ef4444' if i in scene.occupied else '#fff'
        parts.append(f'<rect x="{x-2.5}" y="{scene.height-y-2.5}" width="5" height="5" fill="{color}"/>')
    for p in scene.obstacles.values():
        parts.append(dot(p, '#ef4444', scene.obstacle_radius))
    if controller.path:
        pts = ' '.join(f'{p[0]},{scene.height-p[1]}' for p in controller.path)
        parts.append(f'<polyline points="{pts}" fill="none" stroke="#2563eb" stroke-width="1"/>')
    if controller.goal:
        parts.append(dot(controller.goal, '#22c55e', 4))
    if controller.msg and controller.msg.get('car'):
        p = point(controller.msg['car']['position'])
        parts.append(dot(p, '#a78bfa', scene.radius + scene.margin))
        parts.append(dot(p, '#6d28d9', 3))
    parts.append(f'<title>{html.escape(controller.reason)}</title></svg>')
    Path(destination).write_text('\n'.join(parts))


def console(commands):
    while True:
        try:
            commands.put(input().strip())
        except EOFError:
            commands.put('quit')
            return


async def run(args):
    scene = Scene(args.width, args.height, args.car_radius, args.margin, args.obstacle_radius)
    controller = Controller(scene, args.camera, args.speed)
    commands = queue.Queue()
    threading.Thread(target=console, args=(commands,), daemon=True).start()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    sock.setblocking(False)
    ws, reader = None, None
    seq, ack_at = 0, -float('inf')
    sent = {}
    scanning = False
    last_print = 0
    last_capture_seq = -1

    async def read_acks():
        nonlocal ack_at
        async for raw in ws:
            try:
                ack = json.loads(raw).get('ack')
                if type(ack) is int and ack in sent and time.monotonic() - sent[ack] < MAX_AGE:
                    ack_at = time.monotonic()
                    sent.pop(ack)
            except (ValueError, AttributeError, TypeError):
                pass

    async def send(command):
        nonlocal seq
        seq += 1
        data = payload(command, seq)
        if ws:
            sent[seq] = time.monotonic()
            for old in list(sent):
                if time.monotonic() - sent[old] > 1:
                    sent.pop(old)
            await asyncio.wait_for(ws.send(json.dumps(data, separators=(',', ':'))), .15)
        return data

    try:
        if args.live:
            from websockets.asyncio.client import connect
            ws = await connect(f'ws://{args.car}:{args.car_port}', open_timeout=3, close_timeout=1)
            reader = asyncio.create_task(read_acks())
        print(f"{'LIVE MOTORS' if args.live else 'DRY RUN — no car connection'} | UDP {args.port} | camera {args.camera}", flush=True)
        print('Commands: scan | freeze | goal X Y | plan | arm | stop | reset | status | quit', flush=True)
        print('Remove the car and clear the phone goal before scan. Preview: ' + args.preview, flush=True)
        while True:
            tick = time.monotonic()
            # Bound work so a packet flood cannot starve the stop loop.
            for _ in range(100):
                try:
                    data, _ = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                try:
                    controller.ingest(json.loads(data))
                except (ValueError, UnicodeError):
                    continue
            if scanning and controller.msg and controller.seq != last_capture_seq:
                m = controller.msg
                if m['calibrated'] and time.monotonic() - controller.received <= MAX_AGE:
                    try:
                        scene.capture(m)
                        last_capture_seq = controller.seq
                    except (ValueError, KeyError, TypeError) as e:
                        scanning = False
                        print(f'Scan stopped: {e}', flush=True)
            if args.live and (reader.done() or time.monotonic() - ack_at > MAX_AGE):
                controller.stop('Car acknowledgments unavailable; re-arm after recovery')
                if reader.done():
                    await reader
                    raise ConnectionError('Car connection closed')
            while not commands.empty():
                command = commands.get().split()
                if not command:
                    continue
                op = command[0]
                try:
                    if op == 'quit':
                        return
                    elif op == 'stop':
                        controller.stop()
                    elif op == 'scan':
                        controller.stop('Scanning — motors stopped')
                        if scene.frozen:
                            raise ValueError('Use reset before rescanning')
                        scanning = True
                    elif op == 'freeze':
                        controller.stop('Map frozen')
                        if not scene.has_grid:
                            raise ValueError('Scan a map first')
                        scanning = False
                        scene.frozen = True
                        preview(scene, controller, args.preview)
                        print(f'Frozen {len(scene.known)} known cells, {len(scene.occupied)} occupied. Inspect {args.preview}', flush=True)
                    elif op == 'goal':
                        controller.stop('Goal changed')
                        if len(command) != 3:
                            raise ValueError('Usage: goal X Y (centimeters)')
                        controller.goal = (number(float(command[1])), number(float(command[2])))
                    elif op == 'plan':
                        controller.stop('Preview only')
                        if not controller.healthy(time.monotonic()) or controller.goal is None:
                            raise ValueError('Need a fresh visible marker and a goal')
                        controller.path = scene.plan(point(controller.msg['car']['position']), controller.goal)
                        preview(scene, controller, args.preview)
                        print(f'{len(controller.path)} path points; preview saved (zero means no path)', flush=True)
                    elif op == 'arm':
                        if scanning:
                            raise ValueError('Freeze the map first')
                        if args.live and time.monotonic() - ack_at > MAX_AGE:
                            raise ValueError('No fresh car acknowledgment')
                        controller.arm(time.monotonic())
                    elif op == 'reset':
                        controller.stop('Map reset')
                        scanning = False
                        scene.known.clear()
                        scene.occupied.clear()
                        scene.obstacles.clear()
                        scene.has_grid = scene.frozen = False
                        controller.path = []
                        last_capture_seq = -1
                    elif op == 'status':
                        preview(scene, controller, args.preview)
                        print(f'{controller.reason}; known={len(scene.known)}/{scene.cols*scene.rows}; seq={controller.seq}', flush=True)
                    else:
                        raise ValueError('Unknown command')
                except (ValueError, TypeError, KeyError) as e:
                    controller.stop(str(e))
                    print(f'Stopped: {e}', flush=True)
            command = controller.command(time.monotonic())
            data = await send(command)
            if tick - last_print > 1:
                print(f'{controller.reason} | {json.dumps(data)}', flush=True)
                last_print = tick
            await asyncio.sleep(max(0, .05 - (time.monotonic() - tick)))
    finally:
        controller.stop('Exiting')
        if ws:
            try:
                await send((0, 0, 0))
            except Exception:
                pass  # Firmware watchdog also stops on command loss.
            if reader:
                reader.cancel()
            await ws.close()
        sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', required=True, help='Exact Camera ID shown in the iPhone app')
    parser.add_argument('--live', action='store_true', help='Enable real motor commands')
    parser.add_argument('--car', default='192.168.4.1')
    parser.add_argument('--car-port', type=int, default=8765)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=47800)
    parser.add_argument('--width', type=float, default=200)
    parser.add_argument('--height', type=float, default=150)
    parser.add_argument('--car-radius', type=float, default=13)
    parser.add_argument('--margin', type=float, default=5)
    parser.add_argument('--obstacle-radius', type=float, default=12)
    parser.add_argument('--speed', type=float, default=.18)
    parser.add_argument('--preview', default='hawkeye-map.svg')
    args = parser.parse_args()
    for v in (args.width, args.height, args.car_radius, args.margin, args.obstacle_radius, args.speed):
        if number(v) <= 0:
            parser.error('Dimensions, margins and speed must be positive')
    if args.width > 1000 or args.height > 1000 or args.speed > .3:
        parser.error('Demo limits: arena <= 1000 cm per side; speed <= 0.3')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print('Stopped')


if __name__ == '__main__':
    main()
