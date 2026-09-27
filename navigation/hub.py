#!/usr/bin/env python3
"""Run once on the laptop; Start/Stop, goal, map and status stay on the iPhone."""
import argparse
import asyncio
import json
import socket
import time
from hawkeye import AGE, Navigator, motor_payload


class CarLink:
    def __init__(self, url):
        self.url, self.ws = url, None
        self.ack_at = -float('inf')
        self.seq = 0
        self.sent = {}

    @property
    def ready(self):
        return self.ws is not None and time.monotonic()-self.ack_at < AGE

    async def connect(self):
        from websockets.asyncio.client import connect
        while True:
            try:
                async with connect(self.url, open_timeout=3, close_timeout=1) as ws:
                    self.ws, self.ack_at = ws, -float('inf')
                    self.sent.clear()
                    async for raw in ws:
                        try:
                            ack = json.loads(raw).get('ack')
                            if type(ack) is int and ack in self.sent and time.monotonic()-self.sent[ack] < AGE:
                                self.ack_at = time.monotonic()
                                self.sent.pop(ack)
                        except (ValueError, AttributeError, TypeError):
                            pass
            except Exception as e:
                # Reconnect while the controller stays disarmed; the UI reports status.
                print(f'Car connection: {type(e).__name__}', flush=True)
            finally:
                self.ws = None
                self.ack_at = -float('inf')
            await asyncio.sleep(1)

    async def send(self, command):
        if self.ws is None:
            return
        self.seq += 1
        self.sent[self.seq] = time.monotonic()
        self.sent = {k: v for k, v in self.sent.items() if time.monotonic()-v < 1}
        try:
            await asyncio.wait_for(self.ws.send(json.dumps(motor_payload(command, self.seq))), .1)
        except Exception:
            self.ack_at = -float('inf')


class Receiver(asyncio.DatagramProtocol):
    def __init__(self, nav, car, camera=None):
        self.nav, self.car, self.camera = nav, car, camera
        self.peer = None
        self.transport = None
        self.reply_session = None
        self.reply_seq = -1
        self.reply_received = 0.0

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, raw, addr):
        try:
            msg = json.loads(raw)
            if not isinstance(msg, dict):
                return
            if self.peer != addr:
                if msg.get('type') != 'obs' or not isinstance(msg.get('sessionId'), str):
                    return
                if self.camera and msg.get('cameraId') != self.camera:
                    return
                if self.peer is not None and time.monotonic()-self.nav.received < 1:
                    return
                self.nav.stop('Camera changed — tap Start')
                self.peer = addr
            if (msg.get('type') == 'obs' and isinstance(msg.get('sessionId'), str)
                    and type(msg.get('seq')) is int and msg['seq'] >= 0):
                # Acknowledge receipt even when validation rejects the observation.
                # Otherwise the phone cannot display why the initial handshake failed.
                self.reply_session, self.reply_seq = msg['sessionId'], msg['seq']
                self.reply_received = time.time()
            self.nav.ingest(msg, car_ready=self.car is None or self.car.ready)
        except (ValueError, TypeError, UnicodeError):
            if addr == self.peer:
                self.nav.stop('Invalid camera packet')

    def reply(self):
        if self.peer and self.reply_session:
            status = self.nav.status()
            # Receive/send wall times let the phone measure its clock offset (NTP-style),
            # so the latency check doesn't depend on the two clocks agreeing.
            status.update(sessionId=self.reply_session, seq=self.reply_seq,
                          hubReceivedAt=self.reply_received, hubSentAt=time.time())
            if self.nav.state != 'blocked' and not self.nav.running and self.car is not None and not self.car.ready:
                status['message'] = 'Car not responding'
                status['state'] = 'blocked'
            self.transport.sendto(json.dumps(status, separators=(',', ':')).encode(), self.peer)


def advertise(args):
    from zeroconf import IPVersion, ServiceInfo, Zeroconf
    ip = args.host_ip
    if not ip:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect((args.car, args.car_port))
            ip = sock.getsockname()[0]
    zc = Zeroconf(interfaces=[ip], ip_version=IPVersion.V4Only)
    info = ServiceInfo('_carvision._udp.local.', 'Hawkeye Python._carvision._udp.local.',
                       addresses=[socket.inet_aton(ip)], port=args.port,
                       properties={'version': '1'}, server='hawkeye-python.local.')
    zc.register_service(info)
    print(f'iPhone discovery active on {ip}', flush=True)
    return zc, info


async def run(args):
    nav = Navigator(args.speed, args.dry_run)
    car = None if args.dry_run else CarLink(f'ws://{args.car}:{args.car_port}')
    receiver = Receiver(nav, car, args.camera)
    transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(
        lambda: receiver, local_addr=(args.bind, args.port))
    discovery = None
    task = None
    try:
        if not args.no_discovery:
            # zeroconf's synchronous registration must not run on the asyncio loop.
            discovery = await asyncio.to_thread(advertise, args)
        task = asyncio.create_task(car.connect()) if car else None
        print(('DRY RUN — motors disabled' if args.dry_run else 'Car enabled; waiting for Start on iPhone') + f' | UDP {args.port}', flush=True)
        last_status, last_message = 0, None
        while True:
            tick = time.monotonic()
            cmd = nav.command(tick, car is None or car.ready)
            if car:
                await car.send(cmd)
            if tick-last_status >= .1:
                receiver.reply()
                last_status = tick
            if nav.message != last_message:
                print(nav.message, flush=True)
                last_message = nav.message
            await asyncio.sleep(max(0, .05-(time.monotonic()-tick)))
    finally:
        nav.stop('Hub stopped')
        receiver.reply()
        if car:
            await car.send((0, 0, 0))
        if task:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        transport.close()
        if discovery:
            zc, info = discovery
            await asyncio.to_thread(zc.unregister_service, info)
            await asyncio.to_thread(zc.close)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--car', default='192.168.4.1')
    p.add_argument('--car-port', type=int, default=8765)
    p.add_argument('--port', type=int, default=47800)
    p.add_argument('--bind', default='0.0.0.0')
    p.add_argument('--camera', help='Optional camera ID; otherwise the first phone owns control')
    p.add_argument('--speed', type=float, default=.18, help='Normalized motor power, default 0.18')
    p.add_argument('--dry-run', action='store_true', help='Show route/status on phone without connecting to car')
    p.add_argument('--host-ip', help='Laptop Wi-Fi IP to advertise; auto-detected by default')
    p.add_argument('--no-discovery', action='store_true', help='Use the phone manual hub IP setting')
    args = p.parse_args()
    if not 0 < args.speed <= .3:
        p.error('Speed must be greater than zero and at most 0.3')
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print('Stopped')


if __name__ == '__main__':
    main()
