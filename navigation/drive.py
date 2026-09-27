"""
Drive the Zeus Car from any laptop with the arrow keys.

  Up / Down      forward / back
  Left / Right   turn in place (hold with Up/Down to steer while driving)
  Esc            stop and quit

The laptop must be on the car's Zeus_Car-XXXXXX Wi-Fi (password 12345678) and
the car must run ZeusCarFirmware.ino. Commands go out 20 times a second as the
same {"A":vx,"B":vy,"C":omega,"D":seq} messages CarVisionHub sends. The car
stops by itself if they stop arriving for 300 ms, so letting go of the keys,
closing this window, or losing Wi-Fi all stop it.

    pip install -r requirements.txt
    python drive.py              # car at the default 192.168.4.1
    python drive.py 192.168.4.1  # or give its IP
"""

import json
import sys
import threading
import time

import pygame
from websockets.sync.client import connect

CAR_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.4.1"
URL = f"ws://{CAR_IP}:8765"

SPEED = 600    # forward/back, out of 1000
TURN = 500     # turning, out of 1000
SEND_HZ = 20   # the firmware stops the car after 300 ms without a command


class Car:
    """WebSocket link to the car. Reconnects on its own and tracks the car's acks."""

    def __init__(self, url):
        self.url = url
        self.ws = None
        self.seq = 0
        self.last_ack = 0.0
        self.status = f"Connecting to {url}..."
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                with connect(self.url, open_timeout=3) as ws:
                    self.ws = ws
                    self.status = "Connected, waiting for the car to answer..."
                    for msg in ws:
                        try:
                            if "ack" in json.loads(msg):
                                self.last_ack = time.monotonic()
                        except (ValueError, TypeError):
                            pass  # not one of ours
                self.status = "Car closed the connection, reconnecting..."
            except Exception as e:
                self.status = f"Can't reach car ({type(e).__name__}). On the Zeus_Car Wi-Fi?"
            self.ws = None
            time.sleep(1)

    def send(self, vx, omega):
        ws = self.ws
        if ws is None:
            return
        self.seq = (self.seq + 1) % 100_000
        msg = json.dumps({"A": vx, "B": 0, "C": omega, "D": self.seq}, separators=(",", ":"))
        try:
            ws.send(msg)
        except Exception:
            pass  # the reader thread notices the drop and reconnects

    @property
    def answering(self):
        return time.monotonic() - self.last_ack < 1


def describe(vx, omega):
    parts = []
    if vx:
        parts.append("forward" if vx > 0 else "back")
    if omega:
        parts.append("turning left" if omega > 0 else "turning right")
    return ", ".join(parts) or "stopped"


def main():
    pygame.init()
    screen = pygame.display.set_mode((520, 170))
    pygame.display.set_caption("Zeus Car")
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()
    car = Car(URL)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False

        keys = pygame.key.get_pressed()
        vx = SPEED * (keys[pygame.K_UP] - keys[pygame.K_DOWN])
        omega = TURN * (keys[pygame.K_LEFT] - keys[pygame.K_RIGHT])  # positive = counterclockwise = left
        car.send(vx, omega)

        if car.answering:
            status, color = "Car OK", (80, 200, 120)
        else:
            status, color = car.status, (230, 160, 60)
        screen.fill((25, 25, 30))
        for i, (text, c) in enumerate([
            (status, color),
            (f"Driving: {describe(vx, omega)}", (230, 230, 230)),
            ("Arrow keys drive, Esc quits. Keep this window focused.", (150, 150, 150)),
        ]):
            screen.blit(font.render(text, True, c), (20, 25 + i * 45))
        pygame.display.flip()
        clock.tick(SEND_HZ)

    for _ in range(3):
        car.send(0, 0)
        time.sleep(0.05)
    pygame.quit()


if __name__ == "__main__":
    main()
