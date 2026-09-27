"""
Drive the Zeus Car from any laptop with the arrow keys.

  Up / Down      forward / back
  Left / Right   turn in place (hold with Up/Down to steer while driving)
  Esc            stop and quit

Lights:
  Up, Up, Down, Down within 5 s   lights off -> on and into color mode,
                                  lights on -> off
  In color mode, 1-11             pick a color (see COLORS). Digits typed
                                  within 3 s of each other are one number,
                                  so 1 then 1 is 11 and 1 then 0 is 10.
  0 (as the first digit)          leave color mode (lights stay on)

The laptop must be on the car's Zeus_Car-XXXXXX Wi-Fi (password 12345678) and
the car must run firmware.ino. Commands go out 20 times a second as the same
{"A":vx,"B":vy,"C":omega,"D":seq} messages CarVisionHub sends, plus the light
color as "E","F","G" (r, g, b). The car stops by itself if they stop arriving
for 300 ms, so letting go of the keys, closing this window, or losing Wi-Fi
all stop it.

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

SPEED = 950    # forward/back, out of 1000
TURN = 800     # turning, out of 1000
SEND_HZ = 20   # the firmware stops the car after 300 ms without a command

# Color number -> (name, (r, g, b)). The firmware evens out the LEDs'
# brightness, so these are ordinary color values; tweak any that look off.
COLORS = {
    1: ("white", (255, 255, 255)),
    2: ("black", (0, 0, 0)),
    3: ("red", (255, 0, 0)),
    4: ("green", (0, 255, 0)),
    5: ("yellow", (255, 255, 0)),
    6: ("blue", (0, 0, 255)),
    7: ("brown", (139, 69, 19)),
    8: ("orange", (255, 165, 0)),
    9: ("pink", (255, 105, 180)),
    10: ("purple", (160, 32, 240)),
    11: ("gray", (128, 128, 128)),
}
COMBO = [pygame.K_UP, pygame.K_UP, pygame.K_DOWN, pygame.K_DOWN]
COMBO_SECONDS = 5   # the whole combo must fit in this
DIGIT_SECONDS = 3   # digits closer together than this form one number


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

    def send(self, vx, omega, rgb):
        ws = self.ws
        if ws is None:
            return
        self.seq = (self.seq + 1) % 100_000
        r, g, b = rgb
        msg = json.dumps({"A": vx, "B": 0, "C": omega, "D": self.seq, "E": r, "F": g, "G": b},
                         separators=(",", ":"))
        try:
            ws.send(msg)
        except Exception:
            pass  # the reader thread notices the drop and reconnects

    @property
    def answering(self):
        return time.monotonic() - self.last_ack < 1


class Lights:
    """The light state and the key sequences that change it (see the top of this file)."""

    def __init__(self):
        self.on = False
        self.color = 1
        self.color_mode = False
        self.digits = ""        # color number being typed
        self.last_digit = 0.0
        self.arrows = []        # (key, time) of the last few Up/Down presses
        self.message = ""

    @property
    def rgb(self):
        return COLORS[self.color][1] if self.on else (0, 0, 0)

    def key(self, event, now):
        if event.key in (pygame.K_UP, pygame.K_DOWN):
            self.arrows = (self.arrows + [(event.key, now)])[-4:]
            if [k for k, _ in self.arrows] == COMBO and now - self.arrows[0][1] <= COMBO_SECONDS:
                self.arrows = []
                self.toggle()
        elif self.color_mode and event.unicode and event.unicode in "0123456789":
            self.digit(int(event.unicode), now)

    def toggle(self):
        self.on = not self.on
        self.color_mode = self.on
        self.digits = ""
        self.message = "Lights on. Type a color number, 0 to finish." if self.on else "Lights off"

    def digit(self, d, now):
        if not self.digits and d == 0:
            self.color_mode = False
            self.message = "Left color mode"
            return
        self.digits += str(d)
        self.last_digit = now
        # Don't make them wait when no second digit could give a valid color.
        if int(self.digits) * 10 > max(COLORS):
            self.commit()

    def update(self, now):
        if self.digits and now - self.last_digit > DIGIT_SECONDS:
            self.commit()

    def commit(self):
        n = int(self.digits)
        self.digits = ""
        if n in COLORS:
            self.color = n
            self.message = f"Color {n}: {COLORS[n][0]}"
        else:
            self.message = f"No color {n}, pick 1-{max(COLORS)}"

    def describe(self):
        if not self.on:
            return "Lights: off"
        text = f"Lights: {COLORS[self.color][0]}"
        if self.color_mode:
            text += f"  [color mode, typing: {self.digits}_]" if self.digits else "  [color mode]"
        return text


def describe(vx, omega):
    parts = []
    if vx:
        parts.append("forward" if vx > 0 else "back")
    if omega:
        parts.append("turning left" if omega > 0 else "turning right")
    return ", ".join(parts) or "stopped"


def main():
    pygame.init()
    screen = pygame.display.set_mode((560, 260))
    pygame.display.set_caption("Zeus Car")
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()
    car = Car(URL)
    lights = Lights()

    running = True
    while running:
        now = time.monotonic()
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
            elif event.type == pygame.KEYDOWN:
                lights.key(event, now)
        lights.update(now)

        keys = pygame.key.get_pressed()
        vx = SPEED * (keys[pygame.K_UP] - keys[pygame.K_DOWN])
        omega = TURN * (keys[pygame.K_LEFT] - keys[pygame.K_RIGHT])  # positive = counterclockwise = left
        car.send(vx, omega, lights.rgb)

        if car.answering:
            status, color = "Car OK", (80, 200, 120)
        else:
            status, color = car.status, (230, 160, 60)
        screen.fill((25, 25, 30))
        for i, (text, c) in enumerate([
            (status, color),
            (f"Driving: {describe(vx, omega)}", (230, 230, 230)),
            (lights.describe(), (230, 230, 230)),
            (lights.message, (150, 150, 150)),
            ("Arrows drive, Up Up Down Down = lights, Esc quits.", (150, 150, 150)),
        ]):
            screen.blit(font.render(text, True, c), (20, 25 + i * 45))
        pygame.display.flip()
        clock.tick(SEND_HZ)

    for _ in range(3):
        car.send(0, 0, lights.rgb)
        time.sleep(0.05)
    pygame.quit()


if __name__ == "__main__":
    main()
