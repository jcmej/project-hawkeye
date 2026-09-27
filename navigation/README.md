# Hawkeye Python hub

The laptop is the central hub. `drive.py` stays the keyboard fallback;
`autonomous.py` owns autonomous driving. Do not run both against the car.
The Mac Swift hub must also be closed: it uses the same UDP observation port.

Pipeline: iPhone camera/AR + on-device perception → JSON/UDP 47800 → Python
static map + planner + feedback controller → JSON/WebSocket 8765 → existing
ESP32 + R3 firmware. No video is sent to Python. No new car firmware is needed.

## Setup

Run these commands from the `navigation` directory:

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-autonomy.txt
python autonomous.py --camera cam-123
```

Replace `cam-123` with the exact camera ID displayed by the phone. Dry-run is
the default: it prints motor payloads but never connects to a car. It does not
simulate movement; use the simulator below for movement without hardware.
Python 3.10+ is required. Pygame is not needed for autonomy.

Build and install the updated iPhone app first. Its new optional metadata is
required by Python (older app builds will be rejected). Existing Mac receivers
can continue decoding the phone messages because unknown JSON fields are ignored.
The phone project still requires the team's existing OpenCV framework setup.

Connect the laptop and phone to the car's Wi-Fi. In the phone's manual hub IP
setting, enter the **laptop's Wi-Fi IPv4 address**, not 192.168.4.1 (the car).
This Python hub does not advertise Bonjour. Allow incoming UDP 47800 through
the laptop firewall. Confirm that the car access point permits phone-to-laptop
traffic. Use automatic date/time on both devices; message-age checks require
clocks within approximately 100 ms (future) / 400 ms (past).

Match phone and Python arena settings. Defaults: width 200 cm, height 150 cm,
car radius 13 cm, obstacle radius 12 cm, safety margin 5 cm. These values are
configuration, not measurements: set the car radius to enclose the entire car
(including corners). CLI overrides: `--width`, `--height`, `--car-radius`,
`--obstacle-radius`, `--margin`. Coordinates are cm; origin is marker 2,
+x toward marker 3, +y toward marker 5; heading is radians, positive CCW.

## Static course procedure

1. Remove the car. Place tall, easily observed static obstacles. Pin the arena
   corners and clear the phone's previous obstacle map and goal. For a LiDAR
   phone, enable LiDAR and scan the full intended driving corridor. Low objects
   below the existing depth height thresholds may not be represented.
   With camera background subtraction, capture the **empty floor first**, then
   place obstacles, and keep the phone fixed.
2. Enter `scan` in Python. It unions observed occupied cells; unobserved cells
   remain blocked. A false positive persists until `reset` and a fresh scan.
   Do not move obstacles while scanning. There is no dynamic replanning map.
3. Enter `freeze`. Open `hawkeye-map.svg`: white = observed free, gray = unknown,
   red = obstacles. Inspect coverage and obstacle boundaries. The static map
   stays in laptop memory; restarting the controller requires another scan.
4. Insert the car, with its forward-oriented ArUco marker visible. Mount the
   phone and keep it fixed. Set a goal in Python, e.g. `goal 170 120`. Enter
   `plan`, then refresh the SVG to inspect the blue route. Purple shows the car
   radius plus margin, green the goal. No path means no motion.
5. Enter `arm` in dry-run to inspect command signs. With a real stationary car,
   dry-run will not reach the goal by itself. Enter `stop` when done.
6. For the actual run, restart with `--live --car 192.168.4.1`, repeat scan/freeze,
   goal/plan, and enter `arm`. Start with clear space and verify forward, left
   strafe, and CCW rotation at low power before the obstacle course.

`stop` disarms immediately on the next loop; Ctrl-C or `quit` sends zero and
closes the link. Any marker loss, phone veto, stale observation, invalid AR
mapping, or missing car acknowledgment disarms; recovery **never auto-resumes**.
Use `arm` only after inspecting the scene. `reset` stops and clears the map.
`status` prints status and updates the SVG (the preview is not a live dashboard).

The 20 Hz commands are normalized motor inputs, **not cm/s**. Default translation
power is 0.18; tune with `--speed` (capped at 0.3). Heading correction is capped
at 0.2. The firmware's minimum PWM means even low commands can move noticeably.
The controller follows checked straight segments while holding the initial
heading; it stops if the actual car leaves the safe route. It does not model
braking distance or motor lag. Choose clearance/speed from real measurements.
A car ACK confirms receipt, not actual wheel motion. Firmware's existing 300 ms
watchdog remains the independent command-loss stop.

## Synthetic end-to-end run

Start the simulator in terminal 1 (same virtual environment):

```sh
python simulate.py
```

Immediately start terminal 2:

```sh
python autonomous.py --camera sim --live --car 127.0.0.1
```

Here `--live` talks only to the **local fake car**. During the first 20 seconds,
enter `scan`, then `freeze`. After 20 seconds the synthetic marker appears at
(30,30). Enter `goal 170 120`, `plan`, then `arm`. The simulated car follows the
route around an obstacle, using actual UDP observations and WebSocket motor
commands/ACKs. It should reach the goal in roughly a minute at default power.
Stop the simulator mid-run to verify loss handling. Restart both programs to
repeat the scan phase. The simulator is a simple kinematic test, not proof of
real-car accuracy; it does not synthesize images or validate iPhone perception.

## Observation contract

One UTF-8 JSON datagram per observation. Existing app fields are preserved:

```json
{
  "type": "obs", "cameraId": "cam-123", "sessionId": "session-uuid", "seq": 42,
  "sentAt": 1790470000.0, "calibrated": true,
  "car": {"position": {"x": 30, "y": 30}, "heading": 0},
  "carSource": "marker", "carConfidence": 0.9,
  "goal": null, "obstacles": [], "veto": false, "fps": 30,
  "arena": {"width": 200, "height": 150, "carRadius": 13,
            "obstacleRadius": 12, "safetyMargin": 5}
}
```

`sentAt` is Unix seconds at processing time; replace the example with current
time. It catches network/sender queue delays, not pre-processing capture lag.
`sessionId` is fresh per phone pipeline; decreasing/duplicate sequence numbers
within a session are ignored. `carSource` must be `marker` to drive. Depth-only
tracking is intentionally excluded from this first controller.

Scanning also requires `grid: {cols, rows, visible, occupied}`. Masks are base64,
least-significant-bit first; index = row * cols + col; 5 cm cells, row 0 at y=0.
LiDAR `visible` means historically observed, not necessarily visible this instant.
Do not invent fully observed space to make a route pass on real hardware.

Output: `{"A":180,"B":0,"C":0,"D":43}` over WebSocket, values clamped/scaled
from [-1,1] to [-1000,1000]. A = forward, B = left, C = CCW, D = sequence.
Phone `clearance` is not used as an independent motor command; its phone-side
collision veto remains active.

## Verification

```sh
python -m unittest discover -s tests -v
```

Covers blocked routes, unknown space, footprint clearance, boundary rejection,
sequence/age checks, latched stops, body-frame conversion, and closed-loop goal
arrival. The local WebSocket integration test also checks camera dropout,
acknowledgment loss, and explicit rearming. Real-device frame rate, calibration
accuracy, Wi-Fi peer routing, strafe direction, and stopping distance still
require device validation. The tests never connect to hardware; the integration
test sends commands only to a loopback fake server.
