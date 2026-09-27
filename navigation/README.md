# iPhone → Python → Zeus

The iPhone remains the interface: its AR map, pinned goal, and car tracking feed
Python on the laptop. Python sends motor commands to the existing car firmware
and returns a route and status to the phone. **Start and Stop are on the phone.**
No desktop app, SVG preview, or interactive terminal commands are needed.

## Run

1. Build/install the updated **CarVisionPhone** app. Power the car with the
   existing `firmware.ino`. Connect laptop and phone to the car's Wi-Fi.
2. Once, from this directory:

   ```sh
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements-autonomy.txt
   ```

3. Start the laptop receiver:

   ```sh
   python hub.py
   ```

   It discovers the laptop's interface toward `192.168.4.1`, advertises the same
   Bonjour service the phone already searches for, and connects to the car's
   WebSocket on port 8765. Leave the phone's manual hub IP blank for discovery.
   If discovery is unavailable, enter the laptop's Wi-Fi IPv4 address in the
   phone's existing settings and run `python hub.py --no-discovery`.
   `--host-ip LAPTOP_IP` selects an explicit discovery interface.

4. Use the phone's existing workflow: pin corners, **use correct measured arena
   dimensions**, scan the static obstacles, pin the goal, then mount the phone
   with the car marker visible. The existing mini-map shows obstacles and car.
5. Tap **Start**. Python captures the current map/goal, plans with clearance,
   and drives. The route appears on the same mini-map. Tap **Stop** at any time.

Close the Mac hub (it uses the same UDP port) and do not run `drive.py` at the
same time (competing motor commands). No car firmware changes are required.
Use `--car OTHER_IP` if the car address differs. Incoming UDP port 47800 must be
allowed by the laptop firewall, and the access point must permit phone↔laptop
traffic. Automatic date/time on both devices is needed for packet-age checks.

## First check without motors

```sh
python hub.py --dry-run
```

This receives real phone data and displays the planned route/status on the phone,
but never connects to the car. A stationary real car will not reach the goal in
dry-run. Restart without `--dry-run` when ready to test actual motion. The default
power is 0.18 (`--speed`, maximum 0.3); motor power is not a physical speed in cm/s.
Verify forward, sideways, and rotation directions and stopping distance in clear
space before running the course. Set the phone's car radius to enclose the entire
vehicle, including corners. The planner also applies the existing safety margin.

## Behavior

If Start is disabled, the message beside it now names the missing prerequisite:
Python reply, calibration/AR tracking, car marker ID 0, or pinned goal coordinates.
Discovery alone does not confirm a working two-way navigation connection. Packet
rejections (including clock mismatch) are returned to the phone even before the
first observation is accepted. Rebuild/install the phone app and restart Python
after updating to see these diagnostics.

- One phone controls the car. `--camera cam-711` can restrict it to an exact ID.
- Start uses the phone's **current** goal, arena dimensions, and 5 cm occupancy
  grid. Unknown cells, obstacles, and arena boundaries include car clearance.
  If there is no route, the phone explains why and the car stays stopped.
- The map stays fixed for a run. New obstacle evidence can block a route but
  cannot erase its original obstacles. To change the scene, Stop, use the
  existing Clear obstacles / rescan controls, and Start again.
- The planner uses A* and the controller follows clear segments while holding
  its starting heading. Live car observations correct the car's movement.
- A LiDAR (depth-tracked) car pose is used only if the marker was read within
  the last 1.5 s and the outline fit is at least 75% as good as usual; the car
  then drives at 60% power. Any untrusted pose (car not visible, weak fit, old
  marker, a jump of more than 12 cm) **holds** the run: zero motor commands, and
  driving resumes after 3 consistent readings. Holding for more than 1.5 s
  stops the run. The tuning constants are at the top of `hawkeye.py`.
- AR failure, phone veto, stale packets, car acknowledgment loss, changed
  goal/dimensions, or a blocked route stop the run at once. Recovery requires
  another tap on Start.
- Cells within the car radius of the car's position are never obstacles, so
  parts of the car its learned outline missed can't block its own route.
- The phone sends Stop when backgrounded and before resets/settings changes.
  Losing the phone's stream stops motion within the 0.4 s observation timeout;
  the existing firmware independently stops after 0.3 s without motor commands.
  These are command timeouts, not measured physical stopping times.
- Low objects missed by the phone's LiDAR height thresholds are still missed by
  Python. Scan the whole driving corridor; keep obstacles fixed and the phone
  mounted. Match the configured arena dimensions to the actual arena. The
  learned car outline can improve phone clearance warnings but is not required
  for planning with a conservative manually measured car radius.

The additional observation fields are `arena`, `carSource`, `sentAt` (Unix
seconds at frame processing), `sessionId`, and `navigation` (request ID,
generation, action, hub ID). Existing Mac receivers ignore these additions.
Python replies over the same UDP connection with `navStatus`: state, message,
and path. Ordered requests and a hub session handshake prevent stale Start
packets from resuming motion after Stop or a hub restart.
Motor output remains `{"A":vx,"B":vy,"C":omega,"D":seq}`, scaled to ±1000.

## Verification

```sh
python -m unittest discover -s tests -v
```

Tests exercise blocked/unknown space, car clearance, route following, goal
arrival, stop ordering, stale packets, and a real loopback UDP/WebSocket round
trip with a fake car. They never contact hardware. Hardware motion, camera
accuracy, and braking behavior still need testing with your team's car.
