"""Geometry and phone-driven navigation. Coordinates: cm, headings: radians."""
import base64
import heapq
import math
import time
import uuid

AGE = .4
CELL = 5
# Tracking gaps. A LiDAR (depth-tracked) car pose counts only if the phone read
# the marker within MARKER_GAP and the outline fit is at least MIN_TRACK_QUALITY
# of the car's usual fit; it then drives at LIDAR_SPEED of normal power. Without
# a trusted pose the run holds (zero motor commands) for up to HOLD, resumes
# after RESUME_FRAMES consistent readings, and otherwise stops.
MARKER_GAP = 1.5
MIN_TRACK_QUALITY = .75
LIDAR_SPEED = .6
HOLD = 1.5
RESUME_FRAMES = 3
RESUME_JUMP = 5    # cm between consecutive readings while reacquiring
MAX_JUMP = 12      # cm between consecutive readings while driving
# Steering. The car faces the next waypoint, turning in place while it is more than
# TURN_IN_PLACE off, then drives mostly forward. Within FACE_MIN_DIST of the waypoint
# its bearing is too noisy to steer by, so the car keeps its heading and slides in.
TURN_IN_PLACE = .3  # rad
FACE_MIN_DIST = 8   # cm
# The camera pose is a few hundred ms old, so continuous correction overshoots. Turning
# in place and the final approach to the goal therefore move in short pulses: each
# covers PULSE_FRACTION of the remaining error at the car's measured rate, then the car
# stops and waits for a camera frame captured SETTLE s after it stopped before the next.
# Turning continues until the car is within ALIGNED of the waypoint's bearing, or a
# pulse carries it past the bearing to within TURN_IN_PLACE (the car's smallest step is
# too big to settle closer, and chasing it would swing back and forth forever).
ALIGNED = .1        # rad
APPROACH = 15       # cm from the goal where driving switches to pulses
PULSE_FRACTION = .7
MIN_PULSE, MAX_PULSE = .05, .4   # s (the hub sends commands every .05 s)
SETTLE = .1         # s
TURN_POWER = .2     # motor power for turning in place
# Heading correction while driving is continuous, so it is sized to settle over about
# four camera lags given how fast the car turns (at most DRIVE_TURN_MAX power).
DRIVE_TURN_MAX = .1
# Starting guesses for how fast the car moves (at TURN_POWER and approach power); each
# pulse's measured result refines them, within these bounds.
TURN_RATE, TURN_RATE_RANGE = 2., (.2, 20.)    # rad/s
DRIVE_RATE, DRIVE_RATE_RANGE = 10., (.5, 100.)  # cm/s


def num(x):
    if type(x) not in (int, float) or not math.isfinite(x):
        raise ValueError('Invalid numeric observation')
    return float(x)


def point(p):
    return num(p['x']), num(p['y'])


def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


class Map:
    def __init__(self, msg):
        a = msg['arena']
        self.settings = tuple(num(a[k]) for k in ('width', 'height', 'carRadius', 'safetyMargin', 'obstacleRadius'))
        self.w, self.h, radius, margin, obstacle_radius = self.settings
        if not (20 < self.w <= 500 and 20 < self.h <= 500 and 0 < radius <= 60 and 0 < margin <= 30 and 0 < obstacle_radius <= 60):
            raise ValueError('Invalid arena dimensions or clearance')
        self.car_radius = radius
        self.radius = radius + margin
        self.cols, self.rows = math.ceil(self.w / CELL), math.ceil(self.h / CELL)
        grid = msg['grid']
        if grid['cols'] != self.cols or grid['rows'] != self.rows:
            raise ValueError('Grid does not match arena dimensions')
        n = self.cols * self.rows
        def unpack(key):
            raw = base64.b64decode(grid[key], validate=True)
            if len(raw) != (n+7)//8:
                raise ValueError('Invalid occupancy grid')
            return {i for i in range(n) if raw[i//8] & (1 << (i%8))}
        self.known, self.occupied = unpack('visible'), unpack('occupied')
        self.obstacles = {str(o['id']): point(o['position']) for o in msg['obstacles']}
        self.obstacle_clearance = self.radius + obstacle_radius

    def center(self, i):
        return ((i % self.cols + .5)*CELL, (i // self.cols + .5)*CELL)

    def mask_car(self, p):
        """The car is not an obstacle to itself. The phone masks it using its learned
        outline, which can miss parts of the car (or lag it), so also clear every
        cell under the configured car radius as seen, free floor."""
        reach = self.car_radius + CELL/2
        for y in range(max(0, int((p[1]-reach)//CELL)), min(self.rows, int((p[1]+reach)//CELL)+1)):
            for x in range(max(0, int((p[0]-reach)//CELL)), min(self.cols, int((p[0]+reach)//CELL)+1)):
                i = y*self.cols+x
                if dist(self.center(i), p) <= reach:
                    self.occupied.discard(i)
                    self.known.add(i)

    def index(self, p):
        return int(p[1]//CELL)*self.cols + int(p[0]//CELL)

    def free(self, p):
        r = self.radius
        if not (r <= p[0] <= self.w-r and r <= p[1] <= self.h-r):
            return False
        if any(dist(p, q) <= self.obstacle_clearance for q in self.obstacles.values()):
            return False
        for y in range(max(0, int((p[1]-r)//CELL)), min(self.rows, int((p[1]+r)//CELL)+1)):
            for x in range(max(0, int((p[0]-r)//CELL)), min(self.cols, int((p[0]+r)//CELL)+1)):
                i = y*self.cols+x
                if i in self.occupied or i not in self.known:
                    dx = max(x*CELL-p[0], 0, p[0]-(x+1)*CELL)
                    dy = max(y*CELL-p[1], 0, p[1]-(y+1)*CELL)
                    if math.hypot(dx, dy) <= r:
                        return False
        return True

    def clear(self, a, b):
        n = max(1, math.ceil(dist(a, b)))
        return all(self.free((a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n)) for i in range(n+1))

    def plan(self, start, goal):
        if not self.free(start) or not self.free(goal):
            return []
        if self.clear(start, goal):
            return [start, goal]
        first, end = self.index(start), self.index(goal)
        def pos(i):
            return start if i == first else goal if i == end else self.center(i)
        heap, cost, prev, visited = [(dist(start, goal), first)], {first: 0}, {}, set()
        while heap:
            _, cur = heapq.heappop(heap)
            if cur in visited:
                continue
            visited.add(cur)
            if cur == end:
                path = [goal]
                while cur != first:
                    cur = prev[cur]
                    path.append(pos(cur))
                path.reverse()
                result, i = [start], 0
                while i < len(path)-1:
                    j = len(path)-1
                    while j > i+1 and not self.clear(path[i], path[j]):
                        j -= 1
                    result.append(path[j])
                    i = j
                return result
            x, y = cur % self.cols, cur // self.cols
            for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nx, ny = x+dx, y+dy
                if not (0 <= nx < self.cols and 0 <= ny < self.rows):
                    continue
                nxt = ny*self.cols+nx
                g = cost[cur]+dist(pos(cur), pos(nxt))
                if g >= cost.get(nxt, math.inf) or not self.clear(pos(cur), pos(nxt)):
                    continue
                cost[nxt], prev[nxt] = g, cur
                heapq.heappush(heap, (g+dist(pos(nxt), goal), nxt))
        return []


class Navigator:
    def __init__(self, speed=.18, dry_run=False):
        self.hub_id = str(uuid.uuid4())
        self.speed, self.dry_run = speed, dry_run
        self.session = None
        self.seq = -1
        self.received = -math.inf
        self.last_request = None
        self.generation = -1
        self.retired_sessions = set()
        self.msg = None
        self.map = None
        self.goal = None
        self.path = []
        self.heading = 0
        self.pose = None         # last trusted (position, heading, source)
        self.hold_since = None   # set while holding for a trusted pose
        self.good = 0            # consistent readings while reacquiring
        self.captured = -math.inf  # monotonic time the latest frame was captured
        self.lag = .2            # smoothed camera frame age on arrival, s
        self.turn_rate, self.drive_rate = TURN_RATE, DRIVE_RATE
        self.turning = False
        self.pulse = None        # (kind, command, start, end, start pose) of the current pulse
        self.pulse_stopped = None  # when the last pulse's motors actually stopped
        self.running = False
        self.state, self.message = 'waiting', 'Waiting for iPhone'

    def stop(self, message='Stopped', state='stopped'):
        self.running = False
        self.state, self.message = state, message

    def ingest(self, m, now=None, wall=None, car_ready=True):
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        try:
            if m.get('type') == 'stop':
                if m.get('sessionId') == self.session and m.get('hubId') == self.hub_id:
                    if type(m.get('generation')) is not int or m['generation'] < self.generation:
                        return
                    self.stop()
                    self.generation = m['generation']
                    self.last_request = m.get('requestId')
                return
            if m['type'] != 'obs':
                return
            session, seq = m['sessionId'], m['seq']
            if not isinstance(session, str) or not session or type(seq) is not int or seq < 0:
                raise ValueError('Invalid camera session')
            if session in self.retired_sessions:
                return
            if session == self.session and seq <= self.seq:
                return
            delay = wall-num(m['sentAt'])
            if not -.1 <= delay <= AGE:
                raise ValueError(f'Camera delay {delay*1000:.0f} ms (limit {AGE*1000:.0f}) or phone/laptop clock mismatch')
            request = m['navigation']
            if (not isinstance(request['id'], str) or request['action'] not in ('start', 'stop')
                    or type(request['generation']) is not int or request['generation'] < 0):
                raise ValueError('Invalid navigation request')
            if session != self.session:
                if self.session is not None:
                    self.retired_sessions.add(self.session)
                self.session, self.seq = session, -1
                self.last_request = request['id']  # Never arm just because a phone reconnected.
                self.generation = request['generation']
                self.stop('Connected — tap Start', 'ready')
            self.msg, self.seq, self.received = m, seq, now
            self.captured = now-(wall-num(m['sentAt']))
            self.lag = .8*self.lag + .2*max(0, now-self.captured)
            if request['generation'] < self.generation:
                return
            new_request = request['generation'] > self.generation and request['id'] != self.last_request
            self.generation = request['generation']
            self.last_request = request['id']
            if request['action'] == 'stop':
                if new_request or self.running:
                    self.stop()
                elif self.state == 'blocked':
                    self.stop('Connected — tap Start', 'ready')
                return
            if request.get('hubId') != self.hub_id:
                self.stop('Connected — tap Start', 'ready')
                return
            error = self.problem(now, car_ready)
            if error:
                self.stop(error)
                return
            pose, why = self.tracked_car(m)
            live_map = Map(m)
            goal = point(m['goal'])
            if new_request:
                if pose is None:
                    self.stop(why)
                    return
                self.stop('Planning')
                live_map.mask_car(pose[0])
                self.map, self.goal = live_map, goal
                self.path = self.map.plan(pose[0], goal)
                if not self.path:
                    self.stop('No safe route — check map coverage and clearance')
                    return
                self.heading = pose[1]
                self.pose, self.hold_since, self.good = pose, None, 0
                self.turning, self.pulse, self.pulse_stopped = False, None, None
                self.running = True
                self.state, self.message = 'driving', self.driving_message()
            elif self.running:
                if live_map.settings != self.map.settings or dist(goal, self.goal) > 2:
                    self.stop('Goal or arena changed — tap Start again')
                    return
                self.track(pose, why, now)
                # Keep the initial map. New evidence may block it, never silently clear it,
                # except where the car itself is (last trusted position while holding).
                live_map.mask_car(self.pose[0])
                self.map.occupied |= live_map.occupied
                self.map.mask_car(self.pose[0])   # also forget car cells remembered from lagging poses
                self.map.obstacles.update(live_map.obstacles)
        except (KeyError, TypeError, ValueError, AttributeError, OverflowError) as e:
            self.stop(str(e) or 'Invalid camera observation', 'blocked')

    def problem(self, now, car_ready=True):
        if now-self.received > AGE:
            return 'Camera lost — tap Start after recovery'
        if not car_ready:
            return 'Car not responding — tap Start after recovery'
        m = self.msg
        if m.get('calibrated') is not True:
            return 'AR tracking unavailable'
        if m.get('veto') is not False:
            return 'Obstacle warning — stopped'
        if not m.get('goal'):
            return 'Pin a goal first'
        if not m.get('grid'):
            return 'Scan the obstacle map first'
        return None

    def tracked_car(self, m):
        """The car's (position, heading, source) from one observation, or None and the
        reason it can't be trusted. Not trusting a pose holds a run; it doesn't end it."""
        car, source = m.get('car'), m.get('carSource')
        if not car:
            return None, 'Car not visible'
        p, heading = point(car['position']), num(car['heading'])
        if not 0 < num(m['carConfidence']) <= 1:
            return None, 'Car observation uncertain'
        if source == 'LiDAR':
            age, quality = m.get('markerAge'), m.get('trackQuality')
            if age is None or quality is None:
                return None, 'Car marker not visible'   # phone app too old to rate LiDAR poses
            if not 0 <= num(age) <= MARKER_GAP:
                return None, 'Car marker not seen recently'
            if not MIN_TRACK_QUALITY <= num(quality) <= 1:
                return None, 'LiDAR car fit too weak'
        elif source != 'marker':
            return None, 'Car marker not visible'
        return (p, heading, source), None

    def track(self, pose, why, now):
        """Update the trusted pose during a run, entering or leaving the holding state."""
        if pose is not None and self.hold_since is None and dist(pose[0], self.pose[0]) > MAX_JUMP:
            pose, why = None, 'Car position jumped'
        if pose is None:
            self.good = 0
            if self.hold_since is None:
                self.hold_since = now
            self.state, self.message = 'holding', f'{why} — holding'
            return
        if self.hold_since is None:
            self.pose = pose
            return
        consistent = self.good > 0 and dist(pose[0], self.pose[0]) <= RESUME_JUMP
        self.good = self.good + 1 if consistent else 1
        self.pose = pose
        if self.good >= RESUME_FRAMES:
            self.hold_since, self.good = None, 0
            self.state, self.message = 'driving', self.driving_message()
        else:
            self.message = 'Car found — confirming position'

    def driving_message(self):
        return 'Dry run — motors disabled' if self.dry_run else 'Driving'

    def command(self, now=None, car_ready=True):
        if not self.running:
            return (0, 0, 0)
        now = time.monotonic() if now is None else now
        try:
            error = self.problem(now, car_ready)
            if error:
                self.stop(error)
                return (0, 0, 0)
            if self.hold_since is not None:
                if now - self.hold_since > HOLD:
                    self.stop('Car lost — tap Start after recovery')
                return (0, 0, 0)
            p, heading, source = self.pose
            if not self.map.free(p):
                self.stop('Obstacle too close or car outside mapped space')
                return (0, 0, 0)
            if dist(p, self.goal) < 5:
                self.stop('Goal reached', 'reached')
                return (0, 0, 0)
            while len(self.path) > 2 and self.map.clear(p, self.path[2]):
                self.path.pop(1)
            target = self.path[1]
            if not self.map.clear(p, target):
                self.stop('Route blocked — check scene and tap Start')
                return (0, 0, 0)
            d = dist(p, target)
            if d < .1:
                self.stop('Route needs replanning — tap Start')
                return (0, 0, 0)
            if self.pulse:
                kind, cmd, start, end, (p0, h0) = self.pulse
                if now < end:
                    return cmd
                if self.pulse_stopped is None:
                    self.pulse_stopped = now   # commands arrive every tick, so this can pass `end`
                if self.captured < self.pulse_stopped+SETTLE:
                    return (0, 0, 0)   # wait for a frame showing where the pulse left the car
                moved = dist(p, p0) if kind == 'drive' else abs(wrap(heading-h0))
                self.learn_rate(kind, moved, self.pulse_stopped-start)
                self.pulse, self.pulse_stopped = None, None
                if kind == 'turn' and self.turning and cmd[2]*wrap(self.heading-heading) < 0:
                    # Overshot: a smaller step is not possible, so stop turning if close enough.
                    self.turning = abs(wrap(self.heading-heading)) > TURN_IN_PLACE
            if d > FACE_MIN_DIST:
                self.heading = math.atan2(target[1]-p[1], target[0]-p[0])
            angle = wrap(self.heading-heading)
            if abs(angle) <= ALIGNED or abs(angle) > TURN_IN_PLACE:
                self.turning = abs(angle) > TURN_IN_PLACE
            if self.turning:
                return self.start_pulse('turn', (0, 0, math.copysign(TURN_POWER, angle)), abs(angle), now)
            to_goal = dist(p, self.goal)
            speed = self.speed*min(1, max(.4, to_goal/30))
            if source == 'LiDAR':
                speed *= LIDAR_SPEED
            x, y = (target[0]-p[0])*speed/d, (target[1]-p[1])*speed/d
            body = x*math.cos(heading)+y*math.sin(heading), -x*math.sin(heading)+y*math.cos(heading)
            if to_goal < APPROACH:
                return self.start_pulse('drive', (*body, 0), to_goal, now)
            gain = TURN_POWER/self.turn_rate/(4*max(.1, self.lag))   # power per rad of error
            return (*body, max(-DRIVE_TURN_MAX, min(DRIVE_TURN_MAX, angle*gain)))
        except (KeyError, TypeError, ValueError, IndexError) as e:
            self.stop(f'Invalid observation: {e}')
            return (0, 0, 0)

    def start_pulse(self, kind, cmd, error, now):
        """Move for long enough to cover PULSE_FRACTION of `error` at the car's measured rate."""
        rate = self.turn_rate if kind == 'turn' else self.drive_rate
        length = max(MIN_PULSE, min(MAX_PULSE, PULSE_FRACTION*error/rate))
        self.pulse = (kind, cmd, now, now+length, self.pose[:2])
        return cmd

    def learn_rate(self, kind, moved, length):
        """Blend a pulse's measured speed into the car's estimated rate."""
        if length <= 0 or moved < (.02 if kind == 'turn' else .5):
            return   # too little movement to measure over the pose noise
        lo, hi = TURN_RATE_RANGE if kind == 'turn' else DRIVE_RATE_RANGE
        if kind == 'turn':
            self.turn_rate = max(lo, min(hi, .5*self.turn_rate + .5*moved/length))
        else:
            self.drive_rate = max(lo, min(hi, .5*self.drive_rate + .5*moved/length))

    def status(self):
        return {'type': 'navStatus', 'hubId': self.hub_id, 'sessionId': self.session,
                'seq': self.seq, 'requestId': self.last_request, 'state': self.state,
                'message': self.message, 'path': [{'x': x, 'y': y} for x, y in self.path]}


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def motor_payload(command, seq):
    return dict(zip(('A', 'B', 'C', 'D'), [round(max(-1, min(1, num(v)))*1000) for v in command]+[seq]))
