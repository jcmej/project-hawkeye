"""Dependency-free geometry, protocol validation, and static-scene control (cm/radians)."""
import base64
import heapq
import math
import time
from dataclasses import dataclass, field

CELL = 5.0
MAX_AGE = 0.4


def number(v):
    if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v):
        raise ValueError('Expected a finite number')
    return float(v)


def point(v):
    return number(v['x']), number(v['y'])


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Scene:
    width: float = 200
    height: float = 150
    radius: float = 13
    margin: float = 5
    obstacle_radius: float = 12
    # A scanned map must distinguish unknown cells from observed free space.
    known: set = field(default_factory=set)
    occupied: set = field(default_factory=set)
    obstacles: dict = field(default_factory=dict)
    has_grid: bool = False
    frozen: bool = False

    @property
    def cols(self):
        return math.ceil(self.width / CELL)

    @property
    def rows(self):
        return math.ceil(self.height / CELL)

    def center(self, i):
        return ((i % self.cols + .5) * CELL, (i // self.cols + .5) * CELL)

    def index(self, p):
        return int(p[1] // CELL) * self.cols + int(p[0] // CELL)

    def validate(self, msg):
        a = msg['arena']
        for key, expected in [('width', self.width), ('height', self.height),
                              ('carRadius', self.radius), ('safetyMargin', self.margin),
                              ('obstacleRadius', self.obstacle_radius)]:
            if abs(number(a[key]) - expected) > .01:
                raise ValueError(f'Arena mismatch: {key}; match phone and Python settings')

    def capture(self, msg):
        """Union evidence while scanning an empty-of-car, fixed obstacle course."""
        if self.frozen:
            raise ValueError('Map already frozen')
        if msg.get('car') is not None:
            raise ValueError('Remove the car before capturing the static map')
        grid = msg.get('grid')
        if not grid:
            raise ValueError('No occupancy grid: enable LiDAR or capture a camera background')
        if grid['cols'] != self.cols or grid['rows'] != self.rows:
            raise ValueError('Grid dimensions differ')
        n = self.cols * self.rows
        masks = [base64.b64decode(grid[k], validate=True) for k in ('visible', 'occupied')]
        if any(len(m) != (n + 7) // 8 for m in masks):
            raise ValueError('Invalid grid bitmask length')
        known = {i for i in range(n) if masks[0][i // 8] & (1 << (i % 8))}
        occupied = {i for i in known if masks[1][i // 8] & (1 << (i % 8))}
        obstacles = {str(o['id']): point(o['position']) for o in msg['obstacles']}
        self.known |= known
        self.occupied |= occupied
        self.obstacles.update(obstacles)
        self.has_grid = True

    def free(self, p):
        r = self.radius + self.margin
        if not (r <= p[0] <= self.width - r and r <= p[1] <= self.height - r):
            return False
        if any(distance(p, q) <= r + self.obstacle_radius for q in self.obstacles.values()):
            return False
        # Distance to each nearby blocked square (including unknown cells).
        for y in range(max(0, int((p[1] - r) // CELL)), min(self.rows, int((p[1] + r) // CELL) + 1)):
            for x in range(max(0, int((p[0] - r) // CELL)), min(self.cols, int((p[0] + r) // CELL) + 1)):
                i = y * self.cols + x
                if i not in self.known or i in self.occupied:
                    dx = max(x * CELL - p[0], 0, p[0] - (x + 1) * CELL)
                    dy = max(y * CELL - p[1], 0, p[1] - (y + 1) * CELL)
                    if math.hypot(dx, dy) <= r:
                        return False
        return True

    def segment_free(self, a, b):
        steps = max(1, math.ceil(distance(a, b)))
        return all(self.free((a[0] + (b[0] - a[0]) * i / steps,
                              a[1] + (b[1] - a[1]) * i / steps)) for i in range(steps + 1))

    def plan(self, start, goal):
        if not self.frozen or not self.free(start) or not self.free(goal):
            return []
        if self.segment_free(start, goal):
            return [start, goal]
        first, end = self.index(start), self.index(goal)
        def pos(i):
            return start if i == first else goal if i == end else self.center(i)
        heap, costs, prev = [(distance(start, goal), first)], {first: 0}, {}
        free_cells = {}
        while heap:
            _, cur = heapq.heappop(heap)
            if cur == end:
                path = [goal]
                while cur != first:
                    cur = prev[cur]
                    path.append(pos(cur))
                path.reverse()
                # Only shortcut a segment after checking the whole car footprint.
                out, k = [start], 0
                while k < len(path) - 1:
                    j = len(path) - 1
                    while j > k + 1 and not self.segment_free(path[k], path[j]):
                        j -= 1
                    out.append(path[j])
                    k = j
                return out
            x, y = cur % self.cols, cur // self.cols
            # Cardinal moves avoid diagonal corner cutting.
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < self.cols and 0 <= ny < self.rows):
                    continue
                nxt = ny * self.cols + nx
                if nxt not in free_cells:
                    free_cells[nxt] = self.free(pos(nxt))
                if not free_cells[nxt] or not self.segment_free(pos(cur), pos(nxt)):
                    continue
                g = costs[cur] + distance(pos(cur), pos(nxt))
                if g < costs.get(nxt, math.inf):
                    costs[nxt], prev[nxt] = g, cur
                    heapq.heappush(heap, (g + distance(pos(nxt), goal), nxt))
        return []


class Controller:
    def __init__(self, scene, camera, speed=.18):
        self.scene, self.camera, self.speed = scene, camera, speed
        self.msg = None
        self.received = -math.inf
        self.session = None
        self.seq = -1
        self.armed = False
        self.goal = None
        self.heading = 0
        self.path = []
        self.reason = 'Waiting for phone'

    def stop(self, reason='Stopped'):
        self.armed = False
        self.reason = reason

    def ingest(self, msg, now=None, wall=None):
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        if not isinstance(msg, dict) or msg.get('cameraId') != self.camera:
            return False
        try:
            if msg.get('type') != 'obs':
                raise ValueError('Not an observation')
            self.scene.validate(msg)
            age = wall - number(msg['sentAt'])
            if not (-.1 <= age <= MAX_AGE):
                raise ValueError('Delayed observation or phone/laptop clocks differ')
            session, seq = msg['sessionId'], msg['seq']
            if not isinstance(session, str) or not session or type(seq) is not int or seq < 0:
                raise ValueError('Invalid session/sequence')
            if session == self.session and seq <= self.seq:
                return False
            if type(msg['calibrated']) is not bool or type(msg['veto']) is not bool:
                raise ValueError('Invalid status flags')
            if msg.get('car') is not None:
                point(msg['car']['position'])
                number(msg['car']['heading'])
                if not 0 < number(msg['carConfidence']) <= 1:
                    raise ValueError('Invalid car confidence')
            if self.session is not None and session != self.session:
                self.stop('Phone restarted; verify calibration and re-arm')
            self.session, self.seq = session, seq
            self.msg, self.received = msg, now
            if not self.healthy(now):
                self.stop(self.problem(now))
            return True
        except (KeyError, TypeError, ValueError, OverflowError) as e:
            self.stop(f'Invalid observation: {e}')
            self.msg = None
            return False

    def problem(self, now):
        m = self.msg
        if m is None or now - self.received > MAX_AGE:
            return 'Phone observations stale'
        if not m['calibrated']:
            return 'Camera calibration/tracking unavailable'
        if m['veto']:
            return 'Phone veto: ' + str(m.get('vetoReason', 'obstacle'))
        if not m.get('car') or m.get('carSource') != 'marker':
            return 'Car marker not visible'
        return ''

    def healthy(self, now):
        return not self.problem(now)

    def arm(self, now):
        if not self.healthy(now):
            raise ValueError(self.problem(now))
        if self.goal is None or not self.scene.frozen:
            raise ValueError('Freeze a map and set a goal first')
        p = point(self.msg['car']['position'])
        self.path = self.scene.plan(p, self.goal)
        if not self.path:
            raise ValueError('No collision-free path')
        self.heading = self.msg['car']['heading']
        self.armed = True
        self.reason = 'Driving'

    def command(self, now):
        if not self.armed:
            return (0, 0, 0)
        if not self.healthy(now):
            self.stop(self.problem(now))
            return (0, 0, 0)
        pose = self.msg['car']
        p, heading = point(pose['position']), pose['heading']
        if distance(p, self.goal) < 5:
            self.stop('Goal reached')
            return (0, 0, 0)
        # Advance only when the next segment is clear from the actual current pose.
        while len(self.path) > 2 and self.scene.segment_free(p, self.path[2]):
            self.path.pop(1)
        if len(self.path) < 2 or not self.scene.segment_free(p, self.path[1]):
            self.stop('Car left the safe route; stop and re-arm to replan')
            return (0, 0, 0)
        target = self.path[1]
        d = distance(p, target)
        if d < .1:
            self.stop('Waypoint stalled; re-arm')
            return (0, 0, 0)
        speed = self.speed * min(1, max(.4, distance(p, self.goal) / 30))
        x, y = (target[0] - p[0]) / d * speed, (target[1] - p[1]) / d * speed
        omega = max(-.2, min(.2, 1.2 * wrap(self.heading - heading)))
        return x * math.cos(heading) + y * math.sin(heading), -x * math.sin(heading) + y * math.cos(heading), omega


def payload(command, seq):
    return dict(zip(('A', 'B', 'C', 'D'), [round(max(-1, min(1, number(v))) * 1000) for v in command] + [seq]))
