"""Geometry and phone-driven navigation. Coordinates: cm, headings: radians."""
import base64
import heapq
import math
import time
import uuid

AGE = .4
CELL = 5


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
            if not -.1 <= wall-num(m['sentAt']) <= AGE:
                raise ValueError('Camera delay or phone/laptop clock mismatch')
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
            live_map = Map(m)
            goal = point(m['goal'])
            if new_request:
                self.stop('Planning')
                self.map, self.goal = live_map, goal
                self.path = self.map.plan(point(m['car']['position']), goal)
                if not self.path:
                    self.stop('No safe route — check map coverage and clearance')
                    return
                self.heading = num(m['car']['heading'])
                self.running = True
                self.state, self.message = 'driving', 'Dry run — motors disabled' if self.dry_run else 'Driving'
            elif self.running:
                if live_map.settings != self.map.settings or dist(goal, self.goal) > 2:
                    self.stop('Goal or arena changed — tap Start again')
                    return
                # Keep the initial map. New evidence may block it, never silently clear it.
                self.map.occupied |= live_map.occupied
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
        if not m.get('car') or m.get('carSource') != 'marker':
            return 'Car marker not visible'
        point(m['car']['position'])
        num(m['car']['heading'])
        if not 0 < num(m['carConfidence']) <= 1:
            return 'Car observation uncertain'
        if not m.get('goal'):
            return 'Pin a goal first'
        if not m.get('grid'):
            return 'Scan the obstacle map first'
        return None

    def command(self, now=None, car_ready=True):
        if not self.running:
            return (0, 0, 0)
        try:
            error = self.problem(time.monotonic() if now is None else now, car_ready)
            if error:
                self.stop(error)
                return (0, 0, 0)
            p, heading = point(self.msg['car']['position']), num(self.msg['car']['heading'])
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
            speed = self.speed*min(1, max(.4, dist(p, self.goal)/30))
            x, y = (target[0]-p[0])*speed/d, (target[1]-p[1])*speed/d
            angle = (self.heading-heading+math.pi) % (2*math.pi)-math.pi
            omega = max(-.2, min(.2, angle*1.2))
            if abs(angle) > .4:
                x = y = 0
            return x*math.cos(heading)+y*math.sin(heading), -x*math.sin(heading)+y*math.cos(heading), omega
        except (KeyError, TypeError, ValueError, IndexError) as e:
            self.stop(f'Invalid observation: {e}')
            return (0, 0, 0)

    def status(self):
        return {'type': 'navStatus', 'hubId': self.hub_id, 'sessionId': self.session,
                'seq': self.seq, 'requestId': self.last_request, 'state': self.state,
                'message': self.message, 'path': [{'x': x, 'y': y} for x, y in self.path]}


def motor_payload(command, seq):
    return dict(zip(('A', 'B', 'C', 'D'), [round(max(-1, min(1, num(v)))*1000) for v in command]+[seq]))
