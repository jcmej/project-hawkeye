import base64
import math
import unittest
from hawkeye import Map, Navigator, dist, motor_payload


def observation(seq=0, position=(30, 30), heading=0, occupied=(), known=None,
                request='stop', generation=0, hub='', session='phone'):
    n = 40*30
    def bits(indices):
        raw = bytearray((n+7)//8)
        for i in indices:
            raw[i//8] |= 1 << (i%8)
        return base64.b64encode(raw).decode()
    return {'type': 'obs', 'cameraId': 'cam', 'sessionId': session, 'seq': seq,
            'sentAt': 1000, 'calibrated': True, 'veto': False,
            'car': {'position': {'x': position[0], 'y': position[1]}, 'heading': heading},
            'carSource': 'marker', 'carConfidence': .9,
            'goal': {'x': 170, 'y': 120}, 'obstacles': [],
            'arena': {'width': 200, 'height': 150, 'carRadius': 13, 'safetyMargin': 5, 'obstacleRadius': 12},
            'grid': {'cols': 40, 'rows': 30, 'visible': bits(range(n) if known is None else known), 'occupied': bits(occupied)},
            'navigation': {'id': str(generation), 'generation': generation, 'action': request, 'hubId': hub}}


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.nav = Navigator()
        self.feed(observation())

    def feed(self, m, now=0, ready=True):
        self.nav.ingest(m, now=now, wall=1000, car_ready=ready)

    def start(self, **kw):
        m = observation(seq=1, request='start', generation=1, hub=self.nav.hub_id, **kw)
        self.feed(m)
        return m

    def test_no_implicit_start_and_payload(self):
        self.assertEqual(self.nav.command(0), (0, 0, 0))
        self.start()
        self.assertTrue(self.nav.running)
        self.assertNotEqual(self.nav.command(.1), (0, 0, 0))
        self.assertEqual(motor_payload((.18, -.2, 0), 1), {'A': 180, 'B': -200, 'C': 0, 'D': 1})

    def test_obstacle_route_and_unknown(self):
        obstacle = [y*40+x for y in range(10, 17) for x in range(18, 22)]
        self.start(occupied=obstacle)
        self.assertGreater(len(self.nav.path), 2)
        self.assertTrue(all(self.nav.map.clear(a, b) for a, b in zip(self.nav.path, self.nav.path[1:])))
        scene = Map(observation(known=[]))
        self.assertEqual(scene.plan((30, 30), (170, 120)), [])
        self.assertFalse(scene.free((5, 5)))

    def test_no_route_through_wall(self):
        self.start(occupied=[y*40+20 for y in range(30)])
        self.assertFalse(self.nav.running)
        self.assertIn('No safe route', self.nav.message)

    def test_dropout_requires_new_start(self):
        m = self.start()
        self.assertEqual(self.nav.command(.41), (0, 0, 0))
        m['seq'] = 2
        self.feed(m, now=.5)
        self.assertFalse(self.nav.running)
        m['seq'] = 3
        m['navigation'].update(id='2', generation=2)
        self.feed(m, now=.6)
        self.assertTrue(self.nav.running)

    def test_stop_wins_over_inflight_start(self):
        m = self.start()
        self.feed({'type': 'stop', 'sessionId': 'phone', 'hubId': self.nav.hub_id, 'generation': 2, 'requestId': '2'})
        m['seq'] = 2
        self.feed(m)
        self.assertFalse(self.nav.running)

    def test_veto_source_goal_and_ack_stop(self):
        for change in ({'veto': True}, {'calibrated': False}, {'goal': {'x': 100, 'y': 120}}, {'grid': None}):
            self.setUp()
            m = self.start()
            m.update(change)
            m['seq'] = 2
            self.feed(m)
            self.assertFalse(self.nav.running, change)
        self.setUp()
        self.start()
        self.assertEqual(self.nav.command(.1, car_ready=False), (0, 0, 0))

    def test_stale_sequence_timestamp_and_restart(self):
        m = self.start()
        m['car']['position']['x'] = 500
        self.feed(m)
        self.assertEqual(self.nav.seq, 1)
        # A restarted hub must never execute a request for the prior hub.
        other = Navigator()
        other.ingest(m, now=0, wall=1000)
        m['seq'] = 2
        other.ingest(m, now=.1, wall=1000)
        self.assertFalse(other.running)
        m['sentAt'] = 900
        self.feed(m)
        self.assertFalse(self.nav.running)

    def test_new_obstacle_cannot_be_erased(self):
        m = self.start()
        # Outside the car's own body (masked), inside its clearance.
        blocked = observation(occupied=[6*40+9])['grid']
        m.update(seq=2, grid=blocked)
        self.feed(m)
        self.nav.command(.1)
        self.assertFalse(self.nav.running)

    def frame(self, m, seq, now, **change):
        m = {**m, **change, 'seq': seq}
        self.feed(m, now=now)
        return self.nav.command(now)

    def test_trusted_lidar_pose_drives_slower(self):
        m = self.start(heading=math.atan2(90, 140))   # facing the goal
        marker_cmd = self.frame(m, 2, .05)
        lidar_cmd = self.frame(m, 3, .1, carSource='LiDAR', markerAge=.8, trackQuality=.9)
        self.assertEqual(self.nav.state, 'driving')
        self.assertLess(math.hypot(*lidar_cmd[:2]), math.hypot(*marker_cmd[:2]))
        self.assertGreater(math.hypot(*lidar_cmd[:2]), 0)

    def test_untrusted_pose_holds_then_resumes(self):
        m = self.start()
        for i, change in enumerate(({'car': None}, {'carSource': 'LiDAR'},
                                    {'carSource': 'LiDAR', 'markerAge': 2, 'trackQuality': .9},
                                    {'carSource': 'LiDAR', 'markerAge': .5, 'trackQuality': .5})):
            self.assertEqual(self.frame(m, 2+i, .05*(i+1), **change), (0, 0, 0), change)
            self.assertTrue(self.nav.running)
            self.assertEqual(self.nav.state, 'holding')
        # Two good readings are not enough; the third resumes.
        self.assertEqual(self.frame(m, 10, .25), (0, 0, 0))
        self.assertEqual(self.frame(m, 11, .3), (0, 0, 0))
        self.assertNotEqual(self.frame(m, 12, .35), (0, 0, 0))
        self.assertEqual(self.nav.state, 'driving')

    def test_inconsistent_readings_do_not_resume(self):
        m = self.start()
        self.frame(m, 2, .05, car=None)
        for i in range(6):
            x = 30 if i % 2 else 36   # alternating 6 cm apart
            car = {'position': {'x': x, 'y': 30}, 'heading': 0}
            self.assertEqual(self.frame(m, 3+i, .1+.05*i, car=car), (0, 0, 0))
        self.assertEqual(self.nav.state, 'holding')

    def test_long_hold_stops(self):
        m = self.start()
        self.frame(m, 2, 0, car=None)
        for i in range(1, 32):
            self.frame(m, 2+i, .05*i, car=None)
        self.assertFalse(self.nav.running)
        self.assertIn('Car lost', self.nav.message)
        # Good readings after a stop do not restart without Start.
        for i in range(5):
            self.assertEqual(self.frame(m, 40+i, 1.6+.05*i), (0, 0, 0))
        self.assertFalse(self.nav.running)

    def test_position_jump_holds(self):
        m = self.start()
        car = {'position': {'x': 60, 'y': 30}, 'heading': 0}
        self.assertEqual(self.frame(m, 2, .05, car=car), (0, 0, 0))
        self.assertEqual(self.nav.state, 'holding')

    def test_veto_still_stops_immediately(self):
        m = self.start()
        self.frame(m, 2, .05, car=None)
        self.frame(m, 3, .1, veto=True)
        self.assertFalse(self.nav.running)

    def test_car_body_is_not_an_obstacle(self):
        # Cells under the car (e.g. parts its learned outline missed) at Start and during a run.
        body = [y*40+x for y in range(4, 8) for x in range(4, 8)]
        m = self.start(occupied=body)
        self.assertTrue(self.nav.running, self.nav.message)
        grid = observation(position=(33, 30), occupied=[y*40+x for y in range(4, 8) for x in range(5, 9)])['grid']
        car = {'position': {'x': 33, 'y': 30}, 'heading': 0}
        self.assertNotEqual(self.frame(m, 2, .05, car=car, grid=grid), (0, 0, 0))
        self.assertTrue(self.nav.running, self.nav.message)
        # An obstacle beyond the car's radius still counts.
        self.assertIn(6*40+12, Map(observation(occupied=[6*40+12])).occupied)
        self.frame(m, 3, .1, car=car, grid=observation(occupied=[6*40+12])['grid'])
        self.assertIn(6*40+12, self.nav.map.occupied)

    def test_remembered_car_cells_clear_under_car(self):
        # A lagging pose lets part of the car show up as occupied beside it; once the pose
        # catches up and that cell is under the car, the remembered cell must not stop the run.
        m = self.start()
        self.frame(m, 2, .05, grid=observation(occupied=[1*40+3])['grid'])
        self.assertTrue(self.nav.running, self.nav.message)
        car = {'position': {'x': 24, 'y': 20}, 'heading': 0}
        self.assertNotEqual(self.frame(m, 3, .1, car=car), (0, 0, 0))
        self.assertTrue(self.nav.running, self.nav.message)

    def test_body_frame(self):
        # Facing almost toward the waypoint (bearing ~0.57 rad): drive forward, slightly right.
        self.start(heading=.7)
        vx, vy, _ = self.nav.command(.1)
        self.assertGreater(vx, 0)
        self.assertLess(vy, 0)

    def test_turns_in_place_to_face_waypoint(self):
        self.start(heading=math.pi/2)   # facing +y; the goal is to the right (~0.57 rad)
        vx, vy, omega = self.nav.command(.1)
        self.assertEqual((vx, vy), (0, 0))
        self.assertLess(omega, 0)       # clockwise toward the goal

    def simulate(self, lag, turn_rate, start_heading=0., steps=3000):
        """Drive a simulated car whose pose reaches the hub `lag` s late. Returns
        (final position, final heading, number of turn-direction reversals)."""
        m = self.start(heading=start_heading)
        dt, pos, heading = .05, [30., 30.], start_heading
        history, reversals, last_turn = [], 0, 0
        for i in range(2, steps):
            now = i*dt
            history.append((now, list(pos), heading))
            seen = next(h for h in reversed(history) if h[0] <= now-lag+1e-9) if now-lag >= history[0][0] else history[0]
            m['seq'] = i
            m['car'] = {'position': dict(zip(('x', 'y'), seen[1])), 'heading': seen[2]}
            m['sentAt'] = 1000-(now-seen[0])
            self.feed(m, now=now)
            vx, vy, omega = self.nav.command(now)
            pos[0] += (vx*math.cos(heading) - vy*math.sin(heading))*30*dt
            pos[1] += (vx*math.sin(heading) + vy*math.cos(heading))*30*dt
            heading += omega*turn_rate*dt
            if omega and vx == vy == 0:
                turn = 1 if omega > 0 else -1
                reversals += last_turn and turn != last_turn
                last_turn = turn
            if not self.nav.running:
                break
        return pos, heading, reversals

    def test_lagging_camera_does_not_overturn(self):
        # A fast-turning car seen through a 0.3 s camera lag, starting 90° off the goal.
        pos, _, reversals = self.simulate(lag=.3, turn_rate=12, start_heading=math.pi/2+.57)
        self.assertEqual(self.nav.state, 'reached', self.nav.message)
        self.assertLess(dist(pos, (170, 120)), 5)
        self.assertLessEqual(reversals, 2)

    def test_closed_loop(self):
        obstacle = [y*40+x for y in range(10, 17) for x in range(18, 22)]
        m = self.start(occupied=obstacle)
        pos, heading = [30., 30.], 0.
        for i in range(2, 1600):
            m['seq'] = i
            m['car']['position'] = dict(zip(('x', 'y'), pos))
            m['car']['heading'] = heading
            self.feed(m, now=i*.05)
            vx, vy, omega = self.nav.command(i*.05)
            pos[0] += (vx*math.cos(heading) - vy*math.sin(heading))*30*.05
            pos[1] += (vx*math.sin(heading) + vy*math.cos(heading))*30*.05
            heading += omega*3*.05
            self.assertTrue(self.nav.map.free(pos))
            if not self.nav.running:
                break
        self.assertEqual(self.nav.state, 'reached')
        self.assertLess(dist(pos, (170, 120)), 5)


if __name__ == '__main__':
    unittest.main()
