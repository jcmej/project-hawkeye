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
        for change in ({'veto': True}, {'calibrated': False}, {'carSource': 'LiDAR'}, {'goal': {'x': 100, 'y': 120}}, {'grid': None}):
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
        blocked = observation(occupied=[6*40+8])['grid']
        m.update(seq=2, grid=blocked)
        self.feed(m)
        self.nav.command(.1)
        self.assertFalse(self.nav.running)

    def test_body_frame(self):
        self.start(heading=math.pi/2)
        vx, vy, _ = self.nav.command(.1)
        self.assertGreater(vx, 0)
        self.assertLess(vy, 0)

    def test_closed_loop(self):
        obstacle = [y*40+x for y in range(10, 17) for x in range(18, 22)]
        m = self.start(occupied=obstacle)
        pos = [30., 30.]
        for i in range(2, 1600):
            m['seq'] = i
            m['car']['position'] = dict(zip(('x', 'y'), pos))
            self.feed(m, now=i*.05)
            vx, vy, _ = self.nav.command(i*.05)
            pos[0] += vx*30*.05
            pos[1] += vy*30*.05
            self.assertTrue(self.nav.map.free(pos))
            if not self.nav.running:
                break
        self.assertEqual(self.nav.state, 'reached')
        self.assertLess(dist(pos, (170, 120)), 5)


if __name__ == '__main__':
    unittest.main()
