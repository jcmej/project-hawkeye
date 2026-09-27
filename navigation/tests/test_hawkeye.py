import math
import unittest
from hawkeye import Controller, Scene, distance, payload
from simulate import observation


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.s = Scene()
        self.s.known = set(range(self.s.cols * self.s.rows))
        self.s.has_grid = self.s.frozen = True
        self.c = Controller(self.s, 'sim')
        self.c.goal = (170, 120)

    def feed(self, pos=(30, 30), seq=0, now=0, **changes):
        m = observation(self.s, seq, pos)
        m.update(changes)
        self.c.ingest(m, now=now, wall=m['sentAt'])
        return m

    def test_route_avoids_obstacle_and_unknown(self):
        self.s.occupied = {i for i in self.s.known if 85 < self.s.center(i)[0] < 110 and 50 < self.s.center(i)[1] < 90}
        path = self.s.plan((30, 30), (170, 120))
        self.assertGreater(len(path), 2)
        self.assertTrue(all(self.s.segment_free(a, b) for a, b in zip(path, path[1:])))
        self.s.known.clear()
        self.assertEqual(self.s.plan((30, 30), (170, 120)), [])

    def test_wall_and_boundary_block(self):
        self.s.occupied = {i for i in self.s.known if 95 <= self.s.center(i)[0] <= 105}
        self.assertEqual(self.s.plan((30, 30), (170, 120)), [])
        self.assertEqual(self.s.plan((5, 5), (30, 30)), [])

    def test_stale_latches_stop(self):
        self.feed()
        self.c.arm(0)
        self.assertNotEqual(self.c.command(.1), (0, 0, 0))
        self.assertEqual(self.c.command(.41), (0, 0, 0))
        self.feed(seq=1, now=.42)
        self.assertFalse(self.c.armed)

    def test_veto_and_depth_pose_stop(self):
        for changes in ({'veto': True}, {'carSource': 'LiDAR'}, {'calibrated': False}, {'car': None}):
            self.c = Controller(self.s, 'sim')
            self.c.goal = (170, 120)
            self.feed()
            self.c.arm(0)
            self.feed(seq=1, now=.1, **changes)
            self.assertEqual(self.c.command(.1), (0, 0, 0))
            self.assertFalse(self.c.armed)

    def test_order_restart_invalid_and_age(self):
        m = self.feed(seq=5)
        self.assertFalse(self.c.ingest(m, now=.1, wall=m['sentAt']))
        self.c.arm(0)
        self.feed(seq=0, now=.1, sessionId='new')
        self.assertFalse(self.c.armed)
        self.assertFalse(self.c.ingest(m, now=1, wall=m['sentAt'] + 1))
        m['sentAt'] = float('nan')
        self.assertFalse(self.c.ingest(m, now=1, wall=1))

    def test_scan_freeze_and_dimensions(self):
        scene = Scene()
        m = observation(self.s, 0)
        scene.validate(m)
        scene.capture(m)
        self.assertEqual(scene.known, self.s.known)
        m['car'] = {'position': {'x': 30, 'y': 30}}
        with self.assertRaises(ValueError):
            scene.capture(m)
        m['arena']['width'] = 300
        with self.assertRaises(ValueError):
            scene.validate(m)

    def test_body_frame_and_payload(self):
        self.feed((30, 30))
        self.c.goal = (100, 30)
        self.c.msg['car']['heading'] = math.pi / 2
        self.c.arm(0)
        vx, vy, omega = self.c.command(.1)
        self.assertAlmostEqual(vx, 0)
        self.assertLess(vy, 0)
        self.assertEqual(payload((.6, -.2, .5), 42), {'A': 600, 'B': -200, 'C': 500, 'D': 42})

    def test_closed_loop_reaches_goal_around_obstacle(self):
        self.s.occupied = {i for i in self.s.known if 90 < self.s.center(i)[0] < 110 and 50 < self.s.center(i)[1] < 85}
        pos = [30., 30.]
        self.feed(pos)
        self.c.arm(0)
        for i in range(1, 1600):
            now = i * .05
            self.feed(pos, seq=i, now=now)
            vx, vy, _ = self.c.command(now)
            pos[0] += vx * 30 * .05
            pos[1] += vy * 30 * .05
            self.assertTrue(self.s.free(pos))
            if not self.c.armed:
                break
        self.assertLess(distance(pos, self.c.goal), 5)
        self.assertEqual(self.c.reason, 'Goal reached')


if __name__ == '__main__':
    unittest.main()
