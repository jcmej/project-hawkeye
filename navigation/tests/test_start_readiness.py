import json
import time
import unittest
from hawkeye import Navigator
from hub import Receiver
from test_navigation import observation


class StartReadinessTests(unittest.TestCase):
    def test_rejected_first_frame_reports_reason_without_enabling_motion(self):
        nav = Navigator(dry_run=True)
        receiver = Receiver(nav, None)
        replies = []
        class Transport:
            def sendto(self, data, addr):
                replies.append(json.loads(data))
        receiver.connection_made(Transport())
        m = observation()
        m['sentAt'] = time.time() - 20
        receiver.datagram_received(json.dumps(m).encode(), ('127.0.0.1', 50000))
        receiver.reply()
        self.assertIsNone(nav.session)
        self.assertFalse(nav.running)
        self.assertEqual(replies[-1]['sessionId'], 'phone')
        self.assertEqual(replies[-1]['seq'], 0)
        self.assertEqual(replies[-1]['state'], 'blocked')
        self.assertIn('clock mismatch', replies[-1]['message'])
        m.update(seq=1, sentAt=time.time())
        receiver.datagram_received(json.dumps(m).encode(), ('127.0.0.1', 50000))
        receiver.reply()
        self.assertEqual(replies[-1]['state'], 'ready')
        self.assertFalse(nav.running)

    def test_clock_error_after_handshake_recovers_on_valid_stop_frames(self):
        nav = Navigator()
        m = observation()
        nav.ingest(m, now=0, wall=1000)
        m['seq'] = 1
        nav.ingest(m, now=.1, wall=1002)
        self.assertEqual(nav.state, 'blocked')
        m['seq'] = 2
        nav.ingest(m, now=.2, wall=1000)
        self.assertEqual(nav.state, 'ready')
        self.assertFalse(nav.running)
