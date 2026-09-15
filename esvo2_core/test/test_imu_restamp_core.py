import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from imu_restamp_core import Restamper  # noqa: E402


class RestamperTest(unittest.TestCase):
    def test_even_spacing_and_min_offset(self):
        r = Restamper(window_s=2.0)
        true_offset = 1000.0
        out = []
        for i in range(1000):
            dev_us = 5000 * i
            jitter = 0.0 if i % 7 == 0 else 0.004 + 0.011 * ((i * 37) % 10) / 10.0
            out.append(r.update(dev_us, dev_us * 1e-6 + true_offset + jitter))
        # after the first window, stamps are device time + min offset (jitter-free samples every 7th)
        tail = out[500:]
        gaps = [b - a for a, b in zip(tail[:-1], tail[1:])]
        self.assertTrue(all(abs(g - 0.005) < 1e-6 for g in gaps), gaps[:5])
        self.assertAlmostEqual(tail[0] - 500 * 0.005, true_offset, places=6)

    def test_uint32_wrap(self):
        r = Restamper(window_s=2.0)
        start = 2**32 - 5000 * 10
        stamps = []
        for i in range(20):
            dev_us = (start + 5000 * i) % 2**32
            stamps.append(r.update(dev_us, 50.0 + 0.005 * i))
        gaps = [b - a for a, b in zip(stamps[:-1], stamps[1:])]
        self.assertTrue(all(abs(g - 0.005) < 1e-6 for g in gaps), gaps)

    def test_backward_jump_resets_and_drops(self):
        r = Restamper(window_s=2.0)
        for i in range(100):
            self.assertIsNotNone(r.update(5000 * i, 10.0 + 0.005 * i))
        # device clock restarts (not a wrap: jump back by ~0.5 s, far from 2^32)
        self.assertIsNone(r.update(5000 * 0, 10.5))
        self.assertIsNotNone(r.update(5000 * 1, 10.505))

    def test_output_strictly_increasing(self):
        r = Restamper(window_s=0.05)
        prev = None
        for i in range(400):
            dev_us = 5000 * i
            rx = dev_us * 1e-6 + 3.0 + (0.02 if (i // 20) % 2 else 0.0)
            t = r.update(dev_us, rx)
            if prev is not None:
                self.assertGreater(t, prev)
            prev = t


if __name__ == '__main__':
    unittest.main()
