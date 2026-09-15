"""Re-stamp IMU samples with the IMU's own clock, anchored to ROS time.

offset = min over the last window of (receive_time - device_time). Serial
delay and batching only ever add delay, so the minimum is the best estimate;
the window follows slow clock drift.
"""
from collections import deque

WRAP_US = 2 ** 32
BACKWARD_JUMP_S = 1.0  # a device time step further back than this (and not a wrap) is a reset


class Restamper:
    def __init__(self, window_s=2.0):
        self.window_s = window_s
        self._reset()

    def _reset(self):
        self._last_raw_us = None
        self._wraps = 0
        self._last_dev_s = None
        self._win = deque()  # monotonic deque of (device_s, offset), increasing offsets
        self._last_out = None

    def update(self, device_us, receive_s):
        """Return the published stamp in seconds, or None if the sample is dropped."""
        if self._last_raw_us is not None and device_us < self._last_raw_us:
            if self._last_raw_us - device_us > WRAP_US // 2:
                self._wraps += 1
            else:
                self._reset()
                return None
        self._last_raw_us = device_us
        dev_s = (self._wraps * WRAP_US + device_us) * 1e-6
        if self._last_dev_s is not None and dev_s < self._last_dev_s - BACKWARD_JUMP_S:
            self._reset()
            return None
        self._last_dev_s = dev_s

        off = receive_s - dev_s
        while self._win and self._win[-1][1] >= off:
            self._win.pop()
        self._win.append((dev_s, off))
        while self._win[0][0] < dev_s - self.window_s:
            self._win.popleft()

        out = dev_s + self._win[0][1]
        if self._last_out is not None and out <= self._last_out:
            out = self._last_out + 1e-6
        self._last_out = out
        return out
