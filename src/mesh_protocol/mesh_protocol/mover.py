"""Flies this drone in the background, so slow Gazebo set_pose calls never block the node's radio
handling or the leader's heartbeat.

The drone glides to its target in small steps (`speed` m/s, one step every `period` s), staying at
least `clearance` m above the ground so it never cuts through the crater rim. The latest target wins,
also mid-flight: it turns from wherever it is. A failed step is retried.
"""
import math
import threading
import time


class Mover:
    def __init__(self, teleport_fn, get_pos=lambda: None, speed=10.0, period=0.1,
                 floor=lambda x, y: -math.inf, clearance=2.0, retry=1.0):
        self._fn, self._get_pos, self._floor = teleport_fn, get_pos, floor
        self._speed, self._period, self._clear, self._retry = speed, period, clearance, retry
        self._target, self._cv = None, threading.Condition()
        self._at = None                       # where we put the drone mid-flight (None = ask get_pos)
        threading.Thread(target=self._run, daemon=True).start()

    def request(self, target):
        """Ask to fly to (x, y, z). Returns at once."""
        with self._cv:
            self._target = tuple(float(c) for c in target)
            self._cv.notify()

    def _next(self, here, target):
        """One step from `here` towards `target`, kept above the ground. The last step lands exactly."""
        d = math.dist(here, target)
        step = self._speed * self._period
        if d <= step:
            return target
        x, y, z = (h + (t - h) * step / d for h, t in zip(here, target))
        return (x, y, max(z, self._floor(x, y) + self._clear))

    def _run(self):
        while True:
            with self._cv:
                while self._target is None:
                    self._cv.wait()
                target = self._target
            here = self._at or self._get_pos()
            nxt = target if here is None else self._next(tuple(here), target)
            try:
                ok = self._fn(*nxt)
            except Exception:
                ok = False
            with self._cv:
                if ok:
                    self._at = nxt
                    if nxt == target and self._target == target:
                        self._target, self._at = None, None   # landed: next flight starts from get_pos
                        continue
                    self._cv.wait(self._period)          # a newer target wakes us early
                elif self._target == target:
                    self._at = None                      # failed: re-read where we are, then retry
                    self._cv.wait(self._retry)
