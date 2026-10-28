"""Flies this drone in the background, so slow Gazebo set_pose calls never block the node's radio
handling or the leader's heartbeat.

The drone glides to its target in small steps (`speed` m/s, one step every `period` s), staying at
least `clearance` m above the ground so it never cuts through the crater rim. The latest target wins,
also mid-flight: it turns from wherever it is. A failed step is retried.

Smooth flight (optional): with `accel` set it speeds up and slows down gently instead of starting and
stopping at full speed, and keeps its speed when a new target arrives mid-flight. With `face=True`
the teleport function also gets a yaw (radians), so the drone faces the way it flies.
"""
import math
import threading
import time

MIN_SPEED = 0.5          # m/s: never slower than this near the target (else the last metre takes ages)


class Mover:
    def __init__(self, teleport_fn, get_pos=lambda: None, speed=10.0, period=0.1,
                 floor=lambda x, y: -math.inf, clearance=2.0, retry=1.0, accel=math.inf, face=False):
        self._fn, self._get_pos, self._floor = teleport_fn, get_pos, floor
        self._speed, self._period, self._clear, self._retry = speed, period, clearance, retry
        self._accel, self._face = accel, face
        self._target, self._cv = None, threading.Condition()
        self._at = None                       # where we put the drone mid-flight (None = ask get_pos)
        self._v = 0.0                         # current speed (m/s), kept across a mid-flight re-target
        self._yaw = 0.0                       # current heading (radians)
        threading.Thread(target=self._run, daemon=True).start()

    def request(self, target):
        """Ask to fly to (x, y, z). Returns at once."""
        with self._cv:
            self._target = tuple(float(c) for c in target)
            self._cv.notify()

    def _next(self, here, target):
        """One step from `here` towards `target`, kept above the ground. The last step lands exactly."""
        d = math.dist(here, target)
        if d < 1e-9:
            return target
        stop = math.sqrt(2 * self._accel * d)                    # fastest speed that still stops in time
        self._v = min(self._speed, self._v + self._accel * self._period, max(stop, MIN_SPEED))
        step = self._v * self._period
        if d <= step:
            return target
        x, y, z = (h + (t - h) * step / d for h, t in zip(here, target))
        return (x, y, max(z, self._floor(x, y) + self._clear))

    def _call(self, here, nxt):
        """Teleport to nxt (facing the flight direction if face=True)."""
        if not self._face:
            return self._fn(*nxt)
        if here is not None and math.hypot(nxt[0] - here[0], nxt[1] - here[1]) > 0.01:
            self._yaw = math.atan2(nxt[1] - here[1], nxt[0] - here[0])
        return self._fn(*nxt, self._yaw)

    def _run(self):
        while True:
            with self._cv:
                while self._target is None:
                    self._cv.wait()
                target = self._target
            here = self._at or self._get_pos()
            nxt = target if here is None else self._next(tuple(here), target)
            try:
                ok = self._call(here, nxt)
            except Exception:
                ok = False
            with self._cv:
                if ok:
                    self._at = nxt
                    if nxt == target and self._target == target:
                        self._target, self._at = None, None   # landed: next flight starts from get_pos
                        self._v = 0.0
                        continue
                    self._cv.wait(self._period)          # a newer target wakes us early
                elif self._target == target:
                    self._at, self._v = None, 0.0        # failed: re-read where we are, restart gently
                    self._cv.wait(self._retry)
