"""Moves this drone in the background, so a slow Gazebo set_pose call (up to seconds) never blocks the
node's radio handling or the leader's heartbeat. The latest target wins; a failed move is retried."""
import threading


class Mover:
    def __init__(self, teleport_fn, retry=1.0):
        self._fn, self._retry = teleport_fn, retry
        self._target, self._cv = None, threading.Condition()
        threading.Thread(target=self._run, daemon=True).start()

    def request(self, target):
        """Ask to move to (x, y, z). Returns at once."""
        with self._cv:
            self._target = tuple(target)
            self._cv.notify()

    def _run(self):
        while True:
            with self._cv:
                while self._target is None:
                    self._cv.wait()
                target = self._target
            try:
                ok = self._fn(*target)
            except Exception:
                ok = False
            with self._cv:
                if self._target == target:
                    if ok:
                        self._target = None
                    else:                       # failed: wait, then retry unless a newer target came
                        self._cv.wait(self._retry)
