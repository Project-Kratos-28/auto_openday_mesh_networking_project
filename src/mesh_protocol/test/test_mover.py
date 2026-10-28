import threading
import time

from mesh_protocol.mover import Mover


def test_request_does_not_block():
    gate = threading.Event()
    m = Mover(lambda *t: gate.wait(5) or True)
    t0 = time.time()
    m.request((1.0, 2.0, 3.0))
    assert time.time() - t0 < 0.1
    gate.set()


def test_failed_teleport_is_retried():
    calls = []

    def flaky(*target):
        calls.append(target)
        return len(calls) > 1                              # first try fails (Gazebo busy)
    m = Mover(flaky, retry=0.05)
    m.request((1.0, 2.0, 3.0))
    end = time.time() + 2
    while len(calls) < 2 and time.time() < end:
        time.sleep(0.01)
    assert calls == [(1.0, 2.0, 3.0), (1.0, 2.0, 3.0)]


def test_latest_target_wins():
    seen, gate = [], threading.Event()

    def slow(*target):
        seen.append(target)
        gate.wait(1)
        return True
    m = Mover(slow)
    m.request((1.0, 0.0, 0.0))
    time.sleep(0.05)
    m.request((2.0, 0.0, 0.0))
    m.request((3.0, 0.0, 0.0))
    gate.set()
    end = time.time() + 2
    while len(seen) < 2 and time.time() < end:
        time.sleep(0.01)
    assert seen == [(1.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
