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
    m.request((1.0, 50.0, 0.0))                           # superseded before it is started
    m.request((1.0, 0.0, 0.5))                            # 0.5 m away: one step
    gate.set()
    end = time.time() + 2
    while len(seen) < 2 and time.time() < end:
        time.sleep(0.01)
    assert seen == [(1.0, 0.0, 0.0), (1.0, 0.0, 0.5)]


def _record():
    calls, done = [], threading.Event()

    def fn(*p):
        calls.append(p)
        return True
    return calls, fn


def _wait_until(cond, t=3.0):
    end = time.time() + t
    while not cond() and time.time() < end:
        time.sleep(0.01)
    return cond()


def test_glides_in_small_steps_to_the_target():
    calls, fn = _record()
    m = Mover(fn, get_pos=lambda: (0.0, 0.0, 5.0), speed=20.0, period=0.02)
    m.request((4.0, 0.0, 5.0))
    assert _wait_until(lambda: calls and calls[-1] == (4.0, 0.0, 5.0))
    assert len(calls) >= 8                                    # 4 m at 0.4 m a step
    steps = [((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
             for a, b in zip([(0.0, 0.0, 5.0)] + calls, calls)]
    assert max(steps) <= 20.0 * 0.02 * 1.5 + 1e-6


def test_stays_above_the_ground():
    calls, fn = _record()
    ground = lambda x, y: 10.0 if 1.0 < x < 3.0 else 0.0     # a ridge in the way  # noqa: E731
    m = Mover(fn, get_pos=lambda: (0.0, 0.0, 3.0), speed=20.0, period=0.02, floor=ground, clearance=2.0)
    m.request((4.0, 0.0, 3.0))
    assert _wait_until(lambda: calls and calls[-1] == (4.0, 0.0, 3.0))
    assert all(z >= ground(x, y) + 2.0 - 1e-6 for x, y, z in calls[:-1])


def test_new_target_mid_flight_turns_from_where_it_is():
    calls, fn = _record()
    m = Mover(fn, get_pos=lambda: (0.0, 0.0, 5.0), speed=10.0, period=0.02)
    m.request((10.0, 0.0, 5.0))
    assert _wait_until(lambda: len(calls) >= 5)
    m.request((0.0, 10.0, 5.0))
    assert _wait_until(lambda: calls[-1] == (0.0, 10.0, 5.0))
    turn = next(i for i, c in enumerate(calls) if c[1] > 0)
    assert 0 < calls[turn - 1][0] < 10.0                      # it turned in mid-air, not from the start


def _steps(start, calls):
    return [((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
            for a, b in zip([start] + calls, calls)]


def test_speeds_up_and_slows_down_gently():
    calls, fn = _record()
    m = Mover(fn, get_pos=lambda: (0.0, 0.0, 5.0), speed=20.0, period=0.02, accel=100.0)
    m.request((20.0, 0.0, 5.0))
    assert _wait_until(lambda: calls and calls[-1] == (20.0, 0.0, 5.0))
    steps = _steps((0.0, 0.0, 5.0), calls)
    assert steps[0] < steps[len(steps) // 2]                  # starts slow, full speed in the middle
    assert steps[-2] < steps[len(steps) // 2]                 # slows down before landing
    assert max(b - a for a, b in zip(steps, steps[1:])) <= 100.0 * 0.02 * 0.02 + 1e-6   # no sudden jump


def test_faces_the_way_it_flies():
    calls = []
    m = Mover(lambda *p: calls.append(p) or True, get_pos=lambda: (0.0, 0.0, 5.0),
              speed=20.0, period=0.02, face=True)
    m.request((0.0, 4.0, 5.0))                                # fly along +y: yaw = 90 degrees
    assert _wait_until(lambda: calls and calls[-1][:3] == (0.0, 4.0, 5.0))
    assert all(abs(c[3] - 1.5708) < 0.01 for c in calls)


def test_flight_time_matches_the_mover():
    from mesh_common import flight_time, DRONE_SPEED, DRONE_ACCEL
    for dist in (2.0, 40.0):
        calls, fn = _record()
        m = Mover(fn, get_pos=lambda: (0.0, 0.0, 5.0), speed=DRONE_SPEED, period=0.01, accel=DRONE_ACCEL)
        m.request((dist, 0.0, 5.0))
        assert _wait_until(lambda: calls and calls[-1] == (dist, 0.0, 5.0), t=10)
        assert abs(len(calls) * 0.01 - flight_time(dist)) < 0.25 * flight_time(dist) + 0.1
