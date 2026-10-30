import threading
import time

import pytest

pytest.importorskip("gz.transport13")


def test_simulation_time_going_backwards_is_seen_as_a_restart():
    """Gazebo's 'Restart simulation' sets the simulation time back: the link model must notice."""
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from mesh_link_model.link_model import start_gazebo_poses

    state, lock = {"pos": {}, "yaw": {}}, threading.Lock()
    keep = start_gazebo_poses(state, lock, "restarttest")           # noqa: F841 (keep the subscriber alive)
    node = Node()
    pub = node.advertise("/world/restarttest/pose/info", Pose_V)

    def send(sec):
        m = Pose_V()
        m.header.stamp.sec = sec
        p = m.pose.add()
        p.name, p.position.x, p.orientation.w = "primary_rover", float(sec), 1.0
        for _ in range(10):                                         # the subscriber takes at most 10 messages a second
            pub.publish(m)
            time.sleep(0.2)
            if state.get("sim_t") == float(sec):
                return

    for sec in (100, 101, 102):
        send(sec)
    assert not state.get("reset")                                   # time moving forward: nothing to report
    send(2)                                                         # sim time jumps back from 102 s to 2 s
    assert state["reset"] is True and state["pos"]["rover"][0] == 2.0
