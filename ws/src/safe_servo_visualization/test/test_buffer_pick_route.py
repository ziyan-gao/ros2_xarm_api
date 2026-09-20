import math
import numpy as np
import pytest
from safe_servo_visualization.transport_path import pick_waypoints, buffer_transfer_waypoints


def test_buffer_pick_uses_shared_observation_rail_and_vertical_end_columns():
    start, end, observation = (-.2, -.4, .2), (-.1, .5, .4), (.5, 0., .6)
    qa, qb = (1., 0, 0, 0), (0., 1., 0, 0)
    samples, safe, high = pick_waypoints(start, qa, end, qb, .5, observation_xyz=observation)
    elbow, via = buffer_transfer_waypoints(start, end, observation, high)
    assert elbow == pytest.approx((.35, -.4, high))
    assert via == pytest.approx((.35, .5, high))
    xyz = np.array([p for p, q in samples])
    # Rail crossing survives rounded virtual corners.
    middle = xyz[(xyz[:, 1] > -.3) & (xyz[:, 1] < .4)]
    assert len(middle) > 0
    assert np.allclose(middle[:, 0], .35)
    assert np.all(middle[:, 2] >= safe)
    for p, q in samples:
        if p[2] < safe-1e-8:
            assert (np.linalg.norm(p[:2]-np.array(start[:2])) < 1e-8 or
                    np.linalg.norm(p[:2]-np.array(end[:2])) < 1e-8)
        if abs(np.dot(q, qa)) < math.cos(.01) and abs(np.dot(q, qb)) < math.cos(.01):
            assert p == pytest.approx(via)
    assert xyz[-1] == pytest.approx(end)


def test_default_pick_path_does_not_use_buffer_rail():
    samples, _, _ = pick_waypoints((-.2, -.4, .2), (1, 0, 0, 0),
                                  (-.1, .5, .4), (1, 0, 0, 0), .5)
    assert max(p[0] for p, q in samples) <= -.1+1e-8
