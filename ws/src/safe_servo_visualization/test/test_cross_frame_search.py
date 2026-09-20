from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from safe_servo_visualization.waypoint_search import next_grid
from safe_servo_visualization.transport_path import directed_slerp, rotate
from safe_servo_visualization.continuous_transport import ContinuousTransport
from safe_servo_visualization.staging_slots_node import StagingSlots
from safe_servo_visualization.motion_coordinator_node import MotionCoordinator


def test_grid_order_is_finite():
    phase = (0, 1)
    phases = []
    while phase is not None:
        phases.append(phase)
        phase = next_grid(*phase)
    assert phases == [(0, 1), (1, 1), (0, -1), (1, -1)]


def test_half_turns_have_opposite_midpoints_same_endpoint_and_no_tilt():
    a, b = (1, 0, 0, 0), (0, 1, 0, 0)
    left, right = [directed_slerp(a, b, .5, d) for d in (1, -1)]
    assert not np.allclose(rotate(left, (1, 0, 0)), rotate(right, (1, 0, 0)))
    for direction in (1, -1):
        assert abs(np.dot(directed_slerp(a, b, 1, direction), b)) == pytest.approx(1)
        assert np.allclose(rotate(directed_slerp(a, b, .5, direction), (0, 0, 1)), (0, 0, -1))


def test_timer_advances_hung_pick_request_without_execution():
    calls = []
    h = NS(state='planning', TRANSPORT_PLANNING='planning', TRANSPORT_DIAGNOSING='diagnosing',
           transport_is_pick=True, pick_cross_area=True, pick_grid_deadline=5.,
           _try_pick_grid=lambda: calls.append('retry') or True)
    assert not ContinuousTransport._grid_deadline_tick(h, 4.99)
    assert ContinuousTransport._grid_deadline_tick(h, 5.)
    assert calls == ['retry']
    h.direct_transfer_motion_started = True
    assert not ContinuousTransport._grid_deadline_tick(h, 6.)


def test_inspection_timer_advances_hung_request():
    node = object.__new__(StagingSlots)
    node.state = node.INSPECTION_CHECK
    node.inspection_grid_deadline = 0.
    node.inspection_start_future = object()
    node.inspection_generation = 3
    calls = []
    node._retry_inspection_candidate = calls.append
    node._inspection_tick()
    assert calls == [3]


def test_coordinator_accepts_and_rejects_inspection_route_hints():
    node = object.__new__(MotionCoordinator)
    node.get_logger = lambda: NS(warning=lambda _: None)
    values = [0., .2, .3, .3, math.pi, 0., 0., .1, .1, .1, .03, 0., .5, 1.]
    node.staging_retrieve_target_callback(NS(data=values + [1., 20., -1.]))
    assert node.staging_retrieve_target['inspection_route'] == [1, 20, -1]
    node.staging_retrieve_target_callback(NS(data=values + [2., 20., -1.]))
    assert node.staging_retrieve_target is None
