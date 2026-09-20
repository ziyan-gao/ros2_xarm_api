"""Offline regressions: TOTG fallback must never execute a partial path."""
import copy
from types import SimpleNamespace as NS

import numpy as np
import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath

from safe_servo_visualization.staged_transport_timing import split_transport_request
from test_continuous_transport import Future, planned_harness


def request():
    req = GetCartesianPath.Request()
    req.avoid_collisions = True
    for x, z in [(0., .3), (0., .5), (.2, .5), (.4, .5), (.4, .3)]:
        p = Pose()
        p.position.x, p.position.z = x, z
        p.orientation.w = 1.
        req.waypoints.append(p)
    return req


def harness():
    h, result = planned_harness()
    h.transport_cartesian_request = request()
    h.get_logger = lambda: NS(info=lambda *a: None, warning=lambda *a: None)
    h._transport_guard = lambda cb: cb
    pending = []

    def call(req):
        f = Future(None)
        pending.append((req, f))
        return f

    h.transport_cartesian = NS(call_async=call)
    h._transport_fk_request = lambda joints, cb: setattr(h, 'fk_callback', cb)
    h._transport_pose = lambda value: value
    return h, result, pending


def test_split_keeps_all_poses_orientations_and_request_options():
    req = request()
    original = copy.deepcopy(req)
    parts = split_transport_request(req)
    assert [len(p.waypoints) for p in parts] == [2, 2, 1]
    assert [pose for p in parts for pose in p.waypoints] == req.waypoints
    assert all(p.avoid_collisions for p in parts)
    parts[0].waypoints[0].position.x = 7.
    assert req == original


def test_untimed_full_path_plans_all_stages_before_full_validation():
    h, result, pending = harness()
    for point in result.solution.joint_trajectory.points:
        point.velocities = []
    h._transport_planned(Future(result))
    assert h.staged_timing_used and len(pending) == 1
    assert not h.fault and not h.released
    joined = []
    # Capture entry into the existing full retiming/validation pipeline.
    h._transport_planned = lambda f: joined.append(f.result())
    for index in range(3):
        req, future = pending[index]
        _, timed = planned_harness()
        traj = timed.solution.joint_trajectory
        traj.joint_names = list(h.arm_joint_names)
        traj.points[0].positions = list(h.staged_timing_seed)
        traj.points[1].positions = [v+.01 for v in h.staged_timing_seed]
        future.value = timed
        future.callback(future)
        assert not joined  # FK must pass before planning the next stage.
        p = req.waypoints[-1]
        h.fk_callback(Future((np.array([p.position.x, p.position.y, p.position.z]),
                              np.array([0., 0., 0., 1.]))))
    assert len(joined) == 1 and len(pending) == 3
    assert not h.released and not h.fault
    points = joined[0].solution.joint_trajectory.points
    assert len(points) == 4
    assert all(list(p.velocities) == [0., 0.] for p in points)
    assert not h._fallback_staged_transport_timing('again')


@pytest.mark.parametrize('failure', ['partial', 'untimed', 'start', 'limits', 'fk'])
def test_bad_stage_stops_without_retry_or_release(failure):
    h, result, pending = harness()
    assert h._fallback_staged_transport_timing('timing unavailable')
    if failure == 'partial':
        result.fraction = .5
    elif failure == 'untimed':
        result.solution.joint_trajectory.points[0].velocities = []
    elif failure == 'start':
        result.solution.joint_trajectory.points[0].positions = [.5, .5]
    elif failure == 'limits':
        result.solution.joint_trajectory.points[-1].positions = [3., 3.]
    h._staged_timing_received(Future(result))
    if failure == 'fk':
        h.fk_callback(Future((np.zeros(3), np.array([0., 0., 0., 1.]))))
    assert h.state == h.FAULT and not h.released
    assert len(pending) == 1


@pytest.mark.parametrize('field,value', [
    ('transport_is_return', True), ('transport_is_pick', True),
    ('transport_route_attempt', 1), ('transport_sdk_candidate', True),
    ('direct_transfer_motion_started', True), ('transfer_goal_handle', object()),
    ('staged_timing_used', True), ('state', 'TRANSPORT_EXECUTING')])
def test_fallback_is_narrow_and_pre_execution_only(field, value):
    h, _, pending = harness()
    setattr(h, field, value)
    assert not h._fallback_staged_transport_timing('timing unavailable')
    assert not pending


def test_partial_original_path_does_not_enter_timing_fallback():
    h, result, pending = harness()
    result.fraction = .9
    h._transport_planned(Future(result))
    assert not pending and not getattr(h, 'staged_timing_used', False)


def test_valid_timing_uses_original_pipeline():
    h, result, pending = harness()
    h._transport_planned(Future(result))
    assert h.state == h.TRANSPORT_VALIDATING and not pending


def test_cached_path_cannot_replay_last_connector_as_whole_transport():
    h, _, pending = harness()
    h.reuse_active, h.reuse_operation, h.operation_id = True, 7, 7
    assert not h._fallback_staged_transport_timing('timing unavailable')
    assert not pending


@pytest.mark.parametrize('malformed', ['nan', 'limits'])
def test_invalid_original_geometry_does_not_start_retry(malformed):
    h, result, pending = harness()
    for p in result.solution.joint_trajectory.points:
        p.velocities = []
    result.solution.joint_trajectory.points[-1].positions = [
        float('nan') if malformed == 'nan' else 3., 0.]
    h._transport_planned(Future(result))
    assert h.state == h.FAULT and not pending


def test_stale_callback_cannot_continue_after_fault():
    h, _, pending = harness()
    h.operation_id, h.transport_route_generation = 1, 0
    # Use the production operation/state guard rather than the test stub.
    from safe_servo_visualization.continuous_transport import ContinuousTransport
    h._transport_guard = lambda cb: ContinuousTransport._transport_guard(h, cb)
    assert h._fallback_staged_transport_timing('timing unavailable')
    h._fault('cancelled')
    pending[0][1].callback(pending[0][1])
    assert len(pending) == 1 and h.fault == 'cancelled'
