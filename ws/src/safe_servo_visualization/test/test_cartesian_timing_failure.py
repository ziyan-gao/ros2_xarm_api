"""MoveIt can return a full geometric path after its time parameterizer fails."""
import pytest

from test_continuous_transport import Future, planned_harness
from test_continuous_return import staged_supervisor


def untimed_result():
    h, result = planned_harness()
    for point in result.solution.joint_trajectory.points:
        point.velocities = []
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0
    return h, result


def returning_harness():
    h, result = untimed_result()
    h.transport_is_return = True
    h.transport_seed = (.1, .2)
    h.return_goal_joints = (0., 0.)
    calls = []
    h._fallback_staged_return = lambda reason: calls.append(reason) or True
    return h, result, calls


def test_missing_timing_is_not_an_outbound_index_error_or_automatic_release():
    h, result = untimed_result()
    h._transport_planned(Future(result))
    assert h.state == h.FAULT
    assert 'MoveIt Cartesian timing unavailable' in h.fault
    assert 'array index' not in h.fault
    assert not h.released and not hasattr(h, 'transport_trajectory')


@pytest.mark.parametrize('case', ['empty_velocities', 'zero_timestamps', 'partial_timing'])
def test_return_without_valid_timing_selects_fallback_before_sending_trajectory(case):
    h, result, calls = returning_harness()
    points = result.solution.joint_trajectory.points
    if case == 'empty_velocities':
        points[-1].time_from_start.sec = 1
    elif case == 'zero_timestamps':
        for point in points:
            point.velocities = [0., 0.]
    else:
        points[0].velocities = [0., 0.]
        points[-1].time_from_start.sec = 1
    h._transport_planned(Future(result))
    assert len(calls) == 1 and 'timing unavailable' in calls[0]
    assert not h.fault and not hasattr(h, 'transport_trajectory')


@pytest.mark.parametrize('bad', [
    'missing_joint', 'duplicate_joint', 'short_position', 'nan_position',
    'short_velocity', 'nan_velocity', 'short_acceleration', 'negative_time',
    'joint_bound', 'wrong_start', 'wrong_observation', 'fraction',
])
def test_missing_timing_cannot_hide_invalid_geometry_or_malformed_fields(bad):
    h, result, calls = returning_harness()
    trajectory = result.solution.joint_trajectory
    first, last = trajectory.points
    if bad == 'missing_joint': trajectory.joint_names = ['joint2']
    if bad == 'duplicate_joint': trajectory.joint_names = ['joint2', 'joint2']
    if bad == 'short_position': first.positions = [0.]
    if bad == 'nan_position': first.positions = [float('nan'), 0.]
    if bad == 'short_velocity': last.velocities = [0.]
    if bad == 'nan_velocity': last.velocities = [float('nan'), 0.]
    if bad == 'short_acceleration': last.accelerations = [0.]
    if bad == 'negative_time': last.time_from_start.sec = -1
    if bad == 'joint_bound':
        h.transport_joint_limits['joint2'] = (-.1, .15, 2.)
    if bad == 'wrong_start': h.transport_seed = (1., 1.)
    if bad == 'wrong_observation': h.return_goal_joints = (1., 1.)
    if bad == 'fraction': result.fraction = 1.1
    h._transport_planned(Future(result))
    assert h.state == h.FAULT and not calls
    assert 'array index' not in h.fault
    assert not hasattr(h, 'transport_trajectory')


@pytest.mark.parametrize('robot_error', [0, 52])
def test_totg_failure_reuses_confirmed_pause_and_staged_mode_handoff(robot_error):
    template, result = untimed_result()
    node, pause, moves = staged_supervisor()
    for attr in ('arm_joint_names', 'direct_transfer_periodic_limits',
                 'direct_transfer_periodic_joint_names', 'transport_joint_limits'):
        setattr(node, attr, getattr(template, attr))
    node.transport_seed = node.latest_joint_positions = (.1, .2)
    node.return_goal_joints = (0., 0.)
    node.robot_error = robot_error
    node._transport_planned(Future(result))
    assert not moves and not hasattr(node, 'transport_trajectory')
    if robot_error:
        assert 'fallback blocked' in node.fault
        return
    assert not node.fault and node.state == node.RETURN_DISABLING
    assert not node.continuous_return_completed
    pause.callback(pause)
    assert moves == [.6]


def test_partial_geometric_path_does_not_use_timing_fallback():
    h, result, calls = returning_harness()
    result.fraction = .8
    h._transport_planned(Future(result))
    assert h.state == h.FAULT and not calls
    assert 'KINEMATIC_REJECTED:' in h.fault
