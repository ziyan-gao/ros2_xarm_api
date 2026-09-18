"""Numerical checks of the exact position/velocity/acceleration fields sent to JTC."""
import numpy as np
import pytest
from numpy.polynomial import polynomial as poly
from trajectory_msgs.msg import JointTrajectoryPoint

from safe_servo_visualization.smooth_transport import coefficients
from test_continuous_transport import planned_harness, Future


@pytest.mark.parametrize('middle_velocity', [0., .15])
def test_stop_and_nonstop_paths_have_c2_knots_and_bounded_derivatives(middle_velocity):
    h, result = planned_harness()
    middle = JointTrajectoryPoint()
    middle.positions = [.1, .05]
    middle.velocities = [middle_velocity, middle_velocity/2]
    middle.time_from_start.nanosec = 500000000
    result.solution.joint_trajectory.points.insert(1, middle)
    h.transport_max_joint_jerk = .8  # Deliberately restrictive to test retiming.
    h._transport_planned(Future(result))
    assert not h.fault and h.state == h.TRANSPORT_VALIDATING
    points = h.transport_trajectory.points
    assert h.transport_duration > 1.
    previous_end = None
    for old, point in zip(points, points[1:]):
        dt = ((point.time_from_start.sec-old.time_from_start.sec) +
              (point.time_from_start.nanosec-old.time_from_start.nanosec)*1e-9)
        start, end = [], []
        for p0, p1, v0, v1, a0, a1 in zip(old.positions, point.positions, old.velocities,
                                         point.velocities, old.accelerations, point.accelerations):
            c = coefficients(p0, p1, v0, v1, dt, a0, a1)
            start.append([poly.polyval(0., poly.polyder(c, d))/dt**d for d in range(3)])
            end.append([poly.polyval(1., poly.polyder(c, d))/dt**d for d in range(3)])
            for order, cap in [(1, 1.), (2, 1.5), (3, .4)]:
                values = poly.polyval(np.linspace(0., 1., 1001), poly.polyder(c, order))/dt**order
                assert np.max(np.abs(values)) <= cap*(1+1e-7)
        if previous_end is not None:
            np.testing.assert_allclose(start, previous_end, atol=1e-10)
        previous_end = end
        np.testing.assert_allclose(np.array(start)[:, 2], old.accelerations, atol=1e-10)
        np.testing.assert_allclose(np.array(end)[:, 2], point.accelerations, atol=1e-10)
    if middle_velocity == 0:
        assert list(points[1].velocities) == [0., 0.]
    times = [t for _, t in h.transport_checks]
    assert max(np.diff(times)) <= .020000001
    assert np.max(np.abs(np.diff([q for q, _ in h.transport_checks], axis=0))) <= .020000001


def test_dense_smooth_path_no_longer_restarts_acceleration_at_every_knot(monkeypatch):
    import copy
    from rclpy.duration import Duration
    from safe_servo_visualization.smooth_transport import smooth_and_sample, knot_accelerations
    h, result = planned_harness()
    trajectory = result.solution.joint_trajectory
    trajectory.points = []
    times = np.linspace(0., 1., 101)
    for t in times:
        point = JointTrajectoryPoint()
        point.positions = [.2*t**3*(10-15*t+6*t*t)]*2
        point.velocities = [.2*30*t*t*(1-t)**2]*2
        point.time_from_start = Duration(seconds=float(t)).to_msg()
        trajectory.points.append(point)
    expected = .2*(60*times-180*times**2+120*times**3)
    acc = knot_accelerations(trajectory.points, times)
    np.testing.assert_allclose(acc[:, 0], expected, atol=1e-7)
    original = copy.deepcopy(trajectory)
    _, duration, stretch = smooth_and_sample(trajectory, h.transport_joint_limits, 2., 3., 10., .5)
    assert stretch < 1.4  # True peak jerk is 12 rad/s^3; cap is 5.
    monkeypatch.setattr('safe_servo_visualization.smooth_transport.knot_accelerations',
                        lambda points, times: np.zeros((len(points), 2)))
    _, previous_duration, _ = smooth_and_sample(original, h.transport_joint_limits, 2., 3., 10., .5)
    assert duration < previous_duration*.6
    assert any(abs(a) > .01 for p in trajectory.points[1:-1] for a in p.accelerations)
    assert list(trajectory.points[0].accelerations) == [0., 0.]
    assert list(trajectory.points[-1].accelerations) == [0., 0.]


@pytest.mark.parametrize('limit', [0., -1., float('nan'), float('inf')])
def test_invalid_jerk_limit_never_produces_executable_trajectory(limit):
    h, result = planned_harness()
    h.transport_max_joint_jerk = limit
    h._transport_planned(Future(result))
    assert h.state == h.FAULT
    assert not hasattr(h, 'transport_trajectory')


def test_zero_velocity_dwell_is_allowed():
    h, result = planned_harness()
    middle = JointTrajectoryPoint()
    middle.positions = [0., 0.]
    middle.velocities = [0., 0.]
    middle.time_from_start.nanosec = 500000000
    result.solution.joint_trajectory.points.insert(1, middle)
    h._transport_planned(Future(result))
    assert not h.fault
    assert h.state == h.TRANSPORT_VALIDATING  # Must still pass collision and FK checks.


def test_quintic_overshoot_outside_limits_is_rejected():
    h, result = planned_harness()
    middle = JointTrajectoryPoint()
    middle.positions = [.1, .05]
    middle.velocities = [30., 30.]
    middle.time_from_start.nanosec = 500000000
    result.solution.joint_trajectory.points.insert(1, middle)
    h._transport_planned(Future(result))
    assert h.state == h.FAULT and 'position limits' in h.fault


def test_return_budget_failure_can_still_select_staged_fallback(monkeypatch):
    h, result = planned_harness()
    h.transport_is_return = True
    h.return_goal_joints, h.transport_seed = (0., 0.), (.1, .2)
    def reject(*args):
        raise ValueError('transport validation exceeds the bounded sample budget')
    monkeypatch.setattr('safe_servo_visualization.continuous_transport.smooth_and_sample', reject)
    called = []
    h._fallback_staged_return = lambda reason: called.append(reason) or True
    h._transport_planned(Future(result))
    assert called and not h.fault


def test_feasible_original_timing_is_preserved_exactly():
    from safe_servo_visualization.smooth_transport import smooth_and_sample
    h, result = planned_harness()
    trajectory = result.solution.joint_trajectory
    trajectory.points[-1].time_from_start.sec = 4
    report = {}
    _, duration, ratio = smooth_and_sample(trajectory, h.transport_joint_limits, 2., 3., 10., .5, report)
    assert duration == 4. and ratio == 1.
    assert report['strategy'] == 'unchanged'
    assert report['repaired_segments'] == 0


def test_one_short_difficult_segment_does_not_slow_the_whole_path():
    from rclpy.duration import Duration
    from safe_servo_visualization.smooth_transport import smooth_and_sample
    h, result = planned_harness()
    trajectory = result.solution.joint_trajectory
    trajectory.points = []
    for i, t in enumerate([0., 2., 4., 4.1, 6.1, 8.1]):
        point = JointTrajectoryPoint()
        point.positions = [i*.01]*2
        point.velocities = [0. if i in (0, 5) else .005]*2
        point.time_from_start = Duration(seconds=t).to_msg()
        trajectory.points.append(point)
    report = {}
    _, duration, _ = smooth_and_sample(trajectory, h.transport_joint_limits, 2., 3., 10., .5, report)
    assert report['strategy'] == 'local'
    assert duration < 15.
    assert duration < report['original_duration']*report['initial_worst']['factor']*.5
    assert 0 < report['repaired_segments'] < report['total_segments']
    times = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in trajectory.points]
    assert times[1]-times[0] == pytest.approx(2.)
    assert times[-1]-times[-2] == pytest.approx(2.)
    assert all(all(v > 0 for v in p.velocities) for p in trajectory.points[1:-1])
    # Independent dense evaluation of the sent spline's derivative limits.
    for i, (a, b) in enumerate(zip(trajectory.points, trajectory.points[1:])):
        dt = times[i+1]-times[i]
        c = coefficients(a.positions[0], b.positions[0], a.velocities[0], b.velocities[0],
                         dt, a.accelerations[0], b.accelerations[0])
        for order, cap in [(1, 1.), (2, 1.5), (3, 5.)]:
            assert np.max(np.abs(poly.polyval(np.linspace(0, 1, 1001),
                                             poly.polyder(c, order))/dt**order)) <= cap*(1+1e-7)


def test_failed_local_retiming_does_not_publish_partial_edits(monkeypatch):
    import copy
    from safe_servo_visualization.smooth_transport import smooth_and_sample
    h, result = planned_harness()
    trajectory = result.solution.joint_trajectory
    original = copy.deepcopy(trajectory)
    ticks = iter([0., 11.])
    monkeypatch.setattr('safe_servo_visualization.smooth_transport.time.monotonic', lambda: next(ticks))
    with pytest.raises(ValueError, match='timed out'):
        smooth_and_sample(trajectory, h.transport_joint_limits, 2., 3., 10., .5)
    assert trajectory == original


def test_default_mode_does_not_stretch_for_software_jerk_cap():
    h, result = planned_harness()
    h.transport_enforce_jerk_limit = False
    h.transport_max_joint_jerk = .001  # Ignored only because enforcement is disabled.
    h._transport_planned(Future(result))
    assert not h.fault and h.state == h.TRANSPORT_VALIDATING
    assert h.transport_duration == 1.
    assert h.transport_checks[-1][1] == 1.


def test_velocity_acceleration_limits_remain_enforced_without_jerk_guard():
    h, result = planned_harness()
    h.transport_enforce_jerk_limit = False
    h.direct_transfer_max_joint_speed = .01
    h._transport_planned(Future(result))
    assert not h.fault
    assert h.transport_duration >= 59.9
    h, result = planned_harness()
    h.transport_enforce_jerk_limit = False
    h.direct_transfer_joint_acc = .01
    h._transport_planned(Future(result))
    assert not h.fault and h.transport_duration > 1.


def test_optional_jerk_guard_and_predicted_jerk_diagnostics():
    import copy
    from safe_servo_visualization.smooth_transport import smooth_and_sample
    h, result = planned_harness()
    original = result.solution.joint_trajectory
    guarded, unguarded = copy.deepcopy(original), copy.deepcopy(original)
    a, b = {}, {}
    _, fast_duration, _ = smooth_and_sample(unguarded, h.transport_joint_limits, 2., 3., None, .5, a)
    _, guarded_duration, _ = smooth_and_sample(guarded, h.transport_joint_limits, 2., 3., 10., .5, b)
    assert fast_duration == 1. and guarded_duration > fast_duration
    assert not a['jerk_enforced'] and a['peak_jerk'] > 5.
    assert b['jerk_enforced'] and b['peak_jerk'] <= 5.*(1+1e-7)


def test_joint_bounds_are_still_checked_with_jerk_guard_disabled():
    h, result = planned_harness()
    h.transport_enforce_jerk_limit = False
    h.transport_joint_limits['joint2'] = (-.1, .15, 2.)
    h._transport_planned(Future(result))
    assert h.state == h.FAULT and 'position limits' in h.fault
