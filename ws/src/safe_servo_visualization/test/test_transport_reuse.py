"""Offline path-cache tests. No ROS node, controller, or robot is started."""
import copy
from types import SimpleNamespace as NS

import numpy as np
import pytest
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from safe_servo_visualization.transport_reuse import (
    TransportReuse, attachment_key, overhead_entry, compatible_route, reverse_path, stamp)
from safe_servo_visualization.continuous_transport import ContinuousTransport


DOWN = np.array([1., 0., 0., 0.])


def trajectory():
    path = JointTrajectory(joint_names=['j1', 'j2'])
    for i in range(7):
        path.points.append(JointTrajectoryPoint(
            positions=[i*.1, 0.], velocities=[.1, 0.], accelerations=[.01, 0.],
            time_from_start=Duration(seconds=float(i)).to_msg()))
    return path


def entry():
    path = trajectory()
    poses = {float(i): (np.array([0., -.3+.1*i, .6 if 1 <= i <= 5 else .2]), DOWN.copy())
             for i in range(7)}
    return overhead_entry(path, poses, .5, None, 'key')


def choose(entries, start=(0., -.3, .2), end=(0., .3, .3), key='key', floor=.5, qa=DOWN, qb=DOWN):
    return compatible_route(entries, key, np.array(start), qa, np.array(end), qb, floor, .8, None)


def test_only_contiguous_overhead_knots_saved():
    saved = entry()
    assert len(saved['path'].points) == 5
    assert saved['path'].points[0].positions == pytest.approx([.1, 0.])
    assert saved['path'].points[-1].positions == pytest.approx([.5, 0.])
    assert [stamp(p) for p in saved['path'].points] == [0., 1., 2., 3., 4.]
    path = trajectory()
    poses = {float(i): (np.array([0., .1*i, .6]), DOWN.copy()) for i in range(7) if i != 3}
    saved = overhead_entry(path, poses, .5, None, 'key')
    assert len(saved['path'].points) == 3  # Never bridge the missing FK knot.


def test_reverse_preserves_positions_acceleration_and_reverses_velocity_time():
    path = trajectory()
    reversed_path = reverse_path(path)
    assert reversed_path.points[0].positions == path.points[-1].positions
    assert list(reversed_path.points[2].velocities) == [-.1, 0.]
    assert list(reversed_path.points[2].accelerations) == [.01, 0.]
    assert [stamp(p) for p in reversed_path.points] == list(range(7))
    assert reverse_path(reversed_path) == path


def test_select_direction_and_allow_changed_inspection_xy():
    saved = entry()
    selected = choose([saved], start=(.01, -.31, .21), end=(0., .3, .25))
    assert selected and not selected['reversed']
    selected = choose([saved], start=(0., .3, .2), end=(.015, -.3, .24))
    assert selected['reversed']
    selected['path'].points[0].positions[0] = 100.
    assert saved['path'].points[-1].positions[0] == .5  # no mutation of cache


def test_changed_grasp_yaw_clearance_and_remote_routes_miss_cache():
    saved = entry()
    assert choose([saved], key='changed grasp') is None
    assert choose([saved], floor=.7) is None
    assert choose([saved], start=(2., 2., .2)) is None
    assert choose([saved], qa=np.array([0., 1., 0., 0.])) is None
    saved['poses'][2] = (saved['poses'][2][0], np.array([0., 1., 0., 0.]))
    assert choose([saved]) is None  # no inherited mid-path yaw excursion


def test_attachment_key_preserves_grasp_but_not_item_id():
    scene = dict(attached_item_id='a', attached_item_size_m=[.2, .1, .1],
                 attached_item_center_in_tcp_m=[0, 0, .05],
                 attached_item_orientation_in_tcp_xyzw=[0, 0, 0, 1])
    changed = copy.deepcopy(scene)
    changed['attached_item_id'] = 'b'
    assert attachment_key(scene) == attachment_key(changed)
    changed['attached_item_center_in_tcp_m'][0] = .001
    assert attachment_key(scene) != attachment_key(changed)
    assert attachment_key(scene) != attachment_key(None)


@pytest.mark.parametrize('orientation', ([0., 0., 0., 1.], [1., 0., 0., 0.],
                                        [0., -.6, .8, 0.]))
def test_attachment_quaternion_sign_and_scale_do_not_change_key(orientation):
    scene = dict(attached_item_size_m=[.2, .1, .1],
                 attached_item_center_in_tcp_m=[0., 0., .05],
                 attached_item_orientation_in_tcp_xyzw=orientation)
    opposite = dict(scene, attached_item_orientation_in_tcp_xyzw=[-2*v for v in orientation])
    assert attachment_key(scene) == attachment_key(opposite)
    changed = dict(scene, attached_item_orientation_in_tcp_xyzw=[0., 0., .1, 1.])
    assert attachment_key(scene) != attachment_key(changed)


class Harness(ContinuousTransport):
    def __init__(self):
        self.state = self.TRANSPORT_PLANNING
        self.operation_id = self.reuse_operation = 3
        self.reuse_active = True
        self.reuse_start_future = object()
        self.transport_route_generation = 7
        self.transport_is_pick = True
        self.reuse_deadline = 10.
        self.calls = []

    def get_logger(self):
        return NS(info=lambda s: self.calls.append(s))

    def _transport_start_fk(self, future):
        self.calls.append(future)


def test_cache_timeout_restarts_original_search_and_invalidates_late_callbacks():
    h = Harness()
    assert h._reuse_tick(9.) and h.reuse_active
    callback = h._transport_guard(lambda _: pytest.fail('stale cache callback'))
    assert h._reuse_tick(10.)
    assert not h.reuse_active and h.transport_route_generation == 8
    assert h.calls[-1] is h.reuse_start_future
    assert h.transport_descent_time is None
    callback(None)


def test_no_fallback_after_execution_or_on_other_operation():
    h = Harness()
    h.direct_transfer_motion_started = True
    assert not h._reuse_fallback('bad')
    assert not h.calls
    h.direct_transfer_motion_started = False
    h.operation_id += 1
    assert not h._reuse_fallback('stale')


def test_collision_and_geometry_rejection_restart_planning_not_execution():
    for reason in ('collision', 'geometry'):
        h = Harness()
        h.state = h.TRANSPORT_VALIDATING
        if reason == 'collision':
            h._transport_collision_checked(NS(result=lambda: NS(valid=False)))
        else:
            h._transport_reject_geometry('cached item too low')
        assert not h.reuse_active
        assert h.state == h.TRANSPORT_PLANNING
        assert h.calls[-1] is h.reuse_start_future


def test_incoming_wrong_joint_branch_is_not_spliced():
    h = Harness()
    h.transport_seed = (10., 10.)
    h._reuse_result = lambda _: trajectory()
    h._reuse_incoming(None)
    assert not h.reuse_active
    assert any('different measured-start joint branch' in str(c) for c in h.calls)


def test_combined_cache_candidate_goes_through_existing_validator():
    h = Harness()
    incoming = trajectory()
    cached = trajectory()
    outgoing = trajectory()
    for point in cached.points:
        point.positions[0] += .6
    for point in outgoing.points:
        point.positions[0] += 1.2
    h.arm_joint_names = ['j1', 'j2']
    h.reuse_incoming_path = incoming
    h.reuse_candidate = dict(path=cached)
    h._reuse_result = lambda _: outgoing
    h._transport_planned = lambda f: h.calls.append(f.result())
    h._reuse_outgoing(None)
    result = h.calls[-1]
    assert result.fraction == 1.
    assert len(result.solution.joint_trajectory.points) == 19
    assert h.reuse_active  # validation, not cache lookup, must authorize execution


def test_failed_or_partial_results_are_not_accepted(monkeypatch):
    h = Harness()
    monkeypatch.setattr('safe_servo_visualization.transport_reuse.time.monotonic', lambda: 1.)
    for fraction in (.99, float('nan'), 1.1):
        with pytest.raises(ValueError, match='incomplete'):
            h._reuse_result(NS(result=lambda: NS(error_code=NS(val=1), fraction=fraction)))


def test_cache_is_bounded_and_excludes_return_and_sdk():
    h = Harness()
    h.transport_path_reuse_enabled = True
    h.pick_cross_area = True
    h.transport_path_cache = []
    h.transport_trajectory = trajectory()
    h._begin_reuse_recording(h.transport_trajectory)
    h._mark_reuse_execution()
    h.transport_validated_poses = {float(i): (np.array([0., i*.1, .6]), DOWN) for i in range(7)}
    h.transport_scene = None
    h.transport_clearance = .5
    h._reuse_key = lambda: 'key'
    for _ in range(20):
        h._remember_overhead_path()
    assert len(h.transport_path_cache) == 12
    h.transport_is_return = True
    h._remember_overhead_path()
    assert len(h.transport_path_cache) == 12
    h.transport_is_return = False
    h.transport_sdk_candidate = True
    h.transport_path_cache.clear()
    h._remember_overhead_path()
    assert not h.transport_path_cache


def recording_harness():
    h = Harness()
    h.transport_path_reuse_enabled = True
    h.pick_cross_area = True
    h.transport_path_cache = []
    h.transport_trajectory = trajectory()
    h.transport_scene = None
    h.transport_clearance = .5
    h._reuse_key = lambda: 'key'
    h._begin_reuse_recording(h.transport_trajectory)
    for i in range(7):
        h._record_reuse_pose(float(i), np.array([0., i*.1, .6]), DOWN)
    return h


@pytest.mark.parametrize('change', ('operation', 'generation', 'trajectory', 'unexecuted'))
def test_only_executed_current_operation_and_route_can_be_cached(change):
    h = recording_harness()
    if change != 'unexecuted':
        h._mark_reuse_execution()
    if change == 'operation':
        h.operation_id += 1
    elif change == 'generation':
        h.transport_route_generation += 1
    elif change == 'trajectory':
        h.transport_trajectory = copy.deepcopy(h.transport_trajectory)
    h._remember_overhead_path()
    assert not h.transport_path_cache


@pytest.mark.parametrize('had_jtc_path', (False, True))
def test_sdk_validation_and_restore_do_not_record_or_cache_stale_jtc_path(had_jtc_path):
    h = recording_harness() if had_jtc_path else Harness()
    if had_jtc_path:
        h._mark_reuse_execution()
    else:
        h.transport_path_reuse_enabled = True
        h.transport_path_cache = []
        h.pick_cross_area = True
    h.transport_sdk_candidate = True
    h.arm_joint_names = ['j1', 'j2']
    h.transport_joint_limits = {'j1': (-2., 2., 1.), 'j2': (-2., 2., 1.)}
    h.transport_seed = (0., 0.)
    h._transport_validate_next = lambda: h.calls.append('next')
    h._sdk_validate_trajectory(trajectory())
    assert h.transport_cache_source is None and not h.transport_validated_poses
    # Exercise the real shared geometry callback used by SDK validation.
    h.transport_start_xyz = np.array([0., 0., .6])
    h.transport_end = np.array([.2, .2, .6])
    h.transport_start_q = h.transport_end_q = DOWN.copy()
    h.transport_scene = None
    h.transport_clearance = .4
    h.transport_safe_z = h.transport_high_z = .6
    h.transport_descent_time = None
    h._transport_ceiling = lambda: .8
    h._transport_pose = lambda _: (h.transport_end.copy(), DOWN.copy())
    h._fault = lambda message: pytest.fail(message)
    h._transport_geometry_checked(NS(result=lambda: None))
    assert not h.transport_validated_poses
    # Use the real restoration method: it clears the flag before final FK.
    h.latest_joint_positions = h.sdk_previous = (0., 0.)
    h._transport_result = lambda _: h._remember_overhead_path()
    h._sdk_restored()
    assert not h.transport_sdk_candidate
    assert not h.transport_path_cache


def test_new_jtc_path_can_record_after_sdk_invalidation():
    h = recording_harness()
    h._begin_reuse_recording()  # SDK invalidates the prior JTC path.
    h._record_reuse_pose(1., np.array([0., .1, .6]), DOWN)
    assert not h.transport_validated_poses
    h.operation_id += 1
    h.transport_trajectory = trajectory()
    h._begin_reuse_recording(h.transport_trajectory)
    for i in range(7):
        h._record_reuse_pose(float(i), np.array([0., i*.1, .6]), DOWN)
    h._mark_reuse_execution()
    h._remember_overhead_path()
    assert len(h.transport_path_cache) == 1
