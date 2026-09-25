"""Offline cuMotion/Cartesian joins: no service connects to a robot."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from safe_servo_visualization.transport_alternatives import join_trajectories
from safe_servo_visualization.smooth_transport import smooth_and_sample
from test_transport_alternatives import Harness, Future, trajectory
from test_direct_pickup_lift import Harness as ClearanceHarness
from test_descent_baseline import make_harness


def test_corner_keeps_nonzero_shared_velocity_after_checked_retiming():
    a = trajectory((0., 0.), (.1, 0.))
    b = trajectory((0., .1), (.1, .1), ('j2', 'j1'))
    joined = join_trajectories([a, b], ['j1', 'j2'], blend=True)
    limits = {n: (-6., 6., 1.) for n in joined.joint_names}
    checks, duration, _ = smooth_and_sample(joined, limits, .5, .5, 5., .5)
    assert np.linalg.norm(joined.points[1].velocities) > 1e-4
    assert list(joined.points[0].velocities) == [0., 0.]
    assert list(joined.points[-1].velocities) == [0., 0.]
    assert list(joined.points[1].positions) == [.1, 0.]
    assert duration > 0 and len(checks) > 3
    assert list(a.points[-1].velocities) == [.1, .1]
    assert list(b.joint_names) == ['j2', 'j1']


@pytest.mark.parametrize('end', [(0., 0.), (.1, .2)])
def test_unblendable_reversal_or_stationary_leg_is_rejected(end):
    with pytest.raises(ValueError, match='continuous seam'):
        join_trajectories([trajectory(), trajectory((.1, .2), end)],
                          ['j1', 'j2'], blend=True)


@pytest.mark.parametrize('pick,context', [(False, 'pallet'), (False, 'staging_store'),
                                         (True, 'known_pick')])
def test_all_clearance_legs_are_queued_before_execution(pick, context):
    q = np.array([1., 0., 0., 0.])
    h = ClearanceHarness(transport_is_pick=pick, transport_is_return=False,
        transport_moveit_pipeline_id='isaac_ros_cumotion',
        transport_seed=(0.,)*6, transport_scene=None, transport_route_generation=0,
        transport_target={'transfer_context': context}, direct_moveit_active=True)
    h._transport_ceiling = lambda: .9
    h.get_logger = lambda: Mock()
    h._alternative_next = Mock()
    h._begin_clearance_transfer(np.array([.2, .3, .4]), q, np.array([.5, .6, .4]), q, .5)
    assert [kind for kind, _, _ in h.alternative_segments] == ['cartesian', 'moveit', 'cartesian']
    assert h.clearance_phase == 'continuous'
    assert h.transport_seed == (0.,)*6
    assert h.transport_end == pytest.approx([.5, .6, .4])
    assert not h._advance_clearance_phase(np.zeros(3), q)
    h._alternative_next.assert_called_once()


def test_moveit_endpoint_seeds_cartesian_and_only_complete_route_is_validated(tmp_path):
    h = Harness(tmp_path)
    h.direct_moveit_active = True
    h.clearance_phase = 'continuous'
    h.clearance_has_descent = True
    h.state = h.TRANSPORT_PLANNING
    h.alternative_parts, h.alternative_part_kinds = [], []
    h.alternative_seed = (0., 0.)
    h.alternative_current_xyz = h.transport_start_xyz
    h.alternative_current_q = h.transport_start_q
    h.alternative_active_segment = ('moveit', np.array([.3, -.6, .65]), h.transport_end_q)
    h.alternative_destination = h.alternative_active_segment[1:]
    h.alternative_segments = [('cartesian', h.transport_end, h.transport_end_q)]
    moveit = trajectory()
    h._alternative_moveit_received(Future(NS(motion_plan_response=NS(
        error_code=NS(val=1), trajectory=NS(joint_trajectory=moveit)))))
    assert not h.validated
    _, fk = h.fk_calls.pop()
    fk(Future(h.alternative_destination))
    req, future = h.transport_cartesian.calls[-1]
    assert list(req.start_state.joint_state.position) == [.1, .2]
    assert req.avoid_collisions
    assert not h.validated
    future.value = NS(error_code=NS(val=1), fraction=1.,
                      solution=NS(joint_trajectory=trajectory((.1, .2), (.3, .4))))
    future.callback(future)
    assert not h.validated
    _, fk = h.fk_calls.pop()
    fk(Future(h.alternative_destination))
    assert not h.fault
    assert len(h.validated) == 1
    points = h.validated[0].solution.joint_trajectory.points
    assert len(points) == 3 and np.linalg.norm(points[1].velocities) > 0
    assert h.clearance_descent_index == 1
    assert h.transport_seed == (0., 0.)


@pytest.mark.parametrize('pick,context', [(False, 'staging_store'), (False, 'pallet'),
                                         (True, 'known_pick')])
def test_buffer_routes_do_not_insert_observation_waypoint(pick, context):
    q = np.array([1., 0., 0., 0.])
    h = ClearanceHarness(transport_is_pick=pick, transport_is_return=False,
        transport_moveit_pipeline_id='isaac_ros_cumotion',
        transport_seed=(0.,)*6, transport_scene=None, transport_route_generation=0,
        transport_target={'transfer_context': context}, direct_moveit_active=True,
        transport_via_observation=True, pick_cross_area=True)
    h._transport_ceiling = lambda: .9
    h.get_logger = lambda: Mock()
    h._alternative_next = Mock()
    h._transport_fk_request = Mock()
    h._begin_clearance_transfer(np.array([.2, .3, .4]), q, np.array([.5, .6, .4]), q, .5)
    assert [kind for kind, _, _ in h.alternative_segments] == ['cartesian', 'moveit', 'cartesian']
    assert h.alternative_segments[1][1] == pytest.approx(h.clearance_destination)
    h._transport_fk_request.assert_not_called()
    h._alternative_next.assert_called_once()


def test_continuous_baseline_does_not_move_descent_trigger_to_start(monkeypatch):
    h = make_harness()
    h.clearance_phase = 'continuous'
    h.transport_descent_time = 2.5
    monkeypatch.setattr('safe_servo_visualization.clearance_transfer.time.monotonic', lambda: 10.)
    assert not h._wait_descent_baseline()
    assert h.transport_descent_time == 2.5
    assert h.transport_contact_baseline == 0.


@pytest.mark.parametrize('xyz,t,rejected', [
    ([.5, .6, .54], 2.1, False),
    ([.52, .6, .54], 2.1, True),  # Turn exceeds the clearance margin.
    ([.5, .6, .50], 3., False),
    ([.502, .6, .50], 3., True),  # Straight descent cannot drift sideways.
    ([.2, .3, .4], 1., False),   # No extra global TCP floor on free transfer.
])
def test_continuous_geometry_checks_blend_and_strict_descent(xyz, t, rejected):
    q = np.array([1., 0., 0., 0.])
    h = ClearanceHarness(transport_descent_time=2., clearance_descent_seam_time=2.1,
        clearance_descent_blend_end=2.2,
        clearance_previous_descent_z=float('inf'), transport_radius=.04,
        clearance_destination=np.array([.5, .6, .54]), transport_end=np.array([.5, .6, .4]),
        transport_end_q=q, direct_lift_z=None, direct_lift_verified=False,
        transport_safe_z=.53, transport_is_return=False)
    assert bool(h._continuous_clearance_geometry_issue(np.array(xyz), q, t)) == rejected


def test_endpoint_padding_is_removed_before_join_indexing(tmp_path):
    import copy
    from rclpy.duration import Duration
    h = Harness(tmp_path)
    h.clearance_phase = 'continuous'
    h.alternative_parts, h.alternative_part_kinds = [], []
    h.alternative_seed = (0., 0.)
    padded = trajectory()
    padded.points.insert(1, copy.deepcopy(padded.points[0]))
    padded.points.append(copy.deepcopy(padded.points[-1]))
    for i, point in enumerate(padded.points):
        point.time_from_start = Duration(seconds=float(i)).to_msg()
    h._alternative_segment_ready(padded)
    assert len(h.alternative_parts[0].points) == 2
    assert h.alternative_parts[0].points[0].time_from_start.sec == 0
    assert h.alternative_parts[0].points[-1].time_from_start.sec == 1
    assert len(padded.points) == 4
    assert h.alternative_seed == (.1, .2)


def test_failed_final_cartesian_never_validates_or_executes_prefix(tmp_path):
    h = Harness(tmp_path)
    h.direct_moveit_active = True
    h.clearance_phase = 'continuous'
    h.state = h.TRANSPORT_PLANNING
    h.alternative_parts = [trajectory()]
    h._transport_execute = Mock()
    h._alternative_cartesian_received(Future(NS(error_code=NS(val=1), fraction=.8)))
    assert h.state == 'FAULT'
    assert not h.validated
    h._transport_execute.assert_not_called()


def test_continuous_return_validates_lift_and_exact_joint_goal_together(tmp_path):
    h = Harness(tmp_path)
    h.direct_moveit_active = h.transport_is_return = True
    h.clearance_phase = 'continuous'
    h.clearance_has_descent = False
    h.state = h.TRANSPORT_PLANNING
    h.alternative_parts = [trajectory()]
    h.alternative_part_kinds = ['cartesian']
    h.alternative_seed = (.1, .2)
    h.alternative_active_segment = ('moveit', h.transport_end, h.transport_end_q)
    h.alternative_destination = h.alternative_active_segment[1:]
    h.alternative_segments = []
    h._alternative_moveit_received(Future(NS(motion_plan_response=NS(
        error_code=NS(val=1), trajectory=NS(joint_trajectory=trajectory((.1, .2), (.4, .5)))))))
    assert not h.validated
    _, fk = h.fk_calls.pop()
    fk(Future(h.alternative_destination))
    assert not h.fault
    assert len(h.validated) == 1
    points = h.validated[0].solution.joint_trajectory.points
    assert list(points[0].positions) == [0., 0.]
    assert list(points[-1].positions) == [.4, .5]
    assert h.clearance_descent_index is None


def test_force_trigger_uses_retimed_seam_not_original_planner_time():
    import copy
    from rclpy.duration import Duration
    from test_continuous_transport import planned_harness
    h, result = planned_harness()
    h.clearance_phase = 'continuous'
    h.clearance_descent_index = 2
    h.direct_transfer_max_joint_speed = .05
    a = trajectory(names=h.arm_joint_names)
    midpoint = copy.deepcopy(a.points[-1])
    midpoint.positions = [.05, .1]
    midpoint.time_from_start = Duration(seconds=.5).to_msg()
    a.points.insert(1, midpoint)
    b = trajectory((.1, .2), (.3, .4), h.arm_joint_names)
    result.solution.joint_trajectory = join_trajectories([a, b], h.arm_joint_names, blend=True)
    h._transport_planned(Future(result))
    assert h.state == h.TRANSPORT_VALIDATING
    points = h.transport_trajectory.points
    seconds = lambda p: p.time_from_start.sec+p.time_from_start.nanosec*1e-9
    assert h.transport_descent_time == seconds(points[1])
    assert h.transport_descent_time > .5
    assert h.clearance_descent_seam_time == seconds(points[2])
    assert h.clearance_descent_blend_end == seconds(points[3])
    assert np.linalg.norm(points[2].velocities) > 0


def geometry_harness():
    return ClearanceHarness(
        transport_descent_time=2., clearance_descent_seam_time=2.1,
        clearance_descent_blend_end=2.2, clearance_previous_descent_z=float('inf'),
        transport_radius=.04, clearance_destination=np.array([.5, .6, .54]),
        transport_end=np.array([.5, .6, .4]), transport_end_q=np.array([1., 0., 0., 0.]),
        direct_lift_z=None, direct_lift_verified=False, transport_safe_z=.53,
        transport_is_return=False)


def test_sparse_seam_neighbors_do_not_expand_the_spatial_blend_region():
    h = geometry_harness()
    # Both neighbors are 20 mm from the seam, outside the unchanged 10 mm
    # region. They belong to free transfer and straight descent respectively.
    for t, xyz in [(2., [.48, .6, .54]), (2.08, [.495, .6, .54]),
                   (2.1, [.5, .6, .54]), (2.15, [.5, .6, .52]),
                   (2.3, [.5, .6, .50])]:
        assert h._continuous_clearance_geometry_issue(np.array(xyz), h.transport_end_q, t) is None


@pytest.mark.parametrize('xyz,t', [
    ([.502, .6, .52], 2.15),    # Outgoing samples outside the sphere must be vertical.
    ([.5, .6, .56], 2.15),      # Cannot exit upward instead of descending.
])
def test_outside_blend_region_keeps_phase_constraints(xyz, t):
    h = geometry_harness()
    issue = h._continuous_clearance_geometry_issue(np.array(xyz), h.transport_end_q, t)
    assert issue
    assert 't=' in issue and 'xyz=' in issue and 'distance=' in issue


def test_descent_cannot_reenter_blend_region_after_exiting():
    h = geometry_harness()
    assert h._continuous_clearance_geometry_issue(
        np.array([.5, .6, .52]), h.transport_end_q, 2.15) is None
    assert h._continuous_clearance_geometry_issue(
        np.array([.505, .6, .535]), h.transport_end_q, 2.16)


def test_transfer_accepts_previous_clearance_failure_point():
    h = geometry_harness()
    h.transport_descent_time = 12.
    h.transport_safe_z = .588609
    h.clearance_destination = np.array([.189098, -.363355, .598609])
    xyz = np.array([.500911, .211424, .586831])
    assert h._continuous_clearance_geometry_issue(xyz, h.transport_end_q, 10.0765) is None


@pytest.mark.parametrize('source,destination', [('pallet', 'pallet'),
    ('buffer', 'pallet'), ('pallet', 'buffer'), ('buffer', 'buffer')])
def test_pallet_clearance_lowered_100mm_without_lowering_slot(source, destination):
    q = np.array([1., 0., 0., 0.])
    h = ClearanceHarness(transport_is_pick=False,
        transport_moveit_pipeline_id='isaac_ros_cumotion',
        active_pickup_snapshot={'pickup_source': source},
        transport_target={'transfer_context': 'staging_store' if destination == 'buffer' else 'pallet'},
        transport_slot_clearance_z_m=0., transport_scene=None, transport_seed=(0.,)*6)
    h._transport_ceiling = lambda: 1.
    h.get_logger = lambda: Mock()
    h._plan_clearance_phase = Mock()
    h._begin_clearance_transfer(np.array([.2, .3, .3]), q,
                                np.array([.5, .6, .3]), q, .5)
    assert h.clearance_source_z == pytest.approx(.54 if source == 'buffer' else .44)
    assert h.clearance_destination[2] == pytest.approx(.54 if destination == 'buffer' else .44)


@pytest.mark.parametrize('mode,source,kinds', [
    ('joint_direct', None, ['moveit']),
    (None, 'pallet', ['cartesian', 'cartesian']),
    (None, 'buffer', ['cartesian', 'cartesian']),
])
def test_simple_pick_routes_skip_cumotion_search(mode, source, kinds):
    q = np.array([1., 0., 0., 0.])
    h = ClearanceHarness(transport_is_pick=True, transport_is_return=False,
        transport_moveit_pipeline_id='isaac_ros_cumotion',
        transport_seed=(0.,)*6, transport_scene=None, transport_route_generation=0,
        transport_target={'transfer_context': 'known_pick', 'planned_pregrasp': {
            'approach_mode': mode, 'pickup_source': source}}, direct_moveit_active=True)
    h._transport_ceiling = lambda: 1.
    h.get_logger = lambda: Mock()
    h._alternative_next = Mock()
    h._begin_clearance_transfer(np.array([.2, .3, .6]), q, np.array([.5, .6, .4]), q, .5)
    assert [kind for kind, _, _ in h.alternative_segments] == kinds
    assert h.direct_lift_z is None
    if source:
        assert h.alternative_segments[0][1] == pytest.approx([.5, .6, .6])
        assert h.alternative_segments[1][1] == pytest.approx([.5, .6, .4])
    else:
        assert not h.clearance_has_descent
