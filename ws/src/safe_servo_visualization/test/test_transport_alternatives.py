"""Planning-only route alternatives: no robot or live ROS services."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from safe_servo_visualization.continuous_transport import ContinuousTransport
from safe_servo_visualization.transport_alternatives import join_trajectories, observation_transfer_waypoint


class Future:
    def __init__(self, value=None):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callback = callback


class Client:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, req):
        future = Future()
        self.calls.append((req, future))
        return future


def trajectory(a=(0., 0.), b=(.1, .2), names=('j1', 'j2')):
    msg = JointTrajectory(joint_names=list(names))
    for t, p in ((0., a), (1., b)):
        msg.points.append(JointTrajectoryPoint(
            positions=list(p), velocities=[.1, .1], accelerations=[0., 0.],
            time_from_start=Duration(seconds=t).to_msg()))
    return msg


class Harness(ContinuousTransport):
    def __init__(self, tmp_path):
        self.operation_id = 7
        self.state = self.TRANSPORT_DIAGNOSING
        self.transport_alternatives_enabled = True
        self.transport_lower_waypoint_search_enabled = False
        # Legacy fallback tests explicitly opt in; production defaults to
        # expanded Cartesian search with no sampling-planner fallback.
        self.transport_moveit_fallback_enabled = True
        self.transport_expanded_waypoints_enabled = False
        self.transport_observation_y_offset_mm = 200.0
        self.transport_route_attempt = self.transport_route_generation = 0
        self.transport_is_pick = self.transport_is_return = False
        self.direct_transfer_motion_started = False
        self.transfer_goal_handle = None
        self.transport_scene = dict(attached_item_id='box')
        self.transport_seed = (0., 0.)
        self.arm_joint_names = ['j1', 'j2']
        self.transport_joint_limits = {n: (-6., 6., 2.) for n in self.arm_joint_names}
        path = tmp_path / 'observation.yaml'
        path.write_text('waypoints:\n  observation:\n    joint_names: [j1, j2]\n    positions_rad: [0.2, 0.3]\n')
        self.return_waypoint_file = str(path)
        self.transport_start_xyz = np.array([.2, .5, .15])
        self.transport_start_q = np.array([1., 0., 0., 0.])
        self.transport_end = np.array([.3, -.6, .2])
        self.transport_end_q = np.array([1., 0., 0., 0.])
        self.transport_high_z, self.transport_safe_z = .65, .61
        self.servo_bounds_mm = [0, 0, 0, 0, -100, 800]
        self.motion_speed_percent = 50.
        self.planning_group, self.ik_link_name = 'uf850', 'link_tcp'
        self.transport_motion_plan, self.transport_cartesian = Client(), Client()
        self.compute_ik_client = Client()
        self.fk_calls = []
        self._transport_fk_request = lambda joints, callback: self.fk_calls.append((joints, callback))
        self._transport_pose = lambda result: result
        self.logger = Mock()
        self.fault = ''
        self.validated = []
        self._transport_planned = lambda future: self.validated.append(future.result())

    def get_logger(self):
        return self.logger

    def _fault(self, reason):
        self.fault, self.state = reason, 'FAULT'


def test_slot_yaw_flip_after_five_seconds_restarts_checked_path(tmp_path):
    h = Harness(tmp_path)
    h.transport_slot_yaw_deadline = 105.
    h.alternative_rail_index = 1
    h.alternative_yaw_direction = -1
    h.alternative_grid_deadline = 105.
    h.transport_target = dict(transfer_context='staging_store',
        staging_yaw180_pose=[.32, -.58, .2, 0., 1., 0., 0.])
    assert not h._try_slot_yaw_flip(104.99)
    generation = h.transport_route_generation
    assert h._try_slot_yaw_flip(105.)
    assert h.transport_slot_yaw_flipped
    assert np.allclose(h.transport_end, [.32, -.58, .2])
    assert np.allclose(h.transport_start_q, [1., 0., 0., 0.])
    assert h.transport_route_generation > generation
    assert h._route_candidates()[0][2] == 'elbow'
    assert not h._try_slot_yaw_flip(110.)


@pytest.mark.parametrize('state', ['TRANSPORT_EXECUTING', 'TRANSPORT_VALIDATING'])
def test_slot_yaw_does_not_interrupt_motion_or_validated_plan(tmp_path, state):
    h = Harness(tmp_path)
    h.state = getattr(h, state)
    h.transport_slot_yaw_deadline = 0.
    h.transport_target = dict(transfer_context='staging_store',
        staging_yaw180_pose=[.32, -.58, .2, 0., 1., 0., 0.])
    assert not h._try_slot_yaw_flip(105.)


def test_lower_height_search_covers_full_100mm_without_changing_clearance(tmp_path):
    h = Harness(tmp_path)
    h.transport_lower_waypoint_search_enabled = True
    assert h._lower_waypoint_heights() == pytest.approx([.65-i*.01 for i in range(11)])
    assert h.transport_safe_z == .61
    h.transport_safe_z = .4
    assert h._lower_waypoint_heights() == pytest.approx([.65-i*.01 for i in range(11)])
    h.transport_safe_z = .645
    assert h._lower_waypoint_heights() == pytest.approx([.65-i*.01 for i in range(11)])
    assert h.transport_safe_z == .645


def test_lower_height_retry_precedes_next_xy_and_preserves_safe_plane(tmp_path):
    h = Harness(tmp_path)
    h.transport_lower_waypoint_search_enabled = True
    assert h._try_transport_alternative('IK')
    h.alternative_observation = np.array([.4, .2, .1])
    h._alternative_prepare = Mock()
    for height in [.65-i*.01 for i in range(1, 11)]:
        assert h._try_transport_alternative('IK')
        assert h.transport_high_z == pytest.approx(height)
        assert h.alternative_candidate_index == 0
        assert h.transport_safe_z == .61
        assert h.alternative_seed == h.transport_seed
    assert h._try_transport_alternative('IK')
    assert h.alternative_candidate_index == 1
    assert h.transport_high_z == .65
    assert not h.validated


def test_raised_pre_place_search_is_bounded_and_preserves_target(tmp_path):
    h = Harness(tmp_path)
    h.state = h.TRANSPORT_PLANNING
    h.transport_route_attempt = 2
    h.transport_target = dict(transfer_context='pallet', pre_place_tcp_xyz_m=[.3, -.6, .2])
    h.alternative_segments = []
    h.raised_pre_place_max_mm = 10.
    h.alternative_current_xyz = np.array([.3, -.6, .65])
    h.alternative_active_segment = ('cartesian', h.transport_end, h.transport_end_q)
    candidates = []
    h._alternative_next = lambda: candidates.append(h.alternative_segments.pop())
    assert h._try_raised_pre_place()
    assert h._try_raised_pre_place()
    assert not h._try_raised_pre_place()
    assert [p[1][2] for p in candidates] == pytest.approx([.205, .210])
    assert h.transport_target['pre_place_tcp_xyz_m'] == [.3, -.6, .2]
    assert not h.validated  # Search alone never authorizes execution.


def test_raised_blended_retry_keeps_overhead_route_and_original_seed(tmp_path):
    h = Harness(tmp_path)
    h.state = h.TRANSPORT_PLANNING
    h.transport_route_attempt = 1
    h.transport_target = dict(transfer_context='pallet', pre_place_tcp_xyz_m=[.3, -.6, .2])
    h.raised_pre_place_max_mm = 10.
    h.alternative_current_xyz = np.array([.35, .03, .65])
    h.alternative_current_q = h.transport_end_q
    h.alternative_seed = (.1, .2)
    points = [np.array([.35, -.6, .65]), np.array([.3, -.6, .65]), h.transport_end.copy()]
    h.alternative_segments = [('cartesian_blend', points, h.transport_end_q)]
    h._alternative_next()
    for expected_z in (.205, .210):
        assert h._try_raised_pre_place()
        kind, retried, _ = h.alternative_active_segment
        assert kind == 'cartesian_blend'
        np.testing.assert_allclose(retried[:-1], points[:-1])
        assert retried[-1][2] == pytest.approx(expected_z)
        req, _ = h.transport_cartesian.calls[-1]
        assert list(req.start_state.joint_state.position) == [.1, .2]
        assert req.avoid_collisions
        assert len(req.waypoints) > 2
        assert not h.validated
    assert not h._try_raised_pre_place()
    assert points[-1][2] == .2
    assert not h.fault


@pytest.mark.parametrize('fraction,expected', [(0., False), (.4, False), (.6, False), (.8, True)])
def test_only_vertical_suffix_qualifies_for_height_retry(tmp_path, fraction, expected):
    from safe_servo_visualization.transport_alternatives import pose_message
    h = Harness(tmp_path)
    h.alternative_current_xyz = np.array([.35, .03, .65])
    points = [[.35, -.3, .65], [.35, -.6, .65], [.3, -.6, .65],
              [.3, -.6, .4], [.3, -.6, .2]]
    h.transport_cartesian_request = NS(
        waypoints=[pose_message(np.array(p), h.transport_end_q) for p in points])
    result = NS(error_code=NS(val=1), fraction=fraction)
    assert h._partial_is_final_descent(result) == expected


def test_high_transfer_failure_skips_all_height_retries(tmp_path):
    h = Harness(tmp_path)
    h._partial_is_final_descent = lambda result: False
    h._try_raised_pre_place = Mock()
    h._diagnose_partial_path = Mock()
    result = NS(error_code=NS(val=1), fraction=.45)
    h._alternative_cartesian_received(Future(result))
    h._try_raised_pre_place.assert_not_called()
    h._diagnose_partial_path.assert_called_once_with(result, alternative=True)


def test_raised_single_point_retry_cannot_cut_diagonally(tmp_path):
    h = Harness(tmp_path)
    h.state = h.TRANSPORT_PLANNING
    h.transport_route_attempt = 1
    h.transport_target = dict(transfer_context='pallet', pre_place_tcp_xyz_m=[.3, -.6, .2])
    h.raised_pre_place_max_mm = 10.
    h.alternative_segments = []
    h.alternative_current_xyz = np.array([.35, .03, .65])
    h.alternative_active_segment = ('cartesian', h.transport_end, h.transport_end_q)
    assert not h._try_raised_pre_place()
    assert not h.transport_cartesian.calls


@pytest.mark.parametrize('blocked', ['staging', 'pick', 'return', 'executing', 'nonfinal', 'disabled'])
def test_raised_search_restricted_to_planning_final_pallet_descent(tmp_path, blocked):
    h = Harness(tmp_path)
    h.state = h.TRANSPORT_PLANNING
    h.transport_route_attempt = 2
    h.transport_target = dict(transfer_context='pallet', pre_place_tcp_xyz_m=[.3, -.6, .2])
    h.alternative_segments = []
    h.raised_pre_place_max_mm = 30.
    if blocked == 'staging': h.transport_target['transfer_context'] = 'staging_store'
    if blocked == 'pick': h.transport_is_pick = True
    if blocked == 'return': h.transport_is_return = True
    if blocked == 'executing': h.direct_transfer_motion_started = True
    if blocked == 'nonfinal': h.alternative_segments = [('cartesian', h.transport_end, h.transport_end_q)]
    if blocked == 'disabled': h.raised_pre_place_max_mm = 0.
    assert not h._try_raised_pre_place()


def test_observation_candidate_uses_xy_at_clearance_and_preserves_box(tmp_path):
    h = Harness(tmp_path)
    assert h._try_transport_alternative('collision')
    assert h.transport_route_attempt == 1 and not h.transport_cartesian.calls
    joints, callback = h.fk_calls.pop()
    assert joints == (.2, .3)
    callback(Future((np.array([.4, .2, .05]), h.transport_end_q)))
    req, _ = h.transport_cartesian.calls[-1]
    assert req.avoid_collisions and req.start_state.is_diff
    assert list(req.start_state.joint_state.position) == [0., 0.]
    assert req.waypoints[0].position.x == .2
    assert req.waypoints[0].position.z > .15  # first: lift at source XY
    assert req.waypoints[-1].position.z == h.transport_end[2]
    assert not h.alternative_segments  # all same-orientation translations planned together
    assert any(abs(p.position.x-.25) < 1e-8 and abs(p.position.z-.65) < 1e-8
               for p in req.waypoints)
    assert not h.validated  # never executes/plans acceptance per segment


@pytest.mark.parametrize('offset,expected_y', [(200., .4), (0., .2), (-100., .1)])
def test_observation_offset_only_changes_base_y_at_safe_height(offset, expected_y):
    saved = np.array([.4, .2, .05])
    result = observation_transfer_waypoint(saved, .65, offset)
    assert np.allclose(result, [.4, expected_y, .65])
    assert np.allclose(saved, [.4, .2, .05])  # taught pose remains unchanged


@pytest.mark.parametrize('offset', [math.nan, math.inf, -math.inf])
def test_invalid_observation_offset_rejected(offset):
    with pytest.raises(ValueError):
        observation_transfer_waypoint([.4, .2, .05], .65, offset)


@pytest.mark.parametrize('destination,source,expected', [
    ('staging_store', {'pickup_source': 'pallet'}, True),
    ('pallet', {'pickup_source': 'buffer'}, True),
    ('pallet', {'staging_slot': 0}, True),
    ('pallet', {'pickup_source': 'incoming'}, False),
    ('pallet', {'pickup_source': 'pallet'}, False),
    ('pallet', None, False),
])
def test_buffer_route_selection(destination, source, expected):
    assert ContinuousTransport._uses_buffer_route(
        {'transfer_context': destination}, source) is expected


def test_buffer_transfer_plans_observation_route_before_any_direct_path(tmp_path):
    h = Harness(tmp_path)
    h.state = h.TRANSPORT_PLANNING
    h.transport_via_observation = True
    h.transport_target = dict(pre_place_tcp_xyz_m=list(h.transport_end),
                             transfer_tcp_quaternion_xyzw=list(h.transport_end_q),
                             transport_corner_clearance_z_m=.45)
    h.transport_scene.update(attached_item_size_m=[.2, .15, .13],
                            attached_item_center_in_tcp_m=[0., 0., .065],
                            attached_item_orientation_in_tcp_xyzw=[0., 0., 0., 1.])
    h.transport_radius = .04
    h._transport_start_fk(Future((h.transport_start_xyz, h.transport_start_q)))
    assert h.transport_route_attempt == 1
    assert h.fk_calls  # obtaining taught observation XY
    assert not h.transport_cartesian.calls  # direct source-to-destination path skipped
    assert not h.transport_motion_plan.calls
    h._alternative_failed('observation-side route incomplete')
    assert h.transport_route_attempt == 2  # MoveIt remains the last fallback


def test_disabling_fallback_does_not_skip_required_buffer_waypoint(tmp_path):
    h = Harness(tmp_path)
    h.state = h.TRANSPORT_PLANNING
    h.transport_alternatives_enabled = False
    assert h._try_transport_alternative('buffer route', observation_first=True)
    assert h.transport_route_attempt == 1
    h._alternative_failed('waypoint route incomplete')
    assert h.state == 'FAULT'
    assert not h.transport_motion_plan.calls


def test_full_observation_route_is_passed_to_validation_only_after_descent(tmp_path):
    h = Harness(tmp_path)
    h._try_transport_alternative('collision')
    h.fk_calls.pop()[1](Future((np.array([.4, .2, .05]), h.transport_end_q)))
    for i in range(1):
        req, future = h.transport_cartesian.calls[-1]
        future.value = NS(error_code=NS(val=1), fraction=1.,
                          solution=NS(joint_trajectory=trajectory(
                              tuple(req.start_state.joint_state.position), ((i+1)*.1, (i+1)*.1))))
        future.callback(future)
        assert not h.validated
        h.fk_calls.pop()[1](Future(h.alternative_destination))
    assert len(h.validated) == 1
    assert h.validated[0].fraction == 1.
    assert len(h.validated[0].solution.joint_trajectory.points) == 2
    assert not h.transport_motion_plan.calls


def test_observation_failure_tries_moveit_with_lift_planned_first(tmp_path):
    h = Harness(tmp_path)
    h._try_transport_alternative('collision')
    h._alternative_failed('observation leg incomplete')
    assert h.transport_route_attempt == 2
    assert h.alternative_segments[0][0] == 'moveit'
    assert h.transport_cartesian.calls  # lift planned; no execution
    assert not h.transport_motion_plan.calls
    h._alternative_failed('lift failed again')
    assert h.state == 'FAULT'
    assert not h.validated


def test_missing_observation_file_still_allows_moveit_candidate(tmp_path):
    h = Harness(tmp_path)
    h.return_waypoint_file += '.missing'
    h._try_transport_alternative('partial path')
    assert h.transport_route_attempt == 2
    assert h.transport_cartesian.calls


def test_primary_moveit_constrains_tilt_and_keeps_exact_goal(tmp_path):
    h = Harness(tmp_path)
    h.transport_moveit_primary_enabled = True
    h.transport_moveit_fallback_enabled = False
    h.alternative_seed = h.transport_seed
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    plan = h.transport_motion_plan.calls[-1][0].motion_plan_request
    c = plan.path_constraints.orientation_constraints[0]
    assert c.absolute_x_axis_tolerance == pytest.approx(math.pi / 6)
    assert c.absolute_y_axis_tolerance == pytest.approx(math.pi / 6)
    assert c.absolute_z_axis_tolerance == math.pi
    assert plan.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance == .002


def test_primary_retries_original_seed_and_stops_after_three(tmp_path):
    h = Harness(tmp_path)
    h.transport_moveit_primary_enabled = True
    h.transport_target = dict(pre_place_tcp_xyz_m=list(h.transport_end))
    h._alternative_prepare = Mock()
    for attempt in range(1, 4):
        h.alternative_seed = (1., 2.)
        h.alternative_parts = [trajectory()]
        assert h._try_transport_alternative('descent rejected')
        assert h.primary_moveit_attempt == attempt
        assert h.alternative_seed == h.transport_seed
        assert h.alternative_parts == []
    assert h._try_transport_alternative('descent rejected')
    assert h.state == 'FAULT'
    assert h._alternative_prepare.call_count == 3


def test_primary_does_not_retry_after_execution_starts(tmp_path):
    h = Harness(tmp_path)
    h.transport_moveit_primary_enabled = True
    h.direct_transfer_motion_started = True
    assert not h._try_transport_alternative('execution failed')


def test_moveit_request_is_plan_only_with_free_overhead_orientation(tmp_path):
    h = Harness(tmp_path)
    h.alternative_seed = (.1, .2)
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    req, _ = h.transport_motion_plan.calls[-1]
    plan = req.motion_plan_request
    assert plan.start_state.is_diff  # attached object retained
    assert list(plan.start_state.joint_state.position) == [.1, .2]
    assert plan.pipeline_id == 'ompl'
    assert plan.planner_id == 'RRTstar'
    assert plan.allowed_planning_time == 5.
    assert plan.max_velocity_scaling_factor == .5
    box = plan.path_constraints.position_constraints[0]
    assert box.constraint_region.primitives[0].dimensions[2] == pytest.approx(.19)
    assert box.constraint_region.primitive_poses[0].position.z == pytest.approx(.705)
    assert not plan.path_constraints.orientation_constraints
    assert plan.goal_constraints[0].orientation_constraints[0].absolute_x_axis_tolerance == .002
    assert plan.goal_constraints[0].orientation_constraints[0].absolute_y_axis_tolerance == .002
    assert plan.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance == .002
    assert not h.validated


def test_cumotion_request_has_complete_start_and_no_path_constraints(tmp_path):
    h = Harness(tmp_path)
    h.transport_moveit_pipeline_id = 'isaac_ros_cumotion'
    h.transport_moveit_planner_id = 'cuMotion'
    h.transport_moveit_primary_enabled = True
    h.direct_moveit_active = True
    h.clearance_phase = 'transfer'
    h.transport_moveit_clearance_constraint_enabled = True
    h.alternative_seed = (.1, .2)
    h.state_validity_client = Client()
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    assert not h.transport_motion_plan.calls
    ik, pending = h.compute_ik_client.calls[-1]
    assert ik.ik_request.avoid_collisions
    assert ik.ik_request.robot_state.is_diff
    assert list(ik.ik_request.robot_state.joint_state.position) == [.1, .2]
    pending.value = NS(
        error_code=NS(val=1),
        solution=NS(joint_state=NS(name=['j2', 'j1'], position=[.4, .3])))
    pending.callback(pending)
    plan = h.transport_motion_plan.calls[-1][0].motion_plan_request
    assert plan.pipeline_id == 'isaac_ros_cumotion'
    assert plan.planner_id == 'cuMotion'
    assert not plan.start_state.is_diff
    assert list(plan.start_state.joint_state.position) == [.1, .2]
    assert not plan.path_constraints.position_constraints
    assert not plan.path_constraints.orientation_constraints
    assert [c.position for c in plan.goal_constraints[0].joint_constraints] == [.3, .4]
    assert not plan.goal_constraints[0].position_constraints
    assert not plan.goal_constraints[0].orientation_constraints
    assert not h.state_validity_client.calls
    assert not h.validated


def test_cumotion_endpoint_ik_failure_does_not_submit_planner(tmp_path):
    h = Harness(tmp_path)
    h.transport_moveit_pipeline_id = 'isaac_ros_cumotion'
    h.transport_moveit_planner_id = 'cuMotion'
    h.transport_moveit_primary_enabled = True
    h.direct_moveit_active = True
    h.alternative_seed = (.1, .2)
    h._alternative_failed = Mock()
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    _, pending = h.compute_ik_client.calls[-1]
    pending.value = NS(error_code=NS(val=-31))
    pending.callback(pending)
    assert not h.transport_motion_plan.calls
    h._alternative_failed.assert_called_once_with(
        'MoveIt IK for cuMotion endpoint failed (code=-31)')


def test_cumotion_endpoint_uses_nearest_bounded_equivalent_angles(tmp_path):
    h = Harness(tmp_path)
    h.arm_joint_names = ['joint4', 'joint6']
    h.transport_joint_limits = {
        name: (-2*math.pi, 2*math.pi, 2.) for name in h.arm_joint_names}
    h.transport_moveit_pipeline_id = 'isaac_ros_cumotion'
    h.transport_moveit_planner_id = 'cuMotion'
    h.transport_moveit_primary_enabled = True
    h.direct_moveit_active = True
    h.alternative_seed = (-3.0, -1.38)
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    _, pending = h.compute_ik_client.calls[-1]
    pending.value = NS(error_code=NS(val=1), solution=NS(
        joint_state=NS(
            name=['joint6', 'joint4'], position=[4.73, -2*math.pi+.01])))
    pending.callback(pending)

    constraints = h.transport_motion_plan.calls[-1][0].motion_plan_request \
        .goal_constraints[0].joint_constraints
    assert [constraint.position for constraint in constraints] == pytest.approx(
        [.01, 4.73-2*math.pi])


def test_cumotion_endpoint_ik_uses_narrowed_planner_limits(tmp_path):
    h = Harness(tmp_path)
    h.arm_joint_names = ['joint3', 'joint5']
    h.transport_joint_limits = {
        'joint3': (-math.pi, math.pi, 2.),
        'joint5': (-math.pi, math.pi, 2.),
    }
    h.transport_cumotion_joint_limits = {
        'joint3': (math.radians(-130), math.pi, 2.),
        'joint5': (-math.pi, math.radians(40), 2.),
    }
    h.transport_moveit_pipeline_id = 'isaac_ros_cumotion'
    h.transport_moveit_planner_id = 'cuMotion'
    h.transport_moveit_primary_enabled = True
    h.direct_moveit_active = True
    h.alternative_seed = (-1., -1.)
    h._alternative_failed = Mock()
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    request, pending = h.compute_ik_client.calls[-1]
    constraints = {c.joint_name: c for c in
                   request.ik_request.constraints.joint_constraints}
    assert set(constraints) == {'joint3', 'joint5'}
    for name, (lower, upper, _) in h.transport_cumotion_joint_limits.items():
        constraint = constraints[name]
        assert constraint.position-constraint.tolerance_below == pytest.approx(lower)
        assert constraint.position+constraint.tolerance_above == pytest.approx(upper)

    # Even if an IK plugin ignores constraints, the returned goal is rejected
    # locally and never reaches cuMotion.
    pending.value = NS(error_code=NS(val=1), solution=NS(
        joint_state=NS(name=['joint3', 'joint5'], position=[-1., math.radians(69)])))
    pending.callback(pending)
    assert not h.transport_motion_plan.calls
    assert 'joint5' in h._alternative_failed.call_args.args[0]


def test_first_direct_cartesian_segment_initializes_timing_source(tmp_path):
    h = Harness(tmp_path)
    h.alternative_parts = []
    h.alternative_seed = (0., 0.)
    h.alternative_active_segment = ('cartesian', np.array([.3, -.6, .65]), h.transport_end_q)
    h._transport_fk_request = Mock()
    h._alternative_segment_ready(trajectory(a=(0., 0.), b=(.1, .2)))
    assert h.alternative_part_kinds == ['cartesian']
    h._transport_fk_request.assert_called_once()


def test_cumotion_return_uses_saved_joint_goal(tmp_path):
    h = Harness(tmp_path)
    h.transport_moveit_pipeline_id = 'isaac_ros_cumotion'
    h.transport_moveit_planner_id = 'cuMotion'
    h.direct_moveit_active = h.transport_is_return = True
    h.return_goal_joints = (.4, .5)
    h.alternative_seed = (.1, .2)
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    plan = h.transport_motion_plan.calls[-1][0].motion_plan_request
    goal = plan.goal_constraints[0]
    assert [c.position for c in goal.joint_constraints] == [.4, .5]
    assert not goal.position_constraints
    assert not plan.path_constraints.orientation_constraints


@pytest.mark.parametrize('limit', [0., 950.])
def test_transfer_ceiling_is_independent_of_contact_servo_bounds(tmp_path, limit):
    h = Harness(tmp_path)
    h.transport_workspace_z_max_mm = limit
    h.alternative_seed = (.1, .2)
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    plan = h.transport_motion_plan.calls[-1][0].motion_plan_request
    box = plan.path_constraints.position_constraints[0].constraint_region
    height = box.primitives[0].dimensions[2]
    center = box.primitive_poses[0].position.z
    assert center-height/2 == pytest.approx(h.transport_safe_z)
    assert center+height/2 == pytest.approx(h.transport_safe_z+10. if limit == 0 else .95)
    assert h._transport_ceiling() == (math.inf if limit == 0 else .95)
    assert h.servo_bounds_mm[5] == 800
    assert plan.goal_constraints[0].orientation_constraints
    assert not h.validated  # planning alone never authorizes execution


def test_direct_moveit_has_no_waypoints_or_overhead_region(tmp_path):
    h = Harness(tmp_path)
    h.direct_moveit_active = True
    h.transport_moveit_fallback_enabled = False
    h.transport_moveit_primary_enabled = False
    h.alternative_seed = (.1, .2)
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    plan = h.transport_motion_plan.calls[-1][0].motion_plan_request
    assert not plan.path_constraints.position_constraints
    assert plan.path_constraints.orientation_constraints[0].absolute_x_axis_tolerance == pytest.approx(math.pi / 6)
    assert plan.path_constraints.orientation_constraints[0].absolute_y_axis_tolerance == pytest.approx(math.pi / 6)
    assert plan.goal_constraints[0].position_constraints
    assert not h.validated


def test_direct_return_targets_saved_joint_configuration(tmp_path):
    h = Harness(tmp_path)
    h.direct_moveit_active = h.transport_is_return = True
    h.return_goal_joints = (.4, .5)
    h.alternative_seed = (.1, .2)
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    goal = h.transport_motion_plan.calls[-1][0].motion_plan_request.goal_constraints[0]
    assert [c.position for c in goal.joint_constraints] == [.4, .5]
    assert not goal.position_constraints


def test_direct_rejection_does_not_fallback_to_waypoints(tmp_path):
    h = Harness(tmp_path)
    h.direct_moveit_active = True
    assert h._try_transport_alternative('collision')
    assert h.state == 'FAULT'
    assert not h.fk_calls and not h.transport_motion_plan.calls


@pytest.mark.parametrize('valid', [True, False])
@pytest.mark.parametrize('height_constraint', [True, False, None])
def test_direct_checks_exact_start_constraints_before_ompl(tmp_path, valid, height_constraint):
    h = Harness(tmp_path)
    h.direct_moveit_active = True
    h.clearance_phase = 'transfer'
    if height_constraint is not None:
        h.transport_moveit_clearance_constraint_enabled = height_constraint
    h.alternative_seed = (.1, .2)
    h.alternative_current_xyz = np.array([.3, -.6, .62])
    h.state_validity_client = Client()
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    assert not h.transport_motion_plan.calls
    check, pending = h.state_validity_client.calls[0]
    assert list(check.robot_state.joint_state.position) == [.1, .2]
    assert check.robot_state.is_diff
    assert bool(check.constraints.position_constraints) == bool(height_constraint)
    assert check.constraints.orientation_constraints
    pending.value = NS(valid=valid, contacts=[], constraint_result=[])
    pending.callback(pending)
    if valid:
        plan = h.transport_motion_plan.calls[0][0].motion_plan_request
        assert plan.path_constraints == check.constraints
        assert plan.start_state == check.robot_state
    else:
        assert not h.transport_motion_plan.calls
        assert h.state == 'FAULT'


@pytest.mark.parametrize('state', ['TRANSPORT_EXECUTING', 'TRANSPORT_VERIFYING',
                                  'TRANSPORT_STOPPING', 'FAULT'])
def test_no_fallback_after_execution_or_abort(tmp_path, state):
    h = Harness(tmp_path)
    h.state = state
    assert not h._try_transport_alternative('failure')
    assert not h.fk_calls and not h.transport_motion_plan.calls


@pytest.mark.parametrize('flag', ['transport_is_return', 'transport_is_pick',
                                 'direct_transfer_motion_started'])
def test_alternatives_only_apply_to_unexecuted_carried_transport(tmp_path, flag):
    h = Harness(tmp_path)
    setattr(h, flag, True)
    assert not h._try_transport_alternative('failure')
    assert not h.fk_calls


def test_callback_from_previous_candidate_is_ignored(tmp_path):
    h = Harness(tmp_path)
    called = Mock()
    old = h._transport_guard(called)
    h._try_transport_alternative('collision')
    old(Future())
    called.assert_not_called()


def test_rejected_moveit_response_does_not_execute(tmp_path):
    h = Harness(tmp_path)
    h.transport_route_attempt = 2
    h._alternative_moveit_received(Future(NS(motion_plan_response=NS(error_code=NS(val=-1)))))
    assert h.state == 'FAULT' and 'MoveIt overhead planning failed' in h.fault
    assert not h.validated


@pytest.mark.parametrize('descent_ok', [True, False])
def test_moveit_route_waits_for_complete_descent_plan_before_validation(tmp_path, descent_ok):
    h = Harness(tmp_path)
    h.transport_route_attempt = 1  # observation candidate already rejected
    h._try_transport_alternative('observation route collides')
    req, future = h.transport_cartesian.calls[-1]
    future.value = NS(error_code=NS(val=1), fraction=1.,
                      solution=NS(joint_trajectory=trajectory()))
    future.callback(future)
    h.fk_calls.pop()[1](Future(h.alternative_destination))
    req, future = h.transport_motion_plan.calls[-1]
    future.value = NS(motion_plan_response=NS(error_code=NS(val=1),
                      trajectory=NS(joint_trajectory=trajectory((.1, .2), (.2, .3)))))
    future.callback(future)
    assert not h.validated
    h.fk_calls.pop()[1](Future(h.alternative_destination))
    req, future = h.transport_cartesian.calls[-1]
    assert req.waypoints[0].position.z == h.transport_end[2]
    future.value = NS(error_code=NS(val=1), fraction=1. if descent_ok else .5,
                      solution=NS(joint_trajectory=trajectory((.2, .3), (.3, .4))))
    future.callback(future)
    assert not h.validated
    if descent_ok:
        h.fk_calls.pop()[1](Future(h.alternative_destination))
        assert len(h.validated) == 1
    else:
        assert h.state == 'FAULT'


def test_diagnostic_collision_starts_alternative_without_latching_fault(tmp_path):
    h = Harness(tmp_path)
    h.transport_partial_reason = 'continuous path reaches 43.3%'
    h._diagnostic_finish('collision pairs=link2/carried_item')
    assert h.transport_route_attempt == 1 and not h.fault


def test_timed_collision_rejection_advances_to_next_candidate(tmp_path):
    h = Harness(tmp_path)
    h.transport_route_attempt = 1
    h.state = h.TRANSPORT_VALIDATING
    h._transport_collision_checked(Future(NS(valid=False)))
    assert h.transport_route_attempt == 2 and not h.fault


def test_abort_discards_pending_alternative_plan_callback(tmp_path):
    h = Harness(tmp_path)
    h.transport_route_attempt = 1
    h._try_transport_alternative('observation route failed')
    _, future = h.transport_cartesian.calls[-1]
    h.state = 'FAULT'
    future.value = NS(error_code=NS(val=1), fraction=1.,
                      solution=NS(joint_trajectory=trajectory()))
    future.callback(future)
    assert not h.fk_calls and not h.validated


def test_join_reorders_joints_preserves_rest_seams_and_increasing_times():
    a = trajectory()
    b = trajectory((.2, .1), (.4, .3), ('j2', 'j1'))
    joined = join_trajectories([a, b], ['j1', 'j2'])
    assert [list(p.positions) for p in joined.points] == [[0., 0.], [.1, .2], [.3, .4]]
    assert [p.time_from_start.sec for p in joined.points] == [0, 1, 2]
    assert all(list(p.velocities) == [0., 0.] for p in joined.points)
    assert list(a.points[-1].velocities) == [.1, .1]  # input unchanged


@pytest.mark.parametrize('bad', ['branch_jump', 'untimed', 'nonfinite'])
def test_join_rejects_invalid_or_discontinuous_segments(bad):
    a, b = trajectory(), trajectory((.1, .2), (.3, .4))
    if bad == 'branch_jump':
        b.points[0].positions[0] += 2*math.pi
    elif bad == 'untimed':
        b.points[-1].velocities = []
    else:
        b.points[-1].positions[0] = math.nan
    with pytest.raises(ValueError):
        join_trajectories([a, b], ['j1', 'j2'])


def start_observation(h):
    h.transport_end_q = np.array([math.sqrt(.5), math.sqrt(.5), 0., 0.])
    h._try_transport_alternative('test')
    h.fk_calls.pop()[1](Future((np.array([.4, .2, .05]), h.transport_end_q)))




@pytest.mark.parametrize('context', ['pallet', 'staging_store'])
def test_expanded_search_varies_xy_rotation_and_never_calls_moveit(tmp_path, context):
    from safe_servo_visualization.transport_alternatives import waypoint_candidates
    h = Harness(tmp_path)
    h.transport_target = {'transfer_context': context}
    h.transport_moveit_fallback_enabled = False
    h.transport_expanded_waypoints_enabled = True
    if context == 'staging_store':
        h._try_transport_alternative('test')
        h.fk_calls.pop()[1](Future((np.array([.4, .2, .05]), h.transport_end_q)))
    else:
        start_observation(h)
    candidates = h._route_candidates()
    assert len(candidates) == len(set(candidates)) == 9 * 9 * (1 if context == 'staging_store' else 3)
    expected_offsets = {mm/1000. for mm in range(-100, 101, 25)}
    assert {r[0] for r in candidates} == expected_offsets
    assert {r[1] for r in candidates} == expected_offsets
    assert {r[2] for r in candidates} == ({'destination'} if context == 'staging_store' else {'via', 'elbow', 'destination'})
    for i in range(len(candidates)):
        assert h.alternative_candidate_index == i
        req, _ = h.transport_cartesian.calls[-1]
        assert req.avoid_collisions
        assert list(req.start_state.joint_state.position) == list(h.transport_seed)
        # Only source/destination orientations are used, never tilt detours.
        for kind, xyz, q in h.alternative_segments:
            assert (np.allclose(q, h.transport_start_q) or
                    np.allclose(q, h.transport_end_q))
            points = xyz if kind == 'cartesian_blend' else [xyz]
            for point in points:
                assert np.allclose(point, h.transport_end) or point[2] >= h.transport_high_z
        if context == 'staging_store':
            assert not h.alternative_segments  # One fixed-orientation translation group.
            assert all(np.allclose([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w], h.transport_start_q)
                       for p in req.waypoints)
        else:
            assert np.allclose(h.alternative_segments[-1][2], h.transport_end_q)
        h._alternative_failed('rotation rejected')
    assert h.alternative_rail_index == 1
    assert h.alternative_candidate_index == 0
    h.alternative_grid_deadline = 0.
    h._alternative_failed('alternate rail budget exhausted')
    for _ in range(2):
        h.alternative_grid_deadline = 0.
        h._alternative_failed('counterclockwise grid exhausted')
    assert h.state == 'FAULT'
    assert not h.validated
    assert not h.transport_motion_plan.calls
    assert h.transport_route_attempt == 1  # No silent SDK bypass after exhaustion.


def test_disabled_moveit_cannot_be_called_directly(tmp_path):
    h = Harness(tmp_path)
    h.transport_moveit_fallback_enabled = False
    with pytest.raises(ValueError, match='disabled'):
        h._alternative_moveit(h.transport_end, h.transport_end_q)
    assert not h.transport_motion_plan.calls


def test_waypoint_budget_exhaustion_does_not_start_another_candidate(tmp_path):
    h = Harness(tmp_path)
    h.transport_expanded_waypoints_enabled = True
    h.transport_moveit_fallback_enabled = False
    start_observation(h)
    h.transport_alternative_deadline = 0.
    assert not h._try_transport_alternative('rejected')
    assert h.alternative_candidate_index == 0
    assert not h.transport_motion_plan.calls


def test_yaw_separated_routes_are_bounded_and_moveit_is_last(tmp_path):
    h = Harness(tmp_path)
    start_observation(h)
    for i in range(6):
        assert h.transport_route_attempt == 1
        assert h.alternative_candidate_index == i
        # Every attempt restarts from the original robot seed; no motion occurred.
        req, _ = h.transport_cartesian.calls[-1]
        assert list(req.start_state.joint_state.position) == list(h.transport_seed)
        # The first request is a single fixed-orientation rounded translation;
        # its endpoint is the chosen overhead rotation location.
        assert len(req.waypoints) > 3
        assert all(p.orientation.x == h.transport_start_q[0] for p in req.waypoints)
        segments = h.alternative_segments
        assert len(segments) == 2
        assert segments[0][1][2] == h.transport_high_z
        assert np.allclose(segments[0][2], h.transport_end_q)
        endpoint = req.waypoints[-1].position
        assert np.allclose(segments[0][1], [endpoint.x, endpoint.y, endpoint.z])
        if i in (0, 2, 3):
            assert segments[1][0] == 'cartesian_blend'
            assert np.allclose(segments[1][1][-1], h.transport_end)
        else:
            assert np.allclose(segments[1][1], h.transport_end)
        assert not h.validated
        h._alternative_failed('test rejection')
    assert h.alternative_rail_index == 1
    h.alternative_grid_deadline = 0.
    h._alternative_failed('alternate rail budget exhausted')
    for _ in range(2):
        h.alternative_grid_deadline = 0.
        h._alternative_failed('counterclockwise grid exhausted')
    assert h.transport_route_attempt == 2
    h._alternative_failed('MoveIt also rejected')
    assert h.state == 'FAULT' and not h.validated


def test_partial_leg_diagnoses_then_retries_without_executing(tmp_path):
    h = Harness(tmp_path)
    h.transport_target = {'operation_id': 'test'}
    h.compute_ik_client = Client()
    start_observation(h)
    req, future = h.transport_cartesian.calls[-1]
    future.value = NS(error_code=NS(val=1), fraction=.4694,
                      solution=NS(joint_trajectory=trajectory()))
    future.callback(future)
    assert h.state == h.TRANSPORT_DIAGNOSING
    probe, pending = h.compute_ik_client.calls[-1]
    assert not probe.ik_request.avoid_collisions  # diagnostic only
    assert probe.ik_request.robot_state.is_diff
    assert list(probe.ik_request.robot_state.joint_state.position) == [.1, .2]
    h._diagnostic_finish('probe timed out; cause unresolved')
    assert h.alternative_candidate_index == 1
    assert h.state == h.TRANSPORT_PLANNING and not h.validated
    pending.value = NS(error_code=NS(val=-31))
    pending.callback(pending)  # stale probe cannot skip another route
    assert h.alternative_candidate_index == 1
    future.callback(future)  # stale Cartesian result cannot revive a discarded route
    assert h.alternative_candidate_index == 1


def test_total_alternative_budget_is_not_extended_by_retries(tmp_path):
    h = Harness(tmp_path)
    start_observation(h)
    deadline = h.transport_alternative_deadline
    h._alternative_failed('first')
    assert h.transport_alternative_deadline == deadline
    h.transport_alternative_deadline = 0.
    h._alternative_failed('timeout')
    assert h.state == 'FAULT' and not h.validated


def test_timed_validation_failure_tries_next_observation_variant(tmp_path):
    h = Harness(tmp_path)
    start_observation(h)
    h.state = h.TRANSPORT_VALIDATING
    h._transport_collision_checked(Future(NS(valid=False)))
    assert h.transport_route_attempt == 1 and h.alternative_candidate_index == 1
    assert not h.validated


def test_rotated_route_requires_rotation_and_descent_before_validation(tmp_path):
    h = Harness(tmp_path)
    start_observation(h)
    for i in range(3):
        req, future = h.transport_cartesian.calls[-1]
        future.value = NS(error_code=NS(val=1), fraction=1.,
                          solution=NS(joint_trajectory=trajectory(
                              tuple(req.start_state.joint_state.position), ((i+1)*.1, (i+1)*.1))))
        future.callback(future)
        assert not h.validated
        h.fk_calls.pop()[1](Future(h.alternative_destination))
    assert len(h.validated) == 1
    assert len(h.validated[0].solution.joint_trajectory.points) == 4
