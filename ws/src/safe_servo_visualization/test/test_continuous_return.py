from types import SimpleNamespace as NS

import numpy as np
import pytest
from trajectory_msgs.msg import JointTrajectoryPoint

from safe_servo_visualization.continuous_transport import transport_waypoints
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from safe_servo_visualization.place_pipeline_node import PlacePipeline
from test_continuous_transport import Harness, Future, planned_harness, DOWN


def test_flipped_slot_retreat_keeps_verified_target_without_mutating_coordinator():
    h = Harness()
    h.motion_status = dict(operation_id=13, transfer_context='staging_store',
                          pre_place_tcp_xyz_m=[.1,.2,.19],
                          transfer_tcp_quaternion_xyzw=DOWN)
    h.verified_slot_transfer_target = dict(h.motion_status,
        pre_place_tcp_xyz_m=[.12,.22,.19], transfer_tcp_quaternion_xyzw=[0,1,0,0])
    target = h._return_transport_target()
    assert target['pre_place_tcp_xyz_m'] == [.12,.22,.19]
    assert target['transfer_tcp_quaternion_xyzw'] == [0,1,0,0]
    assert h.motion_status['transfer_tcp_quaternion_xyzw'] == DOWN
    h.motion_status['operation_id'] = 14
    with pytest.raises(ValueError, match='different target'):
        h._return_transport_target()


def test_flipped_retreat_without_verified_snapshot_fails_closed():
    h = Harness()
    h.motion_status = dict(operation_id=13, transfer_context='staging_store')
    h.transport_slot_yaw_flipped = True
    with pytest.raises(ValueError, match='no verified'):
        h._return_transport_target()


@pytest.mark.parametrize('is_return', [False, True])
def test_capture_orientation_guard_only_applies_to_loaded_transfer(is_return):
    h = Harness()
    h.transport_is_return = is_return
    h.return_to_observation = False
    h.transport_target = dict(transfer_context='staging_store',
        pre_place_tcp_xyz_m=[.1,.2,.19], transfer_tcp_quaternion_xyzw=DOWN,
        transport_corner_clearance_z_m=.6)
    h._transport_pose = lambda result: (np.array([.12,.22,.2]), np.array([0,1,0,0]))
    # Stop at the independent ceiling guard after building the vertical path.
    h._transport_ceiling = lambda: .5
    h._transport_start_fk(Future(None))
    if is_return:
        assert 'ceiling' in h.fault
        assert 'orientation changed' not in h.fault
    else:
        assert 'orientation changed' in h.fault


@pytest.mark.parametrize('xyz,q,attached,skip', [
    ([.3, .2, .15], DOWN, '', True),
    ([.3, .2, .5], DOWN, '', True),
    ([.31, .2, .15], DOWN, '', False),
    ([.3, .2, .09], DOWN, '', False),
    ([.3, .2, .51], DOWN, '', False),
    ([.3, .2, .15], [0., 1., 0., 0.], '', False),
    ([.3, .2, .15], DOWN, 'held', False),
])
@pytest.mark.parametrize('direct', [False, True])
def test_return_collision_exception_only_initial_empty_vertical_column(xyz, q, attached, skip, direct):
    h = Harness()
    h.transport_is_return = True
    h.direct_moveit_active = direct
    h.transport_scene = None
    h.planning_scene_status = {'attached_item_id': attached}
    h.transport_start_xyz = np.array([.3, .2, .1])
    h.transport_start_q = np.array(DOWN)
    h.transport_high_z = .5
    h.return_collision_column_open = True
    h.return_collision_previous_z = .1
    h.transport_checks = [((0., 0.), 0.)]
    h.transport_check_index = 0
    h._transport_pose = lambda result: result
    geometry, checked = [], []
    h._transport_geometry_checked = geometry.append
    h._transport_check_collision = checked.append
    h._transport_fk_request = lambda joints, callback: callback(
        Future((np.array(xyz), np.array(q))))
    h._transport_validate_next()
    assert not h.fault
    assert bool(geometry) == skip
    assert bool(checked) != skip
    if not skip:
        # Returning to the column later cannot reopen the exception.
        h.planning_scene_status = {}
        h._return_collision_classified(Future((np.array([.3, .2, .2]), np.array(DOWN))))
        assert len(checked) == 2
        assert not geometry


def test_return_collision_classifier_fails_closed_on_missing_fk():
    h = Harness()
    h._return_collision_classified(Future(None))
    assert 'FK failed' in h.fault


@pytest.mark.parametrize('actual,expected,allowed', [
    ('', None, True), (None, None, True), ('', {}, True),
    ('unexpected_item', None, False),
    ('held', {'attached_item_id': 'held'}, True),
    ('', {'attached_item_id': 'held'}, False),
    ('other', {'attached_item_id': 'held'}, False),
])
def test_execution_attachment_guard(actual, expected, allowed, monkeypatch):
    from trajectory_msgs.msg import JointTrajectory
    monkeypatch.setattr('safe_servo_visualization.continuous_transport.time.monotonic', lambda: 10.)
    h = Harness()
    h.pallet_locked = True
    h.robot_mode, h.robot_state, h.robot_error = 1, 0, 0
    h.robot_state_time = h.last_joint_state_time = h.last_force_time = 10.
    h.status_timeout = h.force_timeout = 1.
    h.motion_status = h.transport_target = {'operation_id': 6}
    h.planning_scene_status = {'attached_item_id': actual}
    h.transport_scene = expected
    h.latest_joint_positions = h.transport_seed = (0., 0.)
    h.transport_trajectory = JointTrajectory()
    h.transport_duration = 1.
    h.operation_id = 10
    sent = []
    def send(goal, **kwargs):
        sent.append(goal)
        return Future(None)
    h.transfer_trajectory_client = NS(send_goal_async=send)
    h._transport_execute()
    assert bool(sent) == allowed
    assert (h.state == h.TRANSPORT_EXECUTING) == allowed
    if not allowed:
        assert 'prerequisites changed' in h.fault


def test_empty_return_lifts_before_tilting_or_crossing():
    start, end = np.array([.3,-.2,.1]),np.array([.5,.02,.6])
    obs_q = [1.,0.,.1,0.]
    samples,safe,high = transport_waypoints(start,DOWN,end,obs_q,.5,None,
                                           allow_tilt_change=True)
    assert high > safe >= .6
    for p,q in samples:
        if p[2] < safe:
            assert np.allclose(p[:2],start[:2]) or np.allclose(p[:2],end[:2])
        if np.allclose(p[:2],start[:2]):
            assert np.allclose(q,DOWN)
    assert np.allclose(samples[-1][0],end)


def test_reverse_path_is_retimed_and_preserves_exact_observation_joints():
    h,result = planned_harness()
    h.transport_is_return = True
    h.return_goal_joints = (0.,0.)
    h.transport_seed = (.1,.2)
    middle = JointTrajectoryPoint()
    middle.positions = [.1,.05]
    middle.velocities = [.2,.1]
    middle.time_from_start.nanosec = 500000000
    result.solution.joint_trajectory.points.insert(1,middle)
    h._transport_planned(Future(result))
    assert not h.fault
    points = h.transport_trajectory.points
    assert list(points[0].positions) == [.1,.2]
    assert list(points[-1].positions) == [0.,0.]
    assert all(v < 0 for v in points[1].velocities)
    assert points[0].time_from_start.sec == points[0].time_from_start.nanosec == 0
    assert np.allclose(h.transport_checks[-1][0],h.return_goal_joints)


def test_reverse_path_cannot_start_from_another_ik_branch():
    h,result = planned_harness()
    h.transport_is_return = True
    h.return_goal_joints = (0.,0.)
    h.transport_seed = (1.,1.)
    h._transport_planned(Future(result))
    assert h.state == h.FAULT
    assert 'measured joints' in h.fault


def test_return_interior_stop_is_smoothed_without_fallback():
    h, result = planned_harness()
    h.transport_is_return = True
    h.return_goal_joints = (0., 0.)
    h.transport_seed = (.1, .2)
    middle = JointTrajectoryPoint()
    middle.positions = [.1, .05]
    middle.velocities = [0., 0.]
    middle.time_from_start.nanosec = 500000000
    result.solution.joint_trajectory.points.insert(1, middle)
    fallbacks = []
    h._fallback_staged_return = lambda reason: fallbacks.append(reason) or True
    h._transport_planned(Future(result))
    assert not h.fault
    assert fallbacks == []
    assert h.state == h.TRANSPORT_VALIDATING
    assert all(list(p.accelerations) == [0., 0.] for p in h.transport_trajectory.points)


def return_supervisor():
    node = object.__new__(PickupSupervisor)
    node.operation_id = 10
    node.state = node.DETACHING
    node.motion_status = {'operation_id':6, 'pre_place_tcp_xyz_m':[.2,.3,.15]}
    node.continuous_return_target_id = 6
    node.operation_kind = 'place'
    node.staging_place_active = False
    node.post_retreat_fault = ''
    node.planning_scene_status = {}
    node._fault = lambda reason: setattr(node,'fault',reason)
    node.fault = ''
    return node


def staged_supervisor():
    import time
    node = return_supervisor()
    node.state = node.TRANSPORT_PLANNING
    node.transport_is_return = True
    node.transfer_goal_handle = None
    node.robot_mode, node.robot_state, node.robot_error = 1, 0, 0
    node.robot_state_time = node.last_joint_state_time = time.monotonic()
    node.status_timeout = 1.
    node.pallet_locked = True
    node.transport_target = dict(node.motion_status)
    node.transport_seed = node.latest_joint_positions = (0., 0.)
    node.robot_tcp_xyz = (0., 0., .13)
    node.direct_tcp_z_offset = .02
    node.return_pre_place_z = .15
    node.return_clearance_z = .6
    node.tolerance = .001
    node.servo_bounds_mm = [0, 0, 0, 0, 0, 900]
    node.get_logger = lambda: NS(warning=lambda *a: None)
    future = Future(NS(success=True))
    node.enable_client = NS(service_is_ready=lambda: True, call_async=lambda req: future)
    calls = []
    node._begin_direct_retreat = lambda: calls.append(node.direct_target_z)
    return node, future, calls


def test_staged_fallback_pauses_before_reusing_mode_handoff_and_is_one_shot():
    node, future, calls = staged_supervisor()
    assert node._fallback_staged_return('intermediate stop')
    assert not calls and node.state == node.RETURN_DISABLING
    assert not node.continuous_return_completed
    assert not node.continuous_return_restoring
    assert not node.return_clearance_pending
    assert node.continuous_return_target_id is None
    assert not node._fallback_staged_return('duplicate')
    future.callback(future)
    assert calls == [.6]  # Saved overhead height, not observation or pre-place Z.
    assert not node.fault


@pytest.mark.parametrize('fault', [
    'robot', 'mode', 'stale', 'attached', 'changed_operation', 'moved',
    'downward', 'ceiling', 'below_pre_place', 'goal', 'pause_unavailable', 'ft',
])
def test_staged_fallback_does_not_bypass_safety_prerequisites(fault):
    node, future, calls = staged_supervisor()
    if fault == 'robot': node.robot_error = 52
    if fault == 'mode': node.robot_mode = 0
    if fault == 'stale': node.robot_state_time = 0.
    if fault == 'attached': node.planning_scene_status['attached_item_id'] = 'held'
    if fault == 'changed_operation': node.motion_status['operation_id'] = 7
    if fault == 'moved': node.latest_joint_positions = (.2, 0.)
    if fault == 'downward': node.return_clearance_z = .10
    if fault == 'ceiling': node.return_clearance_z = 1.
    if fault == 'below_pre_place': node.robot_tcp_xyz = (0., 0., .10)
    if fault == 'goal': node.transfer_goal_handle = object()
    if fault == 'pause_unavailable': node.enable_client.service_is_ready = lambda: False
    if fault == 'ft': node.ft_recovery_required = True
    assert node._fallback_staged_return('intermediate stop')
    assert 'fallback blocked' in node.fault
    assert not calls


@pytest.mark.parametrize('change', ['abort', 'operation', 'robot_error', 'pause_failed'])
def test_staged_pause_completion_cannot_restart_aborted_or_unhealthy_robot(change):
    node, future, calls = staged_supervisor()
    node._fallback_staged_return('intermediate stop')
    if change == 'abort': node.state = node.FAULT
    if change == 'operation': node.operation_id += 1
    if change == 'robot_error': node.robot_error = 52
    if change == 'pause_failed': future = Future(NS(success=False))
    node._staged_return_servo_disabled(future, 10)
    assert not calls


@pytest.mark.parametrize('state', ['TRANSPORT_EXECUTING', 'TRANSPORT_VERIFYING', 'FAULT'])
def test_staged_fallback_cannot_run_after_execution_or_fault(state):
    node, future, calls = staged_supervisor()
    node.state = state
    assert not node._fallback_staged_return('intermediate stop')
    assert not calls


@pytest.mark.parametrize('kind', ['place', 'loading'])
def test_staged_completion_uses_original_observation_plan(kind):
    node = object.__new__(PlacePipeline)
    node.expected_supervisor_operation_id = 9
    node.supervisor_status = {'operation_kind': kind, 'operation_id': 9,
                             'state': 'SUCCEEDED', 'continuous_return_completed': False,
                             'place_fallback_used': kind == 'loading'}
    calls = []
    node._begin_observation_motion = lambda: calls.append('observation')
    node.get_logger = lambda: NS(warning=lambda *a: None)
    (node._tick_contact_place if kind == 'place' else node._tick_loading)()
    assert calls == ['observation']


@pytest.mark.parametrize('case', ['place','early_contact'])
def test_return_is_requested_only_after_release_and_detachment(case):
    node = return_supervisor()
    if case == 'early_contact':
        node.operation_kind = 'loading'
        node.loading_contact_fallback = True
    assert node._should_return_continuously()
    node.planning_scene_status = {'attached_item_id':'still_held'}
    node._begin_continuous_return()
    assert 'confirmed release' in node.fault


def test_disable_confirmation_restores_control_but_does_not_start_motion():
    node = return_supervisor()
    future = Future(NS(success=True))
    node.enable_client = NS(service_is_ready=lambda: True, call_async=lambda req: future)
    events = []
    node._begin_direct_retreat = lambda: events.append('slow_retreat')
    node._plan_continuous_return = lambda: events.append('plan')
    node._begin_continuous_return()
    assert not events and node.state == node.RETURN_DISABLING
    future.callback(future)
    assert events == ['slow_retreat']
    assert node.direct_target_z == .15
    assert node.return_clearance_pending
    node._finish_retreat()  # This is the continuation of the readiness gate.
    assert events == ['slow_retreat','plan']


@pytest.mark.parametrize('case', ['valid', 'stale', 'changed', 'nonfinite', 'ceiling'])
def test_slow_return_preserves_verified_raised_pre_place_for_same_target_only(case):
    node = return_supervisor()
    node.motion_status['pre_place_tcp_xyz_m'] = [.1452, -.3796, -.0414]
    node.raised_retreat_pose = dict(target_id=6, xyz=[.1452, -.3796, -.0264],
                                   original_xyz=[.1452, -.3796, -.0414])
    node.servo_bounds_mm = [-1000, 1000, -1000, 1000, -100, 800]
    node.get_logger = lambda: NS(info=lambda *args: None)
    future = Future(NS(success=True))
    node.enable_client = NS(service_is_ready=lambda: True, call_async=lambda req: future)
    events = []
    node._begin_direct_retreat = lambda: events.append(node.direct_target_z)
    if case == 'stale': node.raised_retreat_pose['target_id'] = 5
    if case == 'changed': node.motion_status['pre_place_tcp_xyz_m'][0] += .01
    if case == 'nonfinite': node.raised_retreat_pose['xyz'][2] = float('nan')
    if case == 'ceiling': node.servo_bounds_mm[5] = -30.
    node._begin_continuous_return()
    if case in ('changed', 'nonfinite', 'ceiling'):
        assert node.fault and not events
    else:
        assert not events and not node.fault
        future.callback(future)
        assert events == pytest.approx([-.0414 if case == 'stale' else -.0264])


def test_late_disable_response_after_abort_is_ignored():
    node = return_supervisor()
    node.state = node.FAULT
    node._restore_ros2_control_mode = lambda: pytest.fail('must not restore after abort')
    node._return_servo_disabled(Future(NS(success=True)),10)


@pytest.mark.parametrize('start_z,expect_motion', [(.10, True), (.20, False)])
def test_slow_clearance_retreat_is_upward_and_speed_capped(start_z, expect_motion):
    node = return_supervisor()
    sent, restored = [], []
    node.retreat_client = NS(service_is_ready=lambda: True,
                            call_async=lambda req: sent.append(req) or Future(None))
    node.operation_kind = 'place'
    node.retreat_start_z = start_z
    node.direct_tcp_z_offset = .02
    node.direct_target_z = .17  # SDK target is .15 m.
    node.return_clearance_pending = True
    node.return_clearance_speed = 10.
    node.retreat_speed, node.retreat_acc = 300., 2000.
    node.tolerance = .001
    node.direct_motion_generation = 0
    node._restore_ros2_control_mode = lambda: restored.append(True)
    node.get_logger = lambda: NS(info=lambda *a: None)
    node._send_direct_retreat()
    assert bool(sent) == expect_motion
    assert bool(restored) != expect_motion
    if sent:
        assert sent[0].speed == 10.
        assert sent[0].acc == 100.
        assert sent[0].relative and sent[0].wait
        assert list(sent[0].pose) == pytest.approx([0,0,50,0,0,0])


def test_continuous_return_blocked_if_slow_retreat_did_not_reach_clearance():
    import time
    node = return_supervisor()
    node.return_clearance_pending = True
    node.return_pre_place_z = .17
    node.direct_tcp_z_offset = .02
    node.tolerance = .001
    node.robot_tcp_xyz = (0.,0.,.1)
    node.robot_state_time = time.monotonic()
    node.status_timeout = 1.
    node._plan_continuous_return()
    assert 'did not reach pre-place' in node.fault


def test_return_readiness_requires_uninterrupted_settle_delay(monkeypatch):
    node = return_supervisor()
    clock = [10.]
    monkeypatch.setattr('safe_servo_visualization.pickup_supervisor_node.time.monotonic',lambda: clock[0])
    node.continuous_return_restoring = True
    node.restore_settle_started = 10.
    node.restore_settle_ready_count = 0
    node.restore_wait_deadline = 20.
    node.post_restore_settle = .75
    node.post_restore_ready_samples = 5
    node.status_timeout = 1.
    node.ros2_control_mode = 1
    node.robot_mode,node.robot_state,node.robot_error = 1,0,0
    node.get_logger = lambda: NS(info=lambda *a: None)
    events = []
    node._finish_retreat = lambda: events.append('plan')
    for t in [10.1,10.2,10.3,10.4,10.5]:
        clock[0] = node.robot_state_time = node.last_joint_state_time = t
        node._restore_readiness_tick()
    assert not events
    clock[0] = 10.6
    node.robot_mode = 0  # Readiness interruption restarts the settling clock.
    node._restore_readiness_tick()
    assert node.restore_settle_started == 10.6
    node.robot_mode = 1
    for t in [10.7,10.8,10.9,11.,11.1,11.2,11.3]:
        clock[0] = node.robot_state_time = node.last_joint_state_time = t
        node._restore_readiness_tick()
    assert not events
    clock[0] = node.robot_state_time = node.last_joint_state_time = 11.4
    node._restore_readiness_tick()
    assert events == ['plan']


@pytest.mark.parametrize('kind', ['place','loading'])
def test_no_second_observation_move_after_connected_return(kind):
    node = object.__new__(PlacePipeline)
    node.expected_supervisor_operation_id = 9
    node.supervisor_status = {'operation_kind':kind,'operation_id':9,
        'state':'SUCCEEDED','continuous_return_completed':True}
    node.publish_status = lambda: None
    node._begin_observation_motion = lambda: pytest.fail('duplicate observation motion')
    (node._tick_contact_place if kind == 'place' else node._tick_loading)()
    assert node.state == node.SUCCEEDED


def test_direct_return_initializes_exit_column_before_planning():
    h = Harness()
    h.transport_is_return = True
    h.transport_moveit_direct_enabled = True
    h.transport_target = dict(pre_place_tcp_xyz_m=[.5, 0., .4],
        transfer_tcp_quaternion_xyzw=DOWN, transport_corner_clearance_z_m=.5)
    h.planning_scene_status = dict(camera_collision_applied=True, add_placed_item_obstacle=True)
    h.transport_motion_plan = NS(service_is_ready=lambda: True)
    h.transport_seed = (0.,)*6
    h._transport_pose = lambda result: (np.array([.3, .2, .1]), np.array(DOWN))
    h.return_collision_column_open = False
    h.return_collision_previous_z = .9
    calls = []
    h._begin_clearance_transfer = lambda *args: calls.append(args)
    h._transport_start_fk(Future(None))
    assert not h.fault
    assert calls and h.direct_moveit_active
    assert h.return_collision_column_open
    assert h.return_collision_previous_z == pytest.approx(.1)


def test_normal_observation_return_uses_native_clearance_without_moveit_cartesian():
    node, future, calls = staged_supervisor()
    node.ros2_control_mode = 1
    node.return_to_observation = True
    node.return_clearance_pending = False
    node.motion_status['transfer_tcp_z_m'] = .6
    node.get_logger = lambda: NS(info=lambda *a: None, warning=lambda *a: None)
    # No MoveIt service stubs: planning must not be used for this vertical leg.
    node._plan_continuous_return()
    assert not node.fault and not calls
    assert not node.continuous_return_restoring
    assert node.return_staged_endpoint == (0., 0., .6)
    future.callback(future)
    assert calls == [.6]


def test_normal_return_mode_mismatch_restores_and_waits_without_motion():
    node, future, calls = staged_supervisor()
    node.ros2_control_mode = 1
    node.robot_mode = 0
    node.return_to_observation = True
    node.return_clearance_pending = False
    restored = []
    node._restore_ros2_control_mode = lambda: restored.append(True)
    node._plan_continuous_return()
    assert not node.fault and not calls
    assert restored == [True] and node.continuous_return_restoring


def test_native_clearance_must_reach_target_before_observation():
    node, _, _ = staged_supervisor()
    node.return_staged_endpoint = (0., 0., .6)
    node._finish_retreat()
    assert 'endpoint not confirmed' in node.fault
    assert node.return_staged_endpoint is not None


def test_mode_change_during_staged_pause_waits_without_sending_retreat():
    node, future, calls = staged_supervisor()
    node.ros2_control_mode = 1
    node.get_logger = lambda: NS(info=lambda *a: None, warning=lambda *a: None)
    node._fallback_staged_return(None)
    node.robot_mode = 0
    restored = []
    node._restore_ros2_control_mode = lambda: restored.append(True)
    future.callback(future)
    assert not node.fault and not calls
    assert restored == [True]
    assert node.continuous_return_restoring and not node.return_staged_fallback_used
    assert node.return_staged_endpoint is None
