"""SDK fallback uses mocked services only; never connects to a robot."""
import math
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from test_transport_alternatives import Harness, Client, Future
from safe_servo_visualization.sdk_transport import joint_line, sdk_joint_samples, nearest_equivalent_joints


def test_equivalent_ik_avoids_full_turn_without_changing_physical_pose():
    limits = {'j1': (-2*math.pi, 2*math.pi, 2.), 'j2': (-2., 2., 2.)}
    raw, seed = (-3.10, .4), (3.10, .2)
    selected = nearest_equivalent_joints(raw, seed, ['j1', 'j2'], limits)
    assert selected == pytest.approx((2*math.pi-3.10, .4))
    assert abs(selected[0]-seed[0]) < .1
    assert np.sin(selected) == pytest.approx(np.sin(raw))
    assert np.cos(selected) == pytest.approx(np.cos(raw))


def test_equivalent_ik_respects_bounded_joint_and_does_not_wrap_seed():
    assert nearest_equivalent_joints([-3.1], [3.1], ['j'], {'j': (-3.14, 3.14)}) == (-3.1,)
    for goal, seed in (([7.], [0.]), ([0.], [7.]), ([math.nan], [0.])):
        with pytest.raises(ValueError, match='joint limits'):
            nearest_equivalent_joints(goal, seed, ['j'], {'j': (-6.28, 6.28)})


def test_sdk_ik_normalizes_before_excursion_check_but_still_validates_path(tmp_path):
    h = sdk_node(tmp_path)
    h.alternative_seed = (3.1, .2)
    h.direct_transfer_max_joint_delta = math.pi
    h._alternative_segment_ready = Mock()
    h._sdk_ik_received(Future(NS(error_code=NS(val=1), solution=NS(joint_state=NS(
        name=h.arm_joint_names, position=[-3.1, .2])))))
    assert not h.fault
    trajectory = h._alternative_segment_ready.call_args.args[0]
    assert list(trajectory.points[0].positions) == [3.1, .2]
    assert trajectory.points[-1].positions == pytest.approx([2*math.pi-3.1, .2])
    assert not h.sdk_joint_client.calls  # FK/collision validation still required


def test_sdk_genuine_excursion_rejection_identifies_joint_and_angles(tmp_path):
    h = sdk_node(tmp_path)
    h.alternative_seed = (3.1, .2)
    h.transport_joint_limits['j1'] = (-3.14, 3.14, 2.)
    h.direct_transfer_max_joint_delta = math.pi
    h._sdk_ik_received(Future(NS(error_code=NS(val=1), solution=NS(joint_state=NS(
        name=h.arm_joint_names, position=[-3.1, .2])))))
    assert 'j1 delta=6.200000 rad' in h.fault
    assert 'raw_goal=-3.100000' in h.fault
    assert not h.sdk_joint_client.calls


def sdk_node(tmp_path):
    h = Harness(tmp_path)
    h.FAULT = 'FAULT'
    h.RETREATING = 'RETREATING'
    h.DISABLING_SERVO = 'DISABLING_SERVO'
    h.sdk_transport_enabled = True
    h.sdk_joint_client = Client()
    h.compute_ik_client = Client()
    h.enable_client = Client()
    h.transport_route_attempt = 2
    h.transport_alternative_deadline = time.monotonic()+60
    h.transport_sdk_active = False
    h.transport_sdk_candidate = False
    h.direct_transfer_succeeded = False
    h.direct_transfer_max_joint_delta = 4.
    h.direct_transfer_max_joint_speed = 2.
    h.direct_transfer_joint_acc = 3.
    h.motion_status = h.transport_target = dict(operation_id=3)
    h.planning_scene_status = h.transport_scene
    h.pallet_locked = True
    h.robot_mode, h.robot_state, h.robot_error = 0, 2, 0
    h.robot_state_time = h.last_force_time = time.monotonic()
    h.status_timeout = h.force_timeout = 1.
    h.latest_force_z = 2.
    h.place_force_threshold = 5.
    h.sdk_report_joints = h.transport_seed
    h.sdk_commands = [(.01, .02)]
    h._begin_direct_retreat = Mock()
    h._restore_ros2_control_mode = Mock()
    return h


def test_fallback_only_after_moveit_rejection_and_never_moves_during_planning(tmp_path):
    h = sdk_node(tmp_path)
    assert h._try_sdk_transport('invalid plan')
    assert h.transport_route_attempt == 3 and h.transport_sdk_candidate
    h.fk_calls.pop()[1](Future((np.array([.4, .2, .05]), h.transport_start_q)))
    assert [s[0] for s in h.alternative_segments] == ['sdk_joint']*3+['cartesian']
    assert np.allclose(h.alternative_segments[0][1], [.25, .5, .65])
    assert np.allclose(h.alternative_segments[1][1], [.25, -.6, .65])
    assert h.transport_cartesian.calls  # planning lift first
    assert not h.sdk_joint_client.calls and not h.enable_client.calls
    assert not h._try_sdk_transport('no recursive SDK retry')


@pytest.mark.parametrize('blocked', ['early', 'disabled', 'executing', 'goal', 'pick', 'return', 'unattached', 'expired', 'fault'])
def test_sdk_fallback_entry_interlocks(tmp_path, blocked):
    h = sdk_node(tmp_path)
    if blocked == 'early': h.transport_route_attempt = 1
    if blocked == 'disabled': h.sdk_transport_enabled = False
    if blocked == 'executing': h.direct_transfer_motion_started = True
    if blocked == 'goal': h.transfer_goal_handle = object()
    if blocked == 'pick': h.transport_is_pick = True
    if blocked == 'return': h.transport_is_return = True
    if blocked == 'unattached': h.transport_scene = {}
    if blocked == 'expired': h.transport_alternative_deadline = 0.
    if blocked == 'fault': h.state = h.FAULT
    assert not h._try_sdk_transport('rejected')
    assert not h.sdk_joint_client.calls and not h.enable_client.calls


def test_sdk_ik_keeps_frame_seed_collision_check_and_limits(tmp_path):
    h = sdk_node(tmp_path)
    h.state = h.TRANSPORT_PLANNING
    h.alternative_seed = (.1, .2)
    h._sdk_plan_joint([.25, .5, .65], h.transport_end_q)
    req, future = h.compute_ik_client.calls[-1]
    assert req.ik_request.avoid_collisions
    assert req.ik_request.robot_state.is_diff
    assert req.ik_request.pose_stamped.header.frame_id == 'link_base'
    assert list(req.ik_request.robot_state.joint_state.position) == [.1, .2]
    future.value = NS(error_code=NS(val=-31))
    future.callback(future)
    assert 'endpoint IK failed' in h.fault
    assert not h.sdk_joint_client.calls


def test_sdk_joint_lines_are_dense_and_limits_checked():
    traj = joint_line([0., 0.], [.4, -.2], ['a', 'b'], .3, .5)
    commands, checks = sdk_joint_samples(traj, ['a', 'b'], dict(a=(-1, 1, 1), b=(-1, 1, 1)))
    assert np.allclose(commands[-1], [.4, -.2])
    assert max(np.max(np.abs(np.array(a[0])-b[0])) for a, b in zip(checks, checks[1:])) <= .01000001
    assert all(abs(q[1]+q[0]/2) < 1e-8 for q, _ in checks)
    with pytest.raises(ValueError, match='joint limits'):
        sdk_joint_samples(traj, ['a', 'b'], dict(a=(-.1, .1, 1), b=(-1, 1, 1)))
    with pytest.raises(ValueError, match='budget'):
        sdk_joint_samples(traj, ['a', 'b'], dict(a=(-1, 1, 1), b=(-1, 1, 1)), max_samples=2)


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_nonfinite_joint_input_rejected(bad):
    with pytest.raises(ValueError):
        joint_line([0., 0.], [bad, .2], ['a', 'b'], .3, .5)


def test_sdk_execution_waits_for_pause_and_exclusive_handoff(tmp_path):
    h = sdk_node(tmp_path)
    h._sdk_begin_execution()
    assert h.state == h.DISABLING_SERVO
    assert not h.sdk_joint_client.calls
    req, future = h.enable_client.calls[-1]
    assert not req.data
    future.value = NS(success=True)
    future.callback(future)
    h._begin_direct_retreat.assert_called_once()
    assert not h.sdk_joint_client.calls  # controller/mode handoff must call dispatch


def test_nonblocking_command_requires_fresh_stopped_arrival_then_restore(tmp_path):
    h = sdk_node(tmp_path)
    h._sdk_begin_execution()
    h._sdk_send_next()  # stand-in for completed exclusive mode-0 handoff
    req, future = h.sdk_joint_client.calls[-1]
    assert not req.wait and not req.relative and req.radius == -1.
    assert req.speed <= .3 and req.acc <= .5
    future.value = NS(ret=0)
    future.callback(future)
    h.robot_state_time = time.monotonic()
    h._sdk_transport_tick()  # command acceptance does not mean arrival
    h._restore_ros2_control_mode.assert_not_called()
    h.sdk_report_joints = h.sdk_goal
    h.robot_state = 1
    h._sdk_transport_tick()  # still moving
    h._restore_ros2_control_mode.assert_not_called()
    h.robot_state = 2
    h._sdk_transport_tick()
    h._restore_ros2_control_mode.assert_called_once()
    assert h.sdk_phase == 'restore'
    assert h.transport_sdk_active  # success withheld until restore/fresh FK


@pytest.mark.parametrize('bad', ['timeout', 'telemetry', 'force_stale', 'force_contact', 'hardware', 'mode', 'scene', 'target', 'deviation'])
def test_sdk_motion_faults_hold_and_block_next_command(tmp_path, bad):
    h = sdk_node(tmp_path)
    h._sdk_begin_execution()
    h._sdk_send_next()
    if bad == 'timeout': h.sdk_deadline = 0.
    if bad == 'telemetry': h.robot_state_time = 0.
    if bad == 'force_stale': h.last_force_time = 0.
    if bad == 'force_contact': h.latest_force_z = 20.
    if bad == 'hardware': h.robot_error = 52
    if bad == 'mode': h.robot_mode = 1
    if bad == 'scene': h.planning_scene_status = dict(attached_item_id='other')
    if bad == 'target': h.motion_status = dict(operation_id=22)
    if bad == 'deviation': h.sdk_report_joints = (1., -1.)
    assert h._sdk_transport_tick()
    assert h.state == h.FAULT
    assert len(h.sdk_joint_client.calls) == 1
    h._restore_ros2_control_mode.assert_not_called()


def test_aborted_or_old_callback_cannot_advance_sdk(tmp_path):
    h = sdk_node(tmp_path)
    h._sdk_begin_execution()
    _, future = h.enable_client.calls[-1]
    h.state = h.FAULT
    future.value = NS(success=True)
    future.callback(future)
    h._begin_direct_retreat.assert_not_called()
    h.state = h.DISABLING_SERVO
    h.operation_id += 1
    future.callback(future)
    h._begin_direct_retreat.assert_not_called()


def test_restore_requires_final_joints_then_fresh_fk(tmp_path):
    h = sdk_node(tmp_path)
    h._sdk_begin_execution()
    h.sdk_previous = (.1, .2)
    h.latest_joint_positions = (.1, .2)
    h._transport_result = Mock()
    h._sdk_restored()
    assert not h.transport_sdk_active
    h._transport_result.assert_called_once()
    assert not h.direct_transfer_succeeded


def test_sdk_command_dispatch_rechecks_health_before_sending(tmp_path):
    h = sdk_node(tmp_path)
    h._sdk_begin_execution()
    h.last_force_time = 0.
    h._sdk_send_next()
    assert h.state == h.FAULT
    assert not h.sdk_joint_client.calls


def test_restore_mismatched_joints_never_authorizes_descent(tmp_path):
    h = sdk_node(tmp_path)
    h._sdk_begin_execution()
    h.latest_joint_positions = (1., 1.)
    h._transport_result = Mock()
    h._sdk_restored()
    assert h.state == h.FAULT
    h._transport_result.assert_not_called()


def test_supervisor_fault_requests_direct_stop_and_invalidates_sdk_callbacks(tmp_path):
    from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
    h = sdk_node(tmp_path)
    h._sdk_begin_execution()
    h.state = h.FAULT  # exercise idempotent fault's stop/invalidation prefix
    h.set_state_client = Client()
    PickupSupervisor._fault(h, 'operator abort')
    assert h.set_state_client.calls[0][0].data == 3
    assert not h.transport_sdk_active and not h.transport_sdk_candidate
    h._restore_ros2_control_mode.assert_not_called()


@pytest.mark.parametrize('failure', ['collision', 'ceiling'])
def test_failed_moveit_validation_can_plan_fresh_sdk_route_but_never_execute_rejected_path(tmp_path, failure):
    h = sdk_node(tmp_path)
    h.state = h.TRANSPORT_VALIDATING
    if failure == 'collision':
        h._transport_collision_checked(Future(NS(valid=False)))
    else:
        h._transport_reject_geometry('timed transport exceeds workspace ceiling')
    assert h.transport_route_attempt == 3 and h.transport_sdk_candidate
    assert not h.sdk_joint_client.calls and not h.enable_client.calls


def test_sdk_validation_failure_stops_without_recursive_fallback(tmp_path):
    h = sdk_node(tmp_path)
    h.transport_route_attempt = 3
    h.state = h.TRANSPORT_VALIDATING
    h._transport_reject_geometry('timed transport exceeds workspace ceiling')
    assert h.state == h.FAULT
    assert not h.sdk_joint_client.calls and not h.enable_client.calls
