"""Raised known-source pickup: planning-only search and guarded contact gates."""
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from test_pickup_workspace import supervisor


def test_search_is_bounded_and_replans_from_original_joint_seed():
    n = object.__new__(PickupSupervisor)
    n.transport_is_pick = True
    n.state = n.TRANSPORT_DIAGNOSING
    n.transport_target = dict(transfer_context='known_pick')
    n.transport_seed = (1., 2.)
    n.raised_pre_pick_max_mm = 10.
    n._transport_fk_request = Mock()
    n.get_logger = Mock(return_value=Mock())
    assert n._try_raised_pick_path('partial')
    assert n.raised_pick_offset_m == pytest.approx(.005)
    assert n._try_raised_pick_path('partial')
    assert n.raised_pick_offset_m == pytest.approx(.01)
    assert not n._try_raised_pick_path('partial')
    assert n._transport_fk_request.call_count == 2
    assert n._transport_fk_request.call_args.args[0] == (1., 2.)


def test_raised_pick_preserves_original_floor_and_rejects_changed_target():
    n = supervisor()
    snapshot = n.motion_status['planned_pregrasp']
    snapshot.update(pickup_source='pallet', pregrasp_z_m=-.0214, top_z_m=-.0514,
                    retrieval_target_id=9)
    n.motion_status['operation_id'] = 10
    n.planning_scene_status = {}
    n._tcp_xyz = lambda: (.128, -.591, -.0014)
    n.raised_pick_ready = dict(target_id=10, retrieval_target_id=9,
                              xyz=[.128, -.591, -.0014], original_z=-.0214)
    n.last_force_time = time.monotonic()
    n.force_timeout = 1.
    _, start, floor = n._validate_pregrasp_ready()
    assert start == pytest.approx(-.0014)
    assert floor == pytest.approx(-.0664)
    n.motion_status['operation_id'] = 11
    with pytest.raises(ValueError, match='handoff changed'):
        n._validate_pregrasp_ready()


def contact_node():
    n = object.__new__(PickupSupervisor)
    n.raised_pick_active = True
    n.direct_place_recovery_active = True
    n.force_threshold = 5.
    n.loading_contact_confirm_samples = 2
    n.raised_pick_contact_count = 0
    n.pregrasp_z = .2
    n.direct_tcp_z_offset = .01
    n.minimum_contact_descent = .005
    n._direct_mode_tcp_xyz = lambda: (0., 0., .16)
    n._fault = Mock()
    n._turn_vacuum_on = Mock()
    n._turn_vacuum_off = Mock()
    return n


def test_pickup_needs_repeated_contact_and_never_releases():
    n = contact_node()
    assert n._raised_pick_contact_check(6., 1.)
    n._turn_vacuum_on.assert_not_called()
    assert n.state == n.WAITING_PLACE_STEP_FEEDBACK
    assert n._raised_pick_contact_check(6., 2.)
    n._turn_vacuum_on.assert_called_once()
    n._turn_vacuum_off.assert_not_called()
    assert n.contact_detected


def test_early_force_and_failed_step_never_activate_suction():
    n = contact_node()
    n._direct_mode_tcp_xyz = lambda: (0., 0., .19)
    assert n._raised_pick_contact_check(6., 1.)
    n._fault.assert_called_once()
    n._turn_vacuum_on.assert_not_called()
    n._release_from_direct_place('step timeout')
    n._turn_vacuum_on.assert_not_called()
    n._turn_vacuum_off.assert_not_called()


def test_deferred_lift_waits_for_ros_restoration_after_direct_pickup():
    n = contact_node()
    n.defer_pickup_lift = True
    n.state = n.VACUUM_ON
    n._restore_ros2_control_mode = Mock()
    n._proceed_to_retreat(vacuum_verified=True)
    n._restore_ros2_control_mode.assert_called_once()
    assert n.state != n.SUCCEEDED
    assert not n.raised_pick_active


def test_full_pickup_uses_direct_retreat_after_contact():
    n = contact_node()
    n.defer_pickup_lift = False
    n._resume_direct_place_recovery_retreat = Mock()
    n._proceed_to_retreat(vacuum_verified=True)
    n._resume_direct_place_recovery_retreat.assert_called_once()


@pytest.mark.parametrize('raised', [False, True])
def test_deferred_pallet_pick_reuses_slow_clearance_before_restoring(raised):
    n = contact_node()
    n.operation_kind = 'pickup'
    n.active_pickup_snapshot = dict(pickup_source='pallet', x_m=.1, y_m=-.3)
    n.raised_pick_active = raised
    n.defer_pickup_lift = True
    n.pregrasp_z = .2
    n.servo_bounds_mm = [0, 0, 0, 0, -.2, 800]
    n._tcp_xyz = lambda: (.1, -.3, .17)
    n._direct_mode_tcp_xyz = lambda: (.1, -.3, .16)
    n._resume_direct_place_recovery_retreat = Mock()
    n._disable_servo_then_direct_retreat = Mock()
    n._restore_ros2_control_mode = Mock()
    n._proceed_to_retreat(vacuum_verified=True)
    assert n.pickup_clearance_pending
    assert n.direct_target_z == pytest.approx(.2)
    assert n.pickup_clearance_start_xy == (.1, -.3)
    assert not n.defer_pickup_lift
    n._restore_ros2_control_mode.assert_not_called()
    if raised:
        n._resume_direct_place_recovery_retreat.assert_called_once()
    else:
        n._disable_servo_then_direct_retreat.assert_called_once()
    n._turn_vacuum_off.assert_not_called()


def test_pallet_clearance_verification_blocks_transfer_on_wrong_height():
    n = contact_node()
    n.pickup_clearance_pending = True
    n.post_retreat_fault = ''
    n.pickup_clearance_target_z = .2
    n.pickup_clearance_start_xy = (.1, -.3)
    n._link_tcp_xyz = lambda: (.1, -.3, .17)
    n._finish_retreat()
    n._fault.assert_called_once()
    assert 'transfer blocked' in n._fault.call_args.args[0]


def live_pick_node():
    n = contact_node()
    n.operation_id = 12
    n.operation_kind = 'pickup'
    n.probe_only = n.dry_run = False
    n.active_pickup_snapshot = dict(pickup_source='pallet', retrieval_target_id=1,
                                    x_m=.1, y_m=-.3)
    n.servo_status_time = n.last_force_time = n.robot_state_time = time.monotonic()
    n.status_timeout = n.force_timeout = 1.
    n.expected_enable_generation = 4
    n.servo_status = dict(enable_generation=4, force_delta_z_n=.1)
    n.robot_error, n.robot_mode, n.robot_state = 0, 1, 0
    n.planning_scene_status = {}
    n._tcp_xyz = lambda: (.1, -.3, .19)
    n.floor_z, n.tolerance, n.xy_tolerance = .14, .001, .01
    n._begin_place_singularity_fallback = Mock()
    return n


def test_live_known_pick_singularity_enters_guarded_recovery_without_suction():
    n = live_pick_node()
    assert n._try_live_pick_singularity_recovery('Very close to a singularity')
    n._begin_place_singularity_fallback.assert_called_once()
    n._turn_vacuum_on.assert_not_called()
    assert n.raised_pick_active and n.raised_place_hold_on_failure


def test_control_tick_routes_fresh_pick_singularity_into_recovery():
    n = live_pick_node()
    n.state = n.DESCENDING
    n._tick_descent_config = lambda: False
    n.servo_status.update(state='FAULT', fault='Very close to a singularity, emergency stop')
    n.control_tick()
    n._begin_place_singularity_fallback.assert_called_once()
    n._fault.assert_not_called()


def soft_pick_node(source='pallet'):
    n = live_pick_node()
    n.active_pickup_snapshot['pickup_source'] = source
    n.state = n.DESCENDING
    n._tick_descent_config = lambda: False
    n._publish_descent_target = Mock()
    n._handle_contact = Mock()
    n.get_logger = Mock(return_value=Mock())
    n.servo_status.update(state='RUNNING', fault='', servo_status=1)
    n.descent_started = time.monotonic()
    n.descent_timeout = 30.
    n.descent_target_z = n.floor_z
    n.singularity_deceleration_started = None
    n.singularity_progress_started = None
    n.singularity_progress_reference_z = None
    n.singularity_deceleration_grace = .75
    n.singularity_no_progress = 1.
    n.singularity_min_progress = .001
    return n


@pytest.mark.parametrize('source', ['pallet', 'buffer'])
def test_known_pick_soft_singularity_recovers_without_waiting_for_30s(source):
    n = soft_pick_node(source)
    n.control_tick()
    n._begin_place_singularity_fallback.assert_not_called()
    n.singularity_deceleration_started -= .8
    n.control_tick()
    n._begin_place_singularity_fallback.assert_called_once()
    n._turn_vacuum_on.assert_not_called()
    n._turn_vacuum_off.assert_not_called()
    n._fault.assert_not_called()


def test_transient_pick_slowdown_resets_debounce():
    n = soft_pick_node()
    n.control_tick()
    n.singularity_deceleration_started -= .8
    n.servo_status['servo_status'] = 0
    n.control_tick()
    assert n.singularity_deceleration_started is None
    n.servo_status['servo_status'] = 1
    n.control_tick()
    n._begin_place_singularity_fallback.assert_not_called()


@pytest.mark.parametrize('invalid', ['incoming', 'probe', 'generation', 'force_stale'])
def test_soft_pick_recovery_does_not_bypass_interlocks(invalid):
    n = soft_pick_node()
    n.singularity_deceleration_started = time.monotonic()-1.
    if invalid == 'incoming': n.active_pickup_snapshot['pickup_source'] = 'incoming'
    if invalid == 'probe': n.probe_only = True
    if invalid == 'generation': n.servo_status['enable_generation'] = 3
    if invalid == 'force_stale': n.last_force_time = time.monotonic()-5.
    n.control_tick()
    n._begin_place_singularity_fallback.assert_not_called()
    n._turn_vacuum_on.assert_not_called()
    if invalid == 'force_stale': n._fault.assert_called_once()


def test_pick_contact_takes_priority_over_slowdown_recovery():
    n = soft_pick_node()
    n.singularity_deceleration_started = time.monotonic()-1.
    n.servo_status['touch_contact'] = True
    n.control_tick()
    n._handle_contact.assert_called_once()
    n._begin_place_singularity_fallback.assert_not_called()


def test_pick_singularity_timeout_uses_guarded_recovery_as_backstop():
    n = soft_pick_node()
    n.descent_started -= 31.
    n.control_tick()
    n._begin_place_singularity_fallback.assert_called_once()
    n._fault.assert_not_called()


def pause_gate_node(monkeypatch):
    clock = [100.]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    n = soft_pick_node()
    n.robot_state = 1
    n.latest_force_z = n.direct_place_force_baseline_z = .1
    n.latest_joint_positions = (.1, .2)
    n.last_joint_state_time = 100.
    assert n._try_live_pick_singularity_recovery('singularity deceleration')
    n.state = n.DISABLING_SERVO
    n._complete_direct_place_pause = Mock()
    n._servo_disabled_for_direct_place(NS(result=lambda: NS(success=True)))
    assert n.live_pick_pause_gate['acknowledged'] == 100.
    n.servo_status['force_delta_z_n'] = None  # bridge clears this on disable
    return n, clock


def pause_sample(n, clock, at, state=2, joints=(.1, .2)):
    clock[0] = at
    n.robot_state = state
    n.latest_joint_positions = joints
    n.servo_status_time = n.robot_state_time = n.last_force_time = n.last_joint_state_time = at
    assert n._live_pick_pause_tick()


def test_moving_pick_pauses_then_waits_for_fresh_stable_stop(monkeypatch):
    n, clock = pause_gate_node(monkeypatch)
    n._complete_direct_place_pause.assert_not_called()  # pause ACK alone is insufficient
    pause_sample(n, clock, 100.1, state=1)
    n._complete_direct_place_pause.assert_not_called()
    pause_sample(n, clock, 100.2)
    pause_sample(n, clock, 100.4)
    n._complete_direct_place_pause.assert_not_called()
    pause_sample(n, clock, 100.5)
    n._complete_direct_place_pause.assert_called_once()
    assert n.live_pick_pause_gate is None and n.direct_tcp_z_offset is None
    n._fault.assert_not_called()
    n._turn_vacuum_on.assert_not_called()
    n._turn_vacuum_off.assert_not_called()


def test_pause_gate_needs_post_ack_feedback_and_restarts_on_joint_motion(monkeypatch):
    n, clock = pause_gate_node(monkeypatch)
    n.robot_state = 2
    clock[0] = 100.3
    n._live_pick_pause_tick()  # all feedback is pre-ACK
    assert n.live_pick_pause_gate['stable_since'] is None
    pause_sample(n, clock, 100.4)
    pause_sample(n, clock, 100.6, joints=(.11, .2))
    pause_sample(n, clock, 100.7, joints=(.11, .2))
    n._complete_direct_place_pause.assert_not_called()
    pause_sample(n, clock, 100.9, joints=(.11, .2))
    n._complete_direct_place_pause.assert_called_once()


@pytest.mark.parametrize('failure,detail', [
    ('timeout', 'timeout'), ('hardware', 'error=52'), ('force_stale', 'force telemetry stale'),
    ('contact', 'contact/load developed'), ('source', 'source changed'),
    ('generation', 'wrong Servo generation'), ('joint_stale', 'joint telemetry stale'),
])
def test_pause_gate_failures_never_handoff_or_grip(monkeypatch, failure, detail):
    n, clock = pause_gate_node(monkeypatch)
    clock[0] = 100.1
    if failure == 'timeout': n.live_pick_pause_gate['deadline'] = 100.
    if failure == 'hardware': n.robot_error = 52
    if failure == 'force_stale': n.last_force_time = 90.
    if failure == 'contact': n.latest_force_z = 10.
    if failure == 'source': n.active_pickup_snapshot['retrieval_target_id'] = 5
    if failure == 'generation': n.servo_status['enable_generation'] = 1
    if failure == 'joint_stale': n.last_joint_state_time = 90.
    assert n._live_pick_pause_tick()
    n._fault.assert_called_once()
    assert detail in n._fault.call_args.args[0]
    n._complete_direct_place_pause.assert_not_called()
    n._turn_vacuum_on.assert_not_called()
    n._turn_vacuum_off.assert_not_called()


def test_late_pause_ack_does_not_restart_aborted_pick(monkeypatch):
    n, clock = pause_gate_node(monkeypatch)
    n.state = n.FAULT
    n._servo_disabled_for_direct_place(NS(result=lambda: NS(success=True)))
    assert not n._live_pick_pause_tick()
    n._complete_direct_place_pause.assert_not_called()


def test_real_pick_recovery_requests_pause_before_tcp_capture_and_mode_handoff():
    from test_transport_alternatives import Client
    n = soft_pick_node()
    n.robot_state = 1
    n.direct_place_recovery_active = n.place_fallback_used = False
    n.singularity_place_step = .003
    n.latest_force_z = .1
    n.enable_client = Client()
    n.publish_status = Mock()
    n._capture_direct_tcp_z_offset = Mock()
    n._deactivate_retreat_controllers = Mock()
    n._begin_place_singularity_fallback = PickupSupervisor._begin_place_singularity_fallback.__get__(n)
    assert n._try_live_pick_singularity_recovery('singularity deceleration')
    request, future = n.enable_client.calls[-1]
    assert request.data is False
    n._capture_direct_tcp_z_offset.assert_not_called()
    n._deactivate_retreat_controllers.assert_not_called()
    n.operation_id += 1
    future.value = NS(success=True)
    future.callback(future)
    assert n.live_pick_pause_gate['acknowledged'] is None
    n._deactivate_retreat_controllers.assert_not_called()


@pytest.mark.parametrize('invalid', ['generation', 'force', 'hardware', 'floor', 'telemetry'])
def test_live_pick_recovery_rejects_unsafe_handoff(invalid):
    n = live_pick_node()
    if invalid == 'generation': n.servo_status['enable_generation'] = 3
    if invalid == 'force': n.servo_status['force_delta_z_n'] = 6.
    if invalid == 'hardware': n.robot_error = 52
    if invalid == 'floor': n.floor_z = .19
    if invalid == 'telemetry': n.last_force_time = time.monotonic()-5
    assert n._try_live_pick_singularity_recovery('singularity')
    n._fault.assert_called_once()
    n._begin_place_singularity_fallback.assert_not_called()
    n._turn_vacuum_on.assert_not_called()


@pytest.mark.parametrize('case', ['incoming', 'probe', 'other_fault'])
def test_live_pick_recovery_does_not_bypass_other_faults(case):
    n = live_pick_node()
    if case == 'incoming': n.active_pickup_snapshot['pickup_source'] = 'incoming'
    if case == 'probe': n.probe_only = True
    reason = 'collision stop' if case == 'other_fault' else 'singularity'
    assert not n._try_live_pick_singularity_recovery(reason)
    n._begin_place_singularity_fallback.assert_not_called()
