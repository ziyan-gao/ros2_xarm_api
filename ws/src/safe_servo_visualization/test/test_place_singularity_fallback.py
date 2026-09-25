import math
import time

from geometry_msgs.msg import WrenchStamped
from moveit_msgs.msg import ServoStatus
import pytest

from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class FakeClock:
    class Now:
        nanoseconds = 10_000_000_000

    def now(self):
        return self.Now()


class PendingFuture:
    def __init__(self):
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback


class RecordingClient:
    def __init__(self):
        self.requests = []
        self.future = PendingFuture()

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def _descending_supervisor(operation_kind, fault=None):
    supervisor = object.__new__(PickupSupervisor)
    supervisor.state = PickupSupervisor.DESCENDING
    supervisor.operation_kind = operation_kind
    supervisor.servo_status = {
        'state': 'FAULT',
        'fault': fault or (
            'MoveIt Servo halted: Very close to a singularity, emergency stop'),
    }
    supervisor._tick_descent_config = lambda: False
    return supervisor


def test_raised_place_failure_never_releases_item():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.direct_place_recovery_active = True
    supervisor.raised_place_hold_on_failure = True
    faults = []
    releases = []
    supervisor._fault = faults.append
    supervisor._turn_vacuum_off = lambda: releases.append(True)
    supervisor._release_from_direct_place('force telemetry stale')
    assert faults == ['raised pre-place descent stopped: force telemetry stale; item remains held']
    assert releases == []


@pytest.mark.parametrize('stale', [False, True])
def test_raised_place_handoff_preserves_floor_and_is_single_use(stale):
    from std_srvs.srv import Trigger
    supervisor = object.__new__(PickupSupervisor)
    supervisor._ft_recovery_blocks_start = lambda response: False
    supervisor.manual_gripper_pending = False
    supervisor.state = supervisor.SUCCEEDED
    supervisor.motion_status = dict(target='transfer', state='SUCCEEDED', operation_id=7,
                                    transfer_tcp_z_m=.6)
    supervisor.direct_transfer_succeeded = True
    supervisor.pallet_locked = True
    supervisor.planning_scene_status = dict(attached_item_id='box')
    supervisor.enable_client = RecordingClient()
    supervisor._tcp_xyz = lambda: (.1, -.3, .21)
    supervisor.raised_pre_place_ready = dict(target_id=6 if stale else 7, item_id='box',
                                            xyz=[.1, -.3, .21], original_z=.2)
    supervisor.last_force_time = time.monotonic()
    supervisor.force_timeout = 1.
    supervisor.operation_id = 10
    supervisor.tolerance = .001
    supervisor.place_workspace_z_min_mm = -200.
    supervisor.max_descent = .04
    supervisor.minimum_contact_descent = .001
    calls = []
    supervisor._begin_place_singularity_fallback = calls.append
    supervisor._reset_servo_then_begin_place = lambda *args: pytest.fail('unexpected Servo descent')
    response = supervisor.start_place_callback(None, Trigger.Response())
    if stale:
        assert not response.success
        assert not calls
        assert supervisor.operation_id == 10
    else:
        assert response.success
        assert supervisor.floor_z == pytest.approx(.16)
        assert supervisor.raised_pre_place_ready is None
        assert supervisor.raised_retreat_pose == dict(
            target_id=7, xyz=[.1, -.3, .21], original_xyz=[.1, -.3, .2])
        assert supervisor.raised_place_hold_on_failure
        assert calls == ['validated raised pre-place approach']


def test_place_singularity_uses_release_and_retreat_fallback():
    supervisor = _descending_supervisor('place')
    fallbacks = []
    faults = []
    supervisor._begin_place_singularity_fallback = fallbacks.append
    supervisor._fault = faults.append

    supervisor.control_tick()

    assert len(fallbacks) == 1
    assert 'singularity' in fallbacks[0]
    assert faults == []


def test_pickup_singularity_remains_a_hard_fault():
    supervisor = _descending_supervisor('pickup')
    fallbacks = []
    faults = []
    supervisor._begin_place_singularity_fallback = fallbacks.append
    supervisor._fault = faults.append

    supervisor.control_tick()

    assert fallbacks == []
    assert len(faults) == 1
    assert 'singularity' in faults[0]


def test_place_external_wrench_limit_holds_item_instead_of_releasing():
    supervisor = _descending_supervisor(
        'place',
        'external wrench safety limit: force=12.12 N, delta_fz=11.37 N')
    fallbacks = []
    faults = []
    supervisor._begin_place_release_fallback = (
        lambda reason, cause: fallbacks.append((reason, cause)))
    supervisor._fault = faults.append

    supervisor.control_tick()

    assert fallbacks == []
    assert len(faults) == 1
    assert 'external wrench safety limit' in faults[0]


def test_pickup_external_wrench_limit_remains_a_hard_fault():
    supervisor = _descending_supervisor(
        'pickup',
        'external wrench safety limit: force=12.12 N, delta_fz=11.37 N')
    fallbacks = []
    faults = []
    supervisor._begin_place_release_fallback = (
        lambda reason, cause: fallbacks.append((reason, cause)))
    supervisor._fault = faults.append

    supervisor.control_tick()

    assert fallbacks == []
    assert len(faults) == 1
    assert 'external wrench safety limit' in faults[0]


def test_singularity_recovery_records_fallback_and_disables_servo_first():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.operation_id = 1
    supervisor.place_fallback_used = False
    supervisor.place_fallback_reason = ''
    supervisor.direct_place_recovery_active = False
    supervisor.direct_place_stepping = False
    supervisor.direct_place_deadline = None
    supervisor.direct_place_force_baseline_z = None
    supervisor.direct_place_step_count = 0
    supervisor.latest_force_z = -8.0
    supervisor.descent_started = time.monotonic()
    supervisor.place_descent_timeout = 20.0
    supervisor.singularity_place_recovery_timeout = 20.0
    supervisor.singularity_place_step = 0.003
    supervisor.enable_client = RecordingClient()
    supervisor.get_logger = lambda: FakeLogger()
    supervisor.direct_tcp_z_offset = None
    supervisor._capture_direct_tcp_z_offset = lambda: setattr(
        supervisor, 'direct_tcp_z_offset', 0.024)
    published = []
    supervisor.publish_status = lambda: published.append(True)

    supervisor._begin_place_singularity_fallback('singularity test')

    assert supervisor.place_fallback_used is True
    assert supervisor.place_fallback_reason == 'singularity test'
    assert supervisor.direct_place_recovery_active is True
    assert supervisor.direct_place_force_baseline_z == -8.0
    # The recovery budget must not be consumed by controller/mode handoff.
    assert supervisor.direct_place_deadline is None
    assert supervisor.state == PickupSupervisor.DISABLING_SERVO
    assert len(supervisor.enable_client.requests) == 1
    assert supervisor.enable_client.requests[0].data is False
    assert published == [True]


def _direct_step_supervisor(current_z=0.100, floor_z=0.080):
    supervisor = object.__new__(PickupSupervisor)
    supervisor.direct_place_stepping = True
    supervisor.direct_place_recovery_active = True
    supervisor.direct_place_deadline = time.monotonic() + 10.0
    supervisor.last_force_time = time.monotonic()
    supervisor.force_timeout = 1.0
    supervisor.status_timeout = 1.0
    supervisor.latest_force_z = -8.0
    supervisor.direct_place_force_baseline_z = -8.0
    supervisor.place_force_threshold = 4.0
    supervisor.floor_z = floor_z
    supervisor.direct_tcp_z_offset = 0.0
    supervisor.tolerance = 0.0005
    supervisor.singularity_place_step = 0.003
    supervisor.singularity_place_step_speed = 10.0
    supervisor.retreat_acc = 100.0
    supervisor.retreat_client = RecordingClient()
    supervisor._direct_mode_tcp_xyz = lambda: (0.4, -0.2, current_z)
    supervisor._release_from_direct_place = lambda *_args, **_kwargs: None
    supervisor.direct_motion_generation = 0
    return supervisor


def test_direct_singularity_recovery_commands_one_relative_3mm_step():
    supervisor = _direct_step_supervisor()

    supervisor._send_direct_place_step()

    assert supervisor.state == PickupSupervisor.RETREATING
    assert len(supervisor.retreat_client.requests) == 1
    request = supervisor.retreat_client.requests[0]
    assert request.relative is True
    assert request.wait is True
    assert all(math.isclose(actual, expected, abs_tol=1e-6) for actual, expected in zip(
        request.pose, [0.0, 0.0, -3.0, 0.0, 0.0, 0.0]))
    assert request.speed == 10.0


def test_direct_singularity_recovery_clamps_last_step_to_floor():
    supervisor = _direct_step_supervisor(current_z=0.0575, floor_z=0.080)
    supervisor.direct_tcp_z_offset = 0.024

    supervisor._send_direct_place_step()

    request = supervisor.retreat_client.requests[0]
    assert math.isclose(request.pose[2], -1.5)
    assert math.isclose(supervisor.retreat_target_z, 0.056)


def test_direct_singularity_recovery_releases_before_step_on_contact():
    supervisor = _direct_step_supervisor()
    supervisor.latest_force_z = -12.5
    releases = []
    supervisor._release_from_direct_place = (
        lambda reason, contact_detected=False:
        releases.append((reason, contact_detected)))

    supervisor._send_direct_place_step()

    assert supervisor.retreat_client.requests == []
    assert len(releases) == 1
    assert releases[0][1] is True
    assert 'contact' in releases[0][0]


def test_completed_direct_step_waits_for_new_force_and_tcp_samples():
    supervisor = _direct_step_supervisor()
    supervisor.state = PickupSupervisor.RETREATING
    supervisor.direct_motion_generation = 3
    supervisor.direct_place_step_count = 0
    supervisor.robot_state_time = time.monotonic()

    class SuccessFuture:
        class Result:
            ret = 0

        def result(self):
            return self.Result()

    supervisor._direct_place_step_completed(SuccessFuture(), 3)

    assert supervisor.state == PickupSupervisor.WAITING_PLACE_STEP_FEEDBACK
    assert supervisor.direct_place_step_count == 1
    completed_at = supervisor.direct_place_step_completed_at
    next_steps = []
    supervisor._send_direct_place_step = lambda: next_steps.append(True)

    supervisor._direct_place_step_feedback_tick()
    assert next_steps == []

    supervisor.last_force_time = completed_at + 0.001
    supervisor.robot_state_time = completed_at + 0.001
    supervisor._direct_place_step_feedback_tick()
    assert next_steps == [True]


def _timed_out_soft_singularity_supervisor(operation_kind, servo_status):
    supervisor = object.__new__(PickupSupervisor)
    supervisor.state = PickupSupervisor.DESCENDING
    supervisor.operation_kind = operation_kind
    supervisor.servo_status = {
        'state': 'RUNNING',
        'fault': '',
        'servo_status': servo_status,
        'enable_generation': 2,
    }
    supervisor.expected_enable_generation = 2
    supervisor.dry_run = False
    supervisor.pregrasp_z = 0.06
    supervisor.floor_z = -0.09
    supervisor.descent_target_z = -0.09
    supervisor.tolerance = 0.001
    supervisor.descent_started = time.monotonic() - 31.0
    supervisor.descent_timeout = 30.0
    supervisor.place_descent_timeout = 20.0
    supervisor._tick_descent_config = lambda: False
    supervisor._publish_descent_target = lambda: None
    supervisor._tcp_xyz = lambda: (0.30, -0.20, 0.03)
    supervisor._contact_reached = lambda: False
    return supervisor


def test_place_timeout_during_soft_singularity_uses_fallback():
    supervisor = _timed_out_soft_singularity_supervisor(
        'place', ServoStatus.DECELERATE_FOR_APPROACHING_SINGULARITY)
    fallbacks = []
    faults = []
    supervisor._begin_place_singularity_fallback = fallbacks.append
    supervisor._fault = faults.append

    supervisor.control_tick()

    assert len(fallbacks) == 1
    assert 'singularity deceleration status 1' in fallbacks[0]
    assert faults == []


def test_pickup_timeout_during_soft_singularity_remains_hard_fault():
    supervisor = _timed_out_soft_singularity_supervisor(
        'pickup', ServoStatus.DECELERATE_FOR_APPROACHING_SINGULARITY)
    fallbacks = []
    faults = []
    supervisor._begin_place_singularity_fallback = fallbacks.append
    supervisor._fault = faults.append

    supervisor.control_tick()

    assert fallbacks == []
    assert len(faults) == 1
    assert 'timed out' in faults[0]


def test_place_timeout_without_singularity_releases_and_retreats():
    supervisor = _timed_out_soft_singularity_supervisor(
        'place', ServoStatus.NO_WARNING)
    fallbacks = []
    faults = []
    supervisor._begin_place_release_fallback = (
        lambda reason, cause: fallbacks.append((reason, cause)))
    supervisor._fault = faults.append

    supervisor.control_tick()

    assert len(fallbacks) == 1
    assert 'timed out' in fallbacks[0][0]
    assert fallbacks[0][1] == 'timeout'
    assert faults == []


def test_persistent_singularity_deceleration_falls_back_before_place_timeout():
    supervisor = _timed_out_soft_singularity_supervisor(
        'place', ServoStatus.DECELERATE_FOR_APPROACHING_SINGULARITY)
    supervisor.descent_started = time.monotonic()
    supervisor.singularity_deceleration_started = None
    supervisor.singularity_progress_started = None
    supervisor.singularity_progress_reference_z = None
    supervisor.singularity_deceleration_grace = 0.75
    supervisor.singularity_no_progress = 1.0
    supervisor.singularity_min_progress = 0.001
    fallbacks = []
    supervisor._begin_place_singularity_fallback = fallbacks.append
    supervisor._fault = lambda _reason: None

    supervisor.control_tick()
    assert fallbacks == []

    supervisor.singularity_deceleration_started -= 0.8
    supervisor.control_tick()
    assert len(fallbacks) == 1
    assert '0.80 s' in fallbacks[0]


def test_step_recovery_budget_starts_after_controller_handoff():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.state = PickupSupervisor.PREPARING_RETREAT
    supervisor.retreat_controller_wait_deadline = time.monotonic() + 2.0
    supervisor.retreat_controller_query_pending = True
    supervisor.joint_state_broadcaster = 'joint_state_broadcaster'
    supervisor.trajectory_controller = 'uf850_traj_controller'
    supervisor.direct_place_stepping = True
    supervisor.direct_place_deadline = None
    supervisor.singularity_place_recovery_timeout = 20.0
    supervisor.get_logger = lambda: FakeLogger()
    sent = []
    supervisor._send_direct_place_step = lambda: sent.append(True)
    supervisor._fault = lambda reason: (_ for _ in ()).throw(AssertionError(reason))

    class ResultFuture:
        def result(self):
            class Result:
                controller = []
            return Result()

    supervisor._retreat_controller_list_completed(ResultFuture())

    assert sent == [True]
    assert supervisor.direct_place_deadline > time.monotonic() + 19.0


def test_latched_place_fault_does_not_reject_next_safe_pregrasp():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.motion_status = {
        'state': 'SUCCEEDED',
        'target': 'pregrasp_box_7',
        'planned_pregrasp': {
            'box_id': 7,
            'planned_stamp_sec': 9.0,
            'x_m': 0.50,
            'y_m': -0.10,
            'pregrasp_z_m': 0.30,
            'top_z_m': 0.20,
        },
    }
    supervisor.servo_status = {
        'state': 'FAULT',
        'fault': (
            'MoveIt Servo halted: Very close to a singularity, emergency stop'),
    }
    supervisor.max_pregrasp_plan_age = 5.0
    supervisor.xy_tolerance = 0.01
    supervisor.pregrasp_z_tolerance = 0.01
    supervisor.retrieval_pregrasp_above_tolerance = 0.05
    supervisor.grasp_offset = 0.0
    supervisor.servo_bounds_mm = [-1000, 1000, -1000, 1000, -1000, 1000]
    supervisor.contact_search_margin = 0.02
    supervisor.max_descent = 0.20
    supervisor.get_clock = lambda: FakeClock()
    supervisor._tcp_xyz = lambda: (0.50, -0.10, 0.30)

    snapshot, start_z, floor_z = supervisor._validate_pregrasp_ready()

    assert snapshot['box_id'] == 7
    assert start_z == 0.30
    assert math.isclose(floor_z, 0.18)


def _low_pallet_pregrasp_supervisor(*, retrieval):
    supervisor = object.__new__(PickupSupervisor)
    snapshot = {
        'box_id': 1005,
        'planned_stamp_sec': 9.0,
        'x_m': 0.128,
        'y_m': -0.591,
        'pregrasp_z_m': -0.021,
        'top_z_m': -0.051,
    }
    if retrieval:
        snapshot['retrieval_target_id'] = 5
    supervisor.motion_status = {
        'state': 'SUCCEEDED',
        'target': 'pregrasp_box_1005',
        'planned_pregrasp': snapshot,
    }
    supervisor.max_pregrasp_plan_age = 5.0
    supervisor.xy_tolerance = 0.01
    supervisor.pregrasp_z_tolerance = 0.01
    supervisor.retrieval_pregrasp_above_tolerance = 0.05
    supervisor.grasp_offset = 0.0
    supervisor.servo_bounds_mm = [-1000, 1000, -1000, 1000, 50, 1000]
    supervisor.place_workspace_z_min_mm = -200.0
    supervisor.contact_search_margin = 0.015
    supervisor.max_descent = 0.15
    supervisor.get_clock = lambda: FakeClock()
    supervisor._tcp_xyz = lambda: (0.128, -0.591, -0.021)
    return supervisor


def test_pallet_retrieval_uses_low_workspace_floor_for_servo_pickup():
    supervisor = _low_pallet_pregrasp_supervisor(retrieval=True)

    _snapshot, start_z, floor_z = supervisor._validate_pregrasp_ready()

    assert start_z == pytest.approx(-0.021)
    assert floor_z == pytest.approx(-0.066)
    assert start_z - floor_z == pytest.approx(0.045)


def test_pallet_retrieval_accepts_moderately_higher_live_pregrasp():
    supervisor = _low_pallet_pregrasp_supervisor(retrieval=True)
    supervisor._tcp_xyz = lambda: (0.128, -0.591, 0.0003)

    _snapshot, start_z, floor_z = supervisor._validate_pregrasp_ready()

    assert start_z == pytest.approx(0.0003)
    assert floor_z == pytest.approx(-0.066)


def test_pallet_retrieval_still_rejects_live_pose_below_pregrasp():
    supervisor = _low_pallet_pregrasp_supervisor(retrieval=True)
    supervisor._tcp_xyz = lambda: (0.128, -0.591, -0.042)

    with pytest.raises(ValueError, match='verified pre-grasp height'):
        supervisor._validate_pregrasp_ready()


def test_normal_pickup_keeps_positive_workspace_floor():
    supervisor = _low_pallet_pregrasp_supervisor(retrieval=False)

    with pytest.raises(ValueError, match='pickup descent -0.071 m'):
        supervisor._validate_pregrasp_ready()


def test_tcp_validation_uses_link_tcp_servo_pose_when_fresh():
    supervisor = object.__new__(PickupSupervisor)
    now = time.monotonic()
    supervisor.status_timeout = 1.0
    supervisor.robot_tcp_xyz = (-0.2682, 0.2677, 0.1492)
    supervisor.robot_state_time = now
    supervisor.servo_status_time = now
    supervisor.last_joint_state_time = now
    supervisor.servo_status = {
        'tcp_x_m': 0.10,
        'tcp_y_m': 0.20,
        'tcp_z_m': 0.30,
        'joint_state_age_sec': 0.0,
    }

    assert supervisor._tcp_xyz() == (0.10, 0.20, 0.30)


def test_direct_handoff_captures_link_tcp_to_sdk_tcp_z_offset():
    supervisor = object.__new__(PickupSupervisor)
    now = time.monotonic()
    supervisor.status_timeout = 1.0
    supervisor.robot_tcp_xyz = (0.10, 0.20, 0.276)
    supervisor.robot_state_time = now
    supervisor.servo_status_time = now
    supervisor.last_joint_state_time = now
    supervisor.servo_status = {
        'tcp_x_m': 0.10,
        'tcp_y_m': 0.20,
        'tcp_z_m': 0.300,
        'joint_state_age_sec': 0.0,
    }
    supervisor.get_logger = lambda: FakeLogger()

    offset = supervisor._capture_direct_tcp_z_offset()

    assert offset == pytest.approx(0.024)
    assert supervisor.direct_tcp_z_offset == pytest.approx(0.024)


def test_linear_loading_contact_requires_consecutive_delta_fz_samples():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.operation_kind = 'loading'
    supervisor.state = PickupSupervisor.RETREATING
    supervisor.loading_contact_fallback = False
    supervisor.loading_force_baseline_z = -8.0
    supervisor.loading_force_over_count = 0
    supervisor.loading_contact_confirm_samples = 2
    supervisor.place_force_threshold = 4.0
    supervisor.retreat_start_z = 0.50
    supervisor.retreat_target_z = 0.30
    supervisor.tolerance = 0.001
    contacts = []
    supervisor._begin_loading_contact_fallback = contacts.append
    message = WrenchStamped()
    message.wrench.force.z = -12.5

    supervisor.force_callback(message)
    assert contacts == []

    supervisor.force_callback(message)
    assert contacts == [4.5]


def test_interrupted_loading_response_cannot_finish_fallback_retreat():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.state = PickupSupervisor.RETREATING
    supervisor.direct_motion_generation = 4

    class StaleFuture:
        def result(self):
            raise AssertionError('stale direct-motion response was consumed')

    supervisor._direct_retreat_completed(StaleFuture(), 3)

    assert supervisor.state == PickupSupervisor.RETREATING
