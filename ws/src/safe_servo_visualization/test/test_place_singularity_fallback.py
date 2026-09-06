import math
import time

from geometry_msgs.msg import WrenchStamped
from moveit_msgs.msg import ServoStatus

from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor


class FakeLogger:
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


def _descending_supervisor(operation_kind):
    supervisor = object.__new__(PickupSupervisor)
    supervisor.state = PickupSupervisor.DESCENDING
    supervisor.operation_kind = operation_kind
    supervisor.servo_status = {
        'state': 'FAULT',
        'fault': (
            'MoveIt Servo halted: Very close to a singularity, emergency stop'),
    }
    supervisor._tick_descent_config = lambda: False
    return supervisor


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


def test_singularity_recovery_records_fallback_and_disables_servo_first():
    supervisor = object.__new__(PickupSupervisor)
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
    published = []
    supervisor.publish_status = lambda: published.append(True)

    supervisor._begin_place_singularity_fallback('singularity test')

    assert supervisor.place_fallback_used is True
    assert supervisor.place_fallback_reason == 'singularity test'
    assert supervisor.direct_place_recovery_active is True
    assert supervisor.direct_place_force_baseline_z == -8.0
    assert supervisor.direct_place_deadline > time.monotonic() + 19.0
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
    supervisor = _direct_step_supervisor(current_z=0.0815, floor_z=0.080)

    supervisor._send_direct_place_step()

    request = supervisor.retreat_client.requests[0]
    assert math.isclose(request.pose[2], -1.5)
    assert math.isclose(supervisor.retreat_target_z, 0.080)


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
