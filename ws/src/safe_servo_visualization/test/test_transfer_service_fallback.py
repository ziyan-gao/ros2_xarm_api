import math
import time
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from safe_servo_visualization.motion_coordinator_node import MotionCoordinator
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from safe_servo_visualization.place_pipeline_node import PlacePipeline


class FakeFuture:
    def __init__(self, result=None):
        self._result = result
        self.callback = None

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self.callback = callback


class FakeClient:
    def __init__(self, result=None):
        self.ready = True
        self.result = result
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = FakeFuture(self.result)
        self.futures.append(future)
        return future


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


def test_direct_transfer_uses_absolute_xarm_pose_and_firmware_ik_fallback():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.operation_kind = 'transfer'
    supervisor.direct_target_pose = (
        410.0, -275.0, 620.0, math.pi, 0.0, -math.pi / 2.0)
    supervisor.direct_target_z = 0.620
    supervisor.retreat_start_z = 0.610
    supervisor.retreat_speed = 75.0
    supervisor.retreat_acc = 200.0
    supervisor.retreat_timeout = 120.0
    supervisor.retreat_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.set_state_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.state = PickupSupervisor.PREPARING_RETREAT
    supervisor.retreat_started = None
    supervisor.retreat_target_z = None
    supervisor.direct_motion_generation = 3
    supervisor.fault = ''
    supervisor.get_logger = lambda: FakeLogger()
    supervisor.publish_status = lambda: None
    supervisor._fault = lambda reason: setattr(supervisor, 'fault', reason)

    supervisor._send_direct_retreat()

    request = supervisor.retreat_client.requests[-1]
    assert request.pose == pytest.approx(supervisor.direct_target_pose)
    assert request.relative is False
    assert request.motion_type == 1
    assert request.wait is True
    assert request.timeout == pytest.approx(120.0)
    assert supervisor.state == PickupSupervisor.RETREATING
    assert supervisor.retreat_target_z == pytest.approx(0.620)
    assert supervisor.fault == ''


def test_direct_transfer_requires_tcp_to_reach_the_requested_xyz():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.operation_kind = 'transfer'
    supervisor.direct_target_pose = (410.0, -275.0, 620.0, math.pi, 0.0, 0.0)
    supervisor.direct_transfer_succeeded = False
    supervisor.dry_run = False
    supervisor.post_retreat_fault = ''
    supervisor.xy_tolerance = 0.015
    supervisor.pregrasp_z_tolerance = 0.015
    supervisor.joint6_name = 'joint6'
    supervisor._joint6_is_moveit_safe = lambda: True
    supervisor._direct_mode_tcp_xyz = lambda: (0.440, -0.275, 0.620)
    supervisor.publish_status = lambda: None
    supervisor.fault = ''
    supervisor._fault = lambda reason: setattr(supervisor, 'fault', reason)

    supervisor._finish_retreat()

    assert supervisor.direct_transfer_succeeded is False
    assert 'target error is too large' in supervisor.fault


def _direct_loading_supervisor():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.operation_kind = 'loading'
    supervisor.loading_contact_fallback = False
    supervisor.direct_target_pose = None
    supervisor.direct_target_z = 0.300
    supervisor.retreat_start_z = 0.470
    supervisor.retreat_speed = 75.0
    supervisor.retreat_acc = 200.0
    supervisor.retreat_timeout = 120.0
    supervisor.place_descent_timeout = 20.0
    supervisor.tolerance = 0.001
    supervisor.retreat_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.set_state_client = FakeClient(SimpleNamespace(ret=0))
    supervisor.state = PickupSupervisor.PREPARING_RETREAT
    supervisor.retreat_started = None
    supervisor.retreat_target_z = None
    supervisor.loading_stop_started = None
    supervisor.loading_stop_action = ''
    supervisor.direct_motion_generation = 0
    supervisor.fault = ''
    supervisor.get_logger = lambda: FakeLogger()
    supervisor.publish_status = lambda: None
    supervisor._fault = lambda reason: setattr(supervisor, 'fault', reason)
    return supervisor


def test_downward_loading_service_is_nonblocking_so_stop_remains_callable():
    supervisor = _direct_loading_supervisor()

    supervisor._send_direct_retreat()

    request = supervisor.retreat_client.requests[-1]
    assert request.relative is True
    assert request.wait is False
    assert supervisor.state == PickupSupervisor.RETREATING


def test_nonblocking_loading_reaching_target_restores_ros_control():
    supervisor = _direct_loading_supervisor()
    supervisor.state = PickupSupervisor.RETREATING
    supervisor.retreat_started = time.monotonic()
    supervisor.pre_descent_wait_callback = None
    supervisor.mode_wait_target = None
    supervisor._direct_mode_tcp_xyz = lambda: (0.4, -0.2, 0.3005)
    restored = []
    supervisor._restore_ros2_control_mode = lambda: restored.append(True)

    supervisor.retreat_tick()

    assert restored == []
    assert supervisor.retreat_started is None
    assert supervisor.state == PickupSupervisor.STOPPING_LOADING
    assert supervisor.set_state_client.requests[-1].data == 3

    stop_future = supervisor.set_state_client.futures[-1]
    stop_future.callback(stop_future)
    assert restored == [True]


def test_nonblocking_loading_timeout_starts_release_fallback():
    supervisor = _direct_loading_supervisor()
    supervisor.state = PickupSupervisor.RETREATING
    supervisor.retreat_started = time.monotonic() - 21.0
    supervisor.pre_descent_wait_callback = None
    supervisor.mode_wait_target = None
    supervisor._direct_mode_tcp_xyz = lambda: (0.4, -0.2, 0.420)
    fallbacks = []
    supervisor._begin_loading_release_fallback = (
        lambda reason, contact_detected=False:
        fallbacks.append((reason, contact_detected)))

    supervisor.retreat_tick()

    assert len(fallbacks) == 1
    assert 'exceeded 20.0 s' in fallbacks[0][0]
    assert fallbacks[0][1] is False


def test_stopping_loading_has_a_bounded_wait():
    supervisor = _direct_loading_supervisor()
    supervisor.state = PickupSupervisor.STOPPING_LOADING
    supervisor.loading_stop_started = time.monotonic() - 21.0
    supervisor.pre_descent_wait_callback = None
    supervisor.mode_wait_target = None

    supervisor.retreat_tick()

    assert 'timed out waiting for xArm to stop' in supervisor.fault
    assert supervisor.loading_stop_started is None


def _failed_transfer_pipeline(pending_motion):
    pipeline = object.__new__(PlacePipeline)
    pipeline.state = PlacePipeline.MOVE_TRANSFER
    pipeline.pending_motion = pending_motion
    pipeline.phase_started = time.monotonic()
    pipeline.motion_timeout = 120.0
    pipeline.motion_status = {
        'state': 'FAULT',
        'target': 'transfer',
        'operation_id': 5,
        'fault': 'MoveIt could not find a plan',
    }
    pipeline.expected_motion_operation_id = 5
    pipeline.expected_supervisor_operation_id = None
    pipeline.supervisor_status = {'operation_id': 9}
    pipeline.transfer_fallback_used = False
    pipeline.transfer_fallback_reason = ''
    pipeline.start_transfer_fallback = FakeClient(
        SimpleNamespace(success=True, message='started'))
    pipeline.accept_direct_transfer = FakeClient(
        SimpleNamespace(success=True, message='accepted'))
    pipeline.get_logger = lambda: FakeLogger()
    pipeline.publish_status = lambda: None
    pipeline.fault = ''
    pipeline._fault = lambda reason: setattr(pipeline, 'fault', reason)
    return pipeline


def test_moveit_transfer_planning_failure_starts_direct_fallback():
    pipeline = _failed_transfer_pipeline('transfer')

    pipeline.tick()

    assert len(pipeline.start_transfer_fallback.requests) == 1
    assert pipeline.pending_motion == 'transfer_fallback_starting'
    assert pipeline.expected_supervisor_operation_id == 10
    assert pipeline.fault == ''


def test_moveit_transfer_execution_failure_does_not_start_direct_fallback():
    pipeline = _failed_transfer_pipeline('transfer_executing')

    pipeline.tick()

    assert pipeline.start_transfer_fallback.requests == []
    assert pipeline.fault == 'MoveIt could not find a plan'


def test_latched_planning_fault_is_ignored_during_fallback_loading():
    pipeline = _failed_transfer_pipeline(None)
    pipeline.state = PlacePipeline.LOAD_PRE_PLACE
    pipeline.transfer_fallback_used = True
    loading_ticks = []
    pipeline._tick_loading = lambda: loading_ticks.append(True)

    pipeline.tick()

    assert loading_ticks == [True]
    assert pipeline.fault == ''


def test_successful_direct_transfer_is_acknowledged_before_loading():
    pipeline = _failed_transfer_pipeline('transfer_fallback_executing')
    pipeline.supervisor_status.update({
        'state': 'SUCCEEDED',
        'operation_kind': 'transfer',
        'operation_id': 10,
        'direct_transfer_succeeded': True,
    })
    pipeline.expected_supervisor_operation_id = 10
    loading = []
    pipeline._begin_loading_motion = lambda: loading.append(True)

    pipeline._tick_transfer()

    assert pipeline.pending_motion == 'transfer_fallback_accepting'
    assert len(pipeline.accept_direct_transfer.requests) == 1
    assert loading == []

    future = pipeline.accept_direct_transfer.futures[-1]
    future.callback(future)
    assert pipeline.pending_motion is None
    assert loading == [True]


def _failed_transfer_coordinator():
    coordinator = object.__new__(MotionCoordinator)
    coordinator.state = MotionCoordinator.FAULT
    coordinator.target = 'transfer'
    coordinator.operation_id = 7
    coordinator.pre_place_tcp_pose = Pose()
    coordinator.transfer_tcp_pose = Pose()
    coordinator.cancel_requested = True
    coordinator.pause_requested = True
    coordinator.fault = 'planning transfer failed'

    def set_state(state, fault=''):
        coordinator.state = state
        coordinator.fault = fault

    coordinator._set_state = set_state
    return coordinator


def test_motion_coordinator_accepts_verified_direct_transfer_without_new_operation():
    coordinator = _failed_transfer_coordinator()
    response = coordinator.accept_direct_transfer_callback(
        Trigger.Request(), Trigger.Response())

    assert response.success is True
    assert coordinator.state == MotionCoordinator.SUCCEEDED
    assert coordinator.fault == ''
    assert coordinator.operation_id == 7
    assert coordinator.target == 'transfer'
    assert coordinator.pre_place_tcp_pose is not None
    assert coordinator.transfer_tcp_pose is not None


def test_motion_coordinator_rejects_direct_transfer_ack_outside_failed_transfer():
    coordinator = _failed_transfer_coordinator()
    coordinator.state = MotionCoordinator.IDLE

    response = coordinator.accept_direct_transfer_callback(
        Trigger.Request(), Trigger.Response())

    assert response.success is False
    assert 'requires a failed transfer' in response.message
    assert coordinator.state == MotionCoordinator.IDLE
