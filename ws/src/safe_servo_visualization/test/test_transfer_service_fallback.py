import math
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
import pytest

from safe_servo_visualization.motion_coordinator_node import MotionCoordinator
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from safe_servo_visualization.place_pipeline_node import PlacePipeline
from std_srvs.srv import Trigger


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


class FakeGoalHandle:
    def __init__(self, result=None, accepted=True):
        self.accepted = accepted
        self.result_future = FakeFuture(result)
        self.cancelled = False

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancelled = True
        return FakeFuture()


class FakeActionClient:
    def __init__(self, goal_handle=None):
        self.ready = True
        self.goal_handle = goal_handle or FakeGoalHandle()
        self.goals = []
        self.futures = []

    def server_is_ready(self):
        return self.ready

    def wait_for_server(self, timeout_sec):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        future = FakeFuture(self.goal_handle)
        self.futures.append(future)
        return future


def test_moveit_service_discovery_wait_is_bounded():
    class DiscoveringClient:
        def __init__(self):
            self.timeout = None

        def wait_for_service(self, timeout_sec):
            self.timeout = timeout_sec
            return True

    client = DiscoveringClient()

    assert PickupSupervisor._wait_for_service(client, 2.0) is True
    assert client.timeout == pytest.approx(2.0)


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


def test_motion_slider_scales_direct_cartesian_speed_and_acceleration():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.direct_cartesian_max_speed = 200.0
    supervisor.direct_cartesian_max_acc = 500.0
    supervisor.direct_transfer_max_joint_speed = 2.14
    supervisor.get_logger = lambda: FakeLogger()

    supervisor.motion_speed_config_callback(
        SimpleNamespace(data=[0.5, 50.0]))

    assert supervisor.motion_speed_percent == pytest.approx(50.0)
    assert supervisor.retreat_speed == pytest.approx(100.0)
    assert supervisor.retreat_acc == pytest.approx(250.0)


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


def test_periodic_ik_joint_is_normalized_to_nearest_hardware_equivalent():
    normalized = PickupSupervisor._nearest_periodic_equivalent(
        3.8, -0.3, -2.0 * math.pi, 2.0 * math.pi)

    assert normalized == pytest.approx(3.8 - 2.0 * math.pi)
    assert abs(normalized + 0.3) < abs(3.8 + 0.3)


def test_moveit_kdl_ik_solution_starts_sampled_collision_validation():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.state = PickupSupervisor.SOLVING_TRANSFER_IK
    supervisor.latest_joint_positions = (0.0,) * 6
    supervisor.arm_joint_names = tuple(f'joint{i}' for i in range(1, 7))
    supervisor.planning_group = 'uf850'
    supervisor.direct_transfer_periodic_joint_names = frozenset(
        ('joint1', 'joint4', 'joint6'))
    supervisor.direct_transfer_periodic_limits = (
        -2.0 * math.pi, 2.0 * math.pi)
    supervisor.direct_transfer_max_joint_delta = 2.6
    supervisor.direct_transfer_sample_step = 0.05
    supervisor.direct_target_joints = None
    supervisor.direct_transfer_validation_samples = []
    supervisor.direct_transfer_validation_index = 0
    supervisor.state_validity_client = FakeClient()
    supervisor.get_logger = lambda: FakeLogger()
    supervisor._fault = pytest.fail

    supervisor._direct_transfer_ik_completed(FakeFuture(SimpleNamespace(
        error_code=SimpleNamespace(val=1, message=''),
        solution=SimpleNamespace(joint_state=SimpleNamespace(
            name=list(supervisor.arm_joint_names),
            position=[0.10, 0.0, 0.0, 0.0, 0.0, 0.0])))))

    assert supervisor.state == PickupSupervisor.VALIDATING_TRANSFER
    assert supervisor.direct_target_joints == pytest.approx(
        (0.10, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert len(supervisor.direct_transfer_validation_samples) == 2
    request = supervisor.state_validity_client.requests[-1]
    assert request.group_name == 'uf850'
    assert request.robot_state.joint_state.position == pytest.approx(
        [0.05, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_validated_joint_line_executes_through_trajectory_controller():
    successful_result = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(
            error_code=FollowJointTrajectory.Result.SUCCESSFUL,
            error_string=''))
    goal_handle = FakeGoalHandle(successful_result)
    supervisor = object.__new__(PickupSupervisor)
    supervisor.state = PickupSupervisor.VALIDATING_TRANSFER
    supervisor.latest_joint_positions = (0.0,) * 6
    supervisor.arm_joint_names = tuple(f'joint{i}' for i in range(1, 7))
    supervisor.direct_target_joints = (0.10, 0.0, 0.0, 0.0, 0.0, 0.0)
    supervisor.direct_transfer_validation_samples = [
        (0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
        supervisor.direct_target_joints,
    ]
    supervisor.direct_transfer_ik_timeout = 2.0
    supervisor.direct_transfer_max_joint_speed = 2.14
    supervisor.direct_transfer_joint_acc = 0.7
    supervisor.retreat_speed = 75.0
    supervisor.trajectory_controller = 'uf850_traj_controller'
    supervisor.transfer_trajectory_client = FakeActionClient(goal_handle)
    supervisor.direct_transfer_motion_started = False
    supervisor.transfer_goal_handle = None
    supervisor.get_logger = lambda: FakeLogger()
    supervisor.publish_status = lambda: None
    supervisor._fault = pytest.fail
    completed = []
    supervisor._finish_retreat = lambda: completed.append(True)

    supervisor._send_joint_trajectory_transfer()

    assert supervisor.state == PickupSupervisor.EXECUTING_TRANSFER
    assert len(supervisor.transfer_trajectory_client.goals) == 1
    goal = supervisor.transfer_trajectory_client.goals[0]
    assert goal.trajectory.joint_names == list(supervisor.arm_joint_names)
    assert len(goal.trajectory.points) == 2
    assert goal.trajectory.points[-1].positions == pytest.approx(
        supervisor.direct_target_joints)
    assert goal.trajectory.points[-1].velocities == pytest.approx([0.0] * 6)
    assert goal.trajectory.points[-1].accelerations == pytest.approx([0.0] * 6)

    send_future = supervisor.transfer_trajectory_client.futures[-1]
    send_future.callback(send_future)
    assert supervisor.direct_transfer_motion_started is True
    goal_handle.result_future.callback(goal_handle.result_future)
    assert completed == [True]


def test_joint_transfer_uses_full_model_limits_at_one_hundred_percent():
    duration, speed, acceleration, fraction = (
        PickupSupervisor._joint_transfer_timing(
            max_delta=3.0,
            operator_percent=100.0,
            maximum_speed=2.14,
            maximum_acceleration=10.0,
        ))

    assert fraction == pytest.approx(1.0)
    assert speed == pytest.approx(2.14)
    assert acceleration == pytest.approx(10.0)
    assert duration == pytest.approx(1.875 * 3.0 / 2.14)


def test_joint_transfer_speed_slider_scales_velocity_and_acceleration():
    duration, speed, acceleration, fraction = (
        PickupSupervisor._joint_transfer_timing(
            max_delta=3.0,
            operator_percent=50.0,
            maximum_speed=2.14,
            maximum_acceleration=10.0,
        ))

    assert fraction == pytest.approx(0.5)
    assert speed == pytest.approx(1.07)
    assert acceleration == pytest.approx(5.0)
    assert duration == pytest.approx(1.875 * 3.0 / 1.07)


def _completed_direct_joint_transfer(actual_joints):
    supervisor = object.__new__(PickupSupervisor)
    supervisor.operation_kind = 'transfer'
    supervisor.direct_target_pose = (
        410.0, -275.0, 620.0, math.pi, 0.0, 0.0)
    supervisor.direct_target_joints = (0.0,) * 6
    supervisor.latest_joint_positions = tuple(actual_joints)
    supervisor.arm_joint_names = tuple(f'joint{i}' for i in range(1, 7))
    supervisor.direct_transfer_periodic_joint_names = frozenset(
        ('joint1', 'joint4', 'joint6'))
    supervisor.direct_transfer_joint_tolerance = 0.03
    supervisor.direct_transfer_succeeded = False
    supervisor.dry_run = False
    supervisor.post_retreat_fault = ''
    supervisor.joint6_name = 'joint6'
    supervisor._joint6_is_moveit_safe = lambda: True
    supervisor._direct_mode_tcp_xyz = lambda: pytest.fail(
        'joint transfer must not compare incompatible SDK and MoveIt TCP frames')
    supervisor.publish_status = lambda: None
    supervisor.fault = ''
    supervisor._fault = lambda reason: setattr(supervisor, 'fault', reason)
    return supervisor


def test_direct_joint_transfer_uses_fresh_joint_feedback_not_sdk_tcp():
    supervisor = _completed_direct_joint_transfer(
        (2.0 * math.pi, 0.01, 0.0, 0.0, 0.0, 0.0))

    supervisor._finish_retreat()

    assert supervisor.state == PickupSupervisor.SUCCEEDED
    assert supervisor.direct_transfer_succeeded is True
    assert supervisor.fault == ''


def test_direct_joint_transfer_rejects_large_joint_feedback_error():
    supervisor = _completed_direct_joint_transfer(
        (0.0, 0.05, 0.0, 0.0, 0.0, 0.0))

    supervisor._finish_retreat()

    assert supervisor.direct_transfer_succeeded is False
    assert 'joint2 feedback error is 0.050 rad' in supervisor.fault


def _direct_loading_supervisor():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.robot_error = 0
    supervisor.operation_kind = 'loading'
    supervisor.loading_contact_fallback = False
    supervisor.direct_target_pose = None
    supervisor.direct_target_z = 0.300
    supervisor.direct_tcp_z_offset = 0.0
    supervisor.direct_command_target_z = 0.300
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


def test_loading_converts_link_tcp_target_to_sdk_tcp_before_direct_motion():
    supervisor = _direct_loading_supervisor()
    supervisor.direct_target_z = 0.235
    supervisor.retreat_start_z = 0.480
    supervisor.direct_tcp_z_offset = 0.024

    supervisor._send_direct_retreat()

    request = supervisor.retreat_client.requests[-1]
    assert request.relative is True
    assert request.pose[:3] == pytest.approx([0.0, 0.0, -269.0])
    assert supervisor.direct_command_target_z == pytest.approx(0.211)
    assert supervisor.retreat_target_z == pytest.approx(0.211)


def test_loading_does_not_stop_at_unconverted_link_tcp_z():
    supervisor = _direct_loading_supervisor()
    supervisor.direct_target_z = 0.235
    supervisor.direct_tcp_z_offset = 0.024
    supervisor.direct_command_target_z = 0.211
    supervisor.state = PickupSupervisor.RETREATING
    supervisor.retreat_started = time.monotonic()
    supervisor.pre_descent_wait_callback = None
    supervisor.mode_wait_target = None
    # This is the old, incorrect stopping point: SDK Z equals the requested
    # link_tcp Z, leaving link_tcp approximately 24 mm too high.
    supervisor._direct_mode_tcp_xyz = lambda: (0.4, -0.2, 0.235)

    supervisor.retreat_tick()

    assert supervisor.state == PickupSupervisor.RETREATING
    assert supervisor.set_state_client.requests == []


def test_nonblocking_loading_reaching_target_restores_ros_control():
    supervisor = _direct_loading_supervisor()
    supervisor.state = PickupSupervisor.RETREATING
    supervisor.retreat_started = time.monotonic()
    supervisor.pre_descent_wait_callback = None
    supervisor.mode_wait_target = None
    supervisor.direct_command_target_z = 0.300
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


def test_moveit_transfer_planning_failure_faults_without_direct_fallback():
    pipeline = _failed_transfer_pipeline('transfer')

    pipeline.tick()

    assert pipeline.start_transfer_fallback.requests == []
    assert pipeline.fault == 'MoveIt could not find a plan'


def test_moveit_transfer_execution_failure_does_not_start_direct_fallback():
    pipeline = _failed_transfer_pipeline('transfer_executing')

    pipeline.tick()

    assert pipeline.start_transfer_fallback.requests == []
    assert pipeline.fault == 'MoveIt could not find a plan'


def _direct_joint_failure_pipeline(motion_started):
    pipeline = object.__new__(PlacePipeline)
    pipeline.pending_motion = 'direct_joint_executing'
    pipeline.expected_supervisor_operation_id = 8
    pipeline.supervisor_status = {
        'state': 'FAULT',
        'operation_kind': 'transfer',
        'operation_id': 8,
        'fault': 'MoveIt/KDL transfer IK failed',
        'direct_transfer_motion_started': motion_started,
    }
    pipeline.motion_status = {
        'state': 'PREPARED', 'target': 'transfer', 'operation_id': 4}
    pipeline.plan_transfer = FakeClient(
        SimpleNamespace(success=True, message='planning'))
    pipeline.transfer_fallback_used = False
    pipeline.transfer_fallback_reason = ''
    pipeline.moveit_transfer_fallback_enabled = False
    pipeline.phase_started = time.monotonic()
    pipeline.get_logger = lambda: FakeLogger()
    pipeline.fault = ''
    pipeline._fault = lambda reason: setattr(pipeline, 'fault', reason)
    return pipeline


def test_moveit_kdl_ik_failure_faults_when_planning_fallback_is_disabled():
    pipeline = _direct_joint_failure_pipeline(False)

    pipeline._tick_transfer()

    assert pipeline.transfer_fallback_used is False
    assert pipeline.plan_transfer.requests == []
    assert 'automatic MoveIt transfer fallback is disabled' in pipeline.fault


def test_moveit_kdl_ik_failure_can_use_explicitly_enabled_rrt_fallback():
    pipeline = _direct_joint_failure_pipeline(False)
    pipeline.moveit_transfer_fallback_enabled = True

    pipeline._tick_transfer()

    assert pipeline.pending_motion == 'transfer'
    assert pipeline.transfer_fallback_used is True
    assert len(pipeline.plan_transfer.requests) == 1
    assert pipeline.fault == ''


def test_perpendicular_transfer_uses_direct_joint_interpolation():
    pipeline = object.__new__(PlacePipeline)
    pipeline.pending_motion = 'transfer_preparing'
    pipeline.expected_motion_operation_id = 4
    pipeline.motion_status = {
        'state': 'PREPARED',
        'target': 'transfer',
        'operation_id': 4,
        'keep_eef_perpendicular_to_pallet': True,
    }
    pipeline.supervisor_status = {'operation_id': 9}
    pipeline.plan_transfer = FakeClient(
        SimpleNamespace(success=True, message='planning'))
    pipeline.start_joint_transfer = FakeClient(
        SimpleNamespace(success=True, message='direct'))
    pipeline.phase_started = time.monotonic()
    pipeline.get_logger = lambda: FakeLogger()
    pipeline._fault = lambda reason: setattr(pipeline, 'fault', reason)

    pipeline._tick_transfer()

    assert pipeline.pending_motion == 'direct_joint_starting'
    assert len(pipeline.start_joint_transfer.requests) == 1
    assert pipeline.plan_transfer.requests == []


def test_direct_joint_execution_failure_does_not_auto_fallback():
    pipeline = _direct_joint_failure_pipeline(True)

    pipeline._tick_transfer()

    assert pipeline.plan_transfer.requests == []
    assert 'direct motion had already started' in pipeline.fault


def test_invalid_direct_interpolation_sample_faults_before_robot_motion():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.state = PickupSupervisor.VALIDATING_TRANSFER
    supervisor.direct_transfer_validation_index = 1
    supervisor.direct_transfer_validation_samples = [(0.0,) * 6] * 5
    faults = []
    supervisor._fault = faults.append

    supervisor._direct_transfer_sample_validated(
        FakeFuture(SimpleNamespace(valid=False)))

    assert faults == [
        'direct transfer interpolation is in collision at sample 2/5']


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
