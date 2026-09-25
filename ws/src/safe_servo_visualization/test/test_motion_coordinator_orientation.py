import math
import time
from types import SimpleNamespace

from geometry_msgs.msg import Pose
import pytest

from safe_servo_visualization.motion_coordinator_node import MotionCoordinator
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from std_msgs.msg import Float64MultiArray


def _coordinator(*, rotate_item_90=False):
    coordinator = object.__new__(MotionCoordinator)
    coordinator.rotate_item_90 = rotate_item_90
    return coordinator


def test_perpendicular_orientation_rejects_localized_pallet_tilt():
    coordinator = _coordinator()
    object_q = coordinator._quaternion_from_rpy(
        math.radians(3.2), math.radians(-2.7), math.radians(41.0))
    unconstrained_q = coordinator._quaternion_from_rpy(
        math.radians(176.0), math.radians(2.0), math.radians(-18.0))

    result = coordinator._perpendicular_tcp_orientation(
        object_q, unconstrained_q)
    tool_z = coordinator._quat_rotate((0.0, 0.0, 1.0), result)

    assert tool_z[0] == pytest.approx(0.0, abs=1e-12)
    assert tool_z[1] == pytest.approx(0.0, abs=1e-12)
    assert tool_z[2] == pytest.approx(-1.0, abs=1e-12)


def test_rotated_item_orientation_remains_vertical():
    coordinator = _coordinator(rotate_item_90=True)
    object_q = coordinator._quaternion_from_rpy(
        math.radians(-2.0), math.radians(2.9), math.radians(-89.0))
    unconstrained_q = coordinator._quaternion_from_rpy(
        math.pi, 0.0, math.radians(27.0))

    result = coordinator._perpendicular_tcp_orientation(
        object_q, unconstrained_q)
    tool_z = coordinator._quat_rotate((0.0, 0.0, 1.0), result)

    assert tool_z == pytest.approx((0.0, 0.0, -1.0), abs=1e-12)


def _pre_place_pose(*, pallet_origin_z, target_xyz, clearance):
    pose = Pose()
    pose.position.x = target_xyz[0]
    pose.position.y = target_xyz[1]
    pose.position.z = pallet_origin_z + target_xyz[2] + clearance
    pose.orientation.w = 1.0
    return pose


def test_transfer_corner_uses_fixed_pallet_frame_height():
    coordinator = _coordinator()
    coordinator.pre_place_clearance = 0.04
    coordinator.transfer_corner_height = 0.47
    coordinator.place_target_xyz = (0.10, 0.20, 0.05)
    coordinator.keep_eef_perpendicular = False
    coordinator.pre_place_pose = _pre_place_pose(
        pallet_origin_z=-0.20,
        target_xyz=coordinator.place_target_xyz,
        clearance=coordinator.pre_place_clearance)
    coordinator.attached_item_geometry = {
        'size': (0.20, 0.10, 0.08),
        # A centered top grasp: TCP local +Z points from the suction cup down
        # to the item center, and TCP is rotated 180 deg about base X.
        'center': (0.0, 0.0, 0.04),
        'orientation': (1.0, 0.0, 0.0, 0.0),
    }

    coordinator._calculate_place_poses()

    assert coordinator.nominal_transfer_corner_z == pytest.approx(0.27)
    assert coordinator.transfer_tcp_pose.position.z == pytest.approx(0.35)


def test_pickup_retreat_uses_same_fixed_corner_height():
    supervisor = object.__new__(PickupSupervisor)
    supervisor.pre_place_clearance = 0.04
    supervisor.transfer_corner_height = 0.47
    supervisor.grasp_offset = 0.0
    supervisor.place_target_xyz = (0.10, 0.20, 0.05)
    supervisor.rotate_item_90 = False
    supervisor.pre_place_pose = _pre_place_pose(
        pallet_origin_z=-0.20,
        target_xyz=supervisor.place_target_xyz,
        clearance=supervisor.pre_place_clearance)

    assert supervisor._transfer_corner_base_z(0.10) == pytest.approx(0.27)
    assert supervisor._pickup_retreat_target_tcp_z(
        0.08, 0.10) == pytest.approx(0.35)


def test_rotated_tilted_pallet_uses_same_corrected_corner_for_both_waypoints():
    coordinator = _coordinator(rotate_item_90=True)
    coordinator.pre_place_clearance = 0.04
    coordinator.transfer_corner_height = 0.47
    coordinator.place_target_xyz = (0.12, 0.18, 0.03)
    coordinator.keep_eef_perpendicular = True
    pallet_q = coordinator._quaternion_from_rpy(
        math.radians(2.0), math.radians(-2.5), math.radians(35.0))
    object_q = coordinator._quat_multiply(
        pallet_q, coordinator._quaternion_from_rpy(0.0, 0.0, -math.pi / 2.0))
    local_pre_place = (
        coordinator.place_target_xyz[0],
        coordinator.place_target_xyz[1],
        coordinator.place_target_xyz[2] + coordinator.pre_place_clearance)
    offset = coordinator._quat_rotate(local_pre_place, pallet_q)
    pose = Pose()
    pose.position.x = 0.40 + offset[0]
    pose.position.y = -0.30 + offset[1]
    pose.position.z = -0.20 + offset[2]
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = object_q
    coordinator.pre_place_pose = pose
    coordinator.attached_item_geometry = {
        'size': (0.20, 0.10, 0.08),
        'center': (0.0, 0.0, 0.04),
        'orientation': (1.0, 0.0, 0.0, 0.0),
    }
    coordinator._calculate_place_poses()

    supervisor = object.__new__(PickupSupervisor)
    supervisor.pre_place_clearance = coordinator.pre_place_clearance
    supervisor.transfer_corner_height = coordinator.transfer_corner_height
    supervisor.place_target_xyz = coordinator.place_target_xyz
    supervisor.rotate_item_90 = True
    supervisor.pre_place_pose = pose

    assert supervisor._transfer_corner_base_z(0.20) == pytest.approx(
        coordinator.nominal_transfer_corner_z)


def test_clockwise_rotation_preserves_requested_minimum_xy_corner():
    coordinator = _coordinator(rotate_item_90=True)
    coordinator.pre_place_clearance = 0.03
    coordinator.transfer_corner_height = 0.47
    coordinator.place_target_xyz = (0.12, 0.18, 0.0)
    coordinator.keep_eef_perpendicular = False
    pose = Pose()
    pose.position.x = 0.12
    pose.position.y = 0.18
    pose.position.z = 0.03
    clockwise_q = coordinator._quaternion_from_rpy(
        0.0, 0.0, -math.pi / 2.0)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = clockwise_q
    coordinator.pre_place_pose = pose
    coordinator.attached_item_geometry = {
        'size': (0.20, 0.10, 0.08),
        'center': (0.0, 0.0, 0.04),
        'orientation': (1.0, 0.0, 0.0, 0.0),
    }

    coordinator._calculate_place_poses()

    # After -90 deg, the original +X edge extends toward pallet -Y. Moving
    # its original corner +0.20 m in pallet Y keeps (0.12, 0.18) as min XY.
    assert coordinator.placement_corner_correction == pytest.approx(
        (0.0, 0.20, 0.0), abs=1e-12)


def test_place_servo_uses_its_lower_z_bound_without_changing_pickup_bound():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    supervisor = object.__new__(PickupSupervisor)
    supervisor.force_threshold = 5.0
    supervisor.place_force_threshold = 4.0
    supervisor.servo_speed_scale = 0.4
    supervisor.place_servo_speed_scale = 1.0
    supervisor.servo_bounds_mm = (160.0, 390.0, -360.0, 360.0, 50.0, 800.0)
    supervisor.place_workspace_z_min_mm = -100.0
    supervisor.config_pub = Publisher()

    supervisor.operation_kind = 'place'
    supervisor._publish_servo_config(touch_mode=True)
    assert supervisor.config_pub.messages[-1].data[5] == -100.0

    supervisor.operation_kind = 'pickup'
    supervisor._publish_servo_config(touch_mode=True)
    assert supervisor.config_pub.messages[-1].data[5] == 50.0

    supervisor.operation_kind = 'place'
    supervisor.staging_place_active = True
    supervisor.staging_bounds_mm = (
        -450.0, 450.0, 100.0, 750.0, -100.0, 800.0)
    supervisor._publish_servo_config(touch_mode=True)
    assert supervisor.config_pub.messages[-1].data[1:7] == pytest.approx(
        [-450.0, 450.0, 100.0, 750.0, -100.0, 800.0])


def test_retrieval_target_accepts_rearrangement_id_and_object_yaw():
    coordinator = object.__new__(MotionCoordinator)
    coordinator.staging_retrieve_target = None
    message = Float64MultiArray()
    message.data = [
        27.0, 0.4, -0.2, 0.3, math.pi, 0.0, 0.5,
        0.22, 0.17, 0.12, 0.03, 0.5, 0.72,
    ]

    coordinator.staging_retrieve_target_callback(message)

    assert coordinator.staging_retrieve_target['target_id'] == 27
    assert coordinator.staging_retrieve_target['object_yaw'] == pytest.approx(0.5)
    assert coordinator.staging_retrieve_target['approach_tcp_z'] == pytest.approx(
        0.72)
    assert coordinator.staging_retrieve_target['size'] == pytest.approx(
        (0.22, 0.17, 0.12))


def test_buffer_retrieval_preserves_recorded_grasp_transform():
    coordinator = object.__new__(MotionCoordinator)
    grasp = [.01, -.02, .06, 1., 0., 0., 0.]
    message = Float64MultiArray(data=[
        2., .4, .2, .3, math.pi, 0., .5,
        .22, .17, .12, .03, .5, .72, 1., *grasp])
    coordinator.staging_retrieve_target_callback(message)
    assert coordinator.staging_retrieve_target['recorded_grasp'] == grasp
    assert coordinator.staging_retrieve_target['object_yaw'] == .5


def test_staging_store_transfer_uses_normal_transfer_status_contract():
    coordinator = object.__new__(MotionCoordinator)
    coordinator.staging_store_transfer_target = None
    coordinator.state = MotionCoordinator.IDLE
    coordinator.operation_id = 7
    coordinator.target = None
    coordinator.transfer_context = ''
    coordinator.transfer_tcp_pose = None
    coordinator.pre_place_tcp_pose = None
    coordinator.nominal_transfer_corner_z = 0.47
    coordinator.fault = ''
    coordinator.cancel_requested = False
    coordinator.pause_requested = False
    coordinator._require_fresh_joint_state = lambda *_args: True
    coordinator._set_state = lambda state, fault='': setattr(
        coordinator, 'state', state)
    message = Float64MultiArray()
    message.data = [
        4.0, -0.125, 0.430, 0.480,
        1.0, 0.0, 0.0, 0.0,
        0.110,
    ]

    coordinator.staging_store_transfer_target_callback(message)
    response = SimpleNamespace(success=False, message='')
    coordinator.prepare_staging_store_transfer_callback(None, response)

    assert response.success
    assert coordinator.state == MotionCoordinator.PREPARED
    assert coordinator.target == 'transfer'
    assert coordinator.transfer_context == 'staging_store'
    assert coordinator.transfer_tcp_pose.position.x == pytest.approx(-0.125)
    assert coordinator.transfer_tcp_pose.position.z == pytest.approx(0.480)
    assert coordinator.pre_place_tcp_pose.position.z == pytest.approx(0.110)


def test_pallet_retrieval_pregrasp_uses_straight_cartesian_plan():
    coordinator = object.__new__(MotionCoordinator)
    coordinator.state = coordinator.SUCCEEDED
    coordinator.staging_retrieve_target = {
        'target_id': 27,
        'contact_tcp_pose': (0.4, -0.2, 0.3, math.pi, 0.0, 0.5),
        'size': (0.22, 0.17, 0.12),
        'clearance': 0.03,
        'object_yaw': 0.5,
        'approach_tcp_z': 0.72,
        'received_at': time.monotonic(),
    }
    coordinator._require_fresh_joint_state = lambda *_args: True
    coordinator.straight_plan_client = SimpleNamespace(
        service_is_ready=lambda: True)
    coordinator.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))
    captured = {}

    def start_straight(pose, box_id, response):
        captured.update(pose=pose, box_id=box_id)
        response.success = True
        return response

    coordinator._start_straight_plan = start_straight
    response = SimpleNamespace(success=False, message='')

    result = coordinator.plan_staging_pregrasp_callback(None, response)

    assert result.success
    assert captured['box_id'] == 1027
    assert captured['pose'].position.x == pytest.approx(0.4)
    assert captured['pose'].position.y == pytest.approx(-0.2)
    assert captured['pose'].position.z == pytest.approx(0.33)


def test_straight_pregrasp_preserves_supervisor_target_contract():
    class PendingFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    class StraightClient:
        def call_async(self, request):
            self.request = request
            self.future = PendingFuture()
            return self.future

    coordinator = object.__new__(MotionCoordinator)
    coordinator.operation_id = 3
    coordinator.cancel_requested = False
    coordinator.pause_requested = False
    coordinator.straight_plan_client = StraightClient()
    coordinator._publish_pregrasp_marker = lambda *_args: None
    coordinator._set_state = lambda state: setattr(coordinator, 'state', state)
    pose = Pose()
    pose.position.x = 0.4
    pose.position.y = -0.2
    pose.position.z = 0.33
    response = SimpleNamespace(success=False, message='')

    result = coordinator._start_straight_plan(pose, 1027, response)

    assert result.success
    assert coordinator.target == 'pregrasp_box_1027'
    assert coordinator.state == coordinator.PLANNING
    assert coordinator.straight_plan_client.request.target is pose


def test_pregrasp_ik_failure_retries_yaw_180_and_updates_snapshot():
    class FinishedFuture:
        def __init__(self, success):
            self._result = SimpleNamespace(success=success)

        def result(self):
            return self._result

    coordinator = object.__new__(MotionCoordinator)
    coordinator.operation_id = 12
    coordinator.cancel_requested = False
    coordinator.pause_requested = False
    coordinator.target = 'pregrasp_box_4'
    coordinator.state = coordinator.PLANNING
    coordinator.planned_pregrasp = {'yaw_rad': 0.2}
    alternate = Pose()
    alternate.position.x = 0.55
    alternate.position.y = 0.02
    alternate.position.z = 0.18
    alternate_yaw = 0.2 - math.pi
    (alternate.orientation.x, alternate.orientation.y,
     alternate.orientation.z, alternate.orientation.w) = \
        coordinator._quaternion_from_rpy(math.pi, 0.0, alternate_yaw)
    coordinator.pregrasp_yaw_fallback = {
        'pose': alternate, 'yaw_rad': alternate_yaw, 'box_id': 4}
    published = []
    coordinator._publish_pregrasp_marker = lambda pose, box_id: published.append(
        (pose, box_id))
    coordinator.publish_status = lambda: None
    coordinator.get_logger = lambda: SimpleNamespace(warning=lambda *_args: None)
    coordinator._set_state = lambda state, fault='': (
        setattr(coordinator, 'state', state), setattr(coordinator, 'fault', fault))
    requested = []
    coordinator._request_pregrasp_ik = lambda request_id, pose: requested.append(
        (request_id, pose))

    coordinator._pregrasp_plan_completed(12, FinishedFuture(False))

    assert requested == [(12, alternate)]
    assert coordinator.planned_pregrasp['yaw_rad'] == pytest.approx(alternate_yaw)
    assert published == [(alternate, 4)]
    assert coordinator.pregrasp_yaw_fallback is None


def test_pregrasp_faults_only_after_both_symmetric_yaws_fail():
    coordinator = object.__new__(MotionCoordinator)
    coordinator.operation_id = 3
    coordinator.cancel_requested = False
    coordinator.pause_requested = False
    coordinator.target = 'pregrasp_box_0'
    coordinator.pregrasp_yaw_fallback = None
    coordinator._set_state = lambda state, fault='': (
        setattr(coordinator, 'state', state), setattr(coordinator, 'fault', fault))
    failed = SimpleNamespace(result=lambda: SimpleNamespace(success=False))

    coordinator._pregrasp_plan_completed(3, failed)

    assert coordinator.state == coordinator.FAULT
    assert 'original and yaw-180' in coordinator.fault


def test_pregrasp_ik_solution_is_ordered_and_sent_as_joint_goal():
    class PendingFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    class JointClient:
        def call_async(self, request):
            self.request = request
            self.future = PendingFuture()
            return self.future

    coordinator = object.__new__(MotionCoordinator)
    coordinator.operation_id = 8
    coordinator.plan_client = JointClient()
    coordinator.pregrasp_yaw_fallback = {'unused': True}
    coordinator._set_state = lambda state, fault='': (
        setattr(coordinator, 'state', state), setattr(coordinator, 'fault', fault))
    solution = SimpleNamespace(
        name=['joint3', 'joint1', 'joint6', 'joint2', 'joint5', 'joint4'],
        position=[3.0, 1.0, 6.0, 2.0, 5.0, 4.0])
    result = SimpleNamespace(
        error_code=SimpleNamespace(val=1),
        solution=SimpleNamespace(joint_state=solution))
    completed = SimpleNamespace(result=lambda: result)

    coordinator._pregrasp_ik_completed(8, completed)

    assert coordinator.plan_client.request.target == pytest.approx(
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    assert coordinator.pregrasp_yaw_fallback == {'unused': True}
