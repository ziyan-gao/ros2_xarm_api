import math

import pytest
from geometry_msgs.msg import Pose

from safe_servo_visualization.motion_coordinator_node import MotionCoordinator
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor


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
