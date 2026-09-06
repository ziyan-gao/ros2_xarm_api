import pytest
from geometry_msgs.msg import TransformStamped

from safe_servo_visualization.planning_scene_obstacles_node import (
    PlanningSceneObstacles,
)


def test_positive_y_wall_preserves_1_2_m_robot_side_clearance():
    node = object.__new__(PlanningSceneObstacles)
    node.base_frame = 'link_base'

    objects = {obj.id: obj for obj in node._table_objects()}
    wall = objects['positive_y_wall']
    dimensions = wall.primitives[0].dimensions
    center = wall.primitive_poses[0].position

    assert dimensions == pytest.approx([6.0, 0.05, 3.0])
    assert center.x == pytest.approx(0.0)
    assert center.y - dimensions[1] / 2.0 == pytest.approx(1.2)
    assert center.z == pytest.approx(0.0)


class IdentityPalletTf:
    def lookup_transform(self, target_frame, source_frame, *_args, **_kwargs):
        assert target_frame == 'link_base'
        assert source_frame == 'pallet_frame'
        transform = TransformStamped()
        transform.transform.translation.x = 0.30
        transform.transform.translation.y = -0.70
        transform.transform.translation.z = -0.15
        transform.transform.rotation.w = 1.0
        return transform


def test_rotated_fallback_obstacle_uses_predicted_pallet_corner_and_z():
    node = object.__new__(PlanningSceneObstacles)
    node.base_frame = 'link_base'
    node.placed_item_counter = 2
    node.tf_buffer = IdentityPalletTf()
    target = {
        'sequence_id': 8,
        'item_id': 9,
        'corner_m': (0.10, 0.20, 0.05),
        'rotated': True,
        'size_m': (0.22, 0.17, 0.12),
    }

    obstacle = node._placed_item_from_random_target(target)
    dimensions = obstacle.primitives[0].dimensions
    pose = obstacle.primitive_poses[0]

    assert obstacle.id == 'placed_item_3'
    assert dimensions == pytest.approx([0.22, 0.17, 0.12])
    # Clockwise rotation swaps the footprint dimensions. The obstacle center
    # is reconstructed from the predicted min-X/min-Y/bottom corner.
    assert pose.position.x == pytest.approx(0.30 + 0.10 + 0.17 / 2.0)
    assert pose.position.y == pytest.approx(-0.70 + 0.20 + 0.22 / 2.0)
    assert pose.position.z == pytest.approx(-0.15 + 0.05 + 0.12 / 2.0)
    assert pose.orientation.z == pytest.approx(-2.0 ** -0.5)
    assert pose.orientation.w == pytest.approx(2.0 ** -0.5)


def test_predicted_pose_requires_matching_active_random_target():
    node = object.__new__(PlanningSceneObstacles)
    target = {'sequence_id': 8, 'item_id': 9}
    node.place_singularity_fallback = True
    node.random_loading_target = target
    node.random_loading_status = {
        'state': 'EXECUTING',
        'pending_sequence_id': 8,
    }
    node.pregrasp_snapshot = {'box_id': 9}

    assert node._active_random_fallback_target() is target

    node.random_loading_status['pending_sequence_id'] = 7
    assert node._active_random_fallback_target() is None
