from types import SimpleNamespace

import pytest
from rclpy.clock import Clock
from moveit_msgs.msg import CollisionObject

from safe_servo_visualization.pallet_localization_node import PalletLocalization
from safe_servo_visualization.planning_scene_obstacles_node import PlanningSceneObstacles


@pytest.mark.parametrize('locked', [True, False])
def test_pallet_marker_is_brown_without_changing_geometry(locked):
    node = object.__new__(PalletLocalization)
    node.preview = ([0., 0., 0.], [0., 0., 0., 1.])
    node.locked = locked
    node.pallet_x, node.pallet_y = .45, .55
    node.get_clock = lambda: Clock()
    node.publish_pre_place_pose = lambda: None
    markers = []
    node.marker_pub = SimpleNamespace(publish=markers.append)
    node.publish_preview('pallet_frame')
    deck = markers[0]
    assert (deck.color.r, deck.color.g, deck.color.b) == pytest.approx((.55, .30, .12))
    assert deck.color.a == pytest.approx(.90 if locked else .32)
    assert (deck.scale.x, deck.scale.y, deck.scale.z) == pytest.approx((.45, .55, .005))


@pytest.mark.parametrize('operation', [CollisionObject.ADD, CollisionObject.REMOVE])
def test_scene_colors_only_added_pallet_and_preserves_objects(operation):
    node = object.__new__(PlanningSceneObstacles)
    pallet = CollisionObject(id='pallet_surface', operation=operation)
    table = CollisionObject(id='table', operation=CollisionObject.ADD)
    scenes = []
    node._apply_scene = lambda scene, *_args: scenes.append(scene)
    node._apply([pallet, table], 'test')
    scene = scenes[0]
    assert scene.world.collision_objects == [pallet, table]
    if operation == CollisionObject.REMOVE:
        assert not scene.object_colors
    else:
        assert len(scene.object_colors) == 1
        color = scene.object_colors[0]
        assert color.id == 'pallet_surface'
        assert (color.color.r, color.color.g, color.color.b, color.color.a) == pytest.approx(
            (.55, .30, .12, .90))
