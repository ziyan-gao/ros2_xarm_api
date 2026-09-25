from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from safe_servo_visualization.planning_scene_obstacles_node import PlanningSceneObstacles


@pytest.mark.parametrize('q, expected', [
    ((0., 0., 0., 1.), (.1, .1825, .3)),
    ((0., 0., 1., 0.), (.1, .2175, .3)),
])
def test_camera_body_uses_calibrated_robot_link_transform(q, expected):
    node = NS(apply_pending=False, camera_applied=False, base_frame='link_base')
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.return_value = NS(transform=NS(
        translation=NS(x=.1, y=.2, z=.3),
        rotation=NS(x=q[0], y=q[1], z=q[2], w=q[3])))
    node._rotate = PlanningSceneObstacles._rotate
    node._box = lambda *a: PlanningSceneObstacles._box(node, *a)
    node._apply_scene = Mock()
    PlanningSceneObstacles._apply_camera_body(node)
    scene, _, done = node._apply_scene.call_args.args
    body = scene.robot_state.attached_collision_objects[0]
    assert scene.is_diff and scene.robot_state.is_diff
    assert body.link_name == body.object.header.frame_id == 'link_eef'
    assert body.touch_links == ['link_eef']
    assert list(body.object.primitives[0].dimensions) == [.02505, .090, .025]
    p = body.object.primitive_poses[0].position
    assert (p.x, p.y, p.z) == pytest.approx(expected)
    assert not node.camera_applied
    done()
    assert node.camera_applied
