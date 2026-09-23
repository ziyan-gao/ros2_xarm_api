"""Offline tests; never connect to robot services."""
import math
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import pytest
from std_msgs.msg import String
from safe_servo_visualization.top_face_geometry import transform, top_points, project
from safe_servo_visualization.top_face_motion import centered_camera_pose, recorded_grasp_rpy, TopFaceMotion
from safe_servo_visualization.top_face_debug_node import TopFaceDebug


@pytest.mark.parametrize('quaternion', [
    [-.0073325496987586315, .0046666108323052975, -.6956960336875639, -.7182837079940257],
    [math.sin(math.radians(15)/2), 0., 0., math.cos(math.radians(15)/2)],
])
def test_target_geometry_preserves_tilt_without_horizontal_gate(quaternion):
    from visualization_msgs.msg import Marker
    marker = Marker()
    marker.header.frame_id = 'link_base'
    marker.pose.position.x = .23
    marker.pose.position.y = -.62
    marker.pose.position.z = -.11
    q = marker.pose.orientation
    q.x, q.y, q.z, q.w = quaternion
    marker.scale.x, marker.scale.y, marker.scale.z = .16, .217, .125
    node = NS(targets={'placed:test:1': marker}, marker_seen={'placed': time.monotonic()},
              base='link_base', _matrix=Mock(return_value=np.eye(4)))
    box, size, points = TopFaceMotion._target_geometry(node, 'placed:test:1')
    expected = transform([.23, -.62, -.11], quaternion)
    assert box == pytest.approx(expected)
    assert points == pytest.approx(top_points(size, expected))
    assert np.ptp(points[1:, 2]) > .001  # No silent flattening of the recorded face.


def test_camera_centering_uses_extrinsic_translation_and_image_center():
    tcp = transform([.4, .2, .6], [1, 0, 0, 0])
    extrinsic = transform([.05, .03, .02], [0, 0, 0, 1])
    points = top_points([.15, .12, .1], transform([.1, -.1, .15], [0, 0, 0, 1]))
    k = np.array([[600., 0, 315.], [0, 610., 245.], [0, 0, 1.]])
    result = centered_camera_pose(tcp, extrinsic, points, k, np.zeros(5), 640, 480, .6)
    pixels = project(points, np.linalg.inv(result@extrinsic), k, np.zeros(5))
    assert pixels[0] == pytest.approx([320, 240])
    assert result[2, 3] == pytest.approx(.6)
    assert result[:3, :3] == pytest.approx(tcp[:3, :3])


def test_camera_rejects_upward_and_too_low():
    points = top_points([.15, .12, .1], transform([0, 0, .15], [0, 0, 0, 1]))
    for z, message in ((.25, 'height'), (.6, 'looking down')):
        with pytest.raises(ValueError, match=message):
            centered_camera_pose(np.eye(4), np.eye(4), points, np.eye(3), np.zeros(5), 640, 480, z)


@pytest.mark.parametrize('tool_yaw,object_yaw,rotation', [(20., 90., 2.), (-30., 15., 90.), (170., -20., 20.)])
def test_pick_preserves_recorded_tool_object_relationship(tool_yaw, object_yaw, rotation):
    # For a downward tool R_tcp_object = Rz(tool_yaw-object_yaw) Rx(pi).
    half = math.radians(tool_yaw-object_yaw)/2
    recorded = [math.cos(half), math.sin(half), 0., 0.]
    roll, pitch, yaw = recorded_grasp_rpy(recorded, math.radians(object_yaw+rotation))
    assert abs(roll) == pytest.approx(math.pi)
    assert pitch == pytest.approx(0.)
    delta = (yaw-math.radians(tool_yaw+rotation)+math.pi) % (2*math.pi)-math.pi
    assert delta == pytest.approx(0.)


def test_pick_does_not_guess_missing_recorded_grasp():
    with pytest.raises(ValueError, match='recorded grasp orientation missing'):
        recorded_grasp_rpy(None, 0.)


def test_scene_preserves_grasp_before_detaching():
    from safe_servo_visualization.planning_scene_obstacles_node import PlanningSceneObstacles
    q = [.9, .1, 0., 0.]
    node = NS(attached_item_orientation=q, placed_item_counter=0, placed_item_visuals={},
              placed_item_ids=[], _publish_placed_item_visuals=Mock(), publish_status=Mock())
    PlanningSceneObstacles._detachment_succeeded(node, 'placed_item_1', NS(id='placed_item_1'))
    assert node.attached_item_orientation is None
    assert node.placed_grasp_orientations[1] == q


def idle():
    node = NS(motion_status={k: {'state': 'IDLE'} for k in ('motion', 'pickup', 'scene', 'slots')},
              motion_seen={k: time.monotonic() for k in ('motion', 'pickup', 'scene', 'slots')})
    return node


def test_idle_interlock_rejects_automatic_loop_and_carried_object():
    node = idle()
    TopFaceMotion._idle_checks(node)
    node.motion_status['test'] = {'state': 'SUCCEEDED', 'random_active': True}
    with pytest.raises(ValueError, match='automatic'):
        TopFaceMotion._idle_checks(node)
    del node.motion_status['test']
    node.motion_status['scene']['attached_item_id'] = 'item'
    with pytest.raises(ValueError, match='empty tool'):
        TopFaceMotion._idle_checks(node)


def test_no_motion_while_sampling_and_clear_cannot_cancel_inflight_operation():
    node = NS(motion_phase='PICKUP', message='', _publish_status=Mock(), _motion_fault=Mock())
    TopFaceDebug._command(node, String(data='{"action":"clear"}'))
    assert node.motion_phase == 'PICKUP'
    node._motion_fault.assert_not_called()
    TopFaceDebug._command(node, String(data='{"action":"stop"}'))
    node._motion_fault.assert_called_once()


def test_stale_result_cannot_start_pick():
    node = NS(_idle_checks=Mock(), _target_geometry=Mock(return_value=(np.eye(4), [.1]*3, np.zeros((5, 3)))),
              snapshot={'meta': {'target': 'box', 'color_stamp': 1}},
              result={'top_center_base_m': [0, 0, .2]},
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)),
              get_parameter=lambda name: NS(value=30))
    with pytest.raises(ValueError, match='expired'):
        TopFaceMotion._begin_motion(node, 'pick', 'box')


def tick_node(phase):
    node = idle()
    node.motion_phase = phase
    node.motion_started = node.phase_started = time.monotonic()
    node.motion_future = None
    node.debug_pick_id = 7
    node.debug_motion_id = 4
    node._motion_fault = Mock()
    node._phase = Mock()
    node.motion_status['motion'].update(operation_id=4, state='SUCCEEDED')
    return node


def test_pick_success_requires_force_and_matching_attachment():
    node = tick_node('PICKUP')
    node.motion_status['pickup'].update(operation_id=7, state='SUCCEEDED', contact_detected=False)
    TopFaceMotion._motion_tick(node)
    assert 'force' in str(node._motion_fault.call_args)
    node._motion_fault.reset_mock()
    node.motion_status['pickup']['contact_detected'] = True
    TopFaceMotion._motion_tick(node)
    node._phase.assert_not_called()
    node.motion_status['scene'].update(attached_item_id='item', last_attached_pickup_operation_id=6)
    TopFaceMotion._motion_tick(node)
    node._phase.assert_not_called()
    node.motion_status['scene']['last_attached_pickup_operation_id'] = 7
    node.debug_slot = None
    TopFaceMotion._motion_tick(node)
    node._phase.assert_called_once_with('PICK_DONE')


def test_superseded_view_is_not_executed():
    node = tick_node('VIEW_PLAN')
    node.motion_status['motion'].update(operation_id=5, state='PLANNED', target='top_face_view')
    TopFaceMotion._motion_tick(node)
    assert 'replaced' in str(node._motion_fault.call_args)


def test_accepted_target_does_not_expire_while_robot_approaches():
    node = tick_node('PICK_SETTLE')
    node.phase_started -= 1
    node.snapshot = {'meta': {'color_stamp': 1., 'base_from_box': np.eye(4), 'size_m': [.1]*3}}
    node.debug_target = 'box'
    node.debug_source_id = None
    node._target_geometry = Mock(return_value=(np.eye(4), [.1]*3, None))
    TopFaceMotion._motion_tick(node)
    node._motion_fault.assert_not_called()
    node._phase.assert_called_once_with('REMOVE_SOURCE')


def test_changed_target_still_blocks_descent_after_approach():
    node = tick_node('PICK_SETTLE')
    node.phase_started -= 1
    node.snapshot = {'meta': {'color_stamp': 1., 'base_from_box': np.eye(4), 'size_m': [.1]*3}}
    node.debug_target = 'box'
    moved = np.eye(4)
    moved[0, 3] = .01
    node._target_geometry = Mock(return_value=(moved, [.1]*3, None))
    TopFaceMotion._motion_tick(node)
    assert 'target changed' in str(node._motion_fault.call_args)
    node._phase.assert_not_called()


def test_slot_inventory_requires_verified_pickup():
    from safe_servo_visualization.staging_slots_node import StagingSlots
    node = NS(state='IDLE', occupied={2: {'item_id': 'box'}},
              pickup_status={'state': 'SUCCEEDED', 'operation_id': 7},
              motion_status={'planned_pregrasp': {'pickup_source': 'buffer', 'retrieval_target_id': 2}},
              scene_status={'attached_item_id': 'box', 'last_attached_pickup_operation_id': 6},
              publish_status=Mock())
    request, response = NS(data=2), NS(ret=0, message='')
    StagingSlots.accept_debug_pick_callback(node, request, response)
    assert response.ret != 0 and 2 in node.occupied
    node.scene_status['last_attached_pickup_operation_id'] = 7
    StagingSlots.accept_debug_pick_callback(node, request, response)
    assert response.ret == 0 and 2 not in node.occupied
