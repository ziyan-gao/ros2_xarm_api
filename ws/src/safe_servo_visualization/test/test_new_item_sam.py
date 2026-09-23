from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from visualization_msgs.msg import Marker
from safe_servo_visualization.new_item_sam import refined_marker, NewItemSAM
from safe_servo_visualization.top_face_automation import TopFaceAutomation
import time
from builtin_interfaces.msg import Time


def test_successful_sam_does_not_require_another_live_depth_box():
    coarse = Marker()
    coarse.pose.position.z = .15
    coarse.scale.x = coarse.scale.y = coarse.scale.z = .1
    node = NS(motion_status={}, pickup_status={}, sam_coarse_pub=Mock(),
              sam_coarse_marker=coarse, new_item_sam_client=Mock(),
              _fresh_box=Mock(side_effect=AssertionError('must not reread live depth')),
              stable_box_pub=Mock(), WAIT_DETECTION='WAIT_DETECTION',
              publish_status=Mock(), get_clock=lambda: NS(now=lambda: NS(to_msg=Time)))
    node.new_item_sam_client.tick.return_value = dict(
        top_center_base_m=[.01, -.01, .2], size_m=[.105, .095, .1], yaw_rad=.05)
    NewItemSAM._tick_new_item_sam(node)
    node._fresh_box.assert_not_called()
    node.new_item_sam_client.cancel.assert_not_called()
    node.stable_box_pub.publish.assert_called_once()
    marker = node.stable_box_pub.publish.call_args.args[0].markers[0]
    assert marker.pose.position.x == .01
    assert marker.pose.position.z == .15
    assert marker.scale.z == .1
    assert node.state == 'WAIT_DETECTION'
    assert node.stable_box_published_at is not None


def test_refinement_keeps_z_and_height_and_does_not_mutate_coarse():
    coarse = Marker()
    coarse.pose.position.z = .15
    coarse.scale.x = coarse.scale.y = coarse.scale.z = .1
    result = dict(top_center_base_m=[.01, -.01, .2], size_m=[.105, .095, .1], yaw_rad=.05)
    refined = refined_marker(coarse, result)
    assert refined.pose.position.z == .15
    assert refined.scale.z == .1
    assert refined.pose.position.x == .01
    assert refined.scale.x == .105
    assert coarse.pose.position.x == 0.
    result['top_center_base_m'][2] = .3
    with pytest.raises(ValueError):
        refined_marker(coarse, result)


def test_sam_failure_waits_without_publishing_pregrasp():
    node = NS(motion_status={}, pickup_status={},
              sam_coarse_pub=Mock(), sam_coarse_marker=Marker(),
              new_item_sam_client=Mock(), _reset_detection_samples=Mock(),
              WAIT_DETECTION='WAIT_DETECTION', publish_status=Mock(), get_logger=lambda: Mock())
    node.new_item_sam_client.tick.side_effect = ValueError('bad mask')
    NewItemSAM._tick_new_item_sam(node)
    assert node.state == 'WAIT_DETECTION'
    node._reset_detection_samples.assert_called_once()
    node.new_item_sam_client.cancel.assert_called_once()
    assert node.new_item_sam_retry_at > time.monotonic()


def test_new_item_owner_requires_test_and_estimator():
    node = NS(inspection=dict(owner='new_item', request_id='one'),
              motion_status={'estimate': dict(state='SAM_REFINEMENT', sam_request_id='one'),
                             'test': dict(state='ESTIMATING')},
              motion_seen={k: time.monotonic() for k in ('test', 'estimate')})
    assert TopFaceAutomation._inspection_owners(node) == {'test', 'estimate'}
    node.motion_status['test']['state'] = 'FAULT'
    with pytest.raises(ValueError):
        TopFaceAutomation._inspection_owners(node)
