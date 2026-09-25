from types import SimpleNamespace as NS
from unittest.mock import Mock
import time

from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from safe_servo_visualization.motion_coordinator_node import MotionCoordinator


def coordinator():
    h = object.__new__(MotionCoordinator)
    h.state, h.operation_id, h.fault = h.IDLE, 4, ''
    h.last_joint_state_time = time.monotonic()
    h.joint_state_timeout = 1.
    h.pallet_locked = True
    h.pre_place_pose, h.pre_place_pose_time = Pose(), time.monotonic()
    h.place_target_xyz = [0., 0., 0.]
    h.attached_item_geometry = None
    h.transfer_tcp_pose = Pose()
    h.transfer_tcp_pose.orientation.w = 1.
    h.get_logger = Mock()
    h._calculate_place_poses = Mock()
    def state(value, reason=''):
        h.state, h.fault = value, reason
    h._set_state = state
    return h


def test_missing_geometry_waits_then_prepares_once():
    h = coordinator()
    response = h.prepare_transfer_callback(None, Trigger.Response())
    assert response.success and h.state == h.WAITING_DATA
    assert h.operation_id == 5
    h._resume_transfer_preparation()
    h._calculate_place_poses.assert_not_called()
    h.attached_item_geometry = {'size': [.1]*3}
    h._resume_transfer_preparation()
    h._resume_transfer_preparation()
    assert h.state == h.PREPARED and not h.waiting_reason
    h._calculate_place_poses.assert_called_once()
    assert h.operation_id == 5


def test_canceled_wait_cannot_resume_on_late_data():
    h = coordinator()
    h.prepare_transfer_callback(None, Trigger.Response())
    assert h._request_cancel(False, Trigger.Response()).success
    h.attached_item_geometry = {'size': [.1]*3}
    h._resume_transfer_preparation()
    assert h.state == h.IDLE
    h._calculate_place_poses.assert_not_called()


def test_invalid_geometry_still_faults():
    h = coordinator()
    h.attached_item_geometry = {'size': [.1]*3}
    h._calculate_place_poses.side_effect = ValueError('invalid geometry')
    assert not h.prepare_transfer_callback(None, Trigger.Response()).success
    assert h.state == h.FAULT


def test_placement_waits_for_matching_pickup_geometry_not_just_item_id():
    from safe_servo_visualization.place_pipeline_node import PlacePipeline
    h = object.__new__(PlacePipeline)
    h.scene_status = {'attached_item_id': 'box', 'last_attached_pickup_operation_id': 19}
    h.motion_status = {'attachment_ready': {'attached_item_id': 'box', 'pickup_operation_id': 18}}
    h.pallet_status = 'LOCKED'
    h.prepare_transfer = Mock()
    h.prepare_transfer.service_is_ready.return_value = True
    assert 'current pickup' in h._start_wait_reason()
    h.motion_status['attachment_ready']['pickup_operation_id'] = 19
    assert h._start_wait_reason() == ''


def test_new_pick_waits_for_detection_without_changing_request_id():
    h = coordinator()
    h.ik_client = Mock()
    h.ik_client.service_is_ready.return_value = True
    h._select_refined_box = Mock(side_effect=ValueError('refined box unavailable'))
    response = h.prepare_new_pick_callback(None, Trigger.Response())
    assert response.success and h.state == h.WAITING_DATA
    assert h.transfer_context == 'known_pick' and h.operation_id == 5
    h._select_refined_box.side_effect = None
    h._select_refined_box.return_value = object()
    h._marker_is_fresh = lambda box: True
    def ready(request, response, **kwargs):
        assert kwargs == {'prepare_only': True, 'prepared_operation': True}
        h.state = h.PREPARED
        response.success = True
    h.plan_pregrasp_callback = ready
    h._resume_transfer_preparation()
    assert h.state == h.PREPARED and h.operation_id == 5
