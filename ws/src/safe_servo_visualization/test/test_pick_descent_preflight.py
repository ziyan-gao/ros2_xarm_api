from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from safe_servo_visualization.continuous_pick import ContinuousPick
from safe_servo_visualization.motion_coordinator_node import MotionCoordinator
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from rclpy.time import Time


@pytest.mark.parametrize('fraction,position,accepted', [(1., .5, True), (.6, .5, False), (1., 3., False)])
def test_contact_probe_blocks_partial_or_out_of_bounds(fraction, position, accepted):
    node = NS(_pick_descent_pending='key', arm_joint_names=['joint3'],
              transport_joint_limits={'joint3': (-2., 2., 1.)},
              _transport_execute=Mock(), _fault=Mock(), _try_pick_grid=Mock(return_value=False),
              get_logger=lambda: Mock())
    result = NS(error_code=NS(val=1), fraction=fraction,
                solution=NS(joint_trajectory=NS(joint_names=['joint3'], points=[NS(positions=[position])])))
    ContinuousPick._pick_descent_result(node, NS(result=lambda: result), 'key')
    assert node._transport_execute.called == accepted
    assert node._fault.called != accepted


def test_camera_route_does_not_probe_contact():
    node = NS(transport_is_pick=True, transport_target={'planned_pregrasp': {'inspection_only': True}})
    assert ContinuousPick._check_pick_descent_before_execution(node) is False


def test_obsolete_probe_does_not_execute():
    node = NS(_pick_descent_pending='new', _transport_execute=Mock())
    ContinuousPick._pick_descent_result(node, Mock(), 'old')
    node._transport_execute.assert_not_called()


@pytest.mark.parametrize('y,source', [(-.4, 'pallet'), (.4, 'buffer')])
def test_camera_move_prepares_shared_route_not_pose_plan(y, source):
    target = PoseStamped()
    target.header.frame_id = 'link_base'
    target.header.stamp = Time(seconds=10).to_msg()
    target.pose.position.y, target.pose.position.z = y, .5
    target.pose.orientation.w = 1.
    node = NS(state='IDLE', IDLE='IDLE', SUCCEEDED='SUCCEEDED', PREPARED='PREPARED',
              top_face_view=target, get_clock=lambda: NS(now=lambda: Time.from_msg(target.header.stamp)),
              _require_fresh_joint_state=Mock(return_value=True), attached_item_geometry=None,
              pallet_locked=True, _clear_pregrasp_snapshot=Mock(), _set_state=Mock(),
              _start_pose_plan=Mock(), operation_id=4)
    response = MotionCoordinator.plan_top_face_view_callback(node, None, Trigger.Response())
    assert response.success
    node._start_pose_plan.assert_not_called()
    node._set_state.assert_called_once_with('PREPARED')
    assert node.transfer_context == 'known_pick'
    assert node.planned_pregrasp == {'inspection_only': True, 'pickup_source': source}
    assert node.target == 'top_face_view'
