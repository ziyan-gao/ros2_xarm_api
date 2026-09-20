"""Use real ROS messages to exercise both loaders' shared publication path."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import Float64MultiArray

from safe_servo_visualization.policy_loading_node import PolicyLoadingNode
from safe_servo_visualization.random_stable_loading_node import RandomStableLoadingNode


@pytest.mark.parametrize('node_type,height', [
    (PolicyLoadingNode, .47), (RandomStableLoadingNode, .59)])
@pytest.mark.parametrize('with_virtual_corner', [False, True])
def test_target_publication_serializes_height_and_geometry(node_type, height, with_virtual_corner):
    node = object.__new__(node_type)
    node.transfer_corner_height = height
    node.target_pub = Mock()
    values = (1., 9., 20., 30., 0., 1., 110., 155., 205., 130., 180., 205.)
    if with_virtual_corner:
        values += (0., 10., 0.)
    pending = SimpleNamespace(target_values=values)

    node._publish_target(pending)

    message = node.target_pub.publish.call_args.args[0]
    assert isinstance(message, Float64MultiArray)
    decoded = deserialize_message(serialize_message(message), Float64MultiArray)
    assert list(decoded.data[:12]) == list(values[:12])
    assert list(decoded.data[12:15]) == list(values[12:15] if with_virtual_corner else values[2:5])
    assert len(decoded.data) == 16
    assert decoded.data[15] == height
    assert pending.target_values == values
    assert node.last_target_publish > 0

    node._publish_target(pending)
    assert list(node.target_pub.publish.call_args.args[0].data) == list(decoded.data)
