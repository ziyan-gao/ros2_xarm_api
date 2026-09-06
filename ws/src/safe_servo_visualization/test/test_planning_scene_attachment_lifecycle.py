import json

from std_msgs.msg import String

from safe_servo_visualization.planning_scene_obstacles_node import (
    PlanningSceneObstacles,
)


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)


def _node(last_attached=-1):
    node = object.__new__(PlanningSceneObstacles)
    node.place_singularity_fallback = False
    node.pending_pickup_operation_id = None
    node.last_attached_pickup_operation_id = last_attached
    node.attachment_pending = False
    node.attached_item_id = ''
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    node.attach_calls = []
    node._attach_detected_item = lambda: node.attach_calls.append(True)
    return node


def _pickup_status(operation_id, *, kind='pickup'):
    message = String()
    message.data = json.dumps({
        'operation_kind': kind,
        'operation_id': operation_id,
        'dry_run': False,
        'contact_detected': True,
        'vacuum_verified': True,
        'place_fallback_used': False,
    })
    return message


def test_stale_consumed_pickup_status_cannot_rearm_attachment():
    node = _node(last_attached=7)

    node.pickup_status_callback(_pickup_status(7))

    assert node.attachment_pending is False
    assert node.pending_pickup_operation_id is None
    assert node.attach_calls == []


def test_new_verified_pickup_operation_arms_attachment_once():
    node = _node(last_attached=7)

    node.pickup_status_callback(_pickup_status(8))

    assert node.attachment_pending is True
    assert node.pending_pickup_operation_id == 8
    assert node.attach_calls == [True]
    assert 'operation 8' in node.logger.infos[0]


def test_place_status_never_arms_item_attachment():
    node = _node(last_attached=7)

    node.pickup_status_callback(_pickup_status(8, kind='place'))

    assert node.attachment_pending is False
    assert node.attach_calls == []


def test_successful_attachment_consumes_pickup_operation_id():
    node = _node(last_attached=7)
    node.pending_pickup_operation_id = 8
    node.publish_status = lambda: None

    node._attachment_succeeded(
        'carried_item_1', (0.2, 0.1, 0.16), (0.0, 0.0, -0.08),
        (0.0, 0.0, 0.0, 1.0), 8)

    assert node.last_attached_pickup_operation_id == 8
    assert node.pending_pickup_operation_id is None
    assert node.attachment_pending is False

    # Simulate completed detachment, then deliver a queued status message
    # from the already-consumed pickup operation.
    node.attached_item_id = ''
    node.pickup_status_callback(_pickup_status(8))
    assert node.attachment_pending is False
    assert node.attach_calls == []
