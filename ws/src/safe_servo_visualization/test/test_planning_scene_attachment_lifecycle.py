import json

from std_msgs.msg import String

from safe_servo_visualization.planning_scene_obstacles_node import (
    PlanningSceneObstacles,
)
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor


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


class TriggerResponse:
    def __init__(self):
        self.success = False
        self.message = ''


def test_clear_picked_item_removes_attachment_without_adding_placed_obstacle():
    node = object.__new__(PlanningSceneObstacles)
    node.apply_pending = False
    node.motion_state = 'SUCCEEDED'
    node.pickup_state = 'SUCCEEDED'
    node.attachment_pending = False
    node.attached_item_id = 'carried_item_9'
    node.base_frame = 'link_base'
    captured = {}

    def apply_scene(scene, description, on_success):
        captured['scene'] = scene
        captured['description'] = description
        captured['on_success'] = on_success
        return True

    node._apply_scene = apply_scene
    response = TriggerResponse()

    node.clear_picked_item_callback(None, response)

    assert response.success is True
    assert 'gripper command unchanged' in response.message
    assert captured['description'] == 'clear picked item carried_item_9'
    assert len(captured['scene'].robot_state.attached_collision_objects) == 1
    assert captured['scene'].robot_state.attached_collision_objects[
        0].object.id == 'carried_item_9'
    assert len(captured['scene'].world.collision_objects) == 1
    assert captured['scene'].world.collision_objects[0].id == 'carried_item_9'


def test_clear_picked_item_is_idempotent_when_nothing_is_attached():
    node = object.__new__(PlanningSceneObstacles)
    node.apply_pending = False
    node.motion_state = 'SUCCEEDED'
    node.pickup_state = 'SUCCEEDED'
    node.attachment_pending = False
    node.attached_item_id = ''
    response = TriggerResponse()

    node.clear_picked_item_callback(None, response)

    assert response.success is True
    assert response.message == 'no picked item to clear'


class FakeServiceResult:
    def __init__(self, **values):
        self.__dict__.update(values)


class FakeFuture:
    def __init__(self, result):
        self._result = result
        self.callback = None

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self.callback = callback


class FakeClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []
        self.future = FakeFuture(FakeServiceResult(success=True, message='ok'))

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def test_manual_open_clears_attachment_without_placement_detach():
    node = object.__new__(PickupSupervisor)
    node.manual_gripper_pending = True
    node.manual_gripper_state = 'open pending'
    node.planning_scene_status = {'attached_item_id': 'carried_item_1000'}
    node.clear_picked_item_client = FakeClient()
    node.detach_item_client = FakeClient()
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    node.publish_status = lambda: None

    node._manual_gripper_completed(
        FakeFuture(FakeServiceResult(ret=0)), close=False)

    assert len(node.clear_picked_item_client.requests) == 1
    assert node.clear_picked_item_client.future.callback == (
        node._manual_clear_picked_completed)
    assert node.detach_item_client.requests == []


def test_manual_close_does_not_change_scene_attachment():
    node = object.__new__(PickupSupervisor)
    node.manual_gripper_pending = True
    node.manual_gripper_state = 'close pending'
    node.planning_scene_status = {'attached_item_id': 'carried_item_1000'}
    node.clear_picked_item_client = FakeClient()
    node.detach_item_client = FakeClient()
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    node.publish_status = lambda: None

    node._manual_gripper_completed(
        FakeFuture(FakeServiceResult(ret=0)), close=True)

    assert node.clear_picked_item_client.requests == []
    assert node.detach_item_client.requests == []
