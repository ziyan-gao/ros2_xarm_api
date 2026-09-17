from types import SimpleNamespace

import pytest

from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from safe_servo_visualization.motion_coordinator_node import MotionCoordinator
from std_srvs.srv import Trigger


class _ReadyClient:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        raise AssertionError('planner must not be called at observation pose')


def _supervisor():
    node = object.__new__(PickupSupervisor)
    node.active_pickup_snapshot = {
        'box_id': 4,
        'x_m': 0.30,
        'y_m': 0.02,
        'center_z_m': 0.10,
        'size_x_m': 0.11,
        'size_y_m': 0.11,
        'size_z_m': 0.15,
        'top_z_m': 0.175,
        'yaw_rad': 0.0,
    }
    node.contact_reference_z = 0.02
    node.minimum_measured_object_height = 0.02
    node.maximum_measured_object_height = 0.45
    node._tcp_xyz = lambda: (0.30, 0.02, 0.17)
    node.object_info_obtained = False
    node.contact_tcp_z = None
    node.corrected_object = None
    return node


def test_contact_z_corrects_height_and_center_pose_together():
    node = _supervisor()

    corrected = node._finalize_contact_object_info()

    assert corrected['size_z_m'] == pytest.approx(0.15)
    assert corrected['center_z_m'] == pytest.approx(0.095)
    assert corrected['top_z_m'] == pytest.approx(0.17)
    assert corrected['x_m'] == pytest.approx(0.30)
    assert corrected['y_m'] == pytest.approx(0.02)
    assert corrected['size_x_m'] == pytest.approx(0.11)
    assert corrected['size_y_m'] == pytest.approx(0.11)
    assert node.object_info_obtained is True


def test_invalid_contact_height_is_not_latched():
    node = _supervisor()
    node._tcp_xyz = lambda: (0.30, 0.02, 0.01)

    with pytest.raises(ValueError, match='outside'):
        node._finalize_contact_object_info()

    assert node.object_info_obtained is False


def test_grasp_at_contact_recomputes_retreat_from_corrected_height():
    node = _supervisor()
    node._finalize_contact_object_info()
    node.state = PickupSupervisor.AWAITING_GRASP
    node.pregrasp_z_tolerance = 0.015
    calls = []
    node._pickup_retreat_target_tcp_z = (
        lambda height, width: calls.append((height, width)) or 0.60)
    node._turn_vacuum_on = lambda: calls.append('vacuum')
    node.publish_status = lambda: None
    response = SimpleNamespace(success=False, message='')

    result = node.grasp_at_contact_callback(None, response)

    assert result.success is True
    assert calls[0] == pytest.approx((0.15, 0.11))
    assert calls[1] == 'vacuum'
    assert node.direct_target_z == pytest.approx(0.60)
    assert node.probe_only is False


def test_observation_motion_is_skipped_when_joint_pose_is_already_reached():
    coordinator = object.__new__(MotionCoordinator)
    coordinator.state = MotionCoordinator.IDLE
    coordinator.plan_client = _ReadyClient()
    coordinator.latest_joint_positions = {
        f'joint{i}': 0.1 * i for i in range(1, 7)}
    coordinator.observation_joint_tolerance = 0.02
    coordinator.operation_id = 2
    coordinator.target = None
    coordinator.planned_pregrasp = {'stale': True}
    coordinator.cancel_requested = False
    coordinator.pause_requested = False
    coordinator._require_fresh_joint_state = lambda response, action: True
    coordinator._load_waypoint = lambda name: [
        0.1 * i + 0.005 for i in range(1, 7)]
    coordinator._clear_pregrasp_snapshot = lambda: None
    coordinator._set_state = lambda state, fault='': setattr(
        coordinator, 'state', state)

    response = coordinator.plan_waypoint('observation', Trigger.Response())

    assert response.success is True
    assert coordinator.state == MotionCoordinator.SUCCEEDED
    assert coordinator.target == 'observation'
    assert coordinator.plan_client.requests == []
