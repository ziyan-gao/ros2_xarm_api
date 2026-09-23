"""Regression: unpack validation and Servo arming must use identical bounds."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from safe_servo_visualization.motion_coordinator_node import MotionCoordinator
from test_place_singularity_fallback import _low_pallet_pregrasp_supervisor
from test_pick_waypoints import coordinator


def supervisor(snapshot=None):
    node = _low_pallet_pregrasp_supervisor(retrieval=True)
    node.operation_kind = 'pickup'
    node.place_workspace_z_min_mm = -100.
    node.staging_bounds_mm = (-450., 450., 100., 750., -80., 1000.)
    node.active_pickup_snapshot = snapshot or node.motion_status['planned_pregrasp']
    node.force_threshold, node.place_force_threshold = 5., 4.
    node.servo_speed_scale, node.place_servo_speed_scale = .4, 1.
    node.config_pub = NS(publish=Mock())
    node.servo_status = dict(touch_mode=True, contact_on_fz_sign_change=False,
                            force_limit_n=5., configured_max_linear_speed_m_s=.05,
                            active_max_linear_speed_m_s=.02,
                            workspace_z_min_m=-.1, workspace_z_max_m=1.)
    return node


@pytest.mark.parametrize('snapshot,expected', [
    ({'pickup_source': 'incoming'}, [-1000., 1000., -1000., 1000., 50., 1000.]),
    ({'pickup_source': 'pallet'}, [-1000., 1000., -1000., 1000., -100., 1000.]),
    ({'pickup_source': 'buffer'}, [-450., 450., 100., 750., -80., 1000.]),
    ({'retrieval_target_id': 10}, [-1000., 1000., -1000., 1000., -100., 1000.]),
    ({'staging_slot': 0}, [-450., 450., 100., 750., -80., 1000.]),
])
def test_published_workspace_depends_on_source_without_changing_force_guards(snapshot, expected):
    node = supervisor(snapshot)
    node._publish_servo_config(touch_mode=True)
    data = node.config_pub.publish.call_args.args[0].data
    assert list(data[1:7]) == expected
    assert list(data[7:]) == [5., 1., 0., 0.]
    assert data[0] == .4


def test_real_unpack_negative_z_validated_floor_is_inside_published_workspace():
    node = supervisor()
    node.motion_status['planned_pregrasp'].update(
        pickup_source='pallet', pregrasp_z_m=-.0214, top_z_m=-.0514)
    node._tcp_xyz = lambda: (.128, -.591, -.0214)
    snapshot, start, floor = node._validate_pregrasp_ready()
    node.active_pickup_snapshot = dict(snapshot)
    node._publish_servo_config(touch_mode=True)
    data = node.config_pub.publish.call_args.args[0].data
    assert floor == pytest.approx(-.0664)
    assert start-floor == pytest.approx(.045)
    assert data[5]/1000 <= floor < start <= data[6]/1000


def test_buffer_validation_and_arming_share_buffer_floor():
    node = supervisor()
    node.motion_status['planned_pregrasp']['pickup_source'] = 'buffer'
    node.staging_bounds_mm = (-450., 450., 100., 750., -60., 1000.)
    snapshot, _, floor = node._validate_pregrasp_ready()
    assert floor == -.06
    node.active_pickup_snapshot = dict(snapshot)
    node._publish_servo_config(touch_mode=True)
    assert node.config_pub.publish.call_args.args[0].data[5] == -60.


@pytest.mark.parametrize('source,workspace,max_descent,expected', [
    ('pallet', -100., .15, -.05),
    ('pallet', -40., .15, -.04),
    ('pallet', -100., .04, -.015),
    ('buffer', -100., .15, -.020),
])
def test_fixed_pallet_floor_preserves_other_limits(source, workspace, max_descent, expected):
    node = supervisor()
    node.pallet_pickup_fixed_floor_enabled = True
    node.pallet_pickup_floor_z = -.05
    node.place_workspace_z_min_mm = workspace
    node.max_descent = max_descent
    node.motion_status['planned_pregrasp'].update(
        pickup_source=source, pregrasp_z_m=.025, top_z_m=-.005)
    node._tcp_xyz = lambda: (.128, -.591, .025)
    assert node._validate_pregrasp_ready()[2] == pytest.approx(expected)


def test_new_item_after_unpack_restores_incoming_floor():
    node = supervisor()
    node._publish_servo_config(touch_mode=True)
    assert node.config_pub.publish.call_args.args[0].data[5] == -100.
    node.active_pickup_snapshot = {'box_id': 7}
    node._publish_servo_config(touch_mode=True)
    assert node.config_pub.publish.call_args.args[0].data[5] == 50.


@pytest.mark.parametrize('status_change', [
    {'workspace_z_min_m': .05}, {'workspace_z_max_m': .8},
    {'workspace_z_min_m': None}, {'workspace_z_min_m': float('nan')},
])
def test_stale_or_missing_workspace_ack_cannot_arm_descent(status_change):
    node = supervisor()
    assert node._descent_config_confirmed()
    node.servo_status.update(status_change)
    assert not node._descent_config_confirmed()


def test_unknown_source_fails_closed():
    with pytest.raises(ValueError, match='unknown pickup source'):
        supervisor({'pickup_source': 'unknown'})._publish_servo_config(touch_mode=True)


@pytest.mark.parametrize('code,source', [(None, 'pallet'), (0., 'pallet'), (1., 'buffer')])
def test_source_survives_retrieval_target_and_prepared_snapshot(code, source):
    node = coordinator()
    values = [2., .3, .4, .2, 3.14159, 0., 0., .1, .15, .2, .03, 0., .6]
    if code is not None:
        values.append(code)
    node.staging_retrieve_target_callback(Float64MultiArray(data=values))
    assert node.staging_retrieve_target['pickup_source'] == source
    response = node.prepare_pick_waypoints_callback(None, Trigger.Response())
    assert response.success
    assert node.planned_pregrasp['pickup_source'] == source


def test_invalid_wire_source_invalidates_previous_target():
    node = coordinator()
    node.get_logger = lambda: Mock()
    values = [2., .3, .4, .2, 3.14159, 0., 0., .1, .15, .2, .03, 0., .6, 2.]
    node.staging_retrieve_target_callback(Float64MultiArray(data=values))
    assert node.staging_retrieve_target is None


def test_bridge_echo_confirms_new_limits_but_not_previous_pickup_limits(monkeypatch):
    import json
    import rclpy
    from rclpy.client import Client
    from safe_servo_package.moveit_servo_bridge import MoveItServoBridge

    def forbid_service(*args, **kwargs):
        pytest.fail('configuration test must not issue robot/Servo service commands')

    monkeypatch.setattr(Client, 'call_async', forbid_service)
    rclpy.init(args=[], domain_id=223)
    bridge = None
    try:
        bridge = MoveItServoBridge()
        bridge.status_pub = NS(publish=Mock())
        node = supervisor()
        node.servo_bounds_mm = (*node.servo_bounds_mm[:5], 800.)
        # Previous incoming pickup has identical speed/force but different Z.
        node.active_pickup_snapshot = {'pickup_source': 'incoming'}
        node._publish_servo_config(touch_mode=True)
        bridge.config_callback(node.config_pub.publish.call_args.args[0])
        node.servo_status = json.loads(bridge.status_pub.publish.call_args.args[0].data)
        assert node._descent_config_confirmed()
        node.active_pickup_snapshot = {'pickup_source': 'pallet'}
        assert not node._descent_config_confirmed()
        node._publish_servo_config(touch_mode=True)
        bridge.config_callback(node.config_pub.publish.call_args.args[0])
        node.servo_status = json.loads(bridge.status_pub.publish.call_args.args[0].data)
        assert node._descent_config_confirmed()
        assert node.servo_status['workspace_z_min_m'] == -.1
        assert not bridge.enabled
    finally:
        if bridge is not None:
            bridge.destroy_node()
        rclpy.shutdown()
