"""Offline contract and adapter tests for pack/unpack/repack composition."""
import json
import time
from types import SimpleNamespace as NS

import pytest
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from safe_servo_visualization.pick_place_workflow import pick_and_place
from safe_servo_visualization.policy_loading_node import PolicyLoadingNode
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from safe_servo_visualization.place_pipeline_node import PlacePipeline
from safe_servo_visualization.staging_slots_node import StagingSlots
from test_continuous_transport import Harness, Future, DOWN
from test_continuous_return import staged_supervisor, return_supervisor


@pytest.mark.parametrize('source,destination', [
    ('incoming', 'pallet'), ('pallet', 'slot'), ('pallet', 'pallet'),
    ('slot', 'pallet'), ('carried', 'slot'),
])
@pytest.mark.parametrize('returning', [True, False])
def test_recipe_always_releases_then_clears_before_ending(source, destination, returning):
    recipe = pick_and_place(source, destination, pre_pick_pose={'record': 1},
                            return_to_observation=returning)
    assert recipe.stages[-3:] == ('contact_place', 'slow_clearance',
                                  'overhead_observation' if returning else 'overhead_handoff')
    assert ('estimate_at_observation' in recipe.stages) == (source == 'incoming')
    if source == 'slot': assert recipe.pickup_service.endswith('retrieve_chained')
    if source == 'pallet': assert recipe.pickup_service.endswith('start_for_transport')
    if not returning: assert recipe.placement_service.endswith('chained')


@pytest.mark.parametrize('source', ['slot', 'pallet'])
def test_known_source_without_record_is_not_estimated_as_new_item(source):
    with pytest.raises(ValueError, match='source record'):
        pick_and_place(source, 'pallet')


@pytest.mark.parametrize('kind,source,destination', [
    ('pack', 'incoming', 'pallet'), ('pack', 'holding', 'pallet'),
    ('unpack', 'pallet', 'slot'), ('repack', 'pallet', 'pallet'),
])
def test_policy_returns_after_every_unpack_or_last_operation(kind, source, destination):
    op = NS(kind=kind, source=source, source_item=object(), step_index=0, step_count=2)
    recipe = PolicyLoadingNode._operation_workflow(op)
    assert recipe.destination == destination
    assert recipe.return_to_observation is (kind == 'unpack')
    op.step_index = 1
    assert PolicyLoadingNode._operation_workflow(op).return_to_observation


def test_known_pickup_requests_deferred_lift_but_normal_pickup_does_not():
    node = object.__new__(PickupSupervisor)
    calls = []
    node._start_pickup_descent = lambda response, **options: calls.append(options)
    node.start_for_transport_callback(None, Trigger.Response())
    node.start_callback(None, Trigger.Response())
    assert calls == [dict(probe_only=False, defer_lift=True), dict(probe_only=False)]


def test_chained_pickup_does_not_move_to_observation_or_lift():
    node = object.__new__(PickupSupervisor)
    node.operation_kind = 'pickup'
    node.defer_pickup_lift = True
    node._should_return_continuously = lambda: False
    node.publish_status = lambda: None
    node._disable_servo_then_direct_retreat = lambda: pytest.fail('unexpected lift')
    node._proceed_to_retreat(vacuum_verified=True)
    assert node.state == node.SUCCEEDED and not node.defer_pickup_lift


def test_slot_chained_retrieval_selects_hold_service_after_settling():
    node = object.__new__(StagingSlots)
    node.return_to_observation = False
    node.motion_status = {'planned_pregrasp': {'record': 1}}
    node.pickup_status = {'operation_id': 3}
    node._settle_tcp = lambda *_: True
    calls = []
    node.start_pickup_hold = NS(service_is_ready=lambda: True,
                               call_async=lambda req: calls.append(req) or Future(NS(success=True)))
    node._settle_retrieval()
    assert len(calls) == 1
    assert node.expected_pickup_operation_id == 4 and node.state == node.PICKING_RETRIEVAL


def test_staging_clearance_tracks_taller_container_and_never_shrinks():
    node = object.__new__(StagingSlots)
    node.transfer_item_bottom_above_pallet = .48
    for height in (570, 450):
        node._container_clearance_status(NS(data=json.dumps({'container_size_mm': [450, 500, height]})))
    assert node.transfer_item_bottom_above_pallet == pytest.approx(.59)


def test_staging_contact_waits_for_transport_and_coordinator_ack():
    node = object.__new__(StagingSlots)
    node.state = node.TRANSFERRING_STORE
    node.store_transfer_phase = 'continuous'
    node.expected_store_transfer_operation_id = 6
    node.expected_motion_operation_id = 2
    node.pickup_status = dict(operation_id=6, state='SUCCEEDED', direct_transfer_succeeded=True)
    node.motion_status = dict(operation_id=2, state='PREPARED')
    ack = Future(NS(success=True))
    node.accept_direct_transfer = NS(service_is_ready=lambda: True, call_async=lambda _: ack)
    calls = []
    node._begin_safe_servo_store = lambda: calls.append('servo')
    node._fault = lambda reason: pytest.fail(reason)
    node._tick_store_transfer()
    assert not calls and node.store_transfer_phase == 'continuous_accepting'
    ack.callback(ack)
    node._tick_store_transfer()
    assert not calls
    node.motion_status['state'] = 'SUCCEEDED'
    node._tick_store_transfer()
    assert calls == ['servo']


@pytest.mark.parametrize('reason,started,fallback', [
    ('KINEMATIC_REJECTED: partial Cartesian path', False, False),
    ('KINEMATIC_REJECTED: path', True, False),
    ('xArm error 52', False, False), ('controller failed', True, False),
])
def test_staging_failure_cannot_skip_lift_when_grasp_started_at_contact(reason, started, fallback):
    node = object.__new__(StagingSlots)
    node.store_transfer_phase = 'continuous'
    node.expected_store_transfer_operation_id = 6
    node.pickup_status = dict(operation_id=6, state='FAULT', fault=reason,
                              direct_transfer_motion_started=started)
    calls = []
    node._begin_store_moveit_fallback = lambda _: calls.append('fallback')
    node._fault = lambda _: calls.append('fault')
    node._tick_store_transfer()
    assert calls == ['fallback' if fallback else 'fault']


def test_chained_release_keeps_slow_servo_pause_handoff_for_staging():
    node = return_supervisor()
    node.staging_place_active = True
    node.return_to_observation = False
    assert node._should_return_continuously()
    pause = Future(NS(success=True))
    node.enable_client = NS(service_is_ready=lambda: True, call_async=lambda _: pause)
    node._begin_continuous_return()
    calls = []
    node._begin_direct_retreat = lambda: calls.append(node.direct_target_z)
    assert not calls and node.return_clearance_pending
    pause.callback(pause)
    assert calls == [.15]  # slow clearance first, NOT the overhead goal


def test_chained_return_does_not_read_observation_file_and_keeps_clearance():
    node, _, _ = staged_supervisor()
    node.return_to_observation = False
    node.return_clearance_pending = False
    node.return_waypoint_file = '/file-that-does-not-exist'
    node.motion_status.update(transfer_tcp_z_m=.62)
    node.transport_high_z = .66
    node.latest_joint_positions = (0., 0.)
    for name in ('transport_fk', 'transport_cartesian', 'state_validity_client'):
        setattr(node, name, NS(service_is_ready=lambda: True))
    calls = []
    node._transport_fk_request = lambda seed, callback: calls.append((seed, callback))
    node._plan_continuous_return()
    assert not node.fault
    assert node.return_goal_joints is None
    assert node.return_clearance_z == .66
    assert calls[0][1] == node._transport_start_fk


def test_chained_return_generates_only_vertical_empty_tool_path():
    node = Harness()
    node.transport_is_return = True
    node.return_to_observation = False
    node.return_goal_joints = None
    node.transport_seed = (0., 0.)
    node.transport_target = dict(pre_place_tcp_xyz_m=[.2, .3, .15],
                                 transfer_tcp_quaternion_xyzw=DOWN,
                                 transport_corner_clearance_z_m=.66)
    node.servo_bounds_mm = [0]*5 + [900]
    node.planning_group, node.ik_link_name = 'uf850', 'link_tcp'
    node.arm_joint_names = ['joint1', 'joint2']
    node.motion_speed_percent = 50.
    node.operation_id = 1
    requests = []
    node.transport_cartesian = NS(call_async=lambda req: requests.append(req) or Future(None))
    result = GetPositionFK.Response()
    result.error_code.val = 1
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = .2, .3, .15
    pose.pose.orientation.x = 1.
    result.pose_stamped = [pose]
    node._transport_start_fk(Future(result))
    assert not node.fault
    points = requests[0].waypoints
    assert all(p.position.x == .2 and p.position.y == .3 for p in points)
    assert points[-1].position.z == pytest.approx(.66)
    # Geometry planning permits initial released-item overlap. Final timed
    # states are classified before execution; only vertical retreat is exempt.
    assert not requests[0].avoid_collisions


def test_chained_place_refuses_completion_at_only_pre_place():
    node = object.__new__(PlacePipeline)
    node.return_to_observation = False
    node.supervisor_status = dict(continuous_return_completed=False)
    faults = []
    node._fault = faults.append
    node._begin_observation_motion()
    assert faults == ['chained placement did not confirm its overhead handoff']


@pytest.mark.parametrize('confirmed', [False, True])
def test_staging_records_release_but_requires_overhead_before_chaining(confirmed):
    node = object.__new__(StagingSlots)
    node.expected_store_place_operation_id = 7
    node.pickup_status = dict(operation_id=7, release_tcp_pose_mm_rad=[0.] * 6,
                             continuous_return_completed=confirmed,
                             return_to_observation=False)
    node.scene_status = dict(placed_item_ids=['placed_item_1'])
    node.placed_ids_before_store = set()
    node.pending_record = dict(slot=2, orientation=(1., 0., 0., 0.))
    node.occupied = {}
    node.return_to_observation = False
    calls = []
    node._fault = lambda _: calls.append('fault')
    node._finish_operation = lambda: calls.append('finish')
    node._begin_observation = lambda: pytest.fail('unexpected observation detour')
    node._complete_safe_servo_store()
    assert node.occupied[2]['placed_obstacle_id'] == 'placed_item_1'
    assert calls == ['finish' if confirmed else 'fault']


def test_chained_retrieval_consumes_slot_only_after_attachment_without_observation():
    node = object.__new__(StagingSlots)
    node.operation = 'retrieve'
    node._mode_tick = node._restore_tick = lambda: None
    node.phase_started = None
    node.state = node.PICKING_RETRIEVAL
    node.expected_pickup_operation_id = 5
    node.pickup_status = dict(operation_id=5, state='SUCCEEDED')
    node.scene_status = {}
    node.active_slot = 1
    node.occupied = {1: {'item': 'record'}}
    node.return_to_observation = False
    calls = []
    node._finish_operation = lambda: calls.append('finish')
    node._begin_observation = lambda: pytest.fail('unexpected observation detour')
    node.tick()
    assert 1 in node.occupied and not calls
    node.scene_status['attached_item_id'] = 'carried_item'
    node.tick()
    assert 1 not in node.occupied and calls == ['finish']
