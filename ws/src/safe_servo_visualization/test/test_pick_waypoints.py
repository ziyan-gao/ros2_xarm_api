"""Offline known-source approach geometry, ownership, and completion gates."""
import json
import math
import time
from unittest.mock import Mock
from types import SimpleNamespace as NS

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK
from std_srvs.srv import Trigger
from lifecycle_msgs.msg import State

from safe_servo_visualization.transport_path import pick_waypoints, quaternion
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from safe_servo_visualization.motion_coordinator_node import MotionCoordinator
from safe_servo_visualization.pickup_pipeline_node import PickupPipeline
from safe_servo_visualization.pick_path_client import PickPathClient
from safe_servo_visualization.staging_slots_node import StagingSlots
from safe_servo_visualization.policy_loading_node import PolicyLoadingNode
from test_continuous_transport import Harness, Future, DOWN


def test_grid_route_holds_clear_height_and_preserves_required_final_pose():
    from safe_servo_visualization.transport_path import grid_pick_waypoints
    start, end = np.array([.27,-.59,.586]), np.array([.141,.585,.425])
    target_q = quaternion([0.,1.,0.,0.])
    samples, safe, high = grid_pick_waypoints(start, DOWN, end, target_q,
                                             .445, [.498,0,.425], (.15,-.15,'via'))
    assert high == pytest.approx(start[2])  # No unnecessary upward motion.
    assert np.allclose(samples[-1][0], end)
    assert abs(samples[-1][1] @ target_q) == pytest.approx(1.)
    previous = start
    for xyz, q in samples:
        if np.linalg.norm(xyz[:2]-previous[:2]) > 1e-8:
            assert xyz[2] >= safe and previous[2] >= safe
        assert xyz[2] <= high+1e-9
        previous = xyz


def test_pick_grid_is_bounded_and_never_retries_executing_motion(monkeypatch):
    from safe_servo_visualization.continuous_pick import ContinuousPick
    h = NS(transport_is_pick=True, pick_cross_area=True, state='planning',
           TRANSPORT_PLANNING='planning', TRANSPORT_DIAGNOSING='diagnosing',
           TRANSPORT_VALIDATING='validating', direct_transfer_motion_started=False,
           transfer_goal_handle=None, transport_route_generation=0, transport_seed=(0.,),
           _transport_start_fk=lambda f: None)
    calls=[]
    h._transport_fk_request=lambda *args: calls.append(args)
    clock=[10.]
    monkeypatch.setattr('safe_servo_visualization.continuous_pick.time.monotonic', lambda: clock[0])
    assert ContinuousPick._try_pick_grid(h)
    assert h.pick_grid_index == 0 and len(calls) == 1
    h.direct_transfer_motion_started=True
    assert not ContinuousPick._try_pick_grid(h)
    h.direct_transfer_motion_started=False
    clock[0]=15.
    assert ContinuousPick._try_pick_grid(h)
    assert h.pick_grid_rail == 1 and h.pick_grid_deadline == 20.
    clock[0]=20.
    assert ContinuousPick._try_pick_grid(h)
    assert h.pick_grid_direction == -1 and h.pick_grid_rail == 0
    clock[0]=25.
    assert ContinuousPick._try_pick_grid(h)
    assert h.pick_grid_rail == 1
    clock[0]=30.
    assert not ContinuousPick._try_pick_grid(h)


def test_alternate_grid_rail_uses_absolute_negative_x():
    from safe_servo_visualization.transport_path import grid_pick_waypoints
    samples, _, _ = grid_pick_waypoints(
        (.2, -.4, .2), (1, 0, 0, 0), (.2, .3, .25), (1, 0, 0, 0),
        .5, (.4981, 0, .425), (0., 0., 'via'), rail_x=-.350)
    assert any(abs(p[0]+.350) < 1e-9 for p, _ in samples)
    assert np.allclose(samples[-1][0], (.2, .3, .25))
    assert all(p[2] >= .5 for p, _ in samples if p[0] < .19)


@pytest.mark.parametrize('end_xy', [(.6, .1), (.201, .1), (.2, .1)])
@pytest.mark.parametrize('start_z', [.15, .75])
def test_pick_path_clears_current_contact_before_xy_or_rotation(end_xy, start_z):
    start = np.array([.2, .1, start_z])
    end = np.array([*end_xy, .25])
    q = quaternion([1., 0., .2, 0.])
    points, safe, high = pick_waypoints(start, q, end, DOWN, .6)
    assert high >= safe >= max(start_z, .602)
    assert np.allclose(points[-1][0], end)
    assert abs(points[-1][1] @ DOWN) == pytest.approx(1.)
    previous = start
    seen_overhead = False
    for xyz, orientation in points:
        if xyz[2] >= safe - 1e-8:
            seen_overhead = True
        if not seen_overhead:
            assert np.allclose(xyz[:2], start[:2])
            assert abs(orientation @ q) == pytest.approx(1.)
            assert xyz[2] >= previous[2] - 1e-9
        if np.linalg.norm(xyz[:2]-previous[:2]) > 1e-8:
            assert min(xyz[2], previous[2]) >= safe-1e-8
        previous = xyz


@pytest.mark.parametrize('clearance', [float('nan'), float('inf')])
def test_invalid_pick_clearance_rejected(clearance):
    with pytest.raises(ValueError):
        pick_waypoints([0, 0, .1], DOWN, [.4, 0, .2], DOWN, clearance)


def contact_supervisor():
    node = object.__new__(PickupSupervisor)
    node.state = node.AWAITING_GRASP
    node.manual_gripper_pending = False
    node._ft_recovery_blocks_start = lambda _: False
    node.dry_run = False
    node.transport_joint_limits = {'joint1': (-6., 6., 2.)}
    node.pallet_locked = True
    node.planning_scene_status = {}
    node.motion_status = dict(operation_id=7, state='PREPARED', transfer_context='known_pick')
    node.operation_id = 4
    node.object_info_obtained = True
    node.publish_status = lambda: None
    pause = Future(NS(success=True))
    node.enable_client = NS(service_is_ready=lambda: True, call_async=lambda _: pause)
    return node, pause


def test_contact_departure_waits_for_pause_and_controller_restoration():
    node, pause = contact_supervisor()
    calls = []
    node._restore_ros2_control_mode = lambda: calls.append('restore_and_settle')
    result = node.start_pick_waypoints(None, Trigger.Response())
    assert result.success and not calls
    assert not node.object_info_obtained
    assert node.state == node.PICK_PATH_DISABLING and node.pick_path_restoring
    assert node.operation_kind == 'pick_approach' and node.pick_path_target_id == 7
    pause.callback(pause)
    assert calls == ['restore_and_settle']


@pytest.mark.parametrize('blocked', ['attached', 'moving', 'fault', 'unprepared'])
def test_pick_approach_rejects_unsafe_start_without_consuming_information(blocked):
    node, _ = contact_supervisor()
    if blocked == 'attached': node.planning_scene_status = dict(attached_item_id='box')
    if blocked == 'moving': node.state = node.DESCENDING
    if blocked == 'fault': node.state = node.FAULT
    if blocked == 'unprepared': node.motion_status['state'] = 'IDLE'
    assert not node.start_pick_waypoints(None, Trigger.Response()).success
    assert node.object_info_obtained and node.operation_id == 4


def test_late_pause_ack_cannot_resume_aborted_pick():
    node, pause = contact_supervisor()
    node._restore_ros2_control_mode = lambda: pytest.fail('late restore')
    node.start_pick_waypoints(None, Trigger.Response())
    node.state = node.FAULT
    pause.callback(pause)


def test_failed_pause_never_starts_pick_motion():
    node, pause = contact_supervisor()
    faults = []
    node._fault = faults.append
    node._restore_ros2_control_mode = lambda: pytest.fail('unconfirmed pause')
    node.start_pick_waypoints(None, Trigger.Response())
    pause.callback(Future(NS(success=False)))
    assert 'handoff failed' in faults[0]


def test_healthy_pick_handoff_preserves_active_ros_control():
    node, _ = contact_supervisor()
    node.state = node.RESTORING_CONTROL
    node.pick_path_restoring = True
    node.hardware_component = 'arm'
    node.robot_state_time = time.monotonic()
    node.status_timeout = 1.
    node.robot_mode = node.ros2_control_mode = 1
    node.robot_error = 0
    node.robot_state = 0
    calls = []
    node._verify_restored_controllers = lambda: calls.append('verify')
    node._set_restore_hardware_state = lambda *_: pytest.fail('unnecessary mode switch')
    node._restore_hardware_state_received(Future(NS(component=[NS(
        name='arm', state=NS(id=State.PRIMARY_STATE_ACTIVE))])))
    assert calls == ['verify']


def test_pick_handoff_completion_plans_without_retreat_or_grasp():
    node, _ = contact_supervisor()
    node.pick_path_restoring = True
    node.post_retreat_fault = ''
    calls = []
    node._plan_pick_path = lambda: calls.append('plan')
    node._finish_retreat()
    assert calls == ['plan']


def coordinator():
    node = object.__new__(MotionCoordinator)
    node.state = node.IDLE
    node._require_fresh_joint_state = lambda *_: True
    node.pallet_locked = True
    node.attached_item_geometry = None
    node.staging_retrieve_target = dict(target_id=2, contact_tcp_pose=(.3, .4, .2, math.pi, 0., 0.),
        size=(.1, .15, .2), clearance=.03, object_yaw=0., approach_tcp_z=.6,
        received_at=time.monotonic())
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=123000000000))
    node.operation_id = 6
    node._clear_pregrasp_snapshot = lambda: None
    node._publish_pregrasp_marker = lambda *_: None
    node._set_state = lambda state: setattr(node, 'state', state)
    return node


def test_prepare_known_pick_does_not_plan_or_execute_direct_to_overhead():
    node = coordinator()
    result = node.prepare_pick_waypoints_callback(None, Trigger.Response())
    assert result.success and node.state == node.PREPARED
    assert node.operation_id == 7 and node.transfer_context == 'known_pick'
    assert node.pre_place_tcp_pose.position.z == pytest.approx(.23)
    assert node.transfer_tcp_pose.position.z == .6
    assert node.planned_pregrasp['retrieval_target_id'] == 2


@pytest.mark.parametrize('valid', [True, False])
def test_coordinator_only_accepts_matching_verified_pick_path(valid):
    node = coordinator()
    node.prepare_pick_waypoints_callback(None, Trigger.Response())
    node.pick_path_status_time = time.monotonic()
    node.pick_path_status = dict(state='SUCCEEDED', pick_path_completed=True,
        operation_kind='pick_approach', pick_path_target_id=7 if valid else 6)
    saved = []
    node._persist_pregrasp_snapshot = lambda: saved.append(True)
    response = node.accept_pick_waypoints_callback(None, Trigger.Response())
    assert response.success == valid and bool(saved) == valid


def path_client():
    futures = {}
    class Client:
        def __init__(self, name): self.name = name
        def service_is_ready(self): return True
        def call_async(self, _):
            future = Future(NS(success=True))
            futures.setdefault(self.name, []).append(future)
            return future
    faults = []
    node = NS(create_client=lambda _, name: Client(name), get_logger=Mock())
    path = PickPathClient(node, faults.append)
    path.reset(7)
    return path, futures, faults


def test_shared_client_waits_for_all_three_completion_signals():
    path, futures, faults = path_client()
    motion = dict(operation_id=7, state='PREPARED', transfer_context='known_pick')
    supervisor = dict(operation_id=4, state='AWAITING_GRASP', prepared_pick_target_id=7)
    assert not path.tick(motion, supervisor)
    start = futures['/pickup_supervisor/start_pick_waypoints'][0]
    start.callback(start)
    supervisor.update(operation_id=5, state='SUCCEEDED', pick_path_completed=True, pick_path_target_id=7)
    assert not path.tick(motion, supervisor)
    ack = futures['/motion_coordinator/accept_pick_waypoints'][0]
    assert not path.tick(motion, supervisor)  # Service still pending.
    ack.callback(ack)
    assert not path.tick(motion, supervisor)  # Coordinator topic still PREPARED.
    motion['state'] = 'SUCCEEDED'
    assert path.tick(motion, supervisor) and not faults


def test_shared_client_ignores_late_callback_after_cancel():
    path, futures, faults = path_client()
    path.tick(dict(operation_id=7, state='PREPARED', transfer_context='known_pick'), dict(operation_id=4, prepared_pick_target_id=7))
    start = futures['/pickup_supervisor/start_pick_waypoints'][0]
    path.cancel()
    start.callback(Future(NS(success=False, message='late')))
    assert not faults and path.phase == 'idle'


def test_empty_pick_contact_does_not_use_placement_release_fallback():
    node = Harness()
    node.state = node.TRANSPORT_EXECUTING
    node.transport_is_pick = True
    node.transport_descent_time = 1.
    node.transport_feedback_time = 2.
    node.transport_contact_baseline = 0.
    node.transport_force_count = 0
    node.place_force_threshold = 4.
    node.loading_contact_confirm_samples = 2
    node._transport_force(5.)
    node._transport_force(5.)
    assert node.state == node.FAULT and not node.released


def test_pick_executor_uses_collision_checked_whole_path_and_ceiling():
    node = Harness()
    node.transport_is_pick = True
    node.transport_target = dict(pre_place_tcp_xyz_m=[.6, .1, .23],
        transfer_tcp_quaternion_xyzw=DOWN, transport_corner_clearance_z_m=.6)
    node.transport_radius = .04
    node.servo_bounds_mm = [0]*5 + [620]  # Blend would exceed this ceiling.
    result = GetPositionFK.Response()
    result.error_code.val = 1
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = .2, .1, .15
    pose.pose.orientation.x = 1.
    result.pose_stamped = [pose]
    node._transport_start_fk(Future(result))
    assert 'ceiling' in node.fault


@pytest.mark.parametrize('source,start_y,uses_rail', [('buffer', .3, False),
                                                  ('buffer', -.4, True),
                                                  ('pallet', .3, True),
                                                  ('pallet', -.4, False)])
def test_same_area_skips_rail_but_cross_area_keeps_it(monkeypatch, source, start_y, uses_rail):
    node = Harness()
    node.transport_is_pick = True
    node.pick_observation_xyz = [.5, 0., .4]
    node.transport_target = dict(pre_place_tcp_xyz_m=[.3, .3 if source == 'buffer' else -.5, .23],
        planned_pregrasp={'pickup_source': source},
        transfer_tcp_quaternion_xyzw=DOWN, transport_corner_clearance_z_m=.6)
    node.transport_radius = .04
    node.servo_bounds_mm = [0]*5 + [620]
    seen = []
    def generate(*args, **kwargs):
        seen.append(kwargs['observation_xyz'])
        return pick_waypoints(*args, **kwargs)
    monkeypatch.setattr('safe_servo_visualization.continuous_transport.pick_waypoints', generate)
    result = GetPositionFK.Response()
    result.error_code.val = 1
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = .2, start_y, .15
    pose.pose.orientation.x = 1.
    result.pose_stamped = [pose]
    node._transport_start_fk(Future(result))
    assert (seen[0] is not None) is uses_rail
    assert 'ceiling' in node.fault


def test_departure_invalidates_incoming_pipeline_marker_without_retreat():
    node = object.__new__(PickupPipeline)
    node.state = node.OBJECT_INFO_READY
    calls = []
    node._clear_object_markers = lambda: calls.append('clear')
    node.publish_status = lambda: None
    node.pickup_status_callback(NS(data=json.dumps(dict(
        operation_kind='pick_approach', object_info_obtained=False))))
    assert node.state == node.IDLE and calls == ['clear']


def test_slot_retrieval_allows_estimation_contact_but_not_active_descent():
    node = object.__new__(StagingSlots)
    node.state = node.IDLE
    node.motion_status = dict(state='SUCCEEDED')
    node.pickup_status = dict(state='AWAITING_GRASP')
    assert not node._busy_reason(allow_contact=True)
    assert node._busy_reason()
    node.pickup_status['state'] = 'DESCENDING'
    assert node._busy_reason(allow_contact=True)


def test_pallet_retrieval_requests_full_pick_path_not_direct_overhead_plan():
    node = object.__new__(PolicyLoadingNode)
    message = object()
    node._pallet_item_retrieval_message = lambda _: message
    published, calls = [], []
    node.retrieval_target_pub = NS(publish=published.append)
    node.motion_status = dict(operation_id=8)
    node.pick_path = NS(reset=lambda op: calls.append(op), prepare='prepare_pick_path')
    node._defer_service = lambda client, label: calls.append(client)
    node._publish_pallet_retrieval_target(object())
    assert published == [message]
    assert calls == [9, 'prepare_pick_path']
    assert node.state == 'REARRANGE_PLAN_APPROACH'


def test_slot_approach_completion_enters_settling_not_another_direct_plan():
    node = object.__new__(StagingSlots)
    node.operation = 'retrieve'
    node._mode_tick = node._restore_tick = lambda: None
    node.phase_started = None
    node.state = node.PLANNING_RETRIEVAL_APPROACH
    node.motion_status, node.pickup_status = {}, {}
    node.pick_path = NS(tick=lambda *_: True)
    node.servo_status_sequence = 9
    node.tick()
    assert node.state == node.SETTLING_RETRIEVAL
    assert node.retrieval_settle_after_sequence == 9
    assert node.retrieval_converged_samples == 0


def test_delayed_ack_waits_without_repeating_motion():
    path, _, faults = path_client()
    path.phase = 'acknowledging'
    path.supervisor_id = 5
    path.ack_started = time.monotonic()-6
    path.pending = True
    assert not path.tick(dict(operation_id=7, state='PREPARED', transfer_context='known_pick'),
                         dict(operation_id=5, state='SUCCEEDED', pick_path_completed=True,
                              pick_path_target_id=7))
    assert not faults and path.pending and path.phase == 'acknowledging'


def test_timed_pick_geometry_rejects_lateral_shortcut_below_container():
    node = Harness()
    node.transport_local_clearance_validation_enabled = True
    node.transport_is_pick = True
    node.servo_bounds_mm = [0]*5 + [1000]
    node.transport_start_xyz = np.array([0., 0., .1])
    node.transport_end = np.array([.6, 0., .2])
    node.transport_clearance = node.transport_safe_z = .6
    node.transport_scene = None
    node.transport_check_index = 0
    node.transport_checks = [((0., 0.), .1)]
    node._transport_pose = lambda _: (np.array([.3, 0., .4]), np.array(DOWN))
    node._transport_geometry_checked(Future(None))
    assert node.state == node.FAULT and 'clearance' in node.fault
