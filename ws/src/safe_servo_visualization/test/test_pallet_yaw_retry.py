"""Offline placement retry checks; no hardware commands."""
import copy
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from safe_servo_visualization.transport_path import rotate
from safe_servo_visualization.joint_equivalence import nearest_equivalent_joints
from test_transport_alternatives import Harness, Future, Client


def make_harness(tmp_path):
    h = Harness(tmp_path)
    h.direct_moveit_active = True
    h.transport_moveit_pipeline_id = 'isaac_ros_cumotion'
    h.transport_target = dict(operation_id=8, transfer_context='pallet',
        pre_place_tcp_xyz_m=[.3, -.6, .2], transfer_tcp_quaternion_xyzw=[1., 0., 0., 0.])
    h.transport_scene['attached_item_center_in_tcp_m'] = [.04, -.02, .1]
    h.planning_scene_status = {'attached_item_id': 'box'}
    h.latest_joint_positions = (.1, .2)
    return h


def test_retry_preserves_box_center_and_replans_once_from_measured_state(tmp_path):
    h = make_harness(tmp_path)
    original = copy.deepcopy(h.transport_target)
    center = h.transport_scene['attached_item_center_in_tcp_m']
    old_center = np.array(original['pre_place_tcp_xyz_m']) + rotate(
        original['transfer_tcp_quaternion_xyzw'], center)
    assert h._try_transport_alternative('Cartesian segment incomplete')
    assert not h.fault and h.transport_pallet_yaw_flipped
    assert len(h.fk_calls) == 1 and h.fk_calls[0][0] == h.latest_joint_positions
    assert h.transport_route_generation == 1
    assert not h.transport_motion_plan.calls and not h.transport_cartesian.calls
    target = h.transport_target
    assert target['transfer_tcp_quaternion_xyzw'] == [0., 1., 0., 0.]
    assert np.array(target['pre_place_tcp_xyz_m']) + rotate(
        target['transfer_tcp_quaternion_xyzw'], center) == pytest.approx(old_center)
    assert original['pre_place_tcp_xyz_m'] == [.3, -.6, .2]
    assert h._try_transport_alternative('second failure')
    assert h.fault and len(h.fk_calls) == 1


@pytest.mark.parametrize('attribute,value', [
    ('direct_transfer_motion_started', True), ('transfer_goal_handle', object()),
    ('transport_is_pick', True), ('transport_is_return', True),
    ('state', 'TRANSPORT_EXECUTING'), ('planning_scene_status', {}),
])
def test_no_yaw_retry_after_execution_or_without_same_payload(tmp_path, attribute, value):
    h = make_harness(tmp_path)
    setattr(h, attribute, value)
    assert not h._try_pallet_yaw_flip('failure')
    assert not h.fk_calls


def test_yaw_retry_ik_and_goal_choose_bounded_small_absolute_wrist(tmp_path):
    h = make_harness(tmp_path)
    h.transport_pallet_yaw_flipped = True
    h.transport_moveit_planner_id = 'cuMotion'
    h.alternative_seed = (.1, 4.5)
    h.state_validity_client = Client()
    h._alternative_moveit([.3, -.6, .65], h.transport_end_q)
    ik, pending = h.compute_ik_client.calls[-1]
    assert ik.ik_request.robot_state.joint_state.position[-1] == pytest.approx(4.5-math.pi)
    assert ik.ik_request.avoid_collisions
    pending.value = NS(error_code=NS(val=1), solution=NS(joint_state=NS(
        name=['j1', 'j2'], position=[.3, 4.6])))
    pending.callback(pending)
    goal = h.transport_motion_plan.calls[-1][0].motion_plan_request.goal_constraints[0]
    assert goal.joint_constraints[-1].position == pytest.approx(4.6-2*math.pi)
    # A narrower interval can force the larger absolute representative.
    assert nearest_equivalent_joints([4.6], [4.5], ['wrist'],
        {'wrist': (0., 6.)}, minimize_wrist=True) == (4.6,)


def test_flipped_pallet_return_uses_verified_pose_only_for_same_target(tmp_path):
    h = make_harness(tmp_path)
    h.motion_status = copy.deepcopy(h.transport_target)
    assert h._try_pallet_yaw_flip('collision')
    h.verified_pallet_yaw_target = copy.deepcopy(h.transport_target)
    assert h._return_transport_target()['pre_place_tcp_xyz_m'] == h.transport_target['pre_place_tcp_xyz_m']
    h.motion_status['operation_id'] = 9
    assert h._return_transport_target() == h.motion_status
