from types import SimpleNamespace as NS
import time
import math

import pytest
from std_srvs.srv import Trigger

from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from safe_servo_visualization.pick_place_pipeline_node import PickPlacePipeline
from safe_servo_visualization.random_stable_loading_node import RandomStableLoadingNode


class Future:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callback = callback


def make_validator():
    node = object.__new__(PickupSupervisor)
    node.state = node.VALIDATING_TRANSFER
    node.operation_id = 3
    node.arm_joint_names = ['joint1', 'joint2']
    node.direct_target_joints = (0.0, 0.0)
    node.direct_target_quaternion = (0.0, 0.0, 0.0, 1.0)
    node.direct_transfer_periodic_joint_names = {'joint1'}
    node.direct_transfer_periodic_limits = (-2 * math.pi, 2 * math.pi)
    node.direct_transfer_ik_timeout = 2.0
    node.planning_group, node.ik_link_name = 'uf850', 'link_tcp'
    node.pre_place_clearance = 0.03
    node.motion_status = {'transfer_context': 'pallet',
                         'transfer_tcp_xyz_m': (0.2, 0.3, 0.5),
                         'pre_place_tcp_z_m': 0.518}
    requests = []
    def call(request):
        requests.append(request)
        return Future(None)
    node.compute_ik_client = NS(service_is_ready=lambda: True, call_async=call)
    errors, executions = [], []
    node._fault = errors.append
    node._send_joint_trajectory_transfer = lambda: executions.append(True)
    node.get_logger = lambda: NS(info=lambda _: None)
    return node, requests, errors, executions


def ik_result(joint1=0.01, code=1):
    return Future(NS(error_code=NS(val=code), solution=NS(
        joint_state=NS(name=['joint2', 'joint1'], position=[0.0, joint1]))))


@pytest.mark.parametrize('code,jump,rejected', [
    (-31, 0.01, True), (1, 0.4, True), (1, 0.02, False),
])
def test_descent_validation_blocks_transfer_until_complete(code, jump, rejected):
    node, requests, errors, executions = make_validator()
    node._check_loading_path_before_transfer()
    node._loading_path_checked(ik_result(jump, code), 3, 0)
    assert bool(errors) is rejected
    assert not executions  # More samples remain even after one successful IK.
    if rejected:
        assert errors[0].startswith('KINEMATIC_REJECTED:')
        assert len(requests) == 1
    else:
        assert list(requests[1].ik_request.robot_state.joint_state.position) == [jump, 0.0]


def test_explicit_five_mm_steps_preserve_orientation_and_reach_endpoint():
    node, requests, errors, executions = make_validator()
    node._check_loading_path_before_transfer()
    for index in range(3):
        node._loading_path_checked(ik_result(0.01 * (index + 1)), 3, index)
    assert not errors
    assert executions == [True]
    assert [r.ik_request.pose_stamped.pose.position.z for r in requests] == pytest.approx(
        [0.495, 0.490, 0.488])
    assert all(r.ik_request.pose_stamped.pose.orientation.w == 1.0 for r in requests)
    assert all(r.ik_request.pose_stamped.pose.position.x == 0.2 for r in requests)


def test_periodic_wrap_is_normalized_before_continuity_check():
    node, requests, errors, executions = make_validator()
    node.direct_target_joints = (math.radians(179), 0.0)
    node._check_loading_path_before_transfer()
    node._loading_path_checked(ik_result(math.radians(-179)), 3, 0)
    assert not errors and not executions
    assert requests[1].ik_request.robot_state.joint_state.position[0] == pytest.approx(
        math.radians(181))


def test_infrastructure_error_is_not_a_candidate_rejection():
    node, _, errors, executions = make_validator()
    node._check_loading_path_before_transfer()
    node._loading_path_checked(ik_result(code=-6), 3, 0)
    assert errors and not errors[0].startswith('KINEMATIC_REJECTED:')
    assert not executions


def test_failed_later_step_and_duplicate_response_cannot_execute_transfer():
    node, requests, errors, executions = make_validator()
    node._check_loading_path_before_transfer()
    node._loading_path_checked(ik_result(), 3, 0)
    node._loading_path_checked(ik_result(), 3, 0)
    assert len(requests) == 2
    node._loading_path_checked(ik_result(code=-31), 3, 1)
    assert 'step 2/3' in errors[0]
    assert len(requests) == 2 and not executions


def test_stale_descent_response_cannot_execute_new_operation():
    node = object.__new__(PickupSupervisor)
    node.state = node.CHECKING_LOADING_PATH
    node.operation_id = 4
    node._loading_path_checked(Future(None), 3, 0)


def test_controller_fault_is_not_a_reselection_request():
    node = object.__new__(PickPlacePipeline)
    node.state, node.fault = 'FAULT', 'xArm error 21 during linear loading'
    response = node.retry_place(None, Trigger.Response())
    assert not response.success


def test_loading_controller_error_faults_before_timeout_release():
    node = object.__new__(PickupSupervisor)
    node.state = node.RETREATING
    node.pre_descent_wait_callback = node.mode_wait_target = None
    node.retreat_started = time.monotonic() - 30.0
    node.operation_kind = 'loading'
    node.loading_contact_fallback = False
    node.robot_error = 21
    errors = []
    node._fault = errors.append
    node.retreat_tick()
    assert errors == ['xArm error 21 during linear loading']


def test_place_retry_does_not_pick_again():
    node = object.__new__(PickPlacePipeline)
    node.state, node.fault = 'FAULT', 'KINEMATIC_REJECTED: partial descent'
    node.operation_id = 8
    node.place_status = {'operation_id': 12}
    calls = []
    node.start_place = NS(service_is_ready=lambda: True, call_async=lambda _: NS(
        add_done_callback=lambda cb: calls.append(cb)))
    node.publish_status = lambda: None
    response = node.retry_place(None, Trigger.Response())
    assert response.success
    assert node.state == 'PLACING'
    assert node.operation_id == 9
    assert node.expected_place_id == 13
    assert len(calls) == 1


def test_reselection_discards_only_pending_and_preserves_item_dimensions():
    node = object.__new__(RandomStableLoadingNode)
    key = (40.0, 0.0, 265.0, False)
    pending = NS(sequence_id=9, item_id=0, placement=object(),
                 raw_dim=NS(raw=lambda: (130, 170, 205)))
    discarded, tasks = [], []
    node.loader = NS(pending=pending, placement_key=lambda _: key,
                     discard_pending=discarded.append)
    node.rejected_loading_poses = set()
    node.planning_worker = NS(submit=lambda fn, **kw: tasks.append(kw))
    node.get_logger = lambda: NS(warning=lambda _: None)
    node._push_visualization = lambda _: None
    node.publish_status = lambda: None
    node._reselect_loading_pose('KINEMATIC_REJECTED: partial descent')
    assert node.rejected_loading_poses == {key}
    assert discarded == [9]
    assert tasks == [{'item_id': 0, 'dimensions_mm': (130, 170, 205)}]
    assert node.retrying_carried_item and node.cycle_auto_start
    assert node.state == 'PLANNING'
