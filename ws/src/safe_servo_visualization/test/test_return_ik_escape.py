"""Bounded post-release upward recovery; mocked services only."""
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from test_continuous_return import staged_supervisor
from test_continuous_transport import Future


def escape_node():
    h, future, calls = staged_supervisor()
    h.state = h.TRANSPORT_DIAGNOSING
    h.return_to_observation = False
    h.return_goal_joints = None
    h.transport_partial_fraction = 0.
    h.return_escape_count = 0
    h.return_escape_active = False
    h.latest_force_z = .2
    h.last_force_time = time.monotonic()
    h.force_timeout = 1.
    h.place_force_threshold = 4.
    h._link_tcp_xyz = lambda: (*h.robot_tcp_xyz[:2], h.robot_tcp_xyz[2]+h.direct_tcp_z_offset)
    h.get_logger = lambda: NS(info=Mock(), warning=Mock())
    return h, future, calls


def test_recovery_pauses_then_uses_slow_retreat_handoff_and_verifies_before_replan():
    h, future, calls = escape_node()
    assert h._try_return_ik_escape()
    assert h.state == h.RETURN_DISABLING and h.return_escape_active
    assert h.return_clearance_pending and h.continuous_return_restoring
    assert not calls
    future.callback(future)
    assert calls == pytest.approx([.155])
    h.robot_tcp_xyz = (0., 0., .135)
    h._verify_return_escape()
    assert not h.return_escape_active and h.return_escape_count == 1


@pytest.mark.parametrize('guard', ['observation', 'not_return', 'partial', 'goal', 'executing', 'fault'])
def test_only_near_start_vertical_unexecuted_return_can_recover(guard):
    h, future, calls = escape_node()
    if guard == 'observation': h.return_to_observation = True
    if guard == 'not_return': h.transport_is_return = False
    if guard == 'partial': h.transport_partial_fraction = .1
    if guard == 'goal': h.transfer_goal_handle = object()
    if guard == 'executing': h.state = h.TRANSPORT_EXECUTING
    if guard == 'fault': h.state = h.FAULT
    assert not h._try_return_ik_escape()
    assert not calls


def test_observed_085_percent_failure_near_start_enters_bounded_recovery():
    h, future, calls = escape_node()
    h.transport_partial_fraction = 1/117
    h.transport_start_xyz = (0., 0., .15)
    h.transport_partial_probe_xyz = (0., 0., .16)
    h.transport_partial_probe_frame = 'link_base'
    assert h._try_return_ik_escape()
    assert not calls  # pause/mode ownership must still be confirmed
    future.callback(future)
    assert calls == pytest.approx([.155])
    assert h.return_escape_count == 1


@pytest.mark.parametrize('bad', ['nan', 'negative', 'fraction', 'distant', 'lateral', 'downward', 'frame', 'missing'])
def test_near_start_recovery_requires_fraction_and_physical_distance(bad):
    h, future, calls = escape_node()
    h.transport_partial_fraction = 1/117
    h.transport_start_xyz = (0., 0., .15)
    h.transport_partial_probe_xyz = (0., 0., .16)
    h.transport_partial_probe_frame = 'link_base'
    if bad == 'nan': h.transport_partial_fraction = float('nan')
    if bad == 'negative': h.transport_partial_fraction = -.01
    if bad == 'fraction': h.transport_partial_fraction = .03
    if bad == 'distant': h.transport_partial_probe_xyz = (0., 0., .18)
    if bad == 'lateral': h.transport_partial_probe_xyz = (.01, 0., .16)
    if bad == 'downward': h.transport_partial_probe_xyz = (0., 0., .14)
    if bad == 'frame': h.transport_partial_probe_frame = 'world'
    if bad == 'missing': h.transport_partial_probe_xyz = None
    assert not h._try_return_ik_escape()
    assert not calls


@pytest.mark.parametrize('guard', ['attached', 'stale_force', 'stale_robot', 'robot_error',
                                  'changed_target', 'moved_joints', 'ceiling'])
def test_unhealthy_or_changed_prerequisites_never_issue_recovery_motion(guard):
    h, future, calls = escape_node()
    if guard == 'attached': h.planning_scene_status['attached_item_id'] = 'box'
    if guard == 'stale_force': h.last_force_time = 0.
    if guard == 'stale_robot': h.robot_state_time = 0.
    if guard == 'robot_error': h.robot_error = 52
    if guard == 'changed_target': h.motion_status['operation_id'] = 8
    if guard == 'moved_joints': h.latest_joint_positions = (.1, 0.)
    if guard == 'ceiling': h.servo_bounds_mm[5] = 152.
    assert h._try_return_ik_escape()
    assert h.fault and not calls


def test_six_steps_maximum_and_absolute_height_bound():
    h, future, calls = escape_node()
    for i in range(6):
        h.state = h.TRANSPORT_DIAGNOSING
        assert h._try_return_ik_escape()
        future.callback(future)
        assert not h.fault
        h.robot_tcp_xyz = (0., 0., .13+(i+1)*.005)
        h._verify_return_escape()
    h.state = h.TRANSPORT_DIAGNOSING
    assert h._try_return_ik_escape()
    assert 'bounded recovery exhausted' in h.fault
    assert len(calls) == 6 and calls[-1] == pytest.approx(.180)


@pytest.mark.parametrize('guard', ['abort', 'operation', 'moved_tcp', 'force', 'deadline'])
def test_late_pause_callback_cannot_revive_unsafe_recovery(guard):
    h, future, calls = escape_node()
    h._try_return_ik_escape()
    if guard == 'abort': h.state = h.FAULT
    if guard == 'operation': h.operation_id += 1
    if guard == 'moved_tcp': h.robot_tcp_xyz = (.1, 0., .13)
    if guard == 'force': h.latest_force_z += 5.
    if guard == 'deadline': h.return_escape_deadline = 0.
    future.callback(future)
    assert not calls


def test_endpoint_mismatch_blocks_replanning():
    h, future, calls = escape_node()
    h._try_return_ik_escape()
    future.callback(future)
    with pytest.raises(ValueError, match='endpoint mismatch'):
        h._verify_return_escape()  # still at old height, despite service success


@pytest.mark.parametrize('code', [-31, -12, -6])
def test_diagnostic_only_enters_recovery_for_no_ik_solution(code):
    h, future, calls = escape_node()
    h._try_return_ik_escape = Mock(return_value=True)
    h._diagnostic_finish = Mock()
    h._diagnostic_ik_received(Future(NS(error_code=NS(val=code))))
    assert h._try_return_ik_escape.called is (code == -31)
    assert h._diagnostic_finish.called is (code != -31)
