from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
from test_continuous_transport import planned_harness, Future


def diagnostic_harness():
    from moveit_msgs.srv import GetCartesianPath
    h, result = planned_harness()
    h.operation_id = 7
    h.transport_target = {'operation_id': 9}
    h.planning_group = 'uf850'
    h.get_logger = lambda: NS(warning=lambda *args: None)
    req = GetCartesianPath.Request()
    req.header.frame_id = 'link_base'
    req.group_name, req.link_name = 'uf850', 'link_tcp'
    req.waypoints = [Pose() for _ in range(10)]
    for i, p in enumerate(req.waypoints):
        p.position.x = i / 10
        p.orientation.w = 1.
    h.transport_cartesian_request = req
    ik_calls, validity_calls = [], []
    def record(calls, req):
        future = Future(None)
        calls.append((req, future))
        return future
    h.compute_ik_client = NS(service_is_ready=lambda: True,
                            call_async=lambda req: record(ik_calls, req))
    h.state_validity_client = NS(service_is_ready=lambda: True,
                                call_async=lambda req: record(validity_calls, req))
    result.fraction = .609
    h._transport_planned(Future(result))
    return h, ik_calls, validity_calls


def ik_success():
    result = GetPositionIK.Response()
    result.error_code.val = 1
    result.solution.joint_state.name = ['joint1', 'joint2']
    result.solution.joint_state.position = [.1, .2]
    return Future(result)


def test_partial_path_probes_nearby_waypoint_without_motion():
    h, calls, _ = diagnostic_harness()
    assert h.state == h.TRANSPORT_DIAGNOSING
    req = calls[0][0].ik_request
    assert not req.avoid_collisions
    assert req.pose_stamped.pose.position.x == pytest.approx(.6)
    assert list(req.robot_state.joint_state.position) == [.2, .1]
    assert req.timeout.nanosec == 200000000


def test_failed_ik_does_not_claim_collision_or_absolute_unreachability():
    h, _, validity = diagnostic_harness()
    h._diagnostic_ik_received(Future(NS(error_code=NS(val=-31))))
    assert h.state == h.FAULT and not validity
    assert 'cause unresolved' in h.fault
    assert 'KINEMATIC_REJECTED' in h.fault


@pytest.mark.parametrize('valid', [True, False])
def test_collision_probe_reports_evidence_but_never_accepts_partial_path(valid):
    h, _, calls = diagnostic_harness()
    h._diagnostic_ik_received(ik_success())
    assert len(calls) == 1
    assert calls[0][0].robot_state.is_diff
    h._diagnostic_validity_received(Future(NS(valid=valid, contacts=[
        NS(contact_body_1='link5', contact_body_2='table')])))
    assert h.state == h.FAULT
    assert ('does not validate the full path' if valid else 'link5/table') in h.fault


def test_diagnostic_out_of_bounds_stops_before_collision_query():
    h, _, calls = diagnostic_harness()
    response = ik_success()
    response.value.solution.joint_state.position = [3., .2]
    h._diagnostic_ik_received(response)
    assert 'outside bounds' in h.fault and not calls


def test_diagnostic_timeout_and_late_callback(monkeypatch):
    h, calls, validity = diagnostic_harness()
    h.robot_error = 0
    monkeypatch.setattr('safe_servo_visualization.continuous_transport.time.monotonic',
                        lambda: h.transport_diagnostic_deadline + .1)
    assert h._transport_tick()
    assert 'timed out' in h.fault
    calls[0][1].callback(ik_success())
    assert not validity


def test_diagnostic_callback_ignored_after_operation_changes():
    h, calls, validity = diagnostic_harness()
    h.operation_id += 1
    calls[0][1].callback(ik_success())
    assert not validity
