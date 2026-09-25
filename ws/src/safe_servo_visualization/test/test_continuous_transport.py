"""No hardware: geometry, timed paths, and release interlocks."""
from types import SimpleNamespace as NS

import numpy as np
import pytest
from action_msgs.msg import GoalStatus
from moveit_msgs.srv import GetCartesianPath
from trajectory_msgs.msg import JointTrajectoryPoint

from safe_servo_visualization.continuous_transport import (
    ContinuousTransport, item_bottom_offset, rotate, transport_waypoints)
from safe_servo_visualization.pickup_supervisor_node import PickupSupervisor
from safe_servo_visualization.pick_place_pipeline_node import PickPlacePipeline
from safe_servo_visualization.place_pipeline_node import PlacePipeline


SCENE = dict(attached_item_size_m=[.2,.15,.1],
             attached_item_center_in_tcp_m=[0,0,.05],
             attached_item_orientation_in_tcp_xyzw=[0,0,0,1])
DOWN = [1,0,0,0]


class Future:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callback = callback


class Harness(ContinuousTransport):
    FAULT = 'FAULT'
    SUCCEEDED = 'SUCCEEDED'

    def _transport_planned(self, future):
        # Numerical tests wait explicitly; production callbacks never wait.
        super()._transport_planned(future)
        pending = getattr(self, '_retime_pending', None)
        if pending is not None:
            try:
                pending[0].result(timeout=30)
            except Exception:
                pass  # The main-thread poll must report worker errors.
            self._poll_transport_timing()
        pool = getattr(self, '_retime_pool', None)
        if pool is not None:
            pool.shutdown(wait=True)
            self._retime_pool = None

    def __init__(self):
        self.operation_id = 'test-operation'
        self.state = self.TRANSPORT_PLANNING
        self.fault = ''
        self.released = False

    def _fault(self, reason):
        self.fault, self.state = reason, self.FAULT

    def _turn_vacuum_off(self):
        self.released = True
        self.state = 'VACUUM_OFF'

    def get_logger(self):
        return NS(info=lambda *a: None, error=lambda *a: None)

    def publish_status(self):
        pass


def verifying_harness(monkeypatch):
    clock = [10.]
    monkeypatch.setattr('safe_servo_visualization.continuous_transport.time.monotonic', lambda: clock[0])
    h = Harness()
    h.state = h.TRANSPORT_EXECUTING
    h.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10000000000))
    h.status_timeout = 1.
    h.transport_end = np.array([.2,.3,.4])
    h.direct_transfer_succeeded = False
    h.latest_joint_positions = (0.,0.)
    h._link_tcp_xyz = lambda: pytest.fail('cached Servo position must not be used')
    calls = []
    h._transport_fk_request = lambda joints,callback: calls.append((joints,callback))
    h._transport_result(Future(NS(status=GoalStatus.STATUS_SUCCEEDED,
                                  result=NS(error_code=0))))
    return h,clock,calls


def test_completed_action_waits_for_post_completion_acquisition(monkeypatch):
    h,clock,calls = verifying_harness(monkeypatch)
    assert h.state == h.TRANSPORT_VERIFYING and not h.direct_transfer_succeeded
    clock[0] = 10.2
    h.transport_joint_sample_time = 10.1
    h.transport_joint_stamp_ns = 9900000000  # queued older message received later
    h._transport_verify_tick(clock[0])
    assert not calls
    h.transport_joint_stamp_ns = 10100000000
    h._transport_verify_tick(clock[0])
    assert len(calls) == 1
    h._transport_verify_tick(clock[0])
    assert len(calls) == 1  # only one outstanding query
    h._transport_pose = lambda result: (h.transport_end.copy(), DOWN)
    calls[0][1](Future(None))
    assert h.state == h.SUCCEEDED and h.direct_transfer_succeeded


def test_final_pose_mismatch_can_settle_but_requires_new_sample(monkeypatch):
    h,clock,calls = verifying_harness(monkeypatch)
    clock[0] = h.transport_joint_sample_time = 10.1
    h.transport_joint_stamp_ns = 10100000000
    h._transport_verify_tick(clock[0])
    h._transport_pose = lambda result: (h.transport_end + [.02,0,0], DOWN)
    calls[-1][1](Future(None))
    assert h.state == h.TRANSPORT_VERIFYING and not h.direct_transfer_succeeded
    h._transport_verify_tick(clock[0])
    assert len(calls) == 1
    clock[0] = h.transport_joint_sample_time = 10.3
    h.transport_joint_stamp_ns = 10300000000
    h._transport_verify_tick(clock[0])
    h._transport_pose = lambda result: (h.transport_end + [.005,0,0], DOWN)
    calls[-1][1](Future(None))
    assert h.state == h.SUCCEEDED


@pytest.mark.parametrize('error', [None,.025])
def test_final_verification_timeout_holds_item(monkeypatch,error):
    h,clock,_ = verifying_harness(monkeypatch)
    h.transport_verify_error = error
    clock[0] = 15.1
    h._transport_verify_tick(clock[0])
    assert h.state == h.FAULT and not h.released
    assert not h.direct_transfer_succeeded
    assert 'timed out' in h.fault


@pytest.mark.parametrize('late', ['stale_sample','deadline','aborted'])
def test_late_final_fk_cannot_authorize_motion(monkeypatch,late):
    h,clock,calls = verifying_harness(monkeypatch)
    clock[0] = h.transport_joint_sample_time = 10.1
    h.transport_joint_stamp_ns = 10100000000
    h._transport_verify_tick(clock[0])
    h._transport_pose = lambda result: (h.transport_end.copy(), DOWN)
    if late == 'aborted':
        h.state = h.FAULT
    else:
        clock[0] = 15.1 if late == 'deadline' else 11.2
    calls[-1][1](Future(None))
    assert not h.direct_transfer_succeeded and not h.released


def test_rounded_geometry_keeps_item_above_clearance_during_lateral_motion():
    start, end = np.array([.2,.3,.1]), np.array([-.3,-.4,.25])
    samples, safe, high = transport_waypoints(start, DOWN, end, [0,1,0,0], .47, SCENE)
    assert high > safe >= .57
    assert np.allclose(samples[-1][0], end)
    for p,q in samples:
        assert np.allclose(rotate(q,[0,0,1]), [0,0,-1])
        if min(np.linalg.norm(p[:2]-start[:2]), np.linalg.norm(p[:2]-end[:2])) > 1e-6:
            assert p[2]+item_bottom_offset(q,SCENE) >= .47
        if p[2] < safe-1e-8:
            assert np.allclose(p[:2], start[:2]) or np.allclose(p[:2],end[:2])
    differences = np.diff([start]+[p for p,q in samples], axis=0)
    nonzero = differences[np.linalg.norm(differences,axis=1)>1e-8]
    directions = nonzero/np.linalg.norm(nonzero,axis=1)[:,None]
    # No right-angle corners in the Cartesian path.
    assert np.min(np.sum(directions[1:]*directions[:-1],axis=1)) > .94


@pytest.mark.parametrize('end,orientation', [([.2,.3,.4],DOWN), ([.4,.5,.4],[0,0,0,1])])
def test_invalid_transport_rejected(end, orientation):
    with pytest.raises(ValueError):
        transport_waypoints([.2,.3,.1],DOWN,end,orientation,.47,SCENE)


def test_blend_radius_is_limited_for_short_transfer():
    samples,safe,high = transport_waypoints([0,0,.1],DOWN,[.02,0,.2],DOWN,.47,SCENE)
    assert high-safe == pytest.approx(.005)
    assert all(-1e-9 <= p[0] <= .02+1e-9 for p,q in samples)


def planned_harness():
    h = Harness()
    h.arm_joint_names = ['joint1','joint2']
    h.transport_seed = (0.,0.)
    h.direct_transfer_periodic_limits = (-6.28,6.28)
    h.direct_transfer_periodic_joint_names = []
    h.transport_joint_limits = {j:(-2.,2.,2.) for j in h.arm_joint_names}
    h.motion_speed_percent = 50.
    h.direct_transfer_max_joint_speed = 2.
    h.direct_transfer_joint_acc = 3.
    h.transport_max_joint_jerk = 10.
    h.transport_enforce_jerk_limit = True
    h.transport_clearance = .47
    h._transport_validate_next = lambda: None
    result = GetCartesianPath.Response()
    result.error_code.val = 1
    result.fraction = 1.
    result.solution.joint_trajectory.joint_names = ['joint2','joint1']
    for positions,t in [([0.,0.],0),([.2,.1],1)]:
        point = JointTrajectoryPoint()
        point.positions,point.velocities = positions,[0.,0.]
        point.time_from_start.sec = t
        result.solution.joint_trajectory.points.append(point)
    return h,result


def test_timed_trajectory_checks_controller_quintic_and_reorders_joints():
    h,result = planned_harness()
    h._transport_planned(Future(result))
    assert h.state == h.TRANSPORT_VALIDATING
    assert list(h.transport_trajectory.points[-1].positions) == [.1,.2]
    assert len(h.transport_checks) >= 51
    midpoint = min(h.transport_checks, key=lambda row: abs(row[1]-h.transport_duration/2))
    assert np.allclose(midpoint[0], [.05,.1], atol=.003)
    assert all(list(p.accelerations) == [0., 0.] for p in h.transport_trajectory.points)


def test_partial_path_never_executes():
    h,result = planned_harness()
    result.fraction = .99
    h._transport_planned(Future(result))
    assert h.fault.startswith('KINEMATIC_REJECTED:')
    assert not hasattr(h,'transport_trajectory')


def test_timing_is_stretched_to_respect_limits():
    h,result = planned_harness()
    h.direct_transfer_max_joint_speed = .01
    h._transport_planned(Future(result))
    assert h.transport_duration >= 59.9
    assert h.transport_checks[-1][1] == h.transport_duration


def test_cartesian_relative_ratio_slows_only_cartesian_timing():
    cartesian, cartesian_result = planned_harness()
    cartesian.motion_speed_percent = 100.
    cartesian.cartesian_transport_speed_ratio = .1
    cartesian.transport_timing_source = 'cartesian'
    cartesian._transport_planned(Future(cartesian_result))

    moveit, moveit_result = planned_harness()
    moveit.motion_speed_percent = 100.
    moveit.cartesian_transport_speed_ratio = .1
    moveit.transport_timing_source = 'moveit'
    moveit._transport_planned(Future(moveit_result))

    assert cartesian.transport_duration > moveit.transport_duration
    assert moveit.transport_duration < 1.2  # Only its normal spline-limit repair.


def test_position_limit_rejection():
    h,result = planned_harness()
    h.transport_joint_limits['joint2'] = (-.1,.15,2.)
    h._transport_planned(Future(result))
    assert 'position limits' in h.fault
    assert h.fault.startswith('KINEMATIC_REJECTED:')


def test_large_start_relative_excursion_still_requires_collision_validation():
    h = Harness()
    h.operation_id = 1
    h.state = h.TRANSPORT_VALIDATING
    h.arm_joint_names = ['joint1','joint2']
    h.planning_group = 'uf850'
    h.transport_seed = (0.,0.)
    h.direct_transfer_max_joint_delta = np.pi
    h.transport_checks = [((3.5,0.),1.)]
    h.transport_check_index = 0
    calls = []
    h.state_validity_client = NS(call_async=lambda req: calls.append(req) or Future(None))
    h._transport_validate_next()
    assert not h.fault and len(calls) == 1
    assert list(calls[0].robot_state.joint_state.position) == [3.5,0.]
    assert h.state == h.TRANSPORT_VALIDATING
    # Removing the excursion cap must not bypass a collision rejection.
    h._transport_collision_checked(Future(NS(valid=False)))
    assert h.state == h.FAULT and 'collision' in h.fault


def test_nonzero_endpoint_velocities_are_normalized_before_all_checks():
    reference, reference_result = planned_harness()
    reference.direct_transfer_max_joint_speed = .01
    reference._transport_planned(Future(reference_result))
    h,result = planned_harness()
    h.direct_transfer_max_joint_speed = .01
    points = result.solution.joint_trajectory.points
    points[0].velocities = [.12, -.08]
    points[-1].velocities = [-.04, .06]
    for point in points:
        point.accelerations = [10., -10.]
    calls = []
    h._transport_validate_next = lambda: calls.append('validate')
    h._transport_planned(Future(result))
    assert h.state == h.TRANSPORT_VALIDATING and calls == ['validate']
    assert list(points[0].velocities) == [0.,0.]
    assert list(points[-1].velocities) == [0.,0.]
    assert all(list(p.accelerations) == [0., 0.] for p in h.transport_trajectory.points)
    # Both the checked spline and its stretched timing must reflect the
    # corrected endpoints, not the original service response derivatives.
    assert h.transport_checks == reference.transport_checks
    assert h.transport_duration == reference.transport_duration


@pytest.mark.parametrize('invalid', ['timing', 'velocity'])
def test_timing_failures_do_not_trigger_kinematic_resampling(invalid):
    h,result = planned_harness()
    points = result.solution.joint_trajectory.points
    if invalid == 'timing':
        points[-1].time_from_start.sec = 0
    elif invalid == 'velocity':
        points[0].velocities = [float('nan'),0.]
    h._transport_planned(Future(result))
    assert h.state == h.FAULT
    assert not h.fault.startswith('KINEMATIC_REJECTED:')
    assert not hasattr(h,'transport_trajectory')


def stopping_harness(monkeypatch):
    clock = [1.]
    monkeypatch.setattr('safe_servo_visualization.continuous_transport.time.monotonic', lambda: clock[0])
    h = Harness()
    h.state = h.TRANSPORT_STOPPING
    h.robot_error = 0
    h.status_timeout = h.force_timeout = 10.
    h.last_force_time = h.last_joint_state_time = 1.
    h.robot_state_time = 1.
    h.robot_mode, h.robot_state = 1,0
    h.transport_stop_started = 1.
    h.transport_stop_stamp = 0.
    h.transport_stop_joints = h.latest_joint_positions = (0.,0.)
    h.transport_still_since = None
    h.transport_terminal = h.transport_cancel_confirmed = True
    h.transport_target = {'transfer_tcp_z_m':.6}
    return h,clock


def test_release_requires_terminal_cancellation_and_fresh_stable_joints(monkeypatch):
    h,clock = stopping_harness(monkeypatch)
    h.transport_terminal = False
    h._transport_tick()
    assert not h.released
    h.transport_terminal = True
    h._transport_tick()
    clock[0] = 1.3
    h._transport_tick()
    assert not h.released  # stale (unchanged timestamp) is not stationary feedback
    h.last_joint_state_time = 1.3
    h._transport_tick()
    assert h.released and h.continuous_contact_retreat


def test_slow_drift_does_not_count_as_stopped(monkeypatch):
    h,clock = stopping_harness(monkeypatch)
    h._transport_tick()
    for n in range(1,5):
        clock[0] = h.last_joint_state_time = 1.+n*.1
        h.latest_joint_positions = (n*.0006,0.)
        h._transport_tick()
    assert not h.released


def test_cancel_rejected_or_stop_timeout_never_releases(monkeypatch):
    h,clock = stopping_harness(monkeypatch)
    h._transport_cancelled(Future(NS(goals_canceling=[])))
    assert h.state == h.FAULT and not h.released
    h.state = h.TRANSPORT_STOPPING
    clock[0] = 6.1
    h._transport_tick()
    assert h.state == h.FAULT and not h.released


def test_delayed_accept_after_abort_is_canceled():
    h = Harness()
    h.operation_id = 3
    h.state = 'FAULT'
    canceled = []
    handle = NS(accepted=True,cancel_goal_async=lambda: canceled.append(True))
    h._transport_goal_received(Future(handle),3)
    assert canceled and not h.released


def test_abort_result_does_not_authorize_release(monkeypatch):
    h,_ = stopping_harness(monkeypatch)
    h._transport_result(Future(NS(status=GoalStatus.STATUS_ABORTED)))
    assert h.state == h.FAULT and not h.released


def test_grasp_hold_does_not_retreat_but_normal_pick_still_does():
    h = object.__new__(PickupSupervisor)
    h.operation_kind = 'pickup'
    h.defer_pickup_lift = True
    h.publish_status = lambda: None
    calls = []
    h._disable_servo_then_direct_retreat = lambda: calls.append('retreat')
    h._proceed_to_retreat()
    assert h.state == h.SUCCEEDED and not calls
    h._proceed_to_retreat()
    assert calls == ['retreat']


def test_combined_continuous_handoff_uses_continuous_place(monkeypatch):
    h = object.__new__(PickPlacePipeline)
    h.state = 'PICKING'
    h.started = 1.
    h.timeout = 300.
    monkeypatch.setattr('safe_servo_visualization.pick_place_pipeline_node.time.monotonic',lambda: 2.)
    h.pick_only = False
    h.use_continuous = True
    h.expected_pickup_id = 4
    h.pickup_status = {'operation_id':4,'state':'SUCCEEDED'}
    h.place_status = {}
    calls = []
    h.start_continuous_place = NS(service_is_ready=lambda: True,
        call_async=lambda req: calls.append(req) or Future(None))
    h.tick()
    assert h.state == 'PLACING' and len(calls) == 1


def test_supervisor_constructs_and_registers_transport_states():
    import rclpy
    rclpy.init()
    node = None
    try:
        node = PickupSupervisor()
        assert ContinuousTransport.TRANSPORT_STATES <= node.ACTIVE
        assert node.transport_radius == .04
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_wait_for_attachment_before_preparing_continuous_place():
    from std_srvs.srv import Trigger
    h = object.__new__(PlacePipeline)
    h.state = h.IDLE
    h.scene_status = {}
    h.operation_id = 0
    h.publish_status = lambda: None
    response = h.start_continuous(None,Trigger.Response())
    assert response.success and h.state == h.WAIT_ATTACHMENT
    calls = []
    h.scene_status = {'attached_item_id':'carried_item_1'}
    h._start = lambda response,continuous: calls.append(continuous) or NS(success=True)
    h.tick()
    assert calls == [True]


def test_force_contact_cancels_but_does_not_release():
    h = Harness()
    h.state = h.TRANSPORT_EXECUTING
    h.operation_id = 1
    h.transport_descent_time = 3.
    h.transport_feedback_time = 2.
    h.transport_force_count = 0
    h.place_force_threshold = 4.
    h.loading_contact_confirm_samples = 2
    h.latest_joint_positions = (0.,0.)
    h.last_joint_state_time = 1.
    calls = []
    h.transfer_goal_handle = NS(cancel_goal_async=lambda: calls.append('cancel') or Future(None))
    h._transport_force(10.)  # carried load is the baseline, not contact
    h.transport_feedback_time = 3.1
    h._transport_force(15.)
    assert not calls
    h._transport_force(15.)
    assert calls == ['cancel'] and h.state == h.TRANSPORT_STOPPING
    assert not h.released


def test_cartesian_request_contains_entire_path_and_requires_collision_checks():
    h = Harness()
    h.operation_id = 1
    h.transport_target = {'pre_place_tcp_xyz_m':[.4,.5,.2],
        'transfer_tcp_quaternion_xyzw':DOWN,'transport_corner_clearance_z_m':.47}
    h.transport_scene = SCENE
    h.transport_radius = .04
    h.servo_bounds_mm = [-1000,1000,-1000,1000,-100,1000]
    h.planning_group = 'uf850'
    h.ik_link_name = 'link_tcp'
    h.arm_joint_names = ['joint1','joint2']
    h.transport_seed = (0.,0.)
    h.motion_speed_percent = 30.
    h._transport_pose = lambda r: (np.array([0.,0.,.1]),np.array(DOWN))
    calls = []
    h.transport_cartesian = NS(call_async=lambda req: calls.append(req) or Future(None))
    h._transport_start_fk(Future(None))
    assert not h.fault
    assert len(calls) == 1 and len(calls[0].waypoints) > 100
    assert calls[0].avoid_collisions and calls[0].max_velocity_scaling_factor == .3
    assert calls[0].waypoints[-1].position.z == .2
