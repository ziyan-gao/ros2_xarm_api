"""Bounded, non-executing probes for a partial Cartesian path.

The service fraction identifies a requested waypoint interval, not the exact
failed internal interpolation sample. Probe results are evidence, not a claim
that a single endpoint explains the whole path or that an IK failure proves
unreachability.
"""
import copy
import math
import time

from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.duration import Duration


class CartesianFailureDiagnostics:
    TRANSPORT_DIAGNOSING = 'TRANSPORT_DIAGNOSING'

    def _diagnostic_finish(self, detail):
        reason = f'{self.transport_partial_reason}; diagnostic: {detail}'
        if getattr(self, 'alternative_diagnostic_pending', False):
            self.alternative_diagnostic_pending = False
            self._alternative_failed(reason)
            return
        raised_pick = getattr(self, '_try_raised_pick_path', None)
        if raised_pick is not None and raised_pick(reason):
            return
        fallback = getattr(self, '_try_transport_alternative', None)
        if fallback is None or not fallback(reason):
            self._fault(reason)

    def _diagnose_partial_path(self, result, alternative=False):
        self.alternative_diagnostic_pending = alternative
        self.transport_partial_fraction = result.fraction
        self.transport_partial_probe_xyz = None
        self.transport_partial_probe_frame = None
        self.transport_partial_reason = (
            f'KINEMATIC_REJECTED: continuous transport path reaches {result.fraction:.1%}')
        if alternative:
            self.transport_partial_reason = (
                f'KINEMATIC_REJECTED: {self.alternative_segment_label} '
                f'reaches {result.fraction:.1%}')
        request = getattr(self, 'transport_cartesian_request', None)
        if request is None or not request.waypoints:
            self._diagnostic_finish('request snapshot unavailable')
            return
        self.state = self.TRANSPORT_DIAGNOSING
        self.transport_diagnostic_deadline = time.monotonic() + 2.0
        try:
            if not self.compute_ik_client.service_is_ready():
                self._diagnostic_finish('IK service unavailable')
                return
            index = min(len(request.waypoints)-1,
                        max(0, int(result.fraction * len(request.waypoints))))
            pose = request.waypoints[index]
            xyz = pose.position
            self.transport_partial_probe_xyz = (xyz.x, xyz.y, xyz.z)
            self.transport_partial_probe_frame = request.header.frame_id
            direction = ('reverse return' if getattr(self, 'return_goal_joints', None) is not None
                         else 'vertical return') if getattr(self, 'transport_is_return', False) else 'outbound'
            self.get_logger().warning(
                f'partial Cartesian {direction}: target={self.transport_target.get("operation_id")}, '
                f'probing waypoint {index+1}/{len(request.waypoints)} near failed interval '
                f'xyz=({xyz.x:.4f},{xyz.y:.4f},{xyz.z:.4f}) in {request.header.frame_id}; '
                'exact failed interpolation sample is not reported by MoveIt')
            probe = GetPositionIK.Request()
            ik = probe.ik_request
            ik.group_name = request.group_name
            ik.ik_link_name = request.link_name
            ik.pose_stamped.header = copy.deepcopy(request.header)
            ik.pose_stamped.pose = copy.deepcopy(pose)
            ik.robot_state = copy.deepcopy(request.start_state)
            partial = result.solution.joint_trajectory
            if partial.points:
                ik.robot_state.joint_state.name = list(partial.joint_names)
                ik.robot_state.joint_state.position = list(partial.points[-1].positions)
            ik.robot_state.is_diff = True
            ik.avoid_collisions = False  # Diagnostic only; never executed.
            ik.timeout = Duration(seconds=0.2).to_msg()
            self.compute_ik_client.call_async(probe).add_done_callback(
                self._transport_guard(self._diagnostic_ik_received))
        except Exception as exc:
            self._diagnostic_finish(f'probe unavailable: {exc}')

    def _diagnostic_ik_received(self, future):
        try:
            result = future.result()
            if result is None or result.error_code.val != 1:
                code = None if result is None else result.error_code.val
                escape = getattr(self, '_try_return_ik_escape', None)
                if code == -31 and escape is not None and escape():
                    return
                self._diagnostic_finish(
                    f'IK probe returned code={code}; cause unresolved '
                    '(IK seed/timeout, reachability or limits); no collision diagnosis')
                return
            joints = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
            for name, (lower, upper, _) in self.transport_joint_limits.items():
                value = joints.get(name)
                if value is None or not math.isfinite(value) or not lower <= value <= upper:
                    self._diagnostic_finish(f'IK probe invalid or outside bounds: {name}={value}')
                    return
            if not self.state_validity_client.service_is_ready():
                self._diagnostic_finish('IK succeeded; state validity service unavailable')
                return
            req = GetStateValidity.Request()
            req.robot_state = result.solution
            req.robot_state.is_diff = True
            req.group_name = self.planning_group
            self.state_validity_client.call_async(req).add_done_callback(
                self._transport_guard(self._diagnostic_validity_received))
        except Exception as exc:
            self._diagnostic_finish(f'IK probe exception: {exc}')

    def _diagnostic_validity_received(self, future):
        try:
            result = future.result()
            if result is None:
                self._diagnostic_finish('state validity response missing')
            elif result.valid:
                self._diagnostic_finish(
                    'nearby waypoint IK is valid in current scene; Cartesian continuity/internal '
                    'sample remains unresolved; this does not validate the full path')
            else:
                pairs = sorted({f'{c.contact_body_1}/{c.contact_body_2}' for c in result.contacts})
                self._diagnostic_finish(
                    'IK probe state invalid; collision pairs=' + (', '.join(pairs[:8]) or 'not reported'))
        except Exception as exc:
            self._diagnostic_finish(f'state validity probe exception: {exc}')
