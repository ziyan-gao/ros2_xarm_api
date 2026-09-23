"""Continuous pallet transport; planning services never command the robot."""
import math
import copy
from concurrent.futures import ThreadPoolExecutor
import time
import xml.etree.ElementTree as ET

import numpy as np
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetStateValidity
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .continuous_return import ContinuousReturn
from .continuous_pick import ContinuousPick
from .cartesian_failure_diagnostics import CartesianFailureDiagnostics
from .smooth_transport import smooth_and_sample
from .transport_alternatives import TransportAlternatives
from .sdk_transport import SdkTransport
from .transport_path import pickup_needs_observation, grid_pick_waypoints
from .transport_alternatives import waypoint_candidates
from .transport_reuse import TransportReuse
from .staged_transport_timing import StagedTransportTiming


def retime_snapshot(trajectory, limits, speed, acceleration, jerk, scale):
    """Worker owns only copies; never touches a ROS node or robot state."""
    timing = {}
    checks, duration, ratio = smooth_and_sample(
        trajectory, limits, speed, acceleration, jerk, scale, timing)
    return trajectory, checks, duration, ratio, timing


def cartesian_timing_issue(trajectory, required_joints):
    """Reject malformed geometry; distinguish an untimed Cartesian solution.

    A full Cartesian fraction can accompany failed MoveIt time parameterization.
    Empty velocities/non-increasing timestamps must not be indexed or executed.
    Inspect *all* points first so missing timing cannot hide malformed geometry.
    """
    names = list(trajectory.joint_names)
    if (len(names) != len(required_joints) or len(set(names)) != len(names) or
            set(names) != set(required_joints)):
        raise ValueError('Cartesian trajectory joint names do not match the arm')
    if len(trajectory.points) < 2:
        raise ValueError('empty transport trajectory')
    problem = None
    previous_ns = None
    for index, point in enumerate(trajectory.points):
        if len(point.positions) != len(names) or not all(math.isfinite(q) for q in point.positions):
            raise ValueError(f'Cartesian point {index} has invalid joint positions')
        for label, values in (('velocities', point.velocities), ('accelerations', point.accelerations)):
            if values and (len(values) != len(names) or not all(math.isfinite(v) for v in values)):
                raise ValueError(f'Cartesian point {index} has invalid {label}')
        if not point.velocities and problem is None:
            problem = f'point {index} has no joint velocities'
        duration = point.time_from_start
        if duration.sec < 0 or not 0 <= duration.nanosec < 1000000000:
            raise ValueError(f'Cartesian point {index} has a malformed timestamp')
        stamp_ns = duration.sec*1000000000 + duration.nanosec
        if ((index == 0 and stamp_ns != 0) or
                (previous_ns is not None and stamp_ns <= previous_ns)) and problem is None:
            problem = f'point {index} has invalid trajectory timing'
        previous_ns = stamp_ns
    return problem


from .transport_path import (
    quaternion, rotate, slerp, item_bottom_offset, transport_waypoints, vertical_retreat_waypoints,
    pick_waypoints,
)


class ContinuousTransport(StagedTransportTiming, TransportReuse, SdkTransport, TransportAlternatives, ContinuousPick, CartesianFailureDiagnostics, ContinuousReturn):
    TRANSPORT_PLANNING = 'TRANSPORT_PLANNING'
    TRANSPORT_VALIDATING = 'TRANSPORT_VALIDATING'
    TRANSPORT_EXECUTING = 'TRANSPORT_EXECUTING'
    TRANSPORT_STOPPING = 'TRANSPORT_STOPPING'
    TRANSPORT_VERIFYING = 'TRANSPORT_VERIFYING'
    TRANSPORT_STATES = {TRANSPORT_PLANNING, TRANSPORT_VALIDATING,
                        TRANSPORT_EXECUTING, TRANSPORT_STOPPING, TRANSPORT_VERIFYING,
                        CartesianFailureDiagnostics.TRANSPORT_DIAGNOSING}

    def _init_continuous_transport(self):
        self._init_transport_reuse()
        self._init_continuous_return()
        self._init_transport_alternatives()
        self._init_sdk_transport()
        self.transport_is_pick = False
        self.pick_path_restoring = False
        self.pick_path_completed = False
        self.pick_path_target_id = None
        self.transport_fk = self.create_client(GetPositionFK, '/compute_fk')
        self.transport_cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.create_service(Trigger, '/pickup_supervisor/start_continuous_transport',
                            self.start_continuous_transport)
        self.create_service(Trigger, '/pickup_supervisor/start_continuous_transport_chained',
                            self.start_continuous_transport_chained)
        self.create_service(Trigger, '/pickup_supervisor/grasp_and_hold', self.grasp_and_hold_callback)
        self.create_service(Trigger, '/pickup_supervisor/start_pick_waypoints', self.start_pick_waypoints)
        self.declare_parameter('continuous_transport_blend_radius_m', 0.04)
        self.transport_radius = float(self.get_parameter('continuous_transport_blend_radius_m').value)
        self.declare_parameter('continuous_transport_max_joint_jerk_rad_s3', 10.0)
        self.declare_parameter('continuous_transport_enforce_jerk_limit', False)
        self.transport_enforce_jerk_limit = bool(
            self.get_parameter('continuous_transport_enforce_jerk_limit').value)
        self.transport_max_joint_jerk = float(
            self.get_parameter('continuous_transport_max_joint_jerk_rad_s3').value)
        if not math.isfinite(self.transport_max_joint_jerk) or self.transport_max_joint_jerk <= 0:
            raise ValueError('continuous transport joint jerk limit must be finite and positive')
        self.transport_feedback_time = 0.0
        self.continuous_contact_retreat = False
        self.transport_joint_limits = {}
        self.create_subscription(String, '/robot_description', self._transport_model,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _transport_model(self, message):
        try:
            limits = {}
            for joint in ET.fromstring(message.data).findall('joint'):
                if joint.get('name') not in self.arm_joint_names:
                    continue
                limit = joint.find('limit')
                limits[joint.get('name')] = (
                    float(limit.get('lower')), float(limit.get('upper')),
                    float(limit.get('velocity')))
            if len(limits) != len(self.arm_joint_names) or any(
                    not all(math.isfinite(v) for v in row) or row[0] >= row[1] or row[2] <= 0
                    for row in limits.values()):
                raise ValueError('missing or invalid bounded arm joints')
            self.transport_joint_limits = limits
        except (ET.ParseError, ValueError, TypeError, AttributeError) as exc:
            self.transport_joint_limits = {}
            self.get_logger().error(f'continuous transport cannot read robot limits: {exc}')

    def start_continuous_transport(self, _request, response):
        return self._start_continuous_transport(response, return_to_observation=True)

    def start_continuous_transport_chained(self, _request, response):
        return self._start_continuous_transport(response, return_to_observation=False)

    def _start_continuous_transport(self, response, return_to_observation):
        if self.state in self.ACTIVE or self.manual_gripper_pending:
            response.message = 'supervisor is busy'
            return response
        if self._ft_recovery_blocks_start(response):
            return response
        if self.dry_run or not self.transport_joint_limits:
            response.message = 'continuous transport requires real-control mode and bounded robot description'
            return response
        if (not self.pallet_locked or not self.planning_scene_status.get('attached_item_id') or
                self.motion_status.get('state') != 'PREPARED' or
                self.motion_status.get('transfer_context') not in ('pallet', 'staging_store')):
            response.message = 'continuous transport requires a prepared destination and attached item'
            return response
        now = time.monotonic()
        if (self.last_joint_state_time is None or now-self.last_joint_state_time > self.status_timeout or
                self.robot_state_time is None or now-self.robot_state_time > self.status_timeout or
                self.robot_error != 0 or self.robot_mode != 1 or self.robot_state not in (0,2)):
            response.message = 'fresh, healthy ROS-control mode 1 telemetry is required'
            return response
        if self.last_force_time is None or now-self.last_force_time > self.force_timeout:
            response.message = 'fresh force telemetry is required'
            return response
        if not all(c.service_is_ready() for c in (
                self.transport_fk, self.transport_cartesian, self.state_validity_client)):
            response.message = 'MoveIt Cartesian/FK/state-validity service unavailable'
            return response
        if not self.transfer_trajectory_client.server_is_ready():
            response.message = 'joint trajectory controller unavailable'
            return response
        self.operation_id += 1
        self.transport_is_pick = False
        self.transport_is_return = False
        self.transport_route_attempt = 0
        self.staged_timing_used = False
        self.staged_timing_active = False
        self.transport_slot_yaw_flipped = False
        self.verified_slot_transfer_target = None
        self.transport_slot_yaw_deadline = now + 5.0
        self.alternative_observation = None
        self.alternative_candidate_index = 0
        self.alternative_diagnostic_pending = False
        self.transport_alternative_deadline = None
        self.raised_pre_place_offset_m = 0.
        self.raised_pre_place_ready = None
        self.raised_retreat_pose = None
        self.pallet_release_rpy_rad = None
        self.raised_place_hold_on_failure = False
        self.transport_route_generation = getattr(self, 'transport_route_generation', 0) + 1
        self.return_to_observation = return_to_observation
        self.continuous_return_completed = False
        self.continuous_return_target_id = (
            self.motion_status.get('operation_id') if (
                self.continuous_return_enabled or not return_to_observation) else None)
        self.operation_kind = 'loading'
        self.staging_place_active = self.motion_status.get('transfer_context') == 'staging_store'
        self.contact_detected = False
        self.fault = ''
        self.post_retreat_fault = ''
        self.place_fallback_used = False
        self.place_fallback_reason = ''
        self.loading_contact_fallback = False
        self.direct_transfer_succeeded = False
        self.direct_transfer_motion_started = False
        self.transport_sdk_candidate = False
        self.transfer_goal_handle = None
        self.transport_started = now
        self.transport_seed = tuple(self.latest_joint_positions)
        self.transport_target = dict(self.motion_status)
        self.transport_via_observation = self._uses_buffer_route(
            self.transport_target, getattr(self, 'active_pickup_snapshot', None))
        self.transport_scene = dict(self.planning_scene_status)
        self.transport_feedback_time = 0.0
        self.transport_descent_time = None
        self.transport_contact_baseline = None
        self.transport_force_count = 0
        self.transport_terminal = False
        self.transport_cancel_confirmed = False
        self.state = self.TRANSPORT_PLANNING
        self._transport_fk_request(self.transport_seed, self._transport_start_fk)
        response.success = True
        response.message = 'planning one collision-checked lift/transfer/descent trajectory'
        self.publish_status()
        return response

    def _transport_guard(self, callback):
        operation = self.operation_id
        generation = getattr(self, 'transport_route_generation', 0)
        return lambda future: callback(future) if (
            operation == self.operation_id and self.state in self.TRANSPORT_STATES and
            generation == getattr(self, 'transport_route_generation', 0)) else None

    def _transport_fk_request(self, joints, callback):
        req = GetPositionFK.Request()
        req.header.frame_id = 'link_base'
        req.fk_link_names = [self.ik_link_name]
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = list(self.arm_joint_names)
        req.robot_state.joint_state.position = list(joints)
        self.transport_fk.call_async(req).add_done_callback(self._transport_guard(callback))

    @staticmethod
    def _transport_pose(result):
        if result is None or result.error_code.val != 1 or not result.pose_stamped:
            raise ValueError('FK failed')
        p = result.pose_stamped[0].pose
        xyz = np.array([p.position.x,p.position.y,p.position.z])
        if not np.isfinite(xyz).all():
            raise ValueError('FK returned a non-finite position')
        return (xyz,
                quaternion([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]))

    def _transport_start_fk(self, future):
        try:
            start, q = self._transport_pose(future.result())
            end = self.transport_target['pre_place_tcp_xyz_m']
            end_q = self.transport_target['transfer_tcp_quaternion_xyzw']
            if (not getattr(self, 'transport_is_return', False) and
                    not getattr(self, 'transport_is_pick', False) and
                    self.transport_target.get('transfer_context') == 'staging_store' and
                    abs(float(quaternion(q) @ quaternion(end_q))) < math.cos(math.radians(.5)/2)):
                self._fault('slot-store TCP orientation changed since capture; prepare a fresh store target')
                return
            clearance = float(self.transport_target['transport_corner_clearance_z_m'])
            if getattr(self, 'transport_is_pick', False):
                end = np.array(end, dtype=float).copy()
                end[2] += getattr(self, 'raised_pick_offset_m', 0.)
                observation = getattr(self, 'pick_observation_xyz', None)
                source = (self.transport_target.get('planned_pregrasp') or {}).get('pickup_source')
                if not pickup_needs_observation(start, end, source):
                    observation = None
                self.pick_cross_area = observation is not None
                self.get_logger().info('pickup route: ' + (
                    'cross-area via observation-side rail' if observation is not None else
                    'same-area overhead approach; no observation detour'))
                samples, self.transport_safe_z, self.transport_high_z = pick_waypoints(
                    start, q, end, end_q, clearance, self.transport_radius,
                    observation_xyz=observation)
                if observation is not None and getattr(self, 'pick_grid_index', -1) >= 0:
                    candidate = waypoint_candidates()[self.pick_grid_index]
                    samples, self.transport_safe_z, self.transport_high_z = grid_pick_waypoints(
                        start, q, end, end_q, clearance, observation, candidate,
                        rail_x=-.350 if getattr(self, 'pick_grid_rail', 0) else None,
                        yaw_direction=getattr(self, 'pick_grid_direction', 1))
                    self.get_logger().info(f'cross-area pickup grid candidate {self.pick_grid_index}: {candidate}')
            elif getattr(self, 'transport_is_return', False) and not getattr(self, 'return_to_observation', True):
                # Empty-tool handoff is vertical, not a rounded lateral route.
                samples, end = vertical_retreat_waypoints(start, q, clearance)
                end_q = q
                self.transport_safe_z = self.transport_high_z = end[2]
            else:
                samples, self.transport_safe_z, self.transport_high_z = transport_waypoints(
                    start, q, end, end_q, clearance, self.transport_scene, self.transport_radius,
                    allow_tilt_change=getattr(self, 'transport_is_return', False))
            if self.transport_high_z > self._transport_ceiling():
                raise ValueError('blended transport exceeds the configured workspace ceiling')
            self.transport_start_xyz = start
            self.transport_start_q = q
            self.return_collision_column_open = True
            self.return_collision_previous_z = float(start[2])
            self.transport_end = np.array(end)
            self.transport_end_q = quaternion(end_q)
            self.transport_clearance = clearance
            if self._try_reuse_transport(future):
                return
            if (getattr(self, 'transport_via_observation', False) and
                    not getattr(self, 'transport_is_pick', False) and
                    not getattr(self, 'transport_is_return', False)):
                if not self._try_transport_alternative('buffer transfer route', observation_first=True):
                    self._fault('buffer transfer could not start its required observation-side route')
                return
            req = GetCartesianPath.Request()
            req.header.frame_id = 'link_base'
            req.group_name = self.planning_group
            req.link_name = self.ik_link_name
            req.start_state.is_diff = True
            req.start_state.joint_state.name = list(self.arm_joint_names)
            req.start_state.joint_state.position = list(self.transport_seed)
            if getattr(self, 'transport_is_return', False) and getattr(self, 'return_goal_joints', None) is not None:
                # Solve backward from the taught observation joints, then
                # reverse the trajectory. This anchors the exact final branch.
                req.start_state.joint_state.position = list(self.return_goal_joints)
                samples = list(reversed([(start,q)] + samples))[1:]
            for xyz, orientation in samples:
                pose = Pose()
                pose.position.x,pose.position.y,pose.position.z = map(float, xyz)
                (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w) = map(float, orientation)
                req.waypoints.append(pose)
            req.max_step = 0.005
            # A released item's contact at the retreat start must not reject
            # the whole return. Final timed-path validation below still checks
            # every state outside the initial empty-tool vertical column.
            req.avoid_collisions = not getattr(self, 'transport_is_return', False)
            req.jump_threshold = 0.0
            scale = max(0.05, min(1.0, self.motion_speed_percent/100))
            req.max_velocity_scaling_factor = scale
            req.max_acceleration_scaling_factor = scale
            self.transport_cartesian_request = req
            self.transport_cartesian.call_async(req).add_done_callback(
                self._transport_guard(self._transport_planned))
        except (ValueError, KeyError, TypeError) as exc:
            self._fault(f'KINEMATIC_REJECTED: continuous transport geometry: {exc}')
        except Exception as exc:
            self._fault(f'continuous transport planning service failed: {exc}')

    def _transport_planned(self, future):
        try:
            result = future.result()
            if result is None or result.error_code.val != 1:
                raise ValueError('Cartesian planning service failed')
            if not math.isfinite(result.fraction):
                raise ValueError('invalid Cartesian fraction')
            if not 0 <= result.fraction <= 1.:
                raise ValueError('Cartesian fraction is outside [0, 1]')
            if result.fraction < 1-1e-6:
                if self._try_pick_grid():
                    return
                self._diagnose_partial_path(result)
                return
            trajectory = result.solution.joint_trajectory
            timing_issue = cartesian_timing_issue(trajectory, self.arm_joint_names)
            if getattr(self, 'transport_is_return', False) and getattr(self, 'return_goal_joints', None) is not None:
                if len(trajectory.points) < 2:
                    raise ValueError('empty return trajectory')
                total = (trajectory.points[-1].time_from_start.sec +
                         trajectory.points[-1].time_from_start.nanosec*1e-9)
                trajectory.points = list(reversed(trajectory.points))
                for point in trajectory.points:
                    t = point.time_from_start.sec+point.time_from_start.nanosec*1e-9
                    if timing_issue is None:
                        point.time_from_start = Duration(seconds=max(0.0,total-t)).to_msg()
                    point.velocities = [-v for v in point.velocities]
            indices = [list(trajectory.joint_names).index(j) for j in self.arm_joint_names]
            if len(trajectory.points) < 2:
                raise ValueError('empty transport trajectory')
            # Normalize the boundary conditions before constructing/checking
            # the controller spline. Cartesian service output can contain
            # nonzero endpoint velocities even for a rest-to-rest move.
            previous = self.transport_seed
            self.transport_checks = []
            previous_t = 0.0
            for i, point in enumerate(trajectory.points):
                values = tuple(float(point.positions[j]) for j in indices)
                lower, upper = self.direct_transfer_periodic_limits
                values = tuple(self._nearest_periodic_equivalent(v, p, lower, upper)
                               if j in self.direct_transfer_periodic_joint_names else v
                               for j,v,p in zip(self.arm_joint_names,values,previous))
                t = point.time_from_start.sec + point.time_from_start.nanosec*1e-9
                if (not all(math.isfinite(v) for v in values) or
                        (timing_issue is None and (t < previous_t or (i and t <= previous_t)))):
                    raise ValueError('invalid transport trajectory')
                if i == 0 and max(abs(a-b) for a,b in zip(values,previous)) > 0.01:
                    raise ValueError('transport trajectory does not start at the measured joints')
                point.positions = list(values)
                if timing_issue is not None:
                    # Check the complete geometric path before permitting the
                    # timing-only return fallback, even for zero-duration paths.
                    for name, value in zip(self.arm_joint_names, values):
                        lo, hi, _ = self.transport_joint_limits[name]
                        if not lo <= value <= hi:
                            raise ValueError(f'untimed Cartesian path exceeds {name} position limits')
                else:
                    point.velocities = [point.velocities[j] for j in indices]
                    if i == 0 or i == len(trajectory.points)-1:
                        point.velocities = [0.0] * len(self.arm_joint_names)
                previous, previous_t = values,t
            trajectory.joint_names = list(self.arm_joint_names)
            if getattr(self, 'transport_is_return', False) and getattr(self, 'return_goal_joints', None) is not None:
                if max(abs(a-b) for a,b in zip(trajectory.points[-1].positions,self.return_goal_joints)) > 1e-4:
                    raise ValueError('return path does not reach the saved observation joint configuration')
            if timing_issue is not None:
                reason = f'MoveIt Cartesian timing unavailable: {timing_issue}'
                if self._fallback_staged_return(reason):
                    return
                if self._fallback_staged_transport_timing(reason):
                    return
                # No applicable retry: hold/fault. Never fabricate timing for
                # an untimed geometric path or retry an execution failure.
                raise ValueError(reason)
            scale = max(0.05, min(1.0, self.motion_speed_percent/100))
            pool = getattr(self, '_retime_pool', None)
            if pool is None:
                self._retime_pool = pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='transport-retime')
            prior = getattr(self, '_retime_pending', None)
            if prior is not None and not prior[0].done():
                raise ValueError('previous transport timing is still finishing; no new timing queued')
            task = pool.submit(retime_snapshot,
                copy.deepcopy(trajectory), copy.deepcopy(self.transport_joint_limits),
                self.direct_transfer_max_joint_speed, self.direct_transfer_joint_acc,
                self.transport_max_joint_jerk if self.transport_enforce_jerk_limit else None,
                scale)
            self._retime_pending = (task, self.operation_id,
                getattr(self, 'transport_route_generation', 0), self.motion_speed_percent)
            self._retime_discarded = False
            self.state = self.TRANSPORT_PLANNING
            self.publish_status()
        except ValueError as exc:
            self._transport_timing_failed(exc)
        except Exception as exc:
            self._transport_timing_failed(exc)

    def _poll_transport_timing(self):
        pending = getattr(self, '_retime_pending', None)
        if pending is None:
            return
        task, operation, generation, speed = pending
        if (getattr(self, '_retime_discarded', False) or
                operation != self.operation_id or self.state != self.TRANSPORT_PLANNING or
                generation != getattr(self, 'transport_route_generation', 0)):
            self._retime_discarded = True
            task.cancel()
            if task.done():
                self._retime_pending = None
            return
        if not task.done():
            return
        self._retime_pending = None
        try:
            if speed != self.motion_speed_percent:
                raise ValueError('motion speed changed during retiming; replan required')
            trajectory, self.transport_checks, self.transport_duration, duration_ratio, timing = task.result()
            scale = max(0.05, min(1.0, speed/100))
            self.transport_trajectory = trajectory
            worst = timing['initial_worst']
            jerk_cap = (f'{self.transport_max_joint_jerk*scale:.3f} rad/s^3'
                        if self.transport_enforce_jerk_limit else 'disabled (software opt-in)')
            self.get_logger().info(
                f'C2 transport timing ({timing["strategy"]}): '
                f'{timing["original_duration"]:.2f}s -> {self.transport_duration:.2f}s '
                f'({duration_ratio:.3f}x overall); '
                f'adjusted {timing["repaired_segments"]}/{timing["total_segments"]} intervals, '
                f'max interval factor={timing["max_segment_factor"]:.3f}, '
                f'passes={timing["iterations"]}; original limiter: '
                f'{worst["derivative"]} joint={worst["joint"]} interval={worst["segment"]}; '
                f'joint jerk cap={jerk_cap}; predicted peak jerk={timing["peak_jerk"]:.3f} rad/s^3')
            self.transport_check_index = 0
            self._begin_reuse_recording(trajectory)
            self.state = self.TRANSPORT_VALIDATING
            self.get_logger().info(
                f'validating {len(self.transport_checks)} timed transport states; '
                f'item-bottom clearance Z={self.transport_clearance:.3f} m')
            self._transport_validate_next()
        except ValueError as exc:
            self._transport_timing_failed(exc)
        except Exception as exc:
            self._transport_timing_failed(exc)

    def _transport_timing_failed(self, exc):
        if self._reuse_fallback(str(exc)):
            return
        if str(exc).startswith('controller interpolation exceeds'):
            self._fault(f'KINEMATIC_REJECTED: {exc}')
            return
        if (str(exc) == 'transport validation exceeds the bounded sample budget' and
                self._fallback_staged_return(str(exc))):
            return
        self._fault(f'continuous transport trajectory validation failed: {exc}')

    def _transport_validate_next(self):
        if self.transport_check_index == len(self.transport_checks):
            self._transport_execute()
            return
        joints, _ = self.transport_checks[self.transport_check_index]
        if getattr(self, 'transport_is_return', False):
            self._transport_fk_request(joints, self._return_collision_classified)
            return
        self._transport_check_collision(joints)

    def _return_collision_classified(self, future):
        try:
            xyz, q = self._transport_pose(future.result())
            empty = (not (self.transport_scene or {}).get('attached_item_id') and
                     not self.planning_scene_status.get('attached_item_id'))
            vertical = (
                getattr(self, 'return_collision_column_open', False) and empty and
                np.linalg.norm(xyz[:2]-self.transport_start_xyz[:2]) <= .0001 and
                xyz[2] >= self.return_collision_previous_z-.0001 and
                xyz[2] <= self.transport_high_z+.0001 and
                abs(float(q @ self.transport_start_q)) >= math.cos(.001/2))
            if vertical:
                self.return_collision_previous_z = max(self.return_collision_previous_z, float(xyz[2]))
                self._transport_geometry_checked(future)
            else:
                # Once rotation/lateral/downward travel begins, never reopen
                # the exemption, even if a later sample crosses the column.
                self.return_collision_column_open = False
                joints, _ = self.transport_checks[self.transport_check_index]
                self._transport_check_collision(joints)
        except Exception as exc:
            self._fault(f'return collision-phase classification failed: {exc}')

    def _transport_check_collision(self, joints):
        # A continuous Cartesian path may legitimately move an axis more than
        # pi from its initial angle. Absolute joint limits are checked on the
        # controller spline above; still collision-check every sampled state.
        req = GetStateValidity.Request()
        req.group_name = self.planning_group
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = list(self.arm_joint_names)
        req.robot_state.joint_state.position = list(joints)
        self.state_validity_client.call_async(req).add_done_callback(
            self._transport_guard(self._transport_collision_checked))

    def _transport_collision_checked(self, future):
        try:
            result = future.result()
            if result is None:
                raise ValueError('state validation returned no response')
            if not result.valid:
                reason = 'KINEMATIC_REJECTED: timed continuous transport is in collision or violates constraints'
                if self._reuse_fallback(reason):
                    return
                if self._try_pick_grid():
                    return
                if not self._try_transport_alternative(reason) and not self._try_sdk_transport(reason):
                    self._fault(reason)
                return
            joints,_ = self.transport_checks[self.transport_check_index]
            self._transport_fk_request(joints, self._transport_geometry_checked)
        except Exception as exc:
            self._fault(f'continuous transport collision check failed: {exc}')

    def _transport_geometry_checked(self, future):
        try:
            xyz,q = self._transport_pose(future.result())
            _,t = self.transport_checks[self.transport_check_index]
            # The Servo XY box is a local descent region, not the free-space
            # transfer workspace. Do not apply it to the complete transfer.
            if xyz[2] > self._transport_ceiling():
                self._transport_reject_geometry('timed transport exceeds workspace ceiling')
                return
            near_start = np.linalg.norm(xyz[:2]-self.transport_start_xyz[:2]) <= 0.003
            near_end = np.linalg.norm(xyz[:2]-self.transport_end[:2]) <= 0.003
            if not near_start and not near_end:
                if xyz[2]+item_bottom_offset(q,self.transport_scene) < self.transport_clearance-0.001:
                    self._transport_reject_geometry('timed transport cuts below item-bottom clearance')
                    return
            if getattr(self, 'transport_is_pick', False):
                # At low Z only the two vertical columns are allowed. Each
                # column retains its own orientation; reorientation is aloft.
                if xyz[2] < self.transport_safe_z - .001:
                    references = ([self.transport_start_q] if near_start else []) + (
                        [self.transport_end_q] if near_end else [])
                    if not references or max(abs(float(q @ ref)) for ref in references) < math.cos(math.radians(2.5)/2):
                        if self._reuse_fallback('pickup approach changes XY/orientation below clearance'):
                            return
                        self._fault('KINEMATIC_REJECTED: pickup approach changes XY/orientation below clearance')
                        return
            elif not getattr(self, 'transport_is_return', False):
                # Allow free overhead orientation, but never use the source/
                # destination column exemption to rotate a low carried item.
                # Check actual rotated corners, not just the nominal TCP plane.
                bottom = xyz[2] + item_bottom_offset(q, self.transport_scene)
                if bottom < self.transport_clearance - .001:
                    references = ([self.transport_start_q] if near_start else []) + (
                        [self.transport_end_q] if near_end else [])
                    if not references or max(abs(float(q @ ref)) for ref in references) < math.cos(math.radians(2.5)/2):
                        self._transport_reject_geometry('timed transport rotates a carried item below clearance')
                        return
            if near_end and xyz[2] < self.transport_high_z-0.001 and self.transport_descent_time is None:
                # Include the end of the downward bend even when pre-place
                # is so high that the straight descent leg has zero length.
                self.transport_descent_time = t
            if self.transport_check_index == len(self.transport_checks)-1:
                if (np.linalg.norm(xyz-self.transport_end) > 0.003 or
                        abs(float(q @ self.transport_end_q)) < math.cos(math.radians(0.5)/2)):
                    if self._reuse_fallback('timed path does not finish at the pre-place pose'):
                        return
                    self._fault('KINEMATIC_REJECTED: timed path does not finish at the pre-place pose')
                    return
            self._record_reuse_pose(t, xyz, q)
            self.transport_check_index += 1
            self._transport_validate_next()
        except Exception as exc:
            self._fault(f'continuous transport geometry check failed: {exc}')

    def _transport_reject_geometry(self, detail):
        reason = 'KINEMATIC_REJECTED: ' + detail
        if self._reuse_fallback(reason):
            return
        if self._try_pick_grid():
            return
        if not self._try_transport_alternative(reason) and not self._try_sdk_transport(reason):
            self._fault(reason)

    def _grid_deadline_tick(self, now):
        if (self.state in (self.TRANSPORT_PLANNING, self.TRANSPORT_DIAGNOSING) and
                not getattr(self, 'direct_transfer_motion_started', False) and
                getattr(self, 'transfer_goal_handle', None) is None):
            if (getattr(self, 'transport_is_pick', False) and
                    getattr(self, 'pick_cross_area', False) and
                    now >= (getattr(self, 'pick_grid_deadline', None) or math.inf)):
                if not self._try_pick_grid():
                    self.transport_route_generation += 1
                    self._fault('cross-area pickup grids exhausted; no motion executed')
                return True
            if (not getattr(self, 'transport_is_pick', False) and
                    not getattr(self, 'transport_is_return', False) and
                    getattr(self, 'transport_route_attempt', 0) == 1 and
                    now >= getattr(self, 'alternative_grid_deadline', math.inf)):
                if not self._try_transport_alternative('grid deadline reached'):
                    self.transport_route_generation += 1
                    self._fault('cross-area transfer grids exhausted; item remains held')
                return True
        return False

    def _transport_execute(self):
        now = time.monotonic()
        if (not self.pallet_locked or self.robot_mode != 1 or self.robot_state not in (0,2) or
                self.robot_error != 0 or self.robot_state_time is None or now-self.robot_state_time > self.status_timeout or
                self.last_joint_state_time is None or now-self.last_joint_state_time > self.status_timeout or
                self.last_force_time is None or now-self.last_force_time > self.force_timeout or
                self.motion_status.get('operation_id') != self.transport_target.get('operation_id') or
                (self.planning_scene_status.get('attached_item_id') or '') !=
                ((self.transport_scene or {}).get('attached_item_id') or '')):
            self._fault('continuous transport prerequisites changed during planning')
            return
        if max(abs(a-b) for a,b in zip(self.latest_joint_positions,self.transport_seed)) > 0.01:
            self._fault('robot moved during continuous transport planning')
            return
        if getattr(self, 'transport_is_pick', False) and self._check_pick_descent_before_execution():
            return
        if getattr(self, 'transport_sdk_candidate', False):
            self._sdk_begin_execution()
            return
        if self._reuse_is_active():
            self.get_logger().info('cross-frame cache accepted after full validation; executing reconnected path')
            self.reuse_active = False
        self.state = self.TRANSPORT_EXECUTING
        self.transport_execution_started = time.monotonic()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.transport_trajectory
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0
        goal.goal_time_tolerance = Duration(seconds=2.0).to_msg()
        operation = self.operation_id
        future = self.transfer_trajectory_client.send_goal_async(
            goal, feedback_callback=lambda msg: self._transport_feedback(msg)
            if operation == self.operation_id else None)
        future.add_done_callback(lambda f: self._transport_goal_received(f, operation))
        label = 'retreat/observation return' if getattr(self, 'transport_is_return', False) else 'lift/transfer/descent'
        if getattr(self, 'transport_is_pick', False):
            label = 'empty-tool pickup approach (lift/overhead/pre-pick)'
        self.get_logger().info(f'executing continuous {label} in {self.transport_duration:.2f} s')

    def _transport_goal_received(self, future, operation):
        try:
            handle = future.result()
            if operation != self.operation_id or self.state != self.TRANSPORT_EXECUTING:
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
                return
            if handle is None or not handle.accepted:
                raise ValueError('controller rejected continuous trajectory')
            self.transfer_goal_handle = handle
            self.direct_transfer_motion_started = True
            self._mark_reuse_execution()
            handle.get_result_async().add_done_callback(self._transport_guard(self._transport_result))
        except Exception as exc:
            self._fault(f'continuous transport action failed: {exc}')

    def _transport_feedback(self, message):
        if self.state != self.TRANSPORT_EXECUTING:
            return
        t = message.feedback.desired.time_from_start
        self.transport_feedback_time = t.sec+t.nanosec*1e-9

    def _transport_result(self, future):
        try:
            result = future.result()
            if self.state == self.TRANSPORT_STOPPING:
                if result.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                    raise ValueError('controller did not confirm a clean contact stop')
                self.transport_terminal = True
                self.transfer_goal_handle = None
                return
            self.transfer_goal_handle = None
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise ValueError(f'controller execution failed: {result.result.error_string}')
            # Cached Servo status is published at 2 Hz and may describe the
            # preceding descent. Verify FK from a complete measured joint
            # sample acquired AFTER controller completion instead.
            self.state = self.TRANSPORT_VERIFYING
            self.transport_verify_started = time.monotonic()
            self.transport_completed_stamp_ns = self.get_clock().now().nanoseconds
            self.transport_verify_sample = None
            self.transport_verify_pending = False
            self.transport_verify_error = None
            self.transport_verify_joint_error = None
            self.get_logger().info('continuous trajectory finished; waiting for fresh final-pose verification')
            self.publish_status()
        except Exception as exc:
            self._fault(f'continuous transport result failed: {exc}')

    def _transport_verify_tick(self, now):
        if now - self.transport_verify_started > 5.0:
            detail = ('no fresh post-completion FK result' if self.transport_verify_error is None
                      else f'last TCP position error={self.transport_verify_error*1000:.1f} mm (limit 10.0 mm)')
            if getattr(self, 'transport_verify_joint_error', None) is not None:
                detail += f'; observation joint error={self.transport_verify_joint_error:.4f} rad'
            disposition = ('item already released; return remains incomplete'
                           if getattr(self, 'transport_is_return', False) else 'item remains held')
            if getattr(self, 'transport_is_pick', False):
                disposition = 'empty-tool approach incomplete; contact pickup blocked'
            self._fault(f'continuous transport final-pose verification timed out: {detail}; {disposition}')
            return
        sample = getattr(self, 'transport_joint_sample_time', None)
        stamp = getattr(self, 'transport_joint_stamp_ns', None)
        if (self.transport_verify_pending or sample is None or stamp is None or
                sample <= self.transport_verify_started or
                stamp <= self.transport_completed_stamp_ns or
                now-sample > self.status_timeout or sample == self.transport_verify_sample):
            return
        self.transport_verify_sample = sample
        self.transport_verify_pending = True
        self.transport_verify_joints = tuple(self.latest_joint_positions)
        self._transport_fk_request(self.transport_verify_joints, self._transport_final_fk)

    def _transport_final_fk(self, future):
        if self.state != self.TRANSPORT_VERIFYING:
            return
        self.transport_verify_pending = False
        now = time.monotonic()
        if now-self.transport_verify_started > 5.0:
            self._transport_verify_tick(now)
            return
        if now-self.transport_verify_sample > self.status_timeout:
            return  # A delayed service response cannot authorize descent.
        try:
            xyz,q = self._transport_pose(future.result())
            self.transport_verify_error = float(np.linalg.norm(xyz-self.transport_end))
            if self.transport_verify_error > 0.01:
                return  # Allow bounded settling; do not relax the tolerance.
            if getattr(self, 'transport_is_pick', False):
                if abs(float(q @ self.transport_end_q)) < math.cos(math.radians(2.5)/2):
                    return
                if getattr(self, 'raised_pick_offset_m', 0.) > 0:
                    if self.transport_verify_error > .002:
                        return
                    self.raised_pick_ready = dict(
                        target_id=self.transport_target['operation_id'],
                        retrieval_target_id=self.transport_target['planned_pregrasp']['retrieval_target_id'],
                        xyz=list(self.transport_end),
                        original_z=float(self.transport_target['pre_place_tcp_xyz_m'][2]))
                self.pick_path_completed = True
            elif getattr(self, 'transport_is_return', False):
                if self.return_goal_joints is not None:
                    self.transport_verify_joint_error = max(
                        abs(a-b) for a,b in zip(self.transport_verify_joints,self.return_goal_joints))
                    if self.transport_verify_joint_error > self.direct_transfer_joint_tolerance:
                        return
                self.continuous_return_completed = True
            else:
                if (getattr(self, 'transport_target', {}).get('transfer_context') == 'staging_store' and
                        abs(float(q @ self.transport_end_q)) < math.cos(math.radians(2.5)/2)):
                    return  # Require settled orientation before acknowledging slot transfer.
                self.direct_transfer_succeeded = True
                if getattr(self, 'raised_pre_place_offset_m', 0.) > 0:
                    if self.transport_verify_error > .002 or abs(float(q @ self.transport_end_q)) < math.cos(.005/2):
                        self.direct_transfer_succeeded = False
                        return
                    self.raised_pre_place_ready = dict(
                        target_id=self.transport_target.get('operation_id'),
                        xyz=list(self.transport_end),
                        original_z=float(self.transport_target['pre_place_tcp_xyz_m'][2]),
                        item_id=self.transport_scene.get('attached_item_id'))
                if getattr(self, 'transport_target', {}).get('transfer_context') == 'staging_store':
                    self.verified_slot_transfer_target = dict(self.transport_target)
                    self.verified_slot_transfer_target['pre_place_tcp_xyz_m'] = list(self.transport_end)
                    self.verified_slot_transfer_target['transfer_tcp_quaternion_xyzw'] = list(self.transport_end_q)
            self._remember_overhead_path()
            self.direct_target_z = float(self.transport_end[2])
            self.state = self.SUCCEEDED
            self.get_logger().info(
                f'continuous final TCP verified: error={self.transport_verify_error*1000:.1f} mm')
            self.publish_status()
        except Exception as exc:
            self._fault(f'continuous transport final FK failed: {exc}')

    def _transport_force(self, force_z):
        if self.state != self.TRANSPORT_EXECUTING or getattr(self, 'transport_is_return', False):
            return
        if self.transport_descent_time is None or self.transport_feedback_time < self.transport_descent_time:
            self.transport_contact_baseline = force_z
            return
        if self.transport_contact_baseline is None:
            self._fault('continuous descent has no loaded force baseline')
            return
        delta = abs(force_z-self.transport_contact_baseline)
        self.transport_force_count = self.transport_force_count+1 if delta >= self.place_force_threshold else 0
        if self.transport_force_count < self.loading_contact_confirm_samples:
            return
        if getattr(self, 'transport_is_pick', False):
            # Contact before pre-pick is unexpected, not placement success.
            # _fault cancels the trajectory; never grasp or run release fallback.
            self._fault('unexpected force contact during pickup approach; descent blocked')
            return
        if self.transfer_goal_handle is None:
            self._fault('cannot cancel continuous transport on contact')
            return
        self.state = self.TRANSPORT_STOPPING
        self.transport_stop_started = time.monotonic()
        self.transport_still_since = None
        self.transport_stop_joints = tuple(self.latest_joint_positions)
        self.transport_stop_stamp = self.last_joint_state_time
        self.transfer_goal_handle.cancel_goal_async().add_done_callback(
            self._transport_guard(self._transport_cancelled))

    def _transport_cancelled(self, future):
        try:
            result = future.result()
            if result is None or not result.goals_canceling:
                raise ValueError('trajectory cancellation rejected')
            self.transport_cancel_confirmed = True
        except Exception as exc:
            self._fault(f'continuous contact stop failed: {exc}')

    def _transport_tick(self):
        if self.state not in self.TRANSPORT_STATES:
            self._poll_transport_timing()  # Discard canceled/obsolete work only.
            return False
        now = time.monotonic()
        if self.robot_error not in (None,0):
            self._fault(f'xArm error {self.robot_error} during continuous transport')
            return True
        if self._reuse_tick(now):
            return True
        if self._grid_deadline_tick(now):
            return True
        deadline = getattr(self, 'transport_alternative_deadline', None)
        if self._try_slot_yaw_flip(now):
            return True
        if (deadline is not None and not self.direct_transfer_motion_started and
                not getattr(self, 'transport_is_pick', False) and
                not getattr(self, 'transport_is_return', False) and
                self.state in (self.TRANSPORT_PLANNING, self.TRANSPORT_VALIDATING,
                               self.TRANSPORT_DIAGNOSING) and now >= deadline):
            self._fault('KINEMATIC_REJECTED: transport alternatives exceeded planning budget; item remains held')
            return True
        if self.state == self.TRANSPORT_DIAGNOSING:
            if now > self.transport_diagnostic_deadline:
                self._diagnostic_finish('probe timed out; cause unresolved')
            return True
        if self.state in (self.TRANSPORT_PLANNING,self.TRANSPORT_VALIDATING):
            if now-self.transport_started > 30:
                self._fault('continuous transport planning/validation timed out')
            else:
                self._poll_transport_timing()
            return True
        if (self.robot_state_time is None or now-self.robot_state_time > self.status_timeout or
                self.robot_mode != 1 or self.robot_state not in (0,1,2) or
                self.last_joint_state_time is None or now-self.last_joint_state_time > self.status_timeout or
                self.last_force_time is None or now-self.last_force_time > self.force_timeout):
            self._fault('stale feedback during continuous transport')
            return True
        if self.state == self.TRANSPORT_EXECUTING:
            if now-self.transport_execution_started > self.transport_duration+5:
                self._fault('continuous transport execution timed out')
            return True
        if self.state == self.TRANSPORT_VERIFYING:
            self._transport_verify_tick(now)
            return True
        if now-self.transport_stop_started > 5:
            self._fault('continuous contact stop not confirmed; item remains held')
            return True
        if not self.transport_terminal or not self.transport_cancel_confirmed:
            return True
        if self.last_joint_state_time == self.transport_stop_stamp:
            return True
        current = tuple(self.latest_joint_positions)
        if max(abs(a-b) for a,b in zip(current,self.transport_stop_joints)) > 0.001:
            self.transport_still_since = None
            self.transport_stop_joints = current
        elif self.transport_still_since is None:
            self.transport_still_since = now
        self.transport_stop_stamp = self.last_joint_state_time
        if self.transport_still_since is not None and now-self.transport_still_since >= 0.25:
            self.loading_contact_fallback = True
            self.place_fallback_used = True
            self.place_fallback_reason = 'force contact during continuous descent'
            self.contact_detected = True
            self.direct_target_z = float(self.transport_target['transfer_tcp_z_m'])
            self.direct_target_pose = None
            self.direct_tcp_z_offset = None
            self.continuous_contact_retreat = True
            self._turn_vacuum_off()
        return True
