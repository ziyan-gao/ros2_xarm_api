"""Continuous pallet transport; planning services never command the robot."""
import math
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
from .cartesian_failure_diagnostics import CartesianFailureDiagnostics
from .smooth_transport import smooth_and_sample


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


def quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('invalid transport orientation')
    return q / np.linalg.norm(q)


def rotate(q, v):
    q = quaternion(q)
    v = np.asarray(v, dtype=float)
    return v + 2 * np.cross(q[:3], np.cross(q[:3], v) + q[3] * v)


def slerp(a, b, u):
    a, b = quaternion(a), quaternion(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > 0.9995:
        return quaternion(a + u * (b - a))
    angle = math.acos(np.clip(dot, -1, 1))
    return (math.sin((1-u)*angle)*a + math.sin(u*angle)*b) / math.sin(angle)


def item_bottom_offset(q, scene):
    if scene is None:
        return 0.0  # Empty-tool return: clearance is specified for the TCP.
    size = np.asarray(scene['attached_item_size_m'], dtype=float)
    center = np.asarray(scene['attached_item_center_in_tcp_m'], dtype=float)
    iq = scene['attached_item_orientation_in_tcp_xyzw']
    if size.shape != (3,) or center.shape != (3,) or not np.isfinite([size, center]).all() or np.any(size <= 0):
        raise ValueError('invalid carried-item geometry')
    return min(rotate(q, center + rotate(iq, size * np.array([x,y,z]) / 2))[2]
               for x in (-1,1) for y in (-1,1) for z in (-1,1))


def transport_waypoints(start, start_q, end, end_q, clearance_z, scene,
                        radius=0.04, step=0.005, allow_tilt_change=False):
    """C2 rounded corners above the item-bottom clearance, vertical end legs.

    Yaw changes only on the elevated straight segment. A 2 mm extra margin
    covers numerical sampling of the carried-item orientation envelope.
    """
    start, end = np.asarray(start, float), np.asarray(end, float)
    start_q, end_q = quaternion(start_q), quaternion(end_q)
    if not np.isfinite([*start, *end, clearance_z, radius, step]).all() or radius <= 0 or step <= 0:
        raise ValueError('invalid continuous transport geometry')
    if not allow_tilt_change and float(rotate(start_q, [0,0,1]) @ rotate(end_q, [0,0,1])) < math.cos(math.radians(2)):
        raise ValueError('pickup and placement tool tilt differ by more than 2 degrees')
    delta = end[:2] - start[:2]
    distance = float(np.linalg.norm(delta))
    if distance < 0.01:
        raise ValueError('insufficient lateral distance for a rounded transfer')
    direction = np.array([* (delta / distance), 0.0])
    r = min(radius, distance / 4)
    bottom = min(item_bottom_offset(slerp(start_q, end_q, u), scene)
                 for u in np.linspace(0, 1, 101))
    safe_z = max(float(clearance_z) - bottom + 0.002, start[2], end[2])
    high_z = safe_z + r
    a = np.array([*start[:2], safe_z])
    b = a + direction*r + np.array([0,0,r])
    d = np.array([*end[:2], safe_z])
    c = d - direction*r + np.array([0,0,r])
    samples = []
    def line(p, q, qa, qb):
        angle = 2*math.acos(min(1, abs(float(quaternion(qa) @ quaternion(qb)))))
        count = max(1, math.ceil(np.linalg.norm(q-p)/step), math.ceil(angle/0.025))
        for u in np.linspace(0, 1, count+1)[1:]:
            samples.append((p+(q-p)*u, slerp(qa, qb, u**3*(10-15*u+6*u*u))))
    def bend(p, q, incoming, outgoing, orientation):
        controls = [p, p+incoming*r/4, p+incoming*r/2,
                    q-outgoing*r/2, q-outgoing*r/4, q]
        for u in np.linspace(0, 1, max(4, math.ceil(2*r/step))+1)[1:]:
            position = sum(math.comb(5,i)*(1-u)**(5-i)*u**i*v
                           for i,v in enumerate(controls))
            samples.append((position, orientation))
    line(start, a, start_q, start_q)
    bend(a, b, np.array([0,0,1]), direction, start_q)
    line(b, c, start_q, end_q)
    bend(c, d, direction, np.array([0,0,-1]), end_q)
    line(d, end, end_q, end_q)
    return samples, safe_z, high_z


class ContinuousTransport(CartesianFailureDiagnostics, ContinuousReturn):
    TRANSPORT_PLANNING = 'TRANSPORT_PLANNING'
    TRANSPORT_VALIDATING = 'TRANSPORT_VALIDATING'
    TRANSPORT_EXECUTING = 'TRANSPORT_EXECUTING'
    TRANSPORT_STOPPING = 'TRANSPORT_STOPPING'
    TRANSPORT_VERIFYING = 'TRANSPORT_VERIFYING'
    TRANSPORT_STATES = {TRANSPORT_PLANNING, TRANSPORT_VALIDATING,
                        TRANSPORT_EXECUTING, TRANSPORT_STOPPING, TRANSPORT_VERIFYING,
                        CartesianFailureDiagnostics.TRANSPORT_DIAGNOSING}

    def _init_continuous_transport(self):
        self._init_continuous_return()
        self.transport_fk = self.create_client(GetPositionFK, '/compute_fk')
        self.transport_cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.create_service(Trigger, '/pickup_supervisor/start_continuous_transport',
                            self.start_continuous_transport)
        self.create_service(Trigger, '/pickup_supervisor/grasp_and_hold', self.grasp_and_hold_callback)
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
                self.motion_status.get('transfer_context') != 'pallet'):
            response.message = 'continuous transport requires a prepared pallet target and attached item'
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
        self.transport_is_return = False
        self.continuous_return_completed = False
        self.continuous_return_target_id = (
            self.motion_status.get('operation_id') if self.continuous_return_enabled else None)
        self.operation_kind = 'loading'
        self.staging_place_active = False
        self.contact_detected = False
        self.fault = ''
        self.post_retreat_fault = ''
        self.place_fallback_used = False
        self.place_fallback_reason = ''
        self.loading_contact_fallback = False
        self.direct_transfer_succeeded = False
        self.direct_transfer_motion_started = False
        self.transfer_goal_handle = None
        self.transport_started = now
        self.transport_seed = tuple(self.latest_joint_positions)
        self.transport_target = dict(self.motion_status)
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
        return lambda future: callback(future) if (
            operation == self.operation_id and self.state in self.TRANSPORT_STATES) else None

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
            clearance = float(self.transport_target['transport_corner_clearance_z_m'])
            samples, self.transport_safe_z, self.transport_high_z = transport_waypoints(
                start, q, end, end_q, clearance, self.transport_scene, self.transport_radius,
                allow_tilt_change=getattr(self, 'transport_is_return', False))
            if self.transport_high_z > self.servo_bounds_mm[5]/1000:
                raise ValueError('blended transport exceeds the configured workspace ceiling')
            self.transport_start_xyz = start
            self.transport_start_q = q
            self.transport_end = np.array(end)
            self.transport_end_q = quaternion(end_q)
            self.transport_clearance = clearance
            req = GetCartesianPath.Request()
            req.header.frame_id = 'link_base'
            req.group_name = self.planning_group
            req.link_name = self.ik_link_name
            req.start_state.is_diff = True
            req.start_state.joint_state.name = list(self.arm_joint_names)
            req.start_state.joint_state.position = list(self.transport_seed)
            if getattr(self, 'transport_is_return', False):
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
            req.avoid_collisions = True
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
                self._diagnose_partial_path(result)
                return
            trajectory = result.solution.joint_trajectory
            timing_issue = cartesian_timing_issue(trajectory, self.arm_joint_names)
            if getattr(self, 'transport_is_return', False):
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
            if getattr(self, 'transport_is_return', False):
                if max(abs(a-b) for a,b in zip(trajectory.points[-1].positions,self.return_goal_joints)) > 1e-4:
                    raise ValueError('return path does not reach the saved observation joint configuration')
            if timing_issue is not None:
                reason = f'MoveIt Cartesian timing unavailable: {timing_issue}'
                if self._fallback_staged_return(reason):
                    return
                # Outbound: keep the item held and stop. Do not invent zero
                # derivatives or blindly execute the raw geometric path.
                raise ValueError(reason)
            scale = max(0.05, min(1.0, self.motion_speed_percent/100))
            timing = {}
            self.transport_checks, self.transport_duration, duration_ratio = smooth_and_sample(
                trajectory, self.transport_joint_limits,
                self.direct_transfer_max_joint_speed, self.direct_transfer_joint_acc,
                self.transport_max_joint_jerk if self.transport_enforce_jerk_limit else None,
                scale, timing)
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
            self.state = self.TRANSPORT_VALIDATING
            self.get_logger().info(
                f'validating {len(self.transport_checks)} timed transport states; '
                f'item-bottom clearance Z={self.transport_clearance:.3f} m')
            self._transport_validate_next()
        except ValueError as exc:
            if str(exc).startswith('controller interpolation exceeds'):
                self._fault(f'KINEMATIC_REJECTED: {exc}')
                return
            if (str(exc) == 'transport validation exceeds the bounded sample budget' and
                    self._fallback_staged_return(str(exc))):
                return
            # Malformed timing/derivatives are not evidence that the sampled
            # loading pose is unreachable. Stop instead of exhausting targets.
            self._fault(f'continuous transport trajectory validation failed: {exc}')
        except Exception as exc:
            self._fault(f'continuous transport planning failed: {exc}')

    def _transport_validate_next(self):
        if self.transport_check_index == len(self.transport_checks):
            self._transport_execute()
            return
        joints, _ = self.transport_checks[self.transport_check_index]
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
                self._fault('KINEMATIC_REJECTED: timed continuous transport is in collision or violates constraints')
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
            if xyz[2] > self.servo_bounds_mm[5]/1000:
                self._fault('KINEMATIC_REJECTED: timed transport exceeds workspace ceiling')
                return
            near_start = np.linalg.norm(xyz[:2]-self.transport_start_xyz[:2]) <= 0.003
            near_end = np.linalg.norm(xyz[:2]-self.transport_end[:2]) <= 0.003
            if not near_start and not near_end:
                if xyz[2]+item_bottom_offset(q,self.transport_scene) < self.transport_clearance-0.001:
                    self._fault('KINEMATIC_REJECTED: timed transport cuts below item-bottom clearance')
                    return
            if not getattr(self, 'transport_is_return', False) and float(rotate(q,[0,0,1]) @ rotate(self.transport_end_q,[0,0,1])) < math.cos(math.radians(2.5)):
                self._fault('KINEMATIC_REJECTED: timed transport violates tool tilt tolerance')
                return
            if near_end and xyz[2] < self.transport_high_z-0.001 and self.transport_descent_time is None:
                # Include the end of the downward bend even when pre-place
                # is so high that the straight descent leg has zero length.
                self.transport_descent_time = t
            if self.transport_check_index == len(self.transport_checks)-1:
                if (np.linalg.norm(xyz-self.transport_end) > 0.003 or
                        abs(float(q @ self.transport_end_q)) < math.cos(math.radians(0.5)/2)):
                    self._fault('KINEMATIC_REJECTED: timed path does not finish at the pre-place pose')
                    return
            self.transport_check_index += 1
            self._transport_validate_next()
        except Exception as exc:
            self._fault(f'continuous transport geometry check failed: {exc}')

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
            xyz,_ = self._transport_pose(future.result())
            self.transport_verify_error = float(np.linalg.norm(xyz-self.transport_end))
            if self.transport_verify_error > 0.01:
                return  # Allow bounded settling; do not relax the tolerance.
            if getattr(self, 'transport_is_return', False):
                self.transport_verify_joint_error = max(
                    abs(a-b) for a,b in zip(self.transport_verify_joints,self.return_goal_joints))
                if self.transport_verify_joint_error > self.direct_transfer_joint_tolerance:
                    return
                self.continuous_return_completed = True
            else:
                self.direct_transfer_succeeded = True
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
            return False
        now = time.monotonic()
        if self.robot_error not in (None,0):
            self._fault(f'xArm error {self.robot_error} during continuous transport')
            return True
        if self.state == self.TRANSPORT_DIAGNOSING:
            if now > self.transport_diagnostic_deadline:
                self._diagnostic_finish('probe timed out; cause unresolved')
            return True
        if self.state in (self.TRANSPORT_PLANNING,self.TRANSPORT_VALIDATING):
            if now-self.transport_started > 30:
                self._fault('continuous transport planning/validation timed out')
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
