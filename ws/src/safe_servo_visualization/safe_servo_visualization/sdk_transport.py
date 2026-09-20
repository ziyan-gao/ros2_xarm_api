"""Last-resort, checked joint moves through the xArm ROS SDK service.

This is not an unchecked Cartesian fallback. IK/scene checks remain in MoveIt;
execution alone uses mode-0 set_servo_angle, never Servo-J. Each SDK segment is
rest-to-rest and monitored before the next command is sent.
"""
import math
import time
from types import SimpleNamespace

import numpy as np
from action_msgs.msg import GoalStatus
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xarm_msgs.srv import MoveJoint

from .transport_alternatives import buffer_transfer_waypoints, pose_message


def nearest_equivalent_joints(goal, seed, names, limits):
    """Choose bounded 2*pi-equivalent angles nearest the preceding UF arm state.

    UF arm joints are revolute. Never wrap the measured seed or relax limits;
    FK and the complete interpolated path are still validated downstream.
    """
    if len(goal) != len(names) or len(seed) != len(names):
        raise ValueError('invalid IK joint layout')
    chosen = []
    for name, value, previous in zip(names, goal, seed):
        lower, upper = limits[name][:2]
        if (not all(math.isfinite(v) for v in (value, previous, lower, upper)) or
                not lower <= value <= upper or not lower <= previous <= upper):
            raise ValueError(f'IK/seed exceeds joint limits: {name}')
        first = math.ceil((lower-value)/(2*math.pi))
        last = math.floor((upper-value)/(2*math.pi))
        candidates = [value+2*math.pi*k for k in range(first, last+1)]
        chosen.append(min(candidates, key=lambda v: (abs(v-previous), abs(v-value))))
    return tuple(chosen)


def joint_line(start, end, names, speed, acceleration):
    """Timed straight joint-space segment for the existing planning pipeline."""
    a, b = np.asarray(start, float), np.asarray(end, float)
    if (a.shape != b.shape or a.shape != (len(names),) or
            not np.isfinite([a, b]).all() or not math.isfinite(speed) or
            not math.isfinite(acceleration) or speed <= 0 or acceleration <= 0):
        raise ValueError('invalid SDK joint segment')
    delta = b-a
    distance = float(np.max(np.abs(delta)))
    duration = max(.2, 1.875*distance/speed, math.sqrt(5.8*distance/acceleration))
    trajectory = JointTrajectory(joint_names=list(names))
    # The SDK executes this straight segment with its own bounded timing.
    # These derivatives are only for joining/planning, not sent to hardware.
    for u in np.linspace(0., 1., max(2, math.ceil(distance/.05)+1)):
        s = 10*u**3-15*u**4+6*u**5
        velocity = delta*(30*u**2-60*u**3+30*u**4)/duration
        acceleration_q = delta*(60*u-180*u**2+120*u**3)/duration**2
        trajectory.points.append(JointTrajectoryPoint(
            positions=list(a+s*delta), velocities=list(velocity),
            accelerations=list(acceleration_q),
            time_from_start=Duration(seconds=float(u*duration)).to_msg()))
    return trajectory


def sdk_joint_samples(trajectory, names, limits, max_step=.01, max_samples=4000):
    """Validate exactly the piecewise joint lines sent to the SDK, not a spline."""
    if list(trajectory.joint_names) != list(names) or len(trajectory.points) < 2:
        raise ValueError('invalid SDK trajectory joint layout')
    commands = []
    checks = []
    previous = None
    for point in trajectory.points:
        q = np.asarray(point.positions, float)
        if q.shape != (len(names),) or not np.isfinite(q).all():
            raise ValueError('invalid SDK joint target')
        if any(not limits[name][0] <= value <= limits[name][1] for name, value in zip(names, q)):
            raise ValueError('SDK target exceeds joint limits')
        if previous is None:
            checks.append((tuple(q), 0.))
        elif np.max(np.abs(q-previous)) > 1e-9:
            count = max(1, math.ceil(float(np.max(np.abs(q-previous)))/max_step))
            if len(checks)+count > max_samples:
                raise ValueError('SDK validation sample budget exceeded')
            base_index = len(checks)
            checks.extend((tuple(previous+(q-previous)*u), float(base_index+i))
                          for i, u in enumerate(np.linspace(0., 1., count+1)[1:]))
            command_count = max(1, math.ceil(float(np.max(np.abs(q-previous)))/.05))
            commands.extend(tuple(previous+(q-previous)*u)
                            for u in np.linspace(0., 1., command_count+1)[1:])
        previous = q
    if not commands:
        raise ValueError('SDK fallback has no motion')
    return commands, checks


class SdkTransport:
    def _init_sdk_transport(self):
        self.declare_parameter('sdk_transport_fallback_enabled', True)
        self.sdk_transport_enabled = bool(self.get_parameter('sdk_transport_fallback_enabled').value)
        self.sdk_joint_client = self.create_client(MoveJoint, '/ufactory/set_servo_angle')
        self.transport_sdk_candidate = False
        self.transport_sdk_active = False

    def _try_sdk_transport(self, reason):
        if (not getattr(self, 'sdk_transport_enabled', False) or
                self.transport_route_attempt != 2 or
                self.state not in (self.TRANSPORT_PLANNING, self.TRANSPORT_VALIDATING,
                                   self.TRANSPORT_DIAGNOSING) or
                getattr(self, 'transport_is_pick', False) or getattr(self, 'transport_is_return', False) or
                self.direct_transfer_motion_started or self.transfer_goal_handle is not None or
                not self.transport_scene or not self.transport_scene.get('attached_item_id') or
                time.monotonic() >= self.transport_alternative_deadline):
            return False
        if not self.sdk_joint_client.service_is_ready() or not self.compute_ik_client.service_is_ready():
            return False
        self.transport_route_attempt = 3
        self.transport_sdk_candidate = True
        self.transport_route_generation += 1
        self.transport_started = time.monotonic()
        self.state = self.TRANSPORT_PLANNING
        self.alternative_seed = tuple(self.transport_seed)
        self.alternative_parts = []
        self.alternative_current_xyz = np.array(self.transport_start_xyz)
        self.alternative_current_q = self.transport_start_q
        self.alternative_segment_number = 0
        self.get_logger().warning(f'MoveIt route exhausted: {reason}; planning checked SDK joint fallback, no motion yet')
        # Recover the saved observation pose via the same read-only FK path.
        import yaml
        try:
            with open(self.return_waypoint_file, encoding='utf-8') as stream:
                saved = yaml.safe_load(stream)['waypoints']['observation']
            mapping = dict(zip(saved['joint_names'], saved['positions_rad']))
            joints = tuple(float(mapping[n]) for n in self.arm_joint_names)
            if any(not math.isfinite(v) or not self.transport_joint_limits[n][0] <= v <=
                   self.transport_joint_limits[n][1] for n, v in zip(self.arm_joint_names, joints)):
                raise ValueError('invalid saved observation joints')
            self._transport_fk_request(joints, self._sdk_observation_received)
        except Exception as exc:
            self._fault(f'SDK fallback setup failed: {exc}; item remains held')
        return True

    def _sdk_observation_received(self, future):
        try:
            observation, _ = self._transport_pose(future.result())
            a, b = buffer_transfer_waypoints(self.transport_start_xyz, self.transport_end,
                                            observation, self.transport_high_z)
            qa, qb = self.transport_start_q, self.transport_end_q
            self.alternative_segments = [
                ('cartesian', np.array([*self.transport_start_xyz[:2], self.transport_high_z]), qa),
                ('sdk_joint', a, qa), ('sdk_joint', b, qb),
                ('sdk_joint', np.array([*self.transport_end[:2], self.transport_high_z]), qb),
                ('cartesian', self.transport_end, qb)]
            self._alternative_next()
        except Exception as exc:
            self._fault(f'SDK fallback route setup failed: {exc}')

    def _sdk_plan_joint(self, xyz, q):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name, ik.ik_link_name = self.planning_group, self.ik_link_name
        ik.robot_state.is_diff = True
        ik.robot_state.joint_state.name = list(self.arm_joint_names)
        ik.robot_state.joint_state.position = list(self.alternative_seed)
        ik.avoid_collisions = True
        ik.pose_stamped.header.frame_id = 'link_base'
        ik.pose_stamped.pose = pose_message(xyz, q)
        ik.timeout = Duration(seconds=2.).to_msg()
        self.compute_ik_client.call_async(request).add_done_callback(
            self._transport_guard(self._sdk_ik_received))

    def _sdk_ik_received(self, future):
        try:
            result = future.result()
            if result is None or result.error_code.val != 1:
                raise ValueError(f'endpoint IK failed: code={None if result is None else result.error_code.val}')
            state = result.solution.joint_state
            mapping = dict(zip(state.name, state.position))
            raw_goal = tuple(float(mapping[n]) for n in self.arm_joint_names)
            goal = nearest_equivalent_joints(raw_goal, self.alternative_seed,
                                             self.arm_joint_names, self.transport_joint_limits)
            if goal != raw_goal:
                self.get_logger().info(
                    f'SDK IK equivalent-angle selection: raw={raw_goal}, selected={goal}, '
                    f'seed={self.alternative_seed}')
            deltas = [abs(a-b) for a, b in zip(goal, self.alternative_seed)]
            index = max(range(len(deltas)), key=deltas.__getitem__)
            if deltas[index] > self.direct_transfer_max_joint_delta:
                raise ValueError(
                    f'SDK fallback exceeds configured joint excursion limit: '
                    f'{self.arm_joint_names[index]} delta={deltas[index]:.6f} rad '
                    f'({math.degrees(deltas[index]):.1f} deg), '
                    f'limit={self.direct_transfer_max_joint_delta:.6f} rad, '
                    f'start={self.alternative_seed[index]:.6f}, '
                    f'raw_goal={raw_goal[index]:.6f}, selected_goal={goal[index]:.6f}; '
                    'no nearer in-limit 2*pi-equivalent angle')
            self._alternative_segment_ready(joint_line(
                self.alternative_seed, goal, self.arm_joint_names, .3, .5))
        except Exception as exc:
            self._fault(f'SDK fallback planning failed: {exc}; item remains held')

    def _sdk_validate_trajectory(self, trajectory):
        # Discard any previous JTC spline/FK recording before SDK joint-line
        # validation. The SDK flag is later cleared during restoration, so it
        # cannot by itself identify which backend actually executed the path.
        self._begin_reuse_recording()
        self.sdk_commands, self.transport_checks = sdk_joint_samples(
            trajectory, self.arm_joint_names, self.transport_joint_limits)
        if max(abs(a-b) for a, b in zip(self.transport_checks[0][0], self.transport_seed)) > .001:
            raise ValueError('SDK path does not start at measured joints')
        self.transport_check_index = 0
        self.transport_descent_time = None
        self.state = self.TRANSPORT_VALIDATING
        self.get_logger().info(f'checking {len(self.transport_checks)} SDK joint-line states before mode handoff')
        self._transport_validate_next()

    def _sdk_guard(self, callback):
        operation, generation = self.operation_id, self.transport_route_generation
        return lambda future: callback(future) if (
            self.transport_sdk_active and self.state != self.FAULT and
            self.operation_id == operation and self.transport_route_generation == generation) else None

    def _sdk_begin_execution(self):
        if not self.enable_client.service_is_ready():
            self._fault('cannot pause Servo for SDK fallback')
            return
        self.transport_sdk_active = True
        self.sdk_phase = 'handoff'
        self.sdk_deadline = time.monotonic() + 180.
        self.sdk_stage_started = time.monotonic()
        self.sdk_force_baseline = self.latest_force_z
        self.sdk_previous = tuple(self.transport_seed)
        self.sdk_command_index = 0
        self.sdk_command_pending = False
        self.sdk_acknowledged = False
        self.get_logger().warning(
            f'SDK final fallback: {len(self.sdk_commands)} checked rest-to-rest joint commands; '
            'pausing Servo and acquiring exclusive mode-0 ownership')
        self.direct_target_z = float(self.transport_end[2])
        self.state = self.DISABLING_SERVO
        request = SetBool.Request(data=False)
        self.enable_client.call_async(request).add_done_callback(self._sdk_guard(self._sdk_paused))

    def _sdk_paused(self, future):
        try:
            result = future.result()
            if result is None or not result.success:
                raise ValueError('Servo pause was not confirmed')
            # Shared lifecycle handoff: controllers/hardware inactive, confirmed
            # mode 0, then _send_direct_retreat dispatches to _sdk_send_next.
            self._begin_direct_retreat()
        except Exception as exc:
            self._fault(f'SDK Servo pause failed: {exc}')

    def _sdk_send_next(self):
        try:
            if self._sdk_health_issue(time.monotonic()):
                raise ValueError('SDK prerequisites changed before command')
            if self.robot_mode != 0 or self.robot_error != 0 or self.robot_state not in (0, 2):
                raise ValueError('direct-mode ownership is not confirmed')
            if self.sdk_command_index == len(self.sdk_commands):
                self.sdk_phase = 'restore'
                self.sdk_stage_started = time.monotonic()
                self._restore_ros2_control_mode()
                return
            actual = getattr(self, 'sdk_report_joints', None)
            if actual is None or max(abs(a-b) for a, b in zip(actual, self.sdk_previous)) > .01:
                raise ValueError('robot moved away from the checked SDK segment start')
            if not self.sdk_joint_client.service_is_ready():
                raise ValueError('SDK joint service unavailable')
            self.sdk_goal = self.sdk_commands[self.sdk_command_index]
            scale = max(.05, min(1., self.motion_speed_percent/100.))
            request = MoveJoint.Request()
            request.angles = list(self.sdk_goal)
            request.speed = min(.3, self.direct_transfer_max_joint_speed*scale,
                                min(v[2] for v in self.transport_joint_limits.values())*scale)
            request.acc = min(.5, self.direct_transfer_joint_acc*scale)
            request.wait, request.relative, request.radius = False, False, -1.
            request.timeout = 30.
            self.state = self.RETREATING
            self.sdk_phase = 'moving'
            self.sdk_stage_started = time.monotonic()
            self.sdk_command_pending = True
            self.sdk_acknowledged = False
            self.direct_transfer_motion_started = True
            self.sdk_joint_client.call_async(request).add_done_callback(self._sdk_guard(self._sdk_command_received))
        except Exception as exc:
            self._fault(f'SDK fallback blocked: {exc}')

    def _sdk_command_received(self, future):
        try:
            result = future.result()
            if result is None or result.ret != 0:
                raise ValueError(f'joint command rejected: ret={None if result is None else result.ret}')
            self.sdk_command_pending = False
            self.sdk_acknowledged = True
            self.sdk_ack_time = time.monotonic()
        except Exception as exc:
            self._fault(f'SDK fallback command failed: {exc}')

    def _sdk_health_issue(self, now):
        geometry_keys = ('attached_item_id', 'attached_item_size_m',
                         'attached_item_center_in_tcp_m', 'attached_item_orientation_in_tcp_xyzw')
        return (now > self.sdk_deadline or now-self.sdk_stage_started > 30. or
                self.robot_error != 0 or self.robot_state_time is None or
                now-self.robot_state_time > self.status_timeout or
                self.last_force_time is None or now-self.last_force_time > self.force_timeout or
                not math.isfinite(self.latest_force_z) or
                not math.isfinite(self.sdk_force_baseline) or
                abs(self.latest_force_z-self.sdk_force_baseline) >= self.place_force_threshold or
                getattr(self, 'ft_recovery_required', False) or
                not self.pallet_locked or
                self.motion_status.get('operation_id') != self.transport_target.get('operation_id') or
                self.planning_scene_status.get('attachment_pending', False) or
                any(self.planning_scene_status.get(key) != self.transport_scene.get(key) for key in geometry_keys))

    def _sdk_transport_tick(self):
        if not getattr(self, 'transport_sdk_active', False):
            return False
        now = time.monotonic()
        if self._sdk_health_issue(now):
            self._fault('SDK fallback timeout, changed scene/target, force contact or unhealthy feedback; item remains held')
            return True
        if self.sdk_phase != 'moving':
            return False  # existing handoff/restore timers retain ownership
        if self.robot_mode != 0 or self.robot_state not in (0, 1, 2):
            self._fault('SDK fallback lost direct-mode readiness')
            return True
        actual = getattr(self, 'sdk_report_joints', None)
        if actual is None:
            self._fault('SDK joint telemetry is missing')
            return True
        a, b, p = map(np.asarray, (self.sdk_previous, self.sdk_goal, actual))
        delta = b-a
        u = float(np.clip((p-a)@delta/max(float(delta@delta), 1e-12), 0., 1.))
        if np.max(np.abs(p-(a+u*delta))) > .01:
            self._fault('SDK motion deviated from its checked joint segment')
            return True
        if (self.sdk_acknowledged and self.robot_state_time > self.sdk_ack_time and
                self.robot_state in (0, 2) and np.max(np.abs(p-b)) <= .003):
            self.sdk_previous = self.sdk_goal
            self.sdk_command_index += 1
            self._sdk_send_next()
        return True

    def _sdk_restored(self):
        # Called only after the existing mode-1/fresh-joints/settle gate succeeds.
        actual = self.latest_joint_positions
        if (actual is None or len(actual) != len(self.sdk_previous) or
                max(abs(a-b) for a, b in zip(actual, self.sdk_previous)) > .01 or
                any(not self.transport_joint_limits[n][0] <= v <= self.transport_joint_limits[n][1]
                    for n, v in zip(self.arm_joint_names, actual))):
            self._fault('SDK fallback final joint feedback mismatch after ROS restore')
            return
        self.transport_sdk_active = False
        self.transport_sdk_candidate = False
        self.get_logger().info('SDK fallback finished; ROS control restored and settled; verifying fresh endpoint FK')
        self._transport_result(SimpleNamespace(result=lambda: SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED, result=SimpleNamespace(error_code=0))))
