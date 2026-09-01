import json
import math
import time
from typing import List, Optional

from geometry_msgs.msg import PoseStamped, WrenchStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker
from xarm.wrapper import XArmAPI


class SafeServoController(Node):
    """Guarded Cartesian servo controller for UFACTORY robots."""

    IDLE = 'IDLE'
    ARMED = 'ARMED'
    RUNNING = 'RUNNING'
    FORCE_PAUSED = 'FORCE_PAUSED'
    TOUCH_GRASPED = 'TOUCH_GRASPED'
    FAULT = 'FAULT'

    def __init__(self):
        super().__init__('safe_servo')

        self.declare_parameter('robot_ip', '192.168.1.232')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('pause_robot_on_fault', True)
        self.declare_parameter('command_topic', '/servo_command')
        self.declare_parameter('force_topic', '/ufactory/uf_ftsensor_ext_states')
        self.declare_parameter('control_period', 0.02)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('force_timeout_sec', 0.5)
        self.declare_parameter('speed_scale', 0.2)

        self.declare_parameter('kp', 0.25)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('derivative_alpha', 0.2)
        self.declare_parameter('max_step_x', 3.0)
        self.declare_parameter('max_step_y', 3.0)
        self.declare_parameter('max_step_z', 1.0)
        self.declare_parameter('max_step_rpy', 2.0)
        self.declare_parameter('max_acceleration_xy', 60.0)
        self.declare_parameter('max_acceleration_z', 30.0)
        self.declare_parameter('max_acceleration_yaw', 40.0)
        self.declare_parameter('sdk_speed', 50.0)
        self.declare_parameter('sdk_acceleration', 100.0)
        self.declare_parameter('target_tolerance_mm', 0.25)

        self.declare_parameter('x_min', 160.0)
        self.declare_parameter('x_max', 390.0)
        self.declare_parameter('y_min', -360.0)
        self.declare_parameter('y_max', 360.0)
        self.declare_parameter('z_min', 50.0)
        self.declare_parameter('z_max', 80.0)
        self.declare_parameter('force_limit', 15.0)
        self.declare_parameter('force_limit_x', 15.0)
        self.declare_parameter('force_limit_y', 15.0)
        self.declare_parameter('force_limit_z', 15.0)
        self.declare_parameter('torque_limit', 3.0)
        self.declare_parameter('force_filter_alpha', 0.2)
        self.declare_parameter('over_limit_samples', 3)
        self.declare_parameter(
            'dry_run_start_pose', [250.0, 0.0, 65.0, 180.0, 0.0, 0.0]
        )

        def p(name):
            return self.get_parameter(name).value
        self.robot_ip = str(p('robot_ip'))
        self.dry_run = bool(p('dry_run'))
        self.pause_robot_on_fault = bool(p('pause_robot_on_fault'))
        self.control_period = float(p('control_period'))
        self.command_timeout_sec = float(p('command_timeout_sec'))
        self.force_timeout_sec = float(p('force_timeout_sec'))
        self.speed_scale = float(p('speed_scale'))
        self.kp = float(p('kp'))
        self.kd = float(p('kd'))
        self.derivative_alpha = float(p('derivative_alpha'))
        self.max_steps = [
            float(p('max_step_x')), float(p('max_step_y')),
            float(p('max_step_z')), float(p('max_step_rpy')),
            float(p('max_step_rpy')), float(p('max_step_rpy')),
        ]
        self.max_accelerations = [
            float(p('max_acceleration_xy')), float(p('max_acceleration_xy')),
            float(p('max_acceleration_z')), 0.0, 0.0,
            float(p('max_acceleration_yaw')),
        ]
        self.sdk_speed = float(p('sdk_speed'))
        self.sdk_acceleration = float(p('sdk_acceleration'))
        self.target_tolerance_mm = float(p('target_tolerance_mm'))
        self.bounds = [
            (float(p('x_min')), float(p('x_max'))),
            (float(p('y_min')), float(p('y_max'))),
            (float(p('z_min')), float(p('z_max'))),
        ]
        self.force_limits = [
            float(p('force_limit_x')), float(p('force_limit_y')),
            float(p('force_limit_z')),
        ]
        self.force_limit = float(p('force_limit'))
        self.torque_limit = float(p('torque_limit'))
        self.force_filter_alpha = float(p('force_filter_alpha'))
        self.over_limit_samples = int(p('over_limit_samples'))
        self.simulated_pose = [float(v) for v in p('dry_run_start_pose')]
        self._validate_parameters()

        self.arm = None
        self.enabled = False
        self.state = self.IDLE
        self.fault_reason = ''
        self.target_pose: Optional[List[float]] = None
        self.target_settled = False
        self.proposed_pose: Optional[List[float]] = None
        self.locked_roll_pitch: Optional[List[float]] = None
        self.last_command_time: Optional[float] = None
        self.last_force_time: Optional[float] = None
        self.filtered_wrench: Optional[List[float]] = None
        self.raw_wrench: Optional[List[float]] = None
        self.over_limit_count = 0
        self.under_limit_count = 0
        self.force_paused = False
        self.touch_to_grasp_mode = False
        self.touch_descent_active = False
        self.touch_locked_pose: Optional[List[float]] = None
        self.prev_error = [0.0, 0.0]
        self.filtered_derivative = [0.0, 0.0]
        self.command_velocity = [0.0] * 6
        self.last_control_time = time.monotonic()

        self.create_subscription(
            Float64MultiArray, str(p('command_topic')), self.command_callback, 10
        )
        self.create_subscription(
            WrenchStamped, str(p('force_topic')), self.force_callback, 10
        )
        self.create_subscription(
            Float64MultiArray, '/safe_servo/config', self.config_callback, 10
        )
        self.status_pub = self.create_publisher(String, '/safe_servo/status', 10)
        self.fault_pub = self.create_publisher(String, '/safe_servo/fault', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/safe_servo/proposed_pose', 10
        )
        self.workspace_pub = self.create_publisher(
            Marker, '/safe_servo/workspace', 1
        )
        self.create_service(SetBool, '/safe_servo/enable', self.enable_callback)
        self.create_service(Trigger, '/safe_servo/reset_fault', self.reset_callback)
        self.create_timer(self.control_period, self.control_loop)
        self.create_timer(0.5, self.publish_status)
        self.create_timer(1.0, self.publish_workspace)

        self.get_logger().info(
            f'safe_servo ready (dry_run={self.dry_run}, enabled=False, '
            f'robot_ip={self.robot_ip})'
        )

    def _validate_parameters(self):
        if self.control_period <= 0.0:
            raise ValueError('control_period must be positive')
        if self.command_timeout_sec <= 0.0 or self.force_timeout_sec <= 0.0:
            raise ValueError('watchdog timeouts must be positive')
        if not 0.0 <= self.derivative_alpha <= 1.0:
            raise ValueError('derivative_alpha must be in [0, 1]')
        if not 0.0 < self.force_filter_alpha <= 1.0:
            raise ValueError('force_filter_alpha must be in (0, 1]')
        if self.over_limit_samples < 1:
            raise ValueError('over_limit_samples must be at least 1')
        if not 0.01 <= self.speed_scale <= 1.0:
            raise ValueError('speed_scale must be in [0.01, 1.0]')
        if any(value <= 0.0 for value in (
            self.max_accelerations[0], self.max_accelerations[2],
            self.max_accelerations[5], self.sdk_speed, self.sdk_acceleration,
        )):
            raise ValueError('acceleration and SDK motion limits must be positive')
        if len(self.simulated_pose) != 6:
            raise ValueError('dry_run_start_pose must contain 6 values')
        for lower, upper in self.bounds:
            if lower >= upper:
                raise ValueError('workspace minimum must be below maximum')

    @staticmethod
    def clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _workspace_violation(self, pose, z_only=False) -> Optional[str]:
        axes = ('X', 'Y', 'Z')
        violations = [
            f'{axes[i]}={pose[i]:.1f} not in [{lower:.1f}, {upper:.1f}]'
            for i, (lower, upper) in enumerate(self.bounds)
            if (not z_only or i == 2)
            if pose[i] < lower or pose[i] > upper
        ]
        return ', '.join(violations) if violations else None

    def command_callback(self, msg: Float64MultiArray):
        if self.touch_to_grasp_mode and not self.enabled:
            # The panel may still publish briefly after contact. Never re-arm
            # or replace a completed touch target without an explicit enable.
            return
        if len(msg.data) != 6 or not all(math.isfinite(v) for v in msg.data):
            self.latch_fault('invalid target: expected 6 finite values')
            return
        target = [float(v) for v in msg.data]
        locked = self.locked_roll_pitch or self.simulated_pose[3:5]
        violation = self._workspace_violation(
            target, z_only=self.touch_to_grasp_mode)
        if violation:
            self.latch_fault(f'target outside workspace; command rejected: {violation}')
            return
        if self.touch_to_grasp_mode:
            if self.touch_locked_pose is None:
                self.latch_fault('touch mode has no locked enable-time pose')
                return
            bounded = self.touch_locked_pose.copy()
            bounded[2] = target[2]
            self.touch_descent_active = target[2] < self.touch_locked_pose[2] - 0.5
            if not self.touch_descent_active:
                self.latch_fault(
                    'touch-to-grasp target must be below the enable-time Z pose')
                return
        else:
            bounded = target[:3] + locked.copy() + [target[5]]
        if target[3:5] != locked:
            self.get_logger().warning(
                'roll/pitch command values are ignored; holding the '
                f'enable-time orientation {locked} degrees'
            )
        target_changed = self.target_pose is None or any(
            abs(bounded[i] - self.target_pose[i]) > 1e-6
            for i in (0, 1, 2, 5)
        )
        self.target_pose = bounded
        if target_changed:
            self.target_settled = False
        self.last_command_time = time.monotonic()

    def force_callback(self, msg: WrenchStamped):
        sample = [
            float(msg.wrench.force.x), float(msg.wrench.force.y),
            float(msg.wrench.force.z), float(msg.wrench.torque.x),
            float(msg.wrench.torque.y), float(msg.wrench.torque.z),
        ]
        if not all(math.isfinite(v) for v in sample):
            self.latch_fault('non-finite force/torque sample')
            return
        self.raw_wrench = sample
        if self.filtered_wrench is None:
            self.filtered_wrench = sample
        else:
            a = self.force_filter_alpha
            self.filtered_wrench = [
                a * new + (1.0 - a) * old
                for new, old in zip(sample, self.filtered_wrench)
            ]
        self.last_force_time = time.monotonic()

        raw_force = sample[:3]
        force = self.filtered_wrench[:3]
        torque = self.filtered_wrench[3:]
        raw_force_norm = math.sqrt(sum(v * v for v in raw_force))
        force_norm = math.sqrt(sum(v * v for v in force))
        instant_force_over_limit = (
            raw_force_norm >= self.force_limit
            or any(
                abs(v) >= limit
                for v, limit in zip(raw_force, self.force_limits)
            )
        )
        force_over_limit = (
            force_norm >= self.force_limit
            or any(abs(v) >= limit for v, limit in zip(force, self.force_limits))
        )
        if any(abs(v) >= self.torque_limit for v in torque):
            self.latch_fault(
                f'torque limit exceeded: force={force}, torque={torque}'
            )
            return

        self.over_limit_count = self.over_limit_count + 1 if force_over_limit else 0
        clear_limit = 0.8 * self.force_limit
        force_clear = (
            force_norm < clear_limit
            and all(abs(v) < 0.8 * limit for v, limit in zip(force, self.force_limits))
        )
        self.under_limit_count = self.under_limit_count + 1 if force_clear else 0
        if (
            self.enabled
            and not self.force_paused
            and (instant_force_over_limit or self.over_limit_count >= self.over_limit_samples)
        ):
            self.force_paused = True
            self.command_velocity = [0.0] * 6
            if self.touch_to_grasp_mode and self.touch_descent_active:
                self._complete_touch_grasp(raw_force, force)
                return
            self.state = self.FORCE_PAUSED
            self.get_logger().warning(
                f'external force hold: raw={raw_force}, filtered={force}, '
                f'limit={self.force_limit} N'
            )
        elif (
            not self.touch_to_grasp_mode and self.force_paused
            and self.under_limit_count >= self.over_limit_samples
        ):
            self.force_paused = False
            self.over_limit_count = 0
            self.state = self.ARMED if self.enabled else self.IDLE
            self.get_logger().info('external force cleared; resuming target motion')

    def config_callback(self, msg: Float64MultiArray):
        """Apply speed, workspace, and optional force threshold atomically."""
        if self.enabled:
            self.get_logger().error(
                'configuration rejected: disable motion before changing safety limits'
            )
            return
        if len(msg.data) not in (7, 8, 9) or not all(
            math.isfinite(v) for v in msg.data
        ):
            self.get_logger().error('invalid config: expected 7 or 8 finite values')
            return
        speed = float(msg.data[0])
        new_bounds = [
            (float(msg.data[1]), float(msg.data[2])),
            (float(msg.data[3]), float(msg.data[4])),
            (float(msg.data[5]), float(msg.data[6])),
        ]
        force_threshold = float(msg.data[7]) if len(msg.data) == 8 else self.force_limit
        if len(msg.data) >= 9:
            force_threshold = float(msg.data[7])
        touch_to_grasp_mode = bool(round(msg.data[8])) if len(msg.data) >= 9 else False
        if (
            not 0.01 <= speed <= 1.0
            or any(lo >= hi for lo, hi in new_bounds)
            or force_threshold <= 0.0
        ):
            self.get_logger().error(
                'config rejected: invalid speed, workspace, or force threshold'
            )
            return
        self.speed_scale = speed
        self.bounds = new_bounds
        self.force_limit = force_threshold
        self.force_limits = [force_threshold] * 3
        self.touch_to_grasp_mode = touch_to_grasp_mode
        self.touch_descent_active = False
        self.get_logger().info(
            f'configuration applied: speed={speed:.0%}, bounds={new_bounds}, '
            f'force_threshold={force_threshold:.2f} N, '
            f'touch_to_grasp={touch_to_grasp_mode}'
        )
        self.publish_workspace()

    def enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            self.enabled = False
            self.touch_descent_active = False
            if self.touch_to_grasp_mode:
                self.target_pose = None
            self.command_velocity = [0.0] * 6
            self.state = self.IDLE if not self.fault_reason else self.FAULT
            self._pause_robot('controller disabled')
            response.success = True
            response.message = 'safe servo disabled'
            return response
        if self.fault_reason:
            response.success = False
            response.message = f'reset fault first: {self.fault_reason}'
            return response
        if not self.dry_run and not self._initialize_robot():
            response.success = False
            response.message = self.fault_reason
            return response
        initial_pose = self.simulated_pose.copy() if self.dry_run else self._read_pose()
        if initial_pose is None:
            response.success = False
            response.message = self.fault_reason or 'failed to read initial pose'
            return response
        violation = self._workspace_violation(
            initial_pose, z_only=self.touch_to_grasp_mode)
        if violation:
            self.latch_fault(
                f'current pose outside workspace; execution rejected: {violation}'
            )
            response.success = False
            response.message = self.fault_reason
            return response
        self.locked_roll_pitch = initial_pose[3:5]
        self.touch_locked_pose = initial_pose.copy() if self.touch_to_grasp_mode else None
        if self.touch_to_grasp_mode and self.filtered_wrench is not None:
            force_norm = math.sqrt(sum(v * v for v in self.filtered_wrench[:3]))
            if force_norm >= 0.8 * self.force_limit:
                response.success = False
                response.message = (
                    f'force must be clear before touch descent: {force_norm:.2f} N')
                return response
        self.command_velocity = [0.0] * 6
        self.prev_error = [0.0, 0.0]
        self.filtered_derivative = [0.0, 0.0]
        self.enabled = True
        self.force_paused = False
        self.under_limit_count = 0
        self.state = self.ARMED
        self.target_pose = None
        self.target_settled = False
        self.last_command_time = None
        response.success = True
        response.message = (
            ('touch-to-grasp armed; holding X/Y/orientation at '
             f'{self.touch_locked_pose}') if self.touch_to_grasp_mode else
            ('safe servo enabled; holding roll/pitch at '
             f'{self.locked_roll_pitch} degrees'))
        return response

    def reset_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self.enabled = False
        self.fault_reason = ''
        self.state = self.IDLE
        self.over_limit_count = 0
        self.under_limit_count = 0
        self.force_paused = False
        self.target_pose = None
        self.target_settled = False
        self.locked_roll_pitch = None
        self.touch_locked_pose = None
        self.touch_descent_active = False
        self.command_velocity = [0.0] * 6
        self.last_command_time = None
        response.success = True
        response.message = 'fault reset; controller remains disabled'
        return response

    def _initialize_robot(self) -> bool:
        try:
            if self.arm is None:
                self.arm = XArmAPI(self.robot_ip)
            if not bool(self.arm.connected):
                self.latch_fault(f'cannot connect to robot at {self.robot_ip}')
                return False
            for name, result in [
                ('motion_enable', self.arm.motion_enable(enable=True)),
                ('set_mode(1)', self.arm.set_mode(1)),
                ('set_state(0)', self.arm.set_state(0)),
            ]:
                if self._return_code(result) != 0:
                    self.latch_fault(f'{name} failed: {result}')
                    return False
            time.sleep(0.1)
            return True
        except Exception as exc:
            self.latch_fault(f'robot initialization exception: {exc}')
            return False

    @staticmethod
    def _return_code(result) -> int:
        if isinstance(result, (tuple, list)):
            return int(result[0])
        return int(result)

    def latch_fault(self, reason: str):
        if self.fault_reason:
            return
        self.fault_reason = reason
        self.enabled = False
        self.command_velocity = [0.0] * 6
        self.state = self.FAULT
        self.get_logger().error(f'FAULT: {reason}')
        msg = String()
        msg.data = reason
        self.fault_pub.publish(msg)
        self._pause_robot(reason)

    def _pause_robot(self, reason: str):
        if self.dry_run or self.arm is None or not self.pause_robot_on_fault:
            return
        try:
            result = self.arm.set_state(3)
            if self._return_code(result) != 0:
                self.get_logger().error(f'failed to pause robot ({reason}): {result}')
        except Exception as exc:
            self.get_logger().error(f'pause robot exception ({reason}): {exc}')

    def _complete_touch_grasp(self, raw_force, filtered_force):
        """Latch contact, halt Z motion, and enable the vacuum output once."""
        self.enabled = False
        self.touch_descent_active = False
        self.target_pose = None
        self.state = self.TOUCH_GRASPED
        self._pause_robot('touch force threshold reached')
        if self.dry_run:
            self.get_logger().info(
                'dry-run touch threshold reached; vacuum command simulated')
            return
        try:
            result = self.arm.set_vacuum_gripper(
                True, wait=False, timeout=3, sync=False)
            if self._return_code(result) != 0:
                self.latch_fault(f'vacuum gripper enable failed: {result}')
                return
            self.get_logger().info(
                f'touch grasp complete: raw_force={raw_force}, '
                f'filtered_force={filtered_force}; vacuum enabled')
        except Exception as exc:
            self.latch_fault(f'vacuum gripper exception: {exc}')

    def control_loop(self):
        now = time.monotonic()
        dt = max(now - self.last_control_time, 1e-6)
        self.last_control_time = now
        if not self.enabled or self.fault_reason or self.target_pose is None:
            return
        if (self.last_command_time is None or
                now - self.last_command_time > self.command_timeout_sec):
            self.latch_fault('command watchdog timeout')
            return
        if self.last_force_time is None or now - self.last_force_time > self.force_timeout_sec:
            self.latch_fault('force watchdog timeout')
            return
        if self.force_paused:
            self.command_velocity = [0.0] * 6
            self.state = self.FORCE_PAUSED
            return

        current = self.simulated_pose.copy() if self.dry_run else self._read_pose()
        if current is None:
            return
        proposed = self._compute_next_pose(current, self.target_pose, dt)
        violation = self._workspace_violation(
            proposed, z_only=self.touch_to_grasp_mode)
        if violation:
            self.latch_fault(
                f'proposed pose outside workspace; command rejected: {violation}'
            )
            return
        self.proposed_pose = proposed
        self._publish_pose(proposed)

        if self.dry_run:
            self.simulated_pose = proposed
        else:
            try:
                result = self.arm.set_servo_cartesian(
                    proposed, speed=self.sdk_speed, mvacc=self.sdk_acceleration
                )
                if self._return_code(result) != 0:
                    self.latch_fault(f'set_servo_cartesian failed: {result}')
                    return
            except Exception as exc:
                self.latch_fault(f'servo command exception: {exc}')
                return
        self.state = self.RUNNING

    def _read_pose(self) -> Optional[List[float]]:
        try:
            result = self.arm.get_position(is_radian=False)
            if (
                isinstance(result, (tuple, list)) and len(result) >= 2
                and self._return_code(result) == 0 and len(result[1]) >= 6
            ):
                return [float(v) for v in result[1][:6]]
            self.latch_fault(f'get_position failed: {result}')
        except Exception as exc:
            self.latch_fault(f'get_position exception: {exc}')
        return None

    def _compute_next_pose(self, current, target, dt):
        error = [target[i] - current[i] for i in range(6)]
        # Roll and pitch are deliberately absent from the controller error.
        error[3] = 0.0
        error[4] = 0.0
        # Follow the shortest yaw path across the +/-180 degree boundary.
        error[5] = (error[5] + 180.0) % 360.0 - 180.0
        controlled_axes = (0, 1, 2, 5)
        if self.target_settled or all(
            abs(error[i]) <= self.target_tolerance_mm for i in controlled_axes
        ):
            self.target_settled = True
            self.command_velocity = [0.0] * 6
            held = target.copy()
            held[3:5] = self.locked_roll_pitch or current[3:5]
            return held
        raw_derivative = [
            (error[i] - self.prev_error[i]) / dt for i in range(2)
        ]
        a = self.derivative_alpha
        self.filtered_derivative = [
            a * raw + (1.0 - a) * old
            for raw, old in zip(raw_derivative, self.filtered_derivative)
        ]
        output = [
            self.kp * error[i] + self.kd * self.filtered_derivative[i]
            for i in range(2)
        ] + [error[2], 0.0, 0.0, error[5]]
        self.prev_error = error[:2]
        desired_step = [
            self.clamp(
                output[i],
                -self.max_steps[i] * self.speed_scale,
                self.max_steps[i] * self.speed_scale,
            )
            for i in range(6)
        ]
        desired_velocity = [step / dt for step in desired_step]
        for i in (0, 1, 2, 5):
            max_delta = self.max_accelerations[i] * dt
            self.command_velocity[i] += self.clamp(
                desired_velocity[i] - self.command_velocity[i],
                -max_delta,
                max_delta,
            )
        proposed = [
            current[i] + self.command_velocity[i] * dt for i in range(6)
        ]
        # Never let a discrete control step cross to the other side of the
        # target.  Crossing plus delayed pose feedback causes limit cycles.
        for i in controlled_axes:
            step = proposed[i] - current[i]
            if step * error[i] > 0.0 and abs(step) >= abs(error[i]):
                proposed[i] = target[i]
                self.command_velocity[i] = 0.0
        proposed[3:5] = self.locked_roll_pitch or current[3:5]
        return proposed

    def _publish_pose(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ufactory_base'
        msg.pose.position.x = pose[0] / 1000.0
        msg.pose.position.y = pose[1] / 1000.0
        msg.pose.position.z = pose[2] / 1000.0
        roll, pitch, yaw = [math.radians(v) for v in pose[3:]]
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        self.pose_pub.publish(msg)

    def publish_status(self):
        msg = String()
        force = self.filtered_wrench[:3] if self.filtered_wrench else None
        raw_force = self.raw_wrench[:3] if self.raw_wrench else None
        msg.data = json.dumps({
            'state': self.state,
            'enabled': self.enabled,
            'dry_run': self.dry_run,
            'fault': self.fault_reason,
            'target_mm_deg': self.target_pose,
            'target_settled': self.target_settled,
            'proposed_mm_deg': self.proposed_pose,
            'filtered_force_n': force,
            'raw_force_n': raw_force,
            'force_threshold_n': self.force_limit,
            'force_paused': self.force_paused,
            'touch_to_grasp_mode': self.touch_to_grasp_mode,
            'touch_descent_active': self.touch_descent_active,
            'speed_scale': self.speed_scale,
            'bounds_mm': self.bounds,
            'locked_roll_pitch_deg': self.locked_roll_pitch,
            'command_velocity_mm_deg_s': self.command_velocity,
        })
        self.status_pub.publish(msg)

    def publish_workspace(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'ufactory_base'
        marker.ns = 'safe_servo'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (self.bounds[0][0] + self.bounds[0][1]) / 2000.0
        marker.pose.position.y = (self.bounds[1][0] + self.bounds[1][1]) / 2000.0
        marker.pose.position.z = (self.bounds[2][0] + self.bounds[2][1]) / 2000.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = (self.bounds[0][1] - self.bounds[0][0]) / 1000.0
        marker.scale.y = (self.bounds[1][1] - self.bounds[1][0]) / 1000.0
        marker.scale.z = (self.bounds[2][1] - self.bounds[2][0]) / 1000.0
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.18
        self.workspace_pub.publish(marker)

    def shutdown(self):
        self.enabled = False
        self._pause_robot('node shutdown')
        if self.arm is not None:
            try:
                self.arm.disconnect()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = SafeServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
