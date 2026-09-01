import json
import math
import time

from action_msgs.msg import GoalStatusArray
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from xarm_msgs.srv import SetInt16


class MoveItServoBridge(Node):
    """Guarded vertical-only command bridge for MoveIt Servo."""

    IDLE = 'IDLE'
    ARMED = 'ARMED'
    RUNNING = 'RUNNING'
    FAULT = 'FAULT'

    def __init__(self):
        super().__init__('safe_servo')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('planning_frame', 'link_base')
        self.declare_parameter('ee_frame', 'link_tcp')
        self.declare_parameter('control_period', 0.02)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('tf_timeout_sec', 0.25)
        self.declare_parameter('max_linear_speed', 0.03)
        self.declare_parameter('kp_z', 3.0)
        self.declare_parameter('target_tolerance_m', 0.0005)
        self.declare_parameter('max_target_delta_m', 0.01)
        self.declare_parameter('z_min', 0.05)
        self.declare_parameter('z_max', 0.80)
        self.declare_parameter('force_topic', '/ufactory/uf_ftsensor_ext_states')
        self.declare_parameter('force_timeout_sec', 0.6)
        self.declare_parameter('force_limit_n', 5.0)
        self.declare_parameter('torque_limit_nm', 0.5)
        self.declare_parameter('trajectory_controller', 'uf850_traj_controller')
        self.declare_parameter('joint_state_broadcaster', 'joint_state_broadcaster')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_state_ready_timeout_sec', 5.0)
        self.declare_parameter('joint_state_ready_samples', 3)
        self.declare_parameter('robot_servo_mode', 1)
        self.declare_parameter('robot_servo_state', 0)

        def p(name):
            return self.get_parameter(name).value
        self.dry_run = bool(p('dry_run'))
        self.planning_frame = str(p('planning_frame'))
        self.ee_frame = str(p('ee_frame'))
        self.period = float(p('control_period'))
        self.command_timeout = float(p('command_timeout_sec'))
        self.tf_timeout = float(p('tf_timeout_sec'))
        self.configured_max_speed = float(p('max_linear_speed'))
        self.max_speed = self.configured_max_speed
        self.kp_z = float(p('kp_z'))
        self.tolerance = float(p('target_tolerance_m'))
        self.max_target_delta = float(p('max_target_delta_m'))
        self.z_bounds = [float(p('z_min')), float(p('z_max'))]
        self.force_topic = str(p('force_topic'))
        self.force_timeout = float(p('force_timeout_sec'))
        self.force_limit = float(p('force_limit_n'))
        self.torque_limit = float(p('torque_limit_nm'))
        self.trajectory_controller = str(p('trajectory_controller'))
        self.joint_state_broadcaster = str(p('joint_state_broadcaster'))
        self.joint_state_topic = str(p('joint_state_topic'))
        self.joint_state_ready_timeout = float(p('joint_state_ready_timeout_sec'))
        self.joint_state_ready_samples = int(p('joint_state_ready_samples'))
        self.robot_servo_mode = int(p('robot_servo_mode'))
        self.robot_servo_state = int(p('robot_servo_state'))
        if (self.period <= 0 or self.command_timeout <= 0 or
                not 0.0 < self.configured_max_speed <= 0.05 or self.kp_z <= 0.0):
            raise ValueError(
                'period, timeout, kp_z, and max speed must be positive; '
                'max_linear_speed must not exceed 0.05 m/s')
        if self.z_bounds[0] >= self.z_bounds[1]:
            raise ValueError('z_min must be below z_max')
        if self.joint_state_ready_timeout <= 0.0:
            raise ValueError('joint_state_ready_timeout_sec must be positive')
        if self.joint_state_ready_samples < 1:
            raise ValueError('joint_state_ready_samples must be at least 1')

        self.enabled = False
        self.state = self.IDLE
        self.fault = ''
        self.target_z = None
        self.enable_z = None
        self.dry_run_z = None
        self.last_force_time = None
        self.force_norm = None
        self.torque_norm = None
        self.last_command_time = None
        self.motion_state = 'UNKNOWN'
        self.active_trajectory = False
        self.servo_status = None
        self.servo_pause_confirmed = False
        self.pause_request_pending = False
        self.servo_motion_ready = False
        self.touch_mode = False
        self.touch_descent = False
        self.touch_contact = False
        self.bypass_force_arm_check = False
        self.enable_force_baseline = None
        self.enable_generation = 0
        self.enable_wrench_baseline = None
        self.latest_wrench = None
        self.force_delta_norm = None
        self.force_delta_z = None
        self.trajectory_controller_active = False
        self.last_joint_state_time = None
        self.joint_state_sequence = 0
        self.ready_wait_sequence = None
        self.ready_wait_deadline = None
        self.ready_wait_callback = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.status_pub = self.create_publisher(String, '/safe_servo/status', 10)
        self.fault_pub = self.create_publisher(String, '/safe_servo/fault', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/safe_servo/proposed_pose', 10)
        self.create_subscription(
            Float64MultiArray, '/servo_command', self.command_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/safe_servo/config', self.config_callback, 10)
        self.create_subscription(
            WrenchStamped, self.force_topic, self.force_callback, 10)
        self.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 10)
        self.create_subscription(
            ServoStatus, '/servo_node/status', self.servo_status_callback, 10)
        self.create_subscription(
            String, '/motion_coordinator/status', self.motion_status_callback, 10)
        self.create_subscription(
            GoalStatusArray,
            '/uf850_traj_controller/follow_joint_trajectory/_action/status',
            self.trajectory_status_callback, 10)
        self.switch_client = self.create_client(
            ServoCommandType, '/servo_node/switch_command_type')
        self.pause_client = self.create_client(
            SetBool, '/servo_node/pause_servo')
        self.controller_switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.controller_list_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.set_mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
        self.set_state_client = self.create_client(SetInt16, '/ufactory/set_state')
        self.create_service(SetBool, '/safe_servo/enable', self.enable_callback)
        self.create_service(Trigger, '/safe_servo/reset_fault', self.reset_callback)
        self.create_timer(0.25, self.ensure_servo_starts_paused)
        self.create_timer(self.period, self.control_loop)
        self.create_timer(0.05, self.readiness_tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'MoveIt Servo bridge ready (dry_run={self.dry_run}, vertical-only)')

    def _current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, self.ee_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException as exc:
            self.latch_fault(f'TF unavailable: {exc}')
            return None
        stamp = transform.header.stamp
        age = self.get_clock().now() - rclpy.time.Time.from_msg(stamp)
        if age.nanoseconds * 1e-9 > self.tf_timeout:
            self.latch_fault('TCP transform is stale')
            return None
        return transform.transform.translation

    def config_callback(self, msg):
        if self.enabled:
            return
        if len(msg.data) == 1 and math.isfinite(msg.data[0]) and msg.data[0] > 0.0:
            self.force_limit = float(msg.data[0])
            self.touch_mode = True
            self.bypass_force_arm_check = False
            self.publish_status()
            return
        if len(msg.data) not in (7, 8, 9, 10) or not all(
                math.isfinite(value) for value in msg.data):
            return
        speed_scale = float(msg.data[0])
        z_min = float(msg.data[5]) / 1000.0
        z_max = float(msg.data[6]) / 1000.0
        force_limit = float(msg.data[7]) if len(msg.data) >= 8 else self.force_limit
        touch_mode = bool(round(msg.data[8])) if len(msg.data) >= 9 else False
        bypass_force = bool(round(msg.data[9])) if len(msg.data) >= 10 else False
        if (0.01 <= speed_scale <= 1.0 and z_min < z_max and
                force_limit > 0.0):
            # Keep the supervisor's scale relative to the configured bridge
            # speed.  The old hard-coded 0.01 silently reset every descent to
            # 10 mm/s even when max_linear_speed was raised at launch.
            self.max_speed = self.configured_max_speed * speed_scale
            self.z_bounds = [z_min, z_max]
            self.force_limit = force_limit
            self.touch_mode = touch_mode
            self.bypass_force_arm_check = bypass_force
            self.get_logger().info(
                f'guarded Servo config applied: speed={self.max_speed:.3f} m/s, '
                f'force_delta_fz={self.force_limit:.1f} N, '
                f'touch_mode={self.touch_mode}')
            self.publish_status()

    def _contact_delta_n(self):
        return self.force_limit

    def _force_safety_cap_n(self):
        return max(50.0, 3.0 * self.force_limit)

    def _update_wrench_delta(self, force, torque):
        self.latest_wrench = force + torque
        if self.enable_wrench_baseline is None:
            self.force_delta_norm = 0.0
            self.force_delta_z = 0.0
            return
        delta = [
            current - baseline
            for current, baseline in zip(
                self.latest_wrench, self.enable_wrench_baseline)]
        self.force_delta_norm = math.sqrt(
            sum(value * value for value in delta[:3]))
        self.force_delta_z = abs(delta[2])

    def force_callback(self, msg):
        force = (float(msg.wrench.force.x), float(msg.wrench.force.y),
                 float(msg.wrench.force.z))
        torque = (float(msg.wrench.torque.x), float(msg.wrench.torque.y),
                  float(msg.wrench.torque.z))
        if not all(math.isfinite(value) for value in force + torque):
            self.latch_fault('non-finite external wrench')
            return
        self.force_norm = math.sqrt(sum(value * value for value in force))
        self.torque_norm = math.sqrt(sum(value * value for value in torque))
        self._update_wrench_delta(force, torque)
        self.last_force_time = time.monotonic()
        if self.enabled and self.touch_mode:
            contact_delta = self._contact_delta_n()
            safety_cap = self._force_safety_cap_n()
            delta = self.force_delta_z or 0.0
            if self.touch_descent:
                if delta >= contact_delta:
                    self._latch_touch_contact()
                elif self.force_norm >= safety_cap or self.torque_norm >= self.torque_limit:
                    self.latch_fault(
                        f'external wrench safety limit: force={self.force_norm:.2f} N, '
                        f'delta_fz={delta:.2f} N, torque={self.torque_norm:.2f} Nm')
            return
        if self.enabled and (
                self.force_norm >= self.force_limit or
                self.torque_norm >= self.torque_limit):
            self.latch_fault(
                f'external wrench limit: force={self.force_norm:.2f} N, '
                f'torque={self.torque_norm:.2f} Nm')

    def joint_state_callback(self, _msg):
        self.last_joint_state_time = time.monotonic()
        self.joint_state_sequence += 1

    def _joint_states_fresh(self):
        return (
            self.last_joint_state_time is not None and
            time.monotonic() - self.last_joint_state_time <= 0.25)

    def _wait_for_fresh_joint_state(self, on_success):
        self.ready_wait_sequence = self.joint_state_sequence
        self.ready_wait_deadline = time.monotonic() + self.joint_state_ready_timeout
        self.ready_wait_callback = on_success
        self.get_logger().info(
            'controllers active; waiting for a post-switch /joint_states update')

    def readiness_tick(self):
        if self.ready_wait_callback is None:
            return
        if (self.ready_wait_sequence is not None and
                self.joint_state_sequence >= (
                    self.ready_wait_sequence + self.joint_state_ready_samples) and
                self.last_joint_state_time is not None and
                time.monotonic() - self.last_joint_state_time <= 0.25):
            callback = self.ready_wait_callback
            self.ready_wait_callback = None
            self.ready_wait_sequence = None
            self.ready_wait_deadline = None
            self.get_logger().info('fresh post-switch /joint_states confirmed')
            callback()
            return
        if (self.ready_wait_deadline is not None and
                time.monotonic() >= self.ready_wait_deadline):
            self.ready_wait_callback = None
            self.ready_wait_sequence = None
            self.ready_wait_deadline = None
            self.latch_fault(
                'controllers report active but /joint_states did not resume '
                f'within {self.joint_state_ready_timeout:.1f} s')

    def _current_z_soft(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, self.ee_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException:
            return None
        return float(transform.transform.translation.z)

    def command_callback(self, msg):
        if len(msg.data) != 6 or not all(math.isfinite(v) for v in msg.data):
            self.latch_fault('target must contain six finite values')
            return
        target_z = float(msg.data[2]) / 1000.0
        if not self.z_bounds[0] <= target_z <= self.z_bounds[1]:
            self.latch_fault(
                f'target Z {target_z:.4f} m outside {self.z_bounds}')
            return
        if self.touch_mode:
            if self.enable_z is not None and target_z >= self.enable_z - 1e-6:
                self.latch_fault(
                    'touch mode requires target Z below the enable-time pose')
                return
            current_z = self._current_z_soft()
            if current_z is None and self.enable_z is not None:
                current_z = self.enable_z
            if current_z is not None:
                floor_z = current_z - self.max_target_delta
                if target_z < floor_z:
                    target_z = floor_z
            self.touch_descent = True
        elif (self.enable_z is not None and
              abs(target_z - self.enable_z) > self.max_target_delta):
            self.latch_fault(
                f'target displacement {target_z - self.enable_z:+.4f} m exceeds '
                f'{self.max_target_delta:.4f} m per enable')
            return
        else:
            self.touch_descent = (
                self.enable_z is not None and target_z < self.enable_z - 1e-6)
        self.target_z = target_z
        if self.dry_run:
            self.dry_run_z = target_z
        self.last_command_time = time.monotonic()
        self.get_logger().info(f'accepted vertical target Z={target_z:.4f} m')

    def servo_status_callback(self, msg):
        self.servo_status = int(msg.code)
        if msg.code in (ServoStatus.HALT_FOR_SINGULARITY,
                        ServoStatus.HALT_FOR_COLLISION,
                        ServoStatus.JOINT_BOUND):
            self.latch_fault(f'MoveIt Servo halted: {msg.message}')

    def motion_status_callback(self, msg):
        try:
            self.motion_state = str(json.loads(msg.data).get('state', 'UNKNOWN'))
            if self.dry_run and self.motion_state in (
                    'PLANNING', 'EXECUTING', 'CANCELING'):
                self.dry_run_z = None
        except (ValueError, TypeError):
            self.motion_state = 'UNKNOWN'

    def trajectory_status_callback(self, msg):
        # action_msgs/GoalStatus: ACCEPTED, EXECUTING, CANCELING only.
        active_codes = {1, 2, 3}
        self.active_trajectory = any(
            status.status in active_codes for status in msg.status_list)

    def ensure_servo_starts_paused(self):
        if self.servo_pause_confirmed or self.pause_request_pending:
            return
        if not self.pause_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = True
        self.pause_request_pending = True
        future = self.pause_client.call_async(request)
        future.add_done_callback(self._startup_pause_completed)

    def _startup_pause_completed(self, future):
        self.pause_request_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'failed to pause Servo at startup: {exc}')
            return
        self.servo_pause_confirmed = bool(response and response.success)
        if not self.servo_pause_confirmed:
            self.get_logger().error('Servo startup pause was rejected')
        else:
            self.get_logger().info('MoveIt Servo startup pause confirmed')

    def _set_servo_paused(self, paused):
        if not self.pause_client.service_is_ready():
            return False
        request = SetBool.Request()
        request.data = paused
        self.pause_client.call_async(request)
        return True

    def _ros_set_mode(self, on_success):
        if not self.set_mode_client.service_is_ready():
            self.latch_fault('ufactory set_mode service is unavailable')
            return
        request = SetInt16.Request()
        request.data = int(self.robot_servo_mode)
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._ros_set_mode_completed(completed, on_success))

    def _ros_set_mode_completed(self, future, on_success):
        try:
            response = future.result()
        except Exception as exc:
            self.latch_fault(f'ufactory set_mode failed: {exc}')
            return
        if response is None or response.ret != 0:
            code = None if response is None else response.ret
            message = '' if response is None else response.message
            self.latch_fault(
                f'ufactory set_mode({self.robot_servo_mode}) failed: '
                f'ret={code} {message}')
            return
        on_success()

    def _ros_set_state(self, on_success):
        if not self.set_state_client.service_is_ready():
            self.latch_fault('ufactory set_state service is unavailable')
            return
        request = SetInt16.Request()
        request.data = int(self.robot_servo_state)
        future = self.set_state_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._ros_set_state_completed(completed, on_success))

    def _ros_set_state_completed(self, future, on_success):
        try:
            response = future.result()
        except Exception as exc:
            self.latch_fault(f'ufactory set_state failed: {exc}')
            return
        if response is None or response.ret != 0:
            code = None if response is None else response.ret
            message = '' if response is None else response.message
            self.latch_fault(
                f'ufactory set_state({self.robot_servo_state}) failed: '
                f'ret={code} {message}')
            return
        on_success()

    def _activate_trajectory_controller(self, on_success):
        if not self.controller_list_client.service_is_ready():
            self.latch_fault('controller_manager list service unavailable')
            return
        future = self.controller_list_client.call_async(ListControllers.Request())
        future.add_done_callback(
            lambda completed: self._activation_state_received(
                completed, on_success))

    def _activation_state_received(self, future, on_success):
        try:
            response = future.result()
        except Exception as exc:
            self.latch_fault(f'failed to inspect Servo controllers: {exc}')
            return
        states = {
            controller.name: controller.state
            for controller in response.controller
        } if response is not None else {}
        required = (self.joint_state_broadcaster, self.trajectory_controller)
        missing = [name for name in required if states.get(name) != 'active']
        if not missing:
            self.trajectory_controller_active = True
            self.get_logger().info('required Servo controllers already active')
            self._wait_for_fresh_joint_state(on_success)
            return
        if not self.controller_switch_client.service_is_ready():
            self.latch_fault('controller_manager switch service unavailable')
            return
        request = SwitchController.Request()
        # xArm mode changes used by direct Cartesian retreat can leave the
        # state broadcaster inactive. MoveIt Servo requires fresh joint states,
        # so restore state feedback together with trajectory ownership.
        request.activate_controllers = missing
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = Duration(seconds=3.0).to_msg()
        future = self.controller_switch_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._activate_trajectory_completed(
                completed, on_success))

    def _activate_trajectory_completed(self, future, on_success):
        try:
            response = future.result()
        except Exception as exc:
            self.latch_fault(
                f'failed to activate {self.trajectory_controller}: {exc}')
            return
        if response is None or not response.ok:
            self.latch_fault(
                f'{self.joint_state_broadcaster} or '
                f'{self.trajectory_controller} is inactive; Servo cannot move')
            return
        self._verify_servo_controllers(on_success)

    def _verify_servo_controllers(self, on_success):
        if not self.controller_list_client.service_is_ready():
            self.latch_fault('controller_manager list service unavailable')
            return
        future = self.controller_list_client.call_async(ListControllers.Request())
        future.add_done_callback(
            lambda completed: self._controller_list_completed(
                completed, on_success))

    def _controller_list_completed(self, future, on_success):
        try:
            response = future.result()
        except Exception as exc:
            self.latch_fault(f'failed to verify Servo controllers: {exc}')
            return
        states = {
            controller.name: controller.state
            for controller in response.controller
        } if response is not None else {}
        inactive = [
            name for name in (
                self.joint_state_broadcaster, self.trajectory_controller)
            if states.get(name) != 'active'
        ]
        if inactive:
            self.latch_fault(
                f'Servo controllers not active after switch: {inactive}; '
                f'states={states}')
            return
        self.trajectory_controller_active = True
        self.get_logger().info(
            f'{self.joint_state_broadcaster} and {self.trajectory_controller} '
            'are active for Servo')
        self._wait_for_fresh_joint_state(on_success)

    def _begin_real_servo_motion(self):
        # A successfully executed MoveIt trajectory already proves that the
        # robot is in ros2_control mode and that the trajectory controller owns
        # its command interfaces. Re-sending set_mode/set_state here is not a
        # harmless no-op on xArm: RobotHW briefly becomes "not ready" and can
        # asynchronously deactivate both controllers about one second later.
        # That race used to start after the place target had been accepted,
        # stopping /joint_states before Servo could move.
        if not self.controller_list_client.service_is_ready():
            self.latch_fault('controller_manager list service unavailable')
            return
        future = self.controller_list_client.call_async(ListControllers.Request())
        future.add_done_callback(self._pre_servo_controller_state_received)

    def _pre_servo_controller_state_received(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.latch_fault(
                f'failed to inspect controllers before Servo start: {exc}')
            return
        states = {
            controller.name: controller.state
            for controller in response.controller
        } if response is not None else {}
        required = (self.joint_state_broadcaster, self.trajectory_controller)
        if (all(states.get(name) == 'active' for name in required) and
                self._joint_states_fresh()):
            self.get_logger().info(
                'ros2_control is already healthy; preserving controller '
                'ownership for Servo')
            self._switch_servo_to_twist()
            return

        self.get_logger().warn(
            'ros2_control is not ready before Servo start; requesting robot '
            f'mode/state recovery (states={states})')
        self._ros_set_mode(self._prepare_robot_state_for_servo)

    def _prepare_robot_state_for_servo(self):
        self._ros_set_state(self._switch_servo_to_twist)

    def _switch_servo_to_twist(self):
        if not self.switch_client.service_is_ready():
            self.latch_fault('Servo command-type service unavailable')
            return
        switch = ServoCommandType.Request()
        switch.command_type = ServoCommandType.Request.TWIST
        if not self.pause_client.service_is_ready():
            self.latch_fault('Servo pause service unavailable')
            return
        switch_future = self.switch_client.call_async(switch)
        switch_future.add_done_callback(self._switch_to_twist_completed)

    def _switch_to_twist_completed(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.latch_fault(f'failed to switch Servo to Twist mode: {exc}')
            return
        if response is None or not response.success:
            self.latch_fault('Servo rejected Twist command mode')
            return
        request = SetBool.Request()
        request.data = False
        unpause_future = self.pause_client.call_async(request)
        unpause_future.add_done_callback(self._servo_unpause_completed)

    def _servo_unpause_completed(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.latch_fault(f'failed to unpause Servo: {exc}')
            return
        if response is None or not response.success:
            self.latch_fault('Servo rejected unpause request')
            return
        self.get_logger().info(
            'MoveIt Servo unpaused; verifying trajectory controller and '
            'joint-state feedback')
        # set_state(0) was already issued by the recovery path when needed.
        # Issuing it again on the normal path can make RobotHW deactivate the
        # active controllers after this callback has returned.
        self._finalize_servo_motion_ready()

    def _finalize_servo_motion_ready(self):
        self._activate_trajectory_controller(self._mark_servo_motion_ready)

    def _mark_servo_motion_ready(self):
        self.servo_motion_ready = True
        self.get_logger().info('MoveIt Servo is in Twist mode and ready')

    def enable_callback(self, request, response):
        if not request.data:
            self.enabled = False
            self.servo_motion_ready = False
            self.target_z = None
            self.enable_z = None
            self.touch_descent = False
            self._publish_zero()
            self._set_servo_paused(True)
            self.state = self.FAULT if self.fault else self.IDLE
            response.success = True
            response.message = 'MoveIt safe servo disabled'
            self.publish_status()
            return response
        if self.fault:
            response.message = f'reset fault first: {self.fault}'
            return response
        if self.motion_state not in ('IDLE', 'SUCCEEDED'):
            response.message = f'motion coordinator is {self.motion_state}'
            return response
        if self.active_trajectory:
            response.message = 'trajectory action is active'
            return response
        if not self._joint_states_fresh():
            age = (None if self.last_joint_state_time is None else
                   time.monotonic() - self.last_joint_state_time)
            response.message = f'/joint_states is not fresh (age={age})'
            return response
        current = self._current_pose()
        if current is None:
            response.message = self.fault
            return response
        if not self.z_bounds[0] <= current.z <= self.z_bounds[1]:
            response.message = (
                f'current Z {current.z:.4f} m outside workspace '
                f'{self.z_bounds}')
            return response
        if self.last_force_time is None or (
                time.monotonic() - self.last_force_time > self.force_timeout):
            response.message = 'external force data is stale'
            return response
        if (self.force_norm is None or self.torque_norm is None or
                ((not self.bypass_force_arm_check) and (not self.touch_mode) and (
                    self.force_norm >= 0.8 * self.force_limit or
                    self.torque_norm >= 0.8 * self.torque_limit))):
            response.message = 'external force/torque is not clear'
            return response
        if not self.dry_run:
            if not self.servo_pause_confirmed:
                response.message = 'Servo startup pause is not confirmed'
                return response
            self._begin_real_servo_motion()
        self.enabled = True
        self.servo_motion_ready = self.dry_run
        self.state = self.ARMED
        self.touch_descent = False
        self.touch_contact = False
        self.enable_generation += 1
        self.enable_force_baseline = self.force_norm
        self.enable_wrench_baseline = self.latest_wrench
        self.force_delta_norm = 0.0
        self.force_delta_z = 0.0
        if self.dry_run and self.dry_run_z is None:
            self.dry_run_z = current.z
        self.enable_z = self.dry_run_z if self.dry_run else current.z
        self.target_z = None
        self.last_command_time = None
        response.success = True
        response.message = (
            'dry-run vertical servo armed' if self.dry_run
            else 'MoveIt vertical servo arming')
        # Publish the cleared touch_contact immediately so the supervisor
        # cannot consume a latched event from the previous operation.
        self.publish_status()
        return response

    def _latch_touch_contact(self):
        if self.touch_contact:
            return
        self.touch_contact = True
        self.target_z = None
        self.enabled = False
        self.servo_motion_ready = False
        self._publish_zero()
        self._set_servo_paused(True)
        self.state = self.IDLE
        self.get_logger().info(
            f'touch contact at force={self.force_norm:.2f} N, '
            f'delta_fz={self.force_delta_z:.2f} N, '
            f'torque={self.torque_norm:.2f} Nm')

    def reset_callback(self, _request, response):
        self.enabled = False
        self.servo_motion_ready = False
        self.target_z = None
        self.dry_run_z = None
        self.touch_descent = False
        self.touch_contact = False
        self.bypass_force_arm_check = False
        self.enable_force_baseline = None
        self.enable_wrench_baseline = None
        self.force_delta_norm = None
        self.force_delta_z = None
        self.trajectory_controller_active = False
        self.ready_wait_callback = None
        self.ready_wait_sequence = None
        self.ready_wait_deadline = None
        self.fault = ''
        self.state = self.IDLE
        self._publish_zero()
        self._set_servo_paused(True)
        response.success = True
        response.message = 'fault reset; servo remains disabled'
        self.publish_status()
        return response

    def control_loop(self):
        if self.touch_contact:
            return
        if not self.enabled or self.target_z is None or self.fault:
            return
        if not self.servo_motion_ready:
            return
        if not self._joint_states_fresh():
            age = (None if self.last_joint_state_time is None else
                   time.monotonic() - self.last_joint_state_time)
            self.latch_fault(
                f'/joint_states watchdog expired during Servo motion: age={age}')
            return
        if (self.last_force_time is None or
                time.monotonic() - self.last_force_time > self.force_timeout):
            self.latch_fault('external force data watchdog timeout')
            return
        if (self.last_command_time is None or
                time.monotonic() - self.last_command_time > self.command_timeout):
            self.latch_fault('command watchdog timeout')
            return
        if self.active_trajectory or self.motion_state not in ('IDLE', 'SUCCEEDED'):
            self.latch_fault('trajectory execution interlock opened')
            return
        current = self._current_pose()
        if current is None:
            return
        error = self.target_z - current.z
        velocity = max(-self.max_speed, min(self.max_speed, self.kp_z * error))
        if abs(error) <= self.tolerance:
            velocity = 0.0
            self.state = self.ARMED
        else:
            self.state = self.RUNNING
        self._publish_proposed(current, self.target_z)
        if not self.dry_run:
            self._publish_twist(velocity)

    def _publish_twist(self, z_velocity):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.planning_frame
        msg.twist.linear.z = float(z_velocity)
        self.twist_pub.publish(msg)
        self.get_logger().debug(f'published vertical Twist z={z_velocity:.4f} m/s')

    def _publish_zero(self):
        if not self.dry_run:
            self._publish_twist(0.0)

    def _publish_proposed(self, current, target_z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.planning_frame
        msg.pose.position.x = current.x
        msg.pose.position.y = current.y
        msg.pose.position.z = target_z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def latch_fault(self, reason):
        if self.fault:
            return
        self.fault = reason
        self.enabled = False
        self.servo_motion_ready = False
        self.ready_wait_callback = None
        self.ready_wait_sequence = None
        self.ready_wait_deadline = None
        self.state = self.FAULT
        self._publish_zero()
        self._set_servo_paused(True)
        msg = String()
        msg.data = reason
        self.fault_pub.publish(msg)
        self.get_logger().error(f'FAULT: {reason}')

    def publish_status(self):
        # This is telemetry only.  Do not use _current_pose() here because a
        # temporary TF lookup failure must not fault an otherwise idle bridge.
        tcp = {}
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, self.ee_frame, rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)).transform
            translation = transform.translation
            rotation = transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))
            tcp = {
                'tcp_x_m': translation.x,
                'tcp_y_m': translation.y,
                'tcp_z_m': translation.z,
                'tcp_yaw_deg': math.degrees(yaw),
            }
        except TransformException:
            pass
        msg = String()
        status = {
            'state': self.state,
            'dry_run': self.dry_run,
            'vertical_only': True,
            'target_z_m': self.target_z,
            'dry_run_z_m': self.dry_run_z if self.dry_run else None,
            'motion_state': self.motion_state,
            'active_trajectory': self.active_trajectory,
            'servo_status': self.servo_status,
            'force_norm_n': self.force_norm,
            'force_x_n': None if self.latest_wrench is None else self.latest_wrench[0],
            'force_y_n': None if self.latest_wrench is None else self.latest_wrench[1],
            'force_z_n': None if self.latest_wrench is None else self.latest_wrench[2],
            'torque_norm_nm': self.torque_norm,
            'force_delta_n': self.force_delta_norm,
            'force_delta_z_n': self.force_delta_z,
            'force_limit_n': self.force_limit,
            'touch_mode': self.touch_mode,
            'touch_descent': self.touch_descent,
            'touch_contact': self.touch_contact,
            'bypass_force_arm_check': self.bypass_force_arm_check,
            'enable_force_baseline_n': self.enable_force_baseline,
            'enable_generation': self.enable_generation,
            'trajectory_controller_active': self.trajectory_controller_active,
            'servo_motion_ready': self.servo_motion_ready,
            'configured_max_linear_speed_m_s': self.configured_max_speed,
            'active_max_linear_speed_m_s': self.max_speed,
            'kp_z': self.kp_z,
            'joint_state_age_sec': (
                None if self.last_joint_state_time is None else
                time.monotonic() - self.last_joint_state_time),
            'fault': self.fault,
        }
        status.update(tcp)
        msg.data = json.dumps(status, separators=(',', ':'))
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItServoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
