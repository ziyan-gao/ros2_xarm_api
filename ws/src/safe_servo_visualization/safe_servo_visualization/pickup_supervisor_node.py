import json
import math
import time

from controller_manager_msgs.srv import ListControllers, SwitchController
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import (
    Call, GetInt16, MoveCartesian, SetInt16, VacuumGripperCtrl)


class PickupSupervisor(Node):
    """Supervise a vertical-only pickup after pre-grasp execution."""

    IDLE = 'IDLE'
    ARMING_DESCENT = 'ARMING_DESCENT'
    DESCENDING = 'DESCENDING'
    DISABLING_DESCENT = 'DISABLING_DESCENT'
    VACUUM_ON = 'VACUUM_ON'
    VACUUM_OFF = 'VACUUM_OFF'
    DETACHING = 'DETACHING'
    VERIFYING_VACUUM = 'VERIFYING_VACUUM'
    DISABLING_SERVO = 'DISABLING_SERVO'
    PREPARING_RETREAT = 'PREPARING_RETREAT'
    RETREATING = 'RETREATING'
    RESTORING_CONTROL = 'RESTORING_CONTROL'
    SUCCEEDED = 'SUCCEEDED'
    FAULT = 'FAULT'

    ACTIVE = {
        ARMING_DESCENT, DESCENDING, DISABLING_DESCENT, DISABLING_SERVO,
        VACUUM_ON, VACUUM_OFF, DETACHING, VERIFYING_VACUUM,
        PREPARING_RETREAT, RETREATING, RESTORING_CONTROL,
    }

    def __init__(self):
        super().__init__('pickup_supervisor')
        self.declare_parameter('refined_boxes_topic', '/pointcloud_detection/boxes')
        self.declare_parameter('target_box_id', -1)
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('max_pregrasp_plan_age_sec', 1800.0)
        self.declare_parameter('grasp_offset_m', 0.0)
        self.declare_parameter('contact_search_margin_m', 0.015)
        self.declare_parameter('max_descent_m', 0.15)
        self.declare_parameter('position_tolerance_m', 0.001)
        self.declare_parameter('xy_tolerance_m', 0.015)
        self.declare_parameter('pregrasp_z_tolerance_m', 0.015)
        self.declare_parameter('minimum_contact_descent_m', 0.005)
        self.declare_parameter('descent_timeout_sec', 30.0)
        self.declare_parameter('retreat_timeout_sec', 120.0)
        self.declare_parameter('retreat_speed_mm_s', 30.0)
        self.declare_parameter('retreat_acc_mm_s2', 200.0)
        self.declare_parameter('ros2_control_mode', 1)
        self.declare_parameter('trajectory_controller', 'uf850_traj_controller')
        self.declare_parameter('joint_state_broadcaster', 'joint_state_broadcaster')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_state_ready_timeout_sec', 5.0)
        self.declare_parameter('joint_state_ready_samples', 3)
        self.declare_parameter('ft_zero_settle_sec', 0.25)
        self.declare_parameter('post_ft_state_settle_sec', 1.25)
        self.declare_parameter('status_timeout_sec', 1.0)
        self.declare_parameter('vacuum_timeout_sec', 5.0)
        self.declare_parameter('vacuum_hardware_version', 1)
        self.declare_parameter('vacuum_on_status', 1)
        self.declare_parameter('require_vacuum_sensor', False)
        # Hold the contacted pose after suction is enabled so the cup has time
        # to seal before the direct vertical retreat starts.
        self.declare_parameter('vacuum_settle_sec', 1.0)
        self.declare_parameter('vacuum_verify_attempts', 10)
        self.declare_parameter('vacuum_verify_interval_sec', 0.5)
        self.declare_parameter('force_contact_threshold_n', 15.0)
        self.declare_parameter('place_force_contact_threshold_n', 4.0)
        # Keep pickup at 20 mm/s with the 50 mm/s bridge cap.
        self.declare_parameter('servo_speed_scale', 0.4)
        self.declare_parameter('place_servo_speed_scale', 1.0)
        self.declare_parameter('workspace_x_min_mm', 160.0)
        self.declare_parameter('workspace_x_max_mm', 390.0)
        self.declare_parameter('workspace_y_min_mm', -360.0)
        self.declare_parameter('workspace_y_max_mm', 360.0)
        self.declare_parameter('workspace_z_min_mm', 50.0)
        self.declare_parameter('workspace_z_max_mm', 800.0)
        self.declare_parameter('transfer_radius_max_mm', 820.0)
        self.declare_parameter('transfer_z_min_mm', 50.0)
        self.declare_parameter('transfer_z_max_mm', 800.0)
        self.declare_parameter('pre_place_clearance_m', 0.04)
        self.declare_parameter('container_height_m', 0.45)

        def p(name):
            return self.get_parameter(name).value
        self.target_box_id = int(p('target_box_id'))
        self.max_detection_age = float(p('max_detection_age_sec'))
        self.max_pregrasp_plan_age = float(p('max_pregrasp_plan_age_sec'))
        self.grasp_offset = float(p('grasp_offset_m'))
        self.contact_search_margin = float(p('contact_search_margin_m'))
        self.max_descent = float(p('max_descent_m'))
        self.tolerance = float(p('position_tolerance_m'))
        self.xy_tolerance = float(p('xy_tolerance_m'))
        self.pregrasp_z_tolerance = float(p('pregrasp_z_tolerance_m'))
        self.minimum_contact_descent = float(p('minimum_contact_descent_m'))
        self.descent_timeout = float(p('descent_timeout_sec'))
        self.retreat_timeout = float(p('retreat_timeout_sec'))
        self.retreat_speed = float(p('retreat_speed_mm_s'))
        self.retreat_acc = float(p('retreat_acc_mm_s2'))
        self.ros2_control_mode = int(p('ros2_control_mode'))
        self.trajectory_controller = str(p('trajectory_controller'))
        self.joint_state_broadcaster = str(p('joint_state_broadcaster'))
        self.joint_state_topic = str(p('joint_state_topic'))
        self.joint_state_ready_timeout = float(p('joint_state_ready_timeout_sec'))
        self.joint_state_ready_samples = int(p('joint_state_ready_samples'))
        self.ft_zero_settle = float(p('ft_zero_settle_sec'))
        self.post_ft_state_settle = float(p('post_ft_state_settle_sec'))
        self.status_timeout = float(p('status_timeout_sec'))
        self.vacuum_timeout = float(p('vacuum_timeout_sec'))
        self.vacuum_hardware_version = int(p('vacuum_hardware_version'))
        self.vacuum_on_status = int(p('vacuum_on_status'))
        self.require_vacuum_sensor = bool(p('require_vacuum_sensor'))
        self.vacuum_settle_sec = float(p('vacuum_settle_sec'))
        self.vacuum_verify_attempts = int(p('vacuum_verify_attempts'))
        self.vacuum_verify_interval = float(p('vacuum_verify_interval_sec'))
        self.force_threshold = float(p('force_contact_threshold_n'))
        self.place_force_threshold = float(
            p('place_force_contact_threshold_n'))
        self.servo_speed_scale = float(p('servo_speed_scale'))
        self.place_servo_speed_scale = float(p('place_servo_speed_scale'))
        self.servo_bounds_mm = (
            float(p('workspace_x_min_mm')), float(p('workspace_x_max_mm')),
            float(p('workspace_y_min_mm')), float(p('workspace_y_max_mm')),
            float(p('workspace_z_min_mm')), float(p('workspace_z_max_mm')),
        )
        self.transfer_radius_max_mm = float(p('transfer_radius_max_mm'))
        self.transfer_z_bounds_mm = (
            float(p('transfer_z_min_mm')), float(p('transfer_z_max_mm')))
        self.pre_place_clearance = float(p('pre_place_clearance_m'))
        self.container_height = float(p('container_height_m'))
        if not 0.0 <= self.contact_search_margin <= self.max_descent:
            raise ValueError(
                'contact_search_margin_m must be within '
                f'[0, max_descent_m={self.max_descent:.3f}]')
        if self.place_force_threshold <= 0.0:
            raise ValueError('place_force_contact_threshold_n must be positive')
        if not (0.01 <= self.servo_speed_scale <= 1.0 and
                0.01 <= self.place_servo_speed_scale <= 1.0):
            raise ValueError('Servo speed scales must be within [0.01, 1.0]')
        if self.joint_state_ready_timeout <= 0.0:
            raise ValueError('joint_state_ready_timeout_sec must be positive')
        if self.joint_state_ready_samples < 1:
            raise ValueError('joint_state_ready_samples must be at least 1')
        if self.ft_zero_settle < 0.2:
            raise ValueError('ft_zero_settle_sec must be at least 0.2')
        if self.post_ft_state_settle <= 0.0:
            raise ValueError('post_ft_state_settle_sec must be positive')
        self.state = self.IDLE
        self.fault = ''
        self.boxes = {}
        self.servo_status = {}
        self.servo_status_time = None
        self.motion_status = {}
        self.pregrasp_z = None
        self.floor_z = None
        self.virtual_z = None
        self.descent_target_z = None
        self.descent_started = None
        self.retreat_started = None
        self.retreat_target_z = None
        self.retreat_start_z = None
        self.retreat_start_xyz = None
        self.direct_target_z = None
        self.direct_target_xyz = None
        self.direct_target_rpy = None
        self.place_target_z = None
        self.post_retreat_fault = ''
        self.dry_run = True
        self.vacuum_verified = False
        self.vacuum_verify_count = 0
        self._vacuum_retry_timer = None
        self._vacuum_settle_timer = None
        self.operation_id = 0
        self.contact_detected = False
        self.expected_enable_generation = None
        self.operation_kind = 'pickup'
        self.planning_scene_status = {}
        self.pallet_locked = False
        self.detach_started = None
        self.last_joint_state_time = None
        self.joint_state_sequence = 0
        self.restore_wait_sequence = None
        self.restore_wait_deadline = None
        self.retreat_controller_wait_deadline = None
        self.retreat_controller_query_pending = False
        self.robot_state = None
        self.robot_mode = None
        self.robot_error = None
        self.robot_state_time = None
        self.robot_tcp_xyz = None
        self._ft_settle_timer = None
        self._post_ft_state_timer = None
        self.pre_descent_wait_callback = None
        self.pre_descent_wait_sequence = None
        self.pre_descent_wait_deadline = None
        self.pre_descent_controller_query_pending = False
        self.pre_descent_activation_attempted = False
        self.descent_config_deadline = None
        self.descent_config_last_publish = None

        self.command_pub = self.create_publisher(
            Float64MultiArray, '/servo_command', 10)
        self.config_pub = self.create_publisher(
            Float64MultiArray, '/safe_servo/config', 10)
        self.status_pub = self.create_publisher(
            String, '/pickup_supervisor/status', 10)
        self.create_subscription(
            MarkerArray, str(p('refined_boxes_topic')), self.boxes_callback, 10)
        self.create_subscription(
            String, '/safe_servo/status', self.servo_status_callback, 10)
        self.create_subscription(
            String, '/motion_coordinator/status', self.motion_status_callback, 10)
        self.create_subscription(
            String, '/planning_scene_obstacles/status',
            self.planning_scene_status_callback, 10)
        self.create_subscription(
            String, '/pallet_localization/status',
            self.pallet_status_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/pallet_localization/config_state',
            self.pallet_config_callback, 10)
        self.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 10)
        self.create_subscription(
            RobotMsg, '/ufactory/robot_states', self.robot_state_callback, 10)
        self.enable_client = self.create_client(SetBool, '/safe_servo/enable')
        self.servo_reset_client = self.create_client(
            Trigger, '/safe_servo/reset_fault')
        self.vacuum_client = self.create_client(
            VacuumGripperCtrl, '/ufactory/set_vacuum_gripper')
        self.vacuum_status_client = self.create_client(
            GetInt16, '/ufactory/get_vacuum_gripper')
        self.ft_zero_client = self.create_client(
            Call, '/ufactory/set_ft_sensor_zero')
        self.plan_pregrasp_client = self.create_client(
            Trigger, '/motion_coordinator/plan_pregrasp')
        self.set_mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
        self.set_state_client = self.create_client(SetInt16, '/ufactory/set_state')
        self.retreat_client = self.create_client(
            MoveCartesian, '/ufactory/set_position')
        self.controller_switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.controller_list_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.detach_item_client = self.create_client(
            Trigger, '/planning_scene_obstacles/detach_item')
        self.create_service(Trigger, '/pickup_supervisor/start', self.start_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_place', self.start_place_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_loading', self.start_loading_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_transfer', self.start_transfer_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/retreat', self.retreat_callback)
        self.create_service(Trigger, '/pickup_supervisor/abort', self.abort_callback)
        self.create_service(Trigger, '/pickup_supervisor/reset', self.reset_callback)
        self.create_timer(0.05, self.control_tick)
        self.create_timer(0.1, self.retreat_tick)
        self.create_timer(0.5, self.publish_status)

    def boxes_callback(self, message):
        self.boxes = {
            int(marker.id): marker for marker in message.markers
            if marker.ns == 'depth_refined_boxes' and
            marker.action == Marker.ADD and marker.header.frame_id == 'link_base'
        }

    def joint_state_callback(self, _message):
        self.last_joint_state_time = time.monotonic()
        self.joint_state_sequence += 1

    def robot_state_callback(self, message):
        self.robot_state = int(message.state)
        self.robot_mode = int(message.mode)
        self.robot_error = int(message.err)
        self.robot_state_time = time.monotonic()
        if len(message.pose) >= 3:
            xyz = tuple(float(value) / 1000.0 for value in message.pose[:3])
            if all(math.isfinite(value) for value in xyz):
                self.robot_tcp_xyz = xyz

    def servo_status_callback(self, message):
        try:
            self.servo_status = json.loads(message.data)
            self.servo_status_time = time.monotonic()
            self.dry_run = bool(self.servo_status.get('dry_run', True))
        except (TypeError, ValueError):
            self.servo_status = {}

    def motion_status_callback(self, message):
        try:
            self.motion_status = json.loads(message.data)
        except (TypeError, ValueError):
            self.motion_status = {}

    def planning_scene_status_callback(self, message):
        try:
            self.planning_scene_status = json.loads(message.data)
        except (TypeError, ValueError):
            self.planning_scene_status = {}

    def pallet_status_callback(self, message):
        self.pallet_locked = message.data == 'LOCKED'

    def pallet_config_callback(self, message):
        if len(message.data) >= 12:
            self.place_target_z = float(message.data[11]) / 1000.0

    def _validate_pregrasp_ready(self):
        if self.motion_status.get('state') != 'SUCCEEDED' or not str(
                self.motion_status.get('target', '')).startswith('pregrasp_box_'):
            raise ValueError('execute a successful pre-grasp plan first')
        snapshot = self.motion_status.get('planned_pregrasp')
        if not isinstance(snapshot, dict):
            raise ValueError('validated pre-grasp snapshot is unavailable')
        expected_target = f"pregrasp_box_{int(snapshot['box_id'])}"
        if self.motion_status.get('target') != expected_target:
            raise ValueError('pre-grasp target does not match its box snapshot')
        plan_age = (
            self.get_clock().now().nanoseconds * 1e-9 -
            float(snapshot['planned_stamp_sec']))
        if plan_age < -0.05 or plan_age > self.max_pregrasp_plan_age:
            raise ValueError(f'pre-grasp snapshot is stale ({plan_age:.1f} s)')
        x, y, z = self._tcp_xyz()
        if abs(x - float(snapshot['x_m'])) > self.xy_tolerance or abs(
                y - float(snapshot['y_m'])) > self.xy_tolerance:
            raise ValueError('TCP is not aligned over the selected box')
        expected_z = float(snapshot['pregrasp_z_m'])
        z_error = z - expected_z
        if abs(z_error) > self.pregrasp_z_tolerance:
            raise ValueError(
                f'TCP is not at the verified pre-grasp height: '
                f'current={z:.4f} m, expected={expected_z:.4f} m, '
                f'error={z_error * 1000.0:+.1f} mm, '
                f'tolerance={self.pregrasp_z_tolerance * 1000.0:.1f} mm')
        estimated_contact_z = float(snapshot['top_z_m']) + self.grasp_offset
        servo_floor_z = self.servo_bounds_mm[4] / 1000.0
        floor_z = max(
            estimated_contact_z - self.contact_search_margin,
            z - self.max_descent,
            servo_floor_z)
        descent = z - floor_z
        if not 0.0 < descent <= self.max_descent:
            raise ValueError(
                f'pickup descent {descent:.3f} m is outside (0, {self.max_descent:.3f}]')
        if self.servo_status.get('fault'):
            raise ValueError(f"safe-servo fault: {self.servo_status['fault']}")
        return snapshot, z, floor_z

    def _tcp_xyz(self):
        keys = ('tcp_x_m', 'tcp_y_m', 'tcp_z_m')
        if self.servo_status_time is None or (
                time.monotonic() - self.servo_status_time > self.status_timeout):
            if (self.robot_tcp_xyz is not None and
                    self.robot_state_time is not None and
                    time.monotonic() - self.robot_state_time <= self.status_timeout):
                return self.robot_tcp_xyz
            raise ValueError('TCP telemetry is stale')
        joint_age = self.servo_status.get('joint_state_age_sec')
        supervisor_joint_age = (
            None if self.last_joint_state_time is None else
            time.monotonic() - self.last_joint_state_time)
        safe_servo_joint_fresh = (
            joint_age is not None and float(joint_age) <= self.status_timeout)
        supervisor_joint_fresh = (
            supervisor_joint_age is not None and
            supervisor_joint_age <= self.status_timeout)
        if not safe_servo_joint_fresh and not supervisor_joint_fresh:
            raise ValueError(
                '/joint_states is stale or unavailable '
                f'(safe_servo_age={joint_age}, '
                f'supervisor_age={supervisor_joint_age})')
        try:
            xyz = tuple(float(self.servo_status[key]) for key in keys)
        except (KeyError, TypeError, ValueError):
            raise ValueError('safe-servo TCP telemetry is unavailable')
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError('safe-servo TCP telemetry is invalid')
        return xyz

    def _publish_servo_config(self, touch_mode, bypass_force=False):
        is_place = self.operation_kind == 'place'
        force = (self.place_force_threshold if is_place else
                 float(self.servo_status.get(
                     'force_limit_n', self.force_threshold)))
        speed_scale = (self.place_servo_speed_scale if is_place else
                       self.servo_speed_scale)
        message = Float64MultiArray()
        message.data = [
            speed_scale,
            self.servo_bounds_mm[0], self.servo_bounds_mm[1],
            self.servo_bounds_mm[2], self.servo_bounds_mm[3],
            self.servo_bounds_mm[4], self.servo_bounds_mm[5],
            force,
            1.0 if touch_mode else 0.0,
            1.0 if bypass_force else 0.0,
            1.0 if is_place else 0.0,
        ]
        self.config_pub.publish(message)

    def start_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'pickup already active in {self.state}'
            return response
        if not self.enable_client.service_is_ready():
            response.message = 'safe-servo enable service is unavailable'
            return response
        try:
            snapshot, z, floor_z = self._validate_pregrasp_ready()
        except ValueError as exc:
            response.message = str(exc)
            return response

        object_height = float(snapshot['size_z_m'])
        if self.place_target_z is None:
            response.message = 'place target Z is unavailable'
            return response
        lift = self.pre_place_clearance + object_height + self.place_target_z
        if lift > self.container_height:
            lift = self.pre_place_clearance + self.place_target_z
        self.direct_target_z = z + lift
        self.direct_target_xyz = None
        self.direct_target_rpy = None
        if self.direct_target_z > self.servo_bounds_mm[5] / 1000.0:
            response.message = (
                f'nominal pickup retreat Z {self.direct_target_z:.3f} m exceeds '
                'the configured workspace ceiling')
            return response
        self.operation_id += 1
        self.operation_kind = 'pickup'
        self.fault = ''
        self.post_retreat_fault = ''
        self.vacuum_verified = False
        self.vacuum_verify_count = 0
        self.contact_detected = False
        self.pregrasp_z = z
        self.direct_target_xyz = None
        self.direct_target_rpy = None
        self.floor_z = floor_z
        self.virtual_z = z
        self.state = self.ARMING_DESCENT
        self._reset_servo_then_zero_and_descend(snapshot, z, floor_z)
        response.success = True
        response.message = (
            f"pickup started for box {int(snapshot['box_id'])}: "
            f'zeroing FT sensor, then contact descent from Z {z:.3f} m; '
            f'dry_run={self.dry_run}')
        return response

    def _reset_servo_then_zero_and_descend(self, snapshot, z, floor_z):
        if not self.servo_reset_client.service_is_ready():
            self._fault('safe-servo reset service is unavailable')
            return
        future = self.servo_reset_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda completed: self._servo_reset_completed(
                completed, snapshot, z, floor_z))

    def _servo_reset_completed(self, future, snapshot, z, floor_z):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'safe-servo reset failed before pickup: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'safe-servo reset rejected before pickup: {message}')
            return
        self.contact_detected = False
        self.expected_enable_generation = None
        self.get_logger().info(
            'cleared stale touch_contact; arming pickup descent with a new '
            'software Fz baseline')
        # Hardware FT zero temporarily puts this UF850 into state 5.  That
        # causes RobotHW to deactivate both controllers from inside its update
        # loop and creates one-second feedback gaps.  The guarded Servo records
        # a fresh force baseline on every enable, so a second hardware zero is
        # unnecessary after the startup initializer.
        self._begin_descent(snapshot, z, floor_z)

    def start_place_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'supervisor already active in {self.state}'
            return response
        if self.motion_status.get('state') != 'SUCCEEDED' or (
                self.motion_status.get('target') != 'transfer_ready'):
            response.message = 'execute transfer and linear loading first'
            return response
        if not self.pallet_locked:
            response.message = 'pallet pose is not LOCKED'
            return response
        if not self.planning_scene_status.get('attached_item_id'):
            response.message = 'no carried item is attached in the planning scene'
            return response
        if not self.enable_client.service_is_ready():
            response.message = 'safe-servo enable service is unavailable'
            return response
        try:
            _, _, z = self._tcp_xyz()
        except ValueError as exc:
            response.message = str(exc)
            return response
        self.operation_id += 1
        self.operation_kind = 'place'
        self.fault = ''
        self.post_retreat_fault = ''
        self.contact_detected = False
        self.pregrasp_z = z
        try:
            self.direct_target_z = float(
                self.motion_status['transfer_tcp_z_m'])
        except (KeyError, TypeError, ValueError):
            response.message = 'recorded transfer retreat height is unavailable'
            return response
        if self.direct_target_z < z - self.tolerance:
            response.message = 'recorded transfer retreat height is below TCP'
            return response
        self.direct_target_xyz = None
        self.direct_target_rpy = None
        servo_floor_z = self.servo_bounds_mm[4] / 1000.0
        self.floor_z = max(z - self.max_descent, servo_floor_z)
        if z - self.floor_z < self.minimum_contact_descent:
            response.message = (
                f'pre-place Z {z:.3f} m leaves less than '
                f'{self.minimum_contact_descent:.3f} m guarded descent above '
                f'the Servo floor {servo_floor_z:.3f} m')
            return response
        self.virtual_z = z
        # Do not hardware-zero while carrying an item. The xArm may reject
        # that operation (C52), and the Servo bridge already records a fresh
        # software wrench baseline every time it is enabled.
        self.state = self.ARMING_DESCENT
        self._reset_servo_then_begin_place(z, self.floor_z)
        response.success = True
        response.message = (
            f'place contact descent is arming from Z {z:.3f} m; '
            f'maximum descent={self.max_descent:.3f} m')
        return response

    def start_loading_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'supervisor already active in {self.state}'
            return response
        if (self.motion_status.get('state') != 'SUCCEEDED' or
                self.motion_status.get('target') != 'transfer_ready'):
            response.message = 'execute the nominal transfer pose first'
            return response
        target_z = self.motion_status.get('pre_place_tcp_z_m')
        try:
            current_z = self._tcp_xyz()[2]
            target_z = float(target_z)
        except (TypeError, ValueError) as exc:
            response.message = f'loading target is unavailable: {exc}'
            return response
        if target_z > current_z + self.tolerance:
            response.message = 'loading target must not move upward'
            return response
        self.operation_id += 1
        self.operation_kind = 'loading'
        self.fault = ''
        self.post_retreat_fault = ''
        self.pregrasp_z = target_z
        self.direct_target_z = target_z
        self.direct_target_xyz = None
        self.direct_target_rpy = None
        self._disable_servo_then_direct_retreat()
        response.success = True
        response.message = (
            f'linear loading started: TCP Z {current_z:.3f} -> {target_z:.3f} m')
        return response

    def start_transfer_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'supervisor already active in {self.state}'
            return response
        if (self.motion_status.get('state') != 'SUCCEEDED' or
                self.motion_status.get('target') != 'transfer_ready'):
            response.message = 'prepare the nominal transfer target first'
            return response
        target = self.motion_status.get('transfer_tcp_xyz_m')
        quaternion = self.motion_status.get('transfer_tcp_quaternion_xyzw')
        if not isinstance(target, (list, tuple)) or len(target) != 3:
            response.message = 'transfer XYZ target is incomplete'
            return response
        if not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4:
            response.message = 'transfer orientation target is incomplete'
            return response
        try:
            current = self._tcp_xyz()
            target = tuple(float(value) for value in target)
            quaternion = tuple(float(value) for value in quaternion)
        except (TypeError, ValueError) as exc:
            response.message = f'transfer target is unavailable: {exc}'
            return response
        if len(target) != 3 or not all(math.isfinite(value) for value in target):
            response.message = 'transfer target is invalid'
            return response
        if len(quaternion) != 4 or not all(
                math.isfinite(value) for value in quaternion):
            response.message = 'transfer orientation is invalid'
            return response
        x_mm, y_mm, z_mm = (value * 1000.0 for value in target)
        radius_mm = math.hypot(x_mm, y_mm)
        if (radius_mm > self.transfer_radius_max_mm or
                not self.transfer_z_bounds_mm[0] <= z_mm <=
                self.transfer_z_bounds_mm[1]):
            response.message = (
                'transfer target is outside the nominal transfer envelope: '
                f'radius={radius_mm:.1f} mm '
                f'(max {self.transfer_radius_max_mm:.1f}), Z={z_mm:.1f} mm '
                f'(limits {self.transfer_z_bounds_mm[0]:.1f}..'
                f'{self.transfer_z_bounds_mm[1]:.1f})')
            return response
        qx, qy, qz, qw = quaternion
        roll = math.atan2(2.0 * (qw * qx + qy * qz),
                          1.0 - 2.0 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(
            1.0, 2.0 * (qw * qy - qz * qx))))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        self.operation_id += 1
        self.operation_kind = 'transfer'
        self.fault = ''
        self.post_retreat_fault = ''
        self.direct_target_xyz = target
        self.direct_target_rpy = (roll, pitch, yaw)
        self.direct_target_z = target[2]
        self._disable_servo_then_direct_retreat()
        response.success = True
        response.message = (
            'nominal Cartesian transfer started: '
            f'[{current[0]:.3f}, {current[1]:.3f}, {current[2]:.3f}] -> '
            f'[{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}] m')
        return response

    def _reset_servo_then_begin_place(self, z, floor_z):
        if not self.servo_reset_client.service_is_ready():
            self._fault('safe-servo reset service is unavailable before place')
            return
        future = self.servo_reset_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda completed: self._place_servo_reset_completed(
                completed, z, floor_z))

    def _place_servo_reset_completed(self, future, z, floor_z):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'safe-servo reset failed before place: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'safe-servo reset rejected before place: {message}')
            return
        self.contact_detected = False
        self.expected_enable_generation = None
        self.get_logger().info(
            'cleared pickup touch_contact; arming pallet contact descent '
            'with a new software Fz baseline')
        self._begin_descent(None, z, floor_z)

    def _begin_descent(self, snapshot, z, floor_z):
        self._publish_servo_config(touch_mode=True)
        self.descent_config_last_publish = time.monotonic()
        self.descent_config_deadline = self.descent_config_last_publish + 2.0
        self.state = self.ARMING_DESCENT
        label = (f'box {int(snapshot["box_id"])}' if snapshot is not None
                 else 'placement surface')
        self.get_logger().info(
            f'waiting for guarded Servo configuration for {label}: '
            f'Z {z:.3f} m to >= {floor_z:.3f} m '
            f'(threshold={self._configured_contact_threshold():.1f} N, '
            f'speed_scale={self._configured_speed_scale():.2f})')

    def _configured_contact_threshold(self):
        if self.operation_kind == 'place':
            return self.place_force_threshold
        return float(self.servo_status.get(
            'force_limit_n', self.force_threshold))

    def _configured_speed_scale(self):
        if self.operation_kind == 'place':
            return self.place_servo_speed_scale
        return self.servo_speed_scale

    def _descent_config_confirmed(self):
        expected_force = self._configured_contact_threshold()
        configured_speed = self.servo_status.get(
            'configured_max_linear_speed_m_s')
        active_speed = self.servo_status.get('active_max_linear_speed_m_s')
        try:
            expected_speed = (
                float(configured_speed) * self._configured_speed_scale())
            return (
                bool(self.servo_status.get('touch_mode')) and
                math.isclose(
                    float(self.servo_status.get('force_limit_n')),
                    expected_force, abs_tol=1e-6) and
                math.isclose(
                    float(active_speed), expected_speed, abs_tol=1e-6))
        except (TypeError, ValueError):
            return False

    def _tick_descent_config(self):
        if self.descent_config_deadline is None:
            return False
        now = time.monotonic()
        if self._descent_config_confirmed():
            self.descent_config_deadline = None
            self.descent_config_last_publish = None
            self.get_logger().info(
                'guarded Servo confirmed contact threshold and speed; arming')
            self._arm_descent()
            return True
        if now >= self.descent_config_deadline:
            self.descent_config_deadline = None
            self.descent_config_last_publish = None
            self._fault(
                'timed out waiting for guarded Servo to confirm '
                f'{self._configured_contact_threshold():.1f} N contact '
                'threshold and descent speed')
            return True
        if (self.descent_config_last_publish is None or
                now - self.descent_config_last_publish >= 0.2):
            self._publish_servo_config(touch_mode=True)
            self.descent_config_last_publish = now
        return True

    def _zero_ft_sensor_then(self, on_success):
        if not self.ft_zero_client.service_is_ready():
            self.get_logger().warn(
                'FT zero service unavailable; using current force as baseline')
            on_success()
            return
        future = self.ft_zero_client.call_async(Call.Request())
        future.add_done_callback(
            lambda completed: self._ft_zero_completed(completed, on_success))

    def _ft_zero_completed(self, future, on_success):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'FT zero call failed ({exc}); using current force as '
                'baseline')
        else:
            if result is None or result.ret != 0:
                code = None if result is None else result.ret
                self.get_logger().warn(
                    f'FT zero returned {code}; using current force as '
                    'baseline')
            else:
                self.get_logger().info('FT sensor re-zeroed at pre-grasp pose')

        # UFACTORY's SDK examples require at least 200 ms after
        # set_ft_sensor_zero before any further robot command. Starting Servo
        # immediately can produce set_servo_angle_j ret=9, robot state 5, and
        # an automatic ros2_control controller shutdown.
        if self._ft_settle_timer is not None:
            self._ft_settle_timer.cancel()
        self._ft_settle_timer = self.create_timer(
            self.ft_zero_settle,
            lambda: self._ft_zero_settle_elapsed(on_success))

    def _ft_zero_settle_elapsed(self, on_success):
        if self._ft_settle_timer is not None:
            self._ft_settle_timer.cancel()
            self._ft_settle_timer = None
        if self.state != self.ARMING_DESCENT:
            return
        robot_state_age = (
            None if self.robot_state_time is None else
            time.monotonic() - self.robot_state_time)
        if robot_state_age is None or robot_state_age > self.status_timeout:
            self._fault('xArm robot state is stale after FT zero')
            return
        if self.robot_error not in (None, 0):
            self._fault(
                f'xArm error {self.robot_error} reported after FT zero; '
                'clear the hardware error before pickup')
            return
        if self.robot_mode != self.ros2_control_mode:
            self._fault(
                f'xArm mode changed to {self.robot_mode} after FT zero; '
                f'expected mode {self.ros2_control_mode}')
            return
        if self.robot_state is not None and self.robot_state <= 2:
            self.get_logger().info(
                'FT zero settle complete; xArm remained motion-ready')
            self._begin_pre_descent_control_wait(on_success)
            return
        if self.robot_state != 5:
            self._fault(
                f'xArm state {self.robot_state} is not motion-ready after '
                'FT zero')
            return
        if not self.set_state_client.service_is_ready():
            self._fault(
                'cannot resume xArm state after FT zero: service unavailable')
            return
        request = SetInt16.Request()
        request.data = 0
        future = self.set_state_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._post_ft_set_state_completed(
                completed, on_success))

    def _post_ft_set_state_completed(self, future, on_success):
        if self.state != self.ARMING_DESCENT:
            return
        if not self._driver_call_ok(future, 'set_state(0) after FT zero'):
            return
        self.get_logger().info(
            'xArm state 0 requested after FT zero; waiting for RobotHW '
            'controller handoff to settle')
        if self._post_ft_state_timer is not None:
            self._post_ft_state_timer.cancel()
        self._post_ft_state_timer = self.create_timer(
            self.post_ft_state_settle,
            lambda: self._post_ft_state_settle_elapsed(on_success))

    def _post_ft_state_settle_elapsed(self, on_success):
        if self._post_ft_state_timer is not None:
            self._post_ft_state_timer.cancel()
            self._post_ft_state_timer = None
        if self.state != self.ARMING_DESCENT:
            return
        self._begin_pre_descent_control_wait(on_success)

    def _begin_pre_descent_control_wait(self, on_success):
        self.pre_descent_wait_callback = on_success
        self.pre_descent_wait_sequence = self.joint_state_sequence
        self.pre_descent_wait_deadline = (
            time.monotonic() + self.joint_state_ready_timeout)
        self.pre_descent_controller_query_pending = False
        self.pre_descent_activation_attempted = False
        self.get_logger().info(
            'waiting for motion-ready xArm state, active controllers, and '
            'fresh /joint_states before pickup descent')

    def _pre_descent_readiness_tick(self):
        if self.pre_descent_wait_callback is None:
            return
        now = time.monotonic()
        if (self.pre_descent_wait_deadline is not None and
                now >= self.pre_descent_wait_deadline):
            self._clear_pre_descent_wait()
            self._fault(
                'xArm/control feedback did not recover after FT zero; '
                f'state={self.robot_state}, mode={self.robot_mode}, '
                f'error={self.robot_error}')
            return
        if (self.robot_state_time is None or
                now - self.robot_state_time > self.status_timeout):
            return
        if self.robot_error not in (None, 0):
            error = self.robot_error
            self._clear_pre_descent_wait()
            self._fault(f'xArm error {error} reported after FT zero')
            return
        if (self.robot_state is None or self.robot_state > 2 or
                self.robot_mode != self.ros2_control_mode):
            return
        if self.pre_descent_controller_query_pending:
            return
        if not self.controller_list_client.service_is_ready():
            return
        self.pre_descent_controller_query_pending = True
        future = self.controller_list_client.call_async(
            ListControllers.Request())
        future.add_done_callback(self._pre_descent_controller_list_completed)

    def _pre_descent_controller_list_completed(self, future):
        self.pre_descent_controller_query_pending = False
        if self.pre_descent_wait_callback is None:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._clear_pre_descent_wait()
            self._fault(f'failed to inspect controllers after FT zero: {exc}')
            return
        states = {
            controller.name: controller.state
            for controller in response.controller
        } if response is not None else {}
        required = (self.joint_state_broadcaster, self.trajectory_controller)
        missing = [name for name in required if states.get(name) != 'active']
        if missing:
            if self.pre_descent_activation_attempted:
                return
            if not self.controller_switch_client.service_is_ready():
                return
            self.pre_descent_activation_attempted = True
            request = SwitchController.Request()
            request.activate_controllers = missing
            request.deactivate_controllers = []
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.activate_asap = True
            request.timeout = Duration(seconds=3.0).to_msg()
            self.pre_descent_controller_query_pending = True
            switch_future = self.controller_switch_client.call_async(request)
            switch_future.add_done_callback(
                self._pre_descent_controller_activation_completed)
            return
        if (self.pre_descent_wait_sequence is None or
                self.joint_state_sequence < (
                    self.pre_descent_wait_sequence +
                    self.joint_state_ready_samples) or
                self.last_joint_state_time is None or
                time.monotonic() - self.last_joint_state_time > 0.25):
            return
        callback = self.pre_descent_wait_callback
        self._clear_pre_descent_wait()
        self.get_logger().info(
            'post-FT-zero xArm state, controllers, and /joint_states '
            'confirmed')
        callback()

    def _pre_descent_controller_activation_completed(self, future):
        self.pre_descent_controller_query_pending = False
        if self.pre_descent_wait_callback is None:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._clear_pre_descent_wait()
            self._fault(f'failed to restore controllers after FT zero: {exc}')
            return
        if result is None or not result.ok:
            self._clear_pre_descent_wait()
            self._fault('controller activation failed after FT zero')

    def _clear_pre_descent_wait(self):
        self.pre_descent_wait_callback = None
        self.pre_descent_wait_sequence = None
        self.pre_descent_wait_deadline = None
        self.pre_descent_controller_query_pending = False
        self.pre_descent_activation_attempted = False

    def _arm_descent(self):
        self.descent_target_z = self.floor_z
        self.expected_enable_generation = int(
            self.servo_status.get('enable_generation', 0)) + 1
        self.state = self.ARMING_DESCENT
        request = SetBool.Request()
        request.data = True
        future = self.enable_client.call_async(request)
        future.add_done_callback(self._arm_completed)

    def retreat_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'pickup already active in {self.state}'
            return response
        try:
            self._tcp_xyz()
        except ValueError as exc:
            response.message = str(exc)
            return response
        self.operation_id += 1
        self.fault = ''
        self.post_retreat_fault = ''
        self._disable_servo_then_direct_retreat()
        response.success = True
        response.message = 'direct vertical retreat to pre-grasp started'
        return response

    def _arm_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'safe-servo enable failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            current_z = float(self.servo_status.get(
                'tcp_z_m', self.pregrasp_z or 0.0))
            descended = (self.pregrasp_z or current_z) - current_z
            if (('force' in message.lower() or 'torque' in message.lower()) and
                    descended >= 0.005):
                self.get_logger().info(
                    f'treating enable rejection as contact after '
                    f'{descended:.3f} m descent: {message}')
                self._handle_contact()
                return
            self._fault(f'safe-servo enable rejected: {message}')
            return
        self.state = self.DESCENDING
        self.descent_started = time.monotonic()
        self._publish_descent_target()

    def _publish_descent_target(self):
        message = Float64MultiArray()
        message.data = [0.0, 0.0, self.descent_target_z * 1000.0, 0.0, 0.0, 0.0]
        self.command_pub.publish(message)

    def _contact_delta_n(self):
        return float(self.servo_status.get(
            'force_limit_n', self.force_threshold))

    def _contact_reached(self):
        if self.servo_status.get('touch_contact'):
            return True
        delta = self.servo_status.get('force_delta_z_n')
        if delta is not None and float(delta) >= self._contact_delta_n():
            return True
        return False

    def _handle_contact(self):
        if self.contact_detected:
            return
        self.contact_detected = True
        self.state = self.DISABLING_DESCENT
        request = SetBool.Request()
        request.data = False
        future = self.enable_client.call_async(request)
        future.add_done_callback(self._contact_disable_completed)

    def _contact_disable_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'safe-servo disable failed after contact: {exc}')
            return
        if result is None or not result.success:
            self._fault('safe-servo did not confirm disable after contact')
            return
        if self.operation_kind == 'place':
            self._turn_vacuum_off()
        else:
            self._turn_vacuum_on()

    def control_tick(self):
        if self._tick_descent_config():
            return
        if self.state == self.DETACHING:
            if not self.planning_scene_status.get('attached_item_id'):
                self.detach_started = None
                self.get_logger().info(
                    'planning scene confirmed carried-item detachment')
                self._disable_servo_then_direct_retreat()
            elif (self.detach_started is not None and
                  time.monotonic() - self.detach_started > 5.0):
                self._retreat_after_vacuum_fault(
                    'timed out waiting for planning-scene detachment')
            return
        if self.state != self.DESCENDING:
            return
        if self.servo_status.get('state') == 'FAULT' or self.servo_status.get('fault'):
            self._fault(f"safe-servo fault: {self.servo_status.get('fault', 'unknown')}")
            return
        self._publish_descent_target()
        if self.dry_run:
            if time.monotonic() - self.descent_started >= 0.20:
                self._handle_contact()
            return
        try:
            current_z = self._tcp_xyz()[2]
        except ValueError as exc:
            self._fault(str(exc))
            return
        descended = (self.pregrasp_z or current_z) - current_z
        current_generation = int(
            self.servo_status.get('enable_generation', -1))
        current_enable_generation = (
            self.expected_enable_generation is not None and
            current_generation >= self.expected_enable_generation)
        if current_enable_generation and self._contact_reached():
            self.get_logger().info(
                f'Fz threshold reached after {descended * 1000.0:.1f} mm; '
                'stopping descent and triggering vacuum')
            self._handle_contact()
            return
        if self.state == self.DESCENDING and current_z <= self.floor_z + self.tolerance:
            self._fault('reached descent floor without contact force')
            return
        if time.monotonic() - self.descent_started > self.descent_timeout:
            self._fault(
                f'continuous vertical descent timed out at Z={current_z:.4f} m '
                f'(floor {self.descent_target_z:.4f} m). '
                'Confirm uf850_traj_controller is active and Servo is unpaused.')

    def retreat_tick(self):
        if self.pre_descent_wait_callback is not None:
            self._pre_descent_readiness_tick()
            return
        if self.state == self.RESTORING_CONTROL:
            self._restore_readiness_tick()
            return
        if (self.state == self.PREPARING_RETREAT and
                self.retreat_controller_wait_deadline is not None):
            self._check_retreat_controllers_inactive()
            return
        if self.state != self.RETREATING:
            return
        if self.retreat_started is None:
            return
        if time.monotonic() - self.retreat_started > self.retreat_timeout:
            self._fault('direct vertical retreat timed out')

    def _disable_servo_then_direct_retreat(self):
        if not self.enable_client.service_is_ready():
            self._begin_direct_retreat()
            return
        self.state = self.DISABLING_SERVO
        request = SetBool.Request()
        request.data = False
        future = self.enable_client.call_async(request)
        future.add_done_callback(self._servo_disabled_for_retreat)

    def _servo_disabled_for_retreat(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'safe-servo disable failed before retreat: {exc}')
            return
        if result is None or not result.success:
            self._fault('safe-servo did not confirm disable before retreat')
            return
        self._begin_direct_retreat()

    def _begin_direct_retreat(self):
        if self.direct_target_z is None:
            self._fault('direct vertical target height is unavailable')
            return
        if self.dry_run:
            self.virtual_z = self.direct_target_z
            self._finish_retreat()
            return
        try:
            self.retreat_start_xyz = self._tcp_xyz()
            self.retreat_start_z = self.retreat_start_xyz[2]
        except ValueError as exc:
            self._fault(f'cannot start direct retreat: {exc}')
            return
        if not self.set_mode_client.service_is_ready():
            self._fault('ufactory set_mode service is unavailable')
            return
        self.state = self.PREPARING_RETREAT
        request = SetInt16.Request()
        request.data = 0
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self._retreat_mode_completed)

    def _retreat_mode_completed(self, future):
        if not self._driver_call_ok(future, 'set_mode(0)'):
            return
        if not self.set_state_client.service_is_ready():
            self._fault('ufactory set_state service is unavailable')
            return
        request = SetInt16.Request()
        request.data = 0
        future = self.set_state_client.call_async(request)
        future.add_done_callback(self._retreat_state_completed)

    def _driver_call_ok(self, future, label):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'{label} failed: {exc}')
            return False
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._fault(f'{label} rejected: ret={code}')
            return False
        return True

    def _retreat_state_completed(self, future):
        if not self._driver_call_ok(future, 'set_state(0)'):
            return
        self.retreat_controller_wait_deadline = (
            time.monotonic() + self.joint_state_ready_timeout)
        self.retreat_controller_query_pending = False
        self.get_logger().info(
            'direct-driver mode requested; waiting for ROS controllers to '
            'become inactive before retreat')

    def _check_retreat_controllers_inactive(self):
        if time.monotonic() >= self.retreat_controller_wait_deadline:
            self.retreat_controller_wait_deadline = None
            self._fault(
                'ROS controllers did not become inactive before direct retreat')
            return
        if self.retreat_controller_query_pending:
            return
        if not self.controller_list_client.service_is_ready():
            self._fault('controller_manager list service is unavailable')
            return
        self.retreat_controller_query_pending = True
        future = self.controller_list_client.call_async(ListControllers.Request())
        future.add_done_callback(self._retreat_controller_list_completed)

    def _retreat_controller_list_completed(self, future):
        self.retreat_controller_query_pending = False
        if (self.state != self.PREPARING_RETREAT or
                self.retreat_controller_wait_deadline is None):
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fault(f'failed to inspect controllers before retreat: {exc}')
            return
        states = {
            controller.name: controller.state
            for controller in response.controller
        } if response is not None else {}
        still_active = [
            name for name in (
                self.joint_state_broadcaster, self.trajectory_controller)
            if states.get(name) == 'active'
        ]
        if still_active:
            return
        self.retreat_controller_wait_deadline = None
        self.get_logger().info(
            'ROS controllers inactive; direct retreat owns the robot')
        self._send_direct_retreat()

    def _send_direct_retreat(self):
        if not self.retreat_client.service_is_ready():
            self._fault('ufactory set_position service is unavailable')
            return
        if self.retreat_start_z is None:
            self._fault('direct retreat start Z snapshot is unavailable')
            return
        current_z = self.retreat_start_z
        absolute_pose = (
            self.direct_target_xyz is not None and
            self.direct_target_rpy is not None)
        if self.direct_target_xyz is None:
            delta = (0.0, 0.0, self.direct_target_z - current_z)
        else:
            start_xyz = self.retreat_start_xyz
            delta = tuple(
                self.direct_target_xyz[index] - start_xyz[index]
                for index in range(3))
        distance_mm = tuple(value * 1000.0 for value in delta)
        if math.sqrt(sum(value * value for value in delta)) <= self.tolerance:
            self._restore_ros2_control_mode()
            return
        request = MoveCartesian.Request()
        request.pose = (
            [*(value * 1000.0 for value in self.direct_target_xyz),
             *self.direct_target_rpy]
            if absolute_pose else [*distance_mm, 0.0, 0.0, 0.0])
        request.speed = self.retreat_speed
        request.acc = self.retreat_acc
        request.mvtime = 0.0
        request.wait = True
        request.relative = not absolute_pose
        self.state = self.RETREATING
        self.retreat_started = time.monotonic()
        self.retreat_target_z = self.direct_target_z
        future = self.retreat_client.call_async(request)
        future.add_done_callback(self._direct_retreat_completed)

    def _direct_retreat_completed(self, future):
        if self.state != self.RETREATING:
            return
        try:
            result = future.result()
        except Exception as exc:
            self.post_retreat_fault = f'set_position retreat failed: {exc}'
            self.get_logger().error(
                f'{self.post_retreat_fault}; restoring ROS 2 control')
            self.retreat_started = None
            self._restore_ros2_control_mode()
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self.post_retreat_fault = (
                f'set_position retreat rejected: ret={code}')
            self.get_logger().error(
                f'{self.post_retreat_fault}; restoring ROS 2 control')
            self.retreat_started = None
            self._restore_ros2_control_mode()
            return
        self.retreat_started = None
        self.get_logger().info('direct vertical Cartesian motion succeeded')
        self._restore_ros2_control_mode()

    def _restore_ros2_control_mode(self):
        if not self.set_mode_client.service_is_ready():
            self._fault('cannot restore ros2_control mode: set_mode unavailable')
            return
        self.state = self.RESTORING_CONTROL
        request = SetInt16.Request()
        request.data = self.ros2_control_mode
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self._restore_mode_completed)

    def _restore_mode_completed(self, future):
        if self.state != self.RESTORING_CONTROL:
            return
        if not self._driver_call_ok(
                future, f'set_mode({self.ros2_control_mode}) restore'):
            return
        if not self.set_state_client.service_is_ready():
            self._fault('cannot restore ros2_control state: set_state unavailable')
            return
        request = SetInt16.Request()
        request.data = 0
        future = self.set_state_client.call_async(request)
        future.add_done_callback(self._restore_state_completed)

    def _restore_state_completed(self, future):
        if self.state != self.RESTORING_CONTROL:
            return
        if not self._driver_call_ok(future, 'set_state(0) restore'):
            return
        self.get_logger().info(
            f'ros2_control robot mode {self.ros2_control_mode} restored')
        self._restore_trajectory_controller()

    def _restore_trajectory_controller(self):
        if not self.controller_list_client.service_is_ready():
            self._fault('controller_manager list service is unavailable')
            return
        future = self.controller_list_client.call_async(ListControllers.Request())
        future.add_done_callback(self._restore_activation_state_received)

    def _restore_activation_state_received(self, future):
        if self.state != self.RESTORING_CONTROL:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fault(f'failed to inspect controllers after retreat: {exc}')
            return
        states = {
            controller.name: controller.state
            for controller in response.controller
        } if response is not None else {}
        required = (self.joint_state_broadcaster, self.trajectory_controller)
        missing = [name for name in required if states.get(name) != 'active']
        if not missing:
            self._begin_post_restore_joint_state_wait()
            return
        if not self.controller_switch_client.service_is_ready():
            self._fault('controller_manager switch service is unavailable')
            return
        request = SwitchController.Request()
        # Direct-driver mode transitions can leave both command and state
        # controllers inactive. MoveIt must not plan until live joint states
        # have been restored as well as trajectory command ownership.
        request.activate_controllers = missing
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = Duration(seconds=3.0).to_msg()
        future = self.controller_switch_client.call_async(request)
        future.add_done_callback(self._restore_controller_completed)

    def _restore_controller_completed(self, future):
        if self.state != self.RESTORING_CONTROL:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'failed to restore trajectory controller: {exc}')
            return
        if result is None or not result.ok:
            self._fault(
                f'failed to activate {self.trajectory_controller} after retreat')
            return
        self._verify_restored_controllers()

    def _verify_restored_controllers(self):
        if not self.controller_list_client.service_is_ready():
            self._fault('controller_manager list service is unavailable')
            return
        future = self.controller_list_client.call_async(ListControllers.Request())
        future.add_done_callback(self._restored_controller_list_completed)

    def _restored_controller_list_completed(self, future):
        if self.state != self.RESTORING_CONTROL:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fault(f'failed to verify restored controllers: {exc}')
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
            self._fault(
                f'controllers inactive after retreat: {inactive}; states={states}')
            return
        self._begin_post_restore_joint_state_wait()

    def _begin_post_restore_joint_state_wait(self):
        self.restore_wait_sequence = self.joint_state_sequence
        self.restore_wait_deadline = (
            time.monotonic() + self.joint_state_ready_timeout)
        self.get_logger().info(
            'controllers restored after retreat; waiting for a new '
            '/joint_states sample before completing the cycle')

    def _restore_readiness_tick(self):
        if self.restore_wait_sequence is None:
            return
        if (self.joint_state_sequence >= (
                self.restore_wait_sequence + self.joint_state_ready_samples) and
                self.last_joint_state_time is not None and
                time.monotonic() - self.last_joint_state_time <= 0.25):
            self.restore_wait_sequence = None
            self.restore_wait_deadline = None
            self.get_logger().info(
                'fresh post-retreat /joint_states confirmed')
            self._finish_retreat()
            return
        if (self.restore_wait_deadline is not None and
                time.monotonic() >= self.restore_wait_deadline):
            self.restore_wait_sequence = None
            self.restore_wait_deadline = None
            self._fault(
                'controllers report active after retreat but /joint_states '
                f'did not resume within {self.joint_state_ready_timeout:.1f} s')

    def _finish_retreat(self):
        if self.post_retreat_fault:
            reason = self.post_retreat_fault
            self.post_retreat_fault = ''
            self._fault(reason)
            return
        self.state = self.SUCCEEDED
        self.publish_status()

    def _proceed_to_retreat(self, vacuum_verified=True):
        self.vacuum_verified = vacuum_verified
        self._disable_servo_then_direct_retreat()

    def _retreat_after_vacuum_fault(self, reason):
        self.get_logger().error(f'{reason}; retreating before reporting fault')
        self.post_retreat_fault = reason
        self._proceed_to_retreat(vacuum_verified=False)

    def _schedule_vacuum_settle(self):
        if self.vacuum_settle_sec > 0:
            if self._vacuum_settle_timer is not None:
                self._vacuum_settle_timer.cancel()
            self._vacuum_settle_timer = self.create_timer(
                self.vacuum_settle_sec, self._vacuum_settle_done)
            return
        self._proceed_to_retreat()

    def _vacuum_settle_done(self):
        if self._vacuum_settle_timer is not None:
            self._vacuum_settle_timer.cancel()
            self._vacuum_settle_timer = None
        if self.state not in (self.VACUUM_ON, self.VERIFYING_VACUUM):
            return
        self._proceed_to_retreat()

    def _turn_vacuum_off(self):
        self.state = self.VACUUM_OFF
        if not self.vacuum_client.service_is_ready():
            self._retreat_after_vacuum_fault(
                'vacuum service is unavailable during placement release')
            return
        request = VacuumGripperCtrl.Request()
        request.on = False
        request.wait = False
        request.timeout = self.vacuum_timeout
        request.delay_sec = 0.0
        request.sync = True
        request.hardware_version = self.vacuum_hardware_version
        future = self.vacuum_client.call_async(request)
        future.add_done_callback(self._vacuum_off_completed)

    def _vacuum_off_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._retreat_after_vacuum_fault(
                f'vacuum release command failed: {exc}')
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._retreat_after_vacuum_fault(
                f'vacuum release command rejected: ret={code}')
            return
        self.vacuum_verified = False
        if not self.detach_item_client.service_is_ready():
            self._retreat_after_vacuum_fault(
                'item released but planning-scene detach service is unavailable')
            return
        self.state = self.DETACHING
        self.detach_started = time.monotonic()
        future = self.detach_item_client.call_async(Trigger.Request())
        future.add_done_callback(self._detach_completed)

    def _detach_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._retreat_after_vacuum_fault(
                f'planning-scene detach failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._retreat_after_vacuum_fault(
                f'planning-scene detach rejected: {message}')
            return
        self.get_logger().info(
            'vacuum released; waiting for planning-scene detach confirmation')

    def _turn_vacuum_on(self):
        self.state = self.VACUUM_ON
        if self.dry_run:
            self.vacuum_verified = True
            self._proceed_to_retreat()
            return
        if not self.vacuum_client.wait_for_service(timeout_sec=3.0):
            self._retreat_after_vacuum_fault(
                'vacuum service is unavailable; restart the stack after enabling '
                'set_vacuum_gripper in config/xarm_user_params.yaml')
            return
        request = VacuumGripperCtrl.Request()
        request.on = True
        request.wait = False
        request.timeout = self.vacuum_timeout
        request.delay_sec = 0.0
        request.sync = True
        request.hardware_version = self.vacuum_hardware_version
        future = self.vacuum_client.call_async(request)
        future.add_done_callback(self._vacuum_on_completed)

    def _vacuum_on_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._retreat_after_vacuum_fault(f'vacuum command failed: {exc}')
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._retreat_after_vacuum_fault(
                f'vacuum command rejected; return code={code}. '
                'Confirm air supply is on and vacuum_hardware_version matches '
                'the gripper wiring.')
            return
        self.vacuum_verify_count = 0
        if not self.require_vacuum_sensor:
            self.get_logger().info(
                'vacuum outputs enabled; retreating after '
                f'{self.vacuum_settle_sec:.1f}s settle')
            self._schedule_vacuum_settle()
            return
        self._request_vacuum_status()

    def _request_vacuum_status(self):
        if not self.vacuum_status_client.service_is_ready():
            self._retreat_after_vacuum_fault(
                'vacuum verification service is unavailable')
            return
        self.state = self.VERIFYING_VACUUM
        future = self.vacuum_status_client.call_async(GetInt16.Request())
        future.add_done_callback(self._vacuum_status_completed)

    def _vacuum_status_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._retreat_after_vacuum_fault(
                f'vacuum verification failed: {exc}')
            return
        if (result is not None and result.ret == 0 and
                int(result.data) == self.vacuum_on_status):
            self.get_logger().info('vacuum pressure sensor confirmed suction')
            self._schedule_vacuum_settle()
            return
        self.vacuum_verify_count += 1
        if self.vacuum_verify_count >= self.vacuum_verify_attempts:
            value = None if result is None else result.data
            self._retreat_after_vacuum_fault(
                f'vacuum sensor status={value} after '
                f'{self.vacuum_verify_count} attempts')
            return
        self.get_logger().info(
            f'vacuum not ready yet ({self.vacuum_verify_count}/'
            f'{self.vacuum_verify_attempts}); retrying')
        if self._vacuum_retry_timer is not None:
            self._vacuum_retry_timer.cancel()
        self._vacuum_retry_timer = self.create_timer(
            self.vacuum_verify_interval, self._vacuum_verify_retry)

    def _vacuum_verify_retry(self):
        if self._vacuum_retry_timer is not None:
            self._vacuum_retry_timer.cancel()
            self._vacuum_retry_timer = None
        if self.state != self.VERIFYING_VACUUM:
            return
        self._request_vacuum_status()

    def _fault(self, reason):
        if self.state == self.FAULT:
            return
        self.fault = reason
        self.state = self.FAULT
        self.restore_wait_sequence = None
        self.restore_wait_deadline = None
        self.retreat_controller_wait_deadline = None
        self.retreat_controller_query_pending = False
        self.descent_config_deadline = None
        self.descent_config_last_publish = None
        self._clear_pre_descent_wait()
        if self._ft_settle_timer is not None:
            self._ft_settle_timer.cancel()
            self._ft_settle_timer = None
        if self._post_ft_state_timer is not None:
            self._post_ft_state_timer.cancel()
            self._post_ft_state_timer = None
        request = SetBool.Request()
        request.data = False
        if self.enable_client.service_is_ready():
            self.enable_client.call_async(request)
        self.get_logger().error(reason)
        self.publish_status()

    def abort_callback(self, _request, response):
        if self.state not in self.ACTIVE:
            response.message = f'no active pickup in state {self.state}'
            return response
        self._fault('pickup aborted by operator')
        response.success = True
        response.message = 'pickup aborted; Servo disable requested'
        return response

    def reset_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'cannot reset active pickup in {self.state}'
            return response
        self.state = self.IDLE
        self.fault = ''
        self.pregrasp_z = None
        self.direct_target_z = None
        self.direct_target_xyz = None
        self.direct_target_rpy = None
        self.floor_z = None
        self.virtual_z = None
        self.descent_target_z = None
        self.vacuum_verified = False
        self.vacuum_verify_count = 0
        self.contact_detected = False
        self.expected_enable_generation = None
        self.operation_kind = 'pickup'
        self.detach_started = None
        self.retreat_started = None
        self.retreat_target_z = None
        self.retreat_start_z = None
        self.retreat_start_xyz = None
        self.post_retreat_fault = ''
        self.restore_wait_sequence = None
        self.restore_wait_deadline = None
        self.retreat_controller_wait_deadline = None
        self.retreat_controller_query_pending = False
        self.descent_config_deadline = None
        self.descent_config_last_publish = None
        self._clear_pre_descent_wait()
        if self._ft_settle_timer is not None:
            self._ft_settle_timer.cancel()
            self._ft_settle_timer = None
        if self._post_ft_state_timer is not None:
            self._post_ft_state_timer.cancel()
            self._post_ft_state_timer = None
        response.success = True
        response.message = 'pickup supervisor reset to IDLE'
        return response

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state,
            'operation_kind': self.operation_kind,
            'fault': self.fault,
            'dry_run': self.dry_run,
            'pregrasp_z_m': self.pregrasp_z,
            'floor_z_m': self.floor_z,
            'descent_target_z_m': self.descent_target_z,
            'retreat_target_z_m': self.retreat_target_z,
            'contact_detected': self.contact_detected,
            'configured_contact_threshold_n':
                self._configured_contact_threshold(),
            'configured_servo_speed_scale': self._configured_speed_scale(),
            'vacuum_verified': self.vacuum_verified,
            'operation_id': self.operation_id,
            'joint_state_age_sec': (
                None if self.last_joint_state_time is None else
                time.monotonic() - self.last_joint_state_time),
            'robot_state': self.robot_state,
            'robot_mode': self.robot_mode,
            'robot_error': self.robot_error,
        }, separators=(',', ':'))
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PickupSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
