import json
import math
import time

from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState,
    SwitchController)
from geometry_msgs.msg import PoseStamped, WrenchStamped
from lifecycle_msgs.msg import State
from moveit_msgs.msg import ServoStatus
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
    STOPPING_LOADING = 'STOPPING_LOADING'
    PREPARING_RETREAT = 'PREPARING_RETREAT'
    RETREATING = 'RETREATING'
    WAITING_PLACE_STEP_FEEDBACK = 'WAITING_PLACE_STEP_FEEDBACK'
    RESTORING_CONTROL = 'RESTORING_CONTROL'
    SUCCEEDED = 'SUCCEEDED'
    FAULT = 'FAULT'

    ACTIVE = {
        ARMING_DESCENT, DESCENDING, DISABLING_DESCENT, DISABLING_SERVO,
        VACUUM_ON, VACUUM_OFF, DETACHING, VERIFYING_VACUUM,
        STOPPING_LOADING, PREPARING_RETREAT, RETREATING,
        WAITING_PLACE_STEP_FEEDBACK, RESTORING_CONTROL,
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
        self.declare_parameter('place_descent_timeout_sec', 20.0)
        self.declare_parameter('singularity_place_recovery_timeout_sec', 20.0)
        self.declare_parameter('singularity_place_step_m', 0.003)
        self.declare_parameter('singularity_place_step_speed_mm_s', 10.0)
        self.declare_parameter('retreat_timeout_sec', 120.0)
        self.declare_parameter('retreat_speed_mm_s', 30.0)
        self.declare_parameter('retreat_acc_mm_s2', 200.0)
        self.declare_parameter('ros2_control_mode', 1)
        self.declare_parameter('trajectory_controller', 'uf850_traj_controller')
        self.declare_parameter('joint_state_broadcaster', 'joint_state_broadcaster')
        self.declare_parameter(
            'hardware_component',
            'uf_robot_hardware/UFRobotSystemHardware')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_state_ready_timeout_sec', 5.0)
        self.declare_parameter('joint_state_ready_samples', 3)
        self.declare_parameter('mode_transition_timeout_sec', 5.0)
        self.declare_parameter('mode_retry_interval_sec', 0.25)
        self.declare_parameter('mode_ready_samples', 3)
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
        self.declare_parameter('force_contact_threshold_n', 5.0)
        self.declare_parameter('place_force_contact_threshold_n', 4.0)
        self.declare_parameter(
            'force_topic', '/ufactory/uf_ftsensor_ext_states')
        self.declare_parameter('force_timeout_sec', 0.5)
        self.declare_parameter('loading_contact_confirm_samples', 2)
        # Keep pickup at 20 mm/s with the 50 mm/s bridge cap.
        self.declare_parameter('servo_speed_scale', 0.4)
        self.declare_parameter('place_servo_speed_scale', 1.0)
        self.declare_parameter('workspace_x_min_mm', 160.0)
        self.declare_parameter('workspace_x_max_mm', 390.0)
        self.declare_parameter('workspace_y_min_mm', -360.0)
        self.declare_parameter('workspace_y_max_mm', 360.0)
        self.declare_parameter('workspace_z_min_mm', 50.0)
        self.declare_parameter('workspace_z_max_mm', 800.0)
        self.declare_parameter('place_workspace_z_min_mm', -100.0)
        self.declare_parameter('pre_place_clearance_m', 0.03)
        self.declare_parameter('transfer_corner_height_m', 0.47)
        self.declare_parameter('joint6_name', 'joint6')
        self.declare_parameter('joint6_moveit_lower_rad', -2.0 * math.pi)
        self.declare_parameter('joint6_moveit_upper_rad', 2.0 * math.pi)
        self.declare_parameter('joint6_limit_margin_rad', 0.02)

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
        self.place_descent_timeout = float(p('place_descent_timeout_sec'))
        self.singularity_place_recovery_timeout = float(
            p('singularity_place_recovery_timeout_sec'))
        self.singularity_place_step = float(p('singularity_place_step_m'))
        self.singularity_place_step_speed = float(
            p('singularity_place_step_speed_mm_s'))
        self.retreat_timeout = float(p('retreat_timeout_sec'))
        self.retreat_speed = float(p('retreat_speed_mm_s'))
        self.retreat_acc = float(p('retreat_acc_mm_s2'))
        self.ros2_control_mode = int(p('ros2_control_mode'))
        self.trajectory_controller = str(p('trajectory_controller'))
        self.joint_state_broadcaster = str(p('joint_state_broadcaster'))
        self.hardware_component = str(p('hardware_component'))
        self.joint_state_topic = str(p('joint_state_topic'))
        self.joint_state_ready_timeout = float(p('joint_state_ready_timeout_sec'))
        self.joint_state_ready_samples = int(p('joint_state_ready_samples'))
        self.mode_transition_timeout = float(p('mode_transition_timeout_sec'))
        self.mode_retry_interval = float(p('mode_retry_interval_sec'))
        self.mode_ready_samples = int(p('mode_ready_samples'))
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
        self.force_topic = str(p('force_topic'))
        self.force_timeout = float(p('force_timeout_sec'))
        self.loading_contact_confirm_samples = int(
            p('loading_contact_confirm_samples'))
        self.servo_speed_scale = float(p('servo_speed_scale'))
        self.place_servo_speed_scale = float(p('place_servo_speed_scale'))
        self.servo_bounds_mm = (
            float(p('workspace_x_min_mm')), float(p('workspace_x_max_mm')),
            float(p('workspace_y_min_mm')), float(p('workspace_y_max_mm')),
            float(p('workspace_z_min_mm')), float(p('workspace_z_max_mm')),
        )
        self.place_workspace_z_min_mm = float(p('place_workspace_z_min_mm'))
        self.pre_place_clearance = float(p('pre_place_clearance_m'))
        self.transfer_corner_height = float(p('transfer_corner_height_m'))
        self.joint6_name = str(p('joint6_name'))
        self.joint6_limits = (
            float(p('joint6_moveit_lower_rad')),
            float(p('joint6_moveit_upper_rad')))
        self.joint6_limit_margin = float(p('joint6_limit_margin_rad'))
        if not 0.0 <= self.contact_search_margin <= self.max_descent:
            raise ValueError(
                'contact_search_margin_m must be within '
                f'[0, max_descent_m={self.max_descent:.3f}]')
        if self.force_threshold <= 0.0:
            raise ValueError('force_contact_threshold_n must be positive')
        if self.place_force_threshold <= 0.0:
            raise ValueError('place_force_contact_threshold_n must be positive')
        if self.force_timeout <= 0.0:
            raise ValueError('force_timeout_sec must be positive')
        if self.loading_contact_confirm_samples < 1:
            raise ValueError('loading_contact_confirm_samples must be at least 1')
        if not (0.01 <= self.servo_speed_scale <= 1.0 and
                0.01 <= self.place_servo_speed_scale <= 1.0):
            raise ValueError('Servo speed scales must be within [0.01, 1.0]')
        if self.joint_state_ready_timeout <= 0.0:
            raise ValueError('joint_state_ready_timeout_sec must be positive')
        if self.joint_state_ready_samples < 1:
            raise ValueError('joint_state_ready_samples must be at least 1')
        if self.mode_transition_timeout <= 0.0:
            raise ValueError('mode_transition_timeout_sec must be positive')
        if self.mode_retry_interval <= 0.0:
            raise ValueError('mode_retry_interval_sec must be positive')
        if self.mode_ready_samples < 1:
            raise ValueError('mode_ready_samples must be at least 1')
        if self.place_descent_timeout <= 0.0:
            raise ValueError('place_descent_timeout_sec must be positive')
        if self.singularity_place_recovery_timeout <= 0.0:
            raise ValueError(
                'singularity_place_recovery_timeout_sec must be positive')
        if not 0.0005 <= self.singularity_place_step <= 0.010:
            raise ValueError('singularity_place_step_m must be within [0.0005, 0.010]')
        if self.singularity_place_step_speed <= 0.0:
            raise ValueError(
                'singularity_place_step_speed_mm_s must be positive')
        if not self.joint6_limits[0] < self.joint6_limits[1]:
            raise ValueError('joint6 MoveIt lower limit must be below upper limit')
        if not 0.0 <= self.joint6_limit_margin < (
                self.joint6_limits[1] - self.joint6_limits[0]) / 2.0:
            raise ValueError('joint6_limit_margin_rad is invalid')
        if self.ft_zero_settle < 0.2:
            raise ValueError('ft_zero_settle_sec must be at least 0.2')
        if self.post_ft_state_settle <= 0.0:
            raise ValueError('post_ft_state_settle_sec must be positive')
        if self.place_workspace_z_min_mm >= self.servo_bounds_mm[5]:
            raise ValueError(
                'place_workspace_z_min_mm must be below workspace_z_max_mm')
        if self.transfer_corner_height <= 0.0:
            raise ValueError('transfer_corner_height_m must be positive')
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
        self.direct_target_pose = None
        self.direct_transfer_succeeded = False
        self.transfer_fallback_reason = ''
        self.place_target_z = None
        self.place_target_xyz = None
        self.rotate_item_90 = False
        self.pre_place_pose = None
        self.post_retreat_fault = ''
        self.dry_run = True
        self.vacuum_verified = False
        self.vacuum_verify_count = 0
        self.manual_gripper_pending = False
        self.manual_gripper_state = 'unknown'
        self._vacuum_retry_timer = None
        self._vacuum_settle_timer = None
        self.operation_id = 0
        self.contact_detected = False
        self.place_fallback_used = False
        self.place_fallback_reason = ''
        self.latest_force_z = None
        self.last_force_time = None
        self.loading_force_baseline_z = None
        self.loading_force_over_count = 0
        self.loading_transfer_z = None
        self.loading_contact_fallback = False
        self.loading_stop_started = None
        self.loading_stop_action = ''
        self.direct_place_recovery_active = False
        self.direct_place_stepping = False
        self.direct_place_deadline = None
        self.direct_place_force_baseline_z = None
        self.direct_place_step_count = 0
        self.direct_place_step_completed_at = None
        self.direct_motion_generation = 0
        self.expected_enable_generation = None
        self.operation_kind = 'pickup'
        self.planning_scene_status = {}
        self.orchestrator_status = {}
        self.pallet_locked = False
        self.detach_started = None
        self.last_joint_state_time = None
        self.joint_state_sequence = 0
        self.joint6_position = None
        self.restore_wait_sequence = None
        self.restore_wait_deadline = None
        self.retreat_controller_wait_deadline = None
        self.retreat_controller_query_pending = False
        self.mode_wait_target = None
        self.mode_wait_label = ''
        self.mode_wait_callback = None
        self.mode_wait_deadline = None
        self.mode_wait_last_command = None
        self.mode_wait_command_pending = False
        self.mode_wait_clear_pending = False
        self.mode_wait_ready_count = 0
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
        for topic in (
                '/pickup_pipeline/status', '/place_pipeline/status',
                '/pick_place_pipeline/status'):
            self.create_subscription(
                String, topic,
                lambda message, name=topic: self.orchestrator_status_callback(
                    name, message), 10)
        self.create_subscription(
            Float64MultiArray, '/pallet_localization/config_state',
            self.pallet_config_callback, 10)
        self.create_subscription(
            PoseStamped, '/pallet_localization/pre_place_pose',
            self.pre_place_pose_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/motion_speed/config',
            self.motion_speed_config_callback, 10)
        self.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 10)
        self.create_subscription(
            RobotMsg, '/ufactory/robot_states', self.robot_state_callback, 10)
        self.create_subscription(
            WrenchStamped, self.force_topic, self.force_callback, 10)
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
        self.clean_error_client = self.create_client(
            Call, '/ufactory/clean_error')
        self.retreat_client = self.create_client(
            MoveCartesian, '/ufactory/set_position')
        self.controller_switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.controller_list_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.hardware_list_client = self.create_client(
            ListHardwareComponents,
            '/controller_manager/list_hardware_components')
        self.hardware_state_client = self.create_client(
            SetHardwareComponentState,
            '/controller_manager/set_hardware_component_state')
        self.detach_item_client = self.create_client(
            Trigger, '/planning_scene_obstacles/detach_item')
        self.create_service(Trigger, '/pickup_supervisor/start', self.start_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_place', self.start_place_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_loading', self.start_loading_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_transfer_fallback',
            self.start_transfer_fallback_callback)
        self.create_service(
            SetBool, '/pickup_supervisor/set_gripper',
            self.set_gripper_callback)
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

    def joint_state_callback(self, message):
        self.last_joint_state_time = time.monotonic()
        self.joint_state_sequence += 1
        try:
            index = message.name.index(self.joint6_name)
            position = float(message.position[index])
        except (ValueError, IndexError, TypeError):
            return
        if math.isfinite(position):
            self.joint6_position = position

    def robot_state_callback(self, message):
        self.robot_state = int(message.state)
        self.robot_mode = int(message.mode)
        self.robot_error = int(message.err)
        self.robot_state_time = time.monotonic()
        if len(message.pose) >= 3:
            xyz = tuple(float(value) / 1000.0 for value in message.pose[:3])
            if all(math.isfinite(value) for value in xyz):
                self.robot_tcp_xyz = xyz

    def force_callback(self, message):
        force_z = float(message.wrench.force.z)
        if not math.isfinite(force_z):
            return
        self.latest_force_z = force_z
        self.last_force_time = time.monotonic()
        if (self.operation_kind != 'loading' or
                self.state != self.RETREATING or
                self.loading_contact_fallback or
                self.loading_force_baseline_z is None or
                self.retreat_start_z is None or
                self.retreat_target_z is None or
                self.retreat_target_z >= self.retreat_start_z - self.tolerance):
            return
        delta_fz = abs(force_z - self.loading_force_baseline_z)
        if delta_fz < self.place_force_threshold:
            self.loading_force_over_count = 0
            return
        self.loading_force_over_count += 1
        if self.loading_force_over_count >= self.loading_contact_confirm_samples:
            self._begin_loading_contact_fallback(delta_fz)

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

    def orchestrator_status_callback(self, name, message):
        try:
            self.orchestrator_status[name] = json.loads(message.data)
        except (TypeError, ValueError):
            self.orchestrator_status[name] = {}

    def _manual_control_busy_reason(self):
        if self.state in self.ACTIVE:
            return f'supervisor is active in {self.state}'
        motion_state = self.motion_status.get('state')
        if motion_state not in (None, 'IDLE', 'SUCCEEDED', 'FAULT'):
            return f'MoveIt coordinator is in {motion_state}'
        for name, status in self.orchestrator_status.items():
            state = status.get('state')
            if state not in (None, 'IDLE', 'SUCCEEDED', 'FAULT'):
                return f'{name} is in {state}'
        return ''

    def pallet_config_callback(self, message):
        if len(message.data) >= 12:
            self.place_target_xyz = tuple(
                float(value) / 1000.0 for value in message.data[9:12])
            self.place_target_z = self.place_target_xyz[2]
        if len(message.data) >= 13:
            self.rotate_item_90 = bool(message.data[12] > 0.5)

    def pre_place_pose_callback(self, message):
        if message.header.frame_id == 'link_base':
            self.pre_place_pose = message.pose

    @staticmethod
    def _quat_multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @classmethod
    def _quat_rotate(cls, vector, quaternion):
        qx, qy, qz, qw = quaternion
        return cls._quat_multiply(
            cls._quat_multiply(quaternion, (*vector, 0.0)),
            (-qx, -qy, -qz, qw))[:3]

    @staticmethod
    def _quaternion_from_yaw(yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    @staticmethod
    def _rpy_from_quaternion(quaternion):
        x, y, z, w = map(float, quaternion)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm <= 1e-9:
            raise ValueError('transfer target has an invalid quaternion')
        x, y, z, w = (value / norm for value in (x, y, z, w))
        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y))
        pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(pitch_term)
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def _transfer_corner_base_z(self, object_x):
        if self.pre_place_pose is None or self.place_target_xyz is None:
            raise ValueError('pallet-frame transfer height is unavailable')
        q = self.pre_place_pose.orientation
        object_q = (float(q.x), float(q.y), float(q.z), float(q.w))
        inverse_item_yaw = self._quaternion_from_yaw(
            math.pi / 2.0 if self.rotate_item_90 else 0.0)
        pallet_q = self._quat_multiply(object_q, inverse_item_yaw)
        local_pre_place = (
            self.place_target_xyz[0],
            self.place_target_xyz[1],
            self.place_target_xyz[2] + self.pre_place_clearance)
        pre_place_offset_z = self._quat_rotate(
            local_pre_place, pallet_q)[2]
        pallet_origin_z = (
            float(self.pre_place_pose.position.z) - pre_place_offset_z)
        transfer_offset_z = self._quat_rotate(
            (self.place_target_xyz[0], self.place_target_xyz[1],
             self.transfer_corner_height),
            pallet_q)[2]
        correction_z = 0.0
        if self.rotate_item_90:
            correction_z = self._quat_rotate(
                (-float(object_x), 0.0, 0.0), object_q)[2]
        return pallet_origin_z + transfer_offset_z + correction_z

    def _pickup_retreat_target_tcp_z(self, object_height, object_x):
        return (
            self._transfer_corner_base_z(object_x) +
            float(object_height) + self.grasp_offset)

    def motion_speed_config_callback(self, message):
        if len(message.data) < 2:
            self.get_logger().warning(
                'ignoring incomplete motion speed configuration')
            return
        speed = float(message.data[1])
        if not math.isfinite(speed) or not 5.0 <= speed <= 100.0:
            self.get_logger().warning(
                f'ignoring service speed outside 5..100 mm/s: {speed}')
            return
        self.retreat_speed = speed
        self.get_logger().info(
            f'non-servo service speed set to {self.retreat_speed:.1f} mm/s')

    def set_gripper_callback(self, request, response):
        action = 'close' if request.data else 'open'
        busy_reason = self._manual_control_busy_reason()
        if busy_reason:
            response.message = f'cannot {action} gripper: {busy_reason}'
            return response
        if self.manual_gripper_pending:
            response.message = 'a gripper command is already pending'
            return response
        if self.dry_run:
            self.manual_gripper_state = f'dry-run {action}'
            response.success = True
            response.message = f'dry run: gripper {action} simulated'
            self.publish_status()
            return response
        if not self.vacuum_client.service_is_ready():
            response.message = 'vacuum gripper service is unavailable'
            return response
        command = VacuumGripperCtrl.Request()
        command.on = bool(request.data)
        command.wait = False
        command.timeout = self.vacuum_timeout
        command.delay_sec = 0.0
        command.sync = True
        command.hardware_version = self.vacuum_hardware_version
        self.manual_gripper_pending = True
        self.manual_gripper_state = f'{action} pending'
        future = self.vacuum_client.call_async(command)
        future.add_done_callback(
            lambda done: self._manual_gripper_completed(
                done, bool(request.data)))
        response.success = True
        response.message = f'gripper {action} requested'
        self.publish_status()
        return response

    def _manual_gripper_completed(self, future, close):
        action = 'close' if close else 'open'
        self.manual_gripper_pending = False
        try:
            result = future.result()
        except Exception as exc:
            self.manual_gripper_state = f'{action} failed: {exc}'
            self.get_logger().error(self.manual_gripper_state)
            self.publish_status()
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self.manual_gripper_state = f'{action} rejected: ret={code}'
            self.get_logger().error(self.manual_gripper_state)
            self.publish_status()
            return
        self.manual_gripper_state = 'closed' if close else 'open'
        self.get_logger().info(f'gripper manually {self.manual_gripper_state}')
        if not close and self.planning_scene_status.get('attached_item_id'):
            if self.detach_item_client.service_is_ready():
                future = self.detach_item_client.call_async(Trigger.Request())
                future.add_done_callback(self._manual_detach_completed)
            else:
                self.manual_gripper_state = (
                    'open; warning: attached planning-scene item remains')
                self.get_logger().error(self.manual_gripper_state)
        self.publish_status()

    def _manual_detach_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.manual_gripper_state = f'open; scene detach failed: {exc}'
            self.get_logger().error(self.manual_gripper_state)
            self.publish_status()
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self.manual_gripper_state = (
                f'open; scene detach rejected: {message}')
            self.get_logger().error(self.manual_gripper_state)
        else:
            self.manual_gripper_state = 'open; scene item detached'
        self.publish_status()

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
        # A Servo fault is deliberately latched by the bridge after it is
        # disabled.  Do not reject a new pickup using that previous
        # operation's fault: start_callback() always calls reset_fault before
        # it publishes a new descent configuration or enables Servo.  The
        # reset response is only successful after the bridge has cleared the
        # fault and published its disabled IDLE state.
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

    def _direct_mode_tcp_xyz(self):
        if (self.robot_tcp_xyz is not None and
                self.robot_state_time is not None and
                time.monotonic() - self.robot_state_time <= self.status_timeout):
            return self.robot_tcp_xyz
        return self._tcp_xyz()

    def _joint6_safe_bounds(self):
        return (
            self.joint6_limits[0] + self.joint6_limit_margin,
            self.joint6_limits[1] - self.joint6_limit_margin)

    def _joint6_is_moveit_safe(self):
        if self.joint6_position is None or not math.isfinite(self.joint6_position):
            return False
        lower, upper = self._joint6_safe_bounds()
        return lower <= self.joint6_position <= upper

    def _publish_servo_config(self, touch_mode, bypass_force=False):
        is_place = self.operation_kind == 'place'
        force = (self.place_force_threshold if is_place else
                 self.force_threshold)
        speed_scale = (self.place_servo_speed_scale if is_place else
                       self.servo_speed_scale)
        z_min = (self.place_workspace_z_min_mm if is_place else
                 self.servo_bounds_mm[4])
        message = Float64MultiArray()
        message.data = [
            speed_scale,
            self.servo_bounds_mm[0], self.servo_bounds_mm[1],
            self.servo_bounds_mm[2], self.servo_bounds_mm[3],
            z_min, self.servo_bounds_mm[5],
            force,
            1.0 if touch_mode else 0.0,
            1.0 if bypass_force else 0.0,
            1.0 if is_place else 0.0,
        ]
        self.config_pub.publish(message)

    def start_callback(self, _request, response):
        if self.manual_gripper_pending:
            response.message = 'wait for the pending gripper command'
            return response
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
        try:
            self.direct_target_z = self._pickup_retreat_target_tcp_z(
                object_height, float(snapshot['size_x_m']))
        except ValueError as exc:
            response.message = str(exc)
            return response
        # The grasped box bottom is nominally object_height + grasp_offset
        # below link_tcp.  Target the absolute TCP Z that puts that corner at
        # the same pallet-frame height used by the transfer waypoint.
        if self.direct_target_z > self.servo_bounds_mm[5] / 1000.0:
            response.message = (
                f'pickup retreat TCP Z {self.direct_target_z:.3f} m for '
                f'pallet corner height {self.transfer_corner_height:.3f} m exceeds '
                'the configured workspace ceiling')
            return response
        self.operation_id += 1
        self.operation_kind = 'pickup'
        self.fault = ''
        self.post_retreat_fault = ''
        self.direct_target_pose = None
        self.direct_transfer_succeeded = False
        self.transfer_fallback_reason = ''
        self.place_fallback_used = False
        self.place_fallback_reason = ''
        self.direct_place_recovery_active = False
        self.direct_place_stepping = False
        self.direct_place_deadline = None
        self.direct_place_force_baseline_z = None
        self.direct_place_step_count = 0
        self.direct_place_step_completed_at = None
        self.loading_contact_fallback = False
        self.loading_stop_started = None
        self.loading_stop_action = ''
        self.loading_force_baseline_z = None
        self.loading_force_over_count = 0
        self.loading_transfer_z = None
        self.vacuum_verified = False
        self.vacuum_verify_count = 0
        self.contact_detected = False
        self.pregrasp_z = z
        self.floor_z = floor_z
        self.virtual_z = z
        self.state = self.ARMING_DESCENT
        stale_servo_fault = str(self.servo_status.get('fault') or '').strip()
        if stale_servo_fault:
            self.get_logger().warning(
                'clearing latched safe-servo fault before pickup: '
                f'{stale_servo_fault}')
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
        if self.manual_gripper_pending:
            response.message = 'wait for the pending gripper command'
            return response
        if self.state in self.ACTIVE:
            response.message = f'supervisor already active in {self.state}'
            return response
        if (self.motion_status.get('target') != 'transfer' or
                (self.motion_status.get('state') != 'SUCCEEDED' and
                 not self.direct_transfer_succeeded)):
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
        self.direct_target_pose = None
        self.place_fallback_used = False
        self.place_fallback_reason = ''
        self.direct_place_recovery_active = False
        self.direct_place_stepping = False
        self.direct_place_deadline = None
        self.direct_place_force_baseline_z = None
        self.direct_place_step_count = 0
        self.direct_place_step_completed_at = None
        self.loading_stop_started = None
        self.loading_stop_action = ''
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
        servo_floor_z = self.place_workspace_z_min_mm / 1000.0
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

    def start_transfer_fallback_callback(self, _request, response):
        if self.manual_gripper_pending:
            response.message = 'wait for the pending gripper command'
            return response
        if self.state in self.ACTIVE:
            response.message = f'supervisor already active in {self.state}'
            return response
        if (self.motion_status.get('state') != 'FAULT' or
                self.motion_status.get('target') != 'transfer'):
            response.message = (
                'direct transfer fallback requires a failed MoveIt transfer plan')
            return response
        if not self.pallet_locked:
            response.message = 'pallet pose is not LOCKED'
            return response
        if not self.planning_scene_status.get('attached_item_id'):
            response.message = 'no carried item is attached in the planning scene'
            return response
        try:
            xyz = tuple(map(float, self.motion_status['transfer_tcp_xyz_m']))
            quaternion = tuple(map(
                float,
                self.motion_status['transfer_tcp_quaternion_xyzw']))
            if len(xyz) != 3 or len(quaternion) != 4:
                raise ValueError('unexpected target vector length')
            if not all(math.isfinite(value) for value in (*xyz, *quaternion)):
                raise ValueError('non-finite target value')
            rpy = self._rpy_from_quaternion(quaternion)
            self.direct_target_pose = (
                xyz[0] * 1000.0, xyz[1] * 1000.0, xyz[2] * 1000.0,
                *rpy)
        except (KeyError, TypeError, ValueError) as exc:
            response.message = f'direct transfer target is unavailable: {exc}'
            return response
        self.operation_id += 1
        self.operation_kind = 'transfer'
        self.fault = ''
        self.post_retreat_fault = ''
        self.direct_transfer_succeeded = False
        self.transfer_fallback_reason = str(
            self.motion_status.get('fault') or 'MoveIt transfer planning failed')
        self.direct_target_z = xyz[2]
        self.get_logger().warning(
            'MoveIt could not plan the transfer; bypassing MoveIt collision '
            'checking and using the direct xArm Cartesian service to the '
            f'validated transfer TCP target [{xyz[0]:.3f}, {xyz[1]:.3f}, '
            f'{xyz[2]:.3f}] m')
        self._disable_servo_then_direct_retreat()
        response.success = True
        response.message = (
            'direct xArm transfer fallback started; collision checking is bypassed')
        self.publish_status()
        return response

    def start_loading_callback(self, _request, response):
        if self.manual_gripper_pending:
            response.message = 'wait for the pending gripper command'
            return response
        if self.state in self.ACTIVE:
            response.message = f'supervisor already active in {self.state}'
            return response
        if (self.motion_status.get('target') != 'transfer' or
                (self.motion_status.get('state') != 'SUCCEEDED' and
                 not self.direct_transfer_succeeded)):
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
        if not self.dry_run and (
                self.last_force_time is None or
                time.monotonic() - self.last_force_time > self.force_timeout or
                self.latest_force_z is None):
            response.message = 'force telemetry is stale before linear loading'
            return response
        self.operation_id += 1
        self.operation_kind = 'loading'
        self.fault = ''
        self.post_retreat_fault = ''
        self.direct_target_pose = None
        self.place_fallback_used = False
        self.place_fallback_reason = ''
        self.direct_place_recovery_active = False
        self.direct_place_stepping = False
        self.direct_place_deadline = None
        self.direct_place_force_baseline_z = None
        self.direct_place_step_count = 0
        self.direct_place_step_completed_at = None
        self.loading_contact_fallback = False
        self.loading_stop_started = None
        self.loading_stop_action = ''
        self.loading_force_over_count = 0
        self.loading_force_baseline_z = self.latest_force_z
        self.loading_transfer_z = current_z
        self.pregrasp_z = target_z
        self.direct_target_z = target_z
        self._disable_servo_then_direct_retreat()
        response.success = True
        response.message = (
            f'force-guarded linear loading started: TCP Z {current_z:.3f} -> '
            f'{target_z:.3f} m, delta-Fz threshold='
            f'{self.place_force_threshold:.1f} N')
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
        return self.force_threshold

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
        if self.manual_gripper_pending:
            response.message = 'wait for the pending gripper command'
            return response
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
        self._disable_descent_for_gripper()

    def _disable_descent_for_gripper(self):
        self.state = self.DISABLING_DESCENT
        request = SetBool.Request()
        request.data = False
        future = self.enable_client.call_async(request)
        future.add_done_callback(self._contact_disable_completed)

    @staticmethod
    def _is_servo_singularity_fault(reason):
        normalized = str(reason).lower()
        return ('moveit servo halted' in normalized and
                'singular' in normalized)

    def _servo_is_singularity_decelerating(self):
        try:
            status = int(self.servo_status.get('servo_status'))
        except (TypeError, ValueError):
            return False
        return status in (
            ServoStatus.DECELERATE_FOR_APPROACHING_SINGULARITY,
            ServoStatus.DECELERATE_FOR_LEAVING_SINGULARITY)

    def _begin_place_singularity_fallback(self, reason):
        if self.direct_place_recovery_active or self.place_fallback_used:
            return
        self.place_fallback_used = True
        self.place_fallback_reason = str(reason)
        self.direct_place_recovery_active = True
        self.direct_place_stepping = False
        self.direct_place_force_baseline_z = self.latest_force_z
        self.direct_place_step_count = 0
        self.direct_place_step_completed_at = None
        # Servo can spend its full descent timeout decelerating near a
        # singularity. Give the guarded 3 mm recovery its own bounded window
        # so the first direct step is not already timed out.
        self.direct_place_deadline = (
            time.monotonic() + self.singularity_place_recovery_timeout)
        self.get_logger().warning(
            'place Servo reached its singularity recovery condition; '
            'switching to '
            f'{self.singularity_place_step * 1000.0:.1f} mm direct vertical '
            'steps with force checks between steps')
        if not self.enable_client.service_is_ready():
            self._fault(
                'safe-servo disable service is unavailable before direct '
                'singularity recovery')
            return
        self.state = self.DISABLING_SERVO
        request = SetBool.Request()
        request.data = False
        future = self.enable_client.call_async(request)
        future.add_done_callback(self._servo_disabled_for_direct_place)
        self.publish_status()

    def _servo_disabled_for_direct_place(self, future):
        if not self.direct_place_recovery_active:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(
                f'safe-servo disable failed before direct place recovery: {exc}')
            return
        if result is None or not result.success:
            self._fault(
                'safe-servo did not confirm disable before direct place recovery')
            return
        if self.dry_run:
            self._release_from_direct_place(
                'dry-run singularity recovery completed', contact_detected=True)
            return
        try:
            self.retreat_start_xyz = self._direct_mode_tcp_xyz()
            self.retreat_start_z = self.retreat_start_xyz[2]
        except ValueError as exc:
            self._fault(f'cannot start direct place recovery: {exc}')
            return
        self.direct_place_stepping = True
        self.state = self.PREPARING_RETREAT
        self._deactivate_retreat_controllers()

    def _direct_place_force_delta(self):
        if (self.direct_place_force_baseline_z is None or
                self.latest_force_z is None):
            return None
        return abs(
            float(self.latest_force_z) -
            float(self.direct_place_force_baseline_z))

    def _send_direct_place_step(self):
        if not self.direct_place_stepping:
            return
        now = time.monotonic()
        if (self.direct_place_deadline is not None and
                now >= self.direct_place_deadline):
            self._release_from_direct_place(
                f'direct singularity recovery reached the '
                f'{self.singularity_place_recovery_timeout:.1f} s recovery '
                'timeout')
            return
        if (self.last_force_time is None or
                now - self.last_force_time > self.force_timeout):
            self._release_from_direct_place(
                'force telemetry became stale during direct singularity recovery')
            return
        force_delta = self._direct_place_force_delta()
        if force_delta is not None and force_delta >= self.place_force_threshold:
            self._release_from_direct_place(
                f'direct singularity recovery detected contact: '
                f'delta_fz={force_delta:.2f} N', contact_detected=True)
            return
        try:
            current_z = self._direct_mode_tcp_xyz()[2]
        except ValueError as exc:
            self._release_from_direct_place(
                f'cannot read TCP during direct singularity recovery: {exc}')
            return
        remaining = current_z - self.floor_z
        if remaining <= self.tolerance:
            self._release_from_direct_place(
                'direct singularity recovery reached the configured place floor')
            return
        if not self.retreat_client.service_is_ready():
            self._release_from_direct_place(
                'ufactory set_position service became unavailable during '
                'direct singularity recovery')
            return
        step = min(self.singularity_place_step, remaining)
        request = MoveCartesian.Request()
        request.pose = [0.0, 0.0, -step * 1000.0, 0.0, 0.0, 0.0]
        request.speed = self.singularity_place_step_speed
        request.acc = self.retreat_acc
        request.mvtime = 0.0
        request.wait = True
        request.timeout = max(2.0, step * 1000.0 /
                              self.singularity_place_step_speed + 1.0)
        request.relative = True
        self.state = self.RETREATING
        self.retreat_started = now
        self.retreat_target_z = current_z - step
        self.direct_motion_generation += 1
        generation = self.direct_motion_generation
        future = self.retreat_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._direct_place_step_completed(
                completed, generation))

    def _direct_place_step_completed(self, future, generation):
        if (not self.direct_place_stepping or
                self.state != self.RETREATING or
                generation != self.direct_motion_generation):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._release_from_direct_place(
                f'direct 3 mm place step failed: {exc}')
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._release_from_direct_place(
                f'direct 3 mm place step was rejected: ret={code}')
            return
        self.direct_place_step_count += 1
        self.retreat_started = None
        self.direct_place_step_completed_at = time.monotonic()
        self.state = self.WAITING_PLACE_STEP_FEEDBACK

    def _direct_place_step_feedback_tick(self):
        if not self.direct_place_stepping:
            return
        now = time.monotonic()
        completed_at = self.direct_place_step_completed_at
        if completed_at is None:
            self._release_from_direct_place(
                'direct singularity recovery lost its step completion time')
            return
        if (self.direct_place_deadline is not None and
                now >= self.direct_place_deadline):
            self._release_from_direct_place(
                f'direct singularity recovery reached the '
                f'{self.singularity_place_recovery_timeout:.1f} s recovery '
                'timeout')
            return
        fresh_force = (
            self.last_force_time is not None and
            self.last_force_time > completed_at)
        fresh_tcp = (
            self.robot_state_time is not None and
            self.robot_state_time > completed_at)
        if fresh_force and fresh_tcp:
            self._send_direct_place_step()
            return
        feedback_timeout = max(self.force_timeout, self.status_timeout)
        if now - completed_at > feedback_timeout:
            missing = []
            if not fresh_force:
                missing.append('force')
            if not fresh_tcp:
                missing.append('TCP')
            self._release_from_direct_place(
                'post-step telemetry timed out during direct singularity '
                f'recovery: missing {", ".join(missing)}')

    def _release_from_direct_place(self, reason, contact_detected=False):
        if not self.direct_place_recovery_active:
            return
        self.direct_place_stepping = False
        self.direct_place_step_completed_at = None
        self.contact_detected = bool(contact_detected)
        self.place_fallback_used = True
        self.place_fallback_reason = str(reason)
        self.retreat_started = None
        self.get_logger().warning(
            f'{reason}; releasing the item, retreating to the transfer '
            'waypoint, and returning to observation')
        self.publish_status()
        self._turn_vacuum_off()

    def _begin_place_release_fallback(self, reason, cause):
        if self.place_fallback_used:
            return
        self.place_fallback_used = True
        self.place_fallback_reason = str(reason)
        self.get_logger().warning(
            f'place descent {cause} fallback: release '
            'the item, retreat vertically to the recorded transfer waypoint, '
            'then allow the place pipeline to return to observation')
        self._disable_descent_for_gripper()
        self.publish_status()

    def _begin_loading_contact_fallback(self, delta_fz):
        self._begin_loading_release_fallback(
            f'contact during linear loading: delta_fz={delta_fz:.2f} N, '
            f'threshold={self.place_force_threshold:.2f} N',
            contact_detected=True)

    def _begin_loading_release_fallback(self, reason, contact_detected=False):
        if self.loading_contact_fallback:
            return
        self.loading_contact_fallback = True
        self.contact_detected = bool(contact_detected)
        self.place_fallback_used = True
        self.place_fallback_reason = str(reason)
        # Invalidate the outstanding downward set_position response before
        # stopping it. A late response must never be mistaken for completion
        # of the subsequent upward fallback motion.
        self.direct_motion_generation += 1
        self.retreat_started = None
        self.state = self.STOPPING_LOADING
        self.loading_stop_started = time.monotonic()
        self.loading_stop_action = 'release'
        self.get_logger().warning(
            f'{self.place_fallback_reason}; stopping descent, releasing the '
            'item, and returning through the transfer waypoint')
        self.publish_status()
        self._request_loading_stop()

    def _begin_loading_completion_stop(self):
        if self.state != self.RETREATING:
            return
        self.direct_motion_generation += 1
        self.retreat_started = None
        self.state = self.STOPPING_LOADING
        self.loading_stop_started = time.monotonic()
        self.loading_stop_action = 'complete'
        self.get_logger().info(
            'pre-place Z reached; stopping the non-blocking xArm command '
            'before restoring ros2_control')
        self.publish_status()
        self._request_loading_stop()

    def _request_loading_stop(self):
        if not self.set_state_client.service_is_ready():
            self._fault(
                'cannot stop linear loading motion: '
                'ufactory set_state service is unavailable')
            return
        request = SetInt16.Request()
        request.data = 3
        future = self.set_state_client.call_async(request)
        future.add_done_callback(self._loading_pause_completed)

    def _loading_pause_completed(self, future):
        if self.state != self.STOPPING_LOADING:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'failed to stop linear loading: {exc}')
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._fault(
                f'xArm rejected linear loading stop: ret={code}')
            return
        self.loading_stop_started = None
        action = self.loading_stop_action
        self.loading_stop_action = ''
        if action == 'complete':
            self.get_logger().info(
                'xArm confirmed pre-place loading stop; restoring ROS 2 control')
            self._restore_ros2_control_mode()
            return
        if action != 'release':
            self._fault('linear loading stopped without a pending completion action')
            return
        if self.loading_transfer_z is None:
            self._fault('loading fallback transfer height is unavailable')
            return
        self.direct_target_z = self.loading_transfer_z
        self._turn_vacuum_off()

    def _contact_disable_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'safe-servo disable failed before gripper action: {exc}')
            return
        if result is None or not result.success:
            self._fault('safe-servo did not confirm disable before gripper action')
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
                self._proceed_to_retreat()
            elif (self.detach_started is not None and
                  time.monotonic() - self.detach_started > 5.0):
                self._retreat_after_vacuum_fault(
                    'timed out waiting for planning-scene detachment')
            return
        if self.state != self.DESCENDING:
            return
        if self.servo_status.get('state') == 'FAULT' or self.servo_status.get('fault'):
            reason = f"safe-servo fault: {self.servo_status.get('fault', 'unknown')}"
            if (self.operation_kind == 'place' and
                    self._is_servo_singularity_fault(reason)):
                self._begin_place_singularity_fallback(reason)
            else:
                self._fault(reason)
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
        descent_timeout = (
            self.place_descent_timeout
            if self.operation_kind == 'place' else self.descent_timeout)
        if time.monotonic() - self.descent_started > descent_timeout:
            reason = (
                f'continuous vertical descent timed out at Z={current_z:.4f} m '
                f'(floor {self.descent_target_z:.4f} m). '
                'Confirm uf850_traj_controller is active and Servo is unpaused.')
            if self.operation_kind == 'place':
                if self._servo_is_singularity_decelerating():
                    status = int(self.servo_status['servo_status'])
                    reason = (
                        f'{reason} MoveIt Servo remained in singularity '
                        f'deceleration status {status}.')
                    self._begin_place_singularity_fallback(reason)
                else:
                    self._begin_place_release_fallback(reason, 'timeout')
            else:
                self._fault(reason)

    def retreat_tick(self):
        if self.pre_descent_wait_callback is not None:
            self._pre_descent_readiness_tick()
            return
        if self.mode_wait_target is not None:
            self._mode_readiness_tick()
            return
        if self.state == self.STOPPING_LOADING:
            if (self.loading_stop_started is not None and
                    time.monotonic() - self.loading_stop_started >
                    self.place_descent_timeout):
                self.loading_stop_started = None
                self._fault(
                    'timed out waiting for xArm to stop linear loading; '
                    'automatic release was blocked because motion stop was '
                    'not confirmed')
            return
        if self.state == self.WAITING_PLACE_STEP_FEEDBACK:
            self._direct_place_step_feedback_tick()
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
        if (self.operation_kind == 'loading' and
                not self.loading_contact_fallback):
            try:
                current_z = self._direct_mode_tcp_xyz()[2]
            except ValueError as exc:
                self._fault(f'cannot monitor linear loading: {exc}')
                return
            if current_z <= self.direct_target_z + self.tolerance:
                self._begin_loading_completion_stop()
                return
            if (time.monotonic() - self.retreat_started >
                    self.place_descent_timeout):
                self._begin_loading_release_fallback(
                    f'linear loading exceeded '
                    f'{self.place_descent_timeout:.1f} s before reaching '
                    f'pre-place Z {self.direct_target_z:.4f} m')
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
            self.retreat_start_xyz = self._direct_mode_tcp_xyz()
            self.retreat_start_z = self.retreat_start_xyz[2]
        except ValueError as exc:
            self._fault(f'cannot start direct retreat: {exc}')
            return
        self.state = self.PREPARING_RETREAT
        self._deactivate_retreat_controllers()

    def _deactivate_retreat_controllers(self):
        if not self.controller_list_client.service_is_ready():
            self._fault('controller_manager list service is unavailable')
            return
        future = self.controller_list_client.call_async(
            ListControllers.Request())
        future.add_done_callback(self._retreat_controller_state_received)

    def _retreat_controller_state_received(self, future):
        if self.state != self.PREPARING_RETREAT:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fault(
                f'failed to inspect controllers before direct retreat: {exc}')
            return
        states = {
            controller.name: controller.state
            for controller in response.controller
        } if response is not None else {}
        required = (self.trajectory_controller, self.joint_state_broadcaster)
        active = [name for name in required if states.get(name) == 'active']
        if not active:
            self._deactivate_retreat_hardware()
            return
        if not self.controller_switch_client.service_is_ready():
            self._fault('controller_manager switch service is unavailable')
            return
        request = SwitchController.Request()
        request.activate_controllers = []
        request.deactivate_controllers = active
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(seconds=3.0).to_msg()
        switch_future = self.controller_switch_client.call_async(request)
        switch_future.add_done_callback(
            self._retreat_controllers_deactivated)

    def _retreat_controllers_deactivated(self, future):
        if self.state != self.PREPARING_RETREAT:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(
                f'failed to deactivate controllers before retreat: {exc}')
            return
        if result is None or not result.ok:
            self._fault(
                'failed to deactivate ROS controllers before direct retreat')
            return
        self.get_logger().info(
            'ROS controllers explicitly deactivated before direct retreat')
        self._deactivate_retreat_hardware()

    def _deactivate_retreat_hardware(self):
        if not self.hardware_state_client.service_is_ready():
            self._fault(
                'controller_manager hardware-state service is unavailable')
            return
        request = SetHardwareComponentState.Request()
        request.name = self.hardware_component
        request.target_state.id = State.PRIMARY_STATE_INACTIVE
        request.target_state.label = 'inactive'
        future = self.hardware_state_client.call_async(request)
        future.add_done_callback(self._retreat_hardware_deactivated)

    def _retreat_hardware_deactivated(self, future):
        if self.state != self.PREPARING_RETREAT:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(
                f'failed to deactivate robot hardware before retreat: {exc}')
            return
        if (result is None or not result.ok or
                result.state.id != State.PRIMARY_STATE_INACTIVE):
            state = None if result is None else result.state.label
            self._fault(
                'robot hardware did not enter inactive state before direct '
                f'retreat; state={state}')
            return
        self.get_logger().info(
            'ros2_control hardware inactive; Servo-J writes are stopped')
        self._begin_mode_wait(
            0, 'direct retreat', self._begin_retreat_ownership_check)

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

    def _begin_retreat_ownership_check(self):
        self.retreat_controller_wait_deadline = (
            time.monotonic() + self.joint_state_ready_timeout)
        self.retreat_controller_query_pending = False
        self.get_logger().info(
            'live telemetry confirms direct mode 0; verifying exclusive '
            'ownership before retreat')

    def _begin_mode_wait(self, target_mode, label, on_ready):
        self.mode_wait_target = int(target_mode)
        self.mode_wait_label = str(label)
        self.mode_wait_callback = on_ready
        self.mode_wait_deadline = (
            time.monotonic() + self.mode_transition_timeout)
        self.mode_wait_last_command = None
        self.mode_wait_command_pending = False
        self.mode_wait_clear_pending = False
        self.mode_wait_ready_count = 0
        self.get_logger().info(
            f'waiting for live xArm telemetry to confirm mode '
            f'{self.mode_wait_target}, motion-ready state, and zero errors '
            f'before {self.mode_wait_label}')
        self._mode_readiness_tick()

    def _clear_mode_wait(self):
        self.mode_wait_target = None
        self.mode_wait_label = ''
        self.mode_wait_callback = None
        self.mode_wait_deadline = None
        self.mode_wait_last_command = None
        self.mode_wait_command_pending = False
        self.mode_wait_clear_pending = False
        self.mode_wait_ready_count = 0

    def _mode_readiness_tick(self):
        if self.mode_wait_target is None:
            return
        now = time.monotonic()
        if self.mode_wait_deadline is not None and now >= self.mode_wait_deadline:
            target = self.mode_wait_target
            label = self.mode_wait_label
            state = self.robot_state
            mode = self.robot_mode
            error = self.robot_error
            self._clear_mode_wait()
            self._fault(
                f'timed out confirming xArm mode {target} before {label}; '
                f'state={state}, mode={mode}, error={error}')
            return
        telemetry_fresh = (
            self.robot_state_time is not None and
            now - self.robot_state_time <= self.status_timeout)
        if telemetry_fresh and self.robot_error not in (None, 0):
            self.mode_wait_ready_count = 0
            if self.robot_error == 52:
                self._request_mode_wait_error_clear(now)
                return
            error = self.robot_error
            label = self.mode_wait_label
            self._clear_mode_wait()
            self._fault(
                f'xArm error {error} reported while preparing {label}')
            return
        ready = (
            telemetry_fresh and self.robot_error == 0 and
            self.robot_mode == self.mode_wait_target and
            self.robot_state is not None and self.robot_state <= 2)
        if ready:
            self.mode_wait_ready_count += 1
            if self.mode_wait_ready_count < self.mode_ready_samples:
                return
            target = self.mode_wait_target
            label = self.mode_wait_label
            callback = self.mode_wait_callback
            self._clear_mode_wait()
            self.get_logger().info(
                f'live xArm telemetry confirmed mode {target} for {label}')
            callback()
            return
        self.mode_wait_ready_count = 0
        if (not self.mode_wait_command_pending and
                not self.mode_wait_clear_pending and
                (self.mode_wait_last_command is None or
                 now - self.mode_wait_last_command >=
                 self.mode_retry_interval)):
            self._request_mode_wait_command(now)

    def _request_mode_wait_error_clear(self, now):
        if self.mode_wait_clear_pending:
            return
        if (self.mode_wait_last_command is not None and
                now - self.mode_wait_last_command < self.mode_retry_interval):
            return
        if not self.clean_error_client.service_is_ready():
            label = self.mode_wait_label
            self._clear_mode_wait()
            self._fault(
                f'cannot clear xArm C52 before {label}: service unavailable')
            return
        self.mode_wait_clear_pending = True
        self.mode_wait_last_command = now
        future = self.clean_error_client.call_async(Call.Request())
        future.add_done_callback(self._mode_wait_error_clear_completed)

    def _mode_wait_error_clear_completed(self, future):
        if self.mode_wait_target is None:
            return
        self.mode_wait_clear_pending = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'xArm C52 clear attempt failed: {exc}; retrying')
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self.get_logger().warning(
                f'xArm C52 clear attempt returned {code}; retrying')
            return
        self.mode_wait_last_command = time.monotonic()
        self.get_logger().info(
            'xArm C52 cleared; reissuing mode/state transition')

    def _request_mode_wait_command(self, now):
        if (not self.set_mode_client.service_is_ready() or
                not self.set_state_client.service_is_ready()):
            label = self.mode_wait_label
            self._clear_mode_wait()
            self._fault(
                f'cannot prepare {label}: xArm mode/state service unavailable')
            return
        self.mode_wait_command_pending = True
        self.mode_wait_last_command = now
        request = SetInt16.Request()
        request.data = self.mode_wait_target
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self._mode_wait_mode_completed)

    def _mode_wait_mode_completed(self, future):
        if self.mode_wait_target is None:
            return
        try:
            result = future.result()
        except Exception as exc:
            self.mode_wait_command_pending = False
            self.get_logger().warning(
                f'xArm mode transition attempt failed: {exc}; retrying')
            return
        if result is None or result.ret != 0:
            self.mode_wait_command_pending = False
            code = None if result is None else result.ret
            self.get_logger().warning(
                f'xArm set_mode({self.mode_wait_target}) returned {code}; '
                'retrying')
            return
        request = SetInt16.Request()
        request.data = 0
        future = self.set_state_client.call_async(request)
        future.add_done_callback(self._mode_wait_state_completed)

    def _mode_wait_state_completed(self, future):
        if self.mode_wait_target is None:
            return
        self.mode_wait_command_pending = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'xArm state transition attempt failed: {exc}; retrying')
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self.get_logger().warning(
                f'xArm set_state(0) returned {code}; retrying')

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
            'ROS controllers inactive; direct xArm service owns the robot')
        if self.direct_place_stepping:
            self._send_direct_place_step()
        else:
            self._send_direct_retreat()

    def _send_direct_retreat(self):
        if not self.retreat_client.service_is_ready():
            self._fault('ufactory set_position service is unavailable')
            return
        if self.retreat_start_z is None:
            self._fault('direct retreat start Z snapshot is unavailable')
            return
        request = MoveCartesian.Request()
        if self.operation_kind == 'transfer':
            if self.direct_target_pose is None:
                self._fault('direct transfer TCP pose is unavailable')
                return
            # set_position uses millimetres for XYZ and radians for RPY. Mode
            # 1 asks the controller to prefer a Cartesian line, then use its
            # own joint-space IK if a linear solution is unavailable.
            request.pose = list(self.direct_target_pose)
            request.relative = False
            request.motion_type = 1
            request.timeout = self.retreat_timeout
            request.radius = -1.0
        else:
            current_z = self.retreat_start_z
            delta = (0.0, 0.0, self.direct_target_z - current_z)
            distance_mm = tuple(value * 1000.0 for value in delta)
            if math.sqrt(sum(value * value for value in delta)) <= self.tolerance:
                self._restore_ros2_control_mode()
                return
            request.pose = [*distance_mm, 0.0, 0.0, 0.0]
            request.relative = True
        request.speed = self.retreat_speed
        request.acc = self.retreat_acc
        request.mvtime = 0.0
        nonblocking_loading_descent = (
            self.operation_kind == 'loading' and
            not self.loading_contact_fallback and
            self.direct_target_z < self.retreat_start_z - self.tolerance)
        # The downward loading move must not occupy xarm_api's service callback:
        # force or timeout handling needs /ufactory/set_state to remain callable.
        # Its completion is monitored from live TCP Z in retreat_tick().
        request.wait = not nonblocking_loading_descent
        self.state = self.RETREATING
        self.retreat_started = time.monotonic()
        self.retreat_target_z = self.direct_target_z
        self.direct_motion_generation += 1
        generation = self.direct_motion_generation
        future = self.retreat_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._direct_retreat_completed(
                completed, generation))

    def _direct_retreat_completed(self, future, generation):
        if (self.state != self.RETREATING or
                generation != self.direct_motion_generation):
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
        if (self.operation_kind == 'loading' and
                not self.loading_contact_fallback):
            self.get_logger().info(
                'non-blocking linear loading command accepted; monitoring '
                'TCP Z, force, and the 20 s place timeout')
            return
        self.retreat_started = None
        self.get_logger().info(
            f'direct {self.operation_kind} Cartesian motion succeeded')
        self._restore_ros2_control_mode()

    def _restore_ros2_control_mode(self):
        self.state = self.RESTORING_CONTROL
        self._inspect_restore_hardware_state()

    def _inspect_restore_hardware_state(self):
        if not self.hardware_list_client.service_is_ready():
            self._fault(
                'controller_manager hardware-list service is unavailable')
            return
        future = self.hardware_list_client.call_async(
            ListHardwareComponents.Request())
        future.add_done_callback(self._restore_hardware_state_received)

    def _restore_hardware_state_received(self, future):
        if self.state != self.RESTORING_CONTROL:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fault(f'failed to inspect robot hardware after retreat: {exc}')
            return
        components = {
            component.name: component
            for component in response.component
        } if response is not None else {}
        component = components.get(self.hardware_component)
        if component is None:
            self._fault(
                f'robot hardware component not found: {self.hardware_component}')
            return
        state_id = component.state.id
        if state_id == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info(
                'ros2_control hardware unexpectedly active after direct '
                'retreat; deactivating it before restoring robot mode')
            self._set_restore_hardware_state(
                State.PRIMARY_STATE_INACTIVE, 'inactive',
                self._restore_hardware_deactivated_for_mode)
            return
        if state_id == State.PRIMARY_STATE_UNCONFIGURED:
            self._set_restore_hardware_state(
                State.PRIMARY_STATE_INACTIVE, 'inactive',
                self._restore_hardware_configured)
            return
        if state_id == State.PRIMARY_STATE_INACTIVE:
            self._begin_restore_mode_wait()
            return
        self._fault(
            'robot hardware is in an unsupported lifecycle state after '
            f'retreat: id={state_id}, label={component.state.label}')

    def _set_restore_hardware_state(
            self, state_id, state_label, callback):
        if not self.hardware_state_client.service_is_ready():
            self._fault(
                'controller_manager hardware-state service is unavailable')
            return
        request = SetHardwareComponentState.Request()
        request.name = self.hardware_component
        request.target_state.id = state_id
        request.target_state.label = state_label
        future = self.hardware_state_client.call_async(request)
        future.add_done_callback(callback)

    def _restore_hardware_configured(self, future):
        if self.state != self.RESTORING_CONTROL:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'failed to configure robot hardware: {exc}')
            return
        if (result is None or not result.ok or
                result.state.id != State.PRIMARY_STATE_INACTIVE):
            state = None if result is None else result.state.label
            self._fault(
                f'robot hardware configuration failed; state={state}')
            return
        self.get_logger().info(
            'ros2_control hardware recovered from unconfigured to inactive')
        self._begin_restore_mode_wait()

    def _restore_hardware_deactivated_for_mode(self, future):
        if self.state != self.RESTORING_CONTROL:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(
                f'failed to deactivate robot hardware before mode restore: '
                f'{exc}')
            return
        if (result is None or not result.ok or
                result.state.id != State.PRIMARY_STATE_INACTIVE):
            state = None if result is None else result.state.label
            self._fault(
                'robot hardware did not become inactive before mode restore; '
                f'state={state}')
            return
        self._begin_restore_mode_wait()

    def _activate_restore_hardware(self):
        self._set_restore_hardware_state(
            State.PRIMARY_STATE_ACTIVE, 'active',
            self._restore_hardware_activated)

    def _restore_hardware_activated(self, future):
        if self.state != self.RESTORING_CONTROL:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'failed to activate robot hardware: {exc}')
            return
        if (result is None or not result.ok or
                result.state.id != State.PRIMARY_STATE_ACTIVE):
            state = None if result is None else result.state.label
            self._fault(f'robot hardware activation failed; state={state}')
            return
        self.get_logger().info(
            'ros2_control hardware lifecycle is active in confirmed mode 1; '
            'restoring controllers')
        self._restore_trajectory_controller()

    def _begin_restore_mode_wait(self):
        self._begin_mode_wait(
            self.ros2_control_mode, 'ros2_control restoration',
            self._activate_restore_hardware)

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
        if not self.dry_run and not self._joint6_is_moveit_safe():
            lower, upper = self._joint6_safe_bounds()
            position = self.joint6_position
            rendered = 'unavailable' if position is None else f'{position:+.3f} rad'
            self._fault(
                f'direct {self.operation_kind} motion left {self.joint6_name} '
                f'at {rendered}, outside the MoveIt-safe interval '
                f'[{lower:+.3f}, {upper:+.3f}] rad; loading/MoveIt motion '
                'has been blocked')
            return
        if self.operation_kind == 'transfer':
            if not self.dry_run:
                try:
                    current_xyz = self._direct_mode_tcp_xyz()
                except ValueError as exc:
                    self._fault(
                        f'cannot verify direct transfer target: {exc}')
                    return
                target_xyz = tuple(
                    value / 1000.0 for value in self.direct_target_pose[:3])
                xy_error = math.hypot(
                    current_xyz[0] - target_xyz[0],
                    current_xyz[1] - target_xyz[1])
                z_error = abs(current_xyz[2] - target_xyz[2])
                if (xy_error > self.xy_tolerance or
                        z_error > self.pregrasp_z_tolerance):
                    self._fault(
                        'direct transfer service returned success but TCP '
                        f'target error is too large: xy={xy_error * 1000.0:.1f} '
                        f'mm, z={z_error * 1000.0:.1f} mm')
                    return
            self.direct_transfer_succeeded = True
        if self.operation_kind == 'place':
            self.direct_place_recovery_active = False
            self.direct_place_stepping = False
            self.direct_place_deadline = None
            self.direct_place_force_baseline_z = None
            self.direct_place_step_completed_at = None
        self.state = self.SUCCEEDED
        self.publish_status()

    def _proceed_to_retreat(self, vacuum_verified=True):
        self.vacuum_verified = vacuum_verified
        if self.operation_kind == 'loading' and self.loading_contact_fallback:
            self._resume_loading_contact_retreat()
            return
        if self.operation_kind == 'place' and self.direct_place_recovery_active:
            self._resume_direct_place_recovery_retreat()
            return
        self._disable_servo_then_direct_retreat()

    def _resume_direct_place_recovery_retreat(self):
        if self.direct_target_z is None:
            self._fault('place recovery transfer height is unavailable')
            return
        try:
            self.retreat_start_xyz = self._direct_mode_tcp_xyz()
            self.retreat_start_z = self.retreat_start_xyz[2]
        except ValueError as exc:
            self._fault(f'cannot start place recovery retreat: {exc}')
            return
        self.direct_place_stepping = False
        self.direct_place_step_completed_at = None
        self.state = self.PREPARING_RETREAT
        # Direct stepping already deactivated ros2_control and owns mode 0.
        # Reconfirm the live mode/state before reversing upward, without a
        # redundant hardware lifecycle transition.
        self._begin_mode_wait(
            0, 'direct place recovery retreat', self._send_direct_retreat)

    def _resume_loading_contact_retreat(self):
        if self.direct_target_z is None:
            self._fault('loading fallback transfer height is unavailable')
            return
        try:
            self.retreat_start_xyz = self._direct_mode_tcp_xyz()
            self.retreat_start_z = self.retreat_start_xyz[2]
        except ValueError as exc:
            self._fault(f'cannot start loading fallback retreat: {exc}')
            return
        self.state = self.PREPARING_RETREAT
        # The interrupted loading move already owns the robot in direct mode
        # with ros2_control inactive. Resume state 0 and reverse vertically;
        # do not perform a redundant hardware lifecycle transition.
        self._begin_mode_wait(
            0, 'loading contact fallback retreat', self._send_direct_retreat)

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
        self.direct_motion_generation += 1
        self.direct_place_stepping = False
        self.fault = reason
        self.state = self.FAULT
        self.restore_wait_sequence = None
        self.restore_wait_deadline = None
        self.retreat_controller_wait_deadline = None
        self.retreat_controller_query_pending = False
        self._clear_mode_wait()
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
        self.direct_target_pose = None
        self.direct_transfer_succeeded = False
        self.transfer_fallback_reason = ''
        self.floor_z = None
        self.virtual_z = None
        self.descent_target_z = None
        self.vacuum_verified = False
        self.vacuum_verify_count = 0
        self.contact_detected = False
        self.place_fallback_used = False
        self.place_fallback_reason = ''
        self.direct_place_recovery_active = False
        self.direct_place_stepping = False
        self.direct_place_deadline = None
        self.direct_place_force_baseline_z = None
        self.direct_place_step_count = 0
        self.direct_place_step_completed_at = None
        self.loading_contact_fallback = False
        self.loading_stop_started = None
        self.loading_stop_action = ''
        self.loading_force_baseline_z = None
        self.loading_force_over_count = 0
        self.loading_transfer_z = None
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
        self._clear_mode_wait()
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
            'transfer_corner_height_pallet_m': self.transfer_corner_height,
            'place_workspace_z_min_m': self.place_workspace_z_min_mm / 1000.0,
            'place_descent_timeout_sec': self.place_descent_timeout,
            'singularity_place_recovery_timeout_sec':
                self.singularity_place_recovery_timeout,
            'contact_detected': self.contact_detected,
            'place_fallback_used': self.place_fallback_used,
            'place_fallback_reason': self.place_fallback_reason,
            'direct_place_recovery_active': self.direct_place_recovery_active,
            'direct_place_step_count': self.direct_place_step_count,
            'singularity_place_step_mm':
                self.singularity_place_step * 1000.0,
            'direct_transfer_succeeded': self.direct_transfer_succeeded,
            'transfer_fallback_reason': self.transfer_fallback_reason,
            'loading_contact_fallback': self.loading_contact_fallback,
            'loading_force_delta_z_n': (
                None if self.latest_force_z is None or
                self.loading_force_baseline_z is None else
                abs(self.latest_force_z - self.loading_force_baseline_z)),
            'configured_contact_threshold_n':
                self._configured_contact_threshold(),
            'configured_servo_speed_scale': self._configured_speed_scale(),
            'vacuum_verified': self.vacuum_verified,
            'manual_gripper_pending': self.manual_gripper_pending,
            'manual_gripper_state': self.manual_gripper_state,
            'operation_id': self.operation_id,
            'joint_state_age_sec': (
                None if self.last_joint_state_time is None else
                time.monotonic() - self.last_joint_state_time),
            'joint6_position_rad': self.joint6_position,
            'joint6_moveit_safe': self._joint6_is_moveit_safe(),
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
