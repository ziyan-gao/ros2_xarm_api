import json
import math
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState,
    SwitchController)
from geometry_msgs.msg import PoseStamped, WrenchStamped
from lifecycle_msgs.msg import State
from moveit_msgs.msg import RobotState, ServoStatus
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
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
    AWAITING_GRASP = 'AWAITING_GRASP'
    VACUUM_ON = 'VACUUM_ON'
    VACUUM_OFF = 'VACUUM_OFF'
    DETACHING = 'DETACHING'
    VERIFYING_VACUUM = 'VERIFYING_VACUUM'
    DISABLING_SERVO = 'DISABLING_SERVO'
    STOPPING_LOADING = 'STOPPING_LOADING'
    SOLVING_TRANSFER_IK = 'SOLVING_TRANSFER_IK'
    VALIDATING_TRANSFER = 'VALIDATING_TRANSFER'
    CHECKING_LOADING_PATH = 'CHECKING_LOADING_PATH'
    EXECUTING_TRANSFER = 'EXECUTING_TRANSFER'
    PREPARING_RETREAT = 'PREPARING_RETREAT'
    RETREATING = 'RETREATING'
    WAITING_PLACE_STEP_FEEDBACK = 'WAITING_PLACE_STEP_FEEDBACK'
    RESTORING_CONTROL = 'RESTORING_CONTROL'
    C52_STOPPING = 'C52_STOPPING'
    RECOVERING_FT = 'RECOVERING_FT'
    SUCCEEDED = 'SUCCEEDED'
    FAULT = 'FAULT'

    ACTIVE = {
        ARMING_DESCENT, DESCENDING, DISABLING_DESCENT, AWAITING_GRASP,
        DISABLING_SERVO,
        VACUUM_ON, VACUUM_OFF, DETACHING, VERIFYING_VACUUM,
        STOPPING_LOADING, PREPARING_RETREAT, RETREATING,
        WAITING_PLACE_STEP_FEEDBACK, RESTORING_CONTROL,
        SOLVING_TRANSFER_IK, VALIDATING_TRANSFER, CHECKING_LOADING_PATH, EXECUTING_TRANSFER,
        C52_STOPPING, RECOVERING_FT,
    }

    def __init__(self):
        super().__init__('pickup_supervisor')
        self.declare_parameter('refined_boxes_topic', '/pointcloud_detection/boxes')
        self.declare_parameter('target_box_id', -1)
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('max_pregrasp_plan_age_sec', 1800.0)
        self.declare_parameter('grasp_offset_m', 0.0)
        self.declare_parameter('contact_reference_z_m', 0.0)
        self.declare_parameter('minimum_measured_object_height_m', 0.02)
        self.declare_parameter('maximum_measured_object_height_m', 0.45)
        self.declare_parameter('contact_search_margin_m', 0.015)
        self.declare_parameter('max_descent_m', 0.15)
        self.declare_parameter('position_tolerance_m', 0.001)
        self.declare_parameter('xy_tolerance_m', 0.015)
        self.declare_parameter('pregrasp_z_tolerance_m', 0.015)
        self.declare_parameter(
            'retrieval_pregrasp_above_tolerance_m', 0.050)
        self.declare_parameter('minimum_contact_descent_m', 0.005)
        self.declare_parameter('descent_timeout_sec', 30.0)
        self.declare_parameter('place_descent_timeout_sec', 20.0)
        self.declare_parameter('singularity_place_recovery_timeout_sec', 20.0)
        self.declare_parameter('singularity_place_step_m', 0.003)
        self.declare_parameter('singularity_place_step_speed_mm_s', 10.0)
        self.declare_parameter('singularity_deceleration_grace_sec', 0.75)
        self.declare_parameter('singularity_no_progress_sec', 1.0)
        self.declare_parameter('singularity_min_progress_m', 0.001)
        self.declare_parameter('retreat_timeout_sec', 120.0)
        self.declare_parameter('retreat_speed_mm_s', 30.0)
        self.declare_parameter('retreat_acc_mm_s2', 200.0)
        self.declare_parameter('direct_cartesian_max_speed_mm_s', 200.0)
        self.declare_parameter('direct_cartesian_max_acc_mm_s2', 500.0)
        self.declare_parameter('ros2_control_mode', 1)
        self.declare_parameter('trajectory_controller', 'uf850_traj_controller')
        self.declare_parameter('joint_state_broadcaster', 'joint_state_broadcaster')
        self.declare_parameter(
            'hardware_component',
            'uf_robot_hardware/UFRobotSystemHardware')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_state_ready_timeout_sec', 5.0)
        self.declare_parameter('joint_state_ready_samples', 3)
        self.declare_parameter('post_restore_settle_sec', 0.75)
        self.declare_parameter('post_restore_ready_samples', 5)
        self.declare_parameter('mode_transition_timeout_sec', 5.0)
        self.declare_parameter('mode_retry_interval_sec', 0.25)
        self.declare_parameter('mode_ready_samples', 3)
        self.declare_parameter('ft_zero_settle_sec', 0.25)
        self.declare_parameter('post_ft_state_settle_sec', 1.25)
        self.declare_parameter('ft_recovery_settle_sec', 0.5)
        self.declare_parameter('ft_recovery_max_attempts', 2)
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
        self.declare_parameter('staging_workspace_x_min_mm', -450.0)
        self.declare_parameter('staging_workspace_x_max_mm', 450.0)
        self.declare_parameter('staging_workspace_y_min_mm', 100.0)
        self.declare_parameter('staging_workspace_y_max_mm', 750.0)
        self.declare_parameter('staging_workspace_z_min_mm', -100.0)
        self.declare_parameter('pre_place_clearance_m', 0.03)
        self.declare_parameter('transfer_corner_height_m', 0.47)
        self.declare_parameter('joint6_name', 'joint6')
        self.declare_parameter('joint6_moveit_lower_rad', -2.0 * math.pi)
        self.declare_parameter('joint6_moveit_upper_rad', 2.0 * math.pi)
        self.declare_parameter('joint6_limit_margin_rad', 0.02)
        self.declare_parameter('planning_group', 'uf850')
        self.declare_parameter('ik_link_name', 'link_tcp')
        self.declare_parameter(
            'arm_joint_names', [f'joint{i}' for i in range(1, 7)])
        self.declare_parameter('direct_transfer_ik_timeout_sec', 2.0)
        self.declare_parameter('direct_transfer_sample_step_rad', 0.05)
        # A direct joint line should never select an IK branch that sweeps any
        # axis by more than half a revolution. Periodic joints are first
        # normalized to their nearest equivalent below.
        self.declare_parameter(
            'direct_transfer_max_joint_delta_rad', math.pi)
        self.declare_parameter('direct_transfer_max_joint_speed_rad_s', 2.14)
        self.declare_parameter('direct_transfer_joint_acc_rad_s2', 10.0)
        self.declare_parameter(
            'direct_transfer_periodic_joint_names',
            ['joint1', 'joint4', 'joint6'])
        self.declare_parameter(
            'direct_transfer_periodic_lower_rad', -2.0 * math.pi)
        self.declare_parameter(
            'direct_transfer_periodic_upper_rad', 2.0 * math.pi)
        self.declare_parameter('direct_transfer_joint_tolerance_rad', 0.03)

        def p(name):
            return self.get_parameter(name).value
        self.target_box_id = int(p('target_box_id'))
        self.max_detection_age = float(p('max_detection_age_sec'))
        self.max_pregrasp_plan_age = float(p('max_pregrasp_plan_age_sec'))
        self.grasp_offset = float(p('grasp_offset_m'))
        self.contact_reference_z = float(p('contact_reference_z_m'))
        self.minimum_measured_object_height = float(
            p('minimum_measured_object_height_m'))
        self.maximum_measured_object_height = float(
            p('maximum_measured_object_height_m'))
        self.contact_search_margin = float(p('contact_search_margin_m'))
        self.max_descent = float(p('max_descent_m'))
        self.tolerance = float(p('position_tolerance_m'))
        self.xy_tolerance = float(p('xy_tolerance_m'))
        self.pregrasp_z_tolerance = float(p('pregrasp_z_tolerance_m'))
        self.retrieval_pregrasp_above_tolerance = float(
            p('retrieval_pregrasp_above_tolerance_m'))
        self.minimum_contact_descent = float(p('minimum_contact_descent_m'))
        self.descent_timeout = float(p('descent_timeout_sec'))
        self.place_descent_timeout = float(p('place_descent_timeout_sec'))
        self.singularity_place_recovery_timeout = float(
            p('singularity_place_recovery_timeout_sec'))
        self.singularity_place_step = float(p('singularity_place_step_m'))
        self.singularity_place_step_speed = float(
            p('singularity_place_step_speed_mm_s'))
        self.singularity_deceleration_grace = float(
            p('singularity_deceleration_grace_sec'))
        self.singularity_no_progress = float(p('singularity_no_progress_sec'))
        self.singularity_min_progress = float(p('singularity_min_progress_m'))
        self.retreat_timeout = float(p('retreat_timeout_sec'))
        self.retreat_speed = float(p('retreat_speed_mm_s'))
        self.retreat_acc = float(p('retreat_acc_mm_s2'))
        self.direct_cartesian_max_speed = float(
            p('direct_cartesian_max_speed_mm_s'))
        self.direct_cartesian_max_acc = float(
            p('direct_cartesian_max_acc_mm_s2'))
        self.motion_speed_percent = 0.0
        self.ros2_control_mode = int(p('ros2_control_mode'))
        self.trajectory_controller = str(p('trajectory_controller'))
        self.joint_state_broadcaster = str(p('joint_state_broadcaster'))
        self.hardware_component = str(p('hardware_component'))
        self.joint_state_topic = str(p('joint_state_topic'))
        self.joint_state_ready_timeout = float(p('joint_state_ready_timeout_sec'))
        self.joint_state_ready_samples = int(p('joint_state_ready_samples'))
        self.post_restore_settle = float(p('post_restore_settle_sec'))
        self.post_restore_ready_samples = int(
            p('post_restore_ready_samples'))
        self.mode_transition_timeout = float(p('mode_transition_timeout_sec'))
        self.mode_retry_interval = float(p('mode_retry_interval_sec'))
        self.mode_ready_samples = int(p('mode_ready_samples'))
        self.ft_zero_settle = float(p('ft_zero_settle_sec'))
        self.post_ft_state_settle = float(p('post_ft_state_settle_sec'))
        self.ft_recovery_settle = float(p('ft_recovery_settle_sec'))
        self.ft_recovery_max_attempts = int(p('ft_recovery_max_attempts'))
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
        self.staging_bounds_mm = (
            float(p('staging_workspace_x_min_mm')),
            float(p('staging_workspace_x_max_mm')),
            float(p('staging_workspace_y_min_mm')),
            float(p('staging_workspace_y_max_mm')),
            float(p('staging_workspace_z_min_mm')),
            self.servo_bounds_mm[5],
        )
        self.pre_place_clearance = float(p('pre_place_clearance_m'))
        self.transfer_corner_height = float(p('transfer_corner_height_m'))
        self.joint6_name = str(p('joint6_name'))
        self.joint6_limits = (
            float(p('joint6_moveit_lower_rad')),
            float(p('joint6_moveit_upper_rad')))
        self.joint6_limit_margin = float(p('joint6_limit_margin_rad'))
        self.planning_group = str(p('planning_group'))
        self.ik_link_name = str(p('ik_link_name'))
        self.arm_joint_names = tuple(map(str, p('arm_joint_names')))
        self.direct_transfer_ik_timeout = float(
            p('direct_transfer_ik_timeout_sec'))
        self.direct_transfer_sample_step = float(
            p('direct_transfer_sample_step_rad'))
        self.direct_transfer_max_joint_delta = float(
            p('direct_transfer_max_joint_delta_rad'))
        self.direct_transfer_max_joint_speed = float(
            p('direct_transfer_max_joint_speed_rad_s'))
        self.direct_transfer_joint_acc = float(
            p('direct_transfer_joint_acc_rad_s2'))
        self.direct_transfer_periodic_joint_names = frozenset(
            map(str, p('direct_transfer_periodic_joint_names')))
        self.direct_transfer_periodic_limits = (
            float(p('direct_transfer_periodic_lower_rad')),
            float(p('direct_transfer_periodic_upper_rad')))
        self.direct_transfer_joint_tolerance = float(
            p('direct_transfer_joint_tolerance_rad'))
        if not 0.0 <= self.contact_search_margin <= self.max_descent:
            raise ValueError(
                'contact_search_margin_m must be within '
                f'[0, max_descent_m={self.max_descent:.3f}]')
        if not (self.pregrasp_z_tolerance <=
                self.retrieval_pregrasp_above_tolerance <= self.max_descent):
            raise ValueError(
                'retrieval_pregrasp_above_tolerance_m must be between '
                'pregrasp_z_tolerance_m and max_descent_m')
        if self.force_threshold <= 0.0:
            raise ValueError('force_contact_threshold_n must be positive')
        if not (0.0 < self.minimum_measured_object_height <
                self.maximum_measured_object_height):
            raise ValueError('measured object height bounds are invalid')
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
        if self.post_restore_settle < 0.5:
            raise ValueError('post_restore_settle_sec must be at least 0.5')
        if self.post_restore_ready_samples < 3:
            raise ValueError('post_restore_ready_samples must be at least 3')
        if self.mode_transition_timeout <= 0.0:
            raise ValueError('mode_transition_timeout_sec must be positive')
        if self.mode_retry_interval <= 0.0:
            raise ValueError('mode_retry_interval_sec must be positive')
        if self.mode_ready_samples < 1:
            raise ValueError('mode_ready_samples must be at least 1')
        if self.place_descent_timeout <= 0.0:
            raise ValueError('place_descent_timeout_sec must be positive')
        if (self.direct_cartesian_max_speed <= 0.0 or
                self.direct_cartesian_max_acc <= 0.0):
            raise ValueError('direct Cartesian speed/acceleration must be positive')
        if self.singularity_place_recovery_timeout <= 0.0:
            raise ValueError(
                'singularity_place_recovery_timeout_sec must be positive')
        if not 0.0005 <= self.singularity_place_step <= 0.010:
            raise ValueError('singularity_place_step_m must be within [0.0005, 0.010]')
        if self.singularity_place_step_speed <= 0.0:
            raise ValueError(
                'singularity_place_step_speed_mm_s must be positive')
        if (self.singularity_deceleration_grace <= 0.0 or
                self.singularity_no_progress <= 0.0 or
                self.singularity_min_progress <= 0.0):
            raise ValueError('singularity early-detection parameters must be positive')
        if (len(self.arm_joint_names) != 6 or
                len(set(self.arm_joint_names)) != 6):
            raise ValueError('arm_joint_names must contain six unique joints')
        if (self.direct_transfer_ik_timeout <= 0.0 or
                self.direct_transfer_sample_step <= 0.0 or
                self.direct_transfer_max_joint_delta <= 0.0 or
                self.direct_transfer_max_joint_speed <= 0.0 or
                self.direct_transfer_joint_acc <= 0.0 or
                self.direct_transfer_joint_tolerance <= 0.0):
            raise ValueError('direct-transfer parameters must be positive')
        if not self.direct_transfer_periodic_joint_names.issubset(
                self.arm_joint_names):
            raise ValueError(
                'direct_transfer_periodic_joint_names must be arm joints')
        if not (self.direct_transfer_periodic_limits[0] <
                self.direct_transfer_periodic_limits[1]):
            raise ValueError(
                'direct-transfer periodic lower limit must be below upper limit')
        if not self.joint6_limits[0] < self.joint6_limits[1]:
            raise ValueError('joint6 MoveIt lower limit must be below upper limit')
        if not 0.0 <= self.joint6_limit_margin < (
                self.joint6_limits[1] - self.joint6_limits[0]) / 2.0:
            raise ValueError('joint6_limit_margin_rad is invalid')
        if self.ft_zero_settle < 0.2:
            raise ValueError('ft_zero_settle_sec must be at least 0.2')
        if self.post_ft_state_settle <= 0.0:
            raise ValueError('post_ft_state_settle_sec must be positive')
        if self.ft_recovery_settle < 0.5:
            raise ValueError('ft_recovery_settle_sec must be at least 0.5')
        if self.ft_recovery_max_attempts < 1:
            raise ValueError('ft_recovery_max_attempts must be at least 1')
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
        # MoveIt/Servo reports the URDF link_tcp pose, while xArm's direct
        # set_position service reports and commands the controller-configured
        # SDK TCP. They are not necessarily colocated. Capture their live Z
        # separation before relinquishing ros2_control so vertical targets can
        # be converted instead of silently mixing the two coordinate origins.
        self.direct_tcp_z_offset = None
        self.direct_command_target_z = None
        self.direct_target_z = None
        self.direct_target_pose = None
        self.direct_target_quaternion = None
        self.direct_target_joints = None
        self.direct_transfer_validation_samples = []
        self.direct_transfer_validation_index = 0
        self.direct_transfer_succeeded = False
        self.direct_transfer_motion_started = False
        self.direct_transfer_ik_started = None
        self.direct_transfer_validation_started = None
        self.direct_transfer_execution_started = None
        self.transfer_goal_handle = None
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
        self.probe_only = False
        self.object_info_obtained = False
        self.contact_tcp_z = None
        self.contact_tcp_xyz = None
        self.corrected_object = None
        self.active_pickup_snapshot = None
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
        self.singularity_deceleration_started = None
        self.singularity_progress_started = None
        self.singularity_progress_reference_z = None
        self.direct_motion_generation = 0
        self.expected_enable_generation = None
        self.operation_kind = 'pickup'
        self.staging_place_active = False
        self.staging_place_config = None
        self.staging_release_tcp_pose = None
        self.planning_scene_status = {}
        self.orchestrator_status = {}
        self.pallet_locked = False
        self.detach_started = None
        self.last_joint_state_time = None
        self.joint_state_sequence = 0
        self.joint6_position = None
        self.latest_joint_positions = None
        self.restore_wait_sequence = None
        self.restore_wait_deadline = None
        self.restore_settle_started = None
        self.restore_settle_ready_count = 0
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
        self.robot_tcp_pose = None
        self.ft_recovery_required = False
        self.ft_recovery_reason = ''
        self.ft_recovery_attempt = 0
        self.ft_recovery_started = None
        self.ft_recovery_zeroed_at = None
        self.ft_recovery_restore_active = False
        self.ft_recovery_timer = None
        self.c52_retreat_active = False
        self.c52_clear_attempts = 0
        self.c52_tcp_snapshot = None
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
            Float64MultiArray, '/object_info_estimation/config',
            self.object_info_config_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/staging_slots/place_config',
            self.staging_place_config_callback, 10)
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
        self.clean_warn_client = self.create_client(
            Call, '/ufactory/clean_warn')
        self.ft_enable_client = self.create_client(
            SetInt16, '/ufactory/set_ft_sensor_enable')
        self.retreat_client = self.create_client(
            MoveCartesian, '/ufactory/set_position')
        self.compute_ik_client = self.create_client(
            GetPositionIK, '/compute_ik')
        self.state_validity_client = self.create_client(
            GetStateValidity, '/check_state_validity')
        self.transfer_trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            f'/{self.trajectory_controller}/follow_joint_trajectory')
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
        self.clear_picked_item_client = self.create_client(
            Trigger, '/planning_scene_obstacles/clear_picked_item')
        self.detach_staged_item_client = self.create_client(
            Trigger, '/planning_scene_obstacles/detach_staged_item')
        self.create_service(Trigger, '/pickup_supervisor/start', self.start_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_probe', self.start_probe_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/grasp_at_contact',
            self.grasp_at_contact_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/clear_object_info',
            self.clear_object_info_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_place', self.start_place_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_staging_place',
            self.start_staging_place_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_loading', self.start_loading_callback)
        self.create_service(
            Trigger, '/pickup_supervisor/start_joint_transfer',
            self.start_joint_transfer_callback)
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
        self.create_service(
            Trigger, '/pickup_supervisor/recover_ft_sensor',
            self.recover_ft_sensor_callback)
        self.create_timer(0.05, self.control_tick)
        self.create_timer(0.1, self.retreat_tick)
        self.create_timer(0.5, self.publish_status)

    def boxes_callback(self, message):
        self.boxes = {
            int(marker.id): marker for marker in message.markers
            if marker.ns == 'depth_refined_boxes' and
            marker.action == Marker.ADD and marker.header.frame_id == 'link_base'
        }

    def object_info_config_callback(self, message):
        if not message.data:
            return
        value = float(message.data[0])
        if not math.isfinite(value) or not -0.5 <= value <= 0.5:
            self.get_logger().warning(
                'ignored contact-reference Z outside [-0.5, 0.5] m')
            return
        if self.state in self.ACTIVE:
            self.get_logger().warning(
                'ignored contact-reference Z change during active motion')
            return
        self.contact_reference_z = value
        self.get_logger().info(
            f'empty-table contact TCP Z set to {value * 1000.0:.1f} mm')

    def joint_state_callback(self, message):
        self.last_joint_state_time = time.monotonic()
        self.joint_state_sequence += 1
        positions = {
            str(name): float(position)
            for name, position in zip(message.name, message.position)
            if math.isfinite(float(position))
        }
        if all(name in positions for name in self.arm_joint_names):
            self.latest_joint_positions = tuple(
                positions[name] for name in self.arm_joint_names)
        try:
            index = message.name.index(self.joint6_name)
            position = float(message.position[index])
        except (ValueError, IndexError, TypeError):
            return
        if math.isfinite(position):
            self.joint6_position = position

    def robot_state_callback(self, message):
        previous_error = self.robot_error
        self.robot_state = int(message.state)
        self.robot_mode = int(message.mode)
        self.robot_error = int(message.err)
        self.robot_state_time = time.monotonic()
        if len(message.pose) >= 6:
            pose = tuple(float(value) for value in message.pose[:6])
            if all(math.isfinite(value) for value in pose):
                self.robot_tcp_pose = pose
        if len(message.pose) >= 3:
            xyz = tuple(float(value) / 1000.0 for value in message.pose[:3])
            if all(math.isfinite(value) for value in xyz):
                self.robot_tcp_xyz = xyz
        if self.robot_error == 52 and previous_error != 52:
            force_age = (
                None if self.last_force_time is None else
                time.monotonic() - self.last_force_time)
            self.get_logger().error(
                'C52 first reported by xArm telemetry: '
                f'supervisor_state={self.state}, '
                f'operation={self.operation_kind}, mode={self.robot_mode}, '
                f'robot_state={self.robot_state}, tcp_m={self.robot_tcp_xyz}, '
                f'force_age_sec={force_age}')
            if self.state in (
                    self.ARMING_DESCENT, self.DESCENDING,
                    self.DISABLING_DESCENT):
                self._begin_c52_interruption()
            elif self.state not in (
                    self.RECOVERING_FT, self.C52_STOPPING,
                    self.PREPARING_RETREAT, self.RETREATING,
                    self.RESTORING_CONTROL):
                self.ft_recovery_required = True
                self.ft_recovery_reason = (
                    'xArm C52: six-axis force/torque sensor '
                    'zero-setting error')
                self._fault(
                    f'{self.ft_recovery_reason}; automatic motion is blocked '
                    'until unloaded FT recovery succeeds')

    def _begin_c52_interruption(self):
        """Stop contact motion and preserve a carried item after xArm C52."""
        if self.c52_retreat_active or self.state == self.C52_STOPPING:
            return
        self.ft_recovery_required = True
        self.ft_recovery_reason = (
            'xArm C52: six-axis force/torque sensor zero-setting error')
        held_item = str(
            self.planning_scene_status.get('attached_item_id') or '')
        if not held_item:
            self._fault(
                f'{self.ft_recovery_reason}; guarded descent stopped. '
                'Recover the unloaded FT sensor before another cycle')
            return
        if self.direct_target_z is None:
            self._fault(
                f'{self.ft_recovery_reason}; the item remains held, but no '
                'recorded recovery waypoint is available')
            return
        self.c52_retreat_active = True
        self.c52_clear_attempts = 0
        self.c52_tcp_snapshot = self.robot_tcp_xyz
        if getattr(self, 'direct_tcp_z_offset', None) is None:
            try:
                self._capture_direct_tcp_z_offset()
            except ValueError as exc:
                self.get_logger().warning(
                    'could not snapshot TCP-frame conversion at C52 onset: '
                    f'{exc}')
        self.state = self.C52_STOPPING
        self.get_logger().error(
            f'{self.ft_recovery_reason}; stopping Servo, preserving vacuum, '
            'and retreating with the item')
        if not self.enable_client.service_is_ready():
            self._fault(
                f'{self.ft_recovery_reason}; safe-servo disable service is '
                'unavailable and automatic retreat was blocked')
            return
        request = SetBool.Request()
        request.data = False
        future = self.enable_client.call_async(request)
        future.add_done_callback(self._c52_servo_stopped)
        self.publish_status()

    def _c52_servo_stopped(self, future):
        if self.state != self.C52_STOPPING:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(
                f'{self.ft_recovery_reason}; failed to stop Servo: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(
                f'{self.ft_recovery_reason}; Servo stop was not confirmed: '
                f'{message}')
            return
        self._disable_ft_for_c52_retreat()

    def _disable_ft_for_c52_retreat(self):
        """Clear C52 without zeroing a sensor carrying an attached load."""
        required_clients = (
            ('FT enable', self.ft_enable_client),
            ('clear error', self.clean_error_client),
            ('clear warning', self.clean_warn_client),
        )
        unavailable = [
            label for label, client in required_clients
            if not client.service_is_ready()
        ]
        if unavailable:
            self._fault(
                f'{self.ft_recovery_reason}; emergency retreat services '
                f'unavailable: {", ".join(unavailable)}. The item remains '
                'held and automatic motion is blocked')
            return
        self.get_logger().warning(
            'disabling the FT sensor before clearing C52; it will remain '
            'disabled until unloaded FT recovery is completed')
        request = SetInt16.Request()
        request.data = 0
        future = self.ft_enable_client.call_async(request)
        future.add_done_callback(self._c52_ft_disabled)

    def _c52_emergency_result(self, future, label):
        if self.state != self.C52_STOPPING:
            return False
        try:
            result = future.result()
        except Exception as exc:
            self._fault(
                f'{self.ft_recovery_reason}; {label} failed during emergency '
                f'retreat preparation: {exc}. The item remains held')
            return False
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._fault(
                f'{self.ft_recovery_reason}; {label} was rejected during '
                f'emergency retreat preparation: ret={code}. The item '
                'remains held')
            return False
        return True

    def _c52_ft_disabled(self, future):
        if not self._c52_emergency_result(future, 'disable FT sensor'):
            return
        future = self.clean_error_client.call_async(Call.Request())
        future.add_done_callback(self._c52_error_cleared)

    def _c52_error_cleared(self, future):
        if not self._c52_emergency_result(future, 'clear xArm error'):
            return
        future = self.clean_warn_client.call_async(Call.Request())
        future.add_done_callback(self._c52_warning_cleared)

    def _c52_warning_cleared(self, future):
        if not self._c52_emergency_result(future, 'clear xArm warning'):
            return
        # The FT sensor intentionally stays disabled. Re-enabling and zeroing
        # it while an item is attached would absorb the payload/contact force
        # into the bias and make subsequent contact detection unsafe.
        self.c52_clear_attempts = 1
        self.get_logger().warning(
            'C52 cleared with the FT sensor disabled; beginning loaded '
            'retreat and preserving vacuum')
        self._begin_direct_retreat()

    def staging_place_config_callback(self, message):
        if len(message.data) < 3:
            return
        try:
            slot = int(message.data[0])
            contact_z = float(message.data[1])
            transfer_z = float(message.data[2])
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in (contact_z, transfer_z)):
            return
        self.staging_place_config = {
            'slot': slot,
            'contact_tcp_z_m': contact_z,
            'transfer_tcp_z_m': transfer_z,
            'received_at': time.monotonic(),
        }

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
        if len(message.data) >= 14:
            self.keep_eef_perpendicular = bool(message.data[13] > 0.5)

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
        speed_percent = float(message.data[1])
        if (not math.isfinite(speed_percent) or
                not 5.0 <= speed_percent <= 100.0):
            self.get_logger().warning(
                'ignoring non-servo speed outside 5..100%: '
                f'{speed_percent}')
            return
        scale = speed_percent / 100.0
        self.motion_speed_percent = speed_percent
        self.retreat_speed = self.direct_cartesian_max_speed * scale
        self.retreat_acc = self.direct_cartesian_max_acc * scale
        self.get_logger().info(
            f'non-servo speed set to {speed_percent:.1f}%: '
            f'{self.retreat_speed:.1f} mm/s service, '
            f'{self.retreat_acc:.1f} mm/s^2 service, '
            f'{self.direct_transfer_max_joint_speed * scale:.3f} '
            'rad/s joint limit')

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
            # A manual release means the operator is removing/discarding the
            # carried item.  Do not use the normal placement detach service:
            # it creates a world collision object at the current TCP pose and
            # can leave the next motion starting in collision with the
            # gripper.  Supervised pallet/staging placement continues to use
            # the placement detach services elsewhere in this node.
            if self.clear_picked_item_client.service_is_ready():
                future = self.clear_picked_item_client.call_async(
                    Trigger.Request())
                future.add_done_callback(self._manual_clear_picked_completed)
            else:
                self.manual_gripper_state = (
                    'open; warning: attached planning-scene item remains')
                self.get_logger().error(self.manual_gripper_state)
        self.publish_status()

    def _manual_clear_picked_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.manual_gripper_state = f'open; scene clear failed: {exc}'
            self.get_logger().error(self.manual_gripper_state)
            self.publish_status()
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self.manual_gripper_state = (
                f'open; scene clear rejected: {message}')
            self.get_logger().error(self.manual_gripper_state)
        else:
            self.manual_gripper_state = 'open; picked scene item cleared'
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
        x_error = x - float(snapshot['x_m'])
        y_error = y - float(snapshot['y_m'])
        if abs(x_error) > self.xy_tolerance or abs(y_error) > self.xy_tolerance:
            raise ValueError(
                'TCP is not aligned over the selected box: '
                f'error XY=({x_error * 1000.0:+.1f}, '
                f'{y_error * 1000.0:+.1f}) mm, '
                f'tolerance={self.xy_tolerance * 1000.0:.1f} mm')
        expected_z = float(snapshot['pregrasp_z_m'])
        z_error = z - expected_z
        is_retrieval = (
            'retrieval_target_id' in snapshot or 'staging_slot' in snapshot)
        # A Cartesian retrieval can finish slightly above its exact endpoint
        # while preserving the verified XY alignment and overhead approach.
        # Guarded contact search can safely start from that higher live pose.
        # Being below the planned pose remains subject to the strict normal
        # tolerance, as does every incoming-table pickup.
        z_tolerance = (
            self.retrieval_pregrasp_above_tolerance
            if is_retrieval and z_error >= 0.0
            else self.pregrasp_z_tolerance)
        if abs(z_error) > z_tolerance:
            raise ValueError(
                f'TCP is not at the verified pre-grasp height: '
                f'current={z:.4f} m, expected={expected_z:.4f} m, '
                f'error={z_error * 1000.0:+.1f} mm, '
                f'tolerance={z_tolerance * 1000.0:.1f} mm')
        estimated_contact_z = float(snapshot['top_z_m']) + self.grasp_offset
        # Incoming-table pickups use the normal positive-Z workspace floor.
        # A planned pallet/staging retrieval can legitimately have its TCP
        # below the robot-base plane, because the localized pallet origin is
        # below that plane.  Those targets were already collision-checked by
        # the mandatory overhead and straight pre-pick plans, so use the same
        # configured low-Z floor as pallet placement while retaining the
        # contact-search and maximum-descent limits below.
        servo_floor_z = (
            self.place_workspace_z_min_mm / 1000.0
            if is_retrieval else self.servo_bounds_mm[4] / 1000.0)
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

    def _link_tcp_xyz(self):
        """Return the fresh MoveIt/Servo link_tcp position without fallback."""
        keys = ('tcp_x_m', 'tcp_y_m', 'tcp_z_m')
        if self.servo_status_time is None or (
                time.monotonic() - self.servo_status_time > self.status_timeout):
            raise ValueError('link_tcp telemetry is stale')
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

    def _tcp_xyz(self):
        try:
            return self._link_tcp_xyz()
        except ValueError as link_error:
            if (self.robot_tcp_xyz is not None and
                    self.robot_state_time is not None and
                    time.monotonic() - self.robot_state_time <=
                    self.status_timeout):
                return self.robot_tcp_xyz
            raise link_error

    def _capture_direct_tcp_z_offset(self):
        """Snapshot link_tcp Z minus SDK TCP Z before direct-mode handoff."""
        link_xyz = self._link_tcp_xyz()
        if (self.robot_tcp_xyz is None or self.robot_state_time is None or
                time.monotonic() - self.robot_state_time > self.status_timeout):
            raise ValueError('xArm SDK TCP telemetry is stale')
        sdk_xyz = tuple(float(value) for value in self.robot_tcp_xyz)
        if not all(math.isfinite(value) for value in sdk_xyz):
            raise ValueError('xArm SDK TCP telemetry is invalid')
        offset = float(link_xyz[2] - sdk_xyz[2])
        if abs(offset) > 0.25:
            raise ValueError(
                f'link_tcp to xArm SDK TCP Z offset {offset:.3f} m is invalid')
        self.direct_tcp_z_offset = offset
        self.get_logger().info(
            'captured direct-mode TCP Z conversion: '
            f'link_tcp - xArm SDK TCP = {offset * 1000.0:+.1f} mm')
        return offset

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

    @staticmethod
    def _nearest_periodic_equivalent(target, current, lower, upper):
        """Return the in-range 2*pi equivalent nearest the current joint."""
        candidates = [
            target + 2.0 * math.pi * turns
            for turns in range(-2, 3)
            if lower <= target + 2.0 * math.pi * turns <= upper
        ]
        if not candidates:
            return target
        return min(candidates, key=lambda candidate: abs(candidate - current))

    @staticmethod
    def _wait_for_service(client, timeout_sec):
        """Wait a bounded time for discovery, including after driver startup."""
        wait = getattr(client, 'wait_for_service', None)
        if wait is None:
            return False
        try:
            return bool(wait(timeout_sec=float(timeout_sec)))
        except (TypeError, RuntimeError):
            return False

    @staticmethod
    def _periodic_joint_error(actual, target):
        return abs((actual - target + math.pi) % (2.0 * math.pi) - math.pi)

    def _publish_servo_config(self, touch_mode, bypass_force=False):
        is_place = self.operation_kind == 'place'
        force = (self.place_force_threshold if is_place else
                 self.force_threshold)
        speed_scale = (self.place_servo_speed_scale if is_place else
                       self.servo_speed_scale)
        staging_place = getattr(self, 'staging_place_active', False)
        bounds = (self.staging_bounds_mm if staging_place else
                  self.servo_bounds_mm)
        z_min = (self.staging_bounds_mm[4] if staging_place else
                 self.place_workspace_z_min_mm if is_place else bounds[4])
        message = Float64MultiArray()
        message.data = [
            speed_scale,
            bounds[0], bounds[1], bounds[2], bounds[3],
            z_min, bounds[5],
            force,
            1.0 if touch_mode else 0.0,
            1.0 if bypass_force else 0.0,
            1.0 if is_place else 0.0,
        ]
        self.config_pub.publish(message)

    def _ft_recovery_blocks_start(self, response):
        if (getattr(self, 'ft_recovery_required', False) or
                getattr(self, 'robot_error', None) == 52):
            self.ft_recovery_required = True
            if not getattr(self, 'ft_recovery_reason', ''):
                self.ft_recovery_reason = (
                    'xArm C52: six-axis force/torque sensor '
                    'zero-setting error')
            response.message = (
                'automatic motion is blocked until unloaded FT sensor '
                'recovery succeeds')
            return True
        return False

    def start_callback(self, _request, response):
        return self._start_pickup_descent(response, probe_only=False)

    def start_probe_callback(self, _request, response):
        return self._start_pickup_descent(response, probe_only=True)

    def _start_pickup_descent(self, response, probe_only):
        if self._ft_recovery_blocks_start(response):
            return response
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
        self.probe_only = bool(probe_only)
        self.object_info_obtained = False
        self.contact_tcp_z = None
        self.contact_tcp_xyz = None
        self.corrected_object = None
        self.active_pickup_snapshot = dict(snapshot)
        self.staging_place_active = False
        self.staging_release_tcp_pose = None
        self.fault = ''
        self.post_retreat_fault = ''
        self.direct_target_pose = None
        self.direct_target_joints = None
        self.direct_tcp_z_offset = None
        self.direct_command_target_z = None
        self.direct_transfer_validation_samples = []
        self.direct_transfer_validation_index = 0
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
        self.singularity_deceleration_started = None
        self.singularity_progress_started = None
        self.singularity_progress_reference_z = None
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
            f"{'object-height probe' if self.probe_only else 'pickup'} "
            f"started for box {int(snapshot['box_id'])}: "
            f'zeroing FT sensor, then contact descent from Z {z:.3f} m; '
            f'dry_run={self.dry_run}')
        return response

    def _finalize_contact_object_info(self):
        if self.active_pickup_snapshot is None:
            raise ValueError('pre-grasp object snapshot is unavailable')
        contact_xyz = tuple(map(float, self._tcp_xyz()))
        contact_z = contact_xyz[2]
        height = contact_z - self.contact_reference_z
        if not (self.minimum_measured_object_height <= height <=
                self.maximum_measured_object_height):
            raise ValueError(
                f'contact-derived object height {height * 1000.0:.1f} mm is '
                f'outside [{self.minimum_measured_object_height * 1000.0:.1f}, '
                f'{self.maximum_measured_object_height * 1000.0:.1f}] mm')
        corrected = dict(self.active_pickup_snapshot)
        corrected['size_z_m'] = height
        corrected['center_z_m'] = self.contact_reference_z + height / 2.0
        corrected['top_z_m'] = contact_z
        corrected['contact_tcp_z_m'] = contact_z
        corrected['contact_reference_z_m'] = self.contact_reference_z
        self.active_pickup_snapshot = corrected
        self.corrected_object = corrected
        self.contact_tcp_z = contact_z
        self.contact_tcp_xyz = contact_xyz
        self.object_info_obtained = True
        return corrected

    def grasp_at_contact_callback(self, _request, response):
        if self.state != self.AWAITING_GRASP or not self.object_info_obtained:
            response.message = (
                f'object information is not ready at contact; state={self.state}')
            return response
        try:
            current_xyz = tuple(map(float, self._tcp_xyz()))
            displacement = math.sqrt(sum(
                (current - measured) ** 2
                for current, measured in zip(
                    current_xyz, self.contact_tcp_xyz)))
            if displacement > self.pregrasp_z_tolerance:
                raise ValueError(
                    'TCP moved away from the measured contact pose; estimate '
                    'object information again')
            self.direct_target_z = self._pickup_retreat_target_tcp_z(
                float(self.corrected_object['size_z_m']),
                float(self.corrected_object['size_x_m']))
        except ValueError as exc:
            self.object_info_obtained = False
            response.message = str(exc)
            self.publish_status()
            return response
        self.probe_only = False
        self._turn_vacuum_on()
        response.success = True
        response.message = 'object information accepted; vacuum pickup started'
        return response

    def clear_object_info_callback(self, _request, response):
        if self.state == self.AWAITING_GRASP:
            response.message = (
                'cannot clear object information while TCP is at contact; '
                'abort or pick the object first')
            return response
        self.object_info_obtained = False
        self.contact_tcp_z = None
        self.contact_tcp_xyz = None
        self.corrected_object = None
        self.active_pickup_snapshot = None
        response.success = True
        response.message = 'latched object information cleared'
        self.publish_status()
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
        if self._ft_recovery_blocks_start(response):
            return response
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
        self.staging_place_active = False
        self.staging_release_tcp_pose = None
        self.fault = ''
        self.post_retreat_fault = ''
        self.direct_target_pose = None
        self.direct_target_joints = None
        self.direct_tcp_z_offset = None
        self.direct_command_target_z = None
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

    def start_staging_place_callback(self, _request, response):
        if self._ft_recovery_blocks_start(response):
            return response
        if self.manual_gripper_pending:
            response.message = 'wait for the pending gripper command'
            return response
        if self.state in self.ACTIVE:
            response.message = f'supervisor already active in {self.state}'
            return response
        if not self.planning_scene_status.get('attached_item_id'):
            response.message = 'no carried item is attached in the planning scene'
            return response
        if not self.enable_client.service_is_ready():
            response.message = 'safe-servo enable service is unavailable'
            return response
        config = self.staging_place_config
        if (not isinstance(config, dict) or
                time.monotonic() - float(config.get('received_at', 0.0)) > 2.0):
            response.message = 'fresh staging placement configuration is unavailable'
            return response
        try:
            _, _, z = self._tcp_xyz()
            contact_z = float(config['contact_tcp_z_m'])
            transfer_z = float(config['transfer_tcp_z_m'])
        except (KeyError, TypeError, ValueError) as exc:
            response.message = f'invalid staging placement configuration: {exc}'
            return response
        if transfer_z < z - self.tolerance:
            response.message = 'staging transfer retreat height is below TCP'
            return response
        floor_z = max(
            contact_z - self.contact_search_margin,
            self.staging_bounds_mm[4] / 1000.0)
        if z - floor_z < self.minimum_contact_descent:
            response.message = (
                f'staging pre-place Z {z:.3f} m leaves less than '
                f'{self.minimum_contact_descent:.3f} m guarded descent')
            return response
        self.operation_id += 1
        # Reuse the mature placement contact, singularity fallback, release,
        # controller-handoff, and vertical-retreat state machine.  The flag
        # selects staging-specific bounds and detachment behavior.
        self.operation_kind = 'place'
        self.staging_place_active = True
        self.staging_release_tcp_pose = None
        self.fault = ''
        self.post_retreat_fault = ''
        self.direct_target_pose = None
        self.direct_target_joints = None
        self.direct_tcp_z_offset = None
        self.direct_command_target_z = None
        self.direct_target_z = transfer_z
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
        self.floor_z = floor_z
        self.virtual_z = z
        self.state = self.ARMING_DESCENT
        self._reset_servo_then_begin_place(z, floor_z)
        response.success = True
        response.message = (
            f'staging slot {int(config["slot"])} safe-servo descent is '
            f'arming from Z {z:.3f} m toward contact Z {contact_z:.3f} m')
        return response

    def start_joint_transfer_callback(self, _request, response):
        """Validate and execute a direct joint interpolation to transfer."""
        if self._ft_recovery_blocks_start(response):
            return response
        if self.manual_gripper_pending:
            response.message = 'wait for the pending gripper command'
            return response
        if self.state in self.ACTIVE:
            response.message = f'supervisor already active in {self.state}'
            return response
        if (self.motion_status.get('state') != 'PREPARED' or
                self.motion_status.get('target') != 'transfer'):
            response.message = 'prepare the transfer target first'
            return response
        if not self.pallet_locked:
            response.message = 'pallet pose is not LOCKED'
            return response
        if not self.planning_scene_status.get('attached_item_id'):
            response.message = 'no carried item is attached in the planning scene'
            return response
        if (self.latest_joint_positions is None or
                self.last_joint_state_time is None or
                time.monotonic() - self.last_joint_state_time > self.status_timeout):
            response.message = 'fresh six-axis joint state is unavailable'
            return response
        if (not self.compute_ik_client.service_is_ready() and
                not self._wait_for_service(
                    self.compute_ik_client,
                    self.direct_transfer_ik_timeout)):
            response.message = 'MoveIt compute_ik service is unavailable'
            return response
        if not self.state_validity_client.service_is_ready():
            response.message = 'MoveIt state-validity service is unavailable'
            return response
        try:
            xyz = tuple(map(float, self.motion_status['transfer_tcp_xyz_m']))
            quaternion = tuple(map(
                float, self.motion_status['transfer_tcp_quaternion_xyzw']))
            if len(xyz) != 3 or len(quaternion) != 4:
                raise ValueError('unexpected target vector length')
            if not all(math.isfinite(value) for value in (*xyz, *quaternion)):
                raise ValueError('non-finite target value')
            rpy = self._rpy_from_quaternion(quaternion)
        except (KeyError, TypeError, ValueError) as exc:
            response.message = f'direct joint transfer target is unavailable: {exc}'
            return response
        self.operation_id += 1
        self.operation_kind = 'transfer'
        self.fault = ''
        self.post_retreat_fault = ''
        self.direct_transfer_succeeded = False
        self.direct_transfer_motion_started = False
        self.direct_transfer_ik_started = time.monotonic()
        self.direct_transfer_validation_started = None
        self.direct_transfer_execution_started = None
        self.transfer_goal_handle = None
        self.transfer_fallback_reason = ''
        self.direct_target_z = xyz[2]
        self.direct_target_pose = (
            xyz[0] * 1000.0, xyz[1] * 1000.0, xyz[2] * 1000.0, *rpy)
        self.direct_target_quaternion = quaternion
        self.direct_target_joints = None
        self.direct_transfer_validation_samples = []
        self.direct_transfer_validation_index = 0
        self.state = self.SOLVING_TRANSFER_IK

        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.planning_group
        ik.robot_state.joint_state.name = list(self.arm_joint_names)
        ik.robot_state.joint_state.position = list(map(
            float, self.latest_joint_positions))
        ik.robot_state.is_diff = True
        # Ask MoveIt for a goal state that is already valid in the current
        # planning scene. The complete interpolated path is checked separately
        # below before any trajectory is submitted to the controller.
        ik.avoid_collisions = True
        ik.ik_link_name = self.ik_link_name
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = 'link_base'
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik.pose_stamped.pose.position.x = xyz[0]
        ik.pose_stamped.pose.position.y = xyz[1]
        ik.pose_stamped.pose.position.z = xyz[2]
        (ik.pose_stamped.pose.orientation.x,
         ik.pose_stamped.pose.orientation.y,
         ik.pose_stamped.pose.orientation.z,
         ik.pose_stamped.pose.orientation.w) = quaternion
        seconds = max(0.001, self.direct_transfer_ik_timeout)
        ik.timeout.sec = int(seconds)
        ik.timeout.nanosec = int((seconds - int(seconds)) * 1e9)
        future = self.compute_ik_client.call_async(request)
        future.add_done_callback(self._direct_transfer_ik_completed)
        response.success = True
        response.message = (
            'MoveIt/KDL IK and sampled transfer collision validation started')
        self.publish_status()
        return response

    def _direct_transfer_ik_completed(self, future):
        if self.state != self.SOLVING_TRANSFER_IK:
            return
        ik_started = getattr(self, 'direct_transfer_ik_started', None)
        elapsed = (
            None if ik_started is None else time.monotonic() - ik_started)
        try:
            result = future.result()
        except Exception as exc:
            self._fault(
                f'MoveIt/KDL transfer IK request failed after '
                f'{elapsed or 0.0:.3f} s: {exc}')
            return
        if result is None or result.error_code.val != 1:
            code = None if result is None else result.error_code.val
            message = '' if result is None else result.error_code.message
            suffix = f' ({message})' if message else ''
            self._fault(
                ('KINEMATIC_REJECTED: ' if code == -31 else '') +
                f'MoveIt/KDL transfer IK failed after '
                f'{elapsed or 0.0:.3f} s: code={code}{suffix}')
            return
        self.get_logger().info(
            f'MoveIt/KDL transfer IK solved in {elapsed or 0.0:.3f} s')
        solution = {
            str(name): float(position)
            for name, position in zip(
                result.solution.joint_state.name,
                result.solution.joint_state.position)
        }
        if not all(name in solution for name in self.arm_joint_names):
            self._fault('MoveIt/KDL transfer IK omitted one or more arm joints')
            return
        start = tuple(self.latest_joint_positions or ())
        if len(start) != len(self.arm_joint_names):
            self._fault('direct transfer lost its joint-state seed')
            return
        lower, upper = self.direct_transfer_periodic_limits
        raw_target = tuple(solution[name] for name in self.arm_joint_names)
        if not all(math.isfinite(value) for value in raw_target):
            self._fault('MoveIt/KDL transfer IK returned non-finite joint values')
            return
        target = tuple(
            self._nearest_periodic_equivalent(goal, current, lower, upper)
            if name in self.direct_transfer_periodic_joint_names else goal
            for name, current, goal in zip(
                self.arm_joint_names, start, raw_target)
        )
        normalized = [
            f'{name}: {raw:+.3f}->{goal:+.3f}'
            for name, raw, goal in zip(
                self.arm_joint_names, raw_target, target)
            if abs(raw - goal) > 1e-9
        ]
        if normalized:
            self.get_logger().info(
                'normalized periodic IK joints to nearest equivalents: ' +
                ', '.join(normalized))
        max_delta = max(abs(goal - current) for current, goal in zip(start, target))
        joint_deltas = tuple(
            goal - current for current, goal in zip(start, target))
        self.get_logger().info(
            'direct transfer joint deltas: ' + ', '.join(
                f'{name}={delta:+.3f} rad'
                for name, delta in zip(self.arm_joint_names, joint_deltas)))
        if max_delta > self.direct_transfer_max_joint_delta:
            self._fault(
                f'direct transfer joint jump {max_delta:.3f} rad exceeds '
                f'{self.direct_transfer_max_joint_delta:.3f} rad')
            return
        sample_count = max(1, math.ceil(
            max_delta / self.direct_transfer_sample_step))
        self.direct_target_joints = target
        self.direct_transfer_validation_samples = [
            tuple(current + (goal - current) * index / sample_count
                  for current, goal in zip(start, target))
            for index in range(1, sample_count + 1)
        ]
        self.direct_transfer_validation_index = 0
        self.direct_transfer_validation_started = time.monotonic()
        self.state = self.VALIDATING_TRANSFER
        self.get_logger().info(
            f'validating {len(self.direct_transfer_validation_samples)} '
            'joint-interpolation samples with MoveIt')
        self._validate_next_direct_transfer_sample()

    def _validate_next_direct_transfer_sample(self):
        if self.state != self.VALIDATING_TRANSFER:
            return
        if (self.direct_transfer_validation_index >=
                len(self.direct_transfer_validation_samples)):
            validation_started = getattr(
                self, 'direct_transfer_validation_started', None)
            elapsed = (
                time.monotonic() - validation_started
                if validation_started is not None else 0.0)
            self.get_logger().info(
                'direct transfer interpolation passed '
                f'{len(self.direct_transfer_validation_samples)} collision '
                f'checks in {elapsed:.3f} s; sending the validated path to '
                f'{self.trajectory_controller}')
            self._check_loading_path_before_transfer()
            return
        request = GetStateValidity.Request()
        request.group_name = self.planning_group
        request.robot_state = RobotState()
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(self.arm_joint_names)
        request.robot_state.joint_state.position = list(
            self.direct_transfer_validation_samples[
                self.direct_transfer_validation_index])
        future = self.state_validity_client.call_async(request)
        future.add_done_callback(self._direct_transfer_sample_validated)

    def _direct_transfer_sample_validated(self, future):
        if self.state != self.VALIDATING_TRANSFER:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'direct transfer collision validation failed: {exc}')
            return
        if result is None or not result.valid:
            sample = self.direct_transfer_validation_index + 1
            total = len(self.direct_transfer_validation_samples)
            self._fault(
                f'direct transfer interpolation is in collision at sample '
                f'{sample}/{total}')
            return
        self.direct_transfer_validation_index += 1
        self._validate_next_direct_transfer_sample()

    def _check_loading_path_before_transfer(self):
        """Check descent from the selected transfer joints before moving."""
        if self.motion_status.get('transfer_context', 'pallet') != 'pallet':
            self._send_joint_trajectory_transfer()
            return
        if not self.compute_ik_client.service_is_ready():
            self._fault('loading-path validation service is unavailable')
            return
        try:
            target_z = float(self.motion_status['pre_place_tcp_z_m']) - self.pre_place_clearance
            xyz = tuple(map(float, self.motion_status['transfer_tcp_xyz_m']))
            if (len(xyz) != 3 or not all(math.isfinite(v) for v in xyz) or
                    not math.isfinite(target_z) or target_z > xyz[2]):
                raise ValueError('invalid loading height')
        except (KeyError, TypeError, ValueError) as exc:
            self._fault(f'loading-path geometry unavailable: {exc}')
            return
        count = max(1, math.ceil((xyz[2] - target_z) / 0.005))
        self.loading_path_samples = [
            (xyz[0], xyz[1], max(target_z, xyz[2] - 0.005 * i))
            for i in range(1, count + 1)
        ]
        self.loading_path_index = 0
        self.loading_path_seed = tuple(self.direct_target_joints)
        self.state = self.CHECKING_LOADING_PATH
        self.loading_path_started = time.monotonic()
        self._request_loading_ik_sample()

    def _request_loading_ik_sample(self):
        if self.state != self.CHECKING_LOADING_PATH:
            return
        if self.loading_path_index == len(self.loading_path_samples):
            self.get_logger().info(
                'vertical loading path passed sequential IK at '
                f'{self.loading_path_index} explicit steps of at most 5 mm; '
                'executing transfer')
            self.state = self.VALIDATING_TRANSFER
            self._send_joint_trajectory_transfer()
            return
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.planning_group
        ik.ik_link_name = self.ik_link_name
        ik.robot_state.is_diff = True
        ik.robot_state.joint_state.name = list(self.arm_joint_names)
        ik.robot_state.joint_state.position = list(self.loading_path_seed)
        ik.pose_stamped.header.frame_id = 'link_base'
        pose = ik.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = (
            self.loading_path_samples[self.loading_path_index])
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = self.direct_target_quaternion
        # Contact is intentional: validate kinematics, retaining the separate
        # collision check for the transfer trajectory.
        ik.avoid_collisions = False
        seconds = max(0.001, self.direct_transfer_ik_timeout)
        ik.timeout.sec = int(seconds)
        ik.timeout.nanosec = int((seconds - int(seconds)) * 1e9)
        operation = self.operation_id
        index = self.loading_path_index
        future = self.compute_ik_client.call_async(request)
        future.add_done_callback(
            lambda done: self._loading_path_checked(done, operation, index))

    def _loading_path_checked(self, future, operation, index):
        if self.state != self.CHECKING_LOADING_PATH or self.operation_id != operation:
            return
        if self.loading_path_index != index:
            return
        location = (f'step {index + 1}/{len(self.loading_path_samples)}, '
                    f'Z={self.loading_path_samples[index][2]:.4f} m')
        try:
            result = future.result()
            if result is None:
                raise ValueError('empty response')
            if result.error_code.val not in (1, -31):
                raise ValueError(f'service error {result.error_code.val}')
            if result.error_code.val == -31:
                self._fault(
                    f'KINEMATIC_REJECTED: no descent IK at {location}')
                return
            joints = result.solution.joint_state
            solution = dict(zip(joints.name, joints.position))
            current = tuple(float(solution[name]) for name in self.arm_joint_names)
            if not all(math.isfinite(v) for v in current):
                raise ValueError('non-finite IK joints')
            lower, upper = self.direct_transfer_periodic_limits
            current = tuple(
                self._nearest_periodic_equivalent(goal, previous, lower, upper)
                if name in self.direct_transfer_periodic_joint_names else goal
                for name, goal, previous in zip(
                    self.arm_joint_names, current, self.loading_path_seed))
            for name, goal, previous in zip(
                    self.arm_joint_names, current, self.loading_path_seed):
                if (name in self.direct_transfer_periodic_joint_names and
                        not lower <= goal <= upper):
                    self._fault(
                        f'KINEMATIC_REJECTED: {name} outside configured '
                        f'periodic joint limits at {location}')
                    return
                if abs(goal - previous) > 0.15:
                    self._fault(
                        f'KINEMATIC_REJECTED: {name} changes '
                        f'{abs(goal - previous):.3f} rad at {location} '
                        '(limit 0.15 rad per <=5 mm step)')
                    return
        except Exception as exc:
            self._fault(f'loading-path validation failed at {location}: {exc}')
            return
        self.loading_path_seed = current
        self.loading_path_index += 1
        self._request_loading_ik_sample()

    @staticmethod
    def _smoothstep5(progress):
        """Return position, first-, and second-derivative quintic scales."""
        u = max(0.0, min(1.0, float(progress)))
        position = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        velocity = 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4
        acceleration = 60.0 * u - 180.0 * u**2 + 120.0 * u**3
        return position, velocity, acceleration

    @staticmethod
    def _joint_transfer_timing(
            max_delta, operator_percent, maximum_speed,
            maximum_acceleration):
        """Return duration and scaled limits for a quintic joint motion."""
        speed_fraction = max(
            0.05, min(1.0, float(operator_percent) / 100.0))
        max_speed = float(maximum_speed) * speed_fraction
        max_acceleration = float(maximum_acceleration) * speed_fraction
        duration = max(
            0.5,
            1.875 * float(max_delta) / max_speed,
            math.sqrt(
                5.774 * float(max_delta) / max_acceleration),
        )
        return duration, max_speed, max_acceleration, speed_fraction

    def _send_joint_trajectory_transfer(self):
        """Execute the collision-checked joint line through ros2_control."""
        if self.state != self.VALIDATING_TRANSFER:
            return
        if not self.transfer_trajectory_client.server_is_ready() and not (
                self.transfer_trajectory_client.wait_for_server(
                    timeout_sec=self.direct_transfer_ik_timeout)):
            self._fault(
                f'{self.trajectory_controller} trajectory action is unavailable')
            return
        start = tuple(self.latest_joint_positions or ())
        target = tuple(self.direct_target_joints or ())
        samples = len(self.direct_transfer_validation_samples)
        if (len(start) != len(self.arm_joint_names) or
                len(target) != len(self.arm_joint_names) or samples < 1):
            self._fault('validated direct-transfer trajectory is unavailable')
            return

        deltas = tuple(goal - current for current, goal in zip(start, target))
        max_delta = max(abs(delta) for delta in deltas)
        # Quintic smoothstep has peak normalized velocity 1.875 and peak
        # normalized acceleration about 5.774. Choose a duration that respects
        # both shared speed-slider velocity and configured acceleration limits.
        duration, max_speed, max_acceleration, speed_fraction = (
            self._joint_transfer_timing(
                max_delta,
                getattr(self, 'motion_speed_percent', 0.0) or
                self.retreat_speed,
                self.direct_transfer_max_joint_speed,
                self.direct_transfer_joint_acc,
            ))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.arm_joint_names)
        for index in range(1, samples + 1):
            progress = index / samples
            position_scale, velocity_scale, acceleration_scale = (
                self._smoothstep5(progress))
            point = JointTrajectoryPoint()
            point.positions = [
                current + delta * position_scale
                for current, delta in zip(start, deltas)
            ]
            point.velocities = [
                delta * velocity_scale / duration for delta in deltas
            ]
            point.accelerations = [
                delta * acceleration_scale / (duration * duration)
                for delta in deltas
            ]
            point.time_from_start = Duration(
                seconds=duration * progress).to_msg()
            goal.trajectory.points.append(point)
        goal.goal_time_tolerance = Duration(seconds=2.0).to_msg()

        self.state = self.EXECUTING_TRANSFER
        self.direct_transfer_execution_started = time.monotonic()
        self.get_logger().info(
            f'executing {samples}-point deterministic joint trajectory over '
            f'{duration:.3f} s (limits {max_speed:.3f} rad/s, '
            f'{max_acceleration:.3f} rad/s^2; '
            f'slider={speed_fraction * 100.0:.1f}%)')
        future = self.transfer_trajectory_client.send_goal_async(goal)
        future.add_done_callback(self._joint_transfer_goal_response)
        self.publish_status()

    def _joint_transfer_goal_response(self, future):
        if self.state != self.EXECUTING_TRANSFER:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fault(f'transfer trajectory request failed: {exc}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self._fault(f'{self.trajectory_controller} rejected transfer trajectory')
            return
        self.transfer_goal_handle = goal_handle
        self.direct_transfer_motion_started = True
        future = goal_handle.get_result_async()
        future.add_done_callback(self._joint_transfer_result)

    def _joint_transfer_result(self, future):
        if self.state != self.EXECUTING_TRANSFER:
            return
        self.transfer_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as exc:
            self._fault(f'transfer trajectory execution failed: {exc}')
            return
        result = None if wrapped is None else wrapped.result
        status = None if wrapped is None else wrapped.status
        error_code = None if result is None else result.error_code
        if (status != GoalStatus.STATUS_SUCCEEDED or
                error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            error_string = '' if result is None else result.error_string
            suffix = f' ({error_string})' if error_string else ''
            self._fault(
                f'transfer trajectory failed: status={status}, '
                f'code={error_code}{suffix}')
            return
        started = self.direct_transfer_execution_started
        elapsed = 0.0 if started is None else time.monotonic() - started
        self.get_logger().info(
            f'deterministic joint transfer completed in {elapsed:.3f} s')
        self._finish_retreat()

    def start_transfer_fallback_callback(self, _request, response):
        if self._ft_recovery_blocks_start(response):
            return response
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
            self.direct_target_joints = None
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
        if self._ft_recovery_blocks_start(response):
            return response
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
        self.direct_target_joints = None
        self.direct_tcp_z_offset = None
        self.direct_command_target_z = None
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
                bool(self.servo_status.get(
                    'contact_on_fz_sign_change')) ==
                (self.operation_kind == 'place') and
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
        if self.state in self.ACTIVE and self.state != self.AWAITING_GRASP:
            response.message = f'pickup already active in {self.state}'
            return response
        try:
            self._tcp_xyz()
        except ValueError as exc:
            response.message = str(exc)
            return response
        if self.state == self.AWAITING_GRASP:
            # An estimation-only cycle may discover that Neuromeka has no
            # stable target. Leave contact without actuating the gripper,
            # first returning vertically to the recorded pre-grasp height.
            if self.pregrasp_z is None:
                response.message = 'recorded pre-grasp height is unavailable'
                return response
            self.direct_target_z = float(self.pregrasp_z)
            self.object_info_obtained = False
            self.contact_tcp_z = None
            self.contact_tcp_xyz = None
            self.corrected_object = None
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
        self.singularity_deceleration_started = None
        self.singularity_progress_started = None
        self.singularity_progress_reference_z = None
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
        # Placement sign mode is validated by the bridge as a combined
        # delta-Fz magnitude plus sign condition. Do not bypass that check
        # here by consuming the raw magnitude alone.
        if self.servo_status.get('contact_on_fz_sign_change'):
            return False
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
        return 'singular' in normalized

    @staticmethod
    def _is_servo_external_wrench_limit(reason):
        normalized = str(reason).lower()
        return (
            'external wrench safety limit' in normalized or
            ('external wrench' in normalized and 'limit' in normalized)
        )

    def _place_singularity_requires_fallback(self, current_z, now):
        """Debounce Servo deceleration and detect lack of vertical progress."""
        if not self._servo_is_singularity_decelerating():
            self.singularity_deceleration_started = None
            self.singularity_progress_started = None
            self.singularity_progress_reference_z = None
            return None
        if getattr(self, 'singularity_deceleration_started', None) is None:
            self.singularity_deceleration_started = now
            self.singularity_progress_started = now
            self.singularity_progress_reference_z = current_z
            return None
        reference = getattr(self, 'singularity_progress_reference_z', None)
        if (reference is None or
                abs(float(reference) - current_z) >= self.singularity_min_progress):
            self.singularity_progress_started = now
            self.singularity_progress_reference_z = current_z
        deceleration_age = now - self.singularity_deceleration_started
        progress_age = now - float(
            getattr(self, 'singularity_progress_started', None) or now)
        if deceleration_age >= self.singularity_deceleration_grace:
            return (
                'MoveIt Servo remained in singularity deceleration for '
                f'{deceleration_age:.2f} s')
        if progress_age >= self.singularity_no_progress:
            return (
                'MoveIt Servo made less than '
                f'{self.singularity_min_progress * 1000.0:.1f} mm Z progress '
                f'for {progress_age:.2f} s near a singularity')
        return None

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
        # Do not start the step deadline until controller handoff is complete.
        # Starting it here allowed a slow handoff to consume the entire
        # recovery window and release the item without attempting one step.
        self.direct_place_deadline = None
        self.get_logger().warning(
            'place Servo reached its singularity recovery condition; '
            'switching to '
            f'{self.singularity_place_step * 1000.0:.1f} mm direct vertical '
            'steps with force checks between steps')
        try:
            self._capture_direct_tcp_z_offset()
        except ValueError as exc:
            self._fault(
                'cannot prepare direct singularity recovery TCP conversion: '
                f'{exc}')
            return
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
            if self.direct_tcp_z_offset is None:
                self._capture_direct_tcp_z_offset()
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
        if self.direct_tcp_z_offset is None:
            self._release_from_direct_place(
                'link_tcp to xArm SDK TCP Z conversion is unavailable')
            return
        sdk_floor_z = self.floor_z - self.direct_tcp_z_offset
        remaining = current_z - sdk_floor_z
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
        elif self.probe_only:
            try:
                corrected = self._finalize_contact_object_info()
            except ValueError as exc:
                self._fault(f'object information estimation failed: {exc}')
                return
            self.state = self.AWAITING_GRASP
            self.get_logger().info(
                'object information ready at contact: box=%d, '
                'size=(%.1f, %.1f, %.1f) mm, center_z=%.1f mm' % (
                    int(corrected['box_id']),
                    corrected['size_x_m'] * 1000.0,
                    corrected['size_y_m'] * 1000.0,
                    corrected['size_z_m'] * 1000.0,
                    corrected['center_z_m'] * 1000.0))
            self.publish_status()
        else:
            self._turn_vacuum_on()

    def control_tick(self):
        if self._tick_descent_config():
            return
        if self.state == self.AWAITING_GRASP:
            try:
                current = tuple(map(float, self._tcp_xyz()))
                displacement = math.sqrt(sum(
                    (value - measured) ** 2
                    for value, measured in zip(
                        current, self.contact_tcp_xyz)))
            except (TypeError, ValueError):
                return
            if displacement > self.pregrasp_z_tolerance:
                self.object_info_obtained = False
                self._fault(
                    'TCP moved away from the measured object contact pose; '
                    'object information invalidated')
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
        if getattr(self, 'robot_error', None) == 52:
            self._begin_c52_interruption()
            return
        if self.servo_status.get('state') == 'FAULT' or self.servo_status.get('fault'):
            reason = f"safe-servo fault: {self.servo_status.get('fault', 'unknown')}"
            if (self.operation_kind == 'place' and
                    self._is_servo_singularity_fault(reason)):
                self._begin_place_singularity_fallback(reason)
            elif (self.operation_kind == 'place' and
                  self._is_servo_external_wrench_limit(reason)):
                self._begin_place_release_fallback(
                    reason, 'external-wrench-limit')
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
            delta_fz = float(self.servo_status.get(
                'force_delta_z_n', 0.0))
            self.get_logger().info(
                f'guarded contact confirmed after '
                f'{descended * 1000.0:.1f} mm '
                f'(delta_fz={delta_fz:.2f} N, '
                f'threshold={self._contact_delta_n():.2f} N); '
                'stopping descent and triggering vacuum')
            self._handle_contact()
            return
        if self.operation_kind == 'place':
            singularity_reason = self._place_singularity_requires_fallback(
                current_z, time.monotonic())
            if singularity_reason:
                self._begin_place_singularity_fallback(singularity_reason)
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
        if self.state == self.CHECKING_LOADING_PATH:
            if time.monotonic() - self.loading_path_started > 10.0:
                self._fault('loading-path validation service timed out')
            return
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
            if self.robot_error not in (None, 0):
                self._fault(f'xArm error {self.robot_error} during linear loading')
                return
            try:
                current_z = self._direct_mode_tcp_xyz()[2]
            except ValueError as exc:
                self._fault(f'cannot monitor linear loading: {exc}')
                return
            command_target_z = self.direct_command_target_z
            if command_target_z is None:
                self._fault(
                    'xArm SDK loading target is unavailable during descent')
                return
            if current_z <= command_target_z + self.tolerance:
                self._begin_loading_completion_stop()
                return
            if (time.monotonic() - self.retreat_started >
                    self.place_descent_timeout):
                self._begin_loading_release_fallback(
                    f'linear loading exceeded '
                    f'{self.place_descent_timeout:.1f} s before reaching '
                    f'link_tcp pre-place Z {self.direct_target_z:.4f} m '
                    f'(xArm SDK Z {command_target_z:.4f} m)')
            return
        if time.monotonic() - self.retreat_started > self.retreat_timeout:
            self._fault('direct vertical retreat timed out')

    def _disable_servo_then_direct_retreat(self):
        if (not self.dry_run and self.operation_kind != 'transfer' and
                self.direct_tcp_z_offset is None):
            try:
                self._capture_direct_tcp_z_offset()
            except ValueError as exc:
                self._fault(
                    'cannot prepare direct vertical TCP conversion: '
                    f'{exc}')
                return
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
            if (self.operation_kind != 'transfer' and
                    self.direct_tcp_z_offset is None):
                self._capture_direct_tcp_z_offset()
            self.retreat_start_xyz = self._direct_mode_tcp_xyz()
        except ValueError as exc:
            if (not self.c52_retreat_active or
                    self.c52_tcp_snapshot is None):
                self._fault(f'cannot start direct retreat: {exc}')
                return
            self.retreat_start_xyz = self.c52_tcp_snapshot
            self.get_logger().warning(
                'using the C52-time TCP snapshot because live TCP telemetry '
                'became stale after the controller stop')
        self.retreat_start_z = self.retreat_start_xyz[2]
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
        if not self.hardware_list_client.service_is_ready():
            self._fault(
                'controller_manager hardware-list service is unavailable')
            return
        future = self.hardware_list_client.call_async(
            ListHardwareComponents.Request())
        future.add_done_callback(self._retreat_hardware_state_received)

    def _retreat_hardware_state_received(self, future):
        if self.state != self.PREPARING_RETREAT:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fault(
                f'failed to inspect robot hardware before retreat: {exc}')
            return
        component = next((
            item for item in (response.component if response else [])
            if item.name == self.hardware_component), None)
        if component is None:
            self._fault('robot hardware component is unavailable before retreat')
            return
        state_id = int(component.state.id)
        if state_id in (
                State.PRIMARY_STATE_INACTIVE,
                State.PRIMARY_STATE_UNCONFIGURED):
            if state_id == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().warning(
                    'ros2_control hardware is already unconfigured; treating '
                    'it as released ownership for fault recovery')
            else:
                self.get_logger().info(
                    'ros2_control hardware is already inactive')
            self._retreat_hardware_is_released()
            return
        if state_id != State.PRIMARY_STATE_ACTIVE:
            self._fault(
                'robot hardware is in unsupported lifecycle state before '
                f'retreat: {component.state.label}')
            return
        self._request_retreat_hardware_inactive()

    def _request_retreat_hardware_inactive(self):
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
        self._retreat_hardware_is_released()

    def _retreat_hardware_is_released(self):
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
        c52_retreat_active = getattr(self, 'c52_retreat_active', False)
        if (c52_retreat_active and
                getattr(self, 'c52_clear_attempts', 0) >= 1):
            label = self.mode_wait_label
            self._clear_mode_wait()
            self._fault(
                f'xArm C52 persisted after one clear attempt before {label}; '
                'the item remains held and automatic motion is blocked')
            return
        if c52_retreat_active:
            self.c52_clear_attempts += 1
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
            # Controller/mode handoff has its own timeout. The guarded-step
            # budget begins only now, immediately before the first command.
            self.direct_place_deadline = (
                time.monotonic() + self.singularity_place_recovery_timeout)
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
            if self.direct_tcp_z_offset is None:
                self._fault(
                    'link_tcp to xArm SDK TCP Z conversion is unavailable')
                return
            # direct_target_z is expressed for MoveIt's link_tcp. Convert it
            # to the SDK TCP origin before forming the relative set_position
            # displacement. For a +24 mm tool offset, link_tcp Z=235 mm is
            # therefore SDK TCP Z=211 mm.
            self.direct_command_target_z = (
                self.direct_target_z - self.direct_tcp_z_offset)
            delta = (
                0.0, 0.0, self.direct_command_target_z - current_z)
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
            self.direct_command_target_z <
            self.retreat_start_z - self.tolerance)
        # The downward loading move must not occupy xarm_api's service callback:
        # force or timeout handling needs /ufactory/set_state to remain callable.
        # Its completion is monitored from live TCP Z in retreat_tick().
        request.wait = not nonblocking_loading_descent
        self.state = self.RETREATING
        self.retreat_started = time.monotonic()
        self.retreat_target_z = (
            self.direct_target_z if self.operation_kind == 'transfer'
            else self.direct_command_target_z)
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
        elapsed = (
            0.0 if self.retreat_started is None else
            time.monotonic() - self.retreat_started)
        self.retreat_started = None
        motion_type = (
            'joint motion' if self.operation_kind == 'transfer' and
            self.direct_target_joints is not None else 'Cartesian motion')
        self.get_logger().info(
            f'direct {self.operation_kind} {motion_type} succeeded in '
            f'{elapsed:.3f} s')
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
        self.restore_settle_started = None
        self.restore_settle_ready_count = 0
        self.get_logger().info(
            'controllers restored after retreat; waiting for a new '
            '/joint_states sample before completing the cycle')

    def _restore_readiness_tick(self):
        now = time.monotonic()
        if self.restore_settle_started is not None:
            telemetry_fresh = (
                self.robot_state_time is not None and
                now - self.robot_state_time <= self.status_timeout)
            joint_state_fresh = (
                self.last_joint_state_time is not None and
                now - self.last_joint_state_time <= 0.25)
            ready = (
                telemetry_fresh and joint_state_fresh and
                self.robot_error == 0 and
                self.robot_mode == self.ros2_control_mode and
                self.robot_state is not None and self.robot_state <= 2)
            if ready:
                self.restore_settle_ready_count += 1
            else:
                self.restore_settle_ready_count = 0
            if (now - self.restore_settle_started >=
                    self.post_restore_settle and
                    self.restore_settle_ready_count >=
                    self.post_restore_ready_samples):
                self.restore_settle_started = None
                self.restore_settle_ready_count = 0
                self.restore_wait_deadline = None
                self.get_logger().info(
                    'post-restore xArm mode, state, error, and joint-state '
                    'telemetry remained stable; motion may resume')
                self._finish_retreat()
                return
            if (self.restore_wait_deadline is not None and
                    now >= self.restore_wait_deadline):
                self.restore_settle_started = None
                self.restore_settle_ready_count = 0
                self.restore_wait_deadline = None
                self._fault(
                    'ros2_control restored but xArm telemetry did not remain '
                    f'stable for {self.post_restore_settle:.2f} s; '
                    f'state={self.robot_state}, mode={self.robot_mode}, '
                    f'error={self.robot_error}')
            return
        if self.restore_wait_sequence is None:
            return
        if (self.joint_state_sequence >= (
                self.restore_wait_sequence + self.joint_state_ready_samples) and
                self.last_joint_state_time is not None and
                now - self.last_joint_state_time <= 0.25):
            self.restore_wait_sequence = None
            self.restore_settle_started = now
            self.restore_settle_ready_count = 0
            self.restore_wait_deadline = (
                now + self.post_restore_settle +
                self.joint_state_ready_timeout)
            self.get_logger().info(
                'fresh post-retreat /joint_states confirmed; beginning '
                f'{self.post_restore_settle:.2f} s stable-state gate')
            return
        if (self.restore_wait_deadline is not None and
                now >= self.restore_wait_deadline):
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
        if getattr(self, 'c52_retreat_active', False):
            self.c52_retreat_active = False
            self.c52_tcp_snapshot = None
            self.fault = (
                f'{self.ft_recovery_reason}; automatic descent was stopped '
                'and the item was preserved at the recovery waypoint. Remove '
                'the item, clear its planning-scene attachment, then run '
                'Recover FT sensor')
            self.state = self.FAULT
            self.get_logger().error(self.fault)
            self.publish_status()
            return
        if getattr(self, 'ft_recovery_restore_active', False):
            self.ft_recovery_restore_active = False
            self.ft_recovery_required = False
            self.ft_recovery_reason = ''
            self.ft_recovery_started = None
            self.ft_recovery_zeroed_at = None
            self.ft_recovery_attempt = 0
            self.state = self.IDLE
            self.fault = ''
            self.get_logger().info(
                'unloaded FT sensor recovery completed; automatic motion is '
                'available again')
            self.publish_status()
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
                direct_target_joints = getattr(
                    self, 'direct_target_joints', None)
                if direct_target_joints is not None:
                    actual_joints = tuple(self.latest_joint_positions or ())
                    if len(actual_joints) != len(direct_target_joints):
                        self._fault(
                            'cannot verify direct joint transfer: fresh arm '
                            'joint feedback is unavailable')
                        return
                    periodic_names = getattr(
                        self, 'direct_transfer_periodic_joint_names',
                        frozenset())
                    errors = tuple(
                        self._periodic_joint_error(actual, target)
                        if name in periodic_names else abs(actual - target)
                        for name, actual, target in zip(
                            self.arm_joint_names, actual_joints,
                            direct_target_joints)
                    )
                    max_index = max(range(len(errors)), key=errors.__getitem__)
                    tolerance = self.direct_transfer_joint_tolerance
                    if errors[max_index] > tolerance:
                        self._fault(
                            'transfer trajectory returned success but '
                            f'{self.arm_joint_names[max_index]} feedback error '
                            f'is {errors[max_index]:.3f} rad (limit '
                            f'{tolerance:.3f} rad)')
                        return
                else:
                    # Retain Cartesian verification for the legacy direct
                    # set_position fallback. xArm SDK TCP telemetry cannot be
                    # compared to the MoveIt link_tcp IK target because their
                    # tool references differ by a fixed offset.
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
                            f'target error is too large: '
                            f'xy={xy_error * 1000.0:.1f} mm, '
                            f'z={z_error * 1000.0:.1f} mm')
                        return
            self.direct_transfer_succeeded = True
        if (self.operation_kind == 'loading' and
                not self.loading_contact_fallback and not self.dry_run):
            try:
                current_link_z = self._link_tcp_xyz()[2]
            except ValueError as exc:
                self._fault(
                    f'cannot verify restored link_tcp loading target: {exc}')
                return
            error = current_link_z - self.direct_target_z
            if abs(error) > self.pregrasp_z_tolerance:
                self._fault(
                    'linear loading stopped outside the requested link_tcp '
                    f'pre-place height: error={error * 1000.0:+.1f} mm '
                    f'(limit {self.pregrasp_z_tolerance * 1000.0:.1f} mm)')
                return
            self.get_logger().info(
                'restored link_tcp verified at the loading target: '
                f'error={error * 1000.0:+.1f} mm')
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
        if self.staging_place_active:
            pose_is_fresh = (
                self.robot_tcp_pose is not None and
                self.robot_state_time is not None and
                time.monotonic() - self.robot_state_time <= self.status_timeout)
            if not pose_is_fresh:
                self._retreat_after_vacuum_fault(
                    'cannot release staged item without a fresh full TCP pose')
                return
            self.staging_release_tcp_pose = tuple(self.robot_tcp_pose)
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
        detach_client = (
            self.detach_staged_item_client
            if self.staging_place_active else self.detach_item_client)
        if not detach_client.service_is_ready():
            self._retreat_after_vacuum_fault(
                'item released but planning-scene detach service is unavailable')
            return
        self.state = self.DETACHING
        self.detach_started = time.monotonic()
        future = detach_client.call_async(Trigger.Request())
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
        transfer_goal = getattr(self, 'transfer_goal_handle', None)
        if transfer_goal is not None:
            transfer_goal.cancel_goal_async()
            self.transfer_goal_handle = None
        self.direct_motion_generation += 1
        self.direct_place_stepping = False
        self.fault = reason
        self.state = self.FAULT
        self.restore_wait_sequence = None
        self.restore_wait_deadline = None
        self.restore_settle_started = None
        self.restore_settle_ready_count = 0
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
        if getattr(self, 'ft_recovery_timer', None) is not None:
            self.ft_recovery_timer.cancel()
            self.ft_recovery_timer = None
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

    def recover_ft_sensor_callback(self, _request, response):
        """Recover and zero the FT sensor only with no attached item."""
        if self.state in self.ACTIVE:
            response.message = f'supervisor is active in {self.state}'
            return response
        attached_item = str(
            self.planning_scene_status.get('attached_item_id') or '')
        if attached_item:
            response.message = (
                f'cannot zero FT sensor while {attached_item} is attached; '
                'remove the physical item and clear its scene attachment first')
            return response
        required_clients = (
            ('FT enable', self.ft_enable_client),
            ('FT zero', self.ft_zero_client),
            ('clear error', self.clean_error_client),
            ('clear warning', self.clean_warn_client),
        )
        unavailable = [
            label for label, client in required_clients
            if not client.service_is_ready()
        ]
        if unavailable:
            response.message = (
                'FT recovery services unavailable: ' + ', '.join(unavailable))
            return response
        self.ft_recovery_required = True
        self.ft_recovery_reason = 'manual FT sensor recovery requested'
        self.ft_recovery_attempt = 0
        self.ft_recovery_started = time.monotonic()
        self.ft_recovery_restore_active = False
        self.post_retreat_fault = ''
        self.c52_retreat_active = False
        self.c52_clear_attempts = 0
        self.c52_tcp_snapshot = None
        self.state = self.RECOVERING_FT
        self.fault = ''
        self._start_ft_recovery_attempt()
        response.success = True
        response.message = (
            'unloaded FT sensor recovery started; robot motion is blocked '
            'until verification completes')
        self.publish_status()
        return response

    def _start_ft_recovery_attempt(self):
        if self.state != self.RECOVERING_FT:
            return
        self.ft_recovery_attempt += 1
        self.get_logger().info(
            f'FT recovery attempt {self.ft_recovery_attempt}/'
            f'{self.ft_recovery_max_attempts}: disabling sensor')
        request = SetInt16.Request()
        request.data = 0
        future = self.ft_enable_client.call_async(request)
        future.add_done_callback(self._ft_recovery_sensor_disabled)

    def _ft_recovery_result(self, future, label):
        try:
            result = future.result()
        except Exception as exc:
            self._retry_or_latch_ft_recovery(f'{label} failed: {exc}')
            return False
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._retry_or_latch_ft_recovery(
                f'{label} rejected: ret={code}')
            return False
        return True

    def _ft_recovery_sensor_disabled(self, future):
        if self.state != self.RECOVERING_FT or not self._ft_recovery_result(
                future, 'disable FT sensor'):
            return
        future = self.clean_error_client.call_async(Call.Request())
        future.add_done_callback(self._ft_recovery_error_cleared)

    def _ft_recovery_error_cleared(self, future):
        if self.state != self.RECOVERING_FT or not self._ft_recovery_result(
                future, 'clear xArm error'):
            return
        future = self.clean_warn_client.call_async(Call.Request())
        future.add_done_callback(self._ft_recovery_warning_cleared)

    def _ft_recovery_warning_cleared(self, future):
        if self.state != self.RECOVERING_FT or not self._ft_recovery_result(
                future, 'clear xArm warning'):
            return
        request = SetInt16.Request()
        request.data = 1
        future = self.ft_enable_client.call_async(request)
        future.add_done_callback(self._ft_recovery_sensor_enabled)

    def _ft_recovery_sensor_enabled(self, future):
        if self.state != self.RECOVERING_FT or not self._ft_recovery_result(
                future, 'enable FT sensor'):
            return
        self._schedule_ft_recovery_timer(self._ft_recovery_zero_sensor)

    def _schedule_ft_recovery_timer(self, callback):
        if self.ft_recovery_timer is not None:
            self.ft_recovery_timer.cancel()
        self.ft_recovery_timer = self.create_timer(
            self.ft_recovery_settle, callback)

    def _ft_recovery_zero_sensor(self):
        if self.ft_recovery_timer is not None:
            self.ft_recovery_timer.cancel()
            self.ft_recovery_timer = None
        if self.state != self.RECOVERING_FT:
            return
        future = self.ft_zero_client.call_async(Call.Request())
        future.add_done_callback(self._ft_recovery_sensor_zeroed)

    def _ft_recovery_sensor_zeroed(self, future):
        if self.state != self.RECOVERING_FT or not self._ft_recovery_result(
                future, 'zero FT sensor'):
            return
        self.ft_recovery_zeroed_at = time.monotonic()
        self._schedule_ft_recovery_timer(self._verify_ft_recovery)

    def _verify_ft_recovery(self):
        if self.ft_recovery_timer is not None:
            self.ft_recovery_timer.cancel()
            self.ft_recovery_timer = None
        if self.state != self.RECOVERING_FT:
            return
        now = time.monotonic()
        robot_state_fresh = (
            self.robot_state_time is not None and
            now - self.robot_state_time <= self.status_timeout)
        force_fresh = (
            self.last_force_time is not None and
            self.last_force_time >= self.ft_recovery_zeroed_at and
            now - self.last_force_time <= self.force_timeout)
        if not robot_state_fresh:
            self._retry_or_latch_ft_recovery(
                'robot telemetry remained stale after FT zero')
            return
        if self.robot_error not in (None, 0):
            self._retry_or_latch_ft_recovery(
                f'xArm error {self.robot_error} remained after FT zero')
            return
        if not force_fresh:
            self._retry_or_latch_ft_recovery(
                'no fresh force sample arrived after FT zero')
            return
        self.get_logger().info(
            'FT sensor zero and fresh force data verified; restoring '
            'ros2_control')
        self.ft_recovery_restore_active = True
        self._restore_ros2_control_mode()

    def _retry_or_latch_ft_recovery(self, reason):
        if self.state != self.RECOVERING_FT:
            return
        self.ft_recovery_reason = reason
        if self.ft_recovery_attempt < self.ft_recovery_max_attempts:
            self.get_logger().warning(
                f'{reason}; retrying unloaded FT recovery')
            self._schedule_ft_recovery_timer(self._restart_ft_recovery_attempt)
            return
        self.ft_recovery_restore_active = False
        self._fault(
            f'FT recovery failed after {self.ft_recovery_attempt} attempts: '
            f'{reason}. Inspect sensor wiring and power before retrying')

    def _restart_ft_recovery_attempt(self):
        if self.ft_recovery_timer is not None:
            self.ft_recovery_timer.cancel()
            self.ft_recovery_timer = None
        self._start_ft_recovery_attempt()

    def reset_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'cannot reset active pickup in {self.state}'
            return response
        if getattr(self, 'ft_recovery_required', False):
            response.message = (
                'FT recovery is required; remove any held item and run '
                'Recover FT sensor before resetting')
            return response
        self.state = self.IDLE
        self.fault = ''
        self.pregrasp_z = None
        self.direct_target_z = None
        self.direct_target_pose = None
        self.direct_target_joints = None
        self.direct_tcp_z_offset = None
        self.direct_command_target_z = None
        self.direct_transfer_validation_samples = []
        self.direct_transfer_validation_index = 0
        self.direct_transfer_succeeded = False
        self.direct_transfer_motion_started = False
        self.direct_transfer_execution_started = None
        self.transfer_goal_handle = None
        self.transfer_fallback_reason = ''
        self.floor_z = None
        self.virtual_z = None
        self.descent_target_z = None
        self.vacuum_verified = False
        self.vacuum_verify_count = 0
        self.contact_detected = False
        self.probe_only = False
        self.object_info_obtained = False
        self.contact_tcp_z = None
        self.contact_tcp_xyz = None
        self.corrected_object = None
        self.active_pickup_snapshot = None
        self.place_fallback_used = False
        self.place_fallback_reason = ''
        self.direct_place_recovery_active = False
        self.direct_place_stepping = False
        self.direct_place_deadline = None
        self.direct_place_force_baseline_z = None
        self.direct_place_step_count = 0
        self.direct_place_step_completed_at = None
        self.singularity_deceleration_started = None
        self.singularity_progress_started = None
        self.singularity_progress_reference_z = None
        self.loading_contact_fallback = False
        self.loading_stop_started = None
        self.loading_stop_action = ''
        self.loading_force_baseline_z = None
        self.loading_force_over_count = 0
        self.loading_transfer_z = None
        self.expected_enable_generation = None
        self.operation_kind = 'pickup'
        self.staging_place_active = False
        self.staging_release_tcp_pose = None
        self.detach_started = None
        self.retreat_started = None
        self.retreat_target_z = None
        self.retreat_start_z = None
        self.retreat_start_xyz = None
        self.post_retreat_fault = ''
        self.c52_retreat_active = False
        self.c52_clear_attempts = 0
        self.c52_tcp_snapshot = None
        self.ft_recovery_restore_active = False
        self.restore_wait_sequence = None
        self.restore_wait_deadline = None
        self.restore_settle_started = None
        self.restore_settle_ready_count = 0
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
        if getattr(self, 'ft_recovery_timer', None) is not None:
            self.ft_recovery_timer.cancel()
            self.ft_recovery_timer = None
        response.success = True
        response.message = 'pickup supervisor reset to IDLE'
        return response

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state,
            'operation_kind': self.operation_kind,
            'staging_place_active': self.staging_place_active,
            'release_tcp_pose_mm_rad': self.staging_release_tcp_pose,
            'fault': self.fault,
            'dry_run': self.dry_run,
            'pregrasp_z_m': self.pregrasp_z,
            'floor_z_m': self.floor_z,
            'descent_target_z_m': self.descent_target_z,
            'retreat_target_z_m': self.retreat_target_z,
            'direct_tcp_z_offset_m': self.direct_tcp_z_offset,
            'direct_command_target_z_m': self.direct_command_target_z,
            'transfer_corner_height_pallet_m': self.transfer_corner_height,
            'non_servo_speed_percent': self.motion_speed_percent,
            'direct_cartesian_speed_mm_s': self.retreat_speed,
            'direct_cartesian_acc_mm_s2': self.retreat_acc,
            'place_workspace_z_min_m': self.place_workspace_z_min_mm / 1000.0,
            'place_descent_timeout_sec': self.place_descent_timeout,
            'singularity_place_recovery_timeout_sec':
                self.singularity_place_recovery_timeout,
            'singularity_deceleration_grace_sec':
                self.singularity_deceleration_grace,
            'contact_detected': self.contact_detected,
            'probe_only': self.probe_only,
            'object_info_obtained': self.object_info_obtained,
            'contact_tcp_z_m': self.contact_tcp_z,
            'contact_tcp_xyz_m': self.contact_tcp_xyz,
            'contact_reference_z_m': self.contact_reference_z,
            'corrected_object': self.corrected_object,
            'place_fallback_used': self.place_fallback_used,
            'place_fallback_reason': self.place_fallback_reason,
            'direct_place_recovery_active': self.direct_place_recovery_active,
            'direct_place_step_count': self.direct_place_step_count,
            'singularity_place_step_mm':
                self.singularity_place_step * 1000.0,
            'direct_transfer_succeeded': self.direct_transfer_succeeded,
            'direct_transfer_motion_started': self.direct_transfer_motion_started,
            'direct_transfer_validation_sample':
                self.direct_transfer_validation_index,
            'direct_transfer_validation_total':
                len(self.direct_transfer_validation_samples),
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
            'ft_recovery_required': self.ft_recovery_required,
            'ft_recovery_reason': self.ft_recovery_reason,
            'ft_recovery_attempt': self.ft_recovery_attempt,
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
