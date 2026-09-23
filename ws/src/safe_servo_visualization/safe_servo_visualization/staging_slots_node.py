import json
import math
import re
import time
from pathlib import Path
import yaml

from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState,
    SwitchController)
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import Call, MoveJoint, SetInt16
from .pick_place_workflow import pick_and_place
from .pick_path_client import PickPathClient
from .slot_inspection import SlotInspection
from .top_face_inspection_client import TopFaceInspectionClient, inspection_enabled, marker_target


class StagingSlots(SlotInspection, Node):
    """Store and retrieve carried boxes in six fixed robot-base slots."""

    IDLE = 'IDLE'
    PREPARING_STORE_TRANSFER = 'PREPARING_STORE_TRANSFER'
    TRANSFERRING_STORE = 'TRANSFERRING_STORE'
    LOADING_STORE = 'LOADING_STORE'
    SOLVING_STORE_IK = 'SOLVING_STORE_IK'
    PREPARING_DIRECT = 'PREPARING_DIRECT'
    MOVING_DIRECT = 'MOVING_DIRECT'
    RESTORING_CONTROL = 'RESTORING_CONTROL'
    SETTLING_STORE = 'SETTLING_STORE'
    PLACING_STORE = 'PLACING_STORE'
    PREPARING_RETRIEVAL = 'PREPARING_RETRIEVAL'
    PLANNING_RETRIEVAL_APPROACH = 'PLANNING_RETRIEVAL_APPROACH'
    EXECUTING_RETRIEVAL_APPROACH = 'EXECUTING_RETRIEVAL_APPROACH'
    PLANNING_RETRIEVAL = 'PLANNING_RETRIEVAL'
    EXECUTING_RETRIEVAL = 'EXECUTING_RETRIEVAL'
    SETTLING_RETRIEVAL = 'SETTLING_RETRIEVAL'
    PICKING_RETRIEVAL = 'PICKING_RETRIEVAL'
    PLANNING_OBSERVATION = 'PLANNING_OBSERVATION'
    EXECUTING_OBSERVATION = 'EXECUTING_OBSERVATION'
    SUCCEEDED = 'SUCCEEDED'
    FAULT = 'FAULT'

    ACTIVE = {
        'SAM_INSPECTION',
        SlotInspection.INSPECTION_FK, SlotInspection.INSPECTION_MOVE,
        SlotInspection.INSPECTION_DEPTH,
        SlotInspection.INSPECTION_CHECK,
        PREPARING_STORE_TRANSFER, TRANSFERRING_STORE, LOADING_STORE,
        SOLVING_STORE_IK, PREPARING_DIRECT, MOVING_DIRECT,
        RESTORING_CONTROL, SETTLING_STORE, PLACING_STORE, PREPARING_RETRIEVAL,
        PLANNING_RETRIEVAL, EXECUTING_RETRIEVAL, SETTLING_RETRIEVAL,
        PICKING_RETRIEVAL,
        PLANNING_RETRIEVAL_APPROACH, EXECUTING_RETRIEVAL_APPROACH,
        PLANNING_OBSERVATION, EXECUTING_OBSERVATION,
    }

    def __init__(self):
        super().__init__('staging_slots')
        self.declare_parameter('slot_x_min_m', -0.375)
        self.declare_parameter('slot_x_max_m', 0.375)
        self.declare_parameter('slot_y_min_m', 0.180)
        self.declare_parameter('slot_y_max_m', 0.680)
        self.declare_parameter('slot_size_m', 0.250)
        self.declare_parameter('slot_surface_z_m', 0.0)
        self.declare_parameter('pre_pick_clearance_m', 0.030)
        self.declare_parameter('transfer_item_bottom_above_pallet_m', 0.480)
        self.declare_parameter('configured_container_clearance_m', 0.470)
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('pallet_frame', 'pallet_frame')
        self.declare_parameter('retrieval_settle_sec', 0.25)
        self.declare_parameter('retrieval_settle_timeout_sec', 5.0)
        self.declare_parameter('retrieval_settle_samples', 3)
        self.declare_parameter('retrieval_xy_tolerance_m', 0.015)
        self.declare_parameter('retrieval_z_tolerance_m', 0.015)
        self.declare_parameter('motion_timeout_sec', 120.0)
        self.declare_parameter('joint_speed_rad_s', 0.30)
        self.declare_parameter('joint_acc_rad_s2', 0.70)
        self.declare_parameter('joint_max_speed_rad_s', 2.14)
        self.declare_parameter('joint_max_acc_rad_s2', 10.0)
        self.declare_parameter('planning_group', 'uf850')
        self.declare_parameter('ik_link_name', 'link_tcp')
        self.declare_parameter('ik_timeout_sec', 1.0)
        self.declare_parameter(
            'arm_joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('trajectory_controller', 'uf850_traj_controller')
        self.declare_parameter('joint_state_broadcaster', 'joint_state_broadcaster')
        self.declare_parameter(
            'hardware_component', 'uf_robot_hardware/UFRobotSystemHardware')
        self.declare_parameter('ros2_control_mode', 1)
        self.declare_parameter('mode_transition_timeout_sec', 5.0)
        self.declare_parameter('mode_retry_interval_sec', 0.25)
        self.declare_parameter('mode_ready_samples', 3)
        self.declare_parameter('joint_state_ready_timeout_sec', 5.0)
        self.declare_parameter('joint_state_ready_samples', 3)
        self.declare_parameter('status_timeout_sec', 1.0)

        def p(name):
            return self.get_parameter(name).value

        self.slot_x_min = float(p('slot_x_min_m'))
        self.slot_x_max = float(p('slot_x_max_m'))
        self.slot_y_min = float(p('slot_y_min_m'))
        self.slot_y_max = float(p('slot_y_max_m'))
        self.slot_size = float(p('slot_size_m'))
        self.surface_z = float(p('slot_surface_z_m'))
        self.clearance = float(p('pre_pick_clearance_m'))
        self.transfer_item_bottom_above_pallet = max(float(
            p('transfer_item_bottom_above_pallet_m')), float(p('configured_container_clearance_m')))
        self.base_frame = str(p('base_frame'))
        self.pallet_frame = str(p('pallet_frame'))
        self.retrieval_settle = max(0.0, float(p('retrieval_settle_sec')))
        self.retrieval_settle_timeout = float(
            p('retrieval_settle_timeout_sec'))
        self.retrieval_settle_samples = max(
            1, int(p('retrieval_settle_samples')))
        self.retrieval_xy_tolerance = float(p('retrieval_xy_tolerance_m'))
        self.retrieval_z_tolerance = float(p('retrieval_z_tolerance_m'))
        self.motion_timeout = float(p('motion_timeout_sec'))
        self.joint_speed = float(p('joint_speed_rad_s'))
        self.joint_acc = float(p('joint_acc_rad_s2'))
        self.joint_max_speed = float(p('joint_max_speed_rad_s'))
        self.joint_max_acc = float(p('joint_max_acc_rad_s2'))
        self.planning_group = str(p('planning_group'))
        self.ik_link_name = str(p('ik_link_name'))
        self.ik_timeout = float(p('ik_timeout_sec'))
        self.arm_joint_names = tuple(map(str, p('arm_joint_names')))
        self.trajectory_controller = str(p('trajectory_controller'))
        self.joint_state_broadcaster = str(p('joint_state_broadcaster'))
        self.hardware_component = str(p('hardware_component'))
        self.ros2_control_mode = int(p('ros2_control_mode'))
        self.mode_transition_timeout = float(p('mode_transition_timeout_sec'))
        self.mode_retry_interval = float(p('mode_retry_interval_sec'))
        self.mode_ready_samples = int(p('mode_ready_samples'))
        self.joint_state_ready_timeout = float(p('joint_state_ready_timeout_sec'))
        self.joint_state_ready_samples = int(p('joint_state_ready_samples'))
        self.status_timeout = float(p('status_timeout_sec'))

        if self.transfer_item_bottom_above_pallet <= self.clearance:
            raise ValueError(
                'transfer_item_bottom_above_pallet_m must exceed '
                'pre_pick_clearance_m')
        if self.retrieval_settle_timeout <= 0.0:
            raise ValueError('retrieval_settle_timeout_sec must be positive')
        if (self.joint_speed <= 0.0 or self.joint_acc <= 0.0 or
                self.joint_max_speed <= 0.0 or self.joint_max_acc <= 0.0):
            raise ValueError('staging joint speed and acceleration must be positive')
        if self.ik_timeout <= 0.0:
            raise ValueError('ik_timeout_sec must be positive')
        if len(self.arm_joint_names) != 6 or len(set(self.arm_joint_names)) != 6:
            raise ValueError('arm_joint_names must contain six unique joints')

        self.slots = self._make_slots()
        self.selected_store_slot = 0
        self.selected_retrieve_slot = 0
        self.occupied = {}
        self.state = self.IDLE
        self.operation = ''
        self.fault = ''
        self.last_result = ''
        self.phase_started = None
        self.scene_status = {}
        self.motion_status = {}
        self.pickup_status = {}
        self.servo_status = {}
        self.servo_status_time = None
        self.servo_status_sequence = 0
        self.robot_state = None
        self.robot_mode = None
        self.robot_error = None
        self.robot_state_time = None
        self.robot_tcp_xyz = None
        self.robot_tcp_sequence = 0
        self.latest_joint_state = None
        self.motion_queue = []
        self.motion_pending = False
        self.ik_targets = []
        self.ik_solutions = []
        self.pending_record = None
        self.active_slot = None
        self.direct_phase = ''
        self.direct_control_claim_started = False
        self.pending_fault = ''
        self.placed_ids_before_store = set()
        self.joint_state_sequence = 0
        self.last_joint_state_time = None
        self.restore_wait_sequence = None
        self.restore_wait_deadline = None
        self.mode_wait_target = None
        self.mode_wait_label = ''
        self.mode_wait_callback = None
        self.mode_wait_deadline = None
        self.mode_wait_last_command = None
        self.mode_wait_command_pending = False
        self.mode_wait_ready_count = 0
        self.expected_motion_operation_id = None
        self.expected_pickup_operation_id = None
        self.expected_store_place_operation_id = None
        self.expected_store_transfer_operation_id = None
        self.store_transfer_phase = ''
        self.store_transfer_fallback_used = False
        self.store_transfer_fallback_reason = ''
        self.return_to_observation = True
        self.waiting_removed_id = ''
        self.retrieval_settle_started = None
        self.retrieval_settle_after_sequence = 0
        self.retrieval_last_checked_sequence = 0
        self.retrieval_converged_samples = 0
        self._staging_place_timer = None
        self._store_transfer_timer = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._init_inspection()
        config_path = str(self.get_parameter('slot_inspection_policy_config_path').value)
        config = yaml.safe_load(Path(config_path).read_text()) if config_path else {}
        self.sam_inspection_enabled = inspection_enabled(config or {})
        self.sam_inspector = TopFaceInspectionClient(self, 'slots')
        self.sam_retrieval_result = None

        self.status_pub = self.create_publisher(String, '/staging_slots/status', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/staging_slots/markers', 10)
        self.retrieve_target_pub = self.create_publisher(
            Float64MultiArray, '/staging_slots/retrieve_target', 10)
        self.place_config_pub = self.create_publisher(
            Float64MultiArray, '/staging_slots/place_config', 10)
        self.store_transfer_target_pub = self.create_publisher(
            Float64MultiArray, '/staging_slots/store_transfer_target', 10)
        self.create_subscription(
            Int32, '/staging_slots/select_store', self._select_store, 10)
        self.create_subscription(
            Int32, '/staging_slots/select_retrieve', self._select_retrieve, 10)
        self.create_subscription(
            String, '/planning_scene_obstacles/status', self._scene_status, 10)
        self.create_subscription(
            String, '/motion_coordinator/status', self._motion_status, 10)
        self.create_subscription(
            String, '/pickup_supervisor/status', self._pickup_status, 10)
        self.create_subscription(
            String, '/safe_servo/status', self._servo_status, 10)
        self.create_subscription(
            RobotMsg, '/ufactory/robot_states', self._robot_state, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state, 10)
        self.create_subscription(
            Float64MultiArray, '/motion_speed/config', self._motion_speed, 10)

        self.set_mode = self.create_client(SetInt16, '/ufactory/set_mode')
        self.set_state = self.create_client(SetInt16, '/ufactory/set_state')
        self.clean_error = self.create_client(Call, '/ufactory/clean_error')
        self.compute_ik = self.create_client(GetPositionIK, '/compute_ik')
        self.move_joint = self.create_client(
            MoveJoint, '/ufactory/set_servo_angle')
        self.list_controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.switch_controller = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.list_hardware = self.create_client(
            ListHardwareComponents,
            '/controller_manager/list_hardware_components')
        self.set_hardware = self.create_client(
            SetHardwareComponentState,
            '/controller_manager/set_hardware_component_state')
        self.remove_staged_obstacle = self.create_client(
            SetInt16, '/planning_scene_obstacles/remove_placed_item')
        self.plan_staging_pregrasp = self.create_client(
            Trigger, '/motion_coordinator/plan_staging_pregrasp')
        self.plan_staging_approach = self.create_client(
            Trigger, '/motion_coordinator/plan_staging_approach')
        self.pick_path = PickPathClient(self, self._fault)
        self.prepare_staging_store_transfer = self.create_client(
            Trigger, '/motion_coordinator/prepare_staging_store_transfer')
        self.start_joint_transfer = self.create_client(
            Trigger, '/pickup_supervisor/start_joint_transfer')
        self.accept_direct_transfer = self.create_client(
            Trigger, '/motion_coordinator/accept_direct_transfer')
        self.plan_transfer = self.create_client(
            Trigger, '/motion_coordinator/plan_transfer')
        self.start_loading = self.create_client(
            Trigger, '/pickup_supervisor/start_loading')
        self.plan_observation = self.create_client(
            Trigger, '/motion_coordinator/plan_observation')
        self.execute_motion = self.create_client(
            Trigger, '/motion_coordinator/execute')
        self.start_pickup = self.create_client(
            Trigger, '/pickup_supervisor/start')
        self.start_pickup_hold = self.create_client(
            Trigger, '/pickup_supervisor/start_for_transport')
        self.start_transport = self.create_client(
            Trigger, '/pickup_supervisor/start_continuous_transport')
        self.start_transport_chained = self.create_client(
            Trigger, '/pickup_supervisor/start_continuous_transport_chained')
        self.start_staging_place = self.create_client(
            Trigger, '/pickup_supervisor/start_staging_place')
        self.enable_servo = self.create_client(SetBool, '/safe_servo/enable')

        self.create_service(Trigger, '/staging_slots/store', self.store_callback)
        self.create_service(
            Trigger, '/staging_slots/store_chained',
            self.store_chained_callback)
        self.create_service(
            Trigger, '/staging_slots/retrieve', self.retrieve_callback)
        self.create_service(
            Trigger, '/staging_slots/retrieve_chained', self.retrieve_chained_callback)
        self.create_service(Trigger, '/staging_slots/reset', self.reset_callback)
        self.create_service(SetInt16, '/staging_slots/accept_debug_pick', self.accept_debug_pick_callback)
        self.abort_supervisor = self.create_client(Trigger, '/pickup_supervisor/abort')
        self.cancel_motion = self.create_client(Trigger, '/motion_coordinator/cancel')
        self.abort_latched = False
        self.create_service(Trigger, '/staging_slots/abort', self.abort_callback)
        for topic in ('/random_stable_loading/status', '/policy_loading/status'):
            self.create_subscription(String, topic, self._container_clearance_status, 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self.publish_status)
        self.publish_status()

    def _container_clearance_status(self, message):
        try:
            status = json.loads(message.data)
            size = status.get('container_size_mm')
            if not isinstance(size, (list, tuple)) or len(size) != 3:
                return
            height = max(float(size[2])/1000.0 + .02,
                         float(status.get('transfer_corner_height_m') or 0.))
            if math.isfinite(height) and height > 0:
                # Never lower the clearance during an active/chained operation.
                self.transfer_item_bottom_above_pallet = max(
                    self.transfer_item_bottom_above_pallet, height)
        except (TypeError, ValueError):
            return

    def _make_slots(self):
        nx = round((self.slot_x_max - self.slot_x_min) / self.slot_size)
        ny = round((self.slot_y_max - self.slot_y_min) / self.slot_size)
        if nx != 3 or ny != 2:
            raise ValueError('staging bounds and slot_size must form a 3x2 grid')
        return [
            (self.slot_x_min + column * self.slot_size,
             self.slot_y_min + row * self.slot_size,
             self.surface_z)
            for row in range(ny) for column in range(nx)
        ]

    @staticmethod
    def _decode(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError, json.JSONDecodeError):
            return {}

    def _scene_status(self, message):
        self.scene_status = self._decode(message)

    def _motion_status(self, message):
        self.motion_status = self._decode(message)

    def _pickup_status(self, message):
        self.pickup_status = self._decode(message)

    def _servo_status(self, message):
        self.servo_status = self._decode(message)
        self.servo_status_time = time.monotonic()
        self.servo_status_sequence += 1

    def _robot_state(self, message):
        self.robot_state = int(message.state)
        self.robot_mode = int(message.mode)
        self.robot_error = int(message.err)
        self.robot_state_time = time.monotonic()
        if len(message.pose) >= 3:
            xyz = tuple(float(value) / 1000.0 for value in message.pose[:3])
            if all(math.isfinite(value) for value in xyz):
                self.robot_tcp_xyz = xyz
                self.robot_tcp_sequence += 1

    def _joint_state(self, message):
        self.joint_state_sequence += 1
        self.last_joint_state_time = time.monotonic()
        positions = {
            str(name): float(position)
            for name, position in zip(message.name, message.position)
            if math.isfinite(float(position))
        }
        if all(name in positions for name in self.arm_joint_names):
            self.latest_joint_state = tuple(
                positions[name] for name in self.arm_joint_names)

    def _motion_speed(self, message):
        if len(message.data) >= 2:
            speed = float(message.data[1])
            if math.isfinite(speed) and 5.0 <= speed <= 100.0:
                scale = speed / 100.0
                self.joint_speed = self.joint_max_speed * scale
                self.joint_acc = self.joint_max_acc * scale

    def _select_store(self, message):
        if 0 <= int(message.data) < len(self.slots):
            self.selected_store_slot = int(message.data)
            self.publish_status()

    def _select_retrieve(self, message):
        if 0 <= int(message.data) < len(self.slots):
            self.selected_retrieve_slot = int(message.data)
            self.publish_status()

    def _busy_reason(self, *, allow_contact=False):
        if getattr(self, 'abort_latched', False):
            return 'staging abort latched; stop/reconcile and reset before another operation'
        if self.state in self.ACTIVE:
            return f'staging operation active in {self.state}'
        motion_state = self.motion_status.get('state')
        if motion_state not in (None, 'IDLE', 'SUCCEEDED', 'FAULT'):
            return f'MoveIt coordinator active in {motion_state}'
        pickup_state = self.pickup_status.get('state')
        if (pickup_state not in (None, 'IDLE', 'SUCCEEDED', 'FAULT') and
                not (allow_contact and pickup_state == 'AWAITING_GRASP')):
            return f'pickup supervisor active in {pickup_state}'
        return ''

    def _attached_geometry(self):
        item_id = str(self.scene_status.get('attached_item_id') or '')
        size = self.scene_status.get('attached_item_size_m')
        center = self.scene_status.get('attached_item_center_in_tcp_m')
        orientation = self.scene_status.get(
            'attached_item_orientation_in_tcp_xyzw')
        if not item_id or not all(isinstance(value, (list, tuple)) for value in (
                size, center, orientation)):
            raise ValueError('no carried item is attached; pick an item first')
        if len(size) != 3 or len(center) != 3 or len(orientation) != 4:
            raise ValueError('attached-item geometry is incomplete')
        values = tuple(map(float, [*size, *center, *orientation]))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('attached-item geometry contains non-finite values')
        return {
            'item_id': item_id,
            'size': tuple(map(float, size)),
            'center': tuple(map(float, center)),
            'orientation': tuple(map(float, orientation)),
        }

    @staticmethod
    def _quat_inverse(quaternion):
        x, y, z, w = map(float, quaternion)
        norm = x*x + y*y + z*z + w*w
        if norm <= 1e-12:
            raise ValueError('attached-item orientation is invalid')
        return (-x/norm, -y/norm, -z/norm, w/norm)

    @staticmethod
    def _quat_multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw*rx + lx*rw + ly*rz - lz*ry,
            lw*ry - lx*rz + ly*rw + lz*rx,
            lw*rz + lx*ry - ly*rx + lz*rw,
            lw*rw - lx*rx - ly*ry - lz*rz,
        )

    @classmethod
    def _quat_rotate(cls, vector, quaternion):
        x, y, z, w = quaternion
        return cls._quat_multiply(
            cls._quat_multiply(quaternion, (*vector, 0.0)),
            (-x, -y, -z, w))[:3]

    @staticmethod
    def _rpy_from_quaternion(quaternion):
        x, y, z, w = quaternion
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw

    def _store_target(self, slot, geometry):
        size_x, size_y, size_z = geometry['size']
        tcp_q = geometry.get('store_tcp_q')
        if tcp_q is None:
            tcp_q = self._quat_inverse(geometry['orientation'])
        object_q = self._quat_multiply(tcp_q, geometry['orientation'])
        roll, pitch, yaw = self._rpy_from_quaternion(object_q)
        if max(abs(roll), abs(pitch)) > math.radians(2.5):
            raise ValueError('fixed-orientation slot placement requires a level item (within 2.5 degrees)')
        # Bound all rotated corners, including small measured tilt. Align the
        # footprint minimum X/Y and lowest corner with the slot origin.
        corners = [self._quat_rotate((sx*size_x/2, sy*size_y/2, sz*size_z/2), object_q)
                   for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        low = [min(p[i] for p in corners) for i in range(3)]
        extent = [max(p[i] for p in corners)-low[i] for i in range(3)]
        if extent[0] > self.slot_size + 1e-9 or extent[1] > self.slot_size + 1e-9:
            raise ValueError(
                f'item footprint {extent[0]*1000:.0f}x{extent[1]*1000:.0f} mm '
                f'does not fit the 250x250 mm slot at its current orientation')
        object_center = tuple(slot[i]-low[i] for i in range(3))
        tcp_to_center = self._quat_rotate(geometry['center'], tcp_q)
        tcp_xyz = tuple(
            object_center[index] - tcp_to_center[index] for index in range(3))
        return (*tcp_xyz, *self._rpy_from_quaternion(tcp_q))

    def _pallet_origin_z(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.pallet_frame, rclpy.time.Time())
        except TransformException as exc:
            raise ValueError(f'live pallet transform is unavailable: {exc}')
        z = float(transform.transform.translation.z)
        if not math.isfinite(z):
            raise ValueError('live pallet transform has invalid Z')
        return z

    @staticmethod
    def _transfer_tcp_z_for_item_bottom(
            contact_tcp_z, slot_surface_z, pallet_origin_z,
            item_bottom_above_pallet):
        desired_item_bottom_z = (
            float(pallet_origin_z) + float(item_bottom_above_pallet))
        return float(contact_tcp_z) + (
            desired_item_bottom_z - float(slot_surface_z))

    @staticmethod
    def _quaternion_from_rpy(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy,
        )

    def store_callback(self, _request, response):
        return self._store_callback(response, return_to_observation=True)

    def store_chained_callback(self, _request, response):
        return self._store_callback(response, return_to_observation=False)

    def _store_callback(self, response, return_to_observation):
        reason = self._busy_reason()
        if reason:
            response.message = reason
            return response
        slot_index = self.selected_store_slot
        if slot_index in self.occupied:
            response.message = f'staging slot {slot_index} is already occupied'
            return response
        try:
            geometry = self._attached_geometry()
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, 'link_tcp', rclpy.time.Time())
            stamp = transform.header.stamp
            age = self.get_clock().now().nanoseconds*1e-9 - (stamp.sec + stamp.nanosec*1e-9)
            if not math.isfinite(age) or not 0 <= age <= self.status_timeout:
                raise ValueError('current TCP orientation TF is stale; cannot freeze slot-store orientation')
            q = transform.transform.rotation
            raw = (q.x, q.y, q.z, q.w)
            norm = math.sqrt(sum(v*v for v in raw))
            if not math.isfinite(norm) or norm < 1e-9:
                raise ValueError('invalid current TCP orientation')
            geometry['store_tcp_q'] = tuple(v/norm for v in raw)
            target = self._store_target(self.slots[slot_index], geometry)
            pallet_origin_z = self._pallet_origin_z()
        except (ValueError, TransformException) as exc:
            response.message = str(exc)
            return response
        if not self.prepare_staging_store_transfer.service_is_ready():
            response.message = 'staging transfer preparation service is unavailable'
            return response
        self.operation = 'store'
        self.expected_store_place_operation_id = None
        self.expected_store_transfer_operation_id = None
        self.workflow = pick_and_place('carried', 'slot', return_to_observation=bool(return_to_observation))
        self.return_to_observation = bool(return_to_observation)
        self.active_slot = slot_index
        self.direct_phase = ''
        self.direct_control_claim_started = False
        self.pending_fault = ''
        self.state = self.PREPARING_STORE_TRANSFER
        self.fault = ''
        self.last_result = ''
        self.store_transfer_phase = 'preparing'
        self.store_transfer_fallback_used = False
        self.store_transfer_fallback_reason = ''
        self.phase_started = time.monotonic()
        self.pending_record = {
            'slot': slot_index, 'slot_flb_m': self.slots[slot_index],
            **geometry, 'target_tcp_pose': target,
        }
        self.placed_ids_before_store = set(
            self.scene_status.get('placed_item_ids') or [])
        pre_place = (target[0], target[1], target[2] + self.clearance, *target[3:])
        target_transfer_z = self._transfer_tcp_z_for_item_bottom(
            target[2], self.slots[slot_index][2], pallet_origin_z,
            self.transfer_item_bottom_above_pallet)
        self.pending_record['transfer_tcp_z'] = target_transfer_z
        self.pending_record['pre_place_tcp_z'] = pre_place[2]
        transfer_quaternion = self._quaternion_from_rpy(*target[3:])
        transfer_target = Float64MultiArray()
        transfer_target.data = [
            float(slot_index), target[0], target[1], target_transfer_z,
            *transfer_quaternion, pre_place[2],
            pallet_origin_z + self.transfer_item_bottom_above_pallet,
        ]
        # Recompute TCP XY for the same slot corner: an off-centre grasp must
        # not simply rotate about the old TCP and move the box outside its slot.
        x, y, z, w = geometry['store_tcp_q']
        flipped_q = (-y, x, w, -z)
        flipped = self._store_target(self.slots[slot_index], {**geometry, 'store_tcp_q': flipped_q})
        self.pending_record['yaw180_target_tcp_pose'] = flipped
        transfer_target.data.extend([*flipped[:2], flipped[2] + self.clearance, *flipped_q])
        self.store_transfer_target_pub.publish(transfer_target)
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self.ik_solutions = []
        self.motion_queue = []
        # Allow the target subscription in motion_coordinator to run before
        # invoking its preparation service.
        self._store_transfer_timer = self.create_timer(
            0.15, self._request_store_transfer_prepare)
        completion_route = (
            'returning to observation'
            if self.return_to_observation else
            'remaining at the raised staging waypoint')
        response.success = True
        response.message = (
            f'storing attached item in slot {slot_index} at FLB '
            f'{tuple(round(v*1000) for v in self.slots[slot_index])} mm; '
            'using validated transfer logic; holding current TCP orientation; '
            f'item bottom={self.transfer_item_bottom_above_pallet*1000:.0f} mm '
            'above pallet; '
            f'{completion_route}'
        )
        self.publish_status()
        return response

    def _request_store_transfer_prepare(self):
        if self._store_transfer_timer is not None:
            self._store_transfer_timer.cancel()
            self._store_transfer_timer = None
        if self.state != self.PREPARING_STORE_TRANSFER:
            return
        future = self.prepare_staging_store_transfer.call_async(
            Trigger.Request())
        future.add_done_callback(self._store_transfer_prepare_completed)

    def _store_transfer_prepare_completed(self, future):
        if self.state != self.PREPARING_STORE_TRANSFER:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'staging transfer preparation failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'staging transfer preparation rejected: {message}')

    def _begin_store_continuous_transfer(self):
        client = self.start_transport if self.return_to_observation else self.start_transport_chained
        if not client.service_is_ready():
            self._fault('shared continuous transport service is unavailable')
            return
        self.state = self.TRANSFERRING_STORE
        self.store_transfer_phase = 'continuous'
        self.expected_store_transfer_operation_id = int(self.pickup_status.get('operation_id', 0)) + 1
        self.placed_ids_before_store = set(self.scene_status.get('placed_item_ids') or [])
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self._request_accepted)

    def _tick_store_continuous_transfer(self):
        if int(self.pickup_status.get('operation_id', -1)) != self.expected_store_transfer_operation_id:
            return
        if self.pickup_status.get('transport_slot_yaw_flipped') and self.pending_record:
            self.pending_record['target_tcp_pose'] = self.pending_record['yaw180_target_tcp_pose']
            self.pending_record['pre_place_tcp_z'] = self.pending_record['target_tcp_pose'][2] + self.clearance
        state = self.pickup_status.get('state')
        if state == 'FAULT':
            reason = self.pickup_status.get('fault', 'continuous staging transfer failed')
            # Chained pickups start at contact, not at the old elevated start.
            # A direct fallback to the slot waypoint could sweep the held box
            # through neighbours. Keep it held; do not skip the required lift.
            self._fault(reason)
            return
        if state != 'SUCCEEDED':
            return
        if self.pickup_status.get('place_fallback_used'):
            self._complete_safe_servo_store()
            return
        if not self.pickup_status.get('direct_transfer_succeeded'):
            self._fault('staging continuous transfer completion was not verified')
            return
        if int(self.motion_status.get('operation_id', -1)) != self.expected_motion_operation_id:
            self._fault('staging target changed during continuous transfer')
            return
        if self.store_transfer_phase == 'continuous':
            if not self.accept_direct_transfer.service_is_ready():
                self._fault('continuous staging handoff acknowledgement unavailable')
                return
            self.store_transfer_phase = 'continuous_accepting'
            self.store_continuous_ack = False
            expected = self.expected_store_transfer_operation_id
            def accepted(future):
                if self.state != self.TRANSFERRING_STORE or expected != self.expected_store_transfer_operation_id:
                    return
                try:
                    response = future.result()
                    if response is None or not response.success:
                        raise ValueError('continuous staging handoff rejected')
                    self.store_continuous_ack = True
                except Exception as exc:
                    self._fault(str(exc))
            self.accept_direct_transfer.call_async(Trigger.Request()).add_done_callback(accepted)
        elif self.store_continuous_ack and self.motion_status.get('state') == 'SUCCEEDED':
            self._begin_safe_servo_store()

    def _begin_store_direct_transfer(self):
        if not self.start_joint_transfer.service_is_ready():
            self._begin_store_moveit_fallback(
                'deterministic joint-transfer service is unavailable')
            return
        self.state = self.TRANSFERRING_STORE
        self.store_transfer_phase = 'direct_starting'
        self.expected_store_transfer_operation_id = int(
            self.pickup_status.get('operation_id', 0)) + 1
        self.phase_started = time.monotonic()
        future = self.start_joint_transfer.call_async(Trigger.Request())
        future.add_done_callback(self._store_direct_transfer_started)

    def _store_direct_transfer_started(self, future):
        if (self.state != self.TRANSFERRING_STORE or
                self.store_transfer_phase != 'direct_starting'):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._begin_store_moveit_fallback(
                f'deterministic staging transfer start failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._begin_store_moveit_fallback(
                f'deterministic staging transfer rejected: {message}')
            return
        self.store_transfer_phase = 'direct_executing'

    def _begin_store_moveit_fallback(self, reason):
        if not self.plan_transfer.service_is_ready():
            self._fault(f'{reason}; MoveIt transfer service is unavailable')
            return
        self.state = self.TRANSFERRING_STORE
        self.store_transfer_phase = 'moveit_planning'
        self.store_transfer_fallback_used = True
        self.store_transfer_fallback_reason = str(reason)
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self.phase_started = time.monotonic()
        self.get_logger().warning(
            f'{reason}; falling back to collision-aware MoveIt staging transfer')
        future = self.plan_transfer.call_async(Trigger.Request())
        future.add_done_callback(self._request_accepted)

    def _accept_store_direct_transfer(self):
        if not self.accept_direct_transfer.service_is_ready():
            self._fault('direct staging transfer acknowledgement is unavailable')
            return
        self.store_transfer_phase = 'direct_accepting'
        future = self.accept_direct_transfer.call_async(Trigger.Request())
        future.add_done_callback(self._store_direct_transfer_accepted)

    def _store_direct_transfer_accepted(self, future):
        if (self.state != self.TRANSFERRING_STORE or
                self.store_transfer_phase != 'direct_accepting'):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'direct staging transfer acknowledgement failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(
                f'direct staging transfer acknowledgement rejected: {message}')

    def _begin_store_loading(self):
        if not self.start_loading.service_is_ready():
            self._fault('staging vertical loading service is unavailable')
            return
        self._publish_store_place_config()
        self.state = self.LOADING_STORE
        self.store_transfer_phase = 'loading'
        self.expected_store_transfer_operation_id = int(
            self.pickup_status.get('operation_id', 0)) + 1
        self.phase_started = time.monotonic()
        future = self.start_loading.call_async(Trigger.Request())
        future.add_done_callback(self._store_loading_started)

    def _store_loading_started(self, future):
        if self.state != self.LOADING_STORE:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'staging vertical loading start failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'staging vertical loading rejected: {message}')

    def _publish_store_place_config(self):
        if self.pending_record is None:
            return
        target = self.pending_record['target_tcp_pose']
        config = Float64MultiArray()
        config.data = [
            float(self.active_slot),
            float(target[2]),
            float(self.pending_record['transfer_tcp_z']),
        ]
        self.place_config_pub.publish(config)

    def _disable_servo_before_direct(self):
        if not self.enable_servo.service_is_ready():
            self._deactivate_controllers()
            return
        request = SetBool.Request()
        request.data = False
        future = self.enable_servo.call_async(request)
        future.add_done_callback(self._servo_disabled)

    def _servo_disabled(self, future):
        if self.state != self.PREPARING_DIRECT:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'safe-servo disable failed before staging: {exc}')
            return
        if result is None or not result.success:
            self._fault('safe-servo did not confirm disable before staging')
            return
        self._deactivate_controllers()

    def retrieve_callback(self, _request, response):
        return self._retrieve(response, return_to_observation=True)

    def retrieve_chained_callback(self, _request, response):
        return self._retrieve(response, return_to_observation=False)

    def _retrieve(self, response, return_to_observation):
        reason = self._busy_reason(allow_contact=True)
        if reason:
            response.message = reason
            return response
        if self.scene_status.get('attached_item_id'):
            response.message = 'cannot retrieve: the robot already carries an item'
            return response
        slot_index = self.selected_retrieve_slot
        record = self.occupied.get(slot_index)
        if record is None:
            response.message = f'staging slot {slot_index} is empty'
            return response
        if (not record.get('obstacle_removed') and
                not self.remove_staged_obstacle.service_is_ready()):
            response.message = 'staged-obstacle removal service is unavailable'
            return response
        placed_id = str(record.get('placed_obstacle_id') or '')
        match = re.fullmatch(r'placed_item_(\d+)', placed_id)
        if not match:
            response.message = (
                f'slot {slot_index} has no valid planning-scene obstacle ID')
            return response
        self.operation = 'retrieve'
        self.return_to_observation = return_to_observation
        self.active_slot = slot_index
        self.state = self.PREPARING_RETRIEVAL
        self.fault = ''
        self.last_result = ''
        self.phase_started = time.monotonic()
        self.sam_retrieval_result = None
        if getattr(self, 'sam_inspection_enabled', False):
            try:
                self.sam_inspector.begin(marker_target(self.scene_status, placed_id))
                self.state = 'SAM_INSPECTION'
            except ValueError as exc:
                self._fault(str(exc))
                response.message = str(exc)
                return response
        elif getattr(self, 'inspection_enabled', False):
            self._begin_slot_inspection()
        else:
            self._begin_retrieval_removal()
        response.success = True
        response.message = f'retrieval requested for staging slot {slot_index}'
        self.publish_status()
        return response

    def _begin_retrieval_removal(self):
        record = self.occupied[self.active_slot]
        self.state = self.PREPARING_RETRIEVAL
        if record.get('obstacle_removed'):
            self._publish_retrieval_target()
        else:
            self.waiting_removed_id = record['placed_obstacle_id']
            request = SetInt16.Request()
            request.data = int(re.fullmatch(r'placed_item_(\d+)', self.waiting_removed_id).group(1))
            future = self.remove_staged_obstacle.call_async(request)
            future.add_done_callback(self._staged_obstacle_removed)

    def _staged_obstacle_removed(self, future):
        if self.state != self.PREPARING_RETRIEVAL:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'failed to remove staged collision object: {exc}')
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._fault(f'staged collision-object removal rejected: ret={code}')
            return
        # The scene service only acknowledges that ApplyPlanningScene was
        # accepted. Wait for its status to confirm removal before asking MoveIt
        # to plan through the former target-object volume.
        self.occupied[self.active_slot]['obstacle_removed'] = True

    def _publish_retrieval_target(self, *, overhead=True):
        record = self.occupied[self.active_slot]
        pose = record['release_tcp_pose']
        contact_reference_z = self._retrieval_contact_reference_z(record)
        try:
            approach_z = max(self._pallet_origin_z() + self.transfer_item_bottom_above_pallet,
                              contact_reference_z + self.clearance + .02)
        except ValueError as exc:
            self._fault(str(exc))
            return
        target = Float64MultiArray()
        target.data = [
            float(self.active_slot),
            pose[0] / 1000.0, pose[1] / 1000.0, contact_reference_z,
            pose[3], pose[4], pose[5],
            *record['size'], self.clearance, float(record.get('object_yaw', 0.0)), approach_z, 1.0,
        ]
        refined = getattr(self, 'sam_retrieval_result', None)
        if getattr(self, 'sam_inspection_enabled', False):
            if refined is None:
                self._fault('validated SAM slot inspection is required before pickup')
                return
            top = refined['top_center_base_m']
            approach_z = max(approach_z, top[2] + self.clearance + .02)
            target.data = [float(self.active_slot), *top, *refined['grasp_rpy_rad'],
                           *refined['size_m'], self.clearance, refined['yaw_rad'], approach_z, 1.0]
        elif 'center' in record and 'orientation' in record:
            target.data.extend([*record['center'], *record['orientation']])
        self.retrieve_target_pub.publish(target)
        self.state = self.PLANNING_RETRIEVAL_APPROACH if overhead else self.PLANNING_RETRIEVAL
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        if overhead:
            self.pick_path.reset(self.expected_motion_operation_id)
        # Let the target subscription run before invoking the planning service.
        timer = self.create_timer(0.15, self._request_retrieval_plan)
        self._one_shot_timer = timer

    @staticmethod
    def _retrieval_contact_reference_z(record):
        pose = record['release_tcp_pose']
        predicted_pose = record.get('target_tcp_pose')
        measured_release_z = float(pose[2]) / 1000.0
        predicted_top_tcp_z = (
            float(predicted_pose[2])
            if isinstance(predicted_pose, (list, tuple)) and
            len(predicted_pose) >= 3 else measured_release_z)
        # Contact compliance or calibration can put the measured release TCP
        # below the geometry-predicted item top. The higher reference ensures
        # the subsequent clearance remains real.
        return max(measured_release_z, predicted_top_tcp_z)

    def _request_retrieval_plan(self):
        self._one_shot_timer.cancel()
        if self.state not in (self.PLANNING_RETRIEVAL, self.PLANNING_RETRIEVAL_APPROACH,
                              self.INSPECTION_MOVE):
            return
        client = (self.pick_path.prepare if self.state in (self.PLANNING_RETRIEVAL_APPROACH, self.INSPECTION_MOVE)
                  else self.plan_staging_pregrasp)
        if not client.service_is_ready():
            self._fault('staging pre-pick planning service is unavailable')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self._request_accepted)

    def _request_accepted(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'staging request failed: {exc}')
            return
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self._fault(f'staging request rejected: {message}')

    def _fresh_joint_seed(self):
        if (self.latest_joint_state is None or
                self.last_joint_state_time is None or
                time.monotonic() - self.last_joint_state_time > self.status_timeout):
            raise ValueError('fresh arm joint state is unavailable for staging IK')
        return self.latest_joint_state

    def _solve_next_store_ik(self, seed):
        if self.state != self.SOLVING_STORE_IK:
            return
        if not self.ik_targets:
            self.motion_queue = list(self.ik_solutions)
            self.state = self.PREPARING_DIRECT
            self.direct_control_claim_started = True
            self._disable_servo_before_direct()
            return
        target = self.ik_targets.pop(0)
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.planning_group
        ik.robot_state.joint_state.name = list(self.arm_joint_names)
        ik.robot_state.joint_state.position = list(map(float, seed))
        ik.robot_state.is_diff = True
        ik.avoid_collisions = True
        ik.ik_link_name = self.ik_link_name
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = 'link_base'
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik.pose_stamped.pose.position.x = target[0]
        ik.pose_stamped.pose.position.y = target[1]
        ik.pose_stamped.pose.position.z = target[2]
        quaternion = self._quaternion_from_rpy(*target[3:])
        (ik.pose_stamped.pose.orientation.x,
         ik.pose_stamped.pose.orientation.y,
         ik.pose_stamped.pose.orientation.z,
         ik.pose_stamped.pose.orientation.w) = quaternion
        seconds = max(0.001, self.ik_timeout)
        ik.timeout.sec = int(seconds)
        ik.timeout.nanosec = int((seconds - int(seconds)) * 1e9)
        future = self.compute_ik.call_async(request)
        future.add_done_callback(self._store_ik_completed)

    def _store_ik_completed(self, future):
        if self.state != self.SOLVING_STORE_IK:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'staging IK request failed: {exc}')
            return
        if result is None or result.error_code.val != 1:
            code = None if result is None else result.error_code.val
            message = '' if result is None else result.error_code.message
            suffix = f' ({message})' if message else ''
            self._fault(f'no collision-free staging IK solution: code={code}{suffix}')
            return
        solution = {
            str(name): float(position)
            for name, position in zip(
                result.solution.joint_state.name,
                result.solution.joint_state.position)
        }
        if not all(name in solution for name in self.arm_joint_names):
            self._fault('staging IK solution omitted one or more arm joints')
            return
        angles = tuple(solution[name] for name in self.arm_joint_names)
        if not all(math.isfinite(value) for value in angles):
            self._fault('staging IK solution contains non-finite joint values')
            return
        self.ik_solutions.append(angles)
        # Seed the next waypoint with this solution to keep both joint targets
        # on the same nearby IK branch.
        self._solve_next_store_ik(angles)

    def _deactivate_controllers(self):
        if not self.list_controllers.service_is_ready():
            self._fault('controller_manager list service is unavailable')
            return
        future = self.list_controllers.call_async(ListControllers.Request())
        future.add_done_callback(self._controllers_received)

    def _controllers_received(self, future):
        if self.state != self.PREPARING_DIRECT:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'failed to inspect controllers: {exc}')
            return
        states = {item.name: item.state for item in result.controller}
        required = (self.trajectory_controller, self.joint_state_broadcaster)
        active = [name for name in required if states.get(name) == 'active']
        if not active:
            self._deactivate_hardware()
            return
        if not self.switch_controller.service_is_ready():
            self._fault('controller switch service is unavailable')
            return
        request = SwitchController.Request()
        request.deactivate_controllers = active
        request.activate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(seconds=3.0).to_msg()
        future = self.switch_controller.call_async(request)
        future.add_done_callback(self._controllers_deactivated)

    def _controllers_deactivated(self, future):
        if getattr(self, 'abort_latched', False):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'controller deactivation failed: {exc}')
            return
        if result is None or not result.ok:
            self._fault('ROS controllers did not deactivate for staging')
            return
        self._deactivate_hardware()

    def _deactivate_hardware(self):
        if getattr(self, 'abort_latched', False):
            return
        if not self.set_hardware.service_is_ready():
            self._fault('hardware lifecycle service is unavailable')
            return
        request = SetHardwareComponentState.Request()
        request.name = self.hardware_component
        request.target_state.id = State.PRIMARY_STATE_INACTIVE
        request.target_state.label = 'inactive'
        future = self.set_hardware.call_async(request)
        future.add_done_callback(self._hardware_deactivated)

    def _hardware_deactivated(self, future):
        if getattr(self, 'abort_latched', False):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'hardware deactivation failed: {exc}')
            return
        if (result is None or not result.ok or
                result.state.id != State.PRIMARY_STATE_INACTIVE):
            self._fault('robot hardware did not enter inactive state')
            return
        self._begin_mode_wait(0, 'staging direct joint motion', self._send_next_joint)

    def _begin_mode_wait(self, target, label, callback):
        if getattr(self, 'abort_latched', False):
            return
        self.mode_wait_target = int(target)
        self.mode_wait_label = label
        self.mode_wait_callback = callback
        self.mode_wait_deadline = time.monotonic() + self.mode_transition_timeout
        self.mode_wait_last_command = None
        self.mode_wait_command_pending = False
        self.mode_wait_ready_count = 0

    def _mode_tick(self):
        if self.mode_wait_target is None:
            return
        now = time.monotonic()
        if now >= self.mode_wait_deadline:
            label = self.mode_wait_label
            self._clear_mode_wait()
            self._fault(f'timed out confirming robot mode for {label}')
            return
        fresh = (
            self.robot_state_time is not None and
            now - self.robot_state_time <= self.status_timeout)
        if fresh and self.robot_error not in (None, 0):
            if self.robot_error == 52 and self.clean_error.service_is_ready():
                if (not self.mode_wait_command_pending and
                        (self.mode_wait_last_command is None or
                         now - self.mode_wait_last_command >= self.mode_retry_interval)):
                    self.mode_wait_command_pending = True
                    self.mode_wait_last_command = now
                    future = self.clean_error.call_async(Call.Request())
                    future.add_done_callback(self._mode_error_cleared)
                return
            self._fault(f'xArm error {self.robot_error} during {self.mode_wait_label}')
            return
        ready = (
            fresh and self.robot_error == 0 and
            self.robot_mode == self.mode_wait_target and
            self.robot_state is not None and self.robot_state <= 2)
        if ready:
            self.mode_wait_ready_count += 1
            if self.mode_wait_ready_count >= self.mode_ready_samples:
                callback = self.mode_wait_callback
                self._clear_mode_wait()
                callback()
            return
        self.mode_wait_ready_count = 0
        if (not self.mode_wait_command_pending and
                (self.mode_wait_last_command is None or
                 now - self.mode_wait_last_command >= self.mode_retry_interval)):
            self._send_mode_command(now)

    def _clear_mode_wait(self):
        self.mode_wait_target = None
        self.mode_wait_label = ''
        self.mode_wait_callback = None
        self.mode_wait_deadline = None
        self.mode_wait_last_command = None
        self.mode_wait_command_pending = False
        self.mode_wait_ready_count = 0

    def _mode_error_cleared(self, _future):
        self.mode_wait_command_pending = False

    def _send_mode_command(self, now):
        if not self.set_mode.service_is_ready() or not self.set_state.service_is_ready():
            self._fault('xArm mode/state services are unavailable')
            return
        self.mode_wait_command_pending = True
        self.mode_wait_last_command = now
        request = SetInt16.Request()
        request.data = self.mode_wait_target
        future = self.set_mode.call_async(request)
        future.add_done_callback(self._mode_set)

    def _mode_set(self, future):
        if getattr(self, 'abort_latched', False):
            return
        try:
            result = future.result()
        except Exception:
            self.mode_wait_command_pending = False
            return
        if result is None or result.ret != 0:
            self.mode_wait_command_pending = False
            return
        request = SetInt16.Request()
        request.data = 0
        future = self.set_state.call_async(request)
        future.add_done_callback(self._state_set)

    def _state_set(self, _future):
        self.mode_wait_command_pending = False

    def _send_next_joint(self):
        if getattr(self, 'abort_latched', False):
            return
        if not self.motion_queue:
            if self.direct_phase == 'approach_to_servo':
                self._restore_control()
            else:
                self._fault('invalid staging direct-motion phase')
            return
        angles = self.motion_queue.pop(0)
        self._send_joint_target(angles)

    def _send_joint_target(self, angles):
        if getattr(self, 'abort_latched', False):
            return
        if not self.move_joint.service_is_ready():
            self._fault('ufactory set_servo_angle service is unavailable')
            return
        request = MoveJoint.Request()
        request.angles = list(map(float, angles))
        request.speed = self.joint_speed
        request.acc = self.joint_acc
        request.mvtime = 0.0
        request.wait = True
        request.timeout = self.motion_timeout
        request.radius = -1.0
        request.relative = False
        self.state = self.MOVING_DIRECT
        self.motion_pending = True
        future = self.move_joint.call_async(request)
        future.add_done_callback(self._joint_target_completed)

    def _joint_target_completed(self, future):
        self.motion_pending = False
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'staging joint motion failed: {exc}')
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._fault(f'staging joint motion rejected: ret={code}')
            return
        self._send_next_joint()

    def _restore_control(self):
        if getattr(self, 'abort_latched', False):
            return
        self.state = self.RESTORING_CONTROL
        if not self.list_hardware.service_is_ready():
            self._fault('hardware-list service is unavailable during restore')
            return
        future = self.list_hardware.call_async(ListHardwareComponents.Request())
        future.add_done_callback(self._restore_hardware_received)

    def _restore_hardware_received(self, future):
        if getattr(self, 'abort_latched', False):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'failed to inspect hardware during restore: {exc}')
            return
        components = {item.name: item for item in result.component}
        component = components.get(self.hardware_component)
        if component is None:
            self._fault(f'hardware component not found: {self.hardware_component}')
            return
        if component.state.id in (
                State.PRIMARY_STATE_ACTIVE,
                State.PRIMARY_STATE_UNCONFIGURED):
            request = SetHardwareComponentState.Request()
            request.name = self.hardware_component
            request.target_state.id = State.PRIMARY_STATE_INACTIVE
            request.target_state.label = 'inactive'
            future = self.set_hardware.call_async(request)
            future.add_done_callback(self._restore_hardware_inactive)
            return
        if component.state.id == State.PRIMARY_STATE_INACTIVE:
            self._begin_mode_wait(
                self.ros2_control_mode, 'staging control restore',
                self._activate_hardware)
            return
        self._final_fault(
            'unsupported hardware lifecycle state during staging restore: '
            f'{component.state.label}')

    def _restore_hardware_inactive(self, future):
        if getattr(self, 'abort_latched', False):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._final_fault(f'hardware configuration/deactivation failed: {exc}')
            return
        if (result is None or not result.ok or
                result.state.id != State.PRIMARY_STATE_INACTIVE):
            state = None if result is None else result.state.label
            self._final_fault(
                f'hardware did not reach inactive during restore: {state}')
            return
        self._begin_mode_wait(
            self.ros2_control_mode, 'staging control restore',
            self._activate_hardware)

    def _activate_hardware(self):
        if getattr(self, 'abort_latched', False):
            return
        request = SetHardwareComponentState.Request()
        request.name = self.hardware_component
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        request.target_state.label = 'active'
        future = self.set_hardware.call_async(request)
        future.add_done_callback(self._hardware_activated)

    def _hardware_activated(self, future):
        if getattr(self, 'abort_latched', False):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'hardware activation failed: {exc}')
            return
        if result is None or not result.ok:
            self._fault('hardware did not reactivate after staging')
            return
        future = self.list_controllers.call_async(ListControllers.Request())
        future.add_done_callback(self._restore_controllers_received)

    def _restore_controllers_received(self, future):
        if getattr(self, 'abort_latched', False):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'failed to inspect controllers during restore: {exc}')
            return
        states = {item.name: item.state for item in result.controller}
        required = (self.joint_state_broadcaster, self.trajectory_controller)
        missing = [name for name in required if states.get(name) != 'active']
        if not missing:
            self._wait_for_joint_states()
            return
        request = SwitchController.Request()
        request.activate_controllers = missing
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout = Duration(seconds=3.0).to_msg()
        future = self.switch_controller.call_async(request)
        future.add_done_callback(self._controllers_restored)

    def _controllers_restored(self, future):
        if getattr(self, 'abort_latched', False):
            return
        try:
            result = future.result()
        except Exception as exc:
            self._fault(f'controller restore failed: {exc}')
            return
        if result is None or not result.ok:
            self._fault('trajectory and state controllers did not reactivate')
            return
        self._wait_for_joint_states()

    def _wait_for_joint_states(self):
        if getattr(self, 'abort_latched', False):
            return
        self.restore_wait_sequence = self.joint_state_sequence
        self.restore_wait_deadline = time.monotonic() + self.joint_state_ready_timeout

    def _restore_tick(self):
        if getattr(self, 'abort_latched', False):
            return
        if self.restore_wait_sequence is None:
            return
        if (self.joint_state_sequence >=
                self.restore_wait_sequence + self.joint_state_ready_samples and
                self.last_joint_state_time is not None and
                time.monotonic() - self.last_joint_state_time <= 0.25):
            self.restore_wait_sequence = None
            self.restore_wait_deadline = None
            self.direct_control_claim_started = False
            if self.pending_fault:
                reason = self.pending_fault
                self.pending_fault = ''
                self._final_fault(reason)
            elif (self.operation == 'store' and
                  self.direct_phase == 'approach_to_servo'):
                self._begin_safe_servo_store()
            else:
                self._begin_observation()
        elif time.monotonic() >= self.restore_wait_deadline:
            self.restore_wait_sequence = None
            self.restore_wait_deadline = None
            self._fault('joint states did not resume after staging control restore')

    def _begin_safe_servo_store(self):
        if self.pending_record is None:
            self._fault('staging placement record is unavailable')
            return
        if not self.start_staging_place.service_is_ready():
            self._fault('staging safe-servo service is unavailable')
            return
        self.state = self.SETTLING_STORE
        self.direct_phase = ''
        self.retrieval_settle_started = time.monotonic()
        self.retrieval_settle_after_sequence = self.servo_status_sequence
        self.retrieval_last_checked_sequence = self.servo_status_sequence
        self.retrieval_converged_samples = 0
        self.publish_status()

    def _arm_safe_servo_store(self):
        self._publish_store_place_config()
        self.expected_store_place_operation_id = int(
            self.pickup_status.get('operation_id', 0)) + 1
        self.state = self.PLACING_STORE
        self._staging_place_timer = self.create_timer(
            0.15, self._request_safe_servo_store)
        self.publish_status()

    def _request_safe_servo_store(self):
        if self._staging_place_timer is not None:
            self._staging_place_timer.cancel()
            self._staging_place_timer = None
        if self.state != self.PLACING_STORE:
            return
        future = self.start_staging_place.call_async(Trigger.Request())
        future.add_done_callback(self._request_accepted)

    @staticmethod
    def _placed_item_sort_key(item_id):
        match = re.fullmatch(r'placed_item_(\d+)', str(item_id))
        return int(match.group(1)) if match else -1

    def _record_confirmed_store_release(self):
        """Commit inventory on release/detach, independently of retreat success."""
        if self.pending_record is None:
            return False
        expected = (self.expected_store_place_operation_id or
                    self.expected_store_transfer_operation_id)
        if expected is None or self.pickup_status.get('operation_id') != expected:
            return False
        release = self.pickup_status.get('release_tcp_pose_mm_rad')
        if (not isinstance(release, (list, tuple)) or len(release) != 6 or
                not all(isinstance(v, (int, float)) and math.isfinite(v) for v in release) or
                self.scene_status.get('attached_item_id')):
            return False
        new_ids = set(self.scene_status.get('placed_item_ids') or []) - (
            self.placed_ids_before_store)
        # A sampled release TCP alone is not proof of release: it is captured
        # before the vacuum command. Require the new detached scene item too.
        if len(new_ids) != 1:
            return False
        placed_id = next(iter(new_ids))
        self.pending_record['release_tcp_pose'] = tuple(map(float, release))
        object_q = self._quat_multiply(
            self._quaternion_from_rpy(*release[3:]), self.pending_record['orientation'])
        self.pending_record['object_yaw'] = self._rpy_from_quaternion(object_q)[2]
        self.pending_record['object_orientation_xyzw'] = object_q
        self.pending_record['placement_result'] = str(
            self.pickup_status.get('place_fallback_reason') or
            'safe-servo force contact')
        self.pending_record['placed_obstacle_id'] = str(placed_id)
        slot = int(self.pending_record['slot'])
        self.occupied[slot] = dict(self.pending_record)
        return True

    def _complete_safe_servo_store(self):
        if not self._record_confirmed_store_release():
            # Scene acknowledgement can arrive after supervisor completion.
            return
        self.pending_record = None
        self.expected_store_place_operation_id = None
        if not self.return_to_observation and not (
                self.pickup_status.get('continuous_return_completed') and
                self.pickup_status.get('return_to_observation') is False):
            # Preserve the released item's record, but do not authorize the
            # next operation until its overhead handoff has been confirmed.
            self._fault('chained staging placement did not confirm its overhead handoff')
            return
        if (self.return_to_observation and not (
                self.pickup_status.get('continuous_return_completed') and
                self.pickup_status.get('return_to_observation', True))):
            self._begin_observation()
        else:
            self._finish_operation()

    def _finish_operation(self):
        self.state = self.SUCCEEDED
        self.last_result = (
            f'{self.operation} completed for slot {self.active_slot}')
        self.publish_status()

    def _begin_observation(self):
        if not self.plan_observation.service_is_ready():
            self._fault('observation planning service is unavailable')
            return
        self.state = self.PLANNING_OBSERVATION
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        future = self.plan_observation.call_async(Trigger.Request())
        future.add_done_callback(self._request_accepted)

    def _execute_latest_motion(self, next_state):
        if not self.execute_motion.service_is_ready():
            self._fault('motion execution service is unavailable')
            return
        self.state = next_state
        future = self.execute_motion.call_async(Trigger.Request())
        future.add_done_callback(self._request_accepted)

    def tick(self):
        if self.state == 'SAM_INSPECTION':
            try:
                result = self.sam_inspector.tick()
                if result is not None:
                    self.sam_retrieval_result = result
                    self._begin_retrieval_removal()
            except Exception as exc:
                self._fault(f'SAM slot inspection failed: {exc}')
            return
        if self.operation == 'store' and self.state in (
                self.TRANSFERRING_STORE, self.PLACING_STORE, self.FAULT):
            self._record_confirmed_store_release()
        self._mode_tick()
        self._restore_tick()
        self._inspection_tick()
        if self.state in self.ACTIVE and self.phase_started is not None and (
                time.monotonic() - self.phase_started > self.motion_timeout * 3.0):
            self._fault(f'staging operation timed out in {self.state}')
            return
        if self.state == self.MOVING_DIRECT and not self.motion_pending:
            return
        if self.state == self.PREPARING_STORE_TRANSFER:
            if (self._motion_matches() and
                    self.motion_status.get('target') == 'transfer' and
                    self.motion_status.get('state') == 'PREPARED'):
                self._begin_store_continuous_transfer()
        elif self.state == self.TRANSFERRING_STORE:
            self._tick_store_transfer()
        elif self.state == self.LOADING_STORE:
            self._tick_store_loading()
        elif self.state == self.SETTLING_STORE:
            self._settle_store()
        elif self.state == self.PLACING_STORE:
            if int(self.pickup_status.get('operation_id', -1)) != int(
                    self.expected_store_place_operation_id or 0):
                return
            place_state = self.pickup_status.get('state')
            if place_state == 'FAULT':
                self._fault(self.pickup_status.get(
                    'fault', 'staging safe-servo placement failed'))
            elif place_state == 'SUCCEEDED':
                self._complete_safe_servo_store()
        elif self.state == self.PREPARING_RETRIEVAL:
            if (self.waiting_removed_id and self.waiting_removed_id not in
                    set(self.scene_status.get('placed_item_ids') or [])):
                self.waiting_removed_id = ''
                self._publish_retrieval_target()
        elif self.state == self.PLANNING_RETRIEVAL_APPROACH:
            if self.pick_path.tick(self.motion_status, self.pickup_status):
                self.state = self.SETTLING_RETRIEVAL
                self.retrieval_settle_started = time.monotonic()
                self.retrieval_settle_after_sequence = self.servo_status_sequence
                self.retrieval_last_checked_sequence = self.servo_status_sequence
                self.retrieval_converged_samples = 0
        elif self.state == self.PLANNING_RETRIEVAL:
            if not self._motion_matches():
                return
            state = self.motion_status.get('state')
            if state == 'FAULT':
                self._fault(self.motion_status.get('fault', 'pre-pick planning failed'))
            elif state == 'PLANNED':
                self._execute_latest_motion(self.EXECUTING_RETRIEVAL)
        elif self.state == self.EXECUTING_RETRIEVAL:
            if self._motion_matches() and self.motion_status.get('state') == 'SUCCEEDED':
                self.state = self.SETTLING_RETRIEVAL
                self.retrieval_settle_started = time.monotonic()
                self.retrieval_settle_after_sequence = self.servo_status_sequence
                self.retrieval_last_checked_sequence = self.servo_status_sequence
                self.retrieval_converged_samples = 0
        elif self.state == self.SETTLING_RETRIEVAL:
            self._settle_retrieval()
        elif self.state == self.PICKING_RETRIEVAL:
            if int(self.pickup_status.get('operation_id', -1)) < int(
                    self.expected_pickup_operation_id or 0):
                return
            pickup_state = self.pickup_status.get('state')
            if pickup_state == 'FAULT':
                self._fault(self.pickup_status.get('fault', 'slot pickup failed'))
            elif pickup_state == 'SUCCEEDED':
                if not self.scene_status.get('attached_item_id'):
                    return  # Grasp success must be accompanied by attachment.
                slot = self.active_slot
                self.occupied.pop(slot, None)
                if getattr(self, 'return_to_observation', True):
                    self._begin_observation()
                else:
                    self._finish_operation()
        elif self.state == self.PLANNING_OBSERVATION:
            if not self._motion_matches():
                return
            state = self.motion_status.get('state')
            if state == 'FAULT':
                self._fault(self.motion_status.get('fault', 'observation planning failed'))
            elif state == 'PLANNED':
                self._execute_latest_motion(self.EXECUTING_OBSERVATION)
        elif self.state == self.EXECUTING_OBSERVATION:
            if self._motion_matches() and self.motion_status.get('state') == 'SUCCEEDED':
                self._finish_operation()

    def _tick_store_transfer(self):
        phase = self.store_transfer_phase
        if phase in ('continuous', 'continuous_accepting'):
            self._tick_store_continuous_transfer()
            return
        if phase in ('direct_starting', 'direct_executing'):
            if int(self.pickup_status.get('operation_id', -1)) < int(
                    self.expected_store_transfer_operation_id or 0):
                return
            if self.pickup_status.get('operation_kind') != 'transfer':
                return
            state = self.pickup_status.get('state')
            if state == 'FAULT':
                reason = self.pickup_status.get(
                    'fault', 'deterministic staging transfer failed')
                if self.pickup_status.get(
                        'direct_transfer_motion_started', False):
                    self._fault(
                        f'{reason}; direct staging motion had already started, '
                        'so automatic MoveIt fallback is unsafe')
                else:
                    self._begin_store_moveit_fallback(reason)
            elif (state == 'SUCCEEDED' and
                  self.pickup_status.get('direct_transfer_succeeded')):
                self._accept_store_direct_transfer()
            return
        if phase == 'direct_accepting':
            if (self._motion_matches() and
                    self.motion_status.get('target') == 'transfer' and
                    self.motion_status.get('state') == 'SUCCEEDED'):
                self._begin_store_loading()
            return
        if not self._motion_matches() or self.motion_status.get(
                'target') != 'transfer':
            return
        state = self.motion_status.get('state')
        if state == 'FAULT':
            self._fault(self.motion_status.get(
                'fault', 'MoveIt staging transfer failed'))
        elif state == 'PLANNED' and phase == 'moveit_planning':
            self.store_transfer_phase = 'moveit_executing'
            self._execute_latest_motion(self.TRANSFERRING_STORE)
        elif state == 'SUCCEEDED' and phase == 'moveit_executing':
            self._begin_store_loading()

    def _tick_store_loading(self):
        if int(self.pickup_status.get('operation_id', -1)) < int(
                self.expected_store_transfer_operation_id or 0):
            return
        if self.pickup_status.get('operation_kind') != 'loading':
            return
        state = self.pickup_status.get('state')
        if state == 'FAULT':
            self._fault(self.pickup_status.get(
                'fault', 'staging vertical loading failed'))
        elif state == 'SUCCEEDED':
            if self.pickup_status.get('place_fallback_used'):
                self._fault(
                    'staging vertical loading released the item through its '
                    'contact fallback: ' + str(self.pickup_status.get(
                        'place_fallback_reason', 'unknown contact')))
                return
            self._begin_safe_servo_store()

    def _motion_matches(self):
        return int(self.motion_status.get('operation_id', -1)) >= int(
            self.expected_motion_operation_id or 0)

    def _fresh_servo_tcp(self):
        if (self.servo_status_time is None or
                time.monotonic() - self.servo_status_time > self.status_timeout):
            return None
        try:
            tcp_xyz = tuple(float(self.servo_status[key]) for key in (
                'tcp_x_m', 'tcp_y_m', 'tcp_z_m'))
            joint_age = float(self.servo_status['joint_state_age_sec'])
        except (KeyError, TypeError, ValueError):
            return None
        if (not all(math.isfinite(value) for value in tcp_xyz) or
                not math.isfinite(joint_age) or joint_age > self.status_timeout):
            return None
        return tcp_xyz

    def _settle_store(self):
        if self.pending_record is None:
            self._fault('staging placement record is unavailable')
            return
        target = self.pending_record['target_tcp_pose']
        snapshot = {
            'x_m': target[0],
            'y_m': target[1],
            'pregrasp_z_m': target[2] + self.clearance,
        }
        if self._settle_tcp(snapshot, 'staging pre-place'):
            self._arm_safe_servo_store()

    def _settle_tcp(self, snapshot, label):
        elapsed = time.monotonic() - float(self.retrieval_settle_started or 0.0)
        if elapsed > self.retrieval_settle_timeout:
            self._fault(
                f'{label} TCP did not settle before safe-servo: '
                f'{self._retrieval_alignment_detail(snapshot)}')
            return False
        if (self.servo_status_sequence <= self.retrieval_settle_after_sequence or
                self.servo_status_sequence == self.retrieval_last_checked_sequence):
            return False
        self.retrieval_last_checked_sequence = self.servo_status_sequence
        tcp_xyz = self._fresh_servo_tcp()
        if tcp_xyz is None:
            self.retrieval_converged_samples = 0
            return False
        x_error, y_error, z_error = self._pregrasp_pose_errors(
            tcp_xyz, snapshot)
        if (abs(x_error) <= self.retrieval_xy_tolerance and
                abs(y_error) <= self.retrieval_xy_tolerance and
                abs(z_error) <= self.retrieval_z_tolerance):
            self.retrieval_converged_samples += 1
        else:
            self.retrieval_converged_samples = 0
        return (
            elapsed >= self.retrieval_settle and
            self.retrieval_converged_samples >= self.retrieval_settle_samples)

    def _settle_retrieval(self):
        snapshot = self.motion_status.get('planned_pregrasp')
        if not isinstance(snapshot, dict):
            self._fault('retrieval pre-pick snapshot is unavailable')
            return
        if not self._settle_tcp(snapshot, 'retrieval pre-pick'):
            return
        client = (self.start_pickup if getattr(self, 'return_to_observation', True)
                  else self.start_pickup_hold)
        if not client.service_is_ready():
            self._fault('pickup supervisor service is unavailable')
            return
        self.state = self.PICKING_RETRIEVAL
        self.expected_pickup_operation_id = int(
            self.pickup_status.get('operation_id', 0)) + 1
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self._request_accepted)

    @staticmethod
    def _pregrasp_pose_errors(actual_xyz, snapshot):
        return (
            float(actual_xyz[0]) - float(snapshot['x_m']),
            float(actual_xyz[1]) - float(snapshot['y_m']),
            float(actual_xyz[2]) - float(snapshot['pregrasp_z_m']),
        )

    def _retrieval_alignment_detail(self, snapshot):
        try:
            tcp_xyz = tuple(float(self.servo_status[key]) for key in (
                'tcp_x_m', 'tcp_y_m', 'tcp_z_m'))
        except (KeyError, TypeError, ValueError):
            return 'live link_tcp telemetry is unavailable'
        errors = self._pregrasp_pose_errors(tcp_xyz, snapshot)
        return 'error XYZ=({:+.1f}, {:+.1f}, {:+.1f}) mm'.format(
            *(value * 1000.0 for value in errors))

    def _fault(self, reason):
        if hasattr(self, 'sam_inspector'):
            self.sam_inspector.cancel()
        if getattr(self, 'abort_latched', False):
            return
        if hasattr(self, 'pick_path'):
            self.pick_path.cancel(stop=True)
        if self.state == self.FAULT:
            return
        self.motion_queue = []
        self.motion_pending = False
        self.ik_targets = []
        self.ik_solutions = []
        self.waiting_removed_id = ''
        self.retrieval_settle_started = None
        self.retrieval_converged_samples = 0
        if self._staging_place_timer is not None:
            self._staging_place_timer.cancel()
            self._staging_place_timer = None
        if self._store_transfer_timer is not None:
            self._store_transfer_timer.cancel()
            self._store_transfer_timer = None
        self._clear_mode_wait()
        if (self.direct_control_claim_started and
                self.state != self.RESTORING_CONTROL):
            self.pending_fault = str(reason)
            self.get_logger().error(
                f'{reason}; restoring ROS control before latching fault')
            self._restore_control()
            return
        self._final_fault(reason)

    def _final_fault(self, reason):
        self.fault = str(reason)
        self.state = self.FAULT
        self.get_logger().error(self.fault)
        self.publish_status()

    def reset_callback(self, _request, response):
        if self.state in self.ACTIVE:
            response.message = f'cannot reset while active in {self.state}'
            return response
        self.state = self.IDLE
        self.abort_latched = False
        self.operation = ''
        self.fault = ''
        self.pending_fault = ''
        self.last_result = 'staging fault/state reset; occupied slots retained'
        self.return_to_observation = True
        self.store_transfer_phase = ''
        self.store_transfer_fallback_used = False
        self.store_transfer_fallback_reason = ''
        self.expected_store_transfer_operation_id = None
        self.ik_targets = []
        self.ik_solutions = []
        response.success = True
        response.message = self.last_result
        self.publish_status()
        return response

    def abort_callback(self, _request, response):
        if hasattr(self, 'sam_inspector'):
            self.sam_inspector.cancel()
        if self.state not in self.ACTIVE:
            response.message = 'no active staging operation'
            return response
        # Do not restore/activate controllers automatically after an abort.
        # In particular, no late callback may schedule the next direct move.
        self.abort_latched = True
        self.motion_queue = []
        self.ik_targets = []
        self.ik_solutions = []
        self.motion_pending = False
        self.pick_path.cancel(stop=True)
        for name in ('_one_shot_timer', '_staging_place_timer', '_store_transfer_timer'):
            timer = getattr(self, name, None)
            if timer is not None:
                timer.cancel()
        self._clear_mode_wait()
        self._final_fault('staging aborted; occupied records retained; reconcile physical item')
        for client in (self.abort_supervisor, self.cancel_motion):
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        if self.set_state.service_is_ready():
            self.set_state.call_async(SetInt16.Request(data=3))
        response.success = True
        response.message = 'staging stop requested; no release or automatic retreat'
        return response

    def accept_debug_pick_callback(self, request, response):
        """Commit standalone debug retrieval only after verified attachment."""
        slot = int(request.data)
        snapshot = self.motion_status.get('planned_pregrasp') or {}
        if (self.state not in ('IDLE', 'SUCCEEDED') or slot not in self.occupied or
                self.pickup_status.get('state') != 'SUCCEEDED' or
                snapshot.get('pickup_source') != 'buffer' or
                snapshot.get('retrieval_target_id') != slot or
                not self.scene_status.get('attached_item_id') or
                self.scene_status.get('last_attached_pickup_operation_id') != self.pickup_status.get('operation_id')):
            response.ret, response.message = 1, 'verified debug slot pickup is required'
            return response
        self.occupied.pop(slot)
        self.publish_status()
        response.ret, response.message = 0, 'debug slot pickup committed'
        return response

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state,
            'operation': self.operation,
            'fault': self.fault,
            'last_result': self.last_result,
            'selected_store_slot': self.selected_store_slot,
            'selected_retrieve_slot': self.selected_retrieve_slot,
            'inspection_enabled': getattr(self, 'inspection_enabled', False),
            'top_face_inspection_enabled': getattr(self, 'sam_inspection_enabled', False),
            'sam_request_id': getattr(getattr(self, 'sam_inspector', None), 'token', None),
            'inspection_backoff_mm': getattr(self, 'inspection_backoff', .1)*1000,
            'inspection_progress': (getattr(self, 'inspection_reason', '')
                                    if self.state == self.INSPECTION_DEPTH else ''),
            'transfer_item_bottom_above_pallet_mm': (
                self.transfer_item_bottom_above_pallet * 1000.0),
            'return_to_observation': self.return_to_observation,
            'store_transfer_phase': self.store_transfer_phase,
            'store_transfer_fallback_used': self.store_transfer_fallback_used,
            'store_transfer_fallback_reason': self.store_transfer_fallback_reason,
            'slots': [
                {
                    'slot': index,
                    'flb_mm': [round(value * 1000.0, 3) for value in corner],
                    'occupied': index in self.occupied,
                    'item_id': (
                        self.occupied[index]['item_id']
                        if index in self.occupied else ''),
                    'obstacle_id': (
                        self.occupied[index].get('placed_obstacle_id', '')
                        if index in self.occupied else ''),
                    'size_m': (
                        list(self.occupied[index]['size'])
                        if index in self.occupied else None),
                }
                for index, corner in enumerate(self.slots)
            ],
        }, separators=(',', ':'))
        self.status_pub.publish(message)
        self._publish_markers()

    def _publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, corner in enumerate(self.slots):
            occupied = index in self.occupied
            deck = Marker()
            deck.header.frame_id = 'link_base'
            deck.header.stamp = stamp
            deck.ns = 'staging_slots'
            deck.id = index * 2
            deck.type = Marker.CUBE
            deck.action = Marker.ADD
            deck.pose.position.x = corner[0] + self.slot_size / 2.0
            deck.pose.position.y = corner[1] + self.slot_size / 2.0
            deck.pose.position.z = corner[2] - 0.0025
            deck.pose.orientation.w = 1.0
            deck.scale.x = self.slot_size
            deck.scale.y = self.slot_size
            deck.scale.z = 0.005
            deck.color.r = 0.95 if occupied else 0.10
            deck.color.g = 0.55 if occupied else 0.75
            deck.color.b = 0.10 if occupied else 0.95
            deck.color.a = 0.22
            markers.markers.append(deck)
            label = Marker()
            label.header = deck.header
            label.ns = 'staging_slot_labels'
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = deck.pose.position.x
            label.pose.position.y = deck.pose.position.y
            label.pose.position.z = corner[2] + 0.025
            label.pose.orientation.w = 1.0
            label.scale.z = 0.025
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'S{index} {"OCCUPIED" if occupied else "EMPTY"}'
            markers.markers.append(label)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = StagingSlots()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
