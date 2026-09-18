import json
import math
from pathlib import Path
import re
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotState, RobotTrajectory
from moveit_msgs.srv import GetPositionIK
from omegaconf import OmegaConf
from packing.real_platform_policy_loading import (
    PendingPhysicalOperation,
    RealPlatformPolicyLoader,
)
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from xarm_msgs.srv import SetInt16

from .random_stable_loading_node import RandomStableLoadingNode


class PolicyLoadingNode(RandomStableLoadingNode):
    """Run direct policy loading or transactional MCTS/A* rearrangement."""

    REARRANGEMENT_STATES = {
        'PLAN_READY', 'REARRANGE_WAIT_TARGET', 'REARRANGE_REMOVE_OBSTACLE',
        'REARRANGE_PLAN_APPROACH', 'REARRANGE_EXECUTE_APPROACH',
        'REARRANGE_PLAN_PICK', 'REARRANGE_EXECUTE_PICK',
        'REARRANGE_SERVO_PICK', 'REARRANGE_STORE',
        'REARRANGE_RETRIEVE', 'REARRANGE_PLACE',
        'REARRANGE_INCOMING', 'REARRANGE_RETREAT_INCOMING',
        'SIMULATING',
    }

    NODE_NAME = 'policy_loading'
    API_PREFIX = '/policy_loading'
    DEFAULT_VISUALIZATION_PORT = 8766
    DEFAULT_CLEARANCE_MM = 20
    DEFAULT_SEED = 0
    VISUALIZATION_DIRECTORY = '/tmp/policy_loading_visualization'
    LOADING_LABEL = 'policy loading'

    def __init__(self):
        super().__init__()
        self.declare_parameter('pallet_unpack_approach_height_m', 0.47)
        self.declare_parameter('simulation_enabled', False)
        self.declare_parameter('simulation_fixed_descent_m', 0.030)
        self.declare_parameter('simulation_segment_duration_sec', 0.65)
        self.declare_parameter('simulation_incoming_x_m', 0.35)
        self.declare_parameter('simulation_incoming_y_m', 0.0)
        self.declare_parameter('simulation_support_z_m', 0.0)
        self.pallet_unpack_approach_height = float(
            self.get_parameter('pallet_unpack_approach_height_m').value)
        minimum_approach_height = self.loader.container_size[2] / 1000.0 + 0.02
        if (self.pallet_unpack_approach_height <
                minimum_approach_height - 1e-9):
            raise ValueError(
                'pallet_unpack_approach_height_m must remain at least 20 mm '
                'above the configured container height')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_status = {}
        self.motion_status = {}
        self.supervisor_status = {}
        self.place_status = {}
        self.staging_status = {}
        self.holding_slots = {}
        self.placed_obstacle_ids = {}
        self.active_operation = None
        self.active_slot = None
        self.expected_motion_operation_id = None
        self.expected_supervisor_operation_id = None
        self.expected_place_operation_id = None
        self.staging_seen_active = False
        self.waiting_removed_obstacle = ''
        self.obstacle_wait_started = None
        self.placed_ids_before_operation = set()
        self.pending_obstacle_key = None
        self.rearrangement_auto_execute = False
        self.simulation_enabled = bool(
            self.get_parameter('simulation_enabled').value)
        self.simulation_fixed_descent = float(
            self.get_parameter('simulation_fixed_descent_m').value)
        self.simulation_segment_duration = float(
            self.get_parameter('simulation_segment_duration_sec').value)
        self.simulation_incoming_x = float(
            self.get_parameter('simulation_incoming_x_m').value)
        self.simulation_incoming_y = float(
            self.get_parameter('simulation_incoming_y_m').value)
        self.simulation_support_z = float(
            self.get_parameter('simulation_support_z_m').value)
        if not 0.005 <= self.simulation_fixed_descent <= 0.100:
            raise ValueError(
                'simulation_fixed_descent_m must be within [0.005, 0.100]')
        if self.simulation_segment_duration <= 0.0:
            raise ValueError('simulation_segment_duration_sec must be positive')
        if not all(math.isfinite(value) for value in (
                self.simulation_incoming_x,
                self.simulation_incoming_y,
                self.simulation_support_z)):
            raise ValueError('simulation incoming pose must be finite')
        self.simulation_joint_names = [f'joint{index}' for index in range(1, 7)]
        self.simulation_joint_positions = [0.0, 0.0, -1.57, 0.0, 0.0, 0.0]
        self.simulation_current_positions = None
        self.simulation_seed_initialized = False
        self.simulation_home_positions = None
        self.simulation_waypoints = []
        self.simulation_solutions = []
        self.simulation_ik_index = 0
        self.simulation_started = None
        self.simulation_duration = 0.0
        self.simulation_completion = ''
        self.simulation_label = ''
        self.simulation_incoming = None
        self.simulation_next_item_id = 1
        self.simulation_sample_count = 0

        self.retrieval_target_pub = self.create_publisher(
            Float64MultiArray, '/staging_slots/retrieve_target', 10)
        self.staging_store_selection_pub = self.create_publisher(
            Int32, '/staging_slots/select_store', 10)
        self.staging_retrieve_selection_pub = self.create_publisher(
            Int32, '/staging_slots/select_retrieve', 10)
        self.create_subscription(
            String, '/planning_scene_obstacles/status',
            self._scene_status_callback, 10)
        self.create_subscription(
            String, '/motion_coordinator/status',
            self._motion_status_callback, 10)
        self.create_subscription(
            String, '/pickup_supervisor/status',
            self._supervisor_status_callback, 10)
        self.create_subscription(
            String, '/place_pipeline/status',
            self._place_status_callback, 10)
        self.create_subscription(
            String, '/staging_slots/status',
            self._staging_status_callback, 10)
        self.plan_retrieval_client = self.create_client(
            Trigger, '/motion_coordinator/plan_staging_pregrasp')
        self.plan_retrieval_approach_client = self.create_client(
            Trigger, '/motion_coordinator/plan_staging_approach')
        self.execute_motion_client = self.create_client(
            Trigger, '/motion_coordinator/execute')
        self.start_supervisor_pick_client = self.create_client(
            Trigger, '/pickup_supervisor/start')
        self.start_place_client = self.create_client(
            Trigger, '/place_pipeline/start')
        self.abort_place_client = self.create_client(
            Trigger, '/place_pipeline/abort')
        self.staging_store_client = self.create_client(
            Trigger, '/staging_slots/store_chained')
        self.staging_retrieve_client = self.create_client(
            Trigger, '/staging_slots/retrieve')
        self.remove_placed_obstacle_client = self.create_client(
            SetInt16, '/planning_scene_obstacles/remove_placed_item')
        self.simulation_ik_client = self.create_client(
            GetPositionIK, '/compute_ik')
        self.simulation_display_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path', 10)
        self.create_subscription(
            JointState, '/joint_states', self._simulation_joint_state_callback, 10)
        self.cancel_motion_client = self.create_client(
            Trigger, '/motion_coordinator/cancel')
        self.abort_supervisor_client = self.create_client(
            Trigger, '/pickup_supervisor/abort')
        self.create_service(
            Trigger, '/policy_loading/plan', self.plan_only_callback)
        self.create_service(
            SetBool, '/policy_loading/set_rearrangement',
            self.set_rearrangement_callback)
        self.create_service(
            SetBool, '/policy_loading/set_simulation',
            self.set_simulation_callback)

    def plan_only_callback(self, _request, response):
        """Estimate the item and publish a policy target without grasping it."""
        if self.simulation_enabled:
            if self.state != 'IDLE' or self.loader.pending is not None:
                response.message = (
                    f'policy loading is not idle; current state is {self.state}')
                return response
            try:
                item_id, dimensions = self._sample_cardboard_simulation_item(
                    auto_start=False)
            except (RuntimeError, TypeError, ValueError) as exc:
                response.message = f'cardboard simulation sampling failed: {exc}'
                return response
            self.continuous_run_active = False
            response.success = True
            response.message = (
                f'planning sampled cardboard item {item_id}: '
                f'{dimensions} mm; execution remains stopped')
            return response
        if self.state != 'IDLE' or self.loader.pending is not None:
            response.message = (
                f'policy loading is not idle; current state is {self.state}')
            return response
        if self.pallet_status != 'LOCKED':
            response.message = 'pallet pose is not LOCKED'
            return response
        pipeline_state = self.pipeline_status.get('state')
        if pipeline_state in self.ACTIVE_PIPELINE_STATES:
            response.message = f'PickAndPlace is already active in {pipeline_state}'
            return response
        if not self.object_info_start_client.service_is_ready():
            response.message = 'object-information estimation service is unavailable'
            return response

        self.state = 'LOCALIZING'
        self.fault = ''
        self.last_result = ''
        self.localization_started = time.monotonic()
        self.cycle_auto_start = False
        self.continuous_run_active = False
        self.abort_requested = False
        future = self.object_info_start_client.call_async(Trigger.Request())
        future.add_done_callback(self._localization_start_completed)
        response.success = True
        response.message = (
            'policy target planning started; PickAndPlace will remain stopped')
        self.publish_status()
        return response

    def _build_loader(self, container_size):
        self.declare_parameter(
            'policy_config_path',
            '/opt/neuromeka_bin_packing/configs/real_platform_policy.yaml')
        config_path = Path(
            str(self.get_parameter('policy_config_path').value)).expanduser()
        config = OmegaConf.to_container(
            OmegaConf.load(config_path), resolve=True)
        if not isinstance(config, dict):
            raise TypeError(f'policy config must be a mapping: {config_path}')
        container_size = self._configured_container_size(
            config, fallback=container_size)
        self.rearrangement_enabled = bool(
            config.get('enable_rearrangement', False))
        checkpoint = Path(str(config['checkpoint'])).expanduser()
        if not checkpoint.is_absolute():
            checkpoint = config_path.parent.parent / checkpoint
        self.declare_parameter(
            'checkpoint_path', str(checkpoint))
        self.declare_parameter(
            'policy_device', str(config.get('device', 'cpu')))
        self.declare_parameter(
            'k_placement', int(config.get('k_placement', 80)))
        # The random-loading base class owns this shared parameter declaration.
        # Mirror the policy YAML into that existing DOUBLE parameter instead of
        # declaring it again in this derived node.
        configured_height_tolerance = float(
            config.get('height_tolerance', 0.0))
        result = self.set_parameters([
            rclpy.parameter.Parameter(
                'height_tolerance', value=configured_height_tolerance),
        ])[0]
        if not result.successful:
            raise ValueError(
                'could not apply policy height_tolerance: ' + result.reason)
        self.declare_parameter(
            'remove_inscribed_ems',
            bool(config.get('remove_inscribed_ems', False)))
        self.declare_parameter(
            'ems_mode',
            str(config.get('ems_mode', 'clipped_ems_by_substantial_contact')))
        self.declare_parameter(
            'check_achievable_lps',
            bool(config.get('check_achievable_lps', False)))
        self.declare_parameter(
            'loading_mode', str(config.get('loading_mode', 'vertical_loading')))
        self.declare_parameter(
            'container_tracking', str(config.get('container_tracking', 'full')))
        self.declare_parameter(
            'rearrangement_iterations', int(config.get('iterations', 100)))
        self.declare_parameter('max_unpack', int(config.get('max_unpack', 6)))
        self.declare_parameter(
            'mcts_max_child', int(config.get('mcts_max_child', 3)))
        self.declare_parameter(
            'optimize_sequence', bool(config.get('optimize_sequence', True)))
        self.declare_parameter(
            'astar_time_limit_sec',
            float(config.get('astar_time_limit_sec', 30.0)))
        self.declare_parameter(
            'target_util', float(config.get('target_util', 0.7)))
        return RealPlatformPolicyLoader(
            checkpoint_path=str(self.get_parameter('checkpoint_path').value),
            device=str(self.get_parameter('policy_device').value),
            container_size=container_size,
            clearance_mm=int(self.get_parameter('clearance_mm').value),
            height_tolerance=float(
                self.get_parameter('height_tolerance').value),
            seed=int(self.get_parameter('seed').value),
            k_placement=int(self.get_parameter('k_placement').value),
            remove_inscribed_ems=bool(
                self.get_parameter('remove_inscribed_ems').value),
            ems_mode=str(self.get_parameter('ems_mode').value),
            check_achievable_lps=bool(
                self.get_parameter('check_achievable_lps').value),
            loading_mode=str(self.get_parameter('loading_mode').value),
            container_tracking=str(
                self.get_parameter('container_tracking').value),
            rearrangement_iterations=int(
                self.get_parameter('rearrangement_iterations').value),
            max_unpack=int(self.get_parameter('max_unpack').value),
            mcts_max_child=int(self.get_parameter('mcts_max_child').value),
            optimize_sequence=bool(
                self.get_parameter('optimize_sequence').value),
            astar_time_limit_sec=float(
                self.get_parameter('astar_time_limit_sec').value),
            target_util=float(self.get_parameter('target_util').value),
        )

    @staticmethod
    def _configured_container_size(config, fallback):
        values = config.get('container_size', fallback)
        try:
            result = tuple(int(value) for value in values)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                'container_size must contain three integer millimetre values'
            ) from exc
        if len(result) != 3 or any(value <= 0 for value in result):
            raise ValueError(
                'container_size must contain three positive millimetre values')
        return result

    def _log_ready(self, container_size):
        del container_size
        self.get_logger().info(
            'policy loading ready: container=%s mm, MCTS/A*=%s, '
            'clearance=%d mm, height_tolerance=%.1f mm, checkpoint=%s, '
            'device=%s' % (
                self.loader.container_size,
                self.rearrangement_enabled,
                self.loader.clearance_mm,
                self.loader.height_tolerance,
                self.loader.checkpoint_path,
                getattr(self.loader.agent, 'device', self.loader.device),
            ))

    @staticmethod
    def _decode(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError, json.JSONDecodeError):
            return {}

    @classmethod
    def _json_safe(cls, value):
        if isinstance(value, dict):
            return {str(key): cls._json_safe(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._json_safe(item) for item in value]
        if hasattr(value, 'item'):
            return cls._json_safe(value.item())
        return value

    def _scene_status_callback(self, message):
        self.scene_status = self._decode(message)
        self._update_pending_obstacle_mapping()

    def _motion_status_callback(self, message):
        self.motion_status = self._decode(message)

    def _supervisor_status_callback(self, message):
        self.supervisor_status = self._decode(message)

    def _place_status_callback(self, message):
        self.place_status = self._decode(message)

    def _staging_status_callback(self, message):
        self.staging_status = self._decode(message)

    def set_rearrangement_callback(self, request, response):
        if self.state not in ('IDLE', 'FAULT') or self.loader.rearrangement is not None:
            response.message = (
                f'cannot change rearrangement mode in state {self.state}')
            return response
        self.rearrangement_enabled = bool(request.data)
        response.success = True
        response.message = (
            'MCTS + A* rearrangement enabled'
            if self.rearrangement_enabled else
            'direct learned-policy loading enabled')
        self.publish_status()
        return response

    def set_simulation_callback(self, request, response):
        if self.state not in ('IDLE', 'FAULT'):
            response.message = (
                f'cannot change simulation mode in state {self.state}')
            return response
        self.simulation_enabled = bool(request.data)
        if not self.simulation_enabled:
            self._clear_simulation_motion()
            self.simulation_seed_initialized = False
            self.simulation_home_positions = None
        response.success = True
        response.message = (
            'policy simulation enabled; physical robot commands are blocked'
            if self.simulation_enabled else
            'policy simulation disabled; physical execution restored')
        self.publish_status()
        return response

    def item_result_callback(self, message):
        if len(message.data) >= 11:
            values = tuple(map(float, message.data[:11]))
            if all(math.isfinite(value) for value in values):
                self.simulation_incoming = {
                    'item_id': int(round(values[0])),
                    'center': values[1:4],
                    'quaternion': values[4:8],
                    'size': values[8:11],
                }
        return super().item_result_callback(message)

    def _sample_cardboard_simulation_item(self, *, auto_start):
        if self.loader.pending is not None or self.loader.rearrangement is not None:
            raise RuntimeError('another policy operation is already pending')
        sampler = self.loader.env.buffer.data_sampler
        dimensions = sampler.sample(1)[0]
        dimensions_mm = tuple(map(int, dimensions.raw()))
        item_id = self.simulation_next_item_id
        self.simulation_next_item_id += 1
        self.simulation_sample_count += 1
        self.simulation_incoming = {
            'item_id': item_id,
            'center': (
                self.simulation_incoming_x,
                self.simulation_incoming_y,
                self.simulation_support_z + dimensions_mm[2] / 2000.0,
            ),
            'quaternion': (0.0, 0.0, 0.0, 1.0),
            'size': tuple(value / 1000.0 for value in dimensions_mm),
        }
        self.state = 'PLANNING'
        self.localization_started = None
        self.fault = ''
        self.target_acknowledged = False
        self.cycle_auto_start = bool(auto_start)
        self.abort_requested = False
        self.planning_future = self.planning_worker.submit(
            self._plan_item,
            item_id=item_id,
            dimensions_mm=dimensions_mm,
        )
        self.last_result = (
            f'sampled cardboard item {item_id}: '
            f'{dimensions_mm[0]} x {dimensions_mm[1]} x '
            f'{dimensions_mm[2]} mm')
        self.get_logger().info(self.last_result)
        self.publish_status()
        return item_id, dimensions_mm

    def _simulation_joint_state_callback(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in self.simulation_joint_names):
            self.simulation_current_positions = [
                float(positions[name]) for name in self.simulation_joint_names]

    def _clear_simulation_motion(self):
        self.simulation_waypoints = []
        self.simulation_solutions = []
        self.simulation_ik_index = 0
        self.simulation_started = None
        self.simulation_duration = 0.0
        self.simulation_completion = ''
        self.simulation_label = ''

    @staticmethod
    def _vertical_pose(x, y, z, yaw):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        # Downward tool Z: roll=pi, pitch=0, followed by base yaw.
        pose.orientation.x = math.cos(yaw / 2.0)
        pose.orientation.y = math.sin(yaw / 2.0)
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose

    def _pallet_tcp_pose(self, item, *, contact=True):
        transform = self.tf_buffer.lookup_transform(
            'link_base', 'pallet_frame', rclpy.time.Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pallet_q = (rotation.x, rotation.y, rotation.z, rotation.w)
        local_z = (
            (item.FLB.z + item.Dim.dz) / 1000.0
            if contact else self.pallet_unpack_approach_height)
        local = (
            (item.FLB.x + item.Dim.dx / 2.0) / 1000.0,
            (item.FLB.y + item.Dim.dy / 2.0) / 1000.0,
            local_z,
        )
        offset = self._quat_rotate(local, pallet_q)
        yaw = self._yaw_from_quaternion(pallet_q)
        if bool(getattr(item, 'rot', False)):
            yaw -= math.pi / 2.0
        return self._vertical_pose(
            translation.x + offset[0],
            translation.y + offset[1],
            translation.z + offset[2],
            yaw)

    def _incoming_tcp_pose(self):
        incoming = self.simulation_incoming
        if incoming is None:
            raise ValueError('simulated incoming item pose is unavailable')
        x, y, center_z = incoming['center']
        yaw = self._yaw_from_quaternion(incoming['quaternion'])
        return self._vertical_pose(
            x, y, center_z + incoming['size'][2] / 2.0, yaw)

    @staticmethod
    def _staging_slot_corner(slot):
        if not 0 <= int(slot) < 6:
            raise ValueError(f'invalid simulated staging slot {slot}')
        column = int(slot) % 3
        row = int(slot) // 3
        return -0.375 + 0.250 * column, 0.180 + 0.250 * row

    def _staging_tcp_pose(self, slot, item):
        x0, y0 = self._staging_slot_corner(slot)
        return self._vertical_pose(
            x0 + item.Dim.dx / 2000.0,
            y0 + item.Dim.dy / 2000.0,
            item.Dim.dz / 1000.0,
            0.0)

    @staticmethod
    def _copy_pose_with_z(pose, z):
        result = Pose()
        result.position.x = pose.position.x
        result.position.y = pose.position.y
        result.position.z = float(z)
        result.orientation = pose.orientation
        return result

    def _simulation_transfer_height(self, source, target):
        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base', 'pallet_frame', rclpy.time.Time())
            pallet_z = float(transform.transform.translation.z)
        except TransformException:
            pallet_z = 0.0
        return max(
            source.position.z + 0.10,
            target.position.z + 0.10,
            pallet_z + self.pallet_unpack_approach_height)

    def _simulation_route(self, source, target):
        descent = self.simulation_fixed_descent
        transfer_z = self._simulation_transfer_height(source, target)
        return [
            self._copy_pose_with_z(source, transfer_z),
            self._copy_pose_with_z(source, source.position.z + descent),
            source,
            self._copy_pose_with_z(source, transfer_z),
            self._copy_pose_with_z(target, transfer_z),
            self._copy_pose_with_z(target, target.position.z + descent),
            target,
            self._copy_pose_with_z(target, transfer_z),
        ]

    def _reserve_simulation_slot(self):
        occupied = set(self.holding_slots.values())
        slot = next((index for index in range(6) if index not in occupied), None)
        if slot is None:
            raise ValueError('no free simulated staging slot is available')
        self.active_slot = int(slot)
        return self.active_slot

    def _simulation_poses_for_operation(self, operation):
        if operation.source == 'incoming':
            source = self._incoming_tcp_pose()
        elif operation.source == 'pallet':
            source = self._pallet_tcp_pose(operation.source_item)
        elif operation.source == 'holding':
            key = self._item_key(operation.source_item)
            slot = self.holding_slots.get(key)
            if slot is None:
                raise ValueError(
                    f'holding item {key} has no simulated staging slot')
            self.active_slot = int(slot)
            source = self._staging_tcp_pose(slot, operation.source_item)
        else:
            raise ValueError(
                f'unsupported simulated operation source {operation.source!r}')

        if operation.kind == 'unpack':
            slot = self._reserve_simulation_slot()
            target = self._staging_tcp_pose(slot, operation.source_item)
        elif operation.target_box is not None:
            target = self._pallet_tcp_pose(operation.target_box)
        else:
            raise ValueError(
                f'simulated {operation.kind} operation has no target')
        return self._simulation_route(source, target)

    def _simulation_poses_for_pending(self):
        pending = self.loader.pending
        if pending is None:
            raise ValueError('simulated loading has no pending target')
        return self._simulation_route(
            self._incoming_tcp_pose(), self._pallet_tcp_pose(pending.box))

    def _start_simulation_motion(self, poses, *, completion, label):
        if not self.simulation_ik_client.service_is_ready():
            self._set_fault('MoveIt /compute_ik service is unavailable')
            return False
        if not poses:
            self._set_fault('simulated motion has no waypoints')
            return False
        self.simulation_waypoints = list(poses)
        if not self.simulation_seed_initialized:
            if self.simulation_current_positions is not None:
                self.simulation_joint_positions = list(
                    self.simulation_current_positions)
            self.simulation_home_positions = list(
                self.simulation_joint_positions)
            self.simulation_seed_initialized = True
        self.simulation_solutions = []
        self.simulation_ik_index = 0
        self.simulation_completion = str(completion)
        self.simulation_label = str(label)
        self.simulation_started = None
        self.simulation_duration = 0.0
        self.state = 'SIMULATING'
        self.fault = ''
        self.last_result = f'Simulating {label}: solving waypoint IK'
        self._request_next_simulation_ik()
        self.publish_status()
        return True

    def _request_next_simulation_ik(self):
        if self.simulation_ik_index >= len(self.simulation_waypoints):
            self._publish_simulation_trajectory()
            return
        seed = (
            self.simulation_solutions[-1]
            if self.simulation_solutions else self.simulation_joint_positions)
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'uf850'
        request.ik_request.ik_link_name = 'link_tcp'
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 1
        request.ik_request.robot_state.joint_state.name = list(
            self.simulation_joint_names)
        request.ik_request.robot_state.joint_state.position = list(seed)
        stamped = PoseStamped()
        stamped.header.frame_id = 'link_base'
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = self.simulation_waypoints[self.simulation_ik_index]
        request.ik_request.pose_stamped = stamped
        future = self.simulation_ik_client.call_async(request)
        future.add_done_callback(self._simulation_ik_completed)

    def _simulation_ik_completed(self, future):
        if self.state != 'SIMULATING':
            return
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'simulation IK request failed: {exc}')
            return
        if (result is None or
                result.error_code.val != MoveItErrorCodes.SUCCESS):
            code = None if result is None else result.error_code.val
            self._set_fault(
                f'simulation IK failed at waypoint '
                f'{self.simulation_ik_index + 1}/'
                f'{len(self.simulation_waypoints)}: code={code}')
            return
        positions = dict(zip(
            result.solution.joint_state.name,
            result.solution.joint_state.position))
        if not all(name in positions for name in self.simulation_joint_names):
            self._set_fault('simulation IK response omitted UF850 joints')
            return
        self.simulation_solutions.append([
            float(positions[name]) for name in self.simulation_joint_names])
        self.simulation_ik_index += 1
        self._request_next_simulation_ik()

    @staticmethod
    def _duration(seconds):
        nanoseconds = int(round(float(seconds) * 1_000_000_000.0))
        return Duration(
            sec=nanoseconds // 1_000_000_000,
            nanosec=nanoseconds % 1_000_000_000)

    def _publish_simulation_trajectory(self):
        start = self.simulation_joint_positions
        display = DisplayTrajectory()
        display.model_id = 'uf850'
        display.trajectory_start = RobotState()
        display.trajectory_start.joint_state.name = list(
            self.simulation_joint_names)
        display.trajectory_start.joint_state.position = list(start)
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(
            self.simulation_joint_names)
        points = [list(start)] + list(self.simulation_solutions)
        if self.simulation_home_positions is not None:
            points.append(list(self.simulation_home_positions))
        for index, positions in enumerate(points):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = self._duration(
                index * self.simulation_segment_duration)
            trajectory.joint_trajectory.points.append(point)
        display.trajectory.append(trajectory)
        self.simulation_display_pub.publish(display)
        self.simulation_joint_positions = list(points[-1])
        self.simulation_duration = max(
            self.simulation_segment_duration,
            (len(points) - 1) * self.simulation_segment_duration)
        self.simulation_started = time.monotonic()
        self.last_result = (
            f'Simulating {self.simulation_label}: '
            f'{len(self.simulation_solutions)} waypoints, fixed descent '
            f'{self.simulation_fixed_descent * 1000.0:.0f} mm')
        self.publish_status()

    def _finish_simulation_motion(self):
        completion = self.simulation_completion
        label = self.simulation_label
        self._clear_simulation_motion()
        if completion == 'direct':
            self._commit_succeeded_pick_place()
        elif completion == 'rearrangement':
            self._complete_rearrangement_operation()
        else:
            self._set_fault(
                f'simulated motion {label!r} has invalid completion action')

    def _plan_item(self, *, item_id, dimensions_mm):
        if self.rearrangement_enabled:
            return self.loader.plan_with_rearrangement(
                item_id=item_id, dimensions_mm=dimensions_mm)
        return super()._plan_item(
            item_id=item_id, dimensions_mm=dimensions_mm)

    def _accept_planning_result(self, pending):
        if not isinstance(pending, PendingPhysicalOperation):
            self.placed_ids_before_operation = self._scene_placed_item_ids()
            result = super()._accept_planning_result(pending)
            if self.simulation_enabled:
                self.target_acknowledged = True
                self.state = 'TARGET_READY'
                self.last_result = (
                    f'policy target {pending.sequence_id} ready for simulation')
                self.publish_status()
            return result
        self.active_operation = pending
        self.state = 'PLAN_READY'
        self.last_result = (
            f'MCTS/A* plan {pending.plan_id} ready: {pending.step_count} '
            f'operations; next={pending.kind}:{pending.source}')
        self._push_visualization(self.last_result)
        self.publish_status()
        if self.cycle_auto_start:
            self.rearrangement_auto_execute = True
            self._begin_rearrangement_operation()

    def start_loading_callback(self, request, response):
        if self.state == 'PLAN_READY' and self.loader.rearrangement is not None:
            self.rearrangement_auto_execute = True
            self.continuous_run_active = self.continuous_loading_enabled
            self._begin_rearrangement_operation()
            response.success = self.state != 'FAULT'
            response.message = (
                'MCTS/A* execution started' if response.success else self.fault)
            return response
        if (self.simulation_enabled and self.state == 'IDLE' and
                self.loader.pending is None):
            try:
                item_id, dimensions = self._sample_cardboard_simulation_item(
                    auto_start=True)
            except (RuntimeError, TypeError, ValueError) as exc:
                response.message = f'cardboard simulation sampling failed: {exc}'
                return response
            self.continuous_run_active = self.continuous_loading_enabled
            response.success = True
            response.message = (
                f'automatic simulation started with cardboard item {item_id}: '
                f'{dimensions} mm')
            return response
        return super().start_loading_callback(request, response)

    def _start_pick_place(self, response=None):
        if not self.simulation_enabled:
            return super()._start_pick_place(response)
        try:
            poses = self._simulation_poses_for_pending()
        except (TransformException, TypeError, ValueError) as exc:
            self._set_fault(f'cannot build simulated pack motion: {exc}')
            if response is not None:
                response.message = self.fault
            return response
        started = self._start_simulation_motion(
            poses, completion='direct', label='pack')
        if response is not None:
            response.success = bool(started)
            response.message = (
                'simulated PickAndPlace trajectory requested'
                if started else self.fault)
        return response

    def _discard_object_info_at_contact(self):
        if self.simulation_enabled:
            return
        return super()._discard_object_info_at_contact()

    def _start_continuous_localization(self):
        if not self.simulation_enabled:
            return super()._start_continuous_localization()
        try:
            self._sample_cardboard_simulation_item(auto_start=True)
        except (RuntimeError, TypeError, ValueError) as exc:
            self._set_fault(f'cardboard simulation sampling failed: {exc}')

    def target_applied_callback(self, message):
        operation = self.active_operation
        if (operation is None or
                self.state != 'REARRANGE_WAIT_TARGET'):
            return super().target_applied_callback(message)
        if not message.data or int(round(message.data[0])) != operation.sequence_id:
            return
        if len(message.data) >= 2 and message.data[1] < 0.5:
            self._set_fault('pallet localization rejected rearrangement target')
            return
        self.target_acknowledged = True
        self._start_rearrangement_source(operation)

    def _begin_rearrangement_operation(self):
        operation = self.loader.current_operation()
        if operation is None:
            self._finish_rearrangement_plan()
            return
        self.active_operation = operation
        self.target_acknowledged = False
        self.placed_ids_before_operation = self._scene_placed_item_ids()
        self.last_result = (
            f'plan {operation.plan_id}, operation '
            f'{operation.step_index + 1}/{operation.step_count}: '
            f'{operation.kind} from {operation.source}')
        if self.simulation_enabled:
            try:
                poses = self._simulation_poses_for_operation(operation)
            except (TransformException, TypeError, ValueError) as exc:
                self._set_fault(
                    f'cannot build simulated {operation.kind} motion: {exc}')
                return
            self._start_simulation_motion(
                poses,
                completion='rearrangement',
                label=f'{operation.kind} from {operation.source}')
            return
        # Object-info estimation leaves the TCP at contact. Before an unpack
        # or holding-item operation, retreat without grasping; the incoming
        # item will be re-estimated when its pack operation is reached.
        if (operation.source != 'incoming' and
                self.pickup_status.get('state') == 'OBJECT_INFO_READY'):
            if not self.object_info_discard_client.service_is_ready():
                self._set_fault('object-info retreat service is unavailable')
                return
            self.state = 'REARRANGE_RETREAT_INCOMING'
            future = self.object_info_discard_client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda done: self._service_response(done, 'object-info retreat'))
            return
        self._prepare_rearrangement_operation(operation)

    def _prepare_rearrangement_operation(self, operation):
        if operation.has_target:
            self.state = 'REARRANGE_WAIT_TARGET'
            self._publish_target(operation)
            self.last_target_publish = time.monotonic()
        else:
            self._start_rearrangement_source(operation)
        self.publish_status()

    def _start_rearrangement_source(self, operation):
        if operation.source == 'incoming':
            self._start_incoming_operation()
        elif operation.source == 'holding':
            self._start_holding_retrieve(operation)
        elif operation.source == 'pallet':
            self._start_pallet_retrieve(operation)
        else:
            self._set_fault(f'unsupported operation source {operation.source!r}')

    def _start_incoming_operation(self):
        if not self.pick_place_client.service_is_ready():
            self._set_fault('PickAndPlace service is unavailable')
            return
        self.expected_pipeline_operation_id = int(
            self.pipeline_status.get('operation_id', 0)) + 1
        self.state = 'REARRANGE_INCOMING'
        future = self.pick_place_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done: self._service_response(done, 'incoming PickAndPlace'))

    @staticmethod
    def _item_key(item):
        return tuple(map(int, item.to_key()))

    def _scene_placed_item_ids(self):
        """Return collision-backed and visual-only placed-item IDs."""
        return set(self.scene_status.get('placed_item_ids') or []).union(
            self.scene_status.get('placed_item_visual_ids') or [])

    def _start_holding_retrieve(self, operation):
        key = self._item_key(operation.source_item)
        slot = self.holding_slots.get(key)
        if slot is None:
            self._set_fault(f'holding item {key} has no assigned staging slot')
            return
        if not self.staging_retrieve_client.service_is_ready():
            self._set_fault('staging retrieval service is unavailable')
            return
        self.active_slot = int(slot)
        message = Int32(data=self.active_slot)
        self.staging_retrieve_selection_pub.publish(message)
        self.state = 'REARRANGE_RETRIEVE'
        self.staging_seen_active = False
        self._defer_service(
            self.staging_retrieve_client, 'staging retrieval')

    def _start_pallet_retrieve(self, operation):
        key = self._item_key(operation.source_item)
        obstacle_id = self.placed_obstacle_ids.get(key, '')
        if not obstacle_id and self.pending_obstacle_key == key:
            self.state = 'REARRANGE_REMOVE_OBSTACLE'
            self.waiting_removed_obstacle = '__pending_registration__'
            self.obstacle_wait_started = time.monotonic()
            return
        match = re.fullmatch(r'placed_item_(\d+)', obstacle_id)
        if match:
            if not self.remove_placed_obstacle_client.service_is_ready():
                self._set_fault('placed-obstacle removal service is unavailable')
                return
            request = SetInt16.Request()
            request.data = int(match.group(1))
            self.state = 'REARRANGE_REMOVE_OBSTACLE'
            self.waiting_removed_obstacle = obstacle_id
            self.obstacle_wait_started = time.monotonic()
            future = self.remove_placed_obstacle_client.call_async(request)
            future.add_done_callback(self._obstacle_removal_completed)
            return
        self._publish_pallet_retrieval_target(operation)

    def _obstacle_removal_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'placed-obstacle removal failed: {exc}')
            return
        if result is None or result.ret != 0:
            code = None if result is None else result.ret
            self._set_fault(f'placed-obstacle removal rejected: ret={code}')

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
    def _yaw_from_quaternion(quaternion):
        x, y, z, w = quaternion
        return math.atan2(
            2.0 * (w*z + x*y),
            1.0 - 2.0 * (y*y + z*z))

    def _pallet_item_retrieval_message(self, operation):
        item = operation.source_item
        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base', 'pallet_frame', rclpy.time.Time())
        except TransformException as exc:
            raise ValueError(f'live pallet transform is unavailable: {exc}')
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pallet_q = (rotation.x, rotation.y, rotation.z, rotation.w)
        local_top_center = (
            (item.FLB.x + item.Dim.dx / 2.0) / 1000.0,
            (item.FLB.y + item.Dim.dy / 2.0) / 1000.0,
            (item.FLB.z + item.Dim.dz) / 1000.0,
        )
        offset = self._quat_rotate(local_top_center, pallet_q)
        local_approach = (
            local_top_center[0],
            local_top_center[1],
            self.pallet_unpack_approach_height,
        )
        approach_offset = self._quat_rotate(local_approach, pallet_q)
        center = (
            translation.x + offset[0],
            translation.y + offset[1],
            translation.z + offset[2],
        )
        yaw = self._yaw_from_quaternion(pallet_q)
        message = Float64MultiArray()
        message.data = [
            float(operation.sequence_id),
            center[0], center[1], center[2],
            math.pi, 0.0, yaw,
            item.Dim.dx / 1000.0,
            item.Dim.dy / 1000.0,
            item.Dim.dz / 1000.0,
            0.030,
            yaw,
            translation.z + approach_offset[2],
        ]
        return message

    def _publish_pallet_retrieval_target(self, operation):
        try:
            message = self._pallet_item_retrieval_message(operation)
        except ValueError as exc:
            self._set_fault(str(exc))
            return
        self.retrieval_target_pub.publish(message)
        self.state = 'REARRANGE_PLAN_APPROACH'
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self._defer_service(
            self.plan_retrieval_approach_client,
            'above-container pallet approach planning')

    def _plan_pallet_pregrasp(self, operation):
        try:
            message = self._pallet_item_retrieval_message(operation)
        except ValueError as exc:
            self._set_fault(str(exc))
            return
        # Refresh the target because the approach trajectory can take longer
        # than the motion coordinator's two-second target freshness window.
        self.retrieval_target_pub.publish(message)
        self.state = 'REARRANGE_PLAN_PICK'
        self.expected_motion_operation_id = int(
            self.motion_status.get('operation_id', 0)) + 1
        self._defer_service(
            self.plan_retrieval_client, 'pallet pre-pick planning')

    def _defer_service(self, client, label, delay=0.15):
        holder = {}

        def invoke():
            holder['timer'].cancel()
            if not client.service_is_ready():
                self._set_fault(f'{label} service is unavailable')
                return
            future = client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda done: self._service_response(done, label))
        holder['timer'] = self.create_timer(delay, invoke)

    def _service_response(self, future, label):
        try:
            result = future.result()
        except Exception as exc:
            self._set_fault(f'{label} failed: {exc}')
            return
        if result is None or not result.success:
            reason = 'no response' if result is None else result.message
            self._set_fault(f'{label} rejected: {reason}')

    def _start_staging_store(self, operation):
        occupied = {
            int(entry['slot']) for entry in self.staging_status.get('slots', [])
            if entry.get('occupied')
        }
        reserved = set(self.holding_slots.values())
        slot = next(
            (index for index in range(6)
             if index not in occupied and index not in reserved),
            None,
        )
        if slot is None:
            self._set_fault('no free staging slot is available for unpacking')
            return
        self.active_slot = slot
        self.staging_store_selection_pub.publish(Int32(data=slot))
        self.state = 'REARRANGE_STORE'
        self.staging_seen_active = False
        self._defer_service(self.staging_store_client, 'staging store')

    def _start_place_only(self):
        if not self.start_place_client.service_is_ready():
            self._set_fault('place pipeline service is unavailable')
            return
        self.expected_place_operation_id = int(
            self.place_status.get('operation_id', 0)) + 1
        self.state = 'REARRANGE_PLACE'
        future = self.start_place_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done: self._service_response(done, 'rearrangement place'))

    def tick(self):
        if self.state == 'SIMULATING':
            if (self.simulation_started is not None and
                    time.monotonic() - self.simulation_started >=
                    self.simulation_duration):
                self._finish_simulation_motion()
            return
        if (self.simulation_enabled and self.state == 'TARGET_READY' and
                (self.auto_start or self.cycle_auto_start)):
            self._start_pick_place()
            return
        if self.state not in self.REARRANGEMENT_STATES:
            super().tick()
            return
        operation = self.active_operation
        if self.state == 'PLAN_READY':
            return
        if operation is None:
            self._set_fault('rearrangement state has no active operation')
            return
        if self.state == 'REARRANGE_RETREAT_INCOMING':
            if self.pickup_status.get('state') == 'FAULT':
                self._set_fault(self.pickup_status.get(
                    'fault', 'incoming-item retreat failed'))
            elif self.pickup_status.get('state') == 'SUCCEEDED':
                self._prepare_rearrangement_operation(operation)
            return
        if self.state == 'REARRANGE_WAIT_TARGET':
            if time.monotonic() - self.last_target_publish >= self.target_republish_sec:
                self._publish_target(operation)
                self.last_target_publish = time.monotonic()
            return
        if self.state == 'REARRANGE_REMOVE_OBSTACLE':
            if self.waiting_removed_obstacle == '__pending_registration__':
                self._update_pending_obstacle_mapping()
                key = self._item_key(operation.source_item)
                if key in self.placed_obstacle_ids:
                    self.waiting_removed_obstacle = ''
                    self._start_pallet_retrieve(operation)
                elif (self.obstacle_wait_started is not None and
                      time.monotonic() - self.obstacle_wait_started >= 2.0):
                    # Collision registration is optional. If no object appears,
                    # continue using the known physical pose and dimensions.
                    self.pending_obstacle_key = None
                    self.waiting_removed_obstacle = ''
                    self._publish_pallet_retrieval_target(operation)
                return
            current = self._scene_placed_item_ids()
            if self.waiting_removed_obstacle not in current:
                self.placed_obstacle_ids.pop(
                    self._item_key(operation.source_item), None)
                self.waiting_removed_obstacle = ''
                self._publish_pallet_retrieval_target(operation)
            elif (self.obstacle_wait_started is not None and
                  time.monotonic() - self.obstacle_wait_started >= 5.0):
                self._set_fault(
                    'timed out waiting for the source pallet obstacle to be removed')
            return
        if self.state == 'REARRANGE_PLAN_APPROACH':
            if not self._motion_is_current():
                return
            motion_state = self.motion_status.get('state')
            if motion_state == 'FAULT':
                self._set_fault(self.motion_status.get(
                    'fault', 'above-container pallet approach planning failed'))
            elif motion_state == 'PLANNED':
                self.state = 'REARRANGE_EXECUTE_APPROACH'
                self.execute_motion_client.call_async(
                    Trigger.Request()).add_done_callback(
                        lambda done: self._service_response(
                            done, 'above-container pallet approach execution'))
            return
        if self.state == 'REARRANGE_EXECUTE_APPROACH':
            if not self._motion_is_current():
                return
            motion_state = self.motion_status.get('state')
            if motion_state == 'FAULT':
                self._set_fault(self.motion_status.get(
                    'fault', 'above-container pallet approach execution failed'))
            elif motion_state == 'SUCCEEDED':
                self._plan_pallet_pregrasp(operation)
            return
        if self.state == 'REARRANGE_PLAN_PICK':
            if not self._motion_is_current():
                return
            motion_state = self.motion_status.get('state')
            if motion_state == 'FAULT':
                self._set_fault(self.motion_status.get(
                    'fault', 'pallet pre-pick planning failed'))
            elif motion_state == 'PLANNED':
                self.state = 'REARRANGE_EXECUTE_PICK'
                self.execute_motion_client.call_async(Trigger.Request()).add_done_callback(
                    lambda done: self._service_response(
                        done, 'pallet pre-pick execution'))
            return
        if self.state == 'REARRANGE_EXECUTE_PICK':
            if not self._motion_is_current():
                return
            motion_state = self.motion_status.get('state')
            if motion_state == 'FAULT':
                self._set_fault(self.motion_status.get(
                    'fault', 'pallet pre-pick execution failed'))
            elif motion_state == 'SUCCEEDED':
                self.expected_supervisor_operation_id = int(
                    self.supervisor_status.get('operation_id', 0)) + 1
                self.state = 'REARRANGE_SERVO_PICK'
                self.start_supervisor_pick_client.call_async(
                    Trigger.Request()).add_done_callback(
                        lambda done: self._service_response(
                            done, 'pallet safe-servo pickup'))
            return
        if self.state == 'REARRANGE_SERVO_PICK':
            if not self._supervisor_is_current():
                return
            supervisor_state = self.supervisor_status.get('state')
            if supervisor_state == 'FAULT':
                self._set_fault(self.supervisor_status.get(
                    'fault', 'pallet pickup failed'))
            elif supervisor_state == 'SUCCEEDED':
                if operation.kind == 'unpack':
                    self._start_staging_store(operation)
                else:
                    self._start_place_only()
            return
        if self.state in ('REARRANGE_STORE', 'REARRANGE_RETRIEVE'):
            staging_state = self.staging_status.get('state')
            if staging_state not in (None, 'IDLE', 'SUCCEEDED', 'FAULT'):
                self.staging_seen_active = True
            if not self.staging_seen_active:
                return
            if staging_state == 'FAULT':
                self._set_fault(self.staging_status.get(
                    'fault', 'staging operation failed'))
            elif staging_state == 'SUCCEEDED':
                if self.state == 'REARRANGE_STORE':
                    self._complete_rearrangement_operation()
                else:
                    self._start_place_only()
            return
        if self.state == 'REARRANGE_PLACE':
            if not self._place_is_current():
                return
            place_state = self.place_status.get('state')
            if place_state == 'FAULT':
                self._set_fault(self.place_status.get(
                    'fault', 'rearrangement placement failed'))
            elif place_state == 'SUCCEEDED':
                self._complete_rearrangement_operation()
            return
        if self.state == 'REARRANGE_INCOMING':
            current = int(self.pipeline_status.get('operation_id', -1))
            if current < int(self.expected_pipeline_operation_id or 0):
                return
            pipeline_state = self.pipeline_status.get('state')
            if pipeline_state == 'FAULT':
                self._set_fault(self.pipeline_status.get(
                    'fault', 'incoming PickAndPlace failed'))
            elif pipeline_state == 'SUCCEEDED':
                self._complete_rearrangement_operation()

    def _motion_is_current(self):
        return int(self.motion_status.get('operation_id', -1)) >= int(
            self.expected_motion_operation_id or 0)

    def _supervisor_is_current(self):
        return int(self.supervisor_status.get('operation_id', -1)) >= int(
            self.expected_supervisor_operation_id or 0)

    def _place_is_current(self):
        return int(self.place_status.get('operation_id', -1)) >= int(
            self.expected_place_operation_id or 0)

    def _complete_rearrangement_operation(self):
        operation = self.active_operation
        if operation is None:
            self._set_fault('cannot commit missing rearrangement operation')
            return
        source_key = (
            None if operation.source_item is None else
            self._item_key(operation.source_item))
        target_key = (
            None if operation.target_box is None else
            self._item_key(operation.target_box))
        try:
            complete = self.loader.commit_current_operation(
                operation.sequence_id)
        except Exception as exc:
            self._set_fault(
                f'robot operation succeeded but virtual-state commit failed: {exc}')
            return
        if operation.kind == 'unpack':
            self.holding_slots[source_key] = int(self.active_slot)
            self.placed_obstacle_ids.pop(source_key, None)
        elif operation.source == 'holding':
            self.holding_slots.pop(source_key, None)
            self.pending_obstacle_key = target_key
        elif operation.kind == 'repack':
            self.placed_obstacle_ids.pop(source_key, None)
            self.pending_obstacle_key = target_key
        else:
            self.pending_obstacle_key = target_key
        self._update_pending_obstacle_mapping()
        self._push_visualization(
            f'Completed operation {operation.step_index + 1}/'
            f'{operation.step_count}: {operation.kind}')
        self.active_slot = None
        self.staging_seen_active = False
        if complete:
            self._finish_rearrangement_plan()
            return
        self.active_operation = self.loader.current_operation()
        self._begin_rearrangement_operation()

    def _update_pending_obstacle_mapping(self):
        if self.pending_obstacle_key is None:
            return
        current = self._scene_placed_item_ids()
        new_ids = current - self.placed_ids_before_operation
        if not new_ids:
            return

        def number(value):
            match = re.fullmatch(r'placed_item_(\d+)', str(value))
            return int(match.group(1)) if match else -1
        placed_id = max(new_ids, key=number)
        self.placed_obstacle_ids[self.pending_obstacle_key] = placed_id
        self.pending_obstacle_key = None

    def _finish_rearrangement_plan(self):
        self.state = 'IDLE'
        self.fault = ''
        self.last_result = 'MCTS/A* execution plan completed'
        self.active_operation = None
        self.rearrangement_auto_execute = False
        self.cycle_auto_start = False
        self.target_acknowledged = False
        self.expected_pipeline_operation_id = None
        self.get_logger().info(self.last_result)
        self._push_visualization(self.last_result)
        continue_loading = (
            self.continuous_loading_enabled and self.continuous_run_active)
        if continue_loading:
            self._start_continuous_localization()
        else:
            self.continuous_run_active = False
            self.publish_status()

    def _commit_succeeded_pick_place(self):
        pending = self.loader.pending
        target_key = None if pending is None else self._item_key(pending.box)
        super()._commit_succeeded_pick_place()
        if target_key is not None and self.state != 'FAULT':
            self.pending_obstacle_key = target_key
            self._update_pending_obstacle_mapping()

    def abort_loading_callback(self, request, response):
        if self.state == 'SIMULATING':
            pending = self.loader.pending
            rearrangement = self.loader.rearrangement
            if pending is not None and rearrangement is None:
                self.loader.discard_pending(pending.sequence_id)
            self._clear_simulation_motion()
            self.fault = ''
            if rearrangement is None:
                self.state = 'IDLE'
                self.active_operation = None
                self.last_result = (
                    'policy simulation aborted; no physical command was sent')
            else:
                self.state = 'PLAN_READY'
                self.active_operation = self.loader.current_operation()
                self.last_result = (
                    'policy simulation paused before committing the active '
                    'operation; press Policy loading to resume')
            self.rearrangement_auto_execute = False
            self.cycle_auto_start = False
            self.continuous_run_active = False
            response.success = True
            response.message = self.last_result
            self.publish_status()
            return response
        if self.simulation_enabled:
            if self.state == 'IDLE' and self.loader.pending is None:
                response.message = 'there is no active policy simulation'
                return response
            if self.state == 'PLANNING' and self.planning_future is not None:
                self.state = 'ABORTING'
                self.abort_requested = True
                response.success = True
                response.message = 'policy simulation planning abort requested'
                self.publish_status()
                return response
            pending = self.loader.pending
            if pending is not None and self.loader.rearrangement is None:
                self.loader.discard_pending(pending.sequence_id)
            if self.loader.rearrangement is not None:
                self.state = 'PLAN_READY'
                self.active_operation = self.loader.current_operation()
                self.last_result = 'policy simulation remains paused'
            else:
                self.state = 'IDLE'
                self.active_operation = None
                self.last_result = (
                    'policy simulation aborted; no physical command was sent')
            self.fault = ''
            self.cycle_auto_start = False
            self.continuous_run_active = False
            self.abort_requested = False
            response.success = True
            response.message = self.last_result
            self.publish_status()
            return response
        if self.state not in self.REARRANGEMENT_STATES:
            return super().abort_loading_callback(request, response)
        for client in (
                self.pick_place_abort_client, self.abort_place_client,
                self.cancel_motion_client, self.abort_supervisor_client):
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        self._set_fault(
            'rearrangement execution aborted; virtual state retains only '
            'physically confirmed operations')
        response.success = True
        response.message = self.fault
        return response

    def reset_pallet_callback(self, request, response):
        if self.simulation_enabled:
            if self.state in ('PLANNING', 'ABORTING', 'SIMULATING'):
                response.message = (
                    f'cannot reset policy simulation in {self.state}')
                return response
            self.loader.reset()
            self.state = 'IDLE'
            self.fault = ''
            self.last_result = 'policy simulation and virtual pallet cleared'
            self.localization_started = None
            self.cycle_auto_start = False
            self.continuous_run_active = False
            self.abort_requested = False
            self.abort_started = None
            self.start_request_pending = False
            self.target_acknowledged = False
            self.expected_pipeline_operation_id = None
            self.holding_slots.clear()
            self.placed_obstacle_ids.clear()
            self.active_operation = None
            self.active_slot = None
            self.pending_obstacle_key = None
            self._clear_simulation_motion()
            self.simulation_seed_initialized = False
            self.simulation_home_positions = None
            self.simulation_incoming = None
            self.simulation_next_item_id = 1
            self.simulation_sample_count = 0
            response.success = True
            response.message = self.last_result
            self._push_visualization(self.last_result)
            self.publish_status()
            return response
        if self.state in self.REARRANGEMENT_STATES - {'PLAN_READY'}:
            response.message = f'cannot reset pallet state in {self.state}'
            return response
        result = super().reset_pallet_callback(request, response)
        if result.success:
            self.holding_slots.clear()
            self.placed_obstacle_ids.clear()
            self.active_operation = None
            self.active_slot = None
            self.pending_obstacle_key = None
            self._clear_simulation_motion()
            self.simulation_seed_initialized = False
            self.simulation_home_positions = None
        return result

    def _log_pending(self, pending):
        self.get_logger().info(
            'policy target %d: corner=(%d, %d, %d) mm, rotate_90=%s, '
            'action=%d, ems=%d, value=%.6f, virtual_dim=(%d, %d, %d) mm' % (
                pending.sequence_id,
                pending.box.FLB.x,
                pending.box.FLB.y,
                pending.box.FLB.z,
                pending.box.rot,
                pending.action_index,
                pending.ems_index,
                pending.predicted_value,
                pending.box.Virtual_Dim.dx,
                pending.box.Virtual_Dim.dy,
                pending.box.Virtual_Dim.dz,
            ))

    def _extend_status(self, payload, pending):
        operation = getattr(self, 'active_operation', None)
        rearrangement = getattr(self.loader, 'rearrangement', None)
        rearrangement_enabled = bool(getattr(
            self, 'rearrangement_enabled', False))
        payload.update({
            'selection_pipeline': (
                'learned_policy_with_mcts_astar'
                if rearrangement_enabled else 'learned_policy_only'),
            'rearrangement_enabled': rearrangement_enabled,
            'mcts_enabled': rearrangement_enabled,
            'astar_enabled': bool(
                rearrangement_enabled and
                getattr(self.loader, 'optimize_sequence', False)),
            'checkpoint': str(self.loader.checkpoint_path),
            'device': str(getattr(self.loader.agent, 'device', self.loader.device)),
            'clearance_mm': int(self.loader.clearance_mm),
            'height_tolerance_mm': float(getattr(
                self.loader, 'height_tolerance', 0.0)),
            'holding_slots': {
                ','.join(map(str, key)): int(slot)
                for key, slot in getattr(self, 'holding_slots', {}).items()
            },
            'simulation_enabled': bool(getattr(
                self, 'simulation_enabled', False)),
            'simulation_fixed_descent_mm': float(getattr(
                self, 'simulation_fixed_descent', 0.030)) * 1000.0,
            'simulation_label': str(getattr(
                self, 'simulation_label', '')) or None,
            'simulation_item_source': 'cardboard',
            'simulation_sample_count': int(getattr(
                self, 'simulation_sample_count', 0)),
        })
        if rearrangement is not None:
            payload.update({
                'rearrangement_plan_id': int(rearrangement.plan_id),
                'rearrangement_step_count': len(rearrangement.steps),
                'rearrangement_step_index': int(rearrangement.step_index),
                'mcts_stats': self._json_safe(rearrangement.mcts_stats),
                'astar_stats': self._json_safe(rearrangement.astar_stats),
            })
        if operation is not None:
            payload.update({
                'operation_kind': operation.kind,
                'operation_source': operation.source,
                'operation_number': int(operation.step_index + 1),
                'operation_count': int(operation.step_count),
                'operation_item_id': int(operation.item_id),
                'active_staging_slot': self.active_slot,
            })
            if operation.target_box is not None:
                payload.update({
                    'target_corner_mm': list(map(
                        int, operation.target_box.FLB.numpy())),
                    'rotate_item_90_deg': bool(operation.target_box.rot),
                    'raw_dimension_mm': list(map(int, operation.raw_dim.raw())),
                    'virtual_dimension_mm': list(map(
                        int, operation.target_box.Virtual_Dim.raw())),
                })
        if pending is None:
            return
        payload.update({
            'target_corner_mm': [
                int(pending.box.FLB.x),
                int(pending.box.FLB.y),
                int(pending.box.FLB.z),
            ],
            'rotate_item_90_deg': bool(pending.box.rot),
            'raw_dimension_mm': list(map(int, pending.raw_dim.raw())),
            'virtual_dimension_mm': list(map(int, pending.box.Virtual_Dim.raw())),
            'policy_action_index': int(pending.action_index),
            'policy_ems_index': int(pending.ems_index),
            'policy_predicted_value': float(pending.predicted_value),
        })


def main(args=None):
    rclpy.init(args=args)
    node = PolicyLoadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
