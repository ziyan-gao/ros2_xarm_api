import json
import math
from pathlib import Path
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Point, Pose, PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
from xarm_msgs.srv import PlanExec, PlanJoint, PlanPose, PlanSingleStraight
import yaml


class MotionCoordinator(Node):
    """Stateful, named-waypoint front end for the xArm MoveIt planner."""

    IDLE = 'IDLE'
    PLANNING = 'PLANNING'
    PREPARED = 'PREPARED'
    PLANNED = 'PLANNED'
    EXECUTING = 'EXECUTING'
    CANCELING = 'CANCELING'
    PAUSED = 'PAUSED'
    SUCCEEDED = 'SUCCEEDED'
    FAULT = 'FAULT'

    def __init__(self):
        super().__init__('motion_coordinator')
        self.declare_parameter(
            'waypoint_file', '/workspace/config/taught_waypoints.yaml')
        self.declare_parameter(
            'pregrasp_snapshot_file',
            '/workspace/config/verified_pregrasp_snapshot.json')
        self.declare_parameter('refined_boxes_topic', '/pointcloud_detection/boxes')
        self.declare_parameter(
            'stable_boxes_topic', '/object_info_estimation/stable_boxes')
        self.declare_parameter('target_box_id', -1)
        self.declare_parameter('pregrasp_clearance_m', 0.03)
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_state_timeout_sec', 0.5)
        self.declare_parameter('observation_joint_tolerance_rad', 0.02)
        self.declare_parameter('pre_place_clearance_m', 0.03)
        self.declare_parameter('transfer_corner_height_m', 0.47)
        self.waypoint_file = Path(
            self.get_parameter('waypoint_file').value).expanduser()
        self.pregrasp_snapshot_file = Path(
            self.get_parameter('pregrasp_snapshot_file').value).expanduser()

        def p(name):
            return self.get_parameter(name).value
        self.target_box_id = int(p('target_box_id'))
        self.pregrasp_clearance = float(p('pregrasp_clearance_m'))
        self.max_detection_age = float(p('max_detection_age_sec'))
        self.joint_state_topic = str(p('joint_state_topic'))
        self.joint_state_timeout = float(p('joint_state_timeout_sec'))
        self.observation_joint_tolerance = float(
            p('observation_joint_tolerance_rad'))
        self.pre_place_clearance = float(p('pre_place_clearance_m'))
        self.transfer_corner_height = float(p('transfer_corner_height_m'))
        self.default_transfer_corner_height = self.transfer_corner_height
        if self.pregrasp_clearance <= 0.0:
            raise ValueError('pregrasp_clearance_m must be positive')
        if self.joint_state_timeout <= 0.0:
            raise ValueError('joint_state_timeout_sec must be positive')
        if self.transfer_corner_height <= 0.0:
            raise ValueError('transfer_corner_height_m must be positive')
        self.state = self.IDLE
        self.target = None
        self.fault = ''
        self.cancel_requested = False
        self.pause_requested = False
        self.operation_id = 0
        self.refined_boxes = {}
        self.stable_refined_boxes = {}
        self.pre_place_pose = None
        self.pre_place_pose_time = None
        self.attached_item_geometry = None
        self.pre_place_tcp_pose = None
        self.transfer_tcp_pose = None
        self.nominal_transfer_corner_z = None
        self.placement_corner_correction = (0.0, 0.0, 0.0)
        self.pallet_locked = False
        self.rotate_item_90 = False
        self.keep_eef_perpendicular = True
        self.place_target_xyz = None
        self.planned_pregrasp = None
        self.staging_retrieve_target = None
        self.staging_store_transfer_target = None
        self.transfer_context = ''
        self.last_joint_state_time = None
        self.latest_joint_positions = {}
        self._restore_pregrasp_snapshot()

        self.plan_client = self.create_client(
            PlanJoint, '/xarm_joint_plan')
        self.pose_plan_client = self.create_client(
            PlanPose, '/xarm_pose_plan')
        self.straight_plan_client = self.create_client(
            PlanSingleStraight, '/xarm_straight_plan')
        self.constrained_pose_plan_client = self.create_client(
            PlanPose, '/xarm_pose_plan_orientation_constrained')
        self.exec_client = self.create_client(
            PlanExec, '/xarm_exec_plan')
        self.cancel_client = self.create_client(
            CancelGoal,
            '/uf850_traj_controller/follow_joint_trajectory/_action/cancel_goal')
        self.status_pub = self.create_publisher(
            String, '/motion_coordinator/status', 10)
        self.pregrasp_marker_pub = self.create_publisher(
            Marker, '/motion_coordinator/pregrasp_pose', 10)
        self.create_subscription(
            MarkerArray, str(p('refined_boxes_topic')),
            self.refined_boxes_callback, 10)
        self.create_subscription(
            MarkerArray, str(p('stable_boxes_topic')),
            self.stable_boxes_callback, 10)
        self.create_subscription(
            PoseStamped, '/pallet_localization/pre_place_pose',
            self.pre_place_pose_callback, 10)
        self.create_subscription(
            String, '/pallet_localization/status',
            self.pallet_status_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/pallet_localization/config_state',
            self.pallet_config_callback, 10)
        self.create_subscription(
            String, '/planning_scene_obstacles/status',
            self.planning_scene_status_callback, 10)
        self.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/staging_slots/retrieve_target',
            self.staging_retrieve_target_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/staging_slots/store_transfer_target',
            self.staging_store_transfer_target_callback, 10)
        self.pick_path_status = {}
        self.pick_path_status_time = 0.0
        self.top_face_view = None
        self.create_subscription(PoseStamped, '/top_face_debug/view_target',
                                 lambda msg: setattr(self, 'top_face_view', msg), 10)
        self.create_service(Trigger, '/motion_coordinator/plan_top_face_view',
                            self.plan_top_face_view_callback)
        self.create_subscription(String, '/pickup_supervisor/status', self._pick_path_status, 10)
        self.create_service(Trigger, '/motion_coordinator/prepare_pick_waypoints',
                            self.prepare_pick_waypoints_callback)
        self.create_service(Trigger, '/motion_coordinator/accept_pick_waypoints',
                            self.accept_pick_waypoints_callback)

        self.create_service(
            Trigger, '/motion_coordinator/plan_observation',
            lambda request, response: self.plan_waypoint(
                'observation', response))
        self.create_service(
            Trigger, '/motion_coordinator/plan_intermediate',
            lambda request, response: self.plan_waypoint(
                'intermediate', response))
        self.create_service(
            Trigger, '/motion_coordinator/prepare_transfer',
            self.prepare_transfer_callback)
        self.create_service(
            Trigger, '/motion_coordinator/prepare_staging_store_transfer',
            self.prepare_staging_store_transfer_callback)
        self.create_service(
            Trigger, '/motion_coordinator/plan_transfer',
            self.plan_transfer_callback)
        self.create_service(
            Trigger, '/motion_coordinator/plan_pre_place',
            self.plan_pre_place_callback)
        self.create_service(
            Trigger, '/motion_coordinator/plan_pregrasp',
            self.plan_pregrasp_callback)
        self.create_service(
            Trigger, '/motion_coordinator/plan_staging_approach',
            self.plan_staging_approach_callback)
        self.create_service(
            Trigger, '/motion_coordinator/plan_staging_pregrasp',
            self.plan_staging_pregrasp_callback)
        self.create_service(
            Trigger, '/motion_coordinator/execute', self.execute_callback)
        self.create_service(
            Trigger, '/motion_coordinator/cancel', self.cancel_callback)
        self.create_service(
            SetBool, '/motion_coordinator/pause', self.pause_callback)
        self.create_service(
            Trigger, '/motion_coordinator/resume', self.resume_callback)
        self.create_service(
            Trigger, '/motion_coordinator/reset', self.reset_callback)
        self.create_service(
            Trigger, '/motion_coordinator/accept_direct_transfer',
            self.accept_direct_transfer_callback)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'motion coordinator ready; waypoint_file={self.waypoint_file}')

    def plan_top_face_view_callback(self, _request, response):
        """Debug camera move; use the normal collision-checked pose planner."""
        if self.state not in (self.IDLE, self.SUCCEEDED):
            response.message = f'motion coordinator is not idle: {self.state}'
            return response
        if not self._require_fresh_joint_state(response, 'top-face observation'):
            return response
        target = self.top_face_view
        if target is None:
            response.message = 'no top-face observation target'
            return response
        age = (self.get_clock().now()-rclpy.time.Time.from_msg(target.header.stamp)).nanoseconds*1e-9
        p, q = target.pose.position, target.pose.orientation
        if (target.header.frame_id != 'link_base' or not 0 <= age < 2 or
                not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 1e-3):
            response.message = 'invalid/stale top-face observation target'
            return response
        if self.attached_item_geometry is not None or not self.pose_plan_client.service_is_ready():
            response.message = 'empty tool and pose planner are required'
            return response
        self.planned_pregrasp = None
        self._clear_pregrasp_snapshot()
        result = self._start_pose_plan(target.pose, -100, response)
        self.target = 'top_face_view'
        self.publish_status()
        return result

    def _restore_pregrasp_snapshot(self):
        try:
            document = json.loads(
                self.pregrasp_snapshot_file.read_text(encoding='utf-8'))
            snapshot = document['planned_pregrasp']
            if document.get('format_version') != 1 or not isinstance(snapshot, dict):
                raise ValueError('unsupported snapshot format')
            required = (
                'box_id', 'x_m', 'y_m', 'center_z_m', 'size_x_m',
                'size_y_m', 'size_z_m', 'top_z_m', 'pregrasp_z_m',
                'yaw_rad', 'planned_stamp_sec')
            if any(key not in snapshot for key in required):
                raise ValueError('snapshot fields are incomplete')
            numeric = [float(snapshot[key]) for key in required[1:]]
            if not all(math.isfinite(value) for value in numeric):
                raise ValueError('snapshot contains non-finite values')
            box_id = int(snapshot['box_id'])
        except FileNotFoundError:
            return
        except (OSError, TypeError, ValueError, KeyError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'ignoring invalid pre-grasp snapshot: {exc}')
            return
        self.planned_pregrasp = snapshot
        self.target = f'pregrasp_box_{box_id}'
        self.state = self.SUCCEEDED
        self.get_logger().warning(
            'restored verified pre-grasp snapshot; live TCP validation is '
            'required before pickup')

    def _persist_pregrasp_snapshot(self):
        target = str(self.target)
        if (not self.planned_pregrasp or
                not target.startswith(
                    ('pregrasp_box_', 'straight_pregrasp_box_'))):
            return
        # The successful execution time is the start of the snapshot validity
        # window. Pickup still independently checks the live TCP pose.
        self.planned_pregrasp['planned_stamp_sec'] = (
            self.get_clock().now().nanoseconds * 1e-9)
        document = {
            'format_version': 1,
            'target': self.target,
            'planned_pregrasp': self.planned_pregrasp,
        }
        temporary = self.pregrasp_snapshot_file.with_suffix('.tmp')
        try:
            self.pregrasp_snapshot_file.parent.mkdir(parents=True, exist_ok=True)
            temporary.write_text(
                json.dumps(document, separators=(',', ':')) + '\n',
                encoding='utf-8')
            temporary.replace(self.pregrasp_snapshot_file)
        except OSError as exc:
            self.get_logger().error(f'cannot persist pre-grasp snapshot: {exc}')

    def _clear_pregrasp_snapshot(self):
        try:
            self.pregrasp_snapshot_file.unlink(missing_ok=True)
        except OSError as exc:
            self.get_logger().warning(f'cannot remove pre-grasp snapshot: {exc}')

    def _set_state(self, state, fault=''):
        self.state = state
        self.fault = fault
        self.publish_status()
        if fault:
            self.get_logger().error(fault)

    def joint_state_callback(self, message):
        self.last_joint_state_time = time.monotonic()
        self.latest_joint_positions = {
            str(name): float(position)
            for name, position in zip(message.name, message.position)
        }

    def _require_fresh_joint_state(self, response, action):
        if self.last_joint_state_time is None:
            response.message = f'cannot {action}: no /joint_states received'
            return False
        age = time.monotonic() - self.last_joint_state_time
        if age > self.joint_state_timeout:
            response.message = (
                f'cannot {action}: /joint_states is stale '
                f'({age:.3f} s > {self.joint_state_timeout:.3f} s)')
            return False
        return True

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'state': self.state,
            'target': self.target,
            'fault': self.fault,
            'operation_id': self.operation_id,
            'planner_ready': self.plan_client.service_is_ready(),
            'pose_planner_ready': self.pose_plan_client.service_is_ready(),
            'straight_planner_ready': self.straight_plan_client.service_is_ready(),
            'constrained_pose_planner_ready': (
                self.constrained_pose_plan_client.service_is_ready()),
            'executor_ready': self.exec_client.service_is_ready(),
            'cancel_ready': self.cancel_client.service_is_ready(),
            'planned_pregrasp': self.planned_pregrasp,
            'pre_place_tcp_z_m': (
                None if self.pre_place_tcp_pose is None else
                self.pre_place_tcp_pose.position.z),
            'pre_place_tcp_xyz_m': (
                None if self.pre_place_tcp_pose is None else [
                    self.pre_place_tcp_pose.position.x,
                    self.pre_place_tcp_pose.position.y,
                    self.pre_place_tcp_pose.position.z]),
            'transport_corner_clearance_z_m': self.nominal_transfer_corner_z,
            'transfer_tcp_z_m': (
                None if self.transfer_tcp_pose is None else
                self.transfer_tcp_pose.position.z),
            'transfer_tcp_xyz_m': (
                None if self.transfer_tcp_pose is None else [
                    self.transfer_tcp_pose.position.x,
                    self.transfer_tcp_pose.position.y,
                    self.transfer_tcp_pose.position.z]),
            'transfer_tcp_quaternion_xyzw': (
                None if self.transfer_tcp_pose is None else [
                    self.transfer_tcp_pose.orientation.x,
                    self.transfer_tcp_pose.orientation.y,
                    self.transfer_tcp_pose.orientation.z,
                    self.transfer_tcp_pose.orientation.w]),
            'nominal_transfer_corner_z_m': self.nominal_transfer_corner_z,
            'transfer_corner_height_pallet_m': self.transfer_corner_height,
            'transfer_context': self.transfer_context,
            'staging_yaw180_pose': ((self.staging_store_transfer_target or {}).get('yaw180_pose')
                                   if self.transfer_context == 'staging_store' else None),
            'rotate_item_90': self.rotate_item_90,
            'keep_eef_perpendicular_to_pallet': self.keep_eef_perpendicular,
            'placement_corner_correction_xyz_m':
                self.placement_corner_correction,
            'joint_state_age_sec': (
                None if self.last_joint_state_time is None else
                time.monotonic() - self.last_joint_state_time),
        }, separators=(',', ':'))
        self.status_pub.publish(message)

    def refined_boxes_callback(self, message):
        boxes = {}
        for marker in message.markers:
            if (marker.ns == 'depth_refined_boxes' and
                    marker.action == Marker.ADD and
                    marker.header.frame_id == 'link_base'):
                boxes[int(marker.id)] = marker
        self.refined_boxes = boxes

    def stable_boxes_callback(self, message):
        boxes = {}
        for marker in message.markers:
            if (marker.ns == 'stable_depth_refined_boxes' and
                    marker.action == Marker.ADD and
                    marker.header.frame_id == 'link_base'):
                boxes[int(marker.id)] = marker
        self.stable_refined_boxes = boxes

    def pre_place_pose_callback(self, message):
        if message.header.frame_id != 'link_base':
            return
        self.pre_place_pose = message.pose
        self.pre_place_pose_time = time.monotonic()

    def pallet_status_callback(self, message):
        self.pallet_locked = message.data == 'LOCKED'

    def pallet_config_callback(self, message):
        height = float(message.data[18]) if len(message.data) >= 19 else 0.0
        if not math.isfinite(height) or height < 0:
            self.get_logger().error('invalid pallet transfer height')
            return
        self.transfer_corner_height = height if height > 0 else getattr(
            self, 'default_transfer_corner_height', getattr(self, 'transfer_corner_height', .47))
        if len(message.data) >= 12:
            self.place_target_xyz = tuple(
                float(value) / 1000.0 for value in message.data[9:12])
        if len(message.data) >= 13:
            self.rotate_item_90 = bool(message.data[12] > 0.5)
        if len(message.data) >= 14:
            self.keep_eef_perpendicular = bool(message.data[13] > 0.5)

    def planning_scene_status_callback(self, message):
        try:
            status = json.loads(message.data)
            size = status.get('attached_item_size_m')
            center = status.get('attached_item_center_in_tcp_m')
            orientation = status.get(
                'attached_item_orientation_in_tcp_xyzw')
            if (status.get('attached_item_id') and len(size) == 3 and
                    len(center) == 3 and len(orientation) == 4):
                values = [*size, *center, *orientation]
                if all(math.isfinite(float(value)) for value in values):
                    self.attached_item_geometry = {
                        'size': tuple(map(float, size)),
                        'center': tuple(map(float, center)),
                        'orientation': tuple(map(float, orientation)),
                    }
                    return
        except (TypeError, ValueError, json.JSONDecodeError):
            pass
        self.attached_item_geometry = None

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
        rotated = cls._quat_multiply(
            cls._quat_multiply(quaternion, (*vector, 0.0)),
            (-qx, -qy, -qz, qw))
        return rotated[:3]

    @staticmethod
    def _quat_inverse(quaternion):
        x, y, z, w = map(float, quaternion)
        norm_squared = x * x + y * y + z * z + w * w
        if norm_squared <= 1e-12:
            raise ValueError('cannot invert a zero-length quaternion')
        return (-x / norm_squared, -y / norm_squared,
                -z / norm_squared, w / norm_squared)

    def _perpendicular_tcp_orientation(self, object_q, unconstrained_tcp_q):
        item_yaw = -math.pi / 2.0 if self.rotate_item_90 else 0.0
        item_yaw_q = self._quaternion_from_rpy(0.0, 0.0, item_yaw)
        pallet_q = self._quat_multiply(
            object_q, self._quat_inverse(item_yaw_q))
        # Loading and guarded descent are along link_base Z.  Pallet
        # localization can contain a few degrees of roll/pitch noise; carrying
        # that noise into the path constraint makes the accurately vertical
        # post-pick start state invalid (the constraint tolerance is 3 deg).
        # Retain the localized pallet yaw, but level its Z axis to link_base so
        # "perpendicular" has the same meaning during pickup, transfer and
        # loading.
        px, py, pz, pw = pallet_q
        pallet_yaw = math.atan2(
            2.0 * (pw * pz + px * py),
            1.0 - 2.0 * (py * py + pz * pz))
        level_pallet_q = self._quaternion_from_rpy(
            0.0, 0.0, pallet_yaw)
        tcp_in_pallet = self._quat_multiply(
            self._quat_inverse(level_pallet_q), unconstrained_tcp_q)
        x, y, z, w = tcp_in_pallet
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z))
        downward_tcp_q = self._quaternion_from_rpy(math.pi, 0.0, yaw)
        return self._quat_multiply(level_pallet_q, downward_tcp_q)

    def _object_corner_to_tcp_pose(self, corner_pose):
        if self.attached_item_geometry is None:
            raise ValueError('attached-item grasp geometry is unavailable')
        geometry = self.attached_item_geometry
        object_q = (
            float(corner_pose.orientation.x),
            float(corner_pose.orientation.y),
            float(corner_pose.orientation.z),
            float(corner_pose.orientation.w))
        relative_q = geometry['orientation']
        tcp_q = self._quat_multiply(
            object_q,
            (-relative_q[0], -relative_q[1], -relative_q[2], relative_q[3]))
        if self.keep_eef_perpendicular:
            tcp_q = self._perpendicular_tcp_orientation(object_q, tcp_q)
        half_size = tuple(value / 2.0 for value in geometry['size'])
        corner = (float(corner_pose.position.x),
                  float(corner_pose.position.y),
                  float(corner_pose.position.z))
        center_offset = self._quat_rotate(half_size, object_q)
        object_center = tuple(
            corner[index] + center_offset[index] for index in range(3))
        tcp_to_center = self._quat_rotate(geometry['center'], tcp_q)
        tcp_position = tuple(
            object_center[index] - tcp_to_center[index] for index in range(3))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = tcp_position
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = tcp_q
        return pose

    def plan_pre_place_callback(self, _request, response):
        return self._plan_place_pose(response, transfer=False)

    def plan_transfer_callback(self, _request, response):
        if self.transfer_context == 'staging_store':
            return self._plan_cached_staging_transfer(response)
        return self._plan_place_pose(response, transfer=True)

    def prepare_transfer_callback(self, _request, response):
        """Cache a validated transfer target without invoking a planner."""
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Busy in state {self.state}'
            return response
        if not self._require_fresh_joint_state(response, 'prepare transfer'):
            return response
        if not self.pallet_locked:
            response.message = 'pallet pose is not LOCKED'
            return response
        if (self.pre_place_pose is None or self.pre_place_pose_time is None or
                time.monotonic() - self.pre_place_pose_time > 1.0):
            response.message = 'fresh pallet-relative pre-place pose is unavailable'
            return response
        try:
            self._calculate_place_poses()
            pose = self.transfer_tcp_pose
        except (TypeError, ValueError) as exc:
            response.message = str(exc)
            return response
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(float(value)) for value in values):
            response.message = 'pallet-relative transfer pose is invalid'
            return response
        self.operation_id += 1
        self.target = 'transfer'
        self.transfer_context = 'pallet'
        self.fault = ''
        self.cancel_requested = self.pause_requested = False
        self._set_state(self.PREPARED)
        response.success = True
        response.message = (
            'Prepared direct-joint transfer target at '
            f'[{pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f}] m; operation_id={self.operation_id}')
        return response

    def staging_store_transfer_target_callback(self, message):
        """Cache a raised staging transfer pose and its vertical pre-place Z."""
        if len(message.data) < 9:
            self.get_logger().warning(
                'ignored incomplete staging store transfer target')
            return
        values = tuple(map(float, message.data[:10]))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                'ignored non-finite staging store transfer target')
            return
        slot = int(round(values[0]))
        quaternion = values[4:8]
        quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
        if slot < 0 or slot >= 6 or quaternion_norm < 1e-6:
            self.get_logger().warning(
                'ignored invalid staging store slot or TCP quaternion')
            return
        if values[8] >= values[3] - 0.005:
            self.get_logger().warning(
                'ignored staging pre-place target without vertical clearance')
            return
        self.staging_store_transfer_target = {
            'slot': slot,
            'transfer_xyz': values[1:4],
            'quaternion': tuple(value / quaternion_norm
                                for value in quaternion),
            'pre_place_z': values[8],
            'corner_clearance_z': values[9] if len(values) > 9 else None,
            'received_at': time.monotonic(),
        }
        if len(message.data) == 17:
            alternate = list(map(float, message.data[10:17]))
            norm = math.sqrt(sum(v*v for v in alternate[3:]))
            if all(math.isfinite(v) for v in alternate) and abs(norm-1.) < 1e-5:
                self.staging_store_transfer_target['yaw180_pose'] = alternate

    def prepare_staging_store_transfer_callback(self, _request, response):
        """Expose a staging waypoint through the normal transfer interface."""
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Busy in state {self.state}'
            return response
        if not self._require_fresh_joint_state(
                response, 'prepare staging store transfer'):
            return response
        target = self.staging_store_transfer_target
        if target is None or time.monotonic() - target['received_at'] > 2.0:
            response.message = 'fresh staging store transfer target is unavailable'
            return response
        transfer = Pose()
        (transfer.position.x, transfer.position.y, transfer.position.z) = (
            target['transfer_xyz'])
        (transfer.orientation.x, transfer.orientation.y,
         transfer.orientation.z, transfer.orientation.w) = target['quaternion']
        pre_place = Pose()
        pre_place.position.x = transfer.position.x
        pre_place.position.y = transfer.position.y
        pre_place.position.z = float(target['pre_place_z'])
        pre_place.orientation = transfer.orientation
        self.transfer_tcp_pose = transfer
        self.pre_place_tcp_pose = pre_place
        self.nominal_transfer_corner_z = target.get('corner_clearance_z')
        self.operation_id += 1
        self.target = 'transfer'
        self.transfer_context = 'staging_store'
        self.fault = ''
        self.cancel_requested = self.pause_requested = False
        self._set_state(self.PREPARED)
        response.success = True
        response.message = (
            f'Prepared staging slot {target["slot"]} transfer at '
            f'[{transfer.position.x:.3f}, {transfer.position.y:.3f}, '
            f'{transfer.position.z:.3f}] m; vertical pre-place Z '
            f'{pre_place.position.z:.3f} m; operation_id={self.operation_id}')
        return response

    def _plan_cached_staging_transfer(self, response):
        """Plan a MoveIt fallback for a prepared staging-store target."""
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Busy in state {self.state}'
            return response
        if not self._require_fresh_joint_state(
                response, 'plan staging store transfer'):
            return response
        if self.transfer_tcp_pose is None or self.pre_place_tcp_pose is None:
            response.message = 'prepared staging transfer poses are unavailable'
            return response
        planner = (
            self.constrained_pose_plan_client
            if self.keep_eef_perpendicular else self.pose_plan_client)
        if not planner.service_is_ready():
            response.message = 'xArm staging transfer planning service is unavailable'
            return response
        pose = self.transfer_tcp_pose
        self.operation_id += 1
        request_id = self.operation_id
        self.target = 'transfer'
        self.planned_pregrasp = None
        self._clear_pregrasp_snapshot()
        self.cancel_requested = self.pause_requested = False
        self._set_state(self.PLANNING)
        request = PlanPose.Request()
        request.target = pose
        future = planner.call_async(request)
        future.add_done_callback(
            lambda completed: self._plan_completed(request_id, completed))
        response.success = True
        response.message = (
            'Planning collision-aware staging transfer fallback at '
            f'[{pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f}] m; operation_id={request_id}')
        return response

    def _calculate_place_poses(self):
        corner_pose = Pose()
        corner_pose.position.x = self.pre_place_pose.position.x
        corner_pose.position.y = self.pre_place_pose.position.y
        corner_pose.position.z = self.pre_place_pose.position.z
        corner_pose.orientation = self.pre_place_pose.orientation
        if self.attached_item_geometry is None:
            raise ValueError('attached-item grasp geometry is unavailable')
        if self.place_target_xyz is None:
            raise ValueError('pallet-frame place target XYZ is unavailable')
        object_q = (
            float(corner_pose.orientation.x),
            float(corner_pose.orientation.y),
            float(corner_pose.orientation.z),
            float(corner_pose.orientation.w))
        item_yaw = -math.pi / 2.0 if self.rotate_item_90 else 0.0
        pallet_q = self._quat_multiply(
            object_q,
            self._quat_inverse(
                self._quaternion_from_rpy(0.0, 0.0, item_yaw)))
        local_pre_place = (
            self.place_target_xyz[0],
            self.place_target_xyz[1],
            self.place_target_xyz[2] + self.pre_place_clearance)
        pre_place_offset = self._quat_rotate(local_pre_place, pallet_q)
        pallet_origin = tuple(
            float(getattr(self.pre_place_pose.position, axis)) -
            pre_place_offset[index]
            for index, axis in enumerate(('x', 'y', 'z')))
        transfer_offset = self._quat_rotate(
            (self.place_target_xyz[0], self.place_target_xyz[1],
             self.transfer_corner_height),
            pallet_q)
        transfer_reference_corner = tuple(
            pallet_origin[index] + transfer_offset[index]
            for index in range(3))
        self.placement_corner_correction = (0.0, 0.0, 0.0)
        if self.rotate_item_90:
            # pre_place_xyz always denotes the minimum pallet-X/minimum
            # pallet-Y/bottom corner of the final footprint. A clockwise
            # rotation about the original object corner makes the footprint
            # extend in negative pallet Y. Shift the original corner by one
            # object X dimension along positive pallet Y so the configured
            # point remains the rotated footprint's requested corner.
            object_x = float(self.attached_item_geometry['size'][0])
            correction = self._quat_rotate((-object_x, 0.0, 0.0), object_q)
            self.placement_corner_correction = correction
            corner_pose.position.x += correction[0]
            corner_pose.position.y += correction[1]
            corner_pose.position.z += correction[2]
        self.pre_place_tcp_pose = self._object_corner_to_tcp_pose(corner_pose)
        transfer_corner = Pose()
        transfer_corner.position.x = (
            transfer_reference_corner[0] + self.placement_corner_correction[0])
        transfer_corner.position.y = (
            transfer_reference_corner[1] + self.placement_corner_correction[1])
        transfer_corner.position.z = (
            transfer_reference_corner[2] + self.placement_corner_correction[2])
        transfer_corner.orientation = corner_pose.orientation
        self.nominal_transfer_corner_z = transfer_corner.position.z
        self.transfer_tcp_pose = self._object_corner_to_tcp_pose(transfer_corner)

    def _plan_place_pose(self, response, transfer):
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Busy in state {self.state}'
            return response
        label = 'transfer' if transfer else 'pre-place'
        if not self._require_fresh_joint_state(response, f'plan {label}'):
            return response
        if not self.pallet_locked:
            response.message = 'pallet pose is not LOCKED'
            return response
        if (self.pre_place_pose is None or self.pre_place_pose_time is None or
                time.monotonic() - self.pre_place_pose_time > 1.0):
            response.message = 'fresh pallet-relative pre-place pose is unavailable'
            return response
        orientation_locked_transfer = transfer and self.keep_eef_perpendicular
        planner = (
            self.constrained_pose_plan_client
            if orientation_locked_transfer else self.pose_plan_client)
        if not planner.service_is_ready():
            planner_name = (
                'orientation-constrained pose'
                if orientation_locked_transfer else 'pose')
            response.message = f'xArm {planner_name} planning service is unavailable'
            return response
        try:
            self._calculate_place_poses()
            pose = self.transfer_tcp_pose if transfer else self.pre_place_tcp_pose
        except (TypeError, ValueError) as exc:
            response.message = str(exc)
            return response
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(float(value)) for value in values):
            response.message = 'pallet-relative pre-place pose is invalid'
            return response
        self.operation_id += 1
        request_id = self.operation_id
        self.target = 'transfer' if transfer else 'pre_place'
        self.planned_pregrasp = None
        self._clear_pregrasp_snapshot()
        self.cancel_requested = self.pause_requested = False
        self._set_state(self.PLANNING)
        # A generic pose plan constrains only the endpoint and may tilt the
        # carried object between pickup and transfer.  The dedicated planner
        # applies a MoveIt path OrientationConstraint to link_tcp: roll and
        # pitch stay near the target while yaw remains free for routing.
        request = PlanPose.Request()
        request.target = pose
        future = planner.call_async(request)
        future.add_done_callback(
            lambda completed: self._plan_completed(request_id, completed))
        response.success = True
        response.message = (
            f'Planning TCP for pallet-relative {label}'
            f'{" with a roll/pitch path constraint" if orientation_locked_transfer else ""} at '
            f'[{pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f}] m; operation_id={request_id}')
        return response

    @staticmethod
    def _quaternion_from_rpy(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _select_refined_box(self):
        stable_boxes = getattr(self, 'stable_refined_boxes', {})
        if self.target_box_id >= 0:
            stable = stable_boxes.get(self.target_box_id)
            if stable is not None and self._marker_is_fresh(stable):
                return stable
            marker = self.refined_boxes.get(self.target_box_id)
            if marker is None:
                raise ValueError(f'refined box id {self.target_box_id} is unavailable')
            return marker
        fresh_stable = [
            marker for marker in stable_boxes.values()
            if self._marker_is_fresh(marker)]
        if len(fresh_stable) == 1:
            return fresh_stable[0]
        if len(self.refined_boxes) != 1:
            raise ValueError(
                'exactly one refined box is required; set target_box_id to select one')
        return next(iter(self.refined_boxes.values()))

    def _marker_is_fresh(self, marker):
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(
            marker.header.stamp)).nanoseconds * 1e-9
        return -0.05 <= age <= self.max_detection_age

    def _make_pregrasp_pose(self, box):
        stamp = box.header.stamp
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds * 1e-9
        if age < -0.05 or age > self.max_detection_age:
            raise ValueError(f'refined box detection is stale ({age:.3f} s)')
        dimensions = (float(box.scale.x), float(box.scale.y), float(box.scale.z))
        if any(not math.isfinite(value) or value <= 0.01 for value in dimensions):
            raise ValueError(f'refined box {box.id} has invalid dimensions {dimensions}')
        q = box.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = Pose()
        pose.position.x = float(box.pose.position.x)
        pose.position.y = float(box.pose.position.y)
        pose.position.z = float(box.pose.position.z + dimensions[2] / 2.0 +
                                self.pregrasp_clearance)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
            self._quaternion_from_rpy(math.pi, 0.0, yaw)
        xyz = (pose.position.x, pose.position.y, pose.position.z)
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError(f'pregrasp contains non-finite coordinates {xyz}')
        return pose

    def _box_yaw(self, box):
        q = box.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _start_pose_plan(self, pose, box_id, response):
        self.operation_id += 1
        request_id = self.operation_id
        self.target = f'pregrasp_box_{int(box_id)}'
        self.cancel_requested = False
        self.pause_requested = False
        self._publish_pregrasp_marker(pose, int(box_id))
        self._set_state(self.PLANNING)
        plan_request = PlanPose.Request()
        plan_request.target = pose
        future = self.pose_plan_client.call_async(plan_request)
        future.add_done_callback(
            lambda completed: self._plan_completed(request_id, completed))
        response.success = True
        response.message = (
            f'Planning pregrasp for box {int(box_id)} at '
            f'[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}] m; '
            f'operation_id={request_id}')
        return response

    def _start_straight_plan(self, pose, box_id, response):
        """Plan one collision-checked Cartesian segment to a target pose."""
        self.operation_id += 1
        request_id = self.operation_id
        # Keep the established semantic target name. Pickup validation keys
        # the verified snapshot to ``pregrasp_box_<id>`` regardless of which
        # planner produced the collision-checked trajectory.
        self.target = f'pregrasp_box_{int(box_id)}'
        self.cancel_requested = False
        self.pause_requested = False
        self._publish_pregrasp_marker(pose, int(box_id))
        self._set_state(self.PLANNING)
        plan_request = PlanSingleStraight.Request()
        plan_request.target = pose
        future = self.straight_plan_client.call_async(plan_request)
        future.add_done_callback(
            lambda completed: self._plan_completed(request_id, completed))
        response.success = True
        response.message = (
            f'Planning straight pregrasp for box {int(box_id)} at '
            f'[{pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f}] m; operation_id={request_id}')
        return response

    def _publish_pregrasp_marker(self, pose, box_id):
        marker = Marker()
        marker.header.frame_id = 'link_base'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pregrasp_target'
        marker.id = int(box_id)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        # RViz Arrow meshes point along their local X axis, whereas link_tcp
        # approaches along its local Z axis. Use explicit points so the marker
        # always depicts the physical downward approach direction.
        marker.pose.orientation.w = 1.0
        start = Point(x=pose.position.x, y=pose.position.y, z=pose.position.z)
        end = Point(x=pose.position.x, y=pose.position.y,
                    z=pose.position.z - 0.10)
        marker.points = [start, end]
        marker.scale.x, marker.scale.y, marker.scale.z = 0.012, 0.025, 0.035
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.8, 0.0, 0.95
        self.pregrasp_marker_pub.publish(marker)

    def plan_pregrasp_callback(self, request, response):
        del request
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Busy in state {self.state}'
            return response
        if not self._require_fresh_joint_state(response, 'plan pre-grasp'):
            return response
        if not self.pose_plan_client.service_is_ready():
            response.message = 'xArm pose planning service is unavailable'
            return response
        try:
            box = self._select_refined_box()
            pose = self._make_pregrasp_pose(box)
            yaw = self._box_yaw(box)
        except ValueError as exc:
            self._set_state(self.FAULT, str(exc))
            response.message = str(exc)
            return response
        self.planned_pregrasp = {
            'box_id': int(box.id),
            'x_m': float(box.pose.position.x),
            'y_m': float(box.pose.position.y),
            'center_z_m': float(box.pose.position.z),
            'size_x_m': float(box.scale.x),
            'size_y_m': float(box.scale.y),
            'size_z_m': float(box.scale.z),
            'top_z_m': float(box.pose.position.z + box.scale.z / 2.0),
            'pregrasp_z_m': float(pose.position.z),
            'yaw_rad': float(yaw),
            'planned_stamp_sec': self.get_clock().now().nanoseconds * 1e-9,
        }
        return self._start_pose_plan(pose, box.id, response)

    def staging_retrieve_target_callback(self, message):
        # [slot, contact-reference TCP xyz_m, release TCP rpy_rad,
        #  unrotated_item_size_xyz_m, clearance_m, object_yaw_rad,
        #  optional approach_tcp_z_m, optional pickup_source (0=pallet, 1=buffer)]
        if len(message.data) < 11:
            self.get_logger().warning('ignored incomplete staging retrieve target')
            return
        values = tuple(map(float, message.data[:21]))
        if len(values) not in (11, 12, 13, 14, 17, 21):
            self.staging_retrieve_target = None
            self.get_logger().warning('ignored incomplete recorded grasp transform')
            return
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning('ignored non-finite staging retrieve target')
            return
        target_id = int(round(values[0]))
        size = values[7:10]
        clearance = values[10]
        object_yaw = values[11] if len(message.data) >= 12 else 0.0
        approach_tcp_z = values[12] if len(message.data) >= 13 else None
        source_code = values[13] if len(values) >= 14 else 0.0
        if source_code not in (0.0, 1.0):
            self.staging_retrieve_target = None
            self.get_logger().warning('ignored unknown retrieval pickup source')
            return
        if target_id < 0 or any(value <= 0.0 for value in size):
            self.get_logger().warning('ignored invalid retrieval target or item size')
            return
        if not 0.005 <= clearance <= 0.100:
            self.get_logger().warning('ignored invalid staging pre-pick clearance')
            return
        pregrasp_z = values[3] + clearance
        if (approach_tcp_z is not None and
                approach_tcp_z < pregrasp_z + 0.010):
            self.get_logger().warning(
                'ignored retrieval approach below the pre-pick safety margin')
            return
        self.staging_retrieve_target = {
            'target_id': target_id,
            'contact_tcp_pose': values[1:7],
            'size': size,
            'clearance': clearance,
            'object_yaw': object_yaw,
            'approach_tcp_z': approach_tcp_z,
            'pickup_source': 'buffer' if source_code == 1.0 else 'pallet',
            'received_at': time.monotonic(),
        }
        if len(values) == 17:
            from .transport_alternatives import waypoint_candidates
            rail, candidate, direction = values[14:17]
            if (rail not in (0., 1.) or direction not in (-1., 1.) or
                    candidate != int(candidate) or not -1 <= candidate < len(waypoint_candidates())):
                self.staging_retrieve_target = None
                self.get_logger().warning('ignored invalid inspection route hint')
                return
            self.staging_retrieve_target['inspection_route'] = [int(rail), int(candidate), int(direction)]
        if len(values) == 21:
            if source_code != 1.0 or abs(sum(v*v for v in values[17:21])-1.) > 1e-3:
                self.staging_retrieve_target = None
                self.get_logger().warning('ignored invalid recorded buffer grasp transform')
                return
            self.staging_retrieve_target['recorded_grasp'] = list(values[14:21])

    def plan_staging_approach_callback(self, _request, response):
        """Plan the mandatory above-container waypoint for pallet retrieval."""
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Busy in state {self.state}'
            return response
        if not self._require_fresh_joint_state(
                response, 'plan staging approach'):
            return response
        if not self.pose_plan_client.service_is_ready():
            response.message = 'xArm pose planning service is unavailable'
            return response
        target = self.staging_retrieve_target
        if target is None or time.monotonic() - target['received_at'] > 2.0:
            response.message = 'fresh staging retrieve target is unavailable'
            return response
        approach_z = target.get('approach_tcp_z')
        if approach_z is None:
            response.message = 'above-container retrieval waypoint is unavailable'
            return response
        target_id = target['target_id']
        x, y, _contact_z, roll, pitch, yaw = target['contact_tcp_pose']
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = float(approach_z)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = self._quaternion_from_rpy(
            roll, pitch, yaw)
        # The overhead waypoint is not a verified grasp approach. Prevent a
        # restored snapshot from being refreshed when this first leg executes.
        self.planned_pregrasp = None
        return self._start_pose_plan(pose, 2000 + target_id, response)

    def prepare_pick_waypoints_callback(self, request, response):
        return self.plan_staging_pregrasp_callback(request, response, prepare_only=True)

    def _pick_path_status(self, message):
        try:
            self.pick_path_status = json.loads(message.data)
            self.pick_path_status_time = time.monotonic()
        except (TypeError, ValueError):
            pass

    def accept_pick_waypoints_callback(self, _request, response):
        status = self.pick_path_status
        if (self.state != self.PREPARED or self.transfer_context != 'known_pick' or
                not self.planned_pregrasp or time.monotonic()-self.pick_path_status_time > 1.0 or
                status.get('state') != 'SUCCEEDED' or not status.get('pick_path_completed') or
                status.get('operation_kind') != 'pick_approach' or
                status.get('pick_path_target_id') != self.operation_id):
            response.message = 'fresh verified pickup-path completion for this target is required'
            return response
        self._persist_pregrasp_snapshot()
        self._set_state(self.SUCCEEDED)
        response.success = True
        response.message = 'verified pickup approach accepted; pre-grasp is ready'
        return response

    def plan_staging_pregrasp_callback(self, _request, response, *, prepare_only=False):
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Busy in state {self.state}'
            return response
        if not self._require_fresh_joint_state(response, 'plan staging pre-pick'):
            return response
        target = self.staging_retrieve_target
        if target is None or time.monotonic() - target['received_at'] > 2.0:
            response.message = 'fresh staging retrieve target is unavailable'
            return response
        if prepare_only and (not self.pallet_locked or
                self.attached_item_geometry is not None or
                target.get('approach_tcp_z') is None):
            response.message = 'pickup path needs locked pallet, empty tool, and overhead clearance'
            return response
        target_id = target['target_id']
        x, y, contact_z, roll, pitch, yaw = target['contact_tcp_pose']
        size_x, size_y, size_z = target['size']
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = contact_z + target['clearance']
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = self._quaternion_from_rpy(
            roll, pitch, yaw)
        box_id = 1000 + target_id
        self.planned_pregrasp = {
            'box_id': box_id,
            'x_m': x,
            'y_m': y,
            'center_z_m': contact_z - size_z / 2.0,
            'size_x_m': size_x,
            'size_y_m': size_y,
            'size_z_m': size_z,
            'top_z_m': contact_z,
            'pregrasp_z_m': pose.position.z,
            'yaw_rad': float(target['object_yaw']),
            'planned_stamp_sec': self.get_clock().now().nanoseconds * 1e-9,
            'retrieval_target_id': target_id,
            'pickup_source': target.get('pickup_source', 'pallet'),
        }
        if 'recorded_grasp' in target:
            self.planned_pregrasp['recorded_grasp'] = target['recorded_grasp']
        if 'inspection_route' in target:
            self.planned_pregrasp['inspection_route'] = target['inspection_route']
        if prepare_only:
            self._clear_pregrasp_snapshot()
            self.pre_place_tcp_pose = pose  # Shared executor endpoint: pre-pick, NOT contact.
            overhead = Pose()
            overhead.position.x, overhead.position.y = x, y
            overhead.position.z = float(target['approach_tcp_z'])
            overhead.orientation = pose.orientation
            self.transfer_tcp_pose = overhead
            self.nominal_transfer_corner_z = float(target['approach_tcp_z'])
            self.transfer_context = 'known_pick'
            self.operation_id += 1
            self.target = f'pregrasp_box_{box_id}'
            self.cancel_requested = self.pause_requested = False
            self._publish_pregrasp_marker(pose, box_id)
            self._set_state(self.PREPARED)
            response.success = True
            response.message = f'Prepared known-item pickup path; operation_id={self.operation_id}'
            return response
        if target.get('approach_tcp_z') is not None:
            if not self.straight_plan_client.service_is_ready():
                response.message = (
                    'xArm Cartesian straight planning service is unavailable')
                return response
            return self._start_straight_plan(pose, box_id, response)
        if not self.pose_plan_client.service_is_ready():
            response.message = 'xArm pose planning service is unavailable'
            return response
        return self._start_pose_plan(pose, box_id, response)

    def _load_waypoint(self, name):
        try:
            with self.waypoint_file.open('r', encoding='utf-8') as stream:
                document = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f'cannot read waypoint file: {exc}')
        if not isinstance(document, dict) or document.get('format_version') != 1:
            raise ValueError('unsupported waypoint file format')
        waypoint = document.get('waypoints', {}).get(name)
        if not isinstance(waypoint, dict):
            raise ValueError(f'waypoint {name!r} is not saved')
        expected_names = [f'joint{i}' for i in range(1, 7)]
        if waypoint.get('joint_names') != expected_names:
            raise ValueError(f'waypoint {name!r} has unexpected joint order')
        positions = waypoint.get('positions_rad')
        if not isinstance(positions, list) or len(positions) != 6:
            raise ValueError(f'waypoint {name!r} must have six joint positions')
        try:
            positions = [float(value) for value in positions]
        except (TypeError, ValueError):
            raise ValueError(f'waypoint {name!r} contains invalid positions')
        return positions

    def plan_waypoint(self, name, response):
        del response  # A fresh response below avoids accidental stale fields.
        response = Trigger.Response()
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Busy in state {self.state}'
            return response
        if not self._require_fresh_joint_state(response, f'plan {name}'):
            return response
        if not self.plan_client.service_is_ready():
            response.message = 'xArm planning service is unavailable'
            return response
        try:
            positions = self._load_waypoint(name)
        except ValueError as exc:
            self._set_state(self.FAULT, str(exc))
            response.message = str(exc)
            return response

        if name == 'observation':
            joint_names = [f'joint{i}' for i in range(1, len(positions) + 1)]
            if all(joint in self.latest_joint_positions for joint in joint_names):
                error = max(abs(self.latest_joint_positions[joint] - target)
                            for joint, target in zip(joint_names, positions))
                if error <= self.observation_joint_tolerance:
                    self.operation_id += 1
                    self.target = name
                    self.planned_pregrasp = None
                    self._clear_pregrasp_snapshot()
                    self.cancel_requested = False
                    self.pause_requested = False
                    self._set_state(self.SUCCEEDED)
                    response.success = True
                    response.message = (
                        'Robot is already at observation pose '
                        f'(max joint error {error:.4f} rad)')
                    return response

        self.operation_id += 1
        request_id = self.operation_id
        self.target = name
        self.planned_pregrasp = None
        self._clear_pregrasp_snapshot()
        self.cancel_requested = False
        self.pause_requested = False
        self._set_state(self.PLANNING)
        request = PlanJoint.Request()
        request.target = positions
        future = self.plan_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._plan_completed(request_id, completed))
        response.success = True
        response.message = f'Planning {name}; operation_id={request_id}'
        return response

    def _plan_completed(self, request_id, future):
        if request_id != self.operation_id:
            return
        try:
            result = future.result()
        except Exception as exc:
            self._set_state(self.FAULT, f'planning service failed: {exc}')
            return
        if result is None or not result.success:
            self._set_state(self.FAULT, f'planning {self.target} failed')
            return
        if self.cancel_requested:
            self._set_state(self.PAUSED if self.pause_requested else self.IDLE)
            return
        self._set_state(self.PLANNED)

    def execute_callback(self, request, response):
        del request
        if self.state != self.PLANNED:
            response.message = f'Execution requires PLANNED state, got {self.state}'
            return response
        if not self._require_fresh_joint_state(response, 'execute trajectory'):
            return response
        if not self.exec_client.service_is_ready():
            response.message = 'xArm execution service is unavailable'
            return response
        request_id = self.operation_id
        execute_request = PlanExec.Request()
        execute_request.wait = True
        self._set_state(self.EXECUTING)
        future = self.exec_client.call_async(execute_request)
        future.add_done_callback(
            lambda completed: self._execution_completed(request_id, completed))
        response.success = True
        response.message = (
            f'Executing {self.target}; operation_id={request_id}')
        return response

    def _execution_completed(self, request_id, future):
        if request_id != self.operation_id:
            return
        try:
            result = future.result()
        except Exception as exc:
            if self.cancel_requested:
                self._set_state(
                    self.PAUSED if self.pause_requested else self.IDLE)
            else:
                self._set_state(self.FAULT, f'execution service failed: {exc}')
            return
        if self.cancel_requested:
            self._set_state(self.PAUSED if self.pause_requested else self.IDLE)
        elif result is not None and result.success:
            self._persist_pregrasp_snapshot()
            self._set_state(self.SUCCEEDED)
        else:
            self._set_state(self.FAULT, f'execution of {self.target} failed')

    def _request_cancel(self, pause, response):
        if self.state not in (self.PLANNING, self.PLANNED, self.EXECUTING):
            response.message = f'Nothing to stop in state {self.state}'
            return response
        self.cancel_requested = True
        self.pause_requested = pause
        if self.state == self.PLANNING:
            self._set_state(self.CANCELING)
            response.success = True
            response.message = 'Planning result will be discarded when ready'
            return response
        if self.state == self.PLANNED:
            self.operation_id += 1
            self._set_state(self.PAUSED if pause else self.IDLE)
            response.success = True
            response.message = 'Pending plan discarded'
            return response
        if not self.cancel_client.service_is_ready():
            self._set_state(
                self.FAULT, 'trajectory cancellation service is unavailable')
            response.message = self.fault
            return response

        self._set_state(self.CANCELING)
        cancel_request = CancelGoal.Request()
        # A zero goal UUID and zero timestamp requests cancellation of all goals
        # on this controller, per the ROS 2 action protocol.
        future = self.cancel_client.call_async(cancel_request)
        future.add_done_callback(self._cancel_completed)
        response.success = True
        response.message = 'Trajectory cancellation requested'
        return response

    def _cancel_completed(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._set_state(self.FAULT, f'cancel service failed: {exc}')
            return
        if result is None or result.return_code != CancelGoal.Response.ERROR_NONE:
            code = None if result is None else result.return_code
            self._set_state(self.FAULT, f'trajectory cancel rejected: {code}')
            return
        self._set_state(self.PAUSED if self.pause_requested else self.IDLE)

    def cancel_callback(self, request, response):
        del request
        return self._request_cancel(False, response)

    def pause_callback(self, request, response):
        if request.data:
            return self._request_cancel(True, response)
        response.success = False
        response.message = 'Use /motion_coordinator/resume to replan safely'
        return response

    def resume_callback(self, request, response):
        del request
        if self.state != self.PAUSED or not self.target:
            response.message = f'Resume requires PAUSED state, got {self.state}'
            return response
        # Resuming means replanning from the actual stopped state. It never
        # continues a stale, partially executed trajectory.
        return self.plan_waypoint(self.target, response)

    def reset_callback(self, request, response):
        del request
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Cannot reset while state is {self.state}'
            return response
        self.operation_id += 1
        self.target = None
        self.transfer_context = ''
        self.planned_pregrasp = None
        self._clear_pregrasp_snapshot()
        self.cancel_requested = False
        self.pause_requested = False
        self._set_state(self.IDLE)
        response.success = True
        response.message = 'Coordinator reset to IDLE'
        return response

    def accept_direct_transfer_callback(self, request, response):
        """Acknowledge a verified direct-service replacement for MoveIt transfer."""
        del request
        if self.state not in (self.FAULT, self.PREPARED) or self.target != 'transfer':
            response.message = (
                'Direct transfer acknowledgement requires a failed transfer '
                'or prepared target; '
                f'got state={self.state}, target={self.target}')
            return response
        if self.pre_place_tcp_pose is None or self.transfer_tcp_pose is None:
            response.message = (
                'Direct transfer acknowledgement requires cached pre-place '
                'and transfer TCP poses')
            return response
        # Preserve operation_id and both cached poses. The direct-motion
        # supervisor already verified the reached TCP; downstream loading and
        # Servo must continue to reference this same placement operation.
        self.cancel_requested = False
        self.pause_requested = False
        self._set_state(self.SUCCEEDED)
        response.success = True
        response.message = (
            'Verified direct transfer accepted; coordinator state is SUCCEEDED')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
