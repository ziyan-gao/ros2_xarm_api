import json
import math
from pathlib import Path
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Point, Pose, PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
from xarm_msgs.srv import PlanExec, PlanJoint, PlanPose
import yaml


class MotionCoordinator(Node):
    """Stateful, named-waypoint front end for the xArm MoveIt planner."""

    IDLE = 'IDLE'
    PLANNING = 'PLANNING'
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
        self.declare_parameter('target_box_id', -1)
        self.declare_parameter('pregrasp_clearance_m', 0.03)
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_state_timeout_sec', 0.5)
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
        if self.pregrasp_clearance <= 0.0:
            raise ValueError('pregrasp_clearance_m must be positive')
        if self.joint_state_timeout <= 0.0:
            raise ValueError('joint_state_timeout_sec must be positive')
        self.state = self.IDLE
        self.target = None
        self.fault = ''
        self.cancel_requested = False
        self.pause_requested = False
        self.operation_id = 0
        self.refined_boxes = {}
        self.pre_place_pose = None
        self.pre_place_pose_time = None
        self.attached_item_geometry = None
        self.pallet_locked = False
        self.planned_pregrasp = None
        self.last_joint_state_time = None
        self._restore_pregrasp_snapshot()

        self.plan_client = self.create_client(
            PlanJoint, '/xarm_joint_plan')
        self.pose_plan_client = self.create_client(
            PlanPose, '/xarm_pose_plan')
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
            PoseStamped, '/pallet_localization/pre_place_pose',
            self.pre_place_pose_callback, 10)
        self.create_subscription(
            String, '/pallet_localization/status',
            self.pallet_status_callback, 10)
        self.create_subscription(
            String, '/planning_scene_obstacles/status',
            self.planning_scene_status_callback, 10)
        self.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 10)

        self.create_service(
            Trigger, '/motion_coordinator/plan_observation',
            lambda request, response: self.plan_waypoint(
                'observation', response))
        self.create_service(
            Trigger, '/motion_coordinator/plan_intermediate',
            lambda request, response: self.plan_waypoint(
                'intermediate', response))
        self.create_service(
            Trigger, '/motion_coordinator/plan_pre_place',
            self.plan_pre_place_callback)
        self.create_service(
            Trigger, '/motion_coordinator/plan_pregrasp',
            self.plan_pregrasp_callback)
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
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f'motion coordinator ready; waypoint_file={self.waypoint_file}')

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
        if not self.planned_pregrasp or not str(self.target).startswith(
                'pregrasp_box_'):
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

    def joint_state_callback(self, _message):
        self.last_joint_state_time = time.monotonic()

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
            'executor_ready': self.exec_client.service_is_ready(),
            'cancel_ready': self.cancel_client.service_is_ready(),
            'planned_pregrasp': self.planned_pregrasp,
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

    def pre_place_pose_callback(self, message):
        if message.header.frame_id != 'link_base':
            return
        self.pre_place_pose = message.pose
        self.pre_place_pose_time = time.monotonic()

    def pallet_status_callback(self, message):
        self.pallet_locked = message.data == 'LOCKED'

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
        if self.state in (self.PLANNING, self.EXECUTING, self.CANCELING):
            response.message = f'Busy in state {self.state}'
            return response
        if not self._require_fresh_joint_state(response, 'plan pre-place'):
            return response
        if not self.pallet_locked:
            response.message = 'pallet pose is not LOCKED'
            return response
        if (self.pre_place_pose is None or self.pre_place_pose_time is None or
                time.monotonic() - self.pre_place_pose_time > 1.0):
            response.message = 'fresh pallet-relative pre-place pose is unavailable'
            return response
        if not self.pose_plan_client.service_is_ready():
            response.message = 'xArm pose planning service is unavailable'
            return response
        try:
            pose = self._object_corner_to_tcp_pose(self.pre_place_pose)
        except ValueError as exc:
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
        self.target = 'pre_place'
        self.planned_pregrasp = None
        self._clear_pregrasp_snapshot()
        self.cancel_requested = self.pause_requested = False
        self._set_state(self.PLANNING)
        request = PlanPose.Request()
        request.target = pose
        future = self.pose_plan_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._plan_completed(request_id, completed))
        response.success = True
        response.message = (
            'Planning TCP for pallet-relative object-corner pre-place at '
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
        if self.target_box_id >= 0:
            marker = self.refined_boxes.get(self.target_box_id)
            if marker is None:
                raise ValueError(f'refined box id {self.target_box_id} is unavailable')
            return marker
        if len(self.refined_boxes) != 1:
            raise ValueError(
                'exactly one refined box is required; set target_box_id to select one')
        return next(iter(self.refined_boxes.values()))

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
        self.planned_pregrasp = None
        self._clear_pregrasp_snapshot()
        self.cancel_requested = False
        self.pause_requested = False
        self._set_state(self.IDLE)
        response.success = True
        response.message = 'Coordinator reset to IDLE'
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
