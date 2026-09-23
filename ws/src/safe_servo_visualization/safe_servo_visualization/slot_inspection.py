"""Guarded inspection before buffer retrieval; never triggers the gripper."""
import math
import time
from pathlib import Path

import numpy as np
import yaml
import rclpy
from moveit_msgs.srv import GetPositionFK, GetCartesianPath
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image, JointState
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from .slot_pose_estimation import inspection_position, estimate_slot_top, stable_slot_pose
from .transport_path import pick_waypoints, pickup_needs_observation, grid_pick_waypoints
from .transport_alternatives import waypoint_candidates
from .waypoint_search import GRID_SECONDS, next_grid


def configured_slot_inspection(config_path, fallback):
    """Policy YAML wins when present; legacy standalone launch keeps its flag."""
    if not config_path:
        return fallback
    with Path(config_path).expanduser().open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('slot inspection policy config must be a YAML mapping')
    enabled = config.get('slot_inspection_enabled', fallback)
    if not isinstance(enabled, bool):
        raise ValueError('slot_inspection_enabled must be a YAML boolean (true/false)')
    return enabled


class SlotInspection:
    INSPECTION_FK = 'INSPECTION_FK'
    INSPECTION_MOVE = 'INSPECTION_MOVE'
    INSPECTION_DEPTH = 'INSPECTION_DEPTH'
    INSPECTION_CHECK = 'INSPECTION_CHECK'

    def _init_inspection(self):
        for name, default in (
                ('slot_inspection_enabled', True),
                ('slot_inspection_policy_config_path', ''),
                ('slot_inspection_backoff_m', .100),
                ('slot_inspection_timeout_sec', 15.),
                ('slot_inspection_camera_frame', 'camera_color_optical_frame'),
                ('slot_inspection_waypoint_file', '/workspace/config/taught_waypoints.yaml')):
            self.declare_parameter(name, default)
        self.inspection_enabled = bool(self.get_parameter('slot_inspection_enabled').value)
        policy_config = str(self.get_parameter('slot_inspection_policy_config_path').value)
        self.inspection_enabled = configured_slot_inspection(policy_config, self.inspection_enabled)
        # Keep ROS parameter introspection aligned with the effective YAML value.
        result = self.set_parameters([rclpy.parameter.Parameter(
            'slot_inspection_enabled', value=self.inspection_enabled)])[0]
        if not result.successful:
            raise ValueError('could not configure slot inspection: ' + result.reason)
        self.get_logger().info(
            f'slot inspection enabled={self.inspection_enabled}; '
            f'policy config={policy_config or "none (ROS parameter)"}; '
            'disabled inspection uses the saved slot pickup pose')
        self.inspection_backoff = float(self.get_parameter('slot_inspection_backoff_m').value)
        inspection_position((0, 0, 0), 0, (1., 0., 0.), self.inspection_backoff)
        self.inspection_timeout = float(self.get_parameter('slot_inspection_timeout_sec').value)
        if not math.isfinite(self.inspection_timeout) or self.inspection_timeout <= 0:
            raise ValueError('slot inspection timeout must be positive')
        self.inspection_camera = self.get_parameter('slot_inspection_camera_frame').value
        self.inspection_waypoint_file = self.get_parameter('slot_inspection_waypoint_file').value
        self.inspection_fk = self.create_client(GetPositionFK, '/compute_fk')
        self.inspection_cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.inspection_k = None
        self.inspection_generation = 0
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
                                 self._inspection_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw',
                                 self._inspection_depth, qos_profile_sensor_data)

    def _inspection_info(self, msg):
        self.inspection_k = np.array(msg.k).reshape(3, 3)

    def _rotation(self, q):
        return np.array([self._quat_rotate(axis, q) for axis in np.eye(3)]).T

    def _inspection_joint_state(self):
        positions = tuple(map(float, self._fresh_joint_seed()))
        names = list(self.arm_joint_names)
        if (len(positions) != len(names) or not names or
                len(set(names)) != len(names) or
                not all(math.isfinite(value) for value in positions)):
            raise ValueError('invalid inspection joint seed')
        # _fresh_joint_seed returns an ordered tuple, not a ROS message.
        return JointState(name=names, position=list(positions))

    def _begin_slot_inspection(self):
        self.inspection_generation += 1
        self.inspection_yaw_flip = False
        self.inspection_grid_index = -1
        self.inspection_grid_rail = 0
        self.inspection_grid_direction = 1
        self.inspection_grid_deadline = time.monotonic() + 5.
        generation = self.inspection_generation
        self.state = self.INSPECTION_FK
        self.inspection_started = time.monotonic()
        try:
            if not self.inspection_fk.service_is_ready():
                raise ValueError('observation FK service unavailable')
            with Path(self.inspection_waypoint_file).open() as stream:
                observation = yaml.safe_load(stream)['waypoints']['observation']
            req = GetPositionFK.Request()
            req.header.frame_id = self.base_frame
            req.fk_link_names = [self.ik_link_name]
            req.robot_state.joint_state.name = observation['joint_names']
            req.robot_state.joint_state.position = observation['positions_rad']
            self.inspection_fk.call_async(req).add_done_callback(
                lambda future: self._inspection_fk_done(future, generation))
        except Exception as exc:
            self._fault(f'slot inspection unavailable: {exc}')

    def _inspection_fk_done(self, future, generation):
        if self.state != self.INSPECTION_FK or generation != self.inspection_generation:
            return
        try:
            result = future.result()
            if result.error_code.val != 1 or not result.pose_stamped:
                raise ValueError('cannot compute observation TCP height')
            self.inspection_observation_z = result.pose_stamped[0].pose.position.z
            position = result.pose_stamped[0].pose.position
            self.inspection_observation_xyz = (position.x, position.y, position.z)
            self._build_inspection_candidate()
        except Exception as exc:
            self._fault(f'slot inspection planning failed: {exc}')

    def _build_inspection_candidate(self):
        try:
            self.inspection_generation += 1
            self.inspection_start_future = None
            record = self.occupied[self.active_slot]
            pose = list(record['release_tcp_pose'])
            if self.inspection_yaw_flip:
                pose[5] = (pose[5] + 2*math.pi) % (2*math.pi) - math.pi
            q = self._quaternion_from_rpy(*pose[3:])
            transform = self.tf_buffer.lookup_transform(
                self.ik_link_name, self.inspection_camera, rclpy.time.Time()).transform
            translation = transform.translation
            camera_offset_base = self._quat_rotate(
                (translation.x, translation.y, translation.z), q)
            xyz = inspection_position(np.array(pose[:3])/1000,
                                      self.inspection_observation_z,
                                      camera_offset_base, self.inspection_backoff)
            self.get_logger().info(
                f'slot inspection XY backoff={self.inspection_backoff*1000:.1f} mm; '
                f'target XYZ=({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}) m; '
                'Z fixed at observation TCP height')
            # The local inspection endpoint may be below the container travel
            # floor. Only the overhead crossing must remain above that floor.
            safe_z = self._pallet_origin_z() + self.transfer_item_bottom_above_pallet
            local_min_z = self._retrieval_contact_reference_z(record) + self.clearance
            if not math.isfinite(safe_z) or not math.isfinite(local_min_z):
                raise ValueError('invalid inspection clearance height')
            if xyz[2] < local_min_z:
                raise ValueError(
                    f'inspection endpoint Z={xyz[2]:.4f} m below slot-item '
                    f'clearance Z={local_min_z:.4f} m')
            self.inspection_approach_z = max(safe_z, float(xyz[2]) + .02)
            self.inspection_target = (*xyz, *pose[3:])
            self.inspection_q = q
            self.inspection_target_message = Float64MultiArray(data=[float(self.active_slot),
                float(xyz[0]), float(xyz[1]), float(xyz[2]-self.clearance),
                *pose[3:], *record['size'], self.clearance,
                float(record.get('object_yaw', 0)), self.inspection_approach_z, 1.])
            self.state = self.INSPECTION_CHECK
            self.inspection_started = time.monotonic()
            self.inspection_grid_deadline = self.inspection_started + GRID_SECONDS
            seed = self._inspection_joint_state()
            self.inspection_seed = seed
            request = GetPositionFK.Request()
            request.header.frame_id = self.base_frame
            request.fk_link_names = [self.ik_link_name]
            request.robot_state.joint_state = seed
            generation = self.inspection_generation
            self.inspection_fk.call_async(request).add_done_callback(
                lambda f: self._inspection_start_fk(f, generation))
        except Exception as exc:
            self._fault(f'slot inspection candidate failed: {exc}')

    def _inspection_start_fk(self, future, generation):
        if self.state != self.INSPECTION_CHECK or generation != self.inspection_generation:
            return
        try:
            result = future.result()
            if result.error_code.val != 1 or not result.pose_stamped:
                raise ValueError('current TCP FK failed')
            p = result.pose_stamped[0].pose
            self.inspection_start_future = future
            cross_area = pickup_needs_observation(
                (p.position.x, p.position.y, p.position.z), self.inspection_target[:3], 'buffer')
            self.inspection_cross_area = cross_area
            samples, _, _ = pick_waypoints(
                (p.position.x, p.position.y, p.position.z),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
                self.inspection_target[:3], self.inspection_q,
                self.inspection_approach_z,
                observation_xyz=self.inspection_observation_xyz if cross_area else None)
            if cross_area and getattr(self, 'inspection_grid_index', -1) >= 0:
                samples, _, _ = grid_pick_waypoints(
                    (p.position.x,p.position.y,p.position.z),
                    (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w),
                    self.inspection_target[:3], self.inspection_q, self.inspection_approach_z,
                    self.inspection_observation_xyz, waypoint_candidates()[self.inspection_grid_index],
                    rail_x=-.350 if getattr(self, 'inspection_grid_rail', 0) else None,
                    yaw_direction=getattr(self, 'inspection_grid_direction', 1))
            if not self.inspection_cartesian.service_is_ready():
                raise ValueError('Cartesian preflight service unavailable')
            request = GetCartesianPath.Request()
            request.header.frame_id = self.base_frame
            request.group_name = self.planning_group
            request.link_name = self.ik_link_name
            request.start_state.joint_state = self.inspection_seed
            request.max_step = .005
            request.avoid_collisions = True
            for xyz, q in samples:
                point = Pose()
                point.position.x, point.position.y, point.position.z = map(float, xyz)
                (point.orientation.x, point.orientation.y, point.orientation.z,
                 point.orientation.w) = map(float, q)
                request.waypoints.append(point)
            self.inspection_request_id = getattr(self, 'inspection_request_id', 0)+1
            request_id = self.inspection_request_id
            self.inspection_cartesian.call_async(request).add_done_callback(
                lambda f: self._inspection_checked(f, generation)
                if request_id == self.inspection_request_id else None)
        except Exception as exc:
            self._fault(f'slot inspection preflight failed: {exc}')

    def _inspection_checked(self, future, generation):
        if self.state != self.INSPECTION_CHECK or generation != self.inspection_generation:
            return
        try:
            result = future.result()
            if (result is None or result.error_code.val != 1 or
                    not math.isfinite(result.fraction) or result.fraction < 1-1e-6 or
                    not result.solution.joint_trajectory.points):
                index = getattr(self, 'inspection_grid_index', -1) + 1
                if (getattr(self, 'inspection_cross_area', False) and
                        time.monotonic() < getattr(self, 'inspection_grid_deadline', 0.) and
                        index < len(waypoint_candidates())):
                    self.inspection_grid_index = index
                    self._inspection_start_fk(self.inspection_start_future, generation)
                    return
                phase = next_grid(getattr(self, 'inspection_grid_rail', 0),
                                  getattr(self, 'inspection_grid_direction', 1))
                if getattr(self, 'inspection_cross_area', False) and phase is not None:
                    self.inspection_grid_rail, self.inspection_grid_direction = phase
                    self.inspection_grid_index = 0
                    self.inspection_grid_deadline = time.monotonic()+5.
                    self._inspection_start_fk(self.inspection_start_future, generation)
                    return
                if not self.inspection_yaw_flip:
                    self.inspection_yaw_flip = True
                    self.inspection_grid_index = -1
                    self.inspection_grid_rail = 0
                    self.inspection_grid_direction = 1
                    self.inspection_grid_deadline = time.monotonic()+5.
                    self.get_logger().warning('inspection preflight failed; trying yaw 180 degrees before any motion')
                    self._build_inspection_candidate()
                    return
                raise ValueError('original and yaw-180 inspection paths are infeasible')
            trajectory = result.solution.joint_trajectory
            if self.inspection_yaw_flip:
                wrist = self.arm_joint_names[-1]
                index = list(trajectory.joint_names).index(wrist)
                current = self.inspection_seed.position[list(self.inspection_seed.name).index(wrist)]
                values = [p.positions[index] for p in trajectory.points]
                # Moving away from zero is not a joint-limit violation. Keep
                # the existing intermediate-excursion guard, but do not reject
                # a valid endpoint merely because abs(end) > abs(start).
                finite = math.isfinite(current) and all(math.isfinite(v) for v in values)
                peak = max(map(abs, values)) if finite else float('nan')
                bound = max(abs(current), abs(values[-1]))+.05 if finite else float('nan')
                if not finite or peak > bound:
                    reason = 'non-finite wrist angles' if not finite else 'intermediate wrist excursion'
                    self.get_logger().warning(
                        f'yaw-180 wrist check rejected candidate: {reason}; '
                        f'{wrist} start={current:.6f}, end={values[-1]:.6f}, '
                        f'peak_abs={peak:.6f}, allowed_peak_abs={bound:.6f} rad; trying another route')
                    self._retry_inspection_candidate(generation)
                    return
            # Preserve the exact successful route selection. Execution must
            # recompute and validate it using its fresh measured joint state.
            self.inspection_target_message.data = list(self.inspection_target_message.data[:14]) + [
                float(getattr(self, 'inspection_grid_rail', 0)),
                float(getattr(self, 'inspection_grid_index', -1)),
                float(getattr(self, 'inspection_grid_direction', 1))]
            self.retrieve_target_pub.publish(self.inspection_target_message)
            self.expected_motion_operation_id = int(self.motion_status.get('operation_id', 0))+1
            self.pick_path.reset(self.expected_motion_operation_id)
            self.state = self.INSPECTION_MOVE
            self._one_shot_timer = self.create_timer(.15, self._request_retrieval_plan)
        except Exception as exc:
            self._fault(f'slot inspection planning failed: {exc}')

    def _retry_inspection_candidate(self, generation):
        from types import SimpleNamespace
        self.inspection_request_id = getattr(self, 'inspection_request_id', 0)+1
        self._inspection_checked(SimpleNamespace(result=lambda: None), generation)

    def _inspection_tick(self):
        if self.state == self.INSPECTION_CHECK:
            if time.monotonic() >= self.inspection_grid_deadline:
                if getattr(self, 'inspection_start_future', None) is None:
                    self._fault('slot inspection start FK timed out')
                else:
                    self._retry_inspection_candidate(self.inspection_generation)
        elif self.state == self.INSPECTION_FK:
            if time.monotonic()-self.inspection_started > 5:
                self._fault('slot inspection FK timed out')
        elif self.state == self.INSPECTION_MOVE:
            if self.pick_path.tick(self.motion_status, self.pickup_status):
                self.state = self.INSPECTION_DEPTH
                self.inspection_started = time.monotonic()
                self.inspection_after = self.get_clock().now().nanoseconds*1e-9 + .5
                self.inspection_last_stamp = self.inspection_after
                self.inspection_samples = []
                self.inspection_reason = 'waiting for fresh depth frames'
        elif self.state == self.INSPECTION_DEPTH:
            # Measurement quality is operator-recoverable. Stay stationary and
            # retry fresh frames until valid or explicitly aborted. This legacy
            # timeout parameter now controls reminder frequency, not failure.
            self.phase_started = time.monotonic()
            if time.monotonic()-self.inspection_started > self.inspection_timeout:
                self.get_logger().warning(
                    f'waiting for stable slot measurement: {self.inspection_reason}; '
                    '20 valid stable frames will automatically resume retrieval')
                self.inspection_started = time.monotonic()

    def _inspection_depth(self, msg):
        if self.state != self.INSPECTION_DEPTH or self.inspection_k is None:
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now = self.get_clock().now().nanoseconds*1e-9
        if stamp <= self.inspection_last_stamp or not 0 <= now-stamp <= .5:
            return
        self.inspection_last_stamp = stamp
        try:
            if msg.header.frame_id != self.inspection_camera:
                raise ValueError('aligned depth optical frame does not match configuration')
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.inspection_camera,
                                                 rclpy.time.Time.from_msg(msg.header.stamp)).transform
            tcp = self.tf_buffer.lookup_transform(self.base_frame, self.ik_link_name,
                                                  rclpy.time.Time.from_msg(msg.header.stamp)).transform
            if np.linalg.norm(np.array([tcp.translation.x, tcp.translation.y, tcp.translation.z])-
                              self.inspection_target[:3]) > .01:
                raise ValueError('TCP has not settled at inspection pose')
            actual_q = (tcp.rotation.x, tcp.rotation.y, tcp.rotation.z, tcp.rotation.w)
            if abs(float(np.dot(actual_q, self.inspection_q))) < math.cos(math.radians(2)/2):
                raise ValueError('TCP inspection orientation has not settled')
            q = (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
            rotation = self._rotation(q)
            translation = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
            record = self.occupied[self.active_slot]
            slot = self.slots[self.active_slot]
            # Check the real top-face footprint, without an artificial metric
            # padding or image-border margin. Actual out-of-frame corners still
            # fail visibility; depth quality and pose stability are checked below.
            saved = record['release_tcp_pose']
            saved_q = self._quaternion_from_rpy(*saved[3:])
            expected_center = np.array(saved[:3])/1000 + self._quat_rotate(record['center'], saved_q)
            object_q = self._quaternion_from_rpy(0., 0., record['object_yaw'])
            corners = np.array([expected_center + self._quat_rotate((x, y, record['size'][2]/2), object_q)
                for x in (-record['size'][0]/2, record['size'][0]/2)
                for y in (-record['size'][1]/2, record['size'][1]/2)])
            camera = (corners-translation) @ rotation
            pixels = camera @ self.inspection_k.T
            pixels = pixels[:, :2]/pixels[:, 2:3]
            if (np.any(camera[:, 2] <= .05) or np.any(pixels < 0) or
                    np.any(pixels[:, 0] >= msg.width) or np.any(pixels[:, 1] >= msg.height)):
                raise ValueError('expected item is not fully visible; adjust camera backoff')
            if msg.encoding not in ('16UC1', 'mono16', '32FC1'):
                raise ValueError('unsupported depth encoding')
            dtype = ('>' if msg.is_bigendian else '<') + ('f4' if msg.encoding == '32FC1' else 'u2')
            unit = 1. if msg.encoding == '32FC1' else .001
            raw = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step//np.dtype(dtype).itemsize)
            z = raw[:, :msg.width][::2, ::2].astype(float)*unit
            v, u = np.mgrid[0:msg.height:2, 0:msg.width:2]
            k = self.inspection_k
            points = np.stack(((u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z), axis=-1).reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > .05)]
            estimate = estimate_slot_top(points @ rotation.T + translation, slot,
                self.slot_size, record['size'], record['object_yaw'], check_boundary=False,
                require_footprint_match=False)
            self.inspection_samples.append(estimate)
            self.inspection_samples = self.inspection_samples[-20:]
            stable = stable_slot_pose(self.inspection_samples)
            self.inspection_reason = f'{len(self.inspection_samples)}/20 frames; waiting for stable pose'
        except Exception as exc:
            self.inspection_samples = []
            self.inspection_reason = str(exc)
            return
        if stable is not None:
            self._accept_stable_inspection(stable, expected_center)

    def _accept_stable_inspection(self, stable, expected_center):
        # Slot edges are inventory boundaries, not physical walls. A stable
        # footprint may extend beyond them; retain the item-association guard.
        if np.linalg.norm(stable[:2]-expected_center[:2]) > .04:
            self.inspection_reason = (
                '20-frame pose is stable but rejected: slot item shifted more '
                'than 40 mm; reconcile manually; continuing collection')
            return
        try:
            self._apply_slot_estimate(stable)
            self._begin_retrieval_removal()
        except Exception as exc:
            self._fault(f'cannot start corrected slot retrieval: {exc}')

    def _apply_slot_estimate(self, estimate):
        record = self.occupied[self.active_slot]
        # Move the saved grasp rigidly with the detected item. Retain the known
        # item-to-TCP transform, dimensions, identity, and force-contact pickup.
        yaw = float(estimate[3])
        object_q = self._quaternion_from_rpy(0., 0., yaw)
        tcp_q = self._quat_multiply(object_q, self._quat_inverse(record['orientation']))
        center = np.array([estimate[0], estimate[1], estimate[2]-record['size'][2]/2])
        tcp_xyz = center-np.array(self._quat_rotate(record['center'], tcp_q))
        record.setdefault('stored_release_tcp_pose', record.get('release_tcp_pose'))
        if getattr(self, 'inspection_yaw_flip', False):
            # The object did not rotate. Rotate the grasp around its TCP,
            # keeping the same contact point, and re-express BOTH transforms.
            roll, pitch, old_yaw = self._rpy_from_quaternion(tcp_q)
            tcp_q = self._quaternion_from_rpy(roll, pitch, old_yaw+math.pi)
            inverse = self._quat_inverse(tcp_q)
            record['center'] = tuple(self._quat_rotate(center-tcp_xyz, inverse))
            record['orientation'] = self._quat_multiply(inverse, object_q)
        rpy = self._rpy_from_quaternion(tcp_q)
        record['release_tcp_pose'] = (*map(float, tcp_xyz*1000), *rpy)
        record['target_tcp_pose'] = (*map(float, tcp_xyz), *rpy)
        record['object_yaw'] = yaw
        record['object_orientation_xyzw'] = object_q
        record['inspection_top_pose_m_rad'] = list(map(float, estimate))
        self.get_logger().info(f'slot {self.active_slot} re-estimated from 20 stable depth frames')
