"""Explicit, operator-triggered top-face debug moves (no automatic pickup)."""
import json
import math
import time

import cv2
import numpy as np
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger
from xarm_msgs.srv import SetInt16

from .top_face_geometry import transform, top_points, project, prompts
from .pick_path_client import PickPathClient


def centered_camera_pose(base_tcp, tcp_camera, points, k, d, width, height, tcp_z):
    """Keep EEF rotation; translate the camera to center the recorded top face.

    Uses the ray of the *image center*, not the optical principal point. Camera
    extrinsics include translation as well as rotation. No measured depth.
    """
    if not np.isfinite(tcp_z) or tcp_z < points[0, 2] + .10:
        raise ValueError('inspection TCP height must exceed the recorded top by 100 mm')
    camera = base_tcp @ tcp_camera
    ray = cv2.undistortPoints(np.array([[[width/2., height/2.]]]), k, d)[0, 0]
    direction = camera[:3, :3] @ np.array([*ray, 1.])
    if direction[2] >= -.1:
        raise ValueError('camera is not looking down; establish a downward inspection orientation first')
    offset = base_tcp[:3, :3] @ tcp_camera[:3, 3]
    distance = (points[0, 2]-tcp_z-offset[2])/direction[2]
    if distance <= .1:
        raise ValueError('inspection camera is too close to the top face')
    result = base_tcp.copy()
    result[:3, 3] = points[0]-direction*distance-offset
    pixels = project(points, np.linalg.inv(result @ tcp_camera), k, d)
    prompts(pixels, width, height)  # Entire face, including image margin.
    return result


def recorded_grasp_rpy(object_in_tcp, refined_yaw):
    """R_base_tcp = R_base_object * inverse(R_tcp_object)."""
    if object_in_tcp is None:
        raise ValueError('recorded grasp orientation missing; place the item again with the updated scene node')
    if not math.isfinite(refined_yaw):
        raise ValueError('invalid refined yaw')
    object_rotation = transform([0, 0, 0], [0, 0, math.sin(refined_yaw/2), math.cos(refined_yaw/2)])[:3, :3]
    grasp_rotation = transform([0, 0, 0], object_in_tcp)[:3, :3]
    r = object_rotation @ grasp_rotation.T
    pitch = math.asin(float(np.clip(-r[2, 0], -1., 1.)))
    if abs(math.cos(pitch)) < .1:
        raise ValueError('recorded grasp orientation is unsuitable for top pickup')
    return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])


class TopFaceMotion:
    def _init_motion(self):
        for name, default in (('inspection_tcp_z_m', 0.), ('refine_max_age_sec', 30.)):
            self.declare_parameter(name, default)
        self.motion_status = {}
        self.motion_seen = {}
        self.motion_phase = None
        self.motion_future = None
        self.motion_started = 0.
        self.pick_path = PickPathClient(self, self._motion_fault)
        self.view_pub = self.create_publisher(PoseStamped, '/top_face_debug/view_target', 10)
        self.pick_pub = self.create_publisher(Float64MultiArray, '/staging_slots/retrieve_target', 10)
        self.motion_clients = {key: self.create_client(Trigger, path) for key, path in {
            'view': '/motion_coordinator/plan_top_face_view',
            'execute': '/motion_coordinator/execute',
            'pick': '/pickup_supervisor/start',
            'cancel': '/motion_coordinator/cancel',
            'abort': '/pickup_supervisor/abort',
            'retreat': '/pickup_supervisor/retreat',
        }.items()}
        self.remove_source = self.create_client(SetInt16, '/planning_scene_obstacles/remove_placed_item')
        self.consume_slot = self.create_client(SetInt16, '/staging_slots/accept_debug_pick')
        for key, topic in {
                'motion': '/motion_coordinator/status', 'pickup': '/pickup_supervisor/status',
                'scene': '/planning_scene_obstacles/status', 'slots': '/staging_slots/status',
                'policy': '/policy_loading/status', 'random': '/random_stable_loading/status',
                'test': '/pick_place_test/status', 'cycle': '/pick_place_pipeline/status',
                'place': '/place_pipeline/status', 'estimate': '/pickup_pipeline/status'}.items():
            self.create_subscription(String, topic, lambda msg, key=key: self._motion_status(key, msg), 10)

    def _motion_status(self, key, msg):
        try:
            self.motion_status[key] = json.loads(msg.data)
            self.motion_seen[key] = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _idle_checks(self):
        owners = getattr(self, '_inspection_owners', lambda: set())()
        for key in ('motion', 'pickup', 'scene', 'slots'):
            if time.monotonic()-self.motion_seen.get(key, 0) > 2:
                raise ValueError(f'{key} status unavailable/stale')
        for key, status in self.motion_status.items():
            if key == 'scene' or key in owners:
                continue
            if status.get('state') not in ('IDLE', 'SUCCEEDED', 'OBJECT_INFO_READY', 'READY', 'AWAITING_GRASP'):
                raise ValueError(f'{key} is not idle: {status.get("state")}')
            if any(status.get(k) for k in ('automatic', 'auto_enabled', 'random_running', 'random_active', 'automatic_enabled', 'auto_active', 'auto_start', 'cycle_auto_start', 'simulation_auto_active', 'continuous_loading_enabled', 'continuous_run_active', 'auto_start_pick_place')):
                raise ValueError(f'stop automatic {key} before debugging')
        scene = self.motion_status['scene']
        if scene.get('attached_item_id') or scene.get('attachment_pending'):
            raise ValueError('empty tool is required')
        pickup = self.motion_status['pickup']
        if pickup.get('robot_error', 0) or pickup.get('ft_recovery_required'):
            raise ValueError('robot/force sensor fault requires recovery before debug motion')

    def _target_geometry(self, key):
        marker = self.targets.get(key)
        if marker is None or time.monotonic()-self.marker_seen.get(key.split(':')[0], 0) > 2:
            raise ValueError('selected target is unavailable/stale')
        stamp = Time().to_msg()
        p, q = marker.pose.position, marker.pose.orientation
        box = self._matrix(self.base, marker.header.frame_id, stamp) @ transform(
            [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        size = [marker.scale.x, marker.scale.y, marker.scale.z]
        # Observation uses the recorded face geometry, including its tilt.
        # The horizontal-plane assumption belongs to SAM refinement, not here.
        return box, size, top_points(size, box)

    def _begin_motion(self, action, key):
        self._idle_checks()
        box, size, points = self._target_geometry(key)
        self.debug_source_id = None
        self.debug_slot = None
        self.debug_target = key
        if action == 'move_to':
            info = self.info
            if info is None:
                raise ValueError('CameraInfo is unavailable')
            stamp = self.get_clock().now().to_msg()
            tcp = self._matrix(self.base, 'link_tcp', Time().to_msg())
            extrinsic = self._matrix('link_tcp', info.header.frame_id, Time().to_msg())
            height = float(self.get_parameter('inspection_tcp_z_m').value)
            pallet = self._matrix(self.base, 'pallet_frame', Time().to_msg())
            clearance = self.motion_status['motion'].get('transfer_corner_height_pallet_m')
            if (not math.isfinite(height) or clearance is None or
                    not math.isfinite(float(clearance)) or float(clearance) <= 0):
                raise ValueError('container clearance/inspection height unavailable')
            height = max(height, float(pallet[2, 3])+float(clearance), float(points[0, 2])+.100)
            # Begin at container clearance, increasing only if the face does
            # not fit the image. Never lower below the container-safe plane.
            for extra in np.arange(0., .201, .01):
                try:
                    pose = centered_camera_pose(tcp, extrinsic, points,
                        np.asarray(info.k).reshape(3, 3), np.asarray(info.d), info.width, info.height,
                        height+float(extra))
                    break
                except ValueError as exc:
                    if 'outside the image margin' not in str(exc) or extra >= .20:
                        raise
            tf = self.tf.lookup_transform(self.base, 'link_tcp', Time()).transform
            target = PoseStamped()
            target.header.frame_id = self.base
            target.header.stamp = stamp
            target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, pose[:3, 3])
            target.pose.orientation = tf.rotation
            self.debug_view_target = target
            self.get_logger().info(
                f'top-face view target XYZ={pose[:3, 3].tolist()} m; '
                f'container-safe starting TCP Z={height:.3f} m')
            self.view_pub.publish(target)
            self.snapshot = self.result = self.mask = self.preview = None
            self._phase('VIEW_TARGET')
        else:
            if (not self.snapshot or self.snapshot['meta']['target'] != key or
                    not self.result or 'top_center_base_m' not in self.result):
                raise ValueError('capture and successfully refine this target first')
            age = self.get_clock().now().nanoseconds*1e-9-self.snapshot['meta']['color_stamp']
            if not 0 <= age <= float(self.get_parameter('refine_max_age_sec').value):
                raise ValueError('refine result expired; capture and run SAM again')
            if not np.allclose(box, self.snapshot['meta']['base_from_box'], atol=.001, rtol=0):
                raise ValueError('recorded target changed since capture; refine again')
            if not np.allclose(size, self.snapshot['meta']['size_m'], atol=.001, rtol=0):
                raise ValueError('target dimensions changed since capture')
            if key.startswith('placed:'):
                mapping = self.motion_status['scene'].get('placed_marker_object_ids', {})
                obstacle = mapping.get(key.split(':')[-1])
                if not obstacle or not obstacle.startswith('placed_item_'):
                    raise ValueError('scene marker identity unavailable; rebuild scene node')
                self.debug_source_id = int(obstacle.removeprefix('placed_item_'))
                for slot in self.motion_status['slots'].get('slots', []):
                    if slot.get('occupied') and slot.get('obstacle_id') == obstacle:
                        self.debug_slot = int(slot['slot'])
            top = list(map(float, self.result['top_center_base_m']))
            yaw = float(self.result['yaw_rad'])
            if not key.startswith('placed:'):
                raise ValueError('Pick requires a placed item with a recorded grasp; use the normal new-item pickup for detections')
            recorded = self.motion_status['scene'].get('placed_marker_grasp_orientations', {}).get(key.split(':')[-1])
            roll, pitch, grasp_yaw = recorded_grasp_rpy(recorded, yaw)
            pallet = self._matrix(self.base, 'pallet_frame', Time().to_msg())
            height = self.motion_status['motion'].get('transfer_corner_height_pallet_m')
            if height is None or not math.isfinite(float(height)):
                raise ValueError('container clearance unavailable')
            overhead = max(pallet[2, 3]+float(height), top[2]+.08)
            self.debug_pick_values = [float(self.debug_slot if self.debug_slot is not None else 0),
                *top, roll, pitch, grasp_yaw, *size, .03, yaw, float(overhead),
                1. if self.debug_slot is not None else 0.]
            self.get_logger().info(
                f'top-face pick contact XYZ={top} m; pre-pick Z={top[2]+.03:.4f} m; '
                f'object yaw={math.degrees(yaw):.2f} deg, tool yaw={math.degrees(grasp_yaw):.2f} deg; '
                f'recorded size={size}, SAM XY={self.result.get("fitted_size_xy_m")} m')
            self.pick_pub.publish(Float64MultiArray(data=self.debug_pick_values))
            self._phase('PICK_TARGET')
        self.motion_started = time.monotonic()
        self.debug_motion_id = int(self.motion_status['motion']['operation_id'])+1

    def _phase(self, phase):
        self.motion_phase = phase
        self.phase_started = time.monotonic()
        self.state, self.message = phase, phase.replace('_', ' ')
        self.get_logger().info(f'top-face motion phase: {phase}')

    def _request(self, client, phase, request=None):
        if not client.service_is_ready():
            raise ValueError('required motion service is unavailable')
        self.motion_future = client.call_async(request or Trigger.Request())
        self._phase(phase)

    def _motion_fault(self, reason):
        self.pick_path.cancel(stop=True)
        for name in ('abort', 'cancel'):
            client = self.motion_clients[name]
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        self.motion_future = None
        self.motion_phase = None
        self.result = None  # Never retry an old grasp result.
        self.state, self.message = 'FAULT', str(reason) + '; stopped, no automatic release/retry. Check physical inventory before restarting loaders.'
        self.get_logger().error(self.message)

    def _motion_tick(self):
        if self.motion_phase is None:
            return
        try:
            if time.monotonic()-self.motion_started > 180:
                raise ValueError('debug motion timed out')
            for key in ('motion', 'pickup', 'scene', 'slots'):
                if time.monotonic()-self.motion_seen.get(key, 0) > 2:
                    raise ValueError(f'lost {key} status during motion')
            motion, pickup = self.motion_status['motion'], self.motion_status['pickup']
            owners = getattr(self, '_inspection_owners', lambda: set())()
            for key in ('policy', 'random', 'test', 'cycle', 'place', 'estimate', 'slots'):
                if key in owners:
                    continue
                status = self.motion_status.get(key)
                if status and status.get('state') not in ('IDLE', 'SUCCEEDED', 'OBJECT_INFO_READY', 'READY', 'AWAITING_GRASP'):
                    raise ValueError(f'another workflow started: {key}')
            if motion.get('state') == 'FAULT' or pickup.get('state') == 'FAULT':
                raise ValueError(motion.get('fault') or pickup.get('fault') or 'motion fault')
            if self.motion_future:
                if not self.motion_future.done():
                    if time.monotonic()-self.phase_started > 10:
                        raise ValueError('motion service response timeout')
                    return
                response = self.motion_future.result()
                self.motion_future = None
                if not (response.success if hasattr(response, 'success') else response.ret == 0):
                    raise ValueError(response.message)
            phase = self.motion_phase
            elapsed = time.monotonic()-self.phase_started
            if phase == 'VIEW_TARGET' and elapsed > .3:
                self._request(self.motion_clients['view'], 'VIEW_PLAN')
            elif phase == 'VIEW_PLAN':
                if int(motion.get('operation_id', -1)) < self.debug_motion_id:
                    return
                if int(motion['operation_id']) != self.debug_motion_id or motion.get('target') != 'top_face_view':
                    raise ValueError('observation plan replaced by another command')
                if motion['state'] == 'PLANNED':
                    self._request(self.motion_clients['execute'], 'VIEW_EXECUTE')
            elif phase == 'VIEW_EXECUTE':
                if int(motion.get('operation_id', -1)) != self.debug_motion_id:
                    raise ValueError('observation execution replaced')
                if motion['state'] == 'SUCCEEDED':
                    self.motion_phase = None
                    self.state, self.message = 'IDLE', 'Camera move completed. Capture / project, then Run SAM.'
            elif phase == 'PICK_TARGET' and elapsed > .3:
                self.pick_path.reset(self.debug_motion_id)
                self._request(self.pick_path.prepare, 'PICK_PATH')
            elif phase == 'PICK_PATH':
                if motion.get('operation_id') == self.debug_motion_id:
                    snap = motion.get('planned_pregrasp') or {}
                    actual = [snap.get(k, float('nan')) for k in ('x_m', 'y_m', 'top_z_m', 'yaw_rad')]
                    expected = self.debug_pick_values[1:4]+[self.debug_pick_values[11]]
                    if not np.allclose(actual, expected, atol=1e-6, rtol=0):
                        raise ValueError('pre-pick target differs from the selected refine result')
                if self.pick_path.tick(motion, pickup):
                    self._phase('PICK_SETTLE')
            elif phase == 'PICK_SETTLE' and elapsed > .75:
                box, size, _ = self._target_geometry(self.debug_target)
                meta = self.snapshot['meta']
                # Freshness is checked when Pick is accepted. The accepted
                # target is frozen for this bounded operation; planning time
                # must not expire it after physically reaching pre-pick.
                if (not np.allclose(box, meta['base_from_box'], atol=.001, rtol=0) or
                        not np.allclose(size, meta['size_m'], atol=.001, rtol=0)):
                    raise ValueError('recorded target changed during approach; re-capture before grasping')
                if self.debug_source_id is not None:
                    self._request(self.remove_source, 'REMOVE_SOURCE', SetInt16.Request(data=self.debug_source_id))
                else:
                    self._phase('REMOVE_SOURCE')
            elif phase == 'REMOVE_SOURCE':
                ids = self.motion_status['scene'].get('placed_item_visual_ids', []) + self.motion_status['scene'].get('placed_item_ids', [])
                if self.debug_source_id is not None and f'placed_item_{self.debug_source_id}' in ids:
                    if elapsed > 5:
                        raise ValueError('source removal acknowledgement timeout')
                    return
                self.debug_pick_id = int(pickup['operation_id'])+1
                self._request(self.motion_clients['pick'], 'PICKUP')
            elif phase == 'PICKUP':
                if int(pickup.get('operation_id', -1)) < self.debug_pick_id:
                    return
                if int(pickup['operation_id']) != self.debug_pick_id:
                    raise ValueError('pickup operation replaced')
                if pickup['state'] == 'SUCCEEDED':
                    if not pickup.get('contact_detected'):
                        raise ValueError('pickup completed without contact-force confirmation')
                    scene = self.motion_status['scene']
                    if not scene.get('attached_item_id') or scene.get('last_attached_pickup_operation_id') != self.debug_pick_id:
                        return
                    if self.debug_slot is not None:
                        self._request(self.consume_slot, 'PICK_DONE', SetInt16.Request(data=self.debug_slot))
                    else:
                        self._phase('PICK_DONE')
            elif phase == 'PICK_DONE':
                self.motion_phase = None
                self.result = None
                self.state, self.message = 'PICKED', 'Force-confirmed pickup and lift completed; vacuum remains ON. Reset experiment inventory before resuming a loader.'
        except Exception as exc:
            self._motion_fault(exc)
