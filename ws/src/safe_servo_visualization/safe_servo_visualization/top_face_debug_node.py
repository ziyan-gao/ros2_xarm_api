"""Top-face inspection sandbox with explicit operator-triggered motion."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
import copy
import json
import math
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .top_face_geometry import transform, top_points, project, prompts, fit_top
from .top_face_motion import TopFaceMotion
from .top_face_automation import TopFaceAutomation


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def image_array(msg):
    """Decode the camera's uncompressed images, respecting row padding/endian.

    Avoid a cv_bridge/OpenCV binary-version dependency in this optional tool.
    """
    layouts = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3),
               'rgba8': ('u1', 4), 'bgra8': ('u1', 4),
               '16UC1': ('u2', 1), '32FC1': ('f4', 1)}
    if msg.encoding not in layouts:
        raise ValueError(f'unsupported camera encoding: {msg.encoding}')
    kind, channels = layouts[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + kind)
    pixel_bytes = dtype.itemsize*channels
    if (msg.width <= 0 or msg.height <= 0 or msg.step < msg.width*pixel_bytes or
            len(msg.data) < msg.step*msg.height):
        raise ValueError('invalid image dimensions/row stride/buffer length')
    shape = (msg.height, msg.width, channels)
    array = np.ndarray(shape, dtype=dtype, buffer=msg.data,
                       strides=(msg.step, pixel_bytes, dtype.itemsize))
    if channels == 1:
        return array[:, :, 0].astype(np.float32) * (.001 if kind == 'u2' else 1.)
    array = array[:, :, :3]
    if msg.encoding.startswith('bgr'):
        array = array[:, :, ::-1]
    return array.copy()


def fresh_frame(colors, now, max_age=.8):
    frames = [c for c in colors if 0 <= now-stamp_seconds(c.header.stamp) <= max_age]
    if not frames:
        raise ValueError('no fresh RGB frame')
    return max(frames, key=lambda c: stamp_seconds(c.header.stamp))


class TopFaceDebug(TopFaceAutomation, TopFaceMotion, Node):
    def __init__(self):
        super().__init__('top_face_debug')
        for name, default in (
                ('base_frame', 'link_base'),
                ('color_topic', '/camera/camera/color/image_raw'),
                ('camera_info_topic', '/camera/camera/color/camera_info'),
                ('results_directory', '/workspace/results/top_face_debug'),
                ('sam_checkpoint', ''), ('sam_model_config', 'configs/sam2.1/sam2.1_hiera_t.yaml'),
                ('sam_device', 'cpu')):
            self.declare_parameter(name, default)
        self.base = self.get_parameter('base_frame').value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.colors = deque(maxlen=12)
        self.info = None
        self.targets = {}
        self.marker_seen = {}
        self.snapshot = None
        self.result = None
        self.preview = None
        self.mask = None
        self.state, self.message = 'IDLE', 'Select target: Move to, Capture / project, Run SAM, then explicit Pick.'
        self.generation = 0
        self.future = None
        self.worker_generation = None
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.predictor = None
        self._init_motion()
        self._init_automation()
        self.status_pub = self.create_publisher(String, '/top_face_debug/status', 10)
        self.image_pub = self.create_publisher(Image, '/top_face_debug/preview', 2)
        self.create_subscription(Image, self.get_parameter('color_topic').value,
                                 lambda msg: self.colors.append(msg), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._info, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, '/planning_scene_obstacles/placed_item_markers',
                                 lambda msg: self._markers('placed', msg), 10)
        self.create_subscription(MarkerArray, '/pointcloud_detection/boxes',
                                 lambda msg: self._markers('detected', msg), 10)
        self.create_subscription(MarkerArray, '/object_info_estimation/sam_coarse_boxes',
                                 lambda msg: self._markers('coarse', msg), 10)
        self.create_subscription(String, '/top_face_debug/command', self._command, 10)
        self.create_timer(.2, self._tick)

    def _info(self, msg):
        self.info = msg

    def _markers(self, source, msg):
        for marker in msg.markers:
            key = f'{source}:{marker.ns}:{marker.id}'
            if marker.action == Marker.DELETEALL:
                self.targets = {k: v for k, v in self.targets.items() if not k.startswith(source+':')}
            elif marker.action == Marker.DELETE:
                self.targets.pop(key, None)
            elif marker.action == Marker.ADD and marker.type == Marker.CUBE:
                self.targets[key] = marker
        self.marker_seen[source] = time.monotonic()

    def _matrix(self, destination, source, stamp):
        if destination == source:
            return np.eye(4)
        tf = self.tf.lookup_transform(destination, source, Time.from_msg(stamp)).transform
        return transform([tf.translation.x, tf.translation.y, tf.translation.z],
                         [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])

    def _capture(self, key):
        self.snapshot = self.result = self.mask = self.preview = None
        marker = self.targets.get(key)
        if marker is None or time.monotonic()-self.marker_seen.get(key.split(':')[0], 0) > 2:
            raise ValueError('target unavailable/stale; refresh scene markers')
        color = fresh_frame(self.colors, self.get_clock().now().nanoseconds*1e-9)
        info = self.info
        if (info is None or info.header.frame_id != color.header.frame_id or
                (info.width, info.height) != (color.width, color.height)):
            raise ValueError('RGB and CameraInfo geometry/frame mismatch')
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial', ''):
            raise ValueError('unsupported camera distortion model')
        if key.startswith('detected:') and abs(stamp_seconds(marker.header.stamp)-stamp_seconds(color.header.stamp)) > .8:
            raise ValueError('detected target is older than the captured frame')
        k = np.array(info.k, dtype=float).reshape(3, 3)
        d = np.array(info.d, dtype=float)
        if not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0, 0], k[1, 1]) <= 0:
            raise ValueError('invalid camera calibration')
        p, q = marker.pose.position, marker.pose.orientation
        base_box = self._matrix(self.base, marker.header.frame_id, color.header.stamp) @ transform(
            [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        base_camera = self._matrix(self.base, color.header.frame_id, color.header.stamp)
        size = [marker.scale.x, marker.scale.y, marker.scale.z]
        prior = top_points(size, base_box)
        pixels = project(prior, np.linalg.inv(base_camera), k, d)
        if color.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            raise ValueError('unsupported RGB encoding')
        rgb = image_array(color)
        meta = dict(target=key, base_frame=self.base, camera_frame=color.header.frame_id,
                    color_stamp=stamp_seconds(color.header.stamp),
                    estimation_method='rgb_mask_recorded_top_plane', uses_measured_depth=False,
                    recorded_top_z_m=float(prior[0, 2]), schema_version=2,
                    size_m=size, base_from_box=base_box.tolist(), base_from_camera=base_camera.tolist(),
                    k=k.tolist(), d=d.tolist(), distortion_model=info.distortion_model,
                    prior_top_base_m=prior.tolist(), projected_pixels=pixels.tolist(),
                    prior_yaw=math.atan2(base_box[1, 0], base_box[0, 0]), diagnostic_only=True)
        try:
            points, box = prompts(pixels, color.width, color.height)
            meta.update(prompt_points=points.tolist(), prompt_box=box.tolist())
            self.message = 'Captured. Green: predicted top. Run SAM once; no robot motion.'
        except ValueError as exc:
            meta['projection_warning'] = str(exc)
            self.message = str(exc) + ' Capture can still be saved.'
        self.snapshot = dict(rgb=rgb, meta=meta)
        self.state = 'CAPTURED'
        self._draw()

    def _segment(self, snapshot):
        """One background worker, lazy optional SAM import; never downloads models."""
        checkpoint = str(self.get_parameter('sam_checkpoint').value)
        if not checkpoint or not Path(checkpoint).is_file():
            raise ValueError('SAM unavailable: configure sam_checkpoint to an existing local SAM2 checkpoint')
        try:
            import torch
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
        except ImportError as exc:
            raise ValueError('SAM2 is not installed in this Python environment') from exc
        meta = snapshot['meta']
        with torch.inference_mode():
            if self.predictor is None:
                model = build_sam2(str(self.get_parameter('sam_model_config').value), checkpoint,
                                  device=str(self.get_parameter('sam_device').value))
                self.predictor = SAM2ImagePredictor(model)
            self.predictor.set_image(snapshot['rgb'])
            points = np.asarray(meta['prompt_points'], dtype=np.float32)
            masks, scores, _ = self.predictor.predict(point_coords=points,
                point_labels=np.ones(len(points), dtype=np.int32),
                box=np.asarray(meta['prompt_box'], dtype=np.float32), multimask_output=True)
        if len(masks) == 0 or len(masks) != len(scores) or not np.isfinite(scores).all():
            raise ValueError('SAM returned no valid mask scores')
        candidates, rejected = [], []
        for mask, score in zip(masks, scores):
            try:
                fit = fit_top(mask, np.asarray(meta['k']), np.asarray(meta['d']),
                              np.asarray(meta['base_from_camera']), meta['prior_top_base_m'],
                              meta['size_m'], meta['prior_yaw'])
                fit['sam_score'] = float(score)
                candidates.append((fit, mask.astype(bool)))
            except ValueError as exc:
                rejected.append(str(exc))
        if not candidates:
            idx = int(np.argmax(scores))
            return dict(error='No validated top face: ' + '; '.join(rejected), diagnostic_only=True), masks[idx].astype(bool)
        return max(candidates, key=lambda item: item[0]['sam_score'])

    def _draw(self):
        if self.snapshot is None:
            return
        rgb, meta = self.snapshot['rgb'], self.snapshot['meta']
        canvas = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self.mask is not None:
            canvas[self.mask] = (.6*canvas[self.mask]+.4*np.array([255, 0, 255])).astype(np.uint8)
        pix = np.clip(np.asarray(meta['projected_pixels']), -100000, 100000).astype(np.int32)
        cv2.polylines(canvas, [pix[1:]], True, (0, 255, 0), 2)
        cv2.drawMarker(canvas, tuple(pix[0]), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
        h, w = canvas.shape[:2]
        cv2.drawMarker(canvas, (w//2, h//2), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
        if self.result and 'top_center_base_m' in self.result:
            center = np.asarray([self.result['top_center_base_m']])
            p = project(center, np.linalg.inv(meta['base_from_camera']), np.asarray(meta['k']), np.asarray(meta['d']))[0]
            cv2.drawMarker(canvas, tuple(np.rint(p).astype(int)), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            self.result['image_center_error_px'] = (p-[w/2, h/2]).tolist()
            if meta.get('new_item_preview'):
                self.result['diagnostic_only'] = True
                self.result['center_xy_base_m'] = center[0, :2].tolist()
                self.result['unchanged_top_z_m'] = meta['recorded_top_z_m']
                self.result['unchanged_height_m'] = meta['size_m'][2]
                delta = (center[0, :2]-np.asarray(meta['prior_top_base_m'])[0, :2])*1000
                self.result['center_delta_xy_mm'] = delta.tolist()
                fitted_xy = self.result.get('fitted_size_xy_m', meta['size_m'][:2])
                self.result['size_delta_xy_mm'] = ((np.asarray(fitted_xy)-meta['size_m'][:2])*1000).tolist()
                cv2.putText(canvas, 'PREVIEW ONLY: dXY=(%+.1f, %+.1f) mm; top Z=%.1f mm unchanged' %
                            (*delta, meta['recorded_top_z_m']*1000),
                            (12, 92), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 220, 255), 1)
            yaw = self.result['yaw_rad']
            # Show what SAM actually fitted, not a second copy of the prior.
            # Physical collision/pickup dimensions remain the recorded ones.
            size = [*self.result.get('fitted_size_xy_m', meta['size_m'][:2]), meta['size_m'][2]]
            box_center = center[0]-[0, 0, size[2]/2]
            fitted = top_points(size, transform(box_center,
                [0, 0, math.sin(yaw/2), math.cos(yaw/2)]))
            outline = project(fitted, np.linalg.inv(meta['base_from_camera']),
                              np.asarray(meta['k']), np.asarray(meta['d']))
            cv2.polylines(canvas, [np.rint(outline[1:]).astype(np.int32)], True, (0, 255, 255), 2)
            pre_pick = center.copy()
            pre_pick[0, 2] += .03
            pre_pixel = project(pre_pick, np.linalg.inv(meta['base_from_camera']),
                                np.asarray(meta['k']), np.asarray(meta['d']))[0]
            if np.isfinite(pre_pixel).all() and np.max(abs(pre_pixel)) < 100000:
                tip = tuple(np.rint(pre_pixel).astype(int))
                cv2.drawMarker(canvas, tip, (255, 255, 0), cv2.MARKER_DIAMOND, 18, 2)
                cv2.line(canvas, tip, tuple(np.rint(p).astype(int)), (255, 255, 0), 1)
            self.result['pre_pick_tcp_base_m'] = pre_pick[0].tolist()
            self.result['pre_pick_tcp_pixel'] = pre_pixel.tolist()
            cv2.putText(canvas, 'CYAN: pre-pick TCP (+30mm); YELLOW: contact center',
                        (12, 50), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 220, 255), 1)
            cv2.putText(canvas,
                        'Prior XY: %.1f x %.1f mm; SAM XY: %.1f x %.1f mm' %
                        tuple(v*1000 for v in [*meta['size_m'][:2], *size[:2]]),
                        (12, 70), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 220, 255), 1)
        cv2.putText(canvas, 'CAPTURED FRAME / EXPLICIT MOTION BUTTONS', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 220, 255), 2)
        self.preview = canvas
        msg = Image(height=h, width=w, encoding='bgr8', step=w*3, data=canvas.tobytes())
        msg.header.frame_id = meta['camera_frame']
        msg.header.stamp = Time(seconds=meta['color_stamp']).to_msg()
        self.image_pub.publish(msg)

    def _save(self):
        if self.snapshot is None:
            raise ValueError('capture a frame first')
        root = Path(str(self.get_parameter('results_directory').value)).expanduser()
        folder = root / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ_') + uuid.uuid4().hex[:8])
        folder.mkdir(parents=True, exist_ok=False)
        np.savez_compressed(folder/'frames.npz', rgb=self.snapshot['rgb'],
                            mask=np.zeros((0, 0), bool) if self.mask is None else self.mask)
        payload = dict(self.snapshot['meta'], result=self.result,
                       sam_model_config=str(self.get_parameter('sam_model_config').value),
                       sam_checkpoint=str(self.get_parameter('sam_checkpoint').value))
        (folder/'metadata.json').write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
        if self.preview is not None and not cv2.imwrite(str(folder/'preview.png'), self.preview):
            raise ValueError('could not write preview PNG')
        self.message = f'Saved: {folder}'

    def _command(self, msg):
        try:
            command = json.loads(msg.data)
            action = command['action']
            if action == 'stop':
                if self.motion_phase is not None or getattr(self, 'inspection', None):
                    self._motion_fault('operator stopped debug motion')
                return
            if getattr(self, 'inspection', None):
                self.message = 'Policy inspection is active; only Stop is accepted.'
                return
            if self.motion_phase is not None:
                self.message = 'Motion is active; only Stop is accepted.'
                return
            if action == 'clear':
                self.generation += 1  # Invalidate any late inference result.
                self.snapshot = self.result = self.mask = self.preview = None
                self.state, self.message = 'IDLE', 'Cleared (running inference, if any, is discarded when it finishes).'
                return
            if self.future is not None:
                raise ValueError('SAM worker is busy; wait or clear its result')
            if action == 'refine_new':
                key = command.get('target', '')
                if not key.startswith('detected:'):
                    raise ValueError('select a detected point-cloud box for new-item preview')
                self.generation += 1
                self._capture(key)
                self.snapshot['meta']['new_item_preview'] = True
                if 'prompt_points' not in self.snapshot['meta']:
                    raise ValueError('coarse top face is not fully visible; capture can be saved')
                self.worker_generation = self.generation
                self.future = self.pool.submit(self._segment, copy.deepcopy(self.snapshot))
                self.state, self.message = 'SEGMENTING', 'New-item preview only: coarse top Z retained; no motion or policy update.'
            elif action in ('move_to', 'pick'):
                self._begin_motion(action, command['target'])
            elif action == 'capture':
                self.generation += 1
                self._capture(command['target'])
            elif action == 'segment':
                if self.snapshot is None or 'prompt_points' not in self.snapshot['meta']:
                    raise ValueError('capture a complete in-view target first')
                if command.get('target') != self.snapshot['meta']['target']:
                    raise ValueError('selection changed; capture the new target first')
                self.result = self.mask = None
                self.worker_generation = self.generation
                self.future = self.pool.submit(self._segment, copy.deepcopy(self.snapshot))
                self.state, self.message = 'SEGMENTING', 'SAM inference on frozen capture; robot will not move.'
            elif action == 'save':
                self._save()
            else:
                raise ValueError('unknown debug action')
        except Exception as exc:
            self.state, self.message = 'ERROR', str(exc)
        finally:
            self._publish_status()

    def _tick(self):
        self._motion_tick()
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            if self.worker_generation == self.generation:
                try:
                    self.result, self.mask = future.result()
                    self.state = 'REJECTED' if 'error' in self.result else 'ESTIMATED'
                    self._draw()
                    self.message = json.dumps(self.result, ensure_ascii=True, allow_nan=False)
                except Exception as exc:
                    self.state, self.message = 'ERROR', str(exc)
        self._automation_tick()
        self._publish_status()

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(dict(state=self.state, message=self.message,
            busy=self.future is not None or self.motion_phase is not None or self.inspection is not None,
            motion_active=self.motion_phase is not None or self.inspection is not None,
            can_pick=self.result is not None and 'top_center_base_m' in self.result and
                not (self.snapshot and self.snapshot['meta'].get('new_item_preview')),
            targets=sorted(self.targets), captured_target=None if self.snapshot is None else self.snapshot['meta']['target'],
            can_segment=self.snapshot is not None and 'prompt_points' in self.snapshot['meta'],
            has_capture=self.snapshot is not None, diagnostic_only=False))
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TopFaceDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
