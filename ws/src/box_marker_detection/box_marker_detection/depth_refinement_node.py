import json
import math
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from box_marker_detection.detector_node import (
    matrix_quaternion, quaternion_matrix, transform_matrix)


def pose_matrix(pose):
    h = np.eye(4)
    h[:3, :3] = quaternion_matrix((pose.orientation.x, pose.orientation.y,
                                    pose.orientation.z, pose.orientation.w))
    h[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return h


def angle_distance(a, b):
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


class DepthBoxRefinement(Node):
    """Refine QR box top-face geometry from aligned RealSense depth."""

    def __init__(self):
        super().__init__('depth_box_refinement')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('crop_scale', 1.35)
        self.declare_parameter('top_band_m', 0.06)
        self.declare_parameter('ransac_threshold_m', 0.007)
        self.declare_parameter('minimum_points', 120)
        self.declare_parameter('pixel_stride', 2)
        self.declare_parameter('max_tf_age_sec', 0.5)
        p = lambda name: self.get_parameter(name).value
        self.depth_topic = str(p('depth_topic'))
        self.info_topic = str(p('camera_info_topic'))
        self.camera_frame = str(p('camera_frame'))
        self.base_frame = str(p('base_frame'))
        self.depth_scale = float(p('depth_scale'))
        self.crop_scale = float(p('crop_scale'))
        self.top_band = float(p('top_band_m'))
        self.ransac_threshold = float(p('ransac_threshold_m'))
        self.minimum_points = int(p('minimum_points'))
        self.pixel_stride = max(1, int(p('pixel_stride')))
        self.max_tf_age = float(p('max_tf_age_sec'))

        self.k = None
        self.qr_boxes = []
        self.info_logged = False
        self.marker_logged = False
        self.depth_count = 0
        self.ready_count = 0
        self.publish_count = 0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, camera_qos)
        self.create_subscription(
            Image, self.depth_topic, self.depth_callback, camera_qos)
        self.create_subscription(
            MarkerArray, '/marker_detection/boxes', self.marker_callback, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/pointcloud_detection/boxes', 10)
        self.diagnostics_pub = self.create_publisher(
            String, '/pointcloud_detection/diagnostics', 10)
        self.rng = np.random.default_rng(7)
        self.create_timer(2.0, self.log_progress)
        self.get_logger().info(
            f'waiting for aligned depth on {self.depth_topic}')

    def info_callback(self, msg):
        self.k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if not self.info_logged:
            self.get_logger().info(
                f'aligned CameraInfo received: {msg.width}x{msg.height}')
            self.info_logged = True

    def marker_callback(self, msg):
        self.qr_boxes = [deepcopy(marker) for marker in msg.markers
                         if marker.ns == 'detected_boxes'
                         and marker.action == Marker.ADD]
        if self.qr_boxes and not self.marker_logged:
            self.get_logger().info(
                f'QR seed received for box {self.qr_boxes[0].id}')
            self.marker_logged = True

    def depth_callback(self, msg):
        self.depth_count += 1
        if self.k is None or not self.qr_boxes:
            return
        self.ready_count += 1
        if msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().warning(
                f'unsupported aligned depth encoding: {msg.encoding}',
                throttle_duration_sec=5.0)
            return
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.step // 2)[:, :msg.width]
        try:
            camera_from_base = self.tf_buffer.lookup_transform(
                self.camera_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warning(
                f'no camera transform for depth refinement: {exc}',
                throttle_duration_sec=2.0)
            return
        image_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tf_stamp = camera_from_base.header.stamp
        transform_time = tf_stamp.sec + tf_stamp.nanosec * 1e-9
        tf_age = abs(image_time - transform_time)
        if tf_age > self.max_tf_age:
            self.get_logger().warning(
                f'skipping depth frame: camera TF is {tf_age:.3f} s old',
                throttle_duration_sec=2.0)
            return
        h_camera_base = transform_matrix(camera_from_base.transform)
        h_base_camera = np.linalg.inv(h_camera_base)
        refined = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        refined.markers.append(clear)
        reports = []
        for qr in self.qr_boxes:
            result = self.refine_box(qr, depth, h_camera_base, h_base_camera)
            if result is None:
                continue
            marker, report = result
            report['tf_age_sec'] = round(tf_age, 4)
            marker.header.stamp = msg.header.stamp
            refined.markers.append(marker)
            refined.markers.append(
                self.make_label(marker, report, msg.header.stamp))
            reports.append(report)
        self.marker_pub.publish(refined)
        diagnostic = String()
        diagnostic.data = json.dumps(reports, separators=(',', ':'))
        self.diagnostics_pub.publish(diagnostic)
        self.publish_count += 1

    def log_progress(self):
        self.get_logger().info(
            f'depth frames={self.depth_count}, ready={self.ready_count}, '
            f'published={self.publish_count}', throttle_duration_sec=10.0)

    def refine_box(self, qr, depth, h_camera_base, h_base_camera):
        h_base_qrbox = pose_matrix(qr.pose)
        dims = np.array([qr.scale.x, qr.scale.y, qr.scale.z], dtype=float)
        top_local = np.array([
            [-dims[0]/2, -dims[1]/2, dims[2]/2, 1.0],
            [ dims[0]/2, -dims[1]/2, dims[2]/2, 1.0],
            [ dims[0]/2,  dims[1]/2, dims[2]/2, 1.0],
            [-dims[0]/2,  dims[1]/2, dims[2]/2, 1.0],
        ]).T
        camera_corners = h_camera_base @ h_base_qrbox @ top_local
        if np.any(camera_corners[2] <= 0.05):
            return None
        u = self.k[0, 0] * camera_corners[0] / camera_corners[2] + self.k[0, 2]
        v = self.k[1, 1] * camera_corners[1] / camera_corners[2] + self.k[1, 2]
        margin = 20
        x0 = max(0, int(np.floor(u.min())) - margin)
        x1 = min(depth.shape[1], int(np.ceil(u.max())) + margin)
        y0 = max(0, int(np.floor(v.min())) - margin)
        y1 = min(depth.shape[0], int(np.ceil(v.max())) + margin)
        if x1 <= x0 or y1 <= y0:
            return None
        vv, uu = np.mgrid[y0:y1:self.pixel_stride,
                          x0:x1:self.pixel_stride]
        z = depth[vv, uu].astype(np.float64) * self.depth_scale
        valid = np.isfinite(z) & (z > 0.08) & (z < 3.0)
        if valid.sum() < self.minimum_points:
            return None
        z, uu, vv = z[valid], uu[valid], vv[valid]
        camera_points = np.vstack((
            (uu - self.k[0, 2]) * z / self.k[0, 0],
            (vv - self.k[1, 2]) * z / self.k[1, 1], z,
            np.ones_like(z)))
        base_points = (h_base_camera @ camera_points)[:3].T
        local = (np.linalg.inv(h_base_qrbox) @ np.vstack(
            (base_points.T, np.ones(len(base_points)))))[:3].T
        keep = (
            (np.abs(local[:, 0]) < dims[0] * self.crop_scale / 2.0)
            & (np.abs(local[:, 1]) < dims[1] * self.crop_scale / 2.0)
            & (np.abs(local[:, 2] - dims[2]/2.0) < self.top_band))
        points = base_points[keep]
        if len(points) < self.minimum_points:
            return None
        plane = self.fit_plane(points)
        if plane is None:
            return None
        normal, inliers = plane
        top = points[inliers]
        if normal[2] < 0.0:
            normal = -normal
        xy = top[:, :2].astype(np.float32)
        (_, _), (width, length), angle_deg = cv2.minAreaRect(xy)
        if width <= 0.01 or length <= 0.01:
            return None
        yaw_a = math.radians(angle_deg)
        candidates = [
            (width, length, yaw_a),
            (length, width, yaw_a + math.pi/2.0),
        ]
        qr_yaw = math.atan2(h_base_qrbox[1, 0], h_base_qrbox[0, 0])
        width, length, yaw = min(
            candidates,
            key=lambda c: abs(c[0]-dims[0]) + abs(c[1]-dims[1])
            + 0.03 * angle_distance(c[2], qr_yaw))
        top_center = np.median(top, axis=0)
        x_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        x_axis -= normal * np.dot(x_axis, normal)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(normal, x_axis)
        rotation = np.column_stack((x_axis, y_axis, normal))
        center = top_center - normal * dims[2] / 2.0
        q = matrix_quaternion(rotation)
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns, marker.id = 'depth_refined_boxes', qr.id
        marker.type, marker.action = Marker.CUBE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(
            float, center)
        marker.pose.orientation.x, marker.pose.orientation.y, \
            marker.pose.orientation.z, marker.pose.orientation.w = map(float, q)
        marker.scale.x, marker.scale.y = float(width), float(length)
        # Top-only depth cannot independently observe box height; retain the
        # QR dictionary height while refining top pose, length and width.
        marker.scale.z = float(dims[2])
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            0.1, 1.0, 0.25, 0.38)
        marker.lifetime = Duration(seconds=0.35).to_msg()
        qr_center = h_base_qrbox[:3, 3]
        report = {
            'id': int(qr.id), 'points': int(len(points)),
            'inliers': int(inliers.sum()),
            'confidence': round(float(inliers.mean()), 3),
            'estimated_length_width_m': [round(float(width), 4),
                                         round(float(length), 4)],
            'qr_length_width_m': [round(float(dims[0]), 4),
                                  round(float(dims[1]), 4)],
            'center_difference_m': [round(float(v), 4)
                                    for v in center-qr_center],
            'yaw_difference_deg': round(math.degrees(
                angle_distance(yaw, qr_yaw)), 2),
            'height_source': 'qr_dictionary',
        }
        return marker, report

    def make_label(self, box, report, stamp):
        label = Marker()
        label.header.stamp = stamp
        label.header.frame_id = self.base_frame
        label.ns, label.id = 'depth_refined_box_labels', box.id
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose = deepcopy(box.pose)
        label.pose.position.z += box.scale.z / 2.0 + 0.11
        label.scale.z = 0.027
        label.color.r, label.color.g, label.color.b, label.color.a = (
            0.3, 1.0, 0.4, 1.0)
        q = box.pose.orientation
        rotation = quaternion_matrix((q.x, q.y, q.z, q.w))
        pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        label.text = (
            f'Depth box {box.id}\n'
            f'P [{box.pose.position.x:.3f}, {box.pose.position.y:.3f}, '
            f'{box.pose.position.z:.3f}] m\n'
            f'RPY [{math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, '
            f'{math.degrees(yaw):.1f}] deg\n'
            f'Size [{box.scale.x*1000:.0f}, {box.scale.y*1000:.0f}, '
            f'{box.scale.z*1000:.0f}] mm\n'
            f'Fit {report["confidence"]:.3f}')
        label.lifetime = box.lifetime
        return label

    def fit_plane(self, points):
        best = None
        count = len(points)
        for _ in range(100):
            sample = points[self.rng.choice(count, 3, replace=False)]
            normal = np.cross(sample[1]-sample[0], sample[2]-sample[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-8:
                continue
            normal /= norm
            distance = np.abs((points-sample[0]) @ normal)
            inliers = distance < self.ransac_threshold
            if best is None or inliers.sum() > best[1].sum():
                best = (normal, inliers)
        if best is None or best[1].sum() < self.minimum_points:
            return None
        inlier_points = points[best[1]]
        _, _, vh = np.linalg.svd(inlier_points-inlier_points.mean(axis=0),
                                 full_matrices=False)
        normal = vh[-1]
        distance = np.abs((points-inlier_points.mean(axis=0)) @ normal)
        return normal, distance < self.ransac_threshold


def main(args=None):
    rclpy.init(args=args)
    node = DepthBoxRefinement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
