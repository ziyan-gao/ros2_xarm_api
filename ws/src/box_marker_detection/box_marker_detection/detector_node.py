import math
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


BOX_DIMENSIONS_MM = {
    1: (210.0, 110.0, 160.0),
    2: (160.0, 110.0, 210.0),
    4: (110.0, 110.0, 150.0),
    6: (160.0, 160.0, 160.0),
    7: (170.0, 170.0, 110.0),
    9: (220.0, 170.0, 120.0),
}

# Printed ArUco sizes are not identical across all boxes.  solvePnP scales
# translation directly from this edge length, so using the 63 mm default for
# marker 5 places its pose about 68 mm too close to the camera.
MARKER_LENGTH_OVERRIDES_M = {
    5: 0.075,
}


def marker_object_points(marker_length):
    half = marker_length / 2.0
    return np.array([
        [-half, half, 0.0], [half, half, 0.0],
        [half, -half, 0.0], [-half, -half, 0.0],
    ], dtype=np.float32)


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([
        [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y],
    ])


def matrix_quaternion(r):
    trace = np.trace(r)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return ((r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s,
                (r[1, 0]-r[0, 1])/s, 0.25*s)
    i = int(np.argmax(np.diag(r)))
    if i == 0:
        s = math.sqrt(1+r[0, 0]-r[1, 1]-r[2, 2])*2
        return (0.25*s, (r[0, 1]+r[1, 0])/s, (r[0, 2]+r[2, 0])/s,
                (r[2, 1]-r[1, 2])/s)
    if i == 1:
        s = math.sqrt(1+r[1, 1]-r[0, 0]-r[2, 2])*2
        return ((r[0, 1]+r[1, 0])/s, 0.25*s, (r[1, 2]+r[2, 1])/s,
                (r[0, 2]-r[2, 0])/s)
    s = math.sqrt(1+r[2, 2]-r[0, 0]-r[1, 1])*2
    return ((r[0, 2]+r[2, 0])/s, (r[1, 2]+r[2, 1])/s, 0.25*s,
            (r[1, 0]-r[0, 1])/s)


def transform_matrix(transform):
    h = np.eye(4)
    h[:3, :3] = quaternion_matrix((transform.rotation.x,
                                    transform.rotation.y,
                                    transform.rotation.z,
                                    transform.rotation.w))
    h[:3, 3] = (transform.translation.x, transform.translation.y,
                 transform.translation.z)
    return h


def matrix_pose(h):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, h[:3, 3])
    q = matrix_quaternion(h[:3, :3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, \
        pose.orientation.w = map(float, q)
    return pose


def pose_rpy_degrees(pose):
    q = pose.orientation
    rotation = quaternion_matrix((q.x, q.y, q.z, q.w))
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class BoxMarkerDetector(Node):
    def __init__(self):
        super().__init__('box_marker_detector')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('marker_length_m', 0.063)
        self.declare_parameter('box_centroid_sign', -1.0)
        p = lambda name: self.get_parameter(name).value
        self.camera_frame = str(p('camera_frame'))
        self.base_frame = str(p('base_frame'))
        self.marker_length = float(p('marker_length_m'))
        self.centroid_sign = float(p('box_centroid_sign'))

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion = None
        self.camera_size = None
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary, cv2.aruco.DetectorParameters())
        self.object_points = marker_object_points(self.marker_length)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            CameraInfo, str(p('camera_info_topic')), self.info_callback, sensor_qos)
        self.create_subscription(
            Image, str(p('image_topic')), self.image_callback, sensor_qos)
        self.image_pub = self.create_publisher(
            Image, '/marker_detection/image', 10)
        self.pose_pub = self.create_publisher(
            PoseArray, '/marker_detection/box_poses', 10)
        self.id_pub = self.create_publisher(
            Int32MultiArray, '/marker_detection/marker_ids', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/marker_detection/boxes', 10)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            'box marker detector waiting for image and CameraInfo')

    def info_callback(self, msg):
        first_info = self.camera_matrix is None
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(msg.d, dtype=np.float64)
        self.camera_size = (msg.width, msg.height)
        if first_info:
            self.get_logger().info(
                f'using live CameraInfo: {msg.width}x{msg.height}, '
                f'frame={msg.header.frame_id}')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
        if self.camera_size != (msg.width, msg.height):
            self.get_logger().error(
                f'CameraInfo {self.camera_size} does not match image '
                f'{(msg.width, msg.height)}')
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        marker_array = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        marker_array.markers.append(delete)
        poses = PoseArray()
        poses.header.stamp = msg.header.stamp
        poses.header.frame_id = self.base_frame
        detected_ids = []

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            try:
                base_to_camera = self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame,
                    rclpy.time.Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.05))
                h_base_camera = transform_matrix(base_to_camera.transform)
            except Exception as exc:
                try:
                    # Joint-state TF commonly trails the camera stamp by one
                    # report cycle. Use the newest pose instead of discarding
                    # an otherwise valid detection.
                    base_to_camera = self.tf_buffer.lookup_transform(
                        self.base_frame, self.camera_frame, rclpy.time.Time())
                    h_base_camera = transform_matrix(base_to_camera.transform)
                    self.get_logger().warning(
                        f'using latest camera TF (exact stamp unavailable): {exc}',
                        throttle_duration_sec=2.0)
                except Exception as latest_exc:
                    h_base_camera = None
                    self.get_logger().warning(
                        f'no camera TF; publishing overlay only: {latest_exc}',
                        throttle_duration_sec=2.0)

            for corner, marker_id_array in zip(corners, ids):
                # OpenCV 4 returns shape (N, 1), while OpenCV 5 may return
                # shape (N,); normalize both forms.
                marker_id = int(np.asarray(marker_id_array).reshape(-1)[0])
                marker_length = MARKER_LENGTH_OVERRIDES_M.get(
                    marker_id, self.marker_length)
                object_points = (
                    marker_object_points(marker_length)
                    if marker_length != self.marker_length
                    else self.object_points)
                ok, rvec, tvec = cv2.solvePnP(
                    object_points, corner.reshape(4, 2),
                    self.camera_matrix, self.distortion,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                cv2.drawFrameAxes(
                    image, self.camera_matrix, self.distortion,
                    rvec, tvec, marker_length / 2.0)
                detected_ids.append(marker_id)
                if h_base_camera is None:
                    continue
                h_camera_marker = np.eye(4)
                h_camera_marker[:3, :3] = cv2.Rodrigues(rvec)[0]
                h_camera_marker[:3, 3] = tvec.reshape(3)
                self.broadcast_raw_marker_tf(
                    marker_id, h_base_camera @ h_camera_marker,
                    msg.header.stamp)
                if marker_id not in BOX_DIMENSIONS_MM:
                    continue
                dimensions = np.asarray(BOX_DIMENSIONS_MM[marker_id]) / 1000.0
                h_marker_box = np.eye(4)
                h_marker_box[2, 3] = self.centroid_sign * dimensions[2] / 2.0
                h_base_box = h_base_camera @ h_camera_marker @ h_marker_box
                pose = matrix_pose(h_base_box)
                poses.poses.append(pose)
                marker_array.markers.extend(
                    self.make_box_markers(marker_id, dimensions, pose, msg.header.stamp))
                self.broadcast_box_tf(marker_id, pose, msg.header.stamp)

        # cv_bridge from Jazzy does not recognize OpenCV 5's CV_8UC3 type
        # number on conversion back to ROS. Construct the standard image
        # message directly; the input conversion remains compatible.
        overlay = Image()
        overlay.header = msg.header
        overlay.height = image.shape[0]
        overlay.width = image.shape[1]
        overlay.encoding = 'bgr8'
        overlay.is_bigendian = False
        overlay.step = image.shape[1] * 3
        overlay.data = np.ascontiguousarray(image).tobytes()
        self.image_pub.publish(overlay)
        self.pose_pub.publish(poses)
        id_msg = Int32MultiArray()
        id_msg.data = detected_ids
        self.id_pub.publish(id_msg)
        self.marker_pub.publish(marker_array)

    def make_box_markers(self, marker_id, dimensions, pose, stamp):
        cube = Marker()
        cube.header.stamp = stamp
        cube.header.frame_id = self.base_frame
        cube.ns, cube.id = 'detected_boxes', marker_id
        cube.type, cube.action = Marker.CUBE, Marker.ADD
        # ROS Python messages are mutable. Keep independent copies so moving
        # the text label cannot also move the cube, PoseArray, or box TF.
        cube.pose = deepcopy(pose)
        cube.scale.x, cube.scale.y, cube.scale.z = map(float, dimensions)
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.1, 0.55, 1.0, 0.38
        cube.lifetime = Duration(seconds=0.3).to_msg()
        label = Marker()
        label.header = cube.header
        label.ns, label.id = 'detected_box_labels', marker_id
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose = deepcopy(pose)
        label.pose.position.z += float(dimensions[2] / 2.0 + 0.04)
        label.scale.z = 0.04
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        roll, pitch, yaw = pose_rpy_degrees(pose)
        label.text = (
            f'QR box {marker_id}\n'
            f'P [{pose.position.x:.3f}, {pose.position.y:.3f}, '
            f'{pose.position.z:.3f}] m\n'
            f'RPY [{roll:.1f}, {pitch:.1f}, {yaw:.1f}] deg\n'
            f'Size [{dimensions[0]*1000:.0f}, {dimensions[1]*1000:.0f}, '
            f'{dimensions[2]*1000:.0f}] mm')
        label.lifetime = cube.lifetime
        return [cube, label]

    def broadcast_box_tf(self, marker_id, pose, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = f'box_{marker_id}'
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def broadcast_raw_marker_tf(self, marker_id, transform, stamp):
        """Publish the marker centre without applying a box-height offset."""
        pose = matrix_pose(transform)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = f'aruco_marker_{marker_id}'
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = BoxMarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
