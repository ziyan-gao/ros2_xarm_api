import math

from geometry_msgs.msg import Point, TransformStamped, WrenchStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker


def transform_matrix(transform):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y],
    ]
    result[:3, 3] = [transform.translation.x, transform.translation.y,
                     transform.translation.z]
    return result


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


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('safe_servo_visualization')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.camera_tf_sent = False
        self.marker_pub = self.create_publisher(
            Marker, '/safe_servo/force_marker', 10)
        self.create_subscription(
            WrenchStamped, '/ufactory/uf_ftsensor_ext_states',
            self.force_callback, 10)
        self.create_timer(0.5, self.publish_camera_tf)

        # The hand-eye calibration was computed from OpenCV images/poses, so
        # its camera axes are the optical axes (x right, y down, z forward).
        # Translation in cam2eef.npz is millimetres; TF uses metres.
        self.h_eef_color_optical = np.eye(4)
        self.h_eef_color_optical[:3, :3] = [
            [-0.03980933, 0.99920357, -0.00272764],
            [-0.99911980, -0.03984182, -0.01312473],
            [-0.01322295, 0.00220275, 0.99991015],
        ]
        self.h_eef_color_optical[:3, 3] = (
            np.array([66.28637961, 30.06775093, 85.75730917]) / 1000.0)

    def publish_camera_tf(self):
        if self.camera_tf_sent:
            return
        try:
            existing = self.tf_buffer.lookup_transform(
                'camera_link', 'camera_color_optical_frame', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return
        h_link_color_optical = transform_matrix(existing.transform)
        h_eef_link = (
            self.h_eef_color_optical @ np.linalg.inv(h_link_color_optical))
        q = matrix_quaternion(h_eef_link[:3, :3])
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_eef'
        msg.child_frame_id = 'camera_link'
        msg.transform.translation.x = float(h_eef_link[0, 3])
        msg.transform.translation.y = float(h_eef_link[1, 3])
        msg.transform.translation.z = float(h_eef_link[2, 3])
        msg.transform.rotation.x, msg.transform.rotation.y, \
            msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
        self.static_tf.sendTransform(msg)
        self.camera_tf_sent = True
        self.get_logger().info(
            'published calibrated link_eef -> camera_link TF '
            '(calibration interpreted in camera_color_optical_frame)')

    def force_callback(self, msg):
        force = np.array([msg.wrench.force.x, msg.wrench.force.y,
                          msg.wrench.force.z], dtype=float)
        magnitude = float(np.linalg.norm(force))
        arrow = Marker()
        arrow.header.stamp = self.get_clock().now().to_msg()
        # link_tcp is the official vacuum-gripper tip frame. The fixed tool
        # joints do not rotate relative to the F/T sensor, so its force axes
        # remain valid while the marker begins at the physical suction tip.
        arrow.header.frame_id = 'link_tcp'
        arrow.ns, arrow.id = 'external_force', 0
        arrow.type, arrow.action = Marker.ARROW, Marker.ADD
        arrow.points = [Point(), Point(
            x=float(force[0]*0.01), y=float(force[1]*0.01),
            z=float(force[2]*0.01))]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.018, 0.025
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.15, 0.05, 0.95
        self.marker_pub.publish(arrow)

        text = Marker()
        text.header = arrow.header
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns, text.id = 'external_force', 1
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.z, text.pose.orientation.w = 0.10, 1.0
        text.scale.z = 0.035
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = (
            f'|F|={magnitude:.1f} N  '
            f'Fz={force[2]:.1f} N (tool bias normal)')
        self.marker_pub.publish(text)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
