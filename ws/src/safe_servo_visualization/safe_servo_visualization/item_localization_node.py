import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


BOX_DIMENSIONS_M = {
    0: (0.220, 0.180, 0.110), 1: (0.200, 0.100, 0.150),
    2: (0.150, 0.100, 0.200), 4: (0.100, 0.100, 0.150),
    5: (0.220, 0.170, 0.080), 6: (0.150, 0.150, 0.150),
    7: (0.190, 0.190, 0.110), 9: (0.220, 0.160, 0.120),
}


class ItemLocalization(Node):
    def __init__(self):
        super().__init__('item_localization')
        self.expected_id = -1
        self.required_samples = 30
        self.position_tolerance = 0.005
        self.angle_tolerance = math.radians(2.0)
        self.visible_ids = []
        self.active_id = None
        self.samples = []
        self.last_stamp = None
        self.collecting = False
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.status_pub = self.create_publisher(
            String, '/item_localization/status', 10)
        self.result_pub = self.create_publisher(
            Float64MultiArray, '/item_localization/result', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/safe_servo/force_marker', 10)
        self.create_subscription(
            Int32MultiArray, '/marker_detection/marker_ids',
            self.ids_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/item_localization/config',
            self.config_callback, 10)
        self.create_service(
            Trigger, '/item_localization/start', self.start_callback)
        self.create_service(
            Trigger, '/item_localization/clear', self.clear_callback)
        self.create_timer(1.0 / 30.0, self.sample_item)

    def ids_callback(self, msg):
        self.visible_ids = [int(value) for value in msg.data
                            if int(value) in BOX_DIMENSIONS_M]

    def config_callback(self, msg):
        if self.collecting or len(msg.data) < 4:
            return
        self.expected_id = int(round(msg.data[0]))
        self.required_samples = max(5, int(round(msg.data[1])))
        self.position_tolerance = max(0.0005, msg.data[2] / 1000.0)
        self.angle_tolerance = math.radians(max(0.1, msg.data[3]))

    def start_callback(self, _, response):
        if self.expected_id >= 0 and self.expected_id not in BOX_DIMENSIONS_M:
            response.success = False
            response.message = f'No dimensions configured for marker {self.expected_id}'
            return response
        self.samples = []
        self.last_stamp = None
        self.active_id = self.expected_id if self.expected_id >= 0 else None
        self.collecting = True
        self.publish_status('DETECTING: waiting for recognized item')
        response.success = True
        response.message = 'Incoming-item detection started'
        return response

    def sample_item(self):
        if not self.collecting:
            return
        candidates = ([self.active_id] if self.active_id is not None else
                      sorted(set(self.visible_ids)))
        if not candidates:
            self.publish_status('DETECTING: no recognized item visible')
            return
        marker_id = candidates[0]
        try:
            tf = self.tf_buffer.lookup_transform(
                'link_base', f'box_{marker_id}', rclpy.time.Time(),
                timeout=Duration(seconds=0.01))
        except Exception:
            self.publish_status(f'DETECTING: waiting for box_{marker_id} TF')
            return
        stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        if self.active_id is None:
            self.active_id = marker_id
        if marker_id != self.active_id:
            return
        self.last_stamp = stamp
        t, q = tf.transform.translation, tf.transform.rotation
        self.samples.append((np.array([t.x, t.y, t.z]),
                             np.array([q.x, q.y, q.z, q.w])))
        self.publish_status(
            f'DETECTING item {marker_id}: {len(self.samples)}/{self.required_samples}')
        if len(self.samples) >= self.required_samples:
            self.finish_sampling()

    def finish_sampling(self):
        positions = np.asarray([sample[0] for sample in self.samples])
        mean_position = positions.mean(axis=0)
        position_spread = float(np.max(np.linalg.norm(
            positions - mean_position, axis=1)))
        quaternions = np.asarray([sample[1] for sample in self.samples])
        reference = quaternions[0]
        quaternions[np.sum(quaternions * reference, axis=1) < 0] *= -1
        values, vectors = np.linalg.eigh(quaternions.T @ quaternions)
        mean_q = vectors[:, np.argmax(values)]
        if np.dot(mean_q, reference) < 0:
            mean_q *= -1
        angular_spread = float(np.max(2.0 * np.arccos(
            np.clip(np.abs(quaternions @ mean_q), 0.0, 1.0))))
        self.collecting = False
        if (position_spread > self.position_tolerance or
                angular_spread > self.angle_tolerance):
            self.publish_status(
                f'UNSTABLE item {self.active_id}: '
                f'{position_spread*1000:.1f} mm, '
                f'{math.degrees(angular_spread):.1f} deg; retry')
            return
        dimensions = BOX_DIMENSIONS_M[self.active_id]
        result = Float64MultiArray()
        result.data = [float(self.active_id), *map(float, mean_position),
                       *map(float, mean_q), *dimensions]
        self.result_pub.publish(result)
        self.publish_marker(mean_position, mean_q, dimensions)
        self.publish_status(
            f'READY item {self.active_id}: '
            f'P=[{mean_position[0]:.3f}, {mean_position[1]:.3f}, '
            f'{mean_position[2]:.3f}] m, '
            f'Size=[{dimensions[0]*1000:.0f}, {dimensions[1]*1000:.0f}, '
            f'{dimensions[2]*1000:.0f}] mm')

    def publish_marker(self, position, q, dimensions):
        cube = Marker()
        cube.header.frame_id = 'link_base'
        cube.header.stamp = self.get_clock().now().to_msg()
        cube.ns, cube.id = 'incoming_item', 0
        cube.type, cube.action = Marker.CUBE, Marker.ADD
        cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = map(
            float, position)
        cube.pose.orientation.x, cube.pose.orientation.y, \
            cube.pose.orientation.z, cube.pose.orientation.w = map(float, q)
        cube.scale.x, cube.scale.y, cube.scale.z = dimensions
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.1, 1.0, 0.2, 0.55
        self.marker_pub.publish(cube)
        label = Marker()
        label.header = cube.header
        label.ns, label.id = 'incoming_item', 1
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x, label.pose.position.y = map(float, position[:2])
        label.pose.position.z = float(position[2] + dimensions[2] / 2 + 0.05)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.04
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f'INCOMING ITEM {self.active_id}'
        self.marker_pub.publish(label)

    def clear_callback(self, _, response):
        self.collecting = False
        self.samples = []
        self.active_id = None
        for marker_id in (0, 1):
            marker = Marker()
            marker.header.frame_id = 'link_base'
            marker.ns, marker.id = 'incoming_item', marker_id
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
        self.publish_status('IDLE')
        response.success = True
        response.message = 'Incoming-item detection cleared'
        return response

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ItemLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
