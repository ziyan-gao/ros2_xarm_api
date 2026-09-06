import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class ItemLocalization(Node):
    """Stabilize depth-refined box pose and dimensions for loading."""

    def __init__(self):
        super().__init__('item_localization')
        self.declare_parameter('refined_boxes_topic', '/pointcloud_detection/boxes')
        self.declare_parameter('max_detection_age_sec', 0.5)
        self.declare_parameter('dimension_tolerance_m', 0.01)

        self.expected_id = -1
        self.required_samples = 30
        self.position_tolerance = 0.005
        self.angle_tolerance = math.radians(2.0)
        self.dimension_tolerance = max(
            0.001, float(self.get_parameter('dimension_tolerance_m').value))
        self.max_detection_age = max(
            0.05, float(self.get_parameter('max_detection_age_sec').value))
        self.refined_boxes = {}
        self.active_id = None
        self.samples = []
        self.last_stamp = None
        self.collecting = False
        self.current_status = 'IDLE'

        self.status_pub = self.create_publisher(
            String, '/item_localization/status', 10)
        self.result_pub = self.create_publisher(
            Float64MultiArray, '/item_localization/result', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/safe_servo/force_marker', 10)
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter('refined_boxes_topic').value),
            self.refined_boxes_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray, '/item_localization/config',
            self.config_callback, 10)
        self.create_service(
            Trigger, '/item_localization/start', self.start_callback)
        self.create_service(
            Trigger, '/item_localization/clear', self.clear_callback)
        self.create_timer(1.0 / 30.0, self.sample_item)
        self.create_timer(0.5, self.republish_status)

    def refined_boxes_callback(self, msg):
        self.refined_boxes = {
            int(marker.id): marker for marker in msg.markers
            if (marker.ns == 'depth_refined_boxes' and
                marker.action == Marker.ADD and
                marker.header.frame_id == 'link_base')
        }

    def config_callback(self, msg):
        if self.collecting or len(msg.data) < 4:
            return
        self.expected_id = int(round(msg.data[0]))
        self.required_samples = max(5, int(round(msg.data[1])))
        self.position_tolerance = max(0.0005, msg.data[2] / 1000.0)
        self.angle_tolerance = math.radians(max(0.1, msg.data[3]))

    def start_callback(self, _, response):
        self.samples = []
        self.last_stamp = None
        self.active_id = self.expected_id if self.expected_id >= 0 else None
        self.collecting = True
        self.publish_status('DETECTING: waiting for one depth-refined box')
        response.success = True
        response.message = 'Depth-refined incoming-item measurement started'
        return response

    def _fresh_marker(self):
        if self.active_id is not None:
            marker = self.refined_boxes.get(self.active_id)
            if marker is None:
                return None, f'waiting for refined box {self.active_id}'
        elif len(self.refined_boxes) == 1:
            marker = next(iter(self.refined_boxes.values()))
        elif not self.refined_boxes:
            return None, 'no depth-refined box visible'
        else:
            ids = sorted(self.refined_boxes)
            return None, f'multiple refined boxes visible {ids}; select an item ID'

        stamp = rclpy.time.Time.from_msg(marker.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age < -0.05 or age > self.max_detection_age:
            return None, f'refined box {marker.id} is stale ({age:.2f} s)'
        return marker, ''

    def sample_item(self):
        if not self.collecting:
            return
        marker, reason = self._fresh_marker()
        if marker is None:
            self.publish_status(f'DETECTING: {reason}')
            return
        stamp = (marker.header.stamp.sec, marker.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        marker_id = int(marker.id)
        if self.active_id is None:
            self.active_id = marker_id
        if marker_id != self.active_id:
            return

        position = np.array([
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
        ], dtype=float)
        quaternion = np.array([
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        ], dtype=float)
        dimensions = np.array([
            marker.scale.x,
            marker.scale.y,
            marker.scale.z,
        ], dtype=float)
        values = np.concatenate((position, quaternion, dimensions))
        if not np.isfinite(values).all() or np.any(dimensions <= 0.0):
            self.publish_status(
                f'DETECTING: refined box {marker_id} has invalid geometry')
            return

        self.last_stamp = stamp
        self.samples.append((position, quaternion, dimensions))
        self.publish_status(
            f'DETECTING refined item {marker_id}: '
            f'{len(self.samples)}/{self.required_samples}')
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

        dimensions = np.asarray([sample[2] for sample in self.samples])
        mean_dimensions = dimensions.mean(axis=0)
        dimension_spread = float(np.max(np.abs(
            dimensions - mean_dimensions)))

        if (position_spread > self.position_tolerance or
                angular_spread > self.angle_tolerance or
                dimension_spread > self.dimension_tolerance):
            self.samples = []
            self.last_stamp = None
            self.publish_status(
                f'UNSTABLE refined item {self.active_id}: '
                f'position={position_spread*1000:.1f} mm, '
                f'angle={math.degrees(angular_spread):.1f} deg, '
                f'dimension={dimension_spread*1000:.1f} mm; collecting again')
            return

        self.collecting = False
        result = Float64MultiArray()
        result.data = [
            float(self.active_id),
            *map(float, mean_position),
            *map(float, mean_q),
            *map(float, mean_dimensions),
        ]
        self.result_pub.publish(result)
        self.publish_marker(mean_position, mean_q, mean_dimensions)
        self.publish_status(
            f'READY refined item {self.active_id}: '
            f'P=[{mean_position[0]:.3f}, {mean_position[1]:.3f}, '
            f'{mean_position[2]:.3f}] m, '
            f'Size=[{mean_dimensions[0]*1000:.0f}, '
            f'{mean_dimensions[1]*1000:.0f}, '
            f'{mean_dimensions[2]*1000:.0f}] mm')

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
        cube.scale.x, cube.scale.y, cube.scale.z = map(float, dimensions)
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = (
            0.1, 1.0, 0.2, 0.55)
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
        label.text = f'REFINED INCOMING ITEM {self.active_id}'
        self.marker_pub.publish(label)

    def clear_callback(self, _, response):
        self.collecting = False
        self.samples = []
        self.active_id = None
        self.last_stamp = None
        for marker_id in (0, 1):
            marker = Marker()
            marker.header.frame_id = 'link_base'
            marker.ns, marker.id = 'incoming_item', marker_id
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
        self.publish_status('IDLE')
        response.success = True
        response.message = 'Incoming-item measurement cleared'
        return response

    def publish_status(self, text):
        self.current_status = text
        self.republish_status()

    def republish_status(self):
        msg = String()
        msg.data = self.current_status
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
