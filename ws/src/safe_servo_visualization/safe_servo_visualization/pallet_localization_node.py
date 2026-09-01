import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker
import yaml


def quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y],
    ])


def matrix_quat(r):
    # Eigenvector averaging below guarantees a unit quaternion; this helper
    # converts the final rotation matrix used for marker-offset composition.
    trace = np.trace(r)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return np.array([(r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s,
                         (r[1, 0]-r[0, 1])/s, 0.25*s])
    i = int(np.argmax(np.diag(r)))
    if i == 0:
        s = math.sqrt(1+r[0, 0]-r[1, 1]-r[2, 2])*2
        return np.array([0.25*s, (r[0, 1]+r[1, 0])/s,
                         (r[0, 2]+r[2, 0])/s, (r[2, 1]-r[1, 2])/s])
    if i == 1:
        s = math.sqrt(1+r[1, 1]-r[0, 0]-r[2, 2])*2
        return np.array([(r[0, 1]+r[1, 0])/s, 0.25*s,
                         (r[1, 2]+r[2, 1])/s, (r[0, 2]-r[2, 0])/s])
    s = math.sqrt(1+r[2, 2]-r[0, 0]-r[1, 1])*2
    return np.array([(r[0, 2]+r[2, 0])/s, (r[1, 2]+r[2, 1])/s,
                     0.25*s, (r[1, 0]-r[0, 1])/s])


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll),
                              math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])


class PalletLocalization(Node):
    def __init__(self):
        super().__init__('pallet_localization')
        self.base_frame = 'link_base'
        self.marker_id = 49
        self.required_samples = 30
        self.position_tolerance = 0.005
        self.angle_tolerance = math.radians(2.0)
        self.pallet_x, self.pallet_y = 1.2, 1.0
        self.offset_rpy = np.zeros(3)
        self.config_path = Path('/workspace/config/pallet_place_config.yaml')
        self.configured_position = np.zeros(3)
        self.configured_q = np.array([0.0, 0.0, 0.0, 1.0])
        self.configured_pose_valid = False
        self.pre_place_xyz = np.array([0.6, 0.5, 0.20])
        self.rotate_item_90 = False
        self.keep_tcp_roll_pitch = False
        self.add_placed_item_obstacle = True
        self.samples = []
        self.last_stamp = None
        self.preview = None
        self.collecting = False
        self.locked = False
        self.current_status = 'UNLOCALIZED'

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        # Reuse the existing RViz Marker display. Namespaces and IDs keep the
        # pallet graphics independent from the external-force markers.
        self.marker_pub = self.create_publisher(
            Marker, '/safe_servo/force_marker', 10)
        self.status_pub = self.create_publisher(
            String, '/pallet_localization/status', 10)
        self.config_state_pub = self.create_publisher(
            Float64MultiArray, '/pallet_localization/config_state', 10)
        self.pre_place_pub = self.create_publisher(
            PoseStamped, '/pallet_localization/pre_place_pose', 10)
        self.create_subscription(
            Float64MultiArray, '/pallet_localization/config',
            self.config_callback, 10)
        self.create_service(
            Trigger, '/pallet_localization/start', self.start_callback)
        self.create_service(
            Trigger, '/pallet_localization/lock', self.lock_callback)
        self.create_service(
            Trigger, '/pallet_localization/use_configured_pose',
            self.use_configured_pose_callback)
        self.create_service(
            Trigger, '/pallet_localization/clear', self.clear_callback)
        self._load_config()
        self.create_timer(1.0 / 30.0, self.sample_marker)
        self.create_timer(1.0, self.publish_config_state)
        self.create_timer(1.0, self.republish_status)
        self.publish_status('UNLOCALIZED')

    def config_callback(self, msg):
        if len(msg.data) < 13:
            return
        if not self.collecting and not self.locked:
            self.marker_id = int(round(msg.data[0]))
            self.required_samples = max(5, int(round(msg.data[1])))
            self.position_tolerance = max(0.0005, msg.data[2] / 1000.0)
            self.angle_tolerance = math.radians(max(0.1, msg.data[3]))
            self.pallet_x = max(0.01, msg.data[4] / 1000.0)
            self.pallet_y = max(0.01, msg.data[5] / 1000.0)
            self.offset_rpy = np.radians(np.asarray(msg.data[6:9], dtype=float))
        self.pre_place_xyz = np.asarray(msg.data[9:12], dtype=float) / 1000.0
        self.rotate_item_90 = bool(msg.data[12] > 0.5)
        if len(msg.data) >= 14:
            self.keep_tcp_roll_pitch = bool(msg.data[13] > 0.5)
        if len(msg.data) >= 15:
            self.add_placed_item_obstacle = bool(msg.data[14] > 0.5)
        self._save_config()
        self.publish_config_state()
        if self.locked:
            self.publish_status('LOCKED')
        elif not self.collecting:
            self.publish_status('CONFIGURED AND SAVED')

    def _load_config(self):
        try:
            document = yaml.safe_load(self.config_path.read_text(encoding='utf-8'))
            if not isinstance(document, dict) or document.get('format_version') != 1:
                raise ValueError('unsupported format')
            geometry = document.get('pallet_geometry', {})
            localization = document.get('localization', {})
            if geometry:
                self.pallet_x = float(geometry['length_x_m'])
                self.pallet_y = float(geometry['length_y_m'])
            if localization:
                self.marker_id = int(localization['marker_id'])
                self.required_samples = int(localization['required_samples'])
                self.position_tolerance = float(
                    localization['position_tolerance_m'])
                self.angle_tolerance = math.radians(float(
                    localization['angle_tolerance_deg']))
                self.offset_rpy = np.radians(np.asarray(
                    localization['marker_offset_rpy_deg'], dtype=float))
            pallet = document.get('pallet_pose')
            pre_place = document['pre_place_pose']
            if isinstance(pallet, dict):
                self.configured_position = np.asarray(
                    pallet['position_m'], dtype=float)
                self.configured_q = np.asarray(
                    pallet['orientation_xyzw'], dtype=float)
                self.configured_pose_valid = True
            self.pre_place_xyz = np.asarray(pre_place['position_m'], dtype=float)
            if 'rotate_item_90_deg' in pre_place:
                self.rotate_item_90 = bool(pre_place['rotate_item_90_deg'])
            else:
                legacy_rpy = np.asarray(pre_place.get(
                    'rpy_deg', [180.0, 0.0, 0.0]), dtype=float)
                self.rotate_item_90 = abs(legacy_rpy[2]) > 45.0
            self.keep_tcp_roll_pitch = bool(
                pre_place.get('keep_tcp_roll_pitch', False))
            self.add_placed_item_obstacle = bool(
                pre_place.get('add_placed_item_obstacle', True))
        except FileNotFoundError:
            return
        except (KeyError, TypeError, ValueError, OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f'ignoring invalid pallet config: {exc}')

    def _save_config(self):
        document = {
            'format_version': 1,
            'frame_id': self.base_frame,
            'pallet_geometry': {
                'length_x_m': float(self.pallet_x),
                'length_y_m': float(self.pallet_y),
                'surface_thickness_m': 0.005,
                'origin': 'corner_of_top_face',
            },
            'localization': {
                'marker_id': int(self.marker_id),
                'required_samples': int(self.required_samples),
                'position_tolerance_m': float(self.position_tolerance),
                'angle_tolerance_deg': math.degrees(self.angle_tolerance),
                'marker_offset_rpy_deg': [
                    float(value) for value in np.degrees(self.offset_rpy)],
            },
            'pallet_pose': ({
                'position_m': [float(value) for value in self.configured_position],
                'orientation_xyzw': [float(value) for value in self.configured_q],
            } if self.configured_pose_valid else None),
            'pre_place_pose': {
                'frame_id': 'pallet_frame',
                'position_m': [float(value) for value in self.pre_place_xyz],
                'rotate_item_90_deg': bool(self.rotate_item_90),
                'keep_tcp_roll_pitch': bool(self.keep_tcp_roll_pitch),
                'add_placed_item_obstacle': bool(
                    self.add_placed_item_obstacle),
            },
        }
        temporary = self.config_path.with_suffix('.tmp')
        try:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            temporary.write_text(yaml.safe_dump(document, sort_keys=False),
                                 encoding='utf-8')
            temporary.replace(self.config_path)
        except OSError as exc:
            self.get_logger().error(f'cannot save pallet config: {exc}')

    def publish_config_state(self):
        msg = Float64MultiArray()
        msg.data = [
            float(self.marker_id), float(self.required_samples),
            self.position_tolerance * 1000.0,
            math.degrees(self.angle_tolerance),
            self.pallet_x * 1000.0, self.pallet_y * 1000.0,
            *[float(value) for value in np.degrees(self.offset_rpy)],
            *[float(value) for value in self.pre_place_xyz * 1000.0],
            1.0 if self.rotate_item_90 else 0.0,
            1.0 if self.keep_tcp_roll_pitch else 0.0,
            1.0 if self.add_placed_item_obstacle else 0.0,
        ]
        self.config_state_pub.publish(msg)

    @staticmethod
    def _matrix_rpy(rotation):
        pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
        return np.array([
            math.atan2(rotation[2, 1], rotation[2, 2]), pitch,
            math.atan2(rotation[1, 0], rotation[0, 0])])

    def use_configured_pose_callback(self, _request, response):
        if not self.configured_pose_valid:
            response.message = 'no saved pallet pose; detect or apply config first'
            return response
        self.preview = (self.configured_position.copy(), self.configured_q.copy())
        self.locked = True
        self.publish_preview('pallet_frame')
        self.publish_status('LOCKED')
        response.success = True
        response.message = 'saved pallet pose loaded and locked'
        return response

    def start_callback(self, _, response):
        if self.locked:
            response.success = False
            response.message = 'Clear the locked pallet before detecting again'
            return response
        self.samples = []
        self.preview = None
        self.last_stamp = None
        self.collecting = True
        self.publish_status(f'DETECTING 0/{self.required_samples}')
        response.success = True
        response.message = f'Collecting ArUco marker {self.marker_id}'
        return response

    def sample_marker(self):
        if self.locked and self.preview is not None:
            self.tf_pub.sendTransform(
                self.make_tf('pallet_frame', *self.preview))
            self.publish_pre_place_pose()
        if not self.collecting:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, f'aruco_marker_{self.marker_id}',
                rclpy.time.Time(), timeout=Duration(seconds=0.01))
        except Exception:
            self.publish_status(
                f'DETECTING {len(self.samples)}/{self.required_samples} '
                f'(marker {self.marker_id} not visible)')
            return
        stamp = (tf.header.stamp.sec, tf.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        t = tf.transform.translation
        q = tf.transform.rotation
        self.samples.append((np.array([t.x, t.y, t.z]),
                             np.array([q.x, q.y, q.z, q.w])))
        self.publish_status(
            f'DETECTING {len(self.samples)}/{self.required_samples}')
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
        eigenvalues, eigenvectors = np.linalg.eigh(quaternions.T @ quaternions)
        mean_q = eigenvectors[:, np.argmax(eigenvalues)]
        if np.dot(mean_q, reference) < 0:
            mean_q *= -1
        dots = np.clip(np.abs(quaternions @ mean_q), 0.0, 1.0)
        angle_spread = float(np.max(2.0 * np.arccos(dots)))
        self.collecting = False
        if (position_spread > self.position_tolerance or
                angle_spread > self.angle_tolerance):
            self.samples = []
            self.publish_status(
                f'UNSTABLE: {position_spread*1000:.1f} mm, '
                f'{math.degrees(angle_spread):.1f} deg; retry')
            return
        rotation = quat_matrix(mean_q) @ rpy_matrix(*self.offset_rpy)
        self.preview = (mean_position, matrix_quat(rotation))
        self.publish_preview('pallet_preview')
        self.publish_status(
            f'READY TO LOCK: spread {position_spread*1000:.1f} mm, '
            f'{math.degrees(angle_spread):.1f} deg')

    def lock_callback(self, _, response):
        if self.preview is None:
            response.success = False
            response.message = 'No stable pallet preview; run detection first'
            return response
        position, q = self.preview
        self.locked = True
        self.configured_position = position.copy()
        self.configured_q = q.copy()
        self.configured_pose_valid = True
        self._save_config()
        self.publish_config_state()
        self.publish_preview('pallet_frame')
        self.publish_status('LOCKED')
        response.success = True
        response.message = 'Pallet frame locked for this packing run'
        return response

    def clear_callback(self, _, response):
        self.collecting = False
        self.locked = False
        self.samples = []
        self.preview = None
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)
        self.publish_status('UNLOCALIZED')
        response.success = True
        response.message = 'Pallet localization cleared; detect and lock again'
        return response

    def make_tf(self, child, position, q):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, \
            msg.transform.translation.z = map(float, position)
        msg.transform.rotation.x, msg.transform.rotation.y, \
            msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
        return msg

    def publish_preview(self, frame):
        position, q = self.preview
        if frame == 'pallet_preview':
            self.tf_pub.sendTransform(self.make_tf(frame, position, q))
        if self.locked:
            self.publish_pre_place_pose()
        deck = Marker()
        deck.header.frame_id = frame
        deck.header.stamp = self.get_clock().now().to_msg()
        deck.ns, deck.id = 'pallet', 0
        deck.type, deck.action = Marker.CUBE, Marker.ADD
        deck.pose.position.x = self.pallet_x / 2.0
        deck.pose.position.y = self.pallet_y / 2.0
        deck.pose.position.z = -0.0025
        deck.pose.orientation.w = 1.0
        deck.scale.x, deck.scale.y, deck.scale.z = self.pallet_x, self.pallet_y, 0.005
        if self.locked:
            deck.color.r, deck.color.g, deck.color.b = 0.1, 0.85, 0.25
        else:
            deck.color.r, deck.color.g, deck.color.b = 1.0, 0.65, 0.05
        deck.color.a = 0.32
        self.marker_pub.publish(deck)
        label = Marker()
        label.header = deck.header
        label.ns, label.id = 'pallet', 1
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.z = 0.08
        label.pose.orientation.w = 1.0
        label.scale.z = 0.045
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = 'PALLET LOCKED' if self.locked else 'PALLET PREVIEW'
        self.marker_pub.publish(label)

    def publish_pre_place_pose(self):
        position, q = self.preview
        rotation = quat_matrix(q)
        target_position = position + rotation @ self.pre_place_xyz
        target_rotation = rotation @ rpy_matrix(
            math.pi, 0.0, math.pi / 2.0 if self.rotate_item_90 else 0.0)
        if self.keep_tcp_roll_pitch:
            try:
                tcp_tf = self.tf_buffer.lookup_transform(
                    self.base_frame, 'link_tcp', rclpy.time.Time(),
                    timeout=Duration(seconds=0.02))
            except Exception:
                # Do not publish a target with silently changed roll/pitch.
                # The coordinator will reject planning when the pose becomes
                # stale, which is safer than using the pallet-normal fallback.
                return
            tcp_q = tcp_tf.transform.rotation
            tcp_rpy = self._matrix_rpy(quat_matrix(
                (tcp_q.x, tcp_q.y, tcp_q.z, tcp_q.w)))
            target_yaw = self._matrix_rpy(target_rotation)[2]
            target_rotation = rpy_matrix(
                tcp_rpy[0], tcp_rpy[1], target_yaw)
        target_q = matrix_quat(target_rotation)
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(
            float, target_position)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = map(float, target_q)
        self.pre_place_pub.publish(msg)

    def publish_status(self, text):
        self.current_status = text
        self.republish_status()

    def republish_status(self):
        msg = String()
        msg.data = self.current_status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PalletLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
