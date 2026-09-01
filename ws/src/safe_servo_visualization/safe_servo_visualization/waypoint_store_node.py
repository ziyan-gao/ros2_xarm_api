from collections import deque
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class WaypointStore(Node):
    """Safely capture operator-taught UF850 configurations to versioned YAML."""

    FORMAT_VERSION = 1

    def __init__(self):
        super().__init__('waypoint_store')
        self.declare_parameter(
            'storage_path', '/workspace/config/taught_waypoints.yaml')
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('tcp_frame', 'link_eef')
        self.declare_parameter('joint_state_max_age_sec', 0.5)
        self.declare_parameter('stationary_window_sec', 0.5)
        self.declare_parameter('stationary_velocity_rad_sec', 0.01)

        self.storage_path = Path(
            self.get_parameter('storage_path').value).expanduser()
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tcp_frame = str(self.get_parameter('tcp_frame').value)
        self.max_age = float(
            self.get_parameter('joint_state_max_age_sec').value)
        self.stationary_window = float(
            self.get_parameter('stationary_window_sec').value)
        self.velocity_limit = float(
            self.get_parameter('stationary_velocity_rad_sec').value)

        self.expected_joints = [f'joint{i}' for i in range(1, 7)]
        self.samples = deque()
        self.last_receipt_time = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.status_pub = self.create_publisher(
            String, '/taught_waypoints/status', 10)

        # The normal SDK driver publishes the namespaced topic, while the
        # MoveIt stack republishes the consolidated /joint_states topic.
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(
            JointState, '/ufactory/joint_states',
            self.joint_state_callback, 10)
        self.create_service(
            Trigger, '/taught_waypoints/save_observation',
            lambda request, response: self.save('observation', response))
        self.create_service(
            Trigger, '/taught_waypoints/save_intermediate',
            lambda request, response: self.save('intermediate', response))
        self.create_service(
            Trigger, '/taught_waypoints/reload', self.reload_callback)
        self.create_timer(2.0, self.publish_summary)
        self.publish_status(f'Ready; storage={self.storage_path}')

    def joint_state_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.expected_joints):
            return
        ordered = tuple(float(positions[name]) for name in self.expected_joints)
        if not all(math.isfinite(value) for value in ordered):
            return

        velocities = dict(zip(msg.name, msg.velocity))
        reported_velocity = max(
            (abs(float(velocities.get(name, 0.0)))
             for name in self.expected_joints), default=0.0)
        now = time.monotonic()
        self.last_receipt_time = now
        self.samples.append((now, ordered, reported_velocity))
        cutoff = now - max(self.stationary_window * 2.0, 1.0)
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def _validated_joint_positions(self):
        now = time.monotonic()
        if self.last_receipt_time is None:
            raise ValueError('no UF850 joint state received')
        age = now - self.last_receipt_time
        if age > self.max_age:
            raise ValueError(f'joint state is stale ({age:.2f} s)')
        if len(self.samples) < 2:
            raise ValueError('not enough joint samples to confirm robot is stopped')

        recent = [sample for sample in self.samples
                  if sample[0] >= now - self.stationary_window]
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < (
                self.stationary_window * 0.8):
            raise ValueError('wait for the stationary validation window')
        if max(sample[2] for sample in recent) > self.velocity_limit:
            raise ValueError('robot is moving; waypoint was not saved')

        max_estimated_velocity = 0.0
        for previous, current in zip(recent, recent[1:]):
            dt = current[0] - previous[0]
            if dt <= 0.0:
                continue
            max_estimated_velocity = max(
                max_estimated_velocity,
                max(abs(a - b) / dt
                    for a, b in zip(current[1], previous[1])))
        if max_estimated_velocity > self.velocity_limit:
            raise ValueError('robot is moving; waypoint was not saved')
        return list(recent[-1][1])

    def _tcp_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.25))
        except TransformException as exc:
            raise ValueError(
                f'TF {self.base_frame}->{self.tcp_frame} unavailable: {exc}')
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = [translation.x, translation.y, translation.z,
                  rotation.x, rotation.y, rotation.z, rotation.w]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('TCP transform contains a non-finite value')
        return {
            'frame_id': self.base_frame,
            'child_frame_id': self.tcp_frame,
            'position_m': {
                'x': float(translation.x), 'y': float(translation.y),
                'z': float(translation.z)},
            'orientation_xyzw': {
                'x': float(rotation.x), 'y': float(rotation.y),
                'z': float(rotation.z), 'w': float(rotation.w)},
        }

    def _read_document(self):
        if not self.storage_path.exists():
            return {'format_version': self.FORMAT_VERSION,
                    'robot': {'type': 'uf850', 'dof': 6},
                    'waypoints': {}}
        try:
            with self.storage_path.open('r', encoding='utf-8') as stream:
                document = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f'cannot read waypoint file: {exc}')
        if not isinstance(document, dict):
            raise ValueError('waypoint file root must be a mapping')
        if document.get('format_version') != self.FORMAT_VERSION:
            raise ValueError('unsupported waypoint file format version')
        if not isinstance(document.get('waypoints'), dict):
            raise ValueError('waypoint file has no valid waypoints mapping')
        return document

    def _atomic_write(self, document):
        self.storage_path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f'.{self.storage_path.name}.', suffix='.tmp',
            dir=self.storage_path.parent)
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(document, stream, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary_name, 0o644)
            os.replace(temporary_name, self.storage_path)
        except Exception:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

    def save(self, name, response):
        try:
            positions = self._validated_joint_positions()
            tcp_pose = self._tcp_pose()
            document = self._read_document()
            document['waypoints'][name] = {
                'saved_at': datetime.now(timezone.utc).isoformat(),
                'joint_names': self.expected_joints,
                'positions_rad': positions,
                'tcp_pose': tcp_pose,
            }
            self._atomic_write(document)
            response.success = True
            response.message = (
                f'Saved {name} waypoint to {self.storage_path}')
        except (OSError, ValueError, yaml.YAMLError) as exc:
            response.success = False
            response.message = f'Cannot save {name}: {exc}'
        self.publish_status(response.message)
        return response

    def reload_callback(self, request, response):
        del request
        try:
            document = self._read_document()
            names = sorted(document['waypoints'])
            response.success = True
            response.message = ('Loaded: ' + ', '.join(names)) if names else (
                'Waypoint file is valid but contains no waypoints')
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        self.publish_status(response.message)
        return response

    def publish_summary(self):
        try:
            document = self._read_document()
            names = sorted(document['waypoints'])
            self.publish_status(
                'Saved waypoints: ' + (', '.join(names) if names else 'none'))
        except ValueError as exc:
            self.publish_status(f'Waypoint file error: {exc}')

    def publish_status(self, text):
        message = String()
        message.data = text
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointStore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
