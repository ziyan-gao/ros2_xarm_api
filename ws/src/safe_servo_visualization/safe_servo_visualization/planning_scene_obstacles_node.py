import json
import math

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, ObjectColor, PlanningScene)
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class PlanningSceneObstacles(Node):
    """Maintain environment geometry and the item carried by the TCP."""

    def __init__(self):
        super().__init__('planning_scene_obstacles')
        self.base_frame = 'link_base'
        self.pallet_x, self.pallet_y, self.pallet_thickness = 1.2, 1.0, 0.005
        self.static_applied = self.pallet_locked = self.pallet_applied = False
        self.apply_pending = self.attachment_pending = False
        self.pregrasp_snapshot = None
        self.pending_pickup_operation_id = None
        self.last_attached_pickup_operation_id = -1
        self.attached_item_id = ''
        self.attached_item_size = None
        self.attached_item_center = None
        self.attached_item_orientation = None
        self.placed_item_ids = []
        self.placed_item_counter = 0
        self.last_placement_error = ''
        self.last_placed_item_pose_source = ''
        self.motion_state = None
        self.add_placed_item_obstacle = True
        self.place_singularity_fallback = False
        self.random_loading_target = None
        self.random_loading_status = {}
        self.touch_links = [
            'link_tcp', 'link_eef', 'ft_sensor_link',
            'xarm_vacuum_gripper_link']
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.apply_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.create_subscription(
            Float64MultiArray, '/pallet_localization/config',
            self.pallet_config_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/pallet_localization/config_state',
            self.pallet_config_callback, 10)
        self.create_subscription(
            String, '/pallet_localization/status', self.pallet_status_callback, 10)
        self.create_subscription(
            String, '/motion_coordinator/status', self.motion_status_callback, 10)
        self.create_subscription(
            String, '/pickup_supervisor/status', self.pickup_status_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/random_stable_loading/target',
            self.random_loading_target_callback, 10)
        self.create_subscription(
            String, '/random_stable_loading/status',
            self.random_loading_status_callback, 10)
        self.create_service(
            Trigger, '/planning_scene_obstacles/detach_item',
            self.detach_item_callback)
        self.create_service(
            Trigger, '/planning_scene_obstacles/clear_placed_items',
            self.clear_placed_items_callback)
        self.status_pub = self.create_publisher(
            String, '/planning_scene_obstacles/status', 10)
        self.create_timer(0.5, self.ensure_scene)

    def _box(self, object_id, size, center, orientation=None):
        obj = CollisionObject()
        obj.header.frame_id, obj.id = self.base_frame, object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in size]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, center)
        if orientation is None:
            pose.orientation.w = 1.0
        else:
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w) = map(float, orientation)
        obj.primitives, obj.primitive_poses = [primitive], [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _table_objects(self):
        # The robot base is yawed -90 degrees relative to the frame in which
        # the table measurements were taken. Express the unchanged tables in
        # link_base with the inverse (+90 degree) rotation.
        table_orientation = (0.0, 0.0, math.sin(math.pi / 4.0),
                             math.cos(math.pi / 4.0))
        # Treat the measured 1.0 m distance as the clearance from link_base to
        # the wall's nearest face.  The collision box therefore starts at
        # x=-1.0 m and extends 50 mm farther in the negative-X direction.
        negative_x_wall_thickness = 0.05
        # The positive-Y wall uses the same convention: its nearest face is
        # y=+1.2 m and its collision volume extends away from the robot.
        positive_y_wall_thickness = 0.05
        return [
            self._box('work_table', (1.5, 2.5, 1.3),
                      (0.6, 0.6, -0.03 - 1.3 / 2.0), table_orientation),
            self._box('secondary_table', (1.3, 2.5, 1.4),
                      (0.2, -2.5, 0.1 - 1.4 / 2.0), table_orientation),
            self._box(
                'negative_x_wall',
                (negative_x_wall_thickness, 6.0, 3.0),
                (-1.0 - negative_x_wall_thickness / 2.0, -1.0, 0.0)),
            self._box(
                'positive_y_wall',
                (6.0, positive_y_wall_thickness, 3.0),
                (0.0, 1.2 + positive_y_wall_thickness / 2.0, 0.0)),
        ]

    def _apply(self, objects, description, on_success=None):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        return self._apply_scene(scene, description, on_success)

    def _apply_scene(self, scene, description, on_success=None):
        if self.apply_pending or not self.apply_client.service_is_ready():
            return False
        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.apply_pending = True
        future = self.apply_client.call_async(request)
        future.add_done_callback(
            lambda done: self._apply_completed(done, description, on_success))
        return True

    def _apply_completed(self, future, description, on_success):
        self.apply_pending = False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'failed to apply {description}: {exc}')
            return
        if result is None or not result.success:
            self.get_logger().error(f'MoveIt rejected {description}')
            return
        if on_success:
            on_success()
        self.get_logger().info(f'applied {description}')

    def ensure_scene(self):
        if not self.static_applied:
            self._apply(self._table_objects(), 'fixed workspace collision objects',
                        lambda: setattr(self, 'static_applied', True))
            return
        if self.pallet_locked and not self.pallet_applied:
            self._apply_locked_pallet()
            return
        if self.attachment_pending and not self.attached_item_id:
            self._attach_detected_item()
        self.publish_status()

    def motion_status_callback(self, message):
        try:
            status = json.loads(message.data)
            self.motion_state = status.get('state')
            snapshot = status.get('planned_pregrasp')
            if isinstance(snapshot, dict):
                self._validate_snapshot(snapshot)
                self.pregrasp_snapshot = dict(snapshot)
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return

    def pickup_status_callback(self, message):
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if status.get('operation_kind') == 'place':
            self.place_singularity_fallback = bool(
                status.get('place_fallback_used'))
        # Attachment is a one-shot event belonging to a specific pickup
        # operation. A queued status from the preceding cycle must not re-arm
        # attachment after detachment and capture the next item's transform
        # while the robot is still moving to pre-grasp.
        if (status.get('operation_kind') != 'pickup' or
                status.get('dry_run', True) or
                not status.get('contact_detected') or
                not status.get('vacuum_verified')):
            return
        try:
            pickup_operation_id = int(status['operation_id'])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning(
                'ignored verified pickup status without a valid operation_id')
            return
        if pickup_operation_id <= self.last_attached_pickup_operation_id:
            return
        if (self.pending_pickup_operation_id is not None and
                self.pending_pickup_operation_id != pickup_operation_id):
            self.get_logger().warning(
                'ignored pickup attachment operation %d while operation %d '
                'is still pending' % (
                    pickup_operation_id,
                    self.pending_pickup_operation_id))
            return
        if self.attached_item_id:
            return
        if not self.attachment_pending:
            self.get_logger().info(
                f'arming item attachment for pickup operation '
                f'{pickup_operation_id}')
        self.pending_pickup_operation_id = pickup_operation_id
        self.attachment_pending = True
        self._attach_detected_item()

    def random_loading_target_callback(self, message):
        if len(message.data) < 12:
            self.get_logger().warning(
                'ignored incomplete random stable-loading target for '
                'fallback obstacle placement')
            return
        values = tuple(map(float, message.data[:12]))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                'ignored non-finite random stable-loading target')
            return
        if any(value <= 0.0 for value in values[6:9]):
            self.get_logger().warning(
                'ignored random stable-loading target with invalid dimensions')
            return
        self.random_loading_target = {
            'sequence_id': int(round(values[0])),
            'item_id': int(round(values[1])),
            'corner_m': tuple(value / 1000.0 for value in values[2:5]),
            'rotated': values[5] > 0.5,
            'size_m': tuple(value / 1000.0 for value in values[6:9]),
        }

    def random_loading_status_callback(self, message):
        try:
            status = json.loads(message.data)
            self.random_loading_status = (
                status if isinstance(status, dict) else {})
        except (json.JSONDecodeError, TypeError):
            self.random_loading_status = {}

    @staticmethod
    def _validate_snapshot(snapshot):
        fields = ('box_id', 'x_m', 'y_m', 'center_z_m', 'size_x_m',
                  'size_y_m', 'size_z_m', 'yaw_rad')
        values = [float(snapshot[key]) for key in fields[1:]]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('non-finite item snapshot')
        if any(float(snapshot[key]) <= 0.0 for key in fields[4:7]):
            raise ValueError('non-positive item dimension')
        int(snapshot['box_id'])

    def _attach_detected_item(self):
        if (self.apply_pending or self.attached_item_id
                or not self.attachment_pending
                or self.pregrasp_snapshot is None):
            return
        snapshot = self.pregrasp_snapshot
        object_id = f"carried_item_{int(snapshot['box_id'])}"
        try:
            # This TF maps link_base coordinates into link_tcp coordinates.
            tf = self.tf_buffer.lookup_transform(
                'link_tcp', self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.25))
        except TransformException as exc:
            self.get_logger().warning(f'cannot attach item without TCP TF: {exc}')
            return
        q = tf.transform.rotation
        tcp_q = (q.x, q.y, q.z, q.w)
        yaw = float(snapshot['yaw_rad'])
        item_q = self._multiply(
            tcp_q, (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
        item_size = (
            snapshot['size_x_m'], snapshot['size_y_m'], snapshot['size_z_m'])
        # The suction TCP is the confirmed top-face contact point. Deriving
        # the vertical attachment offset from the earlier depth center and a
        # later TF sample made that offset sensitive to depth/TF disagreement
        # and could consume the intended pre-place clearance. The pre-grasp is
        # centered laterally, so model the object center as half its measured
        # height below the contact point in the object's local frame.
        center = self._rotate((0.0, 0.0, -item_size[2] / 2.0), item_q)
        obj = self._box(object_id, item_size, center, item_q)
        obj.header.frame_id = 'link_tcp'
        attached = AttachedCollisionObject()
        attached.link_name, attached.touch_links = 'link_tcp', self.touch_links
        attached.object = obj
        scene = PlanningScene()
        scene.is_diff = scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        pickup_operation_id = self.pending_pickup_operation_id
        self._apply_scene(
            scene, f'attached collision object {object_id}',
            lambda: self._attachment_succeeded(
                object_id, item_size, center, item_q,
                pickup_operation_id))

    def _attachment_succeeded(
            self, object_id, item_size, center, orientation,
            pickup_operation_id):
        self.attached_item_id, self.attachment_pending = object_id, False
        self.attached_item_size = tuple(map(float, item_size))
        self.attached_item_center = tuple(map(float, center))
        self.attached_item_orientation = tuple(map(float, orientation))
        if pickup_operation_id is not None:
            self.last_attached_pickup_operation_id = max(
                self.last_attached_pickup_operation_id,
                int(pickup_operation_id))
        self.pending_pickup_operation_id = None
        self.last_placement_error = ''
        self.place_singularity_fallback = False
        self.publish_status()

    def _placed_item_from_attached(self):
        if (self.attached_item_size is None or
                self.attached_item_center is None or
                self.attached_item_orientation is None):
            raise ValueError('attached-item geometry is unavailable')
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, 'link_tcp', rclpy.time.Time(),
            timeout=Duration(seconds=0.25))
        t, q = tf.transform.translation, tf.transform.rotation
        tcp_q = (q.x, q.y, q.z, q.w)
        offset = self._rotate(self.attached_item_center, tcp_q)
        center = (t.x + offset[0], t.y + offset[1], t.z + offset[2])
        orientation = self._multiply(
            tcp_q, self.attached_item_orientation)
        placed_id = f'placed_item_{self.placed_item_counter + 1}'
        return self._box(
            placed_id, self.attached_item_size, center, orientation)

    def _active_random_fallback_target(self):
        target = self.random_loading_target
        if not self.place_singularity_fallback or target is None:
            return None
        if self.random_loading_status.get('state') not in (
                'STARTING', 'EXECUTING'):
            return None
        try:
            pending_sequence = int(
                self.random_loading_status.get('pending_sequence_id'))
            attached_item = int(self.pregrasp_snapshot['box_id'])
        except (KeyError, TypeError, ValueError):
            return None
        if (pending_sequence != target['sequence_id'] or
                attached_item != target['item_id']):
            return None
        return target

    def _placed_item_from_random_target(self, target):
        tf = self.tf_buffer.lookup_transform(
            self.base_frame, 'pallet_frame', rclpy.time.Time(),
            timeout=Duration(seconds=0.25))
        t, q = tf.transform.translation, tf.transform.rotation
        pallet_q = (q.x, q.y, q.z, q.w)
        size_x, size_y, size_z = target['size_m']
        corner_x, corner_y, corner_z = target['corner_m']
        if target['rotated']:
            # The placement flag now means -90 degrees around pallet Z. The
            # target still denotes the rotated footprint's minimum X/Y/bottom
            # corner, so its center offsets are (original Y/2, original X/2).
            local_center = (
                corner_x + size_y / 2.0,
                corner_y + size_x / 2.0,
                corner_z + size_z / 2.0)
            item_yaw_q = (
                0.0, 0.0, math.sin(-math.pi / 4.0),
                math.cos(-math.pi / 4.0))
            orientation = self._multiply(pallet_q, item_yaw_q)
        else:
            local_center = (
                corner_x + size_x / 2.0,
                corner_y + size_y / 2.0,
                corner_z + size_z / 2.0)
            orientation = pallet_q
        offset = self._rotate(local_center, pallet_q)
        center = (t.x + offset[0], t.y + offset[1], t.z + offset[2])
        placed_id = f'placed_item_{self.placed_item_counter + 1}'
        return self._box(
            placed_id, target['size_m'], center, orientation)

    def detach_item_callback(self, _request, response):
        if not self.attached_item_id:
            response.success, response.message = False, 'no attached item'
            return response
        if self.apply_pending:
            response.success = False
            response.message = 'planning-scene update is busy; retry detach'
            return response
        attached = AttachedCollisionObject()
        attached.link_name = 'link_tcp'
        attached.object.id = self.attached_item_id
        attached.object.operation = CollisionObject.REMOVE
        # Remove any world-scene instance with the same ID as well. Depending
        # on the MoveIt scene update order, detaching can otherwise leave a
        # residual world object at the TCP and the next plan starts in
        # collision with the vacuum gripper.
        world_object = CollisionObject()
        world_object.header.frame_id = self.base_frame
        world_object.id = self.attached_item_id
        world_object.operation = CollisionObject.REMOVE
        placed_object = None
        if self.add_placed_item_obstacle:
            try:
                predicted_target = self._active_random_fallback_target()
                if predicted_target is not None:
                    placed_object = self._placed_item_from_random_target(
                        predicted_target)
                    self.last_placed_item_pose_source = (
                        'random_stable_loading_target')
                    self.get_logger().warning(
                        'place singularity fallback: assigning placed-item '
                        'obstacle to the predicted random-loading target pose')
                else:
                    placed_object = self._placed_item_from_attached()
                    self.last_placed_item_pose_source = 'measured_release_pose'
                self.last_placement_error = ''
            except (TransformException, ValueError) as exc:
                # The physical release must still be allowed to complete. Keep
                # the scene internally consistent by removing the attached
                # object, but expose the missing obstacle in status and logs.
                self.last_placement_error = str(exc)
                self.last_placed_item_pose_source = ''
                self.get_logger().error(
                    f'cannot create placed-item collision object: {exc}')
        else:
            self.last_placement_error = ''
            self.last_placed_item_pose_source = ''
        scene = PlanningScene()
        scene.is_diff = scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        scene.world.collision_objects = [world_object]
        if placed_object is not None:
            scene.world.collision_objects.append(placed_object)
            color = ObjectColor()
            color.id = placed_object.id
            color.color.r = 0.95
            color.color.g = 0.65
            color.color.b = 0.10
            color.color.a = 0.90
            scene.object_colors = [color]
        item_id = self.attached_item_id
        placed_id = '' if placed_object is None else placed_object.id
        accepted = self._apply_scene(
            scene, f'placed-item scene update for {item_id}',
            lambda: self._detachment_succeeded(placed_id))
        response.success = accepted
        if accepted and placed_id:
            response.message = (
                f'detach requested for {item_id}; adding {placed_id}')
        elif accepted:
            suffix = ('disabled by place configuration'
                      if not self.add_placed_item_obstacle
                      else 'placed obstacle unavailable')
            response.message = f'detach requested for {item_id}; {suffix}'
        else:
            response.message = 'apply_planning_scene is unavailable'
        return response

    def _detachment_succeeded(self, placed_id):
        self.attached_item_id = ''
        self.attached_item_size = None
        self.attached_item_center = None
        self.attached_item_orientation = None
        self.pregrasp_snapshot = None
        self.attachment_pending = False
        self.pending_pickup_operation_id = None
        self.place_singularity_fallback = False
        if placed_id:
            self.placed_item_counter += 1
            self.placed_item_ids.append(placed_id)
        self.publish_status()

    def clear_placed_items_callback(self, _request, response):
        if self.apply_pending:
            response.message = 'planning-scene update is busy; retry clear'
            return response
        if self.motion_state not in (None, 'IDLE', 'SUCCEEDED', 'FAULT'):
            response.message = (
                f'cannot clear obstacles while MoveIt is in '
                f'{self.motion_state}')
            return response
        if self.attached_item_id or self.attachment_pending:
            response.message = (
                'cannot clear placed obstacles while an item is attached or '
                'awaiting attachment')
            return response
        if not self.placed_item_ids:
            response.success = True
            response.message = 'no placed-item obstacles to clear'
            return response
        objects = []
        for object_id in self.placed_item_ids:
            obj = CollisionObject()
            obj.header.frame_id = self.base_frame
            obj.id = object_id
            obj.operation = CollisionObject.REMOVE
            objects.append(obj)
        count = len(objects)
        accepted = self._apply(
            objects, f'removal of {count} placed-item obstacles',
            self._placed_items_cleared)
        response.success = accepted
        response.message = (
            f'clearing {count} placed-item obstacles'
            if accepted else 'apply_planning_scene is unavailable')
        return response

    def _placed_items_cleared(self):
        self.placed_item_ids = []
        self.last_placement_error = ''
        self.last_placed_item_pose_source = ''
        self.publish_status()

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'static_applied': self.static_applied,
            'pallet_applied': self.pallet_applied,
            'attachment_pending': self.attachment_pending,
            'pending_pickup_operation_id': self.pending_pickup_operation_id,
            'last_attached_pickup_operation_id':
                self.last_attached_pickup_operation_id,
            'attached_item_id': self.attached_item_id,
            'attached_item_size_m': self.attached_item_size,
            'attached_item_center_in_tcp_m': self.attached_item_center,
            'attached_item_orientation_in_tcp_xyzw':
                self.attached_item_orientation,
            'placed_item_ids': list(self.placed_item_ids),
            'placed_item_count': len(self.placed_item_ids),
            'add_placed_item_obstacle': self.add_placed_item_obstacle,
            'last_placement_error': self.last_placement_error,
            'last_placed_item_pose_source': self.last_placed_item_pose_source,
        }, separators=(',', ':'))
        self.status_pub.publish(message)

    def pallet_config_callback(self, message):
        if len(message.data) < 6:
            return
        if not self.pallet_applied:
            x = float(message.data[4]) / 1000.0
            y = float(message.data[5]) / 1000.0
            if math.isfinite(x) and math.isfinite(y) and x > 0.0 and y > 0.0:
                self.pallet_x, self.pallet_y = x, y
        if len(message.data) >= 15:
            self.add_placed_item_obstacle = bool(message.data[14] > 0.5)

    def pallet_status_callback(self, message):
        if message.data == 'LOCKED':
            self.pallet_locked = True
            if not self.pallet_applied:
                self._apply_locked_pallet()
        elif message.data == 'UNLOCALIZED':
            self.pallet_locked = False
            if self.pallet_applied or self.placed_item_ids:
                obj = CollisionObject()
                obj.header.frame_id, obj.id = self.base_frame, 'pallet_surface'
                obj.operation = CollisionObject.REMOVE
                objects = [obj]
                for placed_id in self.placed_item_ids:
                    placed = CollisionObject()
                    placed.header.frame_id = self.base_frame
                    placed.id = placed_id
                    placed.operation = CollisionObject.REMOVE
                    objects.append(placed)
                self._apply(
                    objects, 'pallet and placed-item collision-object removal',
                    self._pallet_objects_removed)

    def _pallet_objects_removed(self):
        self.pallet_applied = False
        self.placed_item_ids = []
        self.last_placement_error = ''
        self.last_placed_item_pose_source = ''
        self.publish_status()

    def _apply_locked_pallet(self):
        if self.apply_pending:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, 'pallet_frame', rclpy.time.Time(),
                timeout=Duration(seconds=0.25))
        except TransformException as exc:
            self.get_logger().error(f'locked pallet TF unavailable: {exc}')
            return
        t, q = tf.transform.translation, tf.transform.rotation
        offset = self._rotate(
            (self.pallet_x / 2.0, self.pallet_y / 2.0,
             -self.pallet_thickness / 2.0), (q.x, q.y, q.z, q.w))
        obj = self._box(
            'pallet_surface',
            (self.pallet_x, self.pallet_y, self.pallet_thickness),
            (t.x + offset[0], t.y + offset[1], t.z + offset[2]),
            (q.x, q.y, q.z, q.w))
        self._apply([obj], 'locked pallet collision surface',
                    lambda: setattr(self, 'pallet_applied', True))

    @staticmethod
    def _rotate(vector, quaternion):
        x, y, z, w = map(float, quaternion)
        vx, vy, vz = map(float, vector)
        tx, ty, tz = (2.0 * (y * vz - z * vy),
                      2.0 * (z * vx - x * vz),
                      2.0 * (x * vy - y * vx))
        return (vx + w * tx + y * tz - z * ty,
                vy + w * ty + z * tx - x * tz,
                vz + w * tz + x * ty - y * tx)

    @staticmethod
    def _multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
                lw * rw - lx * rx - ly * ry - lz * rz)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
