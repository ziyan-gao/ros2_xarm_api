"""GPU planning backend for existing MoveIt. No execution/controller APIs."""
import copy
import gc
import os
from pathlib import Path
import tempfile
import threading
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
import torch
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject, MoveItErrorCodes, PlanningSceneComponents)
from moveit_msgs.srv import GetPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from panel_bridge import normalize, HandleProxy
from ros_plan_only import PlanOnlyServer
from live_attachments import (
    DYNAMIC_PAYLOAD_LINK, bake, dynamic_payload_spheres, pose_values,
    reserve_dynamic_payload)
from uf850_smoke import origin


class LiveServer(PlanOnlyServer):
    PALLET_BARRIER_ID = 'cumotion_virtual_pallet_clearance'
    SLOT_BARRIER_ID = 'cumotion_virtual_slot_clearance'

    def __init__(self):
        self.attachments = []
        self.signature = None
        self.payload_signature = None
        self.model_valid = False
        self.robot_sphere_specs = ()
        self.payload_sphere_specs = ()
        self.collision_sphere_pub = None
        self.clearance_barrier_pub = None
        self.clearance_scene_future = None
        self.clearance_scene_timer = None
        self.request_lock = threading.Lock()
        self.model_dir = tempfile.TemporaryDirectory(prefix='cumotion-live-model-')
        root = ET.parse(os.environ['CUMOTION_TEST_MODEL']+'/uf850.urdf').getroot()
        base = [j for j in root.findall('joint') if j.find('child').get('link') == 'link_base']
        if (len(base) != 1 or base[0].get('type') != 'fixed'
                or base[0].find('parent').get('link') != 'world'
                or not np.allclose(origin(base[0].find('origin')), np.eye(4), atol=1e-9)):
            raise ValueError('Live adapter requires identity world/link_base model')
        srdf = ET.parse(os.environ['CUMOTION_TEST_MODEL']+'/uf850.srdf').getroot()
        self.ignored = {frozenset((p.get('link1'), p.get('link2')))
                        for p in srdf.findall('disable_collisions')}
        super().__init__()
        if not self.has_parameter('parallel_finetune'):
            self.declare_parameter('parallel_finetune', False)
        self.parallel_finetune = bool(
            self.get_parameter('parallel_finetune').value)
        self.get_logger().info(
            f'per-request parallel finetune enabled={self.parallel_finetune}')
        barrier_defaults = {
            'clearance_barriers_enabled': True,
            'pallet_clearance_z_m': 0.470,
            'slot_clearance_z_m': 0.480,
            'slot_surface_z_m': 0.0,
            'slot_x_min_m': -0.375,
            'slot_x_max_m': 0.375,
            'slot_y_min_m': 0.180,
            'slot_y_max_m': 0.680,
            'barrier_top_margin_m': 0.003,
        }
        for name, value in barrier_defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, value)
        self.clearance_barriers_enabled = bool(
            self.get_parameter('clearance_barriers_enabled').value)
        self.clearance_barrier_geometry = {
            name: float(self.get_parameter(name).value)
            for name in barrier_defaults
            if name != 'clearance_barriers_enabled'
        }
        self.validate_clearance_barrier_geometry(
            self.clearance_barrier_geometry)
        sphere_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.collision_sphere_pub = self.create_publisher(
            MarkerArray, '/cumotion/collision_spheres', sphere_qos)
        self.clearance_barrier_pub = self.create_publisher(
            MarkerArray, '/cumotion/clearance_barriers', sphere_qos)
        # TRANSIENT_LOCAL retains this snapshot for RViz reconnects. Publish
        # only on actual model/payload changes: repeatedly sending DELETEALL
        # followed by ADD makes the RViz display status and markers flicker.
        self.publish_collision_spheres()
        initial_barriers = (self.build_clearance_barriers(
            [], self.clearance_barrier_geometry)
            if self.clearance_barriers_enabled else [])
        self.publish_clearance_barriers(initial_barriers)
        self.clearance_scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene')
        if self.clearance_barriers_enabled:
            # move_group starts after this GPU backend. Poll asynchronously
            # until its pallet geometry is available, then retain the marker.
            # Actual planning still rebuilds barriers from each request.
            self.clearance_scene_timer = self.create_timer(
                1.0, self.refresh_clearance_barrier_visualization)
        geometry = self.clearance_barrier_geometry
        self.get_logger().info(
            'cuMotion-only clearance barriers enabled=%s '
            'pallet_clearance_above_surface=%.3f m '
            'slot=[%.3f, %.3f]x[%.3f, %.3f] '
            'surface_z=%.3f m clearance_above_pallet=%.3f m margin=%.3f m' % (
                self.clearance_barriers_enabled,
                geometry['pallet_clearance_z_m'],
                geometry['slot_x_min_m'], geometry['slot_x_max_m'],
                geometry['slot_y_min_m'], geometry['slot_y_max_m'],
                geometry['slot_surface_z_m'], geometry['slot_clearance_z_m'],
                geometry['barrier_top_margin_m']))
        ready_file = os.environ.get('CUMOTION_READY_FILE')
        if ready_file:
            ready = Path(ready_file)
            temporary = ready.with_suffix(ready.suffix + '.tmp')
            temporary.write_text(f'pid={os.getpid()} gpu={torch.cuda.get_device_name(0)}\n')
            os.replace(temporary, ready)
            self.get_logger().info(f'cuMotion readiness marker written to {ready}')

    def prepare_robot_config(self, robot):
        prepared = reserve_dynamic_payload(
            bake(robot, self.attachments, self.model_dir.name))
        self.robot_sphere_specs = self.collision_sphere_specs(
            prepared, self.attachments)
        self.publish_collision_spheres()
        return prepared

    @staticmethod
    def collision_sphere_specs(robot, attachments):
        """Return configured cuRobo spheres in ROS TF-visible link frames."""
        kinematics = robot['robot_cfg']['kinematics']
        result = []
        for link_name, spheres in kinematics.get('collision_spheres', {}).items():
            frame_id, kind = link_name, 'robot'
            transform = None
            prefix = 'cumotion_attachment_'
            if link_name.startswith(prefix):
                try:
                    attached = attachments[int(link_name[len(prefix):])]
                except (ValueError, IndexError):
                    continue
                translation, rotation = pose_values(attached.object.pose)
                frame_id, kind = attached.link_name, 'camera'
                transform = lambda center, r=rotation, t=translation: r.apply(center)+t
            for sphere in spheres:
                center = np.asarray(sphere['center'], dtype=float)
                radius = float(sphere['radius'])
                if center.shape != (3,) or not np.isfinite(center).all():
                    continue
                if not np.isfinite(radius) or radius <= 0.:
                    continue
                if transform is not None:
                    center = transform(center)
                result.append((kind, frame_id, *center.tolist(), radius))
        return tuple(result)

    def publish_collision_spheres(self):
        publisher = getattr(self, 'collision_sphere_pub', None)
        if publisher is None:
            return
        message = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        colors = {
            'robot': (0.05, 0.75, 0.95, 0.18),
            'camera': (0.75, 0.20, 0.95, 0.30),
            'payload': (1.00, 0.35, 0.02, 0.38),
        }
        specs = self.robot_sphere_specs + self.payload_sphere_specs
        for marker_id, (kind, frame_id, x, y, z, radius) in enumerate(specs):
            marker = Marker()
            # A zero stamp requests the latest TF. Joint feedback and this
            # debug publisher run independently, so stamping with now can
            # otherwise produce intermittent extrapolation errors in RViz.
            marker.header.frame_id = frame_id
            marker.ns = f'cumotion_{kind}_collision_spheres'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 2.0*radius
            red, green, blue, alpha = colors[kind]
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = alpha
            message.markers.append(marker)
        publisher.publish(message)

    @staticmethod
    def validate_clearance_barrier_geometry(geometry):
        if not all(np.isfinite(value) for value in geometry.values()):
            raise ValueError('Clearance barrier geometry must be finite')
        if geometry['pallet_clearance_z_m'] <= 0.0:
            raise ValueError('Pallet-relative clearance must be positive')
        if geometry['slot_clearance_z_m'] <= 0.0:
            raise ValueError('Slot clearance above pallet must be positive')
        if (geometry['barrier_top_margin_m'] < 0.0 or
                geometry['barrier_top_margin_m'] >= min(
                    geometry['pallet_clearance_z_m'],
                    geometry['slot_clearance_z_m'])):
            raise ValueError('Barrier top margin is invalid')
        if geometry['slot_x_max_m'] <= geometry['slot_x_min_m']:
            raise ValueError('Slot X bounds are reversed or empty')
        if geometry['slot_y_max_m'] <= geometry['slot_y_min_m']:
            raise ValueError('Slot Y bounds are reversed or empty')

    @staticmethod
    def _box_object(object_id, dimensions, pose):
        obj = CollisionObject()
        obj.header.frame_id = 'link_base'
        obj.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in dimensions]
        obj.primitives = [primitive]
        obj.primitive_poses = [copy.deepcopy(pose)]
        obj.pose.orientation.w = 1.0
        obj.operation = CollisionObject.ADD
        return obj

    @staticmethod
    def bake_world_object_pose(obj):
        """Return one object with object.pose folded into primitive poses."""
        baked = copy.deepcopy(obj)
        translation, rotation = pose_values(baked.pose)
        for pose in baked.primitive_poses:
            point = rotation.apply(
                [pose.position.x, pose.position.y, pose.position.z]) + translation
            primitive_rotation = rotation * pose_values(pose)[1]
            quaternion = primitive_rotation.as_quat()
            pose.position.x, pose.position.y, pose.position.z = map(float, point)
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = map(float, quaternion)
        baked.header.frame_id = 'link_base'
        baked.pose = Pose()
        baked.pose.orientation.w = 1.0
        return baked

    @classmethod
    def build_clearance_barriers(cls, world_objects, geometry):
        """Build conservative local barriers without covering the robot base.

        The pallet barrier follows the measured pallet primitive.  The slot
        barrier follows the six-slot footprint, not the complete work table:
        extending the measured work-table cuboid to clearance would contain
        link_base and make every cuMotion start state collide.
        """
        cls.validate_clearance_barrier_geometry(geometry)
        result = []
        pallet = next((obj for obj in world_objects
                       if obj.id == 'pallet_surface'), None)
        # Both configured clearances are distances above the pallet surface,
        # not absolute link_base Z coordinates. Without a localized pallet we
        # cannot place either barrier safely.
        if pallet is None:
            return result
        if (len(pallet.primitives) != 1 or
                len(pallet.primitive_poses) != 1 or
                pallet.primitives[0].type != SolidPrimitive.BOX or
                len(pallet.primitives[0].dimensions) != 3):
            raise ValueError('pallet_surface must be one box primitive')
        dimensions = pallet.primitives[0].dimensions
        source_pose = pallet.primitive_poses[0]
        surface_top = source_pose.position.z + dimensions[2] / 2.0
        margin = geometry['barrier_top_margin_m']
        pallet_barrier_top = (
            surface_top + geometry['pallet_clearance_z_m'] - margin)
        barrier_height = pallet_barrier_top - surface_top
        if barrier_height <= 0.0:
            raise ValueError('Pallet clearance barrier has no height')
        pose = copy.deepcopy(source_pose)
        pose.position.z = surface_top + barrier_height / 2.0
        result.append(cls._box_object(
            cls.PALLET_BARRIER_ID,
            (dimensions[0], dimensions[1], barrier_height), pose))

        x_min, x_max = geometry['slot_x_min_m'], geometry['slot_x_max_m']
        y_min, y_max = geometry['slot_y_min_m'], geometry['slot_y_max_m']
        z_min = geometry['slot_surface_z_m']
        z_max = surface_top + geometry['slot_clearance_z_m'] - margin
        if z_max <= z_min:
            raise ValueError('Slot clearance barrier has no height')
        pose = Pose()
        pose.position.x = (x_min + x_max) / 2.0
        pose.position.y = (y_min + y_max) / 2.0
        pose.position.z = (z_min + z_max) / 2.0
        pose.orientation.w = 1.0
        result.append(cls._box_object(
            cls.SLOT_BARRIER_ID,
            (x_max - x_min, y_max - y_min, z_max - z_min), pose))
        return result

    def publish_clearance_barriers(self, barriers):
        publisher = getattr(self, 'clearance_barrier_pub', None)
        if publisher is None:
            return
        message = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        for marker_id, obj in enumerate(barriers):
            primitive, pose = obj.primitives[0], obj.primitive_poses[0]
            marker = Marker()
            marker.header.frame_id = obj.header.frame_id
            marker.ns = 'cumotion_clearance_barriers'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = copy.deepcopy(pose)
            marker.scale.x, marker.scale.y, marker.scale.z = primitive.dimensions
            marker.color.r = 1.0
            marker.color.g = 0.12 if obj.id == self.PALLET_BARRIER_ID else 0.55
            marker.color.b = 0.04
            marker.color.a = 0.20
            message.markers.append(marker)
        publisher.publish(message)

    def refresh_clearance_barrier_visualization(self):
        """Populate the pallet marker without requiring an initial plan."""
        future = self.clearance_scene_future
        if future is not None:
            if not future.done():
                return
            self.clearance_scene_future = None
            try:
                response = future.result()
                pallet = next((obj for obj in response.scene.world.collision_objects
                               if obj.id == 'pallet_surface'), None)
                if pallet is None:
                    return
                pallet = self.bake_world_object_pose(pallet)
                barriers = self.build_clearance_barriers(
                    [pallet], self.clearance_barrier_geometry)
                self.publish_clearance_barriers(barriers)
                self.get_logger().info(
                    'Published pallet/slot cuMotion clearance barrier markers')
                if self.clearance_scene_timer is not None:
                    self.clearance_scene_timer.cancel()
                return
            except Exception as error:
                self.get_logger().warning(
                    f'Waiting for pallet clearance visualization: {error}',
                    throttle_duration_sec=5.0)
        if not self.clearance_scene_client.service_is_ready():
            return
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES |
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        self.clearance_scene_future = self.clearance_scene_client.call_async(
            request)

    @staticmethod
    def split_attachments(items):
        cameras = [a for a in items if a.object.id == 'eef_camera_d435i']
        payloads = [a for a in items if a.object.id != 'eef_camera_d435i']
        if len(cameras) != 1:
            raise ValueError('Scene must contain exactly one attached camera')
        if len(payloads) > 1:
            raise ValueError('At most one carried payload is supported')
        return cameras, payloads

    def update_dynamic_payload(self, payloads):
        signature = self.attachment_signature(payloads)
        if signature == self.payload_signature:
            return
        if payloads:
            fitted = dynamic_payload_spheres(payloads[0])
            self.payload_sphere_specs = tuple(
                ('payload', DYNAMIC_PAYLOAD_LINK, *row[:3].tolist(), float(row[3]))
                for row in fitted if row[3] > 0.)
            spheres = self.tensor_args.to_device(fitted)
            self.motion_gen.attach_spheres_to_robot(
                sphere_tensor=spheres, link_name=DYNAMIC_PAYLOAD_LINK)
            self.get_logger().info(
                f'Updated dynamic cuMotion payload {payloads[0].object.id} '
                f'with {int((spheres[:, 3] > 0).sum().item())} spheres')
        else:
            self.motion_gen.detach_spheres_from_robot(DYNAMIC_PAYLOAD_LINK)
            self.payload_sphere_specs = ()
            self.get_logger().info('Detached dynamic cuMotion payload')
        self.payload_signature = signature
        self.publish_collision_spheres()

    def release_motion_gen(self):
        """Drop every owning reference before rebuilding the CUDA model.

        MotionGen construction has a large transient allocation. Keeping the
        previous MotionGen/world checker alive while evaluating the right-hand
        side of ``self.motion_gen = MotionGen(...)`` can exhaust a 10 GiB GPU
        when an attached payload changes.
        """
        if torch.cuda.is_available():
            # Finish kernels which may still reference the old model buffers.
            torch.cuda.synchronize()
        previous = getattr(self, 'motion_gen', None)
        self.motion_gen = None
        # The upstream action server owns a second reference to the same CUDA
        # world checker, so clearing only motion_gen is insufficient.
        self._CumotionActionServer__world_collision = None
        del previous
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    @staticmethod
    def attachment_signature(items):
        """Collision semantics only; ignore stamps, posture, weight and operation metadata."""
        def pose(p):
            return (p.position.x, p.position.y, p.position.z,
                    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        result = []
        for attached in items:
            obj = attached.object
            result.append((attached.link_name, tuple(sorted(attached.touch_links)),
                obj.id, obj.header.frame_id, pose(obj.pose),
                tuple((p.type, tuple(p.dimensions)) for p in obj.primitives),
                tuple(pose(p) for p in obj.primitive_poses),
                tuple((tuple(m.vertices), tuple(tuple(t.vertex_indices) for t in m.triangles))
                      for m in obj.meshes), tuple(pose(p) for p in obj.mesh_poses),
                tuple((tuple(p.coef), pose(q)) for p, q in zip(obj.planes, obj.plane_poses)),
                tuple(obj.subframe_names), tuple(pose(p) for p in obj.subframe_poses)))
        return tuple(sorted(result, key=lambda value: value[2]))

    @staticmethod
    def attachment_identity(items):
        """Identity/mount agreement; full complete-scene geometry is authoritative."""
        return tuple(sorted((a.object.id, a.link_name) for a in items))

    def execute_callback(self, handle):
        # Serialize scene/model updates as well as GPU calls.
        if not self.request_lock.acquire(blocking=False):
            handle.abort()
            result = MoveGroup.Result()
            result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
            return result
        try:
            request = copy.deepcopy(handle.request)
            scene = request.planning_options.planning_scene_diff
            if scene.is_diff:
                raise ValueError('cuMotion requires a complete scene')
            attachments = list(scene.robot_state.attached_collision_objects)
            cameras, payloads = self.split_attachments(attachments)
            start_attachments = request.request.start_state.attached_collision_objects
            if (start_attachments and self.attachment_identity(start_attachments)
                    != self.attachment_identity(attachments)):
                raise ValueError('Start attachment identities/mounts differ from complete scene')
            # Geometry is already in world/base or attachment-parent coordinates.
            # Fixed-frame metadata for unrelated camera optical frames is not used.
            if any(o.header.frame_id not in ('world', 'link_base') for o in scene.world.collision_objects):
                raise ValueError('World geometry must be resolved to world/link_base')
            scene.fixed_frame_transforms = [t for t in scene.fixed_frame_transforms
                if t.child_frame_id == 'link_base' and t.header.frame_id == 'world']
            ignored = set(self.ignored)
            for a in attachments:
                ignored.update(frozenset((a.object.id, n)) for n in a.touch_links)
            scene.robot_state.attached_collision_objects = []
            request.request.start_state.attached_collision_objects = []
            request = normalize(request, ignored)
            scene = request.planning_options.planning_scene_diff
            reserved_ids = {self.PALLET_BARRIER_ID, self.SLOT_BARRIER_ID}
            scene.world.collision_objects = [
                obj for obj in scene.world.collision_objects
                if obj.id not in reserved_ids]
            barriers = []
            if self.clearance_barriers_enabled:
                barriers = self.build_clearance_barriers(
                    scene.world.collision_objects,
                    self.clearance_barrier_geometry)
                if len(barriers) != 2:
                    raise ValueError(
                        "clearance barriers enabled but pallet/slot virtual obstacles "
                        f"could not be built (count={len(barriers)}); refusing unprotected planning")
                # These are the only clearance-height constraints. Keep them
                # present for empty-tool and carried-item cuMotion requests;
                # Cartesian vertical approach/descent is planned separately.
                scene.world.collision_objects.extend(barriers)
            self.publish_clearance_barriers(barriers)
            # The camera changes the robot topology and is baked once. Payload
            # geometry uses preallocated link_tcp spheres and never rebuilds
            # MotionGen or its CUDA graphs.
            signature = self.attachment_signature(cameras)
            if signature != self.signature or not self.model_valid:
                self.model_valid = False
                self.attachments = cameras
                self.get_logger().info('Rebuilding GPU collision model for fixed camera')
                self.release_motion_gen()
                self.load_motion_gen()
                self.warmup()
                self.signature, self.model_valid = signature, True
                self.payload_signature = None
            self.update_dynamic_payload(payloads)
            return super().execute_callback(HandleProxy(handle, request))
        except Exception as error:
            self.get_logger().error(f'Live scene rejected: {error}')
            handle.abort()
            result = MoveGroup.Result()
            result.error_code.val = MoveItErrorCodes.INVALID_MOTION_PLAN
            return result
        finally:
            self.request_lock.release()


def main():
    rclpy.init()
    node = LiveServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
