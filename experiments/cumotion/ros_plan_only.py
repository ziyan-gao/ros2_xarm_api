"""Experimental ROS action adapter. No robot execution; unsupported inputs fail closed."""
import math
import json
import time
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from isaac_ros_cumotion.cumotion_planner import CumotionActionServer
from curobo.types.state import JointState as CuJointState
from curobo.types.math import Pose as CuPose
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
from curobo.geom.types import WorldConfig
from curobo.geom.sdf.world import CollisionCheckerType
from isaac_ros_cumotion.update_kinematics import get_robot_config
import torch
from scipy.spatial.transform import Rotation


UF850_JOINTS = {f'joint{i}' for i in range(1, 7)}


def diagnostic_scalar(value):
    """Convert optional cuMotion timing/counter fields without breaking planning."""
    if value is None:
        return None
    try:
        if hasattr(value, 'item'):
            value = value.item()
        return float(value)
    except (TypeError, ValueError, RuntimeError):
        return str(value)


def goal_kind(request):
    if len(request.goal_constraints) != 1:
        return None
    goal = request.goal_constraints[0]
    has_joints = bool(goal.joint_constraints)
    has_pose = bool(goal.position_constraints or goal.orientation_constraints)
    if has_joints and not has_pose:
        return 'joint'
    if (not has_joints and len(goal.position_constraints) == 1
            and len(goal.orientation_constraints) == 1):
        return 'pose'
    return None


def unsupported(goal):
    request, options = goal.request, goal.planning_options
    scene = options.planning_scene_diff
    if not options.plan_only:
        return 'Only plan_only requests are permitted'
    if request.group_name != 'uf850':
        return 'Expected uf850 planning group'
    if request.start_state.is_diff or not request.start_state.joint_state.name:
        return 'An explicit absolute start state is required'
    kind = goal_kind(request)
    if kind is None:
        return 'Expected one complete joint goal or one position-plus-orientation goal'
    js = request.start_state.joint_state
    if len(js.name) != 6 or set(js.name) != UF850_JOINTS or len(js.position) != 6:
        return 'Start must contain exactly the six UF850 joints'
    if not all(math.isfinite(v) for v in js.position):
        return 'Non-finite joint values'
    goal = request.goal_constraints[0]
    if kind == 'joint':
        goals = goal.joint_constraints
        if len(goals) != 6 or {g.joint_name for g in goals} != UF850_JOINTS:
            return 'Joint goal must contain exactly the six UF850 joints'
        if not all(math.isfinite(g.position) for g in goals):
            return 'Non-finite joint goal'
        if any(not math.isfinite(t) or t < 0. for g in goals
               for t in (g.tolerance_above, g.tolerance_below)):
            return 'Invalid joint goal tolerance'
    else:
        position = goal.position_constraints[0]
        orientation = goal.orientation_constraints[0]
        if (position.header.frame_id not in ('world', 'link_base')
                or orientation.header.frame_id not in ('world', 'link_base')):
            return 'Pose goal must be expressed in world/link_base'
        if not position.link_name or position.link_name != orientation.link_name:
            return 'Pose goal position/orientation link names must match'
        offset = position.target_point_offset
        if any(abs(v) > 1e-9 for v in (offset.x, offset.y, offset.z)):
            return 'Pose goal target-point offsets are unsupported'
        region = position.constraint_region
        if len(region.primitives) != 1 or len(region.primitive_poses) != 1:
            return 'Pose goal requires one primitive tolerance region'
        primitive = region.primitives[0]
        if ((primitive.type == primitive.BOX and len(primitive.dimensions) != 3)
                or (primitive.type == primitive.SPHERE and len(primitive.dimensions) != 1)
                or primitive.type not in (primitive.BOX, primitive.SPHERE)
                or any(not math.isfinite(v) or v < 0. for v in primitive.dimensions)):
            return 'Invalid pose-goal position tolerance region'
        pose = region.primitive_poses[0]
        q = pose.orientation
        oq = orientation.orientation
        values = [pose.position.x, pose.position.y, pose.position.z,
                  q.x, q.y, q.z, q.w, oq.x, oq.y, oq.z, oq.w,
                  orientation.absolute_x_axis_tolerance,
                  orientation.absolute_y_axis_tolerance,
                  orientation.absolute_z_axis_tolerance]
        if not all(math.isfinite(v) for v in values):
            return 'Non-finite pose goal'
        if (abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > 1e-3
                or abs(oq.x*oq.x+oq.y*oq.y+oq.z*oq.z+oq.w*oq.w-1.) > 1e-3):
            return 'Pose-goal quaternion is not normalized'
        if any(v < 0. for v in values[-3:]):
            return 'Invalid pose-goal orientation tolerance'
    constraints = request.path_constraints
    if (constraints.joint_constraints or constraints.position_constraints
            or constraints.orientation_constraints or constraints.visibility_constraints
            or request.trajectory_constraints.constraints):
        return 'Path constraints are not implemented by this cuMotion version; refusing to ignore them'
    if (request.start_state.attached_collision_objects
            or scene.robot_state.attached_collision_objects):
        return 'Attached camera/payload import is pending; refusing to omit attachments'
    if scene.world.octomap.octomap.data or scene.link_padding or scene.link_scale:
        return 'Octomap/padding/scaling import is not implemented'
    acm = scene.allowed_collision_matrix
    if acm.entry_names or acm.default_entry_names:
        return 'Custom allowed-collision matrix is not implemented'
    if scene.is_diff or scene.fixed_frame_transforms:
        return 'Supply a complete world in link_base; scene diffs/transforms are not supported'
    for obj in scene.world.collision_objects:
        if obj.header.frame_id != 'link_base' or obj.operation != obj.ADD:
            return 'World objects must be ADD objects in link_base'
        if obj.meshes or obj.planes or obj.subframe_names:
            return 'First ROS test supports primitive world objects only'
        if not obj.primitives or len(obj.primitives) != len(obj.primitive_poses):
            return 'Missing primitive geometry or poses'
        p, q = obj.pose.position, obj.pose.orientation
        if any(abs(v) > 1e-9 for v in (p.x, p.y, p.z, q.x, q.y, q.z)) or abs(q.w-1.) > 1e-9:
            return 'Object-level pose must be identity; bake poses into primitive_poses'
    return None


def pose_goal(request, tensor_args):
    """Convert one validated MoveIt pose goal to cuRobo's xyz+wxyz pose."""
    goal = request.goal_constraints[0]
    p = goal.position_constraints[0].constraint_region.primitive_poses[0].position
    q = goal.orientation_constraints[0].orientation
    return CuPose.from_list(
        [p.x, p.y, p.z, q.w, q.x, q.y, q.z], tensor_args=tensor_args)


def validate_pose_endpoint(request, trajectory, motion_gen, tensor_args):
    """Verify the returned final FK against the actual MoveIt tolerance region."""
    point = trajectory.points[-1]
    state = motion_gen.get_active_js(CuJointState.from_position(
        tensor_args.to_device([list(point.positions)]),
        joint_names=list(trajectory.joint_names)))
    actual = motion_gen.compute_kinematics(state).ee_pose
    xyz = actual.position[0].detach().cpu().numpy()
    actual_wxyz = actual.quaternion[0].detach().cpu().numpy()
    goal = request.goal_constraints[0]
    position = goal.position_constraints[0]
    region_pose = position.constraint_region.primitive_poses[0]
    center = np.array([region_pose.position.x, region_pose.position.y,
                       region_pose.position.z])
    region_q = region_pose.orientation
    local_error = Rotation.from_quat(
        [region_q.x, region_q.y, region_q.z, region_q.w]).inv().apply(xyz-center)
    primitive = position.constraint_region.primitives[0]
    if primitive.type == primitive.SPHERE:
        position_ok = np.linalg.norm(local_error) <= primitive.dimensions[0] + 1e-4
    else:
        position_ok = np.all(np.abs(local_error) <= np.asarray(primitive.dimensions)/2 + 1e-4)
    orientation = goal.orientation_constraints[0]
    desired = orientation.orientation
    desired_rotation = Rotation.from_quat([desired.x, desired.y, desired.z, desired.w])
    actual_rotation = Rotation.from_quat(
        [actual_wxyz[1], actual_wxyz[2], actual_wxyz[3], actual_wxyz[0]])
    rotation_error = np.abs((desired_rotation.inv()*actual_rotation).as_rotvec())
    tolerances = np.array([orientation.absolute_x_axis_tolerance,
                           orientation.absolute_y_axis_tolerance,
                           orientation.absolute_z_axis_tolerance])
    if not position_ok or not np.all(rotation_error <= tolerances + 1e-4):
        raise ValueError(
            f'Pose goal tolerance violated: position_error={local_error.tolist()}, '
            f'rotation_vector_error={rotation_error.tolist()}')


class PlanOnlyServer(CumotionActionServer):
    def load_motion_gen(self):
        """Pinned release-4.0 initializer with an explicit optimization time budget.

        This is trajectory duration, NOT request timeout or a joint-displacement
        bound. Keep upstream voxel setup and all dynamics/collision validation.
        Private fields below intentionally match our pinned upstream release.
        """
        dt = (self.get_parameter('joint_trajectory_max_dt') if
              self.has_parameter('joint_trajectory_max_dt') else
              self.declare_parameter('joint_trajectory_max_dt', 0.30)).value
        if not math.isfinite(dt) or dt <= 0.:
            raise ValueError('joint_trajectory_max_dt must be positive and finite')
        upstream = lambda name: getattr(self, '_CumotionActionServer__' + name)
        world = WorldConfig.from_dict({
            'cuboid': {'table': {'pose': [0, 0, -.05, 1, 0, 0, 0],
                                  'dims': [2., 2., .1]}},
            'voxel': {'world_voxel': {'dims': upstream('grid_size_m'),
                'pose': [0, 0, 0, 1, 0, 0, 0], 'voxel_size': upstream('voxel_size'),
                'feature_dtype': torch.bfloat16}}})
        robot = get_robot_config(robot_file=upstream('robot_file'),
            urdf_file_path=upstream('urdf_path'), logger=self.get_logger())
        robot = self.prepare_robot_config(robot)
        config = MotionGenConfig.load_from_robot_config(
            robot['robot_cfg'], world, self.tensor_args,
            num_graph_seeds=upstream('num_graph_seeds'),
            num_trajopt_seeds=upstream('num_trajopt_seeds'),
            num_trajopt_noisy_seeds=upstream('num_trajopt_noisy_seeds'),
            trajopt_tsteps=upstream('num_trajopt_time_steps'),
            trajopt_seed_ratio=upstream('trajopt_seed_ratio'),
            interpolation_dt=upstream('interpolation_dt'),
            collision_cache=upstream('collision_cache'),
            collision_checker_type=CollisionCheckerType.VOXEL,
            ee_link_name=upstream('tool_frame'),
            finetune_trajopt_iters=upstream('trajopt_finetune_iters'),
            js_trajopt_dt=dt, maximum_trajectory_dt=dt)
        self.motion_gen = MotionGen(config)
        self._CumotionActionServer__robot_base_frame = self.motion_gen.kinematics.base_link
        checker = self.motion_gen.world_coll_checker
        self._CumotionActionServer__world_collision = checker
        if not upstream('add_ground_plane'):
            self.motion_gen.clear_world_cache()
        self._CumotionActionServer__cumotion_grid_shape = checker.get_voxel_grid(
            'world_voxel').get_grid_shape()[0]
        self.get_logger().info(f'Joint trajectory optimization maximum dt={dt}s; '
                               'joint displacement is not capped')

    def prepare_robot_config(self, robot):
        return robot

    def execute_callback(self, goal_handle):
        reason = unsupported(goal_handle.request)
        if reason:
            self.get_logger().error(reason)
            goal_handle.abort()
            result = MoveGroup.Result()
            result.error_code.val = MoveItErrorCodes.INVALID_MOTION_PLAN
            return result
        result = MoveGroup.Result()
        with self.lock:
            if self.planner_busy:
                goal_handle.abort()
                result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
                return result
            self.planner_busy = True
        try:
            request = goal_handle.request.request
            world = goal_handle.request.planning_options.planning_scene_diff.world.collision_objects
            if not self.update_world_objects(world):
                result.error_code.val = MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE
                goal_handle.abort()
                return result
            js = request.start_state.joint_state
            kind = goal_kind(request)
            joints = request.goal_constraints[0].joint_constraints
            start = self.motion_gen.get_active_js(CuJointState.from_position(
                self.tensor_args.to_device(list(js.position)).unsqueeze(0), joint_names=list(js.name)))
            target = None
            if kind == 'joint':
                target = self.motion_gen.get_active_js(CuJointState.from_position(
                    self.tensor_args.to_device([j.position for j in joints]).unsqueeze(0),
                    joint_names=[j.joint_name for j in joints]))
            scales = [request.max_velocity_scaling_factor, request.max_acceleration_scaling_factor]
            if any(not math.isfinite(s) or not 0. <= s <= 1. for s in scales):
                raise ValueError('Invalid velocity/acceleration scaling')
            scale = min(scales)
            if scale == 0.:
                scale = self.get_parameter('time_dilation_factor').value
            report = {'planner_method': 'plan_single_js' if kind == 'joint' else 'plan_single',
                      'start_names': list(js.name), 'start_rad': list(js.position),
                      'world_ids': [o.id for o in world]}
            if kind == 'joint':
                report.update(goal_names=[j.joint_name for j in joints],
                              goal_rad=[j.position for j in joints])
            else:
                p = request.goal_constraints[0].position_constraints[0]
                o = request.goal_constraints[0].orientation_constraints[0]
                center = p.constraint_region.primitive_poses[0].position
                report.update(goal_link=p.link_name,
                              goal_xyz=[center.x, center.y, center.z],
                              goal_xyzw=[o.orientation.x, o.orientation.y,
                                         o.orientation.z, o.orientation.w])
            self.get_logger().info(json.dumps(report))
            reset_started = time.monotonic()
            self.motion_gen.reset(reset_seed=False)
            reset_seconds = time.monotonic() - reset_started
            config = MotionGenPlanConfig(
                max_attempts=self.get_parameter('max_attempts').value,
                enable_graph_attempt=1, time_dilation_factor=scale,
                parallel_finetune=bool(
                    getattr(self, 'parallel_finetune', True)))
            planning_started = time.monotonic()
            planned = (self.motion_gen.plan_single_js(start, target, config)
                       if kind == 'joint' else
                       self.motion_gen.plan_single(start, pose_goal(request, self.tensor_args), config))
            success = bool(planned.success.item())  # Synchronizes pending CUDA work.
            wall_seconds = time.monotonic() - planning_started
            timing = {
                'event': 'cumotion_timing',
                'kind': kind,
                'wall_seconds': wall_seconds,
                'reset_seconds': reset_seconds,
                'parallel_finetune': config.parallel_finetune,
            }
            for name in ('total_time', 'solve_time', 'trajopt_time', 'graph_time',
                         'finetune_time', 'attempts'):
                value = diagnostic_scalar(getattr(planned, name, None))
                if value is not None:
                    timing[name] = value
            self.get_logger().info(json.dumps(timing))
            self.get_logger().info(f'{kind} planning success={success} '
                                   f'status={planned.status}')
            if not success:
                status = str(planned.status)
                result.error_code.val = (
                    MoveItErrorCodes.START_STATE_IN_COLLISION if 'START_STATE' in status and 'COLLISION' in status
                    else MoveItErrorCodes.START_STATE_INVALID if 'START_STATE' in status
                    else MoveItErrorCodes.GOAL_IN_COLLISION if 'GOAL' in status and 'COLLISION' in status
                    else MoveItErrorCodes.PLANNING_FAILED)
                goal_handle.abort()
                return result
            result.trajectory_start = request.start_state
            result.planned_trajectory = self.get_joint_trajectory(
                planned.optimized_plan, planned.optimized_dt.item())
            trajectory = result.planned_trajectory.joint_trajectory
            if not trajectory.points:
                raise ValueError('Planner returned an empty trajectory')
            if kind == 'joint':
                final = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
                for joint in joints:
                    error = final[joint.joint_name]-joint.position
                    if (not math.isfinite(error) or error > joint.tolerance_above+1e-5
                            or error < -joint.tolerance_below-1e-5):
                        raise ValueError(
                            f'Joint goal tolerance violated: {joint.joint_name}, error={error}')
            else:
                expected_link = self.motion_gen.kinematics.ee_link
                requested_link = request.goal_constraints[0].position_constraints[0].link_name
                if requested_link != expected_link:
                    raise ValueError(
                        f'Pose goal link {requested_link!r} does not match {expected_link!r}')
                validate_pose_endpoint(
                    request, trajectory, self.motion_gen, self.tensor_args)
            # ros2_control requires the terminal velocity to be exactly zero.
            # cuMotion can leave floating-point residue (for example 1e-7),
            # which is physically zero but is rejected by the controller.
            joint_count = len(trajectory.joint_names)
            trajectory.points[-1].velocities = [0.0] * joint_count
            trajectory.points[-1].accelerations = [0.0] * joint_count
            result.planning_time = float(planned.total_time)
            result.error_code.val = MoveItErrorCodes.SUCCESS
            goal_handle.succeed()
            return result
        except Exception as error:
            self.get_logger().error(f'Joint planner error: {error}')
            result.planned_trajectory = type(result.planned_trajectory)()
            result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
            goal_handle.abort()
            return result
        finally:
            with self.lock:
                self.planner_busy = False


def main():
    rclpy.init()
    node = PlanOnlyServer()
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
