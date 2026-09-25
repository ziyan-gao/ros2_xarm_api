"""Headless end-to-end MoveIt pipeline check for the isolated RViz demo."""
import json
import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import rclpy
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from validate_path import MeshValidator


def call(node, service, message_type, request):
    client = node.create_client(message_type, service)
    if not client.wait_for_service(timeout_sec=30.):
        raise RuntimeError(f'Service not ready: {service}')
    pending = client.call_async(request)
    rclpy.spin_until_future_complete(node, pending, timeout_sec=30.)
    if not pending.done() or pending.result() is None:
        raise RuntimeError(f'Service timed out: {service}')
    return pending.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-only', action='store_true')
    args = parser.parse_args()
    model = Path(os.environ['CUMOTION_TEST_MODEL'])
    config = yaml.safe_load((model/'uf850.yml').read_text())['robot_cfg']['kinematics']['cspace']
    names, start = config['joint_names'], config['retract_config']
    target = list(start)
    target[0] += float(os.environ.get('CUMOTION_TEST_GOAL_DELTA', '.1'))
    if os.environ.get('CUMOTION_TEST_GOAL_JSON'):
        target = json.loads(os.environ['CUMOTION_TEST_GOAL_JSON'])
        if len(target) != len(names) or not np.isfinite(target).all():
            raise ValueError('Expected six finite target joint angles')
    rclpy.init()
    node = rclpy.create_node('cumotion_moveit_pipeline_test')
    try:
        params = call(node, '/move_group/get_parameters', GetParameters,
                      GetParameters.Request(names=['allow_trajectory_execution']))
        if params.values[0].type != 1 or params.values[0].bool_value:
            raise RuntimeError('Trajectory execution must be explicitly disabled')
        floor = CollisionObject()
        floor.header.frame_id = 'link_base'
        floor.id = 'synthetic_floor'
        floor.pose.orientation.w = 1.
        floor.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2., 2., .1])]
        pose = Pose()
        pose.orientation.w = 1.
        pose.position.z = -.15
        floor.primitive_poses = [pose]
        scene = ApplyPlanningScene.Request()
        scene.scene.is_diff = True
        scene.scene.world.collision_objects = [floor]
        if not call(node, '/apply_planning_scene', ApplyPlanningScene, scene).success:
            raise RuntimeError('Could not add synthetic floor')
        if args.scene_only:
            print('Synthetic floor loaded; trajectory execution disabled.', flush=True)
            return
        query = GetMotionPlan.Request()
        req = query.motion_plan_request
        req.group_name = 'uf850'
        req.pipeline_id = 'isaac_ros_cumotion'
        req.start_state.joint_state.name = names
        req.start_state.joint_state.position = start
        req.allowed_planning_time = 5.
        req.max_velocity_scaling_factor = .2
        req.max_acceleration_scaling_factor = .2
        goal = Constraints()
        goal.joint_constraints = [JointConstraint(joint_name=n, position=float(q),
            tolerance_above=.01, tolerance_below=.01, weight=1.) for n, q in zip(names, target)]
        req.goal_constraints = [goal]
        result = call(node, '/plan_kinematic_path', GetMotionPlan, query).motion_plan_response
        if result.error_code.val != 1:
            raise RuntimeError(f'MoveIt pipeline failed: {result.error_code.val}')
        traj = result.trajectory.joint_trajectory
        if list(traj.joint_names) != names:
            raise RuntimeError('Unexpected joint order')
        root = ET.parse(model/'uf850.urdf').getroot()
        limits = {j.get('name'): [float(j.find('limit').get('lower')),
                  float(j.find('limit').get('upper'))] for j in root.findall('joint')
                  if j.get('name') in names}
        check = MeshValidator(root, ET.parse(model/'uf850.srdf').getroot()).check(
            np.array([p.positions for p in traj.points]), names, limits, start, target)
        if not check['passed']:
            raise RuntimeError(f'Independent validation failed: {check}')
        print(json.dumps({'moveit_pipeline_success': True, 'execution_enabled': False,
                          'planning_time_s': result.planning_time, 'validation': check}), flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
