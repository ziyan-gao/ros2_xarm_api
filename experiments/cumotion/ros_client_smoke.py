"""Send one synthetic plan-only query, then independently validate the response."""
import argparse
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from validate_path import MeshValidator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    args = parser.parse_args()
    model = Path(args.model)
    config = yaml.safe_load((model/'uf850.yml').read_text())['robot_cfg']['kinematics']['cspace']
    names, start = config['joint_names'], config['retract_config']
    target = list(start)
    target[0] += .1
    rclpy.init()
    node = rclpy.create_node('cumotion_test_client')
    try:
        client = ActionClient(node, MoveGroup, '/cumotion_test/cumotion/move_group')
        if not client.wait_for_server(timeout_sec=60.):
            raise RuntimeError('Planner action unavailable')
        goal = MoveGroup.Goal()
        goal.planning_options.plan_only = True
        goal.request.group_name = 'uf850'
        goal.request.start_state.joint_state.name = names
        goal.request.start_state.joint_state.position = start
        goal.request.max_velocity_scaling_factor = .2
        goal.request.max_acceleration_scaling_factor = .2
        goal.request.allowed_planning_time = 5.
        constraint = Constraints()
        constraint.joint_constraints = [JointConstraint(joint_name=n, position=float(q),
            tolerance_above=.001, tolerance_below=.001, weight=1.) for n, q in zip(names, target)]
        goal.request.goal_constraints = [constraint]
        floor = CollisionObject()
        floor.header.frame_id = 'link_base'
        floor.id = 'synthetic_floor'
        floor.pose.orientation.w = 1.
        floor.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2., 2., .1])]
        pose = Pose()
        pose.orientation.w = 1.
        pose.position.z = -.15
        floor.primitive_poses = [pose]
        goal.planning_options.planning_scene_diff.world.collision_objects = [floor]
        for rejection in ('execution', 'path_constraint'):
            invalid = copy.deepcopy(goal)
            if rejection == 'execution':
                invalid.planning_options.plan_only = False
            else:
                invalid.request.path_constraints.joint_constraints = constraint.joint_constraints
            pending = client.send_goal_async(invalid)
            rclpy.spin_until_future_complete(node, pending, timeout_sec=10.)
            if not pending.done() or not pending.result().accepted:
                raise RuntimeError('Rejection test did not reach the action server')
            rejected = pending.result().get_result_async()
            rclpy.spin_until_future_complete(node, rejected, timeout_sec=10.)
            if not rejected.done() or rejected.result().result.error_code.val != -2:
                raise RuntimeError(f'Unsafe request not rejected: {rejection}')
            print(json.dumps({'rejection_test': rejection, 'passed': True}), flush=True)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('Goal not accepted')
        future = future.result().get_result_async()
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.)
        if not future.done():
            raise RuntimeError('Result timed out; stop the isolated server before retrying')
        result = future.result().result
        if result.error_code.val != 1:
            raise RuntimeError(f'Planner error: {result.error_code.val}')
        trajectory = result.planned_trajectory.joint_trajectory
        if list(trajectory.joint_names) != names:
            raise RuntimeError('Unexpected joint order')
        root = ET.parse(model/'uf850.urdf').getroot()
        limits = {j.get('name'): [float(j.find('limit').get('lower')),
                  float(j.find('limit').get('upper'))] for j in root.findall('joint')
                  if j.get('name') in names}
        validation = MeshValidator(root, ET.parse(model/'uf850.srdf').getroot()).check(
            np.array([p.positions for p in trajectory.points]), names, limits, start, target)
        print(json.dumps({'ros_action_success': True, 'robot_execution': False,
                          'planning_time_s': result.planning_time, 'validation': validation}))
        if not validation['passed']:
            raise RuntimeError('Independent validation failed')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
