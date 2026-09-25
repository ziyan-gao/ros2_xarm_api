"""Adapter ONLY for the isolated, execution-disabled MoveIt panel demo.

Upstream MoveIt plugin does not populate plan_only. This bridge sets it for
the planner-only hop; outer move_group has allow_trajectory_execution=false.
Never use this bridge on the production ROS graph.
"""
import copy
import os
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from uf850_smoke import origin

import rclpy
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from ros_plan_only import PlanOnlyServer


def normalize(goal, ignored):
    goal = copy.deepcopy(goal)
    scene = goal.planning_options.planning_scene_diff
    # Full-scene metadata emitted by MoveIt. Preserve nontrivial information
    # by rejecting anything the experimental model cannot represent.
    if scene.robot_state.attached_collision_objects:
        raise ValueError('Camera/payload attachments are not supported in this demo')
    if any(p.padding != 0. for p in scene.link_padding):
        raise ValueError('Nonzero link padding is unsupported')
    if any(s.scale != 1. for s in scene.link_scale):
        raise ValueError('Non-unit link scale is unsupported')
    acm = scene.allowed_collision_matrix
    if any(acm.default_entry_values):
        raise ValueError('Default allowed-collision overrides are unsupported')
    if len(acm.entry_values) != len(acm.entry_names):
        raise ValueError('Malformed collision matrix')
    for name, row in zip(acm.entry_names, acm.entry_values):
        if len(row.enabled) != len(acm.entry_names):
            raise ValueError('Malformed collision matrix row')
        for other, enabled in zip(acm.entry_names, row.enabled):
            if enabled and name != other and frozenset((name, other)) not in ignored:
                raise ValueError(f'Unsupported collision exemption: {name}/{other}')
    for tf in scene.fixed_frame_transforms:
        t, q = tf.transform.translation, tf.transform.rotation
        if (tf.header.frame_id not in ('world', 'link_base')
                or tf.child_frame_id not in ('world', 'link_base')
                or any(abs(v) > 1e-9 for v in (t.x, t.y, t.z, q.x, q.y, q.z))
                or abs(abs(q.w)-1.) > 1e-9):
            raise ValueError('Only identity world/base transform supported')
    # MoveIt serializes objects in its model frame (world) and may put the
    # placement into object.pose. cuMotion 4.0 reads primitive_poses directly.
    for obj in scene.world.collision_objects:
        if obj.header.frame_id == 'world':
            obj.header.frame_id = 'link_base'
        p, q = obj.pose.position, obj.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1.) > 1e-5:
            raise ValueError('Invalid object pose')
        rotation = Rotation.from_quat(values[3:])
        for pose in obj.primitive_poses:
            t, r = pose.position, pose.orientation
            xyz = rotation.apply([t.x, t.y, t.z])+values[:3]
            quat = (rotation*Rotation.from_quat([r.x, r.y, r.z, r.w])).as_quat()
            t.x, t.y, t.z = map(float, xyz)
            r.x, r.y, r.z, r.w = map(float, quat)
        p.x = p.y = p.z = q.x = q.y = q.z = 0.
        q.w = 1.
    if goal.request.start_state.is_diff or not goal.request.start_state.joint_state.name:
        if goal.request.start_state.joint_state.name:
            raise ValueError('Nonempty differential start is unsupported')
        goal.request.start_state = copy.deepcopy(scene.robot_state)
        goal.request.start_state.is_diff = False
    scene.link_padding = []
    scene.link_scale = []
    scene.fixed_frame_transforms = []
    scene.allowed_collision_matrix = type(acm)()
    goal.planning_options.plan_only = True
    return goal


class HandleProxy:
    def __init__(self, original, request):
        self.original, self.request = original, request

    def __getattr__(self, name):
        return getattr(self.original, name)


class PanelServer(PlanOnlyServer):
    def __init__(self):
        model = os.environ['CUMOTION_TEST_MODEL']
        root = ET.parse(model+'/uf850.urdf').getroot()
        base_joints = [j for j in root.findall('joint') if j.find('child').get('link') == 'link_base']
        if (len(base_joints) != 1 or base_joints[0].get('type') != 'fixed'
                or base_joints[0].find('parent').get('link') != 'world'
                or not np.allclose(origin(base_joints[0].find('origin')), np.eye(4), atol=1e-9)):
            raise RuntimeError('Demo requires an identity world-to-link_base fixed joint')
        srdf = ET.parse(model+'/uf850.srdf').getroot()
        self.ignored = {frozenset((p.get('link1'), p.get('link2')))
                        for p in srdf.findall('disable_collisions')}
        super().__init__()

    def execute_callback(self, handle):
        try:
            request = normalize(handle.request, self.ignored)
        except ValueError as error:
            self.get_logger().error(str(error))
            handle.abort()
            result = MoveGroup.Result()
            result.error_code.val = MoveItErrorCodes.INVALID_MOTION_PLAN
            return result
        return super().execute_callback(HandleProxy(handle, request))


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '87':
        raise RuntimeError('Isolated panel demo requires ROS_DOMAIN_ID=87')
    rclpy.init()
    node = PanelServer()
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
