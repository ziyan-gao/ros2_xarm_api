"""Offline full-scene adapter test; never connects to a robot or executes."""
import json
import sys
import itertools
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene, Constraints, JointConstraint
from rosidl_runtime_py.set_message import set_message_fields
from live_bridge import LiveServer
from live_attachments import bake
from uf850_smoke import fk


class Handle:
    def __init__(self, request):
        self.request = request
        self.state = None
    def abort(self): self.state = 'aborted'
    def succeed(self): self.state = 'succeeded'


def main():
    rclpy.init()
    node = LiveServer()
    try:
        snapshot = json.load(open(sys.argv[1]))
        scene = PlanningScene()
        objects = snapshot['scene']['world']['collision_objects'] + [
            a['object'] for a in snapshot['scene']['robot_state']['attached_collision_objects']]
        for obj in objects:
            if isinstance(obj['operation'], str):
                obj['operation'] = obj['operation'].encode('latin1')
        set_message_fields(scene, snapshot['scene'])
        goal = MoveGroup.Goal()
        goal.request.group_name = 'uf850'
        goal.request.start_state = scene.robot_state
        goal.request.max_velocity_scaling_factor = .1
        goal.request.max_acceleration_scaling_factor = .1
        goal.planning_options.planning_scene_diff = scene
        goal.planning_options.plan_only = True
        js = scene.robot_state.joint_state
        target = list(js.position)
        target[0] += .03
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=n, position=q,
            tolerance_above=.01, tolerance_below=.01, weight=1.) for n,q in zip(js.name,target)]
        goal.request.goal_constraints = [c]
        handle = Handle(goal)
        result = node.execute_callback(handle)
        print(json.dumps({'status': handle.state, 'error_code': result.error_code.val,
                          'points': len(result.planned_trajectory.joint_trajectory.points)}))
        if not node.model_valid:
            raise RuntimeError('Attachment/model import failed')
        if result.error_code.val == -10:
            with tempfile.TemporaryDirectory() as folder:
                original = yaml.safe_load(open(node.get_parameter('yml_file_path').value))
                k = bake(original, scene.robot_state.attached_collision_objects, folder)['robot_cfg']['kinematics']
                frames = fk(ET.parse(k['urdf_path']).getroot(), dict(zip(js.name, js.position)))
                spheres = k['collision_spheres']
                ignored = k['self_collision_ignore']
                overlaps = []
                for a,b in itertools.combinations(spheres, 2):
                    if b in ignored.get(a, []) or a in ignored.get(b, []): continue
                    points = {n: np.array([s['center'] for s in spheres[n]]) @ frames[n][:3,:3].T + frames[n][:3,3] for n in (a,b)}
                    penetration = np.array([s['radius'] for s in spheres[a]])[:,None] + np.array([s['radius'] for s in spheres[b]])[None,:] - np.linalg.norm(points[a][:,None,:]-points[b][None,:,:],axis=2)
                    if penetration.max() > 0: overlaps.append([a,b,float(penetration.max())])
                print(json.dumps({'sphere_overlaps_at_start': overlaps}))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__': main()
