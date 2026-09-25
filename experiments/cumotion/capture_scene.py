"""Read-only MoveIt scene capture. Never plans, executes, or applies a scene."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='New JSON file; never overwrites')
    parser.add_argument('--service', default='/get_planning_scene')
    parser.add_argument('--timeout', type=float, default=15.)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('timeout must be positive')
    output = Path(args.output)
    if output.exists():
        parser.error('output already exists')
    rclpy.init()
    node = rclpy.create_node('cumotion_read_only_scene_capture')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        client = node.create_client(GetPlanningScene, args.service)
        if not client.wait_for_service(timeout_sec=args.timeout):
            raise RuntimeError('MoveIt scene service unavailable; no stack was started')
        request = GetPlanningScene.Request()
        fields = PlanningSceneComponents
        request.components.components = (
            fields.SCENE_SETTINGS | fields.ROBOT_STATE | fields.ROBOT_STATE_ATTACHED_OBJECTS
            | fields.WORLD_OBJECT_GEOMETRY | fields.OCTOMAP | fields.TRANSFORMS
            | fields.ALLOWED_COLLISION_MATRIX | fields.LINK_PADDING_AND_SCALING)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('Scene snapshot timed out or failed')
        scene = future.result().scene
        if not scene.robot_state.joint_state.name:
            raise RuntimeError('Scene has no joint state')
        world = scene.world.collision_objects
        attached = scene.robot_state.attached_collision_objects
        # Resolve all geometry frames, including local attached-object frames.
        frames = {obj.header.frame_id for obj in world}
        frames.update(a.object.header.frame_id or a.link_name for a in attached)
        frames.update(a.link_name for a in attached)
        frames.update(('link_tcp', 'link_eef', 'camera_link'))
        if '' in frames:
            raise RuntimeError('World object has an empty frame; refusing ambiguous snapshot')
        frames.discard('link_base')
        transforms = {}
        deadline = time.monotonic()+args.timeout
        while frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            for frame in list(frames):
                try:
                    tf = buffer.lookup_transform('link_base', frame, rclpy.time.Time())
                except TransformException:
                    continue
                transforms[frame] = message_to_ordereddict(tf)
                frames.remove(frame)
        if frames:
            raise RuntimeError(f'Missing transforms: {sorted(frames)}')
        attached_ids = [a.object.id for a in attached]
        report = {
            'schema_version': 1,
            'captured_utc': datetime.now(timezone.utc).isoformat(),
            'robot_execution': False,
            'base_frame': 'link_base',
            'scene': message_to_ordereddict(scene),
            'base_transforms': transforms,
            'world_object_ids': [o.id for o in world],
            'attached_object_ids': attached_ids,
            'camera_included': 'eef_camera_d435i' in attached_ids,
            'note': 'Capture while stationary. TF is latest available, not an atomic scene/TF sample. '
                    'Payload identity and completeness require operator review. '
                    'No geometry, ACM, padding or octomap may be silently discarded during import.',
        }
        # Serialize before creating the file; x mode protects existing snapshots.
        encoded = json.dumps(report, indent=2, allow_nan=False)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x') as stream:
            stream.write(encoded+'\n')
        print(json.dumps({'saved': str(output), 'world': report['world_object_ids'],
                          'attached': attached_ids, 'camera_included': report['camera_included']}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
