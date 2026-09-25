"""Bake attached boxes into separate fixed links; never discard geometry."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

DYNAMIC_PAYLOAD_LINK = 'link_tcp'
DYNAMIC_PAYLOAD_SPHERES = 24
DYNAMIC_PAYLOAD_TOUCH_LINKS = {
    'link_tcp', 'link_eef', 'ft_sensor_link', 'xarm_vacuum_gripper_link'}
# cuRobo's payload spheres are deliberately conservative and may overlap the
# rigid wrist/tool-stack approximation.  MoveIt validates the returned path
# against the exact geometry, so ignore only this fixed near-mount chain here.
DYNAMIC_PAYLOAD_NEAR_MOUNT_LINKS = DYNAMIC_PAYLOAD_TOUCH_LINKS | {
    'link6', 'camera_stand_link'}
# The camera and a grasped box are both rigidly mounted to the EEF.  Their
# sphere fits can overlap even when MoveIt's exact boxes do not.  cuMotion
# must not reject an otherwise valid endpoint because of that approximation;
# MoveIt's ValidateSolution adapter still checks the exact attached geometry.
DYNAMIC_PAYLOAD_RIGID_ATTACHMENT_IDS = {'eef_camera_d435i'}


def pose_values(pose):
    p, q = pose.position, pose.orientation
    xyz, quat = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
    if not np.isfinite(xyz + quat).all() or abs(np.linalg.norm(quat)-1.) > 1e-5:
        raise ValueError('Invalid attachment pose')
    return np.array(xyz), Rotation.from_quat(quat)


def reserve_dynamic_payload(robot):
    """Preallocate disabled link_tcp spheres for zero-warm-up payload updates."""
    robot = copy.deepcopy(robot)
    k = robot['robot_cfg']['kinematics']
    root = ET.parse(k['urdf_path']).getroot()
    if DYNAMIC_PAYLOAD_LINK not in {link.get('name') for link in root.findall('link')}:
        raise ValueError(f'{DYNAMIC_PAYLOAD_LINK} is missing from robot URDF')
    if DYNAMIC_PAYLOAD_LINK not in k['link_names']:
        k['link_names'].append(DYNAMIC_PAYLOAD_LINK)
    if DYNAMIC_PAYLOAD_LINK not in k['collision_link_names']:
        k['collision_link_names'].append(DYNAMIC_PAYLOAD_LINK)
    extra = dict(k.get('extra_collision_spheres') or {})
    existing = extra.get(DYNAMIC_PAYLOAD_LINK)
    if existing not in (None, DYNAMIC_PAYLOAD_SPHERES):
        raise ValueError('Conflicting dynamic payload sphere reservation')
    extra[DYNAMIC_PAYLOAD_LINK] = DYNAMIC_PAYLOAD_SPHERES
    k['extra_collision_spheres'] = extra
    ignored = set(k['self_collision_ignore'].get(DYNAMIC_PAYLOAD_LINK, []))
    ignored.update(DYNAMIC_PAYLOAD_NEAR_MOUNT_LINKS-{DYNAMIC_PAYLOAD_LINK})
    k['self_collision_ignore'][DYNAMIC_PAYLOAD_LINK] = sorted(ignored)
    return robot


def dynamic_payload_spheres(attached):
    """Approximate one link_tcp-attached box in link_tcp coordinates."""
    obj = attached.object
    if (attached.link_name != DYNAMIC_PAYLOAD_LINK or
            obj.header.frame_id != DYNAMIC_PAYLOAD_LINK):
        raise ValueError('Dynamic payload must be expressed in link_tcp')
    if (obj.operation != obj.ADD or obj.meshes or obj.planes or
            obj.subframe_names or not obj.primitives or
            len(obj.primitives) != len(obj.primitive_poses)):
        raise ValueError('Dynamic payload requires attached primitive boxes')
    if not set(attached.touch_links).issubset(DYNAMIC_PAYLOAD_TOUCH_LINKS):
        raise ValueError('Dynamic payload has unsupported touch links')
    from curobo.geom.sphere_fit import SphereFitType, fit_spheres_to_mesh
    import trimesh
    obj_center, obj_rotation = pose_values(obj.pose)
    per_primitive = max(1, DYNAMIC_PAYLOAD_SPHERES // len(obj.primitives))
    fitted = []
    for primitive, primitive_pose in zip(obj.primitives, obj.primitive_poses):
        dims = np.asarray(primitive.dimensions, dtype=float)
        if (primitive.type != primitive.BOX or dims.shape != (3,) or
                not np.isfinite(dims).all() or np.any(dims <= 0)):
            raise ValueError('Expected finite positive dynamic payload box dimensions')
        center, rotation = pose_values(primitive_pose)
        points, radii = fit_spheres_to_mesh(
            trimesh.creation.box(extents=dims), per_primitive,
            surface_sphere_radius=0.001,
            fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE)
        for point, radius in zip(points, radii):
            in_object = rotation.apply(point)+center
            in_parent = obj_rotation.apply(in_object)+obj_center
            fitted.append([*in_parent.tolist(), float(radius)])
    if not fitted:
        raise ValueError('Dynamic payload sphere fitting returned no spheres')
    result = np.zeros((DYNAMIC_PAYLOAD_SPHERES, 4), dtype=np.float32)
    result[:, 3] = -10.0
    count = min(len(fitted), DYNAMIC_PAYLOAD_SPHERES)
    result[:count] = np.asarray(fitted[:count], dtype=np.float32)
    return result


def bake(robot, attachments, destination):
    robot = copy.deepcopy(robot)
    k = robot['robot_cfg']['kinematics']
    root = ET.parse(k['urdf_path']).getroot()
    links = {link.get('name') for link in root.findall('link')}
    seen = set()
    for index, attached in enumerate(attachments):
        obj = attached.object
        if obj.operation != obj.ADD:
            raise ValueError('Attached geometry must use ADD in a full scene')
        if not obj.id or obj.id in seen:
            raise ValueError('Missing or duplicate attachment id')
        seen.add(obj.id)
        if attached.link_name not in links or obj.header.frame_id != attached.link_name:
            raise ValueError('Attachment must be expressed in its parent link')
        if obj.meshes or obj.planes or obj.subframe_names or not obj.primitives:
            raise ValueError('Only attached primitive boxes are supported')
        if len(obj.primitives) != len(obj.primitive_poses):
            raise ValueError('Attachment primitive/pose count mismatch')
        if any(name not in links for name in attached.touch_links):
            raise ValueError('Unknown attachment touch link')
        name = f'cumotion_attachment_{index}'
        if name in links:
            raise ValueError('Attachment link name collision')
        xyz, rotation = pose_values(obj.pose)
        link = ET.SubElement(root, 'link', name=name)
        joint = ET.SubElement(root, 'joint', name=name+'_fixed', type='fixed')
        ET.SubElement(joint, 'parent', link=attached.link_name)
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)),
                      rpy=' '.join(map(str, rotation.as_euler('xyz'))))
        spheres = []
        for primitive, pose in zip(obj.primitives, obj.primitive_poses):
            dims = np.array(primitive.dimensions)
            if primitive.type != primitive.BOX or dims.shape != (3,) or not np.isfinite(dims).all() or np.any(dims <= 0):
                raise ValueError('Expected finite positive attached box dimensions')
            center, rot = pose_values(pose)
            # Use cuRobo's standard near-surface/volume approximation. MoveIt's
            # response adapter still validates the resulting path against the
            # exact box; this avoids large circumscribed-box false positives.
            from curobo.geom.sphere_fit import SphereFitType, fit_spheres_to_mesh
            import trimesh
            points, radii = fit_spheres_to_mesh(
                trimesh.creation.box(extents=dims), 24, surface_sphere_radius=0.001,
                fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE)
            balls = [{'center': point.tolist(), 'radius': float(radius)}
                     for point, radius in zip(points, radii)]
            for ball in balls:
                ball['center'] = (rot.apply(ball['center'])+center).tolist()
            spheres.extend(balls)
            collision = ET.SubElement(link, 'collision')
            ET.SubElement(collision, 'origin', xyz=' '.join(map(str, center)),
                          rpy=' '.join(map(str, rot.as_euler('xyz'))))
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size=' '.join(map(str, dims)))
        k['collision_spheres'][name] = spheres
        k['collision_link_names'].append(name)
        k['link_names'].append(name)
        k['self_collision_buffer'][name] = 0.
        ignored = set(attached.touch_links)
        if obj.id in DYNAMIC_PAYLOAD_RIGID_ATTACHMENT_IDS:
            ignored.add(DYNAMIC_PAYLOAD_LINK)
            payload_ignored = set(
                k['self_collision_ignore'].get(DYNAMIC_PAYLOAD_LINK, []))
            payload_ignored.add(name)
            k['self_collision_ignore'][DYNAMIC_PAYLOAD_LINK] = sorted(
                payload_ignored)
        k['self_collision_ignore'][name] = sorted(ignored)
    path = Path(destination)/'attached.urdf'
    ET.ElementTree(root).write(path, encoding='unicode')
    k['urdf_path'] = str(path)
    return robot
