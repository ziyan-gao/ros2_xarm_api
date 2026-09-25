"""Offline UF850 model test. No ROS nodes, networking or execution interfaces.

Conservative slab enclosures with local AABB covers are experimental. This
is NOT production geometry: runtime camera and payload attachments are absent.
"""
import argparse
import itertools
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import trimesh
import yaml
from ament_index_python.packages import get_package_share_directory


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, :3] = Rotation.from_euler(
            'xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    return transform


def fk(root, positions):
    """Independent URDF-chain FK for comparison against cuRobo kernels."""
    transforms = {'world': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            motion = np.eye(4)
            if joint.get('type') != 'fixed':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                motion[:3, :3] = Rotation.from_rotvec(
                    axis * positions[joint.get('name')]).as_matrix()
            transforms[joint.find('child').get('link')] = (
                transforms[parent] @ origin(joint.find('origin')) @ motion)
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError('URDF contains disconnected joints')
    return transforms


def sphere_cover(bounds, pitch=0.04):
    """Cover an entire AABB using circumscribed spheres of tiled cells."""
    low, high = bounds
    counts = np.maximum(1, np.ceil((high-low)/pitch).astype(int))
    size = (high-low)/counts
    radius = float(np.linalg.norm(size)/2 + 0.001)
    return [{'center': (low+(np.array(index)+0.5)*size).tolist(), 'radius': radius}
            for index in itertools.product(*(range(n) for n in counts))]


def sliced_cover(mesh, pitch):
    """Enclose each mesh slab, rather than its entire rectangular bounding box.

For a watertight polyhedral solid, each clipped slab lies in the convex hull
of its clipped boundary vertices. A sphere enclosing all these vertices
therefore encloses the slab, including its interior. Never shrink to fit a
planning result. Open meshes fall back to the conservative AABB cover.
"""
    if not mesh.is_watertight:
        return sphere_cover(mesh.bounds, pitch), 'aabb_open_mesh_fallback'
    axis = int(np.argmax(mesh.extents))
    lo, hi = mesh.bounds[:, axis]
    edges = np.linspace(lo, hi, max(1, int(np.ceil((hi-lo)/pitch)))+1)
    normal = np.eye(3)[axis]
    balls = []
    for lower, upper in zip(edges[:-1], edges[1:]):
        slab = mesh.slice_plane(normal*(lower-1e-8), normal)
        slab = slab.slice_plane(normal*(upper+1e-8), -normal)
        if not len(slab.vertices):
            continue
        points = np.unique(slab.vertices, axis=0)
        try:
            center, _ = trimesh.nsphere.minimum_nsphere(points)
        except (ValueError, RuntimeError, np.linalg.LinAlgError):
            center = (points.min(axis=0)+points.max(axis=0))/2
        # Independently recompute enclosure radius, including numerical slack.
        radius = float(np.linalg.norm(points-center, axis=1).max()+0.001)
        assert np.isfinite(center).all() and np.isfinite(radius)
        balls.append({'center': center.tolist(), 'radius': radius})
    if not balls:
        raise ValueError('Empty slab sphere model')
    return balls, 'closed_mesh_slab_enclosure'


def apply_planning_joint_limits(root, lower_deg=None, upper_deg=None):
    """Narrow limits in the exported cuMotion-only URDF.

    The source xArm URDF and ros2_control limits are deliberately untouched.
    Each override may only make an existing physical interval smaller.
    """
    lower_deg = lower_deg or {}
    upper_deg = upper_deg or {}
    requested = set(lower_deg) | set(upper_deg)
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    missing = requested - set(joints)
    if missing:
        raise ValueError(f'planning-limit joints are absent from URDF: {sorted(missing)}')
    applied = {}
    for name in sorted(requested):
        limit = joints[name].find('limit')
        if limit is None or limit.get('lower') is None or limit.get('upper') is None:
            raise ValueError(f'{name} has no bounded position interval')
        physical_low = float(limit.get('lower'))
        physical_high = float(limit.get('upper'))
        low = math.radians(float(lower_deg[name])) if name in lower_deg else physical_low
        high = math.radians(float(upper_deg[name])) if name in upper_deg else physical_high
        if not all(np.isfinite(v) for v in (physical_low, physical_high, low, high)):
            raise ValueError(f'{name} planning limits must be finite')
        if low < physical_low or high > physical_high:
            raise ValueError(
                f'{name} planning interval [{low}, {high}] rad may not expand '
                f'physical interval [{physical_low}, {physical_high}] rad')
        if low >= high:
            raise ValueError(f'{name} planning lower limit must be below upper limit')
        limit.set('lower', f'{low:.17g}')
        limit.set('upper', f'{high:.17g}')
        applied[name] = {
            'physical_rad': [physical_low, physical_high],
            'planning_rad': [low, high],
        }
    return applied


def main():
    # cuRobo imports initialize CUDA; keep geometry helpers usable without GPU.
    from curobo.types.base import TensorDeviceType
    from curobo.types.state import JointState
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoints', required=True)
    parser.add_argument('--plan', action='store_true')
    parser.add_argument('--export-only', action='store_true', help='Export without GPU smoke planning')
    parser.add_argument('--export-model', help='Create a new directory with ROS planner model files')
    parser.add_argument('--sphere-model', choices=['aabb', 'slabs', 'curobo'], default='slabs')
    parser.add_argument('--sphere-pitch', type=float, default=0.04,
                        help='AABB covering-cell size, meters; larger is coarser')
    parser.add_argument('--joint3-min-deg', type=float,
                        help='cuMotion-only joint3 lower position bound in degrees')
    parser.add_argument('--joint5-max-deg', type=float,
                        help='cuMotion-only joint5 upper position bound in degrees')
    options = parser.parse_args()
    if options.export_only and not options.export_model:
        parser.error('--export-only requires --export-model')
    if not 0.01 <= options.sphere_pitch <= 0.1:
        parser.error('sphere-pitch must be between 0.01 and 0.1 m')
    if options.sphere_model == 'curobo':
        from curobo.geom.sphere_fit import SphereFitType, fit_spheres_to_mesh
    description = Path(get_package_share_directory('xarm_description'))
    moveit = Path(get_package_share_directory('xarm_moveit_config'))
    common = ['robot_type:=uf850', 'dof:=6', 'add_vacuum_gripper:=true']
    xml = subprocess.check_output(['xacro', str(description/'urdf/xarm_device.urdf.xacro'),
                                   *common, 'ros2_control_plugin:=mock_components/GenericSystem'])
    root = ET.fromstring(xml)
    srdf = ET.fromstring(subprocess.check_output(
        ['xacro', str(moveit/'srdf/xarm.srdf.xacro'), *common]))
    for mesh in root.findall('.//mesh'):
        filename = mesh.get('filename')
        if filename.startswith('package://'):
            package, relative = filename[len('package://'):].split('/', 1)
            mesh.set('filename', str(Path(get_package_share_directory(package))/relative))
    # The offline file has no hardware/control or Gazebo plugin declarations.
    for tag in ('ros2_control', 'gazebo', 'transmission'):
        for child in root.findall(tag):
            root.remove(child)
    planning_limit_overrides = apply_planning_joint_limits(
        root,
        lower_deg=({'joint3': options.joint3_min_deg}
                   if options.joint3_min_deg is not None else {}),
        upper_deg=({'joint5': options.joint5_max_deg}
                   if options.joint5_max_deg is not None else {}))
    names = [f'joint{i}' for i in range(1, 7)]
    waypoint = yaml.safe_load(Path(options.waypoints).read_text())['waypoints']['observation']
    saved = dict(zip(waypoint['joint_names'], waypoint['positions_rad']))
    q = [float(saved[name]) for name in names]
    moveit_limits = yaml.safe_load((Path(get_package_share_directory('xarm_moveit_config')) /
        'config/uf850/joint_limits.yaml').read_text())['joint_limits']
    accelerations = []
    limits = {}
    for joint in root.findall('joint'):
        if joint.get('name') in names:
            limit = joint.find('limit')
            configured = moveit_limits[joint.get('name')]
            velocity = min(float(limit.get('velocity')), float(configured['max_velocity']))
            acceleration = float(configured['max_acceleration'])
            if not all(math.isfinite(v) and v > 0 for v in (velocity, acceleration)):
                raise ValueError('invalid MoveIt joint dynamics limits')
            limit.set('velocity', str(velocity))
            accelerations.append(acceleration)
            limits[joint.get('name')] = [float(limit.get('lower')), float(limit.get('upper'))]
    for name, angle in zip(names, q):
        assert limits[name][0] <= angle <= limits[name][1], (name, angle, limits[name])
    spheres = {}
    fitting = {}
    for link in root.findall('link'):
        balls = []
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            mesh = geometry.find('mesh')
            if mesh is not None:
                shape = trimesh.load(mesh.get('filename'), force='mesh')
                shape.apply_scale(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '))
                bounds = shape.bounds
            elif geometry.find('cylinder') is not None:
                cylinder = geometry.find('cylinder')
                r, h = float(cylinder.get('radius')), float(cylinder.get('length'))
                bounds = np.array([[-r, -r, -h/2], [r, r, h/2]])
                # Circumscribed polygon, not an inscribed under-approximation.
                shape = trimesh.creation.cylinder(radius=r/np.cos(np.pi/64), height=h, sections=64)
            elif geometry.find('box') is not None:
                half = np.fromstring(geometry.find('box').get('size'), sep=' ')/2
                bounds = np.array([-half, half])
                shape = trimesh.creation.box(extents=2*half)
            elif geometry.find('sphere') is not None:
                r = float(geometry.find('sphere').get('radius'))
                bounds = np.array([[-r]*3, [r]*3])
            else:
                raise ValueError('Unsupported collision geometry')
            transform = origin(collision.find('origin'))
            # Thin sensor/tool geometry needs finer covering cells; a large
            # circumsphere otherwise reaches back into the wrist geometry.
            pitch = min(options.sphere_pitch, 0.02) if link.get('name') == 'ft_sensor_link' else options.sphere_pitch
            if options.sphere_model == 'curobo' and geometry.find('sphere') is None:
                count = int(np.clip(np.ceil(shape.area/(options.sphere_pitch**2)), 6, 24))
                points, radii = fit_spheres_to_mesh(
                    shape, count, surface_sphere_radius=0.002,
                    fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE)
                fitted = [{'center': point.tolist(), 'radius': float(radius)}
                          for point, radius in zip(points, radii)]
                method = 'curobo_voxel_volume_sample_surface'
            elif options.sphere_model == 'slabs' and link.get('name') == 'link_base':
                fitted, method = sphere_cover(bounds, 0.08), 'stationary_base_aabb'
            elif options.sphere_model == 'slabs' and link.get('name') == 'link5':
                fitted, method = sphere_cover(bounds, 0.08), 'wrist_aabb'
            elif options.sphere_model == 'slabs' and link.get('name') == 'ft_sensor_link':
                fitted, method = sphere_cover(bounds, 0.02), 'thin_sensor_aabb'
            elif options.sphere_model == 'slabs' and geometry.find('sphere') is None:
                fitted, method = sliced_cover(shape, options.sphere_pitch)
            else:
                fitted, method = sphere_cover(bounds, pitch), 'aabb'
            fitting.setdefault(link.get('name'), []).append(method)
            for ball in fitted:
                ball['center'] = (transform[:3, :3] @ ball['center'] + transform[:3, 3]).tolist()
                balls.append(ball)
        if balls:
            spheres[link.get('name')] = balls
    ignore = {}
    for pair in srdf.findall('disable_collisions'):
        a, b = pair.get('link1'), pair.get('link2')
        if a in spheres and b in spheres:
            ignore.setdefault(a, []).append(b)
    transforms = fk(root, saved)
    overlaps = []
    for a, b in itertools.combinations(spheres, 2):
        if b in ignore.get(a, []) or a in ignore.get(b, []):
            continue
        points = {}
        for name in (a, b):
            transform = transforms[name]
            points[name] = np.array([ball['center'] for ball in spheres[name]]) @ transform[:3, :3].T + transform[:3, 3]
        radii_a = np.array([ball['radius'] for ball in spheres[a]])
        radii_b = np.array([ball['radius'] for ball in spheres[b]])
        penetration = radii_a[:, None] + radii_b[None, :] - np.linalg.norm(
            points[a][:, None, :] - points[b][None, :, :], axis=2)
        if penetration.max() > 0:
            overlaps.append({'links': [a, b], 'sphere_overlap_m': float(penetration.max())})
    with tempfile.TemporaryDirectory(prefix='uf850-cumotion-') as tmp:
        urdf = Path(tmp)/'uf850.urdf'
        ET.ElementTree(root).write(urdf, encoding='unicode')
        cfg = {'kinematics': {
            'urdf_path': str(urdf), 'base_link': 'link_base', 'ee_link': 'link_tcp',
            'link_names': list(spheres), 'collision_link_names': list(spheres),
            'collision_spheres': spheres, 'collision_sphere_buffer': 0.0,
            'self_collision_ignore': ignore,
            'self_collision_buffer': {link: 0.0 for link in spheres},
            'use_global_cumul': True,
            'cspace': {'joint_names': names, 'retract_config': q,
                       'null_space_weight': [1.0]*6, 'cspace_distance_weight': [1.0]*6,
                       'max_acceleration': min(accelerations), 'max_jerk': 10.0,
                       'position_limit_clip': 0.0}}}
        if options.export_model:
            destination = Path(options.export_model).resolve()
            destination.mkdir(parents=True, exist_ok=False)
            ET.ElementTree(root).write(destination/'uf850.urdf', encoding='unicode')
            ET.ElementTree(srdf).write(destination/'uf850.srdf', encoding='unicode')
            cfg['kinematics']['urdf_path'] = str(destination/'uf850.urdf')
            (destination/'uf850.yml').write_text(yaml.safe_dump({'robot_cfg': cfg}))
            print(json.dumps({'exported_model': str(destination),
                              'camera_included': False, 'payload_included': False}), flush=True)
        if options.export_only:
            return
        args = TensorDeviceType()
        t0 = time.monotonic()
        planner = MotionGen(MotionGenConfig.load_from_robot_config(
            cfg, {'cuboid': {'floor': {'dims': [2., 2., .1],
                                      'pose': [0, 0, -.15, 1, 0, 0, 0]}}},
            args, interpolation_dt=.02))
        rng = np.random.default_rng(42)
        samples = [q] + [rng.uniform([limits[n][0] for n in names],
                                    [limits[n][1] for n in names]).tolist() for _ in range(20)]
        xyz_error, angle_error = 0., 0.
        for sample in samples:
            state = JointState.from_position(args.to_device([sample]), joint_names=names)
            pose = planner.compute_kinematics(state).ee_pose
            actual = fk(root, dict(zip(names, sample)))['link_tcp']
            xyz_error = max(xyz_error, float(np.linalg.norm(
                pose.position[0].cpu().numpy()-actual[:3, 3])))
            wxyz = pose.quaternion[0].cpu().numpy()
            delta = Rotation.from_quat(wxyz[[1, 2, 3, 0]]).inv() * Rotation.from_matrix(actual[:3, :3])
            angle_error = max(angle_error, float(delta.magnitude()))
        assert xyz_error < 1e-5 and angle_error < 1e-4, (xyz_error, angle_error)
        report = {'model': 'UF850_OFFLINE_INCOMPLETE_SCENE', 'robot_execution': False,
                  'camera_included': False, 'payload_included': False,
                  'sphere_count': sum(map(len, spheres.values())),
                  'sphere_pitch_m': options.sphere_pitch,
                  'sphere_model': options.sphere_model, 'fitting_methods': fitting,
                  'spheres_per_link': {name: len(balls) for name, balls in spheres.items()},
                  'initial_sphere_overlaps': overlaps,
                  'collision_links': list(spheres), 'fk_samples': len(samples),
                  'max_fk_position_error_m': xyz_error,
                  'max_fk_angle_error_rad': angle_error, 'joint_limits_rad': limits,
                  'planning_limit_overrides': planning_limit_overrides}
        print(json.dumps(dict(report, phase='kinematics_verified')), flush=True)
        if options.plan:
            from validate_path import MeshValidator
            validator = MeshValidator(root, srdf)
            planner.warmup(enable_graph=True)
            torch.cuda.synchronize()
            report['initialization_and_warmup_s'] = time.monotonic()-t0
            start = JointState.from_position(args.to_device([q]), joint_names=names)
            goal_q = q.copy()
            goal_q[0] += .10
            assert limits['joint1'][0] <= goal_q[0] <= limits['joint1'][1]
            goal = planner.compute_kinematics(JointState.from_position(
                args.to_device([goal_q]), joint_names=names)).ee_pose.clone()
            joint_goal = JointState.from_position(args.to_device([goal_q]), joint_names=names)
            tj = time.monotonic()
            joint_result = planner.plan_single_js(start, joint_goal, MotionGenPlanConfig(max_attempts=2))
            torch.cuda.synchronize()
            report['joint_goal_test'] = {'success': bool(joint_result.success.item()),
                                         'status': str(joint_result.status),
                                         'wall_s': time.monotonic()-tj}
            print(json.dumps({'phase': 'joint_goal_test', **report['joint_goal_test']}), flush=True)
            t0 = time.monotonic()
            result = planner.plan_single(start, goal, MotionGenPlanConfig(max_attempts=2))
            torch.cuda.synchronize()
            report.update(success=bool(result.success.item()), status=str(result.status),
                          planning_wall_s=time.monotonic()-t0)
            for label, planned in [('joint_goal_test', joint_result), ('pose_goal_test', result)]:
                if bool(planned.success.item()):
                    trajectory = planned.get_interpolated_plan()
                    if list(trajectory.joint_names) != names:
                        raise ValueError('Unexpected trajectory joint order')
                    try:
                        validation = validator.check(trajectory.position.cpu().numpy(),
                                                     names, limits, q, goal_q)
                    except ValueError as error:
                        validation = {'passed': False, 'error': str(error)}
                    report[label + '_validation'] = validation
        print(json.dumps(report), flush=True)
        if options.plan and (not report['success'] or not all(
                report.get(label + '_validation', {}).get('passed', False)
                for label in ('joint_goal_test', 'pose_goal_test'))):
            raise SystemExit(2)


if __name__ == '__main__':
    main()
