"""Offline sampled validation. No ROS or execution interfaces.

Uses original URDF collision shapes, not cuRobo's fitted spheres. This is
sampled collision detection, NOT a continuous collision-free certificate.
"""
import itertools
import time

import fcl
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from uf850_smoke import fk, origin


def densify(path, step=0.005):
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or len(path) == 0 or not np.isfinite(path).all():
        raise ValueError('Invalid trajectory')
    out = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b-a))/step)))
        out.extend(a+(b-a)*i/n for i in range(1, n+1))
    return np.array(out)


class MeshValidator:
    def __init__(self, root, srdf):
        self.root = root
        self.shapes = []
        for link in root.findall('link'):
            for collision in link.findall('collision'):
                g = collision.find('geometry')
                if g.find('mesh') is not None:
                    m = g.find('mesh')
                    mesh = trimesh.load(m.get('filename'), force='mesh')
                    mesh.apply_scale(np.fromstring(m.get('scale', '1 1 1'), sep=' '))
                    shape = fcl.BVHModel()
                    shape.beginModel(len(mesh.vertices), len(mesh.faces))
                    shape.addSubModel(np.asarray(mesh.vertices), np.asarray(mesh.faces, dtype=np.int32))
                    shape.endModel()
                elif g.find('cylinder') is not None:
                    c = g.find('cylinder')
                    shape = fcl.Cylinder(float(c.get('radius')), float(c.get('length')))
                elif g.find('box') is not None:
                    shape = fcl.Box(*np.fromstring(g.find('box').get('size'), sep=' '))
                elif g.find('sphere') is not None:
                    shape = fcl.Sphere(float(g.find('sphere').get('radius')))
                else:
                    raise ValueError('Unsupported URDF collision geometry')
                self.shapes.append((link.get('name'), origin(collision.find('origin')),
                                    fcl.CollisionObject(shape)))
        ignored = {frozenset((p.get('link1'), p.get('link2')))
                   for p in srdf.findall('disable_collisions')}
        self.pairs = [(a, b) for a, b in itertools.combinations(range(len(self.shapes)), 2)
                      if self.shapes[a][0] != self.shapes[b][0]
                      and frozenset((self.shapes[a][0], self.shapes[b][0])) not in ignored]
        # Exact same synthetic world as the smoke planner, not the real table.
        self.floor = fcl.CollisionObject(fcl.Box(2., 2., .1),
                                        fcl.Transform(np.array([0., 0., -.15])))

    def check(self, path, names, limits, start, goal):
        t0 = time.monotonic()
        path = densify(path)
        if path.shape[1] != len(names):
            raise ValueError('Joint count mismatch')
        low, high = np.array([limits[n] for n in names]).T
        if np.any(path < low-1e-6) or np.any(path > high+1e-6):
            raise ValueError('Trajectory exceeds joint bounds')
        if np.max(np.abs(path[0]-start)) > .01:
            raise ValueError('Trajectory start mismatch')
        reference = fk(self.root, dict(zip(names, goal)))['link_tcp']
        max_error = np.zeros(3)
        distance = 0.
        previous = None
        for i, q in enumerate(path):
            frames = fk(self.root, dict(zip(names, q)))
            tcp = frames['link_tcp']
            # Rotation-vector components in the desired TCP orientation frame.
            # Diagnostic bound; not a claim that cuMotion enforces path constraints.
            error = Rotation.from_matrix(reference[:3, :3].T @ tcp[:3, :3]).as_rotvec()
            max_error = np.maximum(max_error, np.abs(error))
            if previous is not None:
                distance += np.linalg.norm(tcp[:3, 3]-previous)
            previous = tcp[:3, 3].copy()
            for link, local, obj in self.shapes:
                tf = frames[link] @ local
                obj.setTransform(fcl.Transform(tf[:3, :3], tf[:3, 3]))
                if fcl.collide(obj, self.floor, fcl.CollisionRequest(), fcl.CollisionResult()):
                    raise ValueError(f'Floor collision at sample {i}: {link}')
            for a, b in self.pairs:
                if fcl.collide(self.shapes[a][2], self.shapes[b][2],
                               fcl.CollisionRequest(), fcl.CollisionResult()):
                    raise ValueError(f'Self collision at sample {i}: '
                                     f'{self.shapes[a][0]} / {self.shapes[b][0]}')
        pos_error = float(np.linalg.norm(tcp[:3, 3]-reference[:3, 3]))
        angle_error = float(Rotation.from_matrix(reference[:3, :3].T @ tcp[:3, :3]).magnitude())
        if pos_error > .005 or angle_error > .05:
            raise ValueError(f'Goal mismatch: {pos_error} m / {angle_error} rad')
        tilt_ok = bool(np.all(max_error[:2] <= np.pi/6+1e-6))
        return {'passed': tilt_ok, 'sampled_mesh_collision_free': True,
                'samples': len(path), 'max_joint_sample_step_rad': .005,
                'orientation_reference': 'goal TCP; local rotation-vector components',
                'max_orientation_error_deg': np.rad2deg(max_error).tolist(),
                'within_30_degree_xy_bound': tilt_ok,
                'eef_path_length_m': float(distance), 'goal_position_error_m': pos_error,
                'goal_angle_error_rad': angle_error, 'validation_wall_s': time.monotonic()-t0}
