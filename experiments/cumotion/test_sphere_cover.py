"""Geometry-only regression checks. Run inside the isolated test image."""
import unittest
import numpy as np
import trimesh
import math
import xml.etree.ElementTree as ET

import pytest

from uf850_smoke import apply_planning_joint_limits, sliced_cover, sphere_cover


def test_planning_joint_limits_only_narrow_requested_intervals():
    root = ET.fromstring('''<robot>
      <joint name="joint3" type="revolute"><limit lower="-3" upper="1"/></joint>
      <joint name="joint5" type="revolute"><limit lower="-2" upper="2"/></joint>
    </robot>''')
    applied = apply_planning_joint_limits(
        root, lower_deg={'joint3': -130.0}, upper_deg={'joint5': 40.0})
    joints = {j.get('name'): j.find('limit') for j in root.findall('joint')}
    assert float(joints['joint3'].get('lower')) == pytest.approx(math.radians(-130.0))
    assert float(joints['joint3'].get('upper')) == 1.0
    assert float(joints['joint5'].get('lower')) == -2.0
    assert float(joints['joint5'].get('upper')) == pytest.approx(math.radians(40.0))
    assert set(applied) == {'joint3', 'joint5'}


def test_planning_joint_limits_cannot_expand_physical_interval():
    root = ET.fromstring(
        '<robot><joint name="joint3" type="revolute">'
        '<limit lower="-2" upper="1"/></joint></robot>')
    with pytest.raises(ValueError, match='may not expand physical interval'):
        apply_planning_joint_limits(root, lower_deg={'joint3': -130.0})


def assert_covered(test, points, balls):
    centers = np.array([b['center'] for b in balls])
    radii = np.array([b['radius'] for b in balls])
    for chunk in np.array_split(points, 20):
        if len(chunk):
            gaps = np.linalg.norm(chunk[:, None, :]-centers[None, :, :], axis=2)-radii
            test.assertLessEqual(float(gaps.min(axis=1).max()), 1e-8)


class CoverTests(unittest.TestCase):
    def test_aabb_interior(self):
        bounds = np.array([[-.06, -.1, -.02], [.03, .32, .02]])
        points = np.random.default_rng(4).uniform(*bounds, size=(5000, 3))
        assert_covered(self, points, sphere_cover(bounds, .04))

    def test_closed_slab_surface_and_interior(self):
        mesh = trimesh.creation.cylinder(radius=.045, height=.2, sections=64)
        balls, method = sliced_cover(mesh, .04)
        self.assertEqual(method, 'closed_mesh_slab_enclosure')
        surface, _ = trimesh.sample.sample_surface(mesh, 5000, seed=42)
        # Interpolation toward the centroid stays inside this convex test solid.
        interior = surface*np.random.default_rng(5).uniform(0, 1, (5000, 1))
        assert_covered(self, np.vstack([mesh.vertices, surface, interior]), balls)

    def test_open_mesh_falls_back(self):
        mesh = trimesh.creation.box(extents=[.1, .2, .3])
        mesh.update_faces(np.arange(len(mesh.faces)-1))
        balls, method = sliced_cover(mesh, .04)
        self.assertEqual(method, 'aabb_open_mesh_fallback')
        assert_covered(self, mesh.vertices, balls)


if __name__ == '__main__':
    unittest.main()
