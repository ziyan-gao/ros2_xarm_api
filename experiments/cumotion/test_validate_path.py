import unittest
import xml.etree.ElementTree as ET

import numpy as np

from validate_path import MeshValidator, densify


def model(z=0.5, obstacle=False):
    box = '<collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>'
    extra = (f'<link name="obstacle">{box}</link><joint name="fixed" type="fixed">'
             f'<parent link="world"/><child link="obstacle"/><origin xyz="0 0 {z}"/>'
             '</joint>') if obstacle else ''
    return ET.fromstring(
        f'<robot name="test"><link name="world"/><link name="link_tcp">{box}</link>'
        '<joint name="joint1" type="revolute"><parent link="world"/>'
        f'<child link="link_tcp"/><origin xyz="0 0 {z}"/><axis xyz="0 0 1"/>'
        f'</joint>{extra}</robot>')


class ValidationTests(unittest.TestCase):
    def check_path(self, root, path=((0.,), (0.1,)), srdf='<robot/>'):
        return MeshValidator(root, ET.fromstring(srdf)).check(
            np.array(path), ['joint1'], {'joint1': [-1., 1.]}, [0.], [0.1])

    def test_valid(self):
        self.assertTrue(self.check_path(model())['passed'])

    def test_densify(self):
        samples = densify([[0, 0], [.1, -.08]])
        self.assertLessEqual(np.max(np.abs(np.diff(samples, axis=0))), .005+1e-12)
        with self.assertRaises(ValueError):
            densify([[np.nan]])

    def test_bounds(self):
        with self.assertRaisesRegex(ValueError, 'joint bounds'):
            self.check_path(model(), ((0.,), (1.1,)))

    def test_floor(self):
        with self.assertRaisesRegex(ValueError, 'Floor collision'):
            self.check_path(model(z=-.15))

    def test_self_collision(self):
        with self.assertRaisesRegex(ValueError, 'Self collision'):
            self.check_path(model(obstacle=True))

    def test_srdf_exclusion(self):
        self.assertTrue(self.check_path(model(obstacle=True), srdf=(
            '<robot><disable_collisions link1="obstacle" link2="link_tcp"/></robot>'))['passed'])

    def test_tilt_rejection(self):
        root = model()
        root.find('joint/axis').set('xyz', '1 0 0')
        self.assertFalse(self.check_path(root, ((0.,), (.8,), (.1,)))['passed'])


if __name__ == '__main__':
    unittest.main()
