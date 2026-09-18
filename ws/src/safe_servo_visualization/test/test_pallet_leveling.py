import math
from types import SimpleNamespace

import numpy as np

from safe_servo_visualization.pallet_localization_node import (
    PalletLocalization, level_quaternion, matrix_quat, quat_matrix,
    rpy_matrix)


def test_level_quaternion_preserves_yaw_and_removes_roll_pitch():
    yaw = math.radians(88.0)
    measured = matrix_quat(
        rpy_matrix(math.radians(-2.0), math.radians(1.5), yaw))

    leveled = level_quaternion(measured)
    rotation = quat_matrix(leveled)

    assert np.allclose(leveled[:2], [0.0, 0.0])
    assert np.allclose(rotation[2], [0.0, 0.0, 1.0], atol=1e-12)
    actual_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    assert math.isclose(actual_yaw, yaw, abs_tol=1e-12)


class _Logger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)

    def info(self, _message):
        pass


def _loading_target_node():
    node = object.__new__(PalletLocalization)
    node.pallet_x = 0.45
    node.pallet_y = 0.55
    node.locked = False
    node._save_config = lambda: None
    node.publish_config_state = lambda: None
    node.get_logger = lambda: _Logger()
    acknowledgements = []
    node._acknowledge_loading_target = (
        lambda sequence, accepted, reason: acknowledgements.append(
            (sequence, accepted, reason)))
    return node, acknowledgements


def test_symmetric_target_uses_physical_corner_for_robot_and_virtual_for_bounds():
    node, acknowledgements = _loading_target_node()
    message = SimpleNamespace(data=[
        1, 4, 20, 20, 0, 0,
        110, 110, 150, 150, 150, 150,
        0, 0, 0,
    ])

    node.loading_target_callback(message)

    np.testing.assert_allclose(node.pre_place_xyz, [0.02, 0.02, 0.0])
    assert acknowledgements == [(1, True, 0)]


def test_symmetric_target_rejects_virtual_envelope_outside_pallet():
    node, acknowledgements = _loading_target_node()
    message = SimpleNamespace(data=[
        2, 4, 320, 20, 0, 0,
        110, 110, 150, 150, 150, 150,
        310, 0, 0,
    ])

    node.loading_target_callback(message)

    assert acknowledgements == [(2, False, 2)]
