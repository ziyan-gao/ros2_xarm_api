import math

import numpy as np

from safe_servo_visualization.pallet_localization_node import (
    level_quaternion, matrix_quat, quat_matrix, rpy_matrix)


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
