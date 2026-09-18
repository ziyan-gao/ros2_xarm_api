import math

import numpy as np
import pytest

from safe_servo_visualization.pickup_pipeline_node import (
    PickupPipeline,
    round_dimension_down_m,
    summarize_box_samples,
)


def test_twenty_stable_box_samples_are_averaged():
    samples = []
    for index in range(20):
        offset = (index - 9.5) * 0.0001
        yaw = offset
        samples.append((
            np.array([0.40 + offset, -0.10, 0.08]),
            np.array([0.0, 0.0, math.sin(yaw / 2.0),
                      math.cos(yaw / 2.0)]),
            np.array([0.149 + offset, 0.136 - offset, 0.150]),
        ))

    summary = summarize_box_samples(samples)

    assert summary['position'] == pytest.approx([0.40, -0.10, 0.08])
    assert summary['dimensions'] == pytest.approx([0.149, 0.136, 0.150])
    assert summary['position_spread_m'] < 0.001
    assert math.degrees(summary['angular_spread_rad']) < 0.1
    assert summary['dimension_spread_m'] < 0.001


@pytest.mark.parametrize(
    ('measured_m', 'expected_m'),
    [(0.149, 0.145), (0.136, 0.135), (0.131, 0.130), (0.150, 0.150)],
)
def test_final_xy_dimension_is_rounded_down(measured_m, expected_m):
    assert round_dimension_down_m(measured_m, 5.0) == pytest.approx(expected_m)


def test_quaternion_sign_does_not_make_stable_samples_look_unstable():
    samples = [
        (np.zeros(3), np.array([0.0, 0.0, 0.0, sign]), np.ones(3))
        for sign in (1.0, -1.0) for _ in range(10)
    ]

    summary = summarize_box_samples(samples)

    assert summary['angular_spread_rad'] == pytest.approx(0.0)


def test_pipeline_status_uses_rounded_xy_without_changing_motion_snapshot():
    node = object.__new__(PickupPipeline)
    raw = {'size_x_m': 0.149, 'size_y_m': 0.136, 'size_z_m': 0.151}
    node.pickup_status = {'corrected_object': raw}
    node.xy_dimension_rounding_mm = 5.0

    displayed = node._rounded_corrected_object()

    assert displayed['size_x_m'] == pytest.approx(0.145)
    assert displayed['size_y_m'] == pytest.approx(0.135)
    assert displayed['size_z_m'] == pytest.approx(0.151)
    assert raw['size_x_m'] == pytest.approx(0.149)
