import math

import numpy as np
import pytest

from box_marker_detection.depth_refinement_node import DepthBoxRefinement


def make_estimator():
    estimator = object.__new__(DepthBoxRefinement)
    estimator.minimum_points = 50
    estimator.ransac_threshold = 0.003
    estimator.depth_only_max_tilt = math.radians(8.0)
    estimator.depth_only_dimension_bounds = (0.04, 0.45)
    estimator.depth_only_support_z = 0.0
    estimator.depth_only_height_offset = -0.02
    estimator.depth_only_base_z_bounds = (0.07, 0.30)
    estimator.depth_only_tcp_x_bounds = (0.05, 0.40)
    estimator.depth_only_tcp_y_bounds = (-0.30, 0.30)
    estimator.depth_only_box_id = 0
    estimator.base_frame = 'link_base'
    estimator.rng = np.random.default_rng(7)
    return estimator


def test_depth_only_estimates_horizontal_box_and_rejects_outside_roi():
    estimator = make_estimator()
    x, y = np.meshgrid(
        np.linspace(0.16, 0.28, 25),
        np.linspace(-0.09, 0.09, 31))
    rng = np.random.default_rng(11)
    top = np.column_stack((
        x.ravel(), y.ravel(),
        0.15 + rng.normal(0.0, 0.0004, x.size)))
    outside = np.column_stack((
        np.full(200, 0.60), np.linspace(-0.1, 0.1, 200),
        np.full(200, 0.24)))

    result = estimator.estimate_depth_only_from_points(
        np.vstack((top, outside)), np.eye(4))

    assert result is not None
    marker, report = result
    assert sorted((marker.scale.x, marker.scale.y)) == pytest.approx(
        [0.12, 0.18], abs=0.006)
    assert marker.scale.z == pytest.approx(0.13, abs=0.002)
    assert report['raw_height_m'] == pytest.approx(0.15, abs=0.002)
    assert report['height_offset_m'] == -0.02
    assert report['method'] == 'depth_only'
    assert report['state'] == 'READY'


def test_depth_only_requires_enough_points_in_tcp_roi():
    estimator = make_estimator()
    points = np.column_stack((
        np.full(60, 0.60), np.linspace(-0.1, 0.1, 60),
        np.full(60, 0.15)))

    assert estimator.estimate_depth_only_from_points(
        points, np.eye(4)) is None
