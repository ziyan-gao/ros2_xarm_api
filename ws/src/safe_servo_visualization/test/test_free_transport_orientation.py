"""Free overhead rotation must not relax low-column or final-pose checks."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from safe_servo_visualization.continuous_transport import ContinuousTransport


DOWN = np.array([1., 0., 0., 0.])
SIDE = np.array([0., math.sqrt(.5), 0., math.sqrt(.5)])


def check_geometry(xyz, q, final=False, ceiling=None, local_clearance=True):
    node = ContinuousTransport.__new__(ContinuousTransport)
    node.transport_start_xyz = np.array([.2, .5, .2])
    node.transport_end = np.array([.3, -.6, .2])
    if final:
        node.transport_end = np.array(xyz)  # position matches; test orientation alone
    node.transport_start_q = node.transport_end_q = DOWN
    node.transport_scene = dict(attached_item_size_m=[.4, .15, .1],
                                attached_item_center_in_tcp_m=[0., 0., .05],
                                attached_item_orientation_in_tcp_xyzw=[0., 0., 0., 1.])
    node.transport_clearance = .47
    node.transport_local_clearance_validation_enabled = local_clearance
    node.transport_high_z, node.transport_safe_z = .65, .57
    node.servo_bounds_mm = [-1000, 1000, -1000, 1000, -100, 1000]
    if ceiling is not None:
        node.transport_workspace_z_max_mm = ceiling
    node.transport_check_index = 0
    node.transport_checks = [((), 0.)] * (1 if final else 2)
    node.transport_descent_time = None
    node._transport_pose = lambda _: (np.array(xyz), q)
    node._transport_validate_next = Mock()
    node._transport_reject_geometry = Mock()
    node._fault = Mock()
    node._transport_geometry_checked(NS(result=lambda: None))
    return node


@pytest.mark.parametrize('xy', [[.4, .1], [.2, .5], [.3, -.6]])
def test_overhead_tilt_allowed_including_above_columns(xy):
    node = check_geometry([*xy, .8], SIDE)
    node._transport_validate_next.assert_called_once()
    node._transport_reject_geometry.assert_not_called()
    node._fault.assert_not_called()


@pytest.mark.parametrize('xy', [[.4, .1], [.2, .5], [.3, -.6]])
def test_rotated_item_bottom_not_tcp_alone_must_clear_pallet(xy):
    # TCP is above nominal safe Z (.57), but the tilted wide box bottom is .40.
    node = check_geometry([*xy, .60], SIDE)
    node._transport_reject_geometry.assert_called_once()
    node._transport_validate_next.assert_not_called()


@pytest.mark.parametrize('xy', [[.2, .5], [.3, -.6]])
def test_low_vertical_column_retains_required_orientation(xy):
    node = check_geometry([*xy, .2], DOWN)
    node._transport_validate_next.assert_called_once()
    node._fault.assert_not_called()
    wrong_yaw = check_geometry([*xy, .2], np.array([0., 1., 0., 0.]))
    wrong_yaw._transport_reject_geometry.assert_called_once()


def test_disabled_local_clearance_relies_on_virtual_obstacle_collision():
    node = check_geometry([.4, .1, .60], SIDE, ceiling=0., local_clearance=False)
    node._transport_reject_geometry.assert_not_called()
    node._transport_validate_next.assert_called_once()


def test_free_orientation_does_not_relax_endpoint_orientation():
    node = check_geometry([.3, -.6, .8], SIDE, final=True)
    node._fault.assert_called_once()
    assert 'pre-place pose' in node._fault.call_args.args[0]
    node._transport_validate_next.assert_not_called()


def test_workspace_ceiling_still_enforced():
    node = check_geometry([.4, .1, 1.1], SIDE)
    node._transport_reject_geometry.assert_called_once_with('timed transport exceeds workspace ceiling')


def test_disabled_transfer_ceiling_accepts_height_but_retains_clearance_checks():
    node = check_geometry([.4, .1, 1.1], SIDE, ceiling=0.)
    node._transport_reject_geometry.assert_not_called()
    node._transport_validate_next.assert_called_once()
    low = check_geometry([.4, .1, .60], SIDE, ceiling=0.)
    low._transport_reject_geometry.assert_called_once_with('timed transport cuts below item-bottom clearance')
    low._transport_validate_next.assert_not_called()


def test_explicit_transfer_ceiling_overrides_servo_ceiling():
    node = check_geometry([.4, .1, .900], SIDE, ceiling=800.)
    node._transport_reject_geometry.assert_called_once_with('timed transport exceeds workspace ceiling')
