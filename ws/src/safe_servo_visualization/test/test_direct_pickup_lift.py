from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from safe_servo_visualization.continuous_transport import ContinuousTransport
from safe_servo_visualization.clearance_transfer import ClearanceTransfer


class Harness(ClearanceTransfer, NS):
    TRANSPORT_PLANNING = 'PLANNING'


@pytest.mark.parametrize('context', ['pallet', 'staging_store'])
def test_direct_transfer_requires_vertical_lift_before_moveit(context):
    start, q = np.array([.2, .3, .4]), np.array([1., 0., 0., 0.])
    h = Harness(transport_moveit_direct_enabled=True, transport_is_pick=False,
           transport_is_return=False, transport_seed=(0.,)*6,
           transport_scene=None, transport_route_generation=0,
           transport_target=dict(pre_place_tcp_xyz_m=[.5, .6, .4],
                                 transfer_tcp_quaternion_xyzw=q,
                                 transport_corner_clearance_z_m=.5,
                                 transfer_context=context),
           planning_scene_status=dict(camera_collision_applied=True,
                                      add_placed_item_obstacle=True))
    h._transport_pose = lambda _: (start, q)
    h.transport_motion_plan = NS(service_is_ready=lambda: True)
    h._transport_ceiling = lambda: .8
    h.get_logger = lambda: Mock()
    h._alternative_next = Mock()
    h._alternative_moveit = Mock()
    h._fault = Mock()
    ContinuousTransport._transport_start_fk(h, NS(result=lambda: None))
    h._fault.assert_not_called()
    h._alternative_next.assert_called_once()
    h._alternative_moveit.assert_not_called()
    assert len(h.alternative_segments) == 1  # Only lift is planned before execution.
    first = h.alternative_segments[0]
    assert first[0] == 'cartesian'
    assert first[1] == pytest.approx([.2, .3, .54])
    assert h.transport_safe_z == pytest.approx(.53)
    assert first[2] == pytest.approx(q)
    assert not h.direct_lift_verified
    assert h.alternative_seed == (0.,)*6
    h.transport_verify_error = .001
    h.transport_verify_joints = (.1,)*6
    assert h._advance_clearance_phase(np.array([.2, .3, .539]), q)
    assert h.clearance_phase == 'transfer'
    assert h.alternative_seed == (.1,)*6
    assert h.alternative_segments[0][0] == 'moveit'
    assert h.alternative_segments[0][1] == pytest.approx([.5, .6, .54])
    assert h.alternative_current_xyz[2] > h.transport_safe_z
    h.transport_verify_joints = (.2,)*6
    assert h._advance_clearance_phase(np.array([.5, .6, .539]), q)
    assert h.clearance_phase == 'descend'
    assert h.alternative_segments[0][0] == 'cartesian'
    assert h.alternative_segments[0][1] == pytest.approx([.5, .6, .4])


def test_slot_clearance_uses_fixed_maximum_payload_drop():
    h = Harness(transport_is_pick=False, active_pickup_snapshot={'pickup_source': 'buffer'},
                transport_target={'transfer_context': 'pallet'}, transport_slot_clearance_z_m=.35,
                transport_scene=dict(attached_item_id='box', attached_item_size_m=[.1,.1,.2],
                                     attached_item_center_in_tcp_m=[0,0,.1],
                                     attached_item_orientation_in_tcp_xyzw=[0,0,0,1]),
                transport_max_payload_drop_m=.3, transport_seed=(0.,)*6)
    h._transport_ceiling = lambda: 1.
    h.get_logger = lambda: Mock()
    h._plan_clearance_phase = Mock()
    h._begin_clearance_transfer(np.array([0.,0.,.3]), np.array([1.,0.,0.,0.]),
                                np.array([.4,.4,.4]), np.array([1.,0.,0.,0.]), .5)
    assert h.clearance_source_z == pytest.approx(.66)
    assert h.clearance_destination[2] == pytest.approx(.81)
    assert h.transport_safe_z == pytest.approx(.65)


def test_same_area_top_face_inspection_uses_direct_cartesian_motion():
    q = np.array([1., 0., 0., 0.])
    h = Harness(
        transport_is_pick=True,
        pick_cross_area=False,
        clearance_last_region='buffer',
        active_pickup_snapshot={'pickup_source': 'buffer'},
        transport_target={
            'target': 'top_face_view',
            'planned_pregrasp': {
                'inspection_only': True,
                'pickup_source': 'pallet',
            },
            'transfer_context': 'known_pick',
        },
        transport_slot_clearance_z_m=.35,
        transport_scene=None,
        transport_seed=(0.,) * 6,
    )
    h._transport_ceiling = lambda: 1.
    h.get_logger = lambda: Mock()
    h._plan_clearance_phase = Mock()
    final = np.array([.4, .2, .5])
    h._begin_clearance_transfer(np.array([0., 0., .3]), q, final, q, .5)
    assert h.clearance_source_z == pytest.approx(.36)
    assert h.clearance_destination == pytest.approx(final)
    assert h.clearance_phase == 'inspection_cartesian'


def test_cross_area_top_face_inspection_retains_terminal_cartesian_approach():
    q = np.array([1., 0., 0., 0.])
    h = Harness(
        transport_is_pick=True,
        pick_cross_area=True,
        clearance_last_region='pallet',
        active_pickup_snapshot={'pickup_source': 'pallet'},
        transport_target={
            'target': 'top_face_view',
            'planned_pregrasp': {
                'inspection_only': True,
                'pickup_source': 'buffer',
            },
            'transfer_context': 'known_pick',
        },
        transport_slot_clearance_z_m=.35,
        transport_scene=None,
        transport_seed=(0.,) * 6,
    )
    h._transport_ceiling = lambda: 1.
    h.get_logger = lambda: Mock()
    h._plan_clearance_phase = Mock()
    h._begin_clearance_transfer(
        np.array([0., 0., .5]), q, np.array([.4, .2, .35]), q, .5)
    assert h.clearance_destination == pytest.approx([.4, .2, .39])
    assert h.clearance_final[0] == pytest.approx([.4, .2, .35])
    assert h.clearance_phase == 'lift'


def test_non_inspection_pick_retains_terminal_clearance_margin():
    q = np.array([1., 0., 0., 0.])
    h = Harness(
        transport_is_pick=True,
        active_pickup_snapshot={'pickup_source': 'pallet'},
        transport_target={
            'target': 'known_pick',
            'planned_pregrasp': {'pickup_source': 'pallet'},
            'transfer_context': 'known_pick',
        },
        transport_slot_clearance_z_m=.35,
        transport_scene=None,
        transport_seed=(0.,) * 6,
    )
    h._transport_ceiling = lambda: 1.
    h.get_logger = lambda: Mock()
    h._plan_clearance_phase = Mock()
    h._begin_clearance_transfer(
        np.array([0., 0., .3]), q, np.array([.4, .2, .5]), q, .5)
    assert h.clearance_destination[2] == pytest.approx(.54)


@pytest.mark.parametrize('error', [.003, .009])
def test_phase_waits_for_settled_feedback(error):
    h = Harness(direct_moveit_active=True, clearance_phase='lift',
                transport_verify_error=error)
    h._plan_clearance_phase = Mock()
    assert h._advance_clearance_phase(np.zeros(3), np.array([1.,0.,0.,0.]))
    assert h.clearance_phase == 'lift'
    h._plan_clearance_phase.assert_not_called()


def test_observation_return_ends_after_moveit_without_contact_descent():
    h = Harness(direct_moveit_active=True, clearance_phase='transfer',
                transport_is_return=True)
    h._plan_clearance_phase = Mock()
    assert not h._advance_clearance_phase(np.zeros(3), np.array([1.,0.,0.,0.]))
    h._plan_clearance_phase.assert_not_called()
