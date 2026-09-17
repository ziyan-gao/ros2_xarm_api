import math
import time

import pytest

from safe_servo_visualization.staging_slots_node import StagingSlots


def _staging_geometry():
    staging = object.__new__(StagingSlots)
    staging.slot_x_min = -0.375
    staging.slot_x_max = 0.375
    staging.slot_y_min = 0.180
    staging.slot_y_max = 0.680
    staging.slot_size = 0.250
    staging.surface_z = 0.0
    return staging


def test_six_staging_slots_use_requested_flb_corners():
    staging = _staging_geometry()

    assert staging._make_slots() == pytest.approx([
        (-0.375, 0.180, 0.0),
        (-0.125, 0.180, 0.0),
        (0.125, 0.180, 0.0),
        (-0.375, 0.430, 0.0),
        (-0.125, 0.430, 0.0),
        (0.125, 0.430, 0.0),
    ])


def test_store_target_aligns_unrotated_item_flb_to_slot_corner():
    staging = _staging_geometry()
    geometry = {
        'size': (0.200, 0.100, 0.080),
        # Center is 40 mm along TCP +Z; target TCP points downward.
        'center': (0.0, 0.0, 0.040),
        'orientation': (1.0, 0.0, 0.0, 0.0),
    }

    pose = staging._store_target((-0.375, 0.180, 0.0), geometry)

    assert pose[:3] == pytest.approx((-0.275, 0.230, 0.080))
    assert abs(abs(pose[3]) - math.pi) < 1e-12
    assert pose[4:] == pytest.approx((0.0, 0.0), abs=1e-12)


def test_store_target_does_not_rotate_oversize_footprint_to_make_it_fit():
    staging = _staging_geometry()
    geometry = {
        'size': (0.260, 0.100, 0.080),
        'center': (0.0, 0.0, 0.040),
        'orientation': (1.0, 0.0, 0.0, 0.0),
    }

    with pytest.raises(ValueError, match='unrotated 250x250 mm slot'):
        staging._store_target((-0.375, 0.180, 0.0), geometry)


def test_retrieval_clearance_uses_higher_predicted_top_when_release_is_low():
    record = {
        'release_tcp_pose': (100.0, 200.0, 75.0, math.pi, 0.0, 0.0),
        'target_tcp_pose': (0.1, 0.2, 0.080, math.pi, 0.0, 0.0),
    }

    assert StagingSlots._retrieval_contact_reference_z(record) == pytest.approx(
        0.080)


def test_retrieval_clearance_keeps_higher_measured_release_pose():
    record = {
        'release_tcp_pose': (100.0, 200.0, 85.0, math.pi, 0.0, 0.0),
        'target_tcp_pose': (0.1, 0.2, 0.080, math.pi, 0.0, 0.0),
    }

    assert StagingSlots._retrieval_contact_reference_z(record) == pytest.approx(
        0.085)


def test_store_transfer_height_is_relative_to_live_pallet_origin():
    # Contact TCP Z=80 mm puts the item bottom on the slot surface at 0 mm.
    # Pallet origin -181 mm plus 480 mm gives item-bottom Z=299 mm, so the
    # transfer TCP is 80+299=379 mm in link_base.
    assert StagingSlots._transfer_tcp_z_for_item_bottom(
        0.080, 0.0, -0.181, 0.480) == pytest.approx(0.379)


def test_pregrasp_pose_errors_report_actual_minus_target():
    errors = StagingSlots._pregrasp_pose_errors(
        (-0.2682, 0.2677, 0.1492),
        {'x_m': -0.2677, 'y_m': 0.2676, 'pregrasp_z_m': 0.1500})

    assert errors == pytest.approx((-0.0005, 0.0001, -0.0008))


def test_tcp_settle_requires_three_new_converged_servo_samples():
    staging = object.__new__(StagingSlots)
    staging.retrieval_settle_started = time.monotonic()
    staging.retrieval_settle_timeout = 5.0
    staging.retrieval_settle = 0.0
    staging.retrieval_settle_samples = 3
    staging.retrieval_settle_after_sequence = 10
    staging.retrieval_last_checked_sequence = 10
    staging.retrieval_converged_samples = 0
    staging.retrieval_xy_tolerance = 0.015
    staging.retrieval_z_tolerance = 0.015
    staging.status_timeout = 1.0
    staging.servo_status_time = time.monotonic()
    staging.servo_status = {
        'tcp_x_m': 0.1005,
        'tcp_y_m': 0.1995,
        'tcp_z_m': 0.3005,
        'joint_state_age_sec': 0.01,
    }
    staging._fault = lambda reason: pytest.fail(reason)
    snapshot = {'x_m': 0.1, 'y_m': 0.2, 'pregrasp_z_m': 0.3}

    for sequence in (11, 12):
        staging.servo_status_sequence = sequence
        assert not staging._settle_tcp(snapshot, 'test pose')

    staging.servo_status_sequence = 13
    assert staging._settle_tcp(snapshot, 'test pose')


def test_chained_store_finishes_at_raised_waypoint_without_observation():
    staging = object.__new__(StagingSlots)
    staging.return_to_observation = False
    staging.pickup_status = {
        'release_tcp_pose_mm_rad': [100.0, 200.0, 80.0, math.pi, 0.0, 0.0],
        'place_fallback_reason': '',
    }
    staging.scene_status = {'placed_item_ids': ['placed_item_12']}
    staging.placed_ids_before_store = set()
    staging.pending_record = {'slot': 2, 'item_id': 'held_item'}
    staging.expected_store_place_operation_id = 7
    staging.occupied = {}
    staging.operation = 'store'
    staging.active_slot = 2
    staging.publish_status = lambda: None
    staging._begin_observation = lambda: pytest.fail(
        'chained store must not plan observation')

    staging._complete_safe_servo_store()

    assert staging.state == StagingSlots.SUCCEEDED
    assert staging.last_result == 'store completed for slot 2'
    assert staging.occupied[2]['placed_obstacle_id'] == 'placed_item_12'
