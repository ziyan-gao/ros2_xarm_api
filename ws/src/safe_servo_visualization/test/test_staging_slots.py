import math
import time
from types import SimpleNamespace

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


def test_shared_speed_percentage_scales_staging_joint_limits():
    staging = object.__new__(StagingSlots)
    staging.joint_max_speed = 2.14
    staging.joint_max_acc = 10.0

    staging._motion_speed(SimpleNamespace(data=[0.5, 50.0]))

    assert staging.joint_speed == pytest.approx(1.07)
    assert staging.joint_acc == pytest.approx(5.0)


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


def test_yaw180_store_target_preserves_corner_with_off_center_grasp():
    staging = _staging_geometry()
    geometry = dict(size=(.2, .1, .08), center=(.02, .01, .04),
                    orientation=(1., 0., 0., 0.), store_tcp_q=(1., 0., 0., 0.))
    slot = (-.375, .18, 0.)
    original = staging._store_target(slot, geometry)
    flipped = staging._store_target(slot, {**geometry, 'store_tcp_q': (0., 1., 0., 0.)})
    assert flipped[0]-original[0] == pytest.approx(.04)
    assert flipped[1]-original[1] == pytest.approx(-.02)
    assert flipped[2] == pytest.approx(original[2])


def test_store_target_does_not_rotate_oversize_footprint_to_make_it_fit():
    staging = _staging_geometry()
    geometry = {
        'size': (0.260, 0.100, 0.080),
        'center': (0.0, 0.0, 0.040),
        'orientation': (1.0, 0.0, 0.0, 0.0),
    }

    with pytest.raises(ValueError, match='250x250 mm slot'):
        staging._store_target((-0.375, 0.180, 0.0), geometry)


def test_fixed_yaw_slot_target_aligns_rotated_bounds():
    staging = _staging_geometry()
    q = staging._quaternion_from_rpy(math.pi, 0., math.pi/2)
    geometry = dict(size=(.2, .1, .08), center=(0., 0., .04),
                    orientation=(1., 0., 0., 0.), store_tcp_q=q)
    pose = staging._store_target((0., 0., 0.), geometry)
    assert pose[:3] == pytest.approx((.05, .1, .08))
    assert staging._quaternion_from_rpy(*pose[3:]) == pytest.approx(q)


def test_fixed_diagonal_footprint_rejected_without_rotating_to_fit():
    staging = _staging_geometry()
    geometry = dict(size=(.2, .2, .08), center=(0., 0., .04),
                    orientation=(1., 0., 0., 0.),
                    store_tcp_q=staging._quaternion_from_rpy(math.pi, 0., math.pi/4))
    with pytest.raises(ValueError, match='does not fit'):
        staging._store_target((0., 0., 0.), geometry)


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
        'operation_id': 7,
        'release_tcp_pose_mm_rad': [100.0, 200.0, 80.0, math.pi, 0.0, 0.0],
        'place_fallback_reason': '',
        'continuous_return_completed': True,
        'return_to_observation': False,
    }
    staging.scene_status = {'placed_item_ids': ['placed_item_12']}
    staging.placed_ids_before_store = set()
    staging.pending_record = {'slot': 2, 'item_id': 'held_item', 'orientation': (1., 0., 0., 0.)}
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


def _store_release_fixture():
    staging = object.__new__(StagingSlots)
    staging.operation = 'store'
    staging.state = StagingSlots.PLACING_STORE
    staging.expected_store_place_operation_id = 7
    staging.expected_store_transfer_operation_id = 6
    staging.pending_record = {'slot': 0, 'item_id': 'held_item', 'orientation': (1., 0., 0., 0.)}
    staging.occupied = {}
    staging.placed_ids_before_store = {'placed_item_1'}
    staging.pickup_status = {
        'operation_id': 7, 'state': 'RETREATING',
        'release_tcp_pose_mm_rad': [100., 200., 80., math.pi, 0., 0.],
    }
    staging.scene_status = {'attached_item_id': '',
                            'placed_item_ids': ['placed_item_1', 'placed_item_2']}
    staging._mode_tick = staging._restore_tick = lambda: None
    staging.phase_started = None
    return staging


def test_release_records_actual_object_yaw_for_retrieval():
    staging = _store_release_fixture()
    staging.pickup_status['release_tcp_pose_mm_rad'][5] = math.pi/2
    assert staging._record_confirmed_store_release()
    assert staging.occupied[0]['object_yaw'] == pytest.approx(math.pi/2)
    assert staging.occupied[0]['orientation'] == (1., 0., 0., 0.)


def test_recorded_grasp_snapshot_rejects_bad_quaternion():
    from safe_servo_visualization.planning_scene_obstacles_node import PlanningSceneObstacles
    snapshot = dict(box_id=1, x_m=0., y_m=0., center_z_m=.1,
                    size_x_m=.1, size_y_m=.2, size_z_m=.1, yaw_rad=.4,
                    recorded_grasp=[0., 0., .05, 1., 0., 0., 0.])
    PlanningSceneObstacles._validate_snapshot(snapshot)
    snapshot['recorded_grasp'][3] = 0.
    with pytest.raises(ValueError, match='recorded grasp'):
        PlanningSceneObstacles._validate_snapshot(snapshot)


def test_store_inventory_commits_before_retreat_finishes_and_survives_fault():
    staging = _store_release_fixture()
    staging.tick()
    assert staging.state == StagingSlots.PLACING_STORE
    assert staging.occupied[0]['placed_obstacle_id'] == 'placed_item_2'
    assert staging.pending_record is not None  # motion still in progress
    staging.state = StagingSlots.FAULT
    staging.pickup_status['state'] = 'FAULT'
    staging.tick()
    assert staging.state == StagingSlots.FAULT
    assert staging.occupied[0]['release_tcp_pose'][2] == 80.


def test_late_detach_after_retreat_fault_still_commits_slot():
    staging = _store_release_fixture()
    staging.state = StagingSlots.FAULT
    staging.scene_status['placed_item_ids'] = ['placed_item_1']
    staging.tick()
    assert not staging.occupied
    staging.scene_status['placed_item_ids'].append('placed_item_2')
    staging.tick()
    assert 0 in staging.occupied
    assert staging.state == StagingSlots.FAULT


@pytest.mark.parametrize('invalid', ['old_operation', 'new_operation', 'attached',
                                    'missing_pose', 'nan_pose', 'no_detach', 'ambiguous'])
def test_store_release_requires_current_operation_and_detach_evidence(invalid):
    staging = _store_release_fixture()
    if invalid == 'old_operation':
        staging.pickup_status['operation_id'] = 6
    elif invalid == 'new_operation':
        staging.pickup_status['operation_id'] = 8
    elif invalid == 'attached':
        staging.scene_status['attached_item_id'] = 'held_item'
    elif invalid == 'missing_pose':
        staging.pickup_status['release_tcp_pose_mm_rad'] = None
    elif invalid == 'nan_pose':
        staging.pickup_status['release_tcp_pose_mm_rad'][2] = math.nan
    elif invalid == 'no_detach':
        staging.scene_status['placed_item_ids'] = ['placed_item_1']
    else:
        staging.scene_status['placed_item_ids'].append('placed_item_3')
    assert not staging._record_confirmed_store_release()
    assert not staging.occupied


def test_staging_transfer_failure_before_motion_uses_moveit_fallback():
    staging = object.__new__(StagingSlots)
    staging.state = StagingSlots.TRANSFERRING_STORE
    staging.store_transfer_phase = 'direct_executing'
    staging.expected_store_transfer_operation_id = 12
    staging.pickup_status = {
        'operation_id': 12,
        'operation_kind': 'transfer',
        'state': 'FAULT',
        'fault': 'joint interpolation is in collision',
        'direct_transfer_motion_started': False,
    }
    captured = {}
    staging._begin_store_moveit_fallback = lambda reason: captured.update(
        reason=reason)
    staging._fault = lambda reason: pytest.fail(reason)

    staging._tick_store_transfer()

    assert captured['reason'] == 'joint interpolation is in collision'


def test_staging_transfer_does_not_replan_after_motion_started():
    staging = object.__new__(StagingSlots)
    staging.state = StagingSlots.TRANSFERRING_STORE
    staging.store_transfer_phase = 'direct_executing'
    staging.expected_store_transfer_operation_id = 12
    staging.pickup_status = {
        'operation_id': 12,
        'operation_kind': 'transfer',
        'state': 'FAULT',
        'fault': 'trajectory controller aborted',
        'direct_transfer_motion_started': True,
    }
    captured = {}
    staging._begin_store_moveit_fallback = lambda reason: pytest.fail(reason)
    staging._fault = lambda reason: captured.update(reason=reason)

    staging._tick_store_transfer()

    assert 'automatic MoveIt fallback is unsafe' in captured['reason']


def test_successful_staging_loading_advances_to_safe_servo_place():
    staging = object.__new__(StagingSlots)
    staging.expected_store_transfer_operation_id = 21
    staging.pickup_status = {
        'operation_id': 21,
        'operation_kind': 'loading',
        'state': 'SUCCEEDED',
        'place_fallback_used': False,
    }
    captured = {'started': False}
    staging._begin_safe_servo_store = lambda: captured.update(started=True)
    staging._fault = lambda reason: pytest.fail(reason)

    staging._tick_store_loading()

    assert captured['started']
