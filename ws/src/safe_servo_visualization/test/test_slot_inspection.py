import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from safe_servo_visualization.slot_pose_estimation import (
    inspection_position, estimate_slot_top, stable_slot_pose, validate_slot_boundary)
from safe_servo_visualization.staging_slots_node import StagingSlots
from safe_servo_visualization.slot_inspection import configured_slot_inspection


@pytest.mark.parametrize('text, fallback, expected', [
    ('slot_inspection_enabled: false', True, False),
    ('slot_inspection_enabled: true', False, True),
    ('container_size: [450, 550, 450]', True, True),
    ('container_size: [450, 550, 450]', False, False),
])
def test_slot_inspection_yaml_overrides_legacy_flag(tmp_path, text, fallback, expected):
    config = tmp_path / 'policy.yaml'
    config.write_text(text, encoding='utf-8')
    assert configured_slot_inspection(str(config), fallback) is expected


@pytest.mark.parametrize('text', ['slot_inspection_enabled: "false"',
                                  'slot_inspection_enabled: 0',
                                  'slot_inspection_enabled: null', '[]', ''])
def test_invalid_inspection_config_is_not_silently_accepted(tmp_path, text):
    config = tmp_path / 'policy.yaml'
    config.write_text(text, encoding='utf-8')
    with pytest.raises(ValueError):
        configured_slot_inspection(str(config), True)


def test_standalone_inspection_config_and_missing_file(tmp_path):
    assert configured_slot_inspection('', False) is False
    assert configured_slot_inspection('', True) is True
    with pytest.raises(FileNotFoundError):
        configured_slot_inspection(str(tmp_path / 'missing.yaml'), True)


@pytest.mark.parametrize('observation_z', [.6, .4249, .18])
def test_inspection_requests_serialize_real_joint_state_from_tuple_seed(observation_z):
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import GetPositionFK, GetCartesianPath
    from rclpy.serialization import serialize_message, deserialize_message
    from sensor_msgs.msg import JointState

    node = object.__new__(StagingSlots)
    node.arm_joint_names = tuple(f'joint{i}' for i in range(1, 7))
    positions = (.1, -.2, .3, -.4, .5, -.6)
    node._fresh_joint_seed = lambda: positions
    node.base_frame, node.ik_link_name, node.planning_group = 'link_base', 'link_tcp', 'uf850'
    node.inspection_generation = 1
    node.inspection_yaw_flip = False
    node.inspection_camera = 'camera_color_optical_frame'
    node.inspection_observation_z, node.inspection_backoff = observation_z, .1
    node.inspection_observation_xyz = (.5, 0., .6)
    node.clearance = .03
    node.transfer_item_bottom_above_pallet = .47
    node._pallet_origin_z = lambda: 0.
    node.active_slot = 0
    node.occupied = {0: {'release_tcp_pose': (300., 200., 160., math.pi, 0., 0.),
                        'size': (.15, .1, .16), 'object_yaw': 0.}}
    node.tf_buffer = NS(lookup_transform=lambda *args: NS(
        transform=NS(translation=NS(x=.07, y=0., z=.08))))
    node.get_logger = lambda: NS(info=lambda msg: None)
    faults = []
    node._fault = faults.append
    requests, callbacks = [], []

    def call(request):
        # Exercise generated ROS conversion code, not a mocked service schema.
        restored = deserialize_message(serialize_message(request), type(request))
        requests.append(restored)
        return NS(add_done_callback=callbacks.append)

    node.inspection_fk = NS(call_async=call)
    node.inspection_cartesian = NS(call_async=call, service_is_ready=lambda: True)
    node._build_inspection_candidate()
    if observation_z == .18:
        assert len(faults) == 1 and 'below slot-item clearance' in faults[0]
        assert not requests
        return
    assert not faults
    endpoint_z = observation_z
    assert node.inspection_target[2] == pytest.approx(endpoint_z)
    assert node.inspection_target_message.data[12] == pytest.approx(max(.47, endpoint_z + .02))
    assert node.inspection_target[:2] == pytest.approx((.2, .2))
    assert isinstance(node.inspection_seed, JointState)
    assert isinstance(requests[0], GetPositionFK.Request)
    assert requests[0].robot_state.joint_state.name == list(node.arm_joint_names)
    assert list(requests[0].robot_state.joint_state.position) == list(positions)

    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = .3, .2, .6
    pose.pose.orientation.x = 1.
    callbacks[0](NS(result=lambda: NS(error_code=NS(val=1), pose_stamped=[pose])))
    assert isinstance(requests[1], GetCartesianPath.Request)
    assert requests[1].start_state.joint_state.name == list(node.arm_joint_names)
    assert list(requests[1].start_state.joint_state.position) == list(positions)
    assert requests[1].avoid_collisions and requests[1].waypoints
    assert requests[1].waypoints[-1].position.z == pytest.approx(endpoint_z)
    assert max(p.position.z for p in requests[1].waypoints) >= .47


@pytest.mark.parametrize('positions', [(0.,)*5, (float('nan'),)*6, (float('inf'),)*6])
def test_inspection_rejects_invalid_seed_before_ros_serialization(positions):
    node = object.__new__(StagingSlots)
    node.arm_joint_names = tuple(f'joint{i}' for i in range(6))
    node._fresh_joint_seed = lambda: positions
    with pytest.raises(ValueError, match='invalid inspection joint seed'):
        node._inspection_joint_state()


def test_inspection_backoff_uses_mounting_xy_and_keeps_observation_z():
    assert inspection_position((.1, .2, .3), .6, (.03, .04, .9), .1) == pytest.approx((.04, .12, .6))
    assert inspection_position((.1, .2, .3), .6, (0., -.07, -.9), .1) == pytest.approx((.1, .3, .6))
    for invalid in (-.1, float('nan'), .5):
        with pytest.raises(ValueError):
            inspection_position((0, 0, 0), .6, (1, 0, 0), invalid)


def test_inspection_mounting_offset_rotates_with_retrieval_orientation():
    node = object.__new__(StagingSlots)
    q = node._quaternion_from_rpy(0., 0., math.pi/2)
    offset = node._quat_rotate((.07, 0., .08), q)
    assert inspection_position((.1, .2, .3), .6, offset, .1) == pytest.approx((.1, .1, .6))


def test_inspection_rejects_undefined_xy_direction():
    for offset in ((0, 0, 1), (float('nan'), 0, 0)):
        with pytest.raises(ValueError):
            inspection_position((0, 0, 0), .6, offset, .1)
    assert inspection_position((.1, .2, .3), .6, (0, 0, 1), 0) == pytest.approx((.1, .2, .6))


def cloud(yaw=.08, size=(.15, .1), center=(.125, .125), height=.17):
    x, y = np.meshgrid(np.linspace(-size[0]/2, size[0]/2, 45),
                       np.linspace(-size[1]/2, size[1]/2, 35))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((center[0]+c*x.ravel()-s*y.ravel(),
                            center[1]+s*x.ravel()+c*y.ravel(),
                            np.full(x.size, height)))


def test_slot_estimate_retains_dimension_axis_and_small_yaw_shift():
    points = np.vstack((cloud(), cloud(center=(.5, .5)), cloud(height=0)))
    assert estimate_slot_top(points, (0, 0, 0), .25, (.15, .1, .17), 0) == pytest.approx(
        (.125, .125, .17, .08), abs=1e-5)


@pytest.mark.parametrize('points', [cloud(size=(.07, .05)), cloud(height=.3),
                                    cloud(yaw=.7), cloud()[:10]])
def test_wrong_or_incomplete_slot_top_is_rejected(points):
    with pytest.raises(ValueError):
        estimate_slot_top(points, (0, 0, 0), .25, (.15, .1, .17), 0)


def test_stability_requires_twenty_consistent_frames():
    samples = [[.1, .2, .17, .08]]*20
    assert stable_slot_pose(samples[:19]) is None
    assert stable_slot_pose(samples) == pytest.approx(samples[0])
    assert stable_slot_pose(samples[:19]+[[.12, .2, .17, .08]]) is None
    assert stable_slot_pose(samples[:19]+[[.1, .2, .17, .2]]) is None


@pytest.mark.parametrize('points', [cloud(size=(.11, .075)), cloud(yaw=.5)])
def test_stability_mode_does_not_reject_dimension_or_prior_yaw_difference(points):
    pose = estimate_slot_top(points, (0, 0, 0), .25, (.15, .1, .17), 0,
                             check_boundary=False, require_footprint_match=False)
    assert stable_slot_pose([pose]*20) == pytest.approx(pose)


def test_rolling_window_recovers_after_unstable_frames_age_out():
    samples = [[.09 if i % 2 else .12, .125, .17, 0.] for i in range(20)]
    assert stable_slot_pose(samples) is None
    samples.extend([[.125, .125, .17, 0.]]*20)
    assert stable_slot_pose(samples) == pytest.approx([.125, .125, .17, 0.])


def test_boundary_checked_on_stable_median_not_each_sample():
    samples = [[.071, .125, .17, 0.]]*19 + [[.069, .125, .17, 0.]]
    with pytest.raises(ValueError, match='outside'):
        validate_slot_boundary(samples[-1], (0, 0, 0), .25, (.15, .1, .17))
    stable = stable_slot_pose(samples)
    assert stable is not None
    validate_slot_boundary(stable, (0, 0, 0), .25, (.15, .1, .17))


def test_stable_inspection_accepts_overhanging_box_but_not_large_shift():
    node = object.__new__(StagingSlots)
    calls = []
    node._apply_slot_estimate = lambda pose: calls.append('apply')
    node._begin_retrieval_removal = lambda: calls.append('retrieve')
    node._fault = lambda reason: pytest.fail(reason)
    pose = np.array([.069, .125, .17, 0.])
    with pytest.raises(ValueError, match='outside'):
        validate_slot_boundary(pose, (0, 0, 0), .25, (.15, .1, .17))
    node._accept_stable_inspection(pose, np.array([.075, .125, .085]))
    assert calls == ['apply', 'retrieve']
    calls.clear()
    node._accept_stable_inspection(pose, np.array([.15, .125, .085]))
    assert not calls
    assert '40 mm' in node.inspection_reason


def test_correction_preserves_grasp_transform_and_dimensions():
    node = object.__new__(StagingSlots)
    record = {'size': (.15, .1, .17), 'center': (.01, -.02, .08),
              'orientation': (1., 0., 0., 0.), 'placed_obstacle_id': 'placed_item_42'}
    node.occupied = {2: record}
    node.active_slot = 2
    node.get_logger = lambda: NS(info=lambda msg: None)
    node._apply_slot_estimate(np.array([.12, .24, .17, .1]))
    pose = record['release_tcp_pose']
    q = node._quaternion_from_rpy(*pose[3:])
    center = np.array(pose[:3])/1000 + node._quat_rotate(record['center'], q)
    assert center == pytest.approx((.12, .24, .085))
    assert node._quat_multiply(q, record['orientation']) == pytest.approx(
        record['object_orientation_xyzw'])
    assert record['size'] == (.15, .1, .17)
    assert record['placed_obstacle_id'] == 'placed_item_42'
    assert node._retrieval_contact_reference_z(record) == pytest.approx(pose[2]/1000)


def test_stale_fk_callback_cannot_restart_aborted_inspection():
    node = object.__new__(StagingSlots)
    node.state = node.FAULT
    node.inspection_generation = 2
    node._inspection_fk_done(None, 2)
    node.state = node.INSPECTION_FK
    node._inspection_fk_done(None, 1)


def test_inspection_preflight_failure_tries_only_one_yaw_flip():
    node = object.__new__(StagingSlots)
    node.state = node.INSPECTION_CHECK
    node.inspection_generation = 1
    node.inspection_yaw_flip = False
    calls = []
    node._build_inspection_candidate = lambda: calls.append('yaw180')
    node.get_logger = lambda: NS(warning=lambda msg: None)
    node._fault = calls.append
    failed = NS(result=lambda: NS(error_code=NS(val=-31)))
    node._inspection_checked(failed, 1)
    assert calls == ['yaw180']
    node._inspection_checked(failed, 1)
    assert len(calls) == 2 and 'infeasible' in calls[-1]


def test_flipped_grasp_preserves_world_object_and_contact_point():
    node = object.__new__(StagingSlots)
    node.inspection_yaw_flip = True
    node.active_slot = 0
    node.get_logger = lambda: NS(info=lambda msg: None)
    original = (100., 200., 160., math.pi, 0., 0.)
    node.occupied = {0: {'release_tcp_pose': original, 'size': (.15, .1, .16),
                        'center': (.01, -.02, .08), 'orientation': (1., 0., 0., 0.)}}
    node._apply_slot_estimate(np.array([.1, .2, .16, 0.]))
    record = node.occupied[0]
    pose = record['release_tcp_pose']
    q = node._quaternion_from_rpy(*pose[3:])
    world_center = np.array(pose[:3])/1000 + node._quat_rotate(record['center'], q)
    assert world_center == pytest.approx((.1, .2, .08))
    object_q = node._quat_multiply(q, record['orientation'])
    assert abs(object_q[3]) == pytest.approx(1.)
    assert np.array(pose[:3])/1000 == pytest.approx((.09, .18, .16))
    assert record['stored_release_tcp_pose'] == original


@pytest.mark.parametrize('current, values, accepted', [
    (.2, [.2, .4, .6], True),
    (-.2, [-.2, -.4, -.6], True),
    (.6, [.6, .4, .2], True),
    (.2, [.2, .8, .6], False),
    (.2, [.2, float('nan'), .6], False),
    (float('nan'), [.2, .4, .6], False),
])
def test_yaw_flip_accepts_away_from_zero_but_retains_excursion_check(current, values, accepted):
    node = object.__new__(StagingSlots)
    node.state = node.INSPECTION_CHECK
    node.inspection_generation = 1
    node.inspection_yaw_flip = True
    node.arm_joint_names = ['joint1', 'joint6']
    node.inspection_seed = NS(name=['joint1', 'joint6'], position=[0., current])
    node.inspection_target_message = NS(data=[0.]*14)
    node.motion_status = {'operation_id': 7}
    published, retries, warnings = [], [], []
    node.retrieve_target_pub = NS(publish=published.append)
    node._retry_inspection_candidate = retries.append
    node.get_logger = lambda: NS(warning=warnings.append)
    node._fault = lambda message: pytest.fail(message)
    node.pick_path = NS(reset=lambda _: None)
    node.create_timer = lambda *args: None
    path = NS(joint_names=['joint6', 'joint1'],
              points=[NS(positions=[v, 0.]) for v in values])
    result = NS(error_code=NS(val=1), fraction=1., solution=NS(joint_trajectory=path))
    node._inspection_checked(NS(result=lambda: result), 1)
    assert bool(published) == accepted
    assert retries == ([] if accepted else [1])
    if accepted:
        assert node.state == node.INSPECTION_MOVE
        assert len(published[0].data) == 17  # preserve route handoff
    else:
        assert node.state == node.INSPECTION_CHECK
        assert all(s in warnings[0] for s in ('start=', 'end=', 'peak_abs=', 'rad'))


def test_inspection_timeout_warns_and_waits_without_retrieval_or_grasp():
    node = object.__new__(StagingSlots)
    node.state = node.INSPECTION_DEPTH
    node.inspection_started = 0.
    node.inspection_timeout = 15.
    node.inspection_reason = 'slot not visible'
    faults = []
    node._fault = faults.append
    warnings = []
    node.get_logger = lambda: NS(warning=warnings.append)
    node._inspection_tick()
    assert not faults
    assert warnings and 'slot not visible' in warnings[0]
    assert node.state == node.INSPECTION_DEPTH
    assert node.phase_started > 0
    node._inspection_tick()
    assert len(warnings) == 1


@pytest.mark.parametrize('inspection_enabled', [True, False])
def test_retrieval_inspects_before_removing_target_obstacle(inspection_enabled):
    node = object.__new__(StagingSlots)
    node._busy_reason = lambda **kwargs: ''
    node.scene_status = {}
    node.selected_retrieve_slot = 1
    node.occupied = {1: {'placed_obstacle_id': 'placed_item_7'}}
    node.remove_staged_obstacle = NS(service_is_ready=lambda: True)
    node.inspection_enabled = inspection_enabled
    calls = []
    node._begin_slot_inspection = lambda: calls.append('inspect')
    node._begin_retrieval_removal = lambda: calls.append('remove')
    node.publish_status = lambda: None
    result = node._retrieve(NS(success=False, message=''), False)
    assert result.success
    assert calls == (['inspect'] if inspection_enabled else ['remove'])
    assert not node.occupied[1].get('obstacle_removed')


def test_twenty_fresh_depth_frames_correct_target_before_removal():
    from sensor_msgs.msg import Image
    node = object.__new__(StagingSlots)
    node.state = node.INSPECTION_DEPTH
    node.inspection_k = np.array([[400., 0, 320], [0, 400., 240], [0, 0, 1.]])
    node.inspection_last_stamp = 0.
    node.inspection_camera = 'camera_color_optical_frame'
    node.base_frame, node.ik_link_name = 'link_base', 'link_tcp'
    node.inspection_target = (.125, .125, .7, math.pi, 0, 0)
    node.inspection_q = (1., 0, 0, 0)
    node.inspection_samples = []
    node.active_slot = 0
    node.slots, node.slot_size = [(0., 0., 0.)], .25
    node.occupied = {0: {'size': (.15, .1, .17), 'center': (0., 0., .08),
        'orientation': (1., 0, 0, 0), 'object_yaw': 0.,
        'release_tcp_pose': (125., 125., 165., math.pi, 0., 0.),
        'placed_obstacle_id': 'placed_item_9'}}
    transform = NS(translation=NS(x=.125, y=.125, z=.7),
                   rotation=NS(x=1., y=0., z=0., w=0.))
    node.tf_buffer = NS(lookup_transform=lambda *args: NS(transform=transform))
    node.get_logger = lambda: NS(info=lambda msg: None)
    calls = []
    node._begin_retrieval_removal = lambda: calls.append('remove')
    node._fault = lambda reason: pytest.fail(reason)
    image = Image(height=480, width=640, encoding='16UC1', step=1280)
    image.header.frame_id = node.inspection_camera
    u, v = np.meshgrid(np.arange(640), np.arange(480))
    depth = np.where((abs((u-320)*.53/400) <= .075) &
                     (abs((v-240)*.53/400) <= .05), 530, 0).astype('<u2')
    image.data = depth.tobytes()
    for i in range(1, 21):
        image.header.stamp.sec = i
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int((i+.01)*1e9)))
        node._inspection_depth(image)
        if i < 20:
            assert not calls
        # Duplicate timestamps must not count toward the stable window.
        node._inspection_depth(image)
        assert len(node.inspection_samples) == i
    assert calls == ['remove']
    assert node.occupied[0]['inspection_top_pose_m_rad'][:3] == pytest.approx((.125, .125, .17), abs=.002)


@pytest.mark.parametrize('cx, visible', [(10.5, True), (9.5, False), (14.5, False)])
def test_inspection_visibility_uses_real_corners_without_extra_margin(monkeypatch, cx, visible):
    from sensor_msgs.msg import Image
    node = object.__new__(StagingSlots)
    node.state = node.INSPECTION_DEPTH
    node.inspection_k = np.array([[100., 0., cx], [0., 100., 12.], [0., 0., 1.]])
    node.inspection_last_stamp = 0.
    node.inspection_camera = 'camera_color_optical_frame'
    node.base_frame, node.ik_link_name = 'link_base', 'link_tcp'
    node.inspection_target = (0., 0., 0., 0., 0., 0.)
    node.inspection_q = (0., 0., 0., 1.)
    node.inspection_samples = []
    node.active_slot = 0
    node.slots, node.slot_size = [(0., 0., 0.)], .25
    node.occupied = {0: {'size': (.1, .1, .1), 'center': (0., 0., 0.),
                         'object_yaw': 0., 'release_tcp_pose': (0., 0., 450., 0., 0., 0.)}}
    transform = NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.))
    node.tf_buffer = NS(lookup_transform=lambda *args: NS(transform=transform))
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1010000000))
    calls = []

    def estimate(*args, **kwargs):
        calls.append('estimate')
        return (0., 0., .5, 0.)

    monkeypatch.setattr('safe_servo_visualization.slot_inspection.estimate_slot_top', estimate)
    image = Image(height=24, width=24, encoding='16UC1', step=48)
    image.header.frame_id = node.inspection_camera
    image.header.stamp.sec = 1
    image.data = np.full((24, 24), 500, dtype='<u2').tobytes()
    # True corners project at cx +/- 10 px. At cx=10.5 they are visible,
    # although either the old 15 mm padding or 3 px border would reject them.
    node._inspection_depth(image)
    assert bool(calls) == visible
    assert len(node.inspection_samples) == int(visible)
    if not visible:
        assert 'not fully visible' in node.inspection_reason
