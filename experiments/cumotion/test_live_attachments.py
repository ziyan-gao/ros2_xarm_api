import copy
import xml.etree.ElementTree as ET
from types import SimpleNamespace
import numpy as np
import pytest
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from live_attachments import (
    DYNAMIC_PAYLOAD_LINK, DYNAMIC_PAYLOAD_SPHERES,
    DYNAMIC_PAYLOAD_NEAR_MOUNT_LINKS, DYNAMIC_PAYLOAD_TOUCH_LINKS, bake,
    dynamic_payload_spheres, reserve_dynamic_payload)
from live_bridge import LiveServer


def model(tmp_path):
    path = tmp_path/'base.urdf'
    path.write_text(
        '<robot name="test"><link name="link_eef"/><link name="link_tcp"/></robot>')
    return {'robot_cfg': {'kinematics': {'urdf_path': str(path),
        'collision_spheres': {}, 'collision_link_names': [],
        'link_names': ['link_eef', 'link_tcp'],
        'self_collision_buffer': {}, 'self_collision_ignore': {}}}}


def camera():
    a = AttachedCollisionObject()
    a.link_name = a.object.header.frame_id = 'link_eef'
    a.object.id = 'eef_camera_d435i'
    a.object.pose.orientation.w = 1.
    a.object.pose.position.x = .065
    a.touch_links = ['link_eef']
    a.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.02505, .09, .025])]
    p = Pose()
    p.orientation.w = 1.
    a.object.primitive_poses = [p]
    return a


def payload():
    a = camera()
    a.link_name = a.object.header.frame_id = DYNAMIC_PAYLOAD_LINK
    a.object.id = 'carried_item_7'
    a.object.pose.position.x = 0.
    a.touch_links = [DYNAMIC_PAYLOAD_LINK, 'link_eef', 'ft_sensor_link',
                     'xarm_vacuum_gripper_link']
    a.object.primitives[0].dimensions = [.2, .1, .16]
    a.object.primitive_poses[0].position.z = -.08
    return a


def pallet_surface():
    obj = CollisionObject()
    obj.header.frame_id = 'link_base'
    obj.id = 'pallet_surface'
    primitive = SolidPrimitive(type=SolidPrimitive.BOX,
                               dimensions=[.45, .55, .005])
    pose = Pose()
    pose.position.x = .0721
    pose.position.y = -.4724
    pose.position.z = -.184
    pose.orientation.z = np.sin(.1)
    pose.orientation.w = np.cos(.1)
    obj.primitives = [primitive]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


def barrier_geometry():
    return {
        'pallet_clearance_z_m': .47,
        'slot_clearance_z_m': .48,
        'slot_surface_z_m': 0.,
        'slot_x_min_m': -.375,
        'slot_x_max_m': .375,
        'slot_y_min_m': .18,
        'slot_y_max_m': .68,
        'barrier_top_margin_m': .003,
    }


def test_attachment_is_separate_moving_link(tmp_path):
    original = model(tmp_path)
    saved = copy.deepcopy(original)
    result = bake(original, [camera()], tmp_path)['robot_cfg']['kinematics']
    root = ET.parse(result['urdf_path']).getroot()
    assert original == saved
    assert root.find('joint/parent').get('link') == 'link_eef'
    assert result['self_collision_ignore']['cumotion_attachment_0'] == [
        'link_eef', 'link_tcp']
    assert result['self_collision_ignore'][DYNAMIC_PAYLOAD_LINK] == [
        'cumotion_attachment_0']
    balls = result['collision_spheres']['cumotion_attachment_0']
    assert 1 <= len(balls) <= 24
    assert all(np.isfinite(b['center']).all() and np.isfinite(b['radius'])
               and b['radius'] > 0. for b in balls)
    assert all(np.all(np.abs(b['center']) <= np.array([.02505, .09, .025])/2 + .002)
               for b in balls)


@pytest.mark.parametrize('case', ['frame', 'size', 'type', 'quat', 'touch'])
def test_invalid_attachment_rejected(tmp_path, case):
    a = camera()
    if case == 'frame': a.object.header.frame_id = 'world'
    if case == 'size': a.object.primitives[0].dimensions[0] = -1.
    if case == 'type': a.object.primitives[0].type = SolidPrimitive.SPHERE
    if case == 'quat': a.object.pose.orientation.w = 0.
    if case == 'touch': a.touch_links = ['missing']
    with pytest.raises(ValueError): bake(model(tmp_path), [a], tmp_path)


def test_signature_ignores_non_collision_metadata():
    first, second = camera(), camera()
    first.object.header.stamp.sec = 123
    first.object.operation = first.object.REMOVE
    first.weight = 99.
    first.detach_posture.header.frame_id = 'ignored'
    assert LiveServer.attachment_signature([first]) == LiveServer.attachment_signature([second])


def test_signature_detects_geometry_change():
    first, second = camera(), camera()
    second.object.primitives[0].dimensions[1] += .001
    assert LiveServer.attachment_signature([first]) != LiveServer.attachment_signature([second])


def test_complete_scene_is_geometry_authority_for_same_mount():
    first, second = camera(), camera()
    second.object.primitives[0].dimensions[1] += .001
    assert LiveServer.attachment_identity([first]) == LiveServer.attachment_identity([second])
    second.link_name = 'other_link'
    assert LiveServer.attachment_identity([first]) != LiveServer.attachment_identity([second])


def test_dynamic_payload_reservation_and_fit(tmp_path):
    reserved = reserve_dynamic_payload(model(tmp_path))['robot_cfg']['kinematics']
    assert reserved['extra_collision_spheres'][DYNAMIC_PAYLOAD_LINK] == DYNAMIC_PAYLOAD_SPHERES
    assert DYNAMIC_PAYLOAD_LINK in reserved['collision_link_names']
    spheres = dynamic_payload_spheres(payload())
    assert spheres.shape == (DYNAMIC_PAYLOAD_SPHERES, 4)
    assert np.isfinite(spheres).all()
    assert 1 <= int(np.count_nonzero(spheres[:, 3] > 0)) <= DYNAMIC_PAYLOAD_SPHERES


def test_collision_sphere_visualization_uses_ros_visible_parent_frame(tmp_path):
    prepared = reserve_dynamic_payload(
        bake(model(tmp_path), [camera()], tmp_path))
    specs = LiveServer.collision_sphere_specs(prepared, [camera()])
    assert specs
    assert {value[0] for value in specs} == {'camera'}
    assert {value[1] for value in specs} == {'link_eef'}
    # The synthetic cuMotion camera link is mounted +65 mm in link_eef X.
    assert all(abs(value[2]-.065) <= .02505/2 + .002 for value in specs)
    assert all(value[-1] > 0. for value in specs)


def test_clearance_barriers_follow_pallet_and_slot_footprints():
    pallet = pallet_surface()
    barriers = LiveServer.build_clearance_barriers(
        [pallet], barrier_geometry())
    assert [obj.id for obj in barriers] == [
        LiveServer.PALLET_BARRIER_ID, LiveServer.SLOT_BARRIER_ID]
    pallet_barrier, slot_barrier = barriers
    pallet_top = (pallet.primitive_poses[0].position.z +
                  pallet.primitives[0].dimensions[2] / 2.)
    assert pallet_barrier.primitives[0].dimensions[:2] == pytest.approx([.45, .55])
    assert (pallet_barrier.primitive_poses[0].orientation ==
            pallet.primitive_poses[0].orientation)
    expected_pallet_top = pallet_top + .47 - .100 - .003
    assert (pallet_barrier.primitive_poses[0].position.z +
            pallet_barrier.primitives[0].dimensions[2] / 2.) == pytest.approx(
                expected_pallet_top)
    assert (pallet_barrier.primitive_poses[0].position.z -
            pallet_barrier.primitives[0].dimensions[2] / 2.) == pytest.approx(pallet_top)
    expected_slot_top = pallet_top + .48 - .003
    assert slot_barrier.primitives[0].dimensions == pytest.approx(
        [.75, .5, expected_slot_top])
    slot_pose = slot_barrier.primitive_poses[0]
    assert [slot_pose.position.x, slot_pose.position.y,
            slot_pose.position.z] == pytest.approx(
                [0., .43, expected_slot_top / 2.])


def test_object_level_pallet_pose_is_baked_for_scene_query():
    pallet = pallet_surface()
    primitive_pose = copy.deepcopy(pallet.primitive_poses[0])
    pallet.pose = primitive_pose
    pallet.primitive_poses[0] = Pose()
    pallet.primitive_poses[0].orientation.w = 1.
    baked = LiveServer.bake_world_object_pose(pallet)
    assert baked.header.frame_id == 'link_base'
    assert baked.pose.position.x == 0.
    assert baked.pose.orientation.w == 1.
    pose = baked.primitive_poses[0]
    assert [pose.position.x, pose.position.y, pose.position.z] == pytest.approx(
        [.0721, -.4724, -.184])
    assert [pose.orientation.z, pose.orientation.w] == pytest.approx(
        [np.sin(.1), np.cos(.1)])


def test_clearance_barriers_do_not_fake_missing_pallet_geometry():
    barriers = LiveServer.build_clearance_barriers([], barrier_geometry())
    assert barriers == []


def test_reserved_payload_preserves_camera_only_self_collision_exemption(tmp_path):
    kinematics = reserve_dynamic_payload(
        bake(model(tmp_path), [camera()], tmp_path))['robot_cfg']['kinematics']
    payload_ignored = set(
        kinematics['self_collision_ignore'][DYNAMIC_PAYLOAD_LINK])
    camera_ignored = set(
        kinematics['self_collision_ignore']['cumotion_attachment_0'])

    assert 'cumotion_attachment_0' in payload_ignored
    assert (DYNAMIC_PAYLOAD_TOUCH_LINKS-{DYNAMIC_PAYLOAD_LINK}).issubset(
        payload_ignored)
    assert {'link6', 'camera_stand_link'}.issubset(payload_ignored)
    assert payload_ignored == (
        DYNAMIC_PAYLOAD_NEAR_MOUNT_LINKS-{DYNAMIC_PAYLOAD_LINK} |
        {'cumotion_attachment_0'})
    assert DYNAMIC_PAYLOAD_LINK in camera_ignored
    # Robot links outside the physical grasp/touch assembly remain checked.
    assert 'link_base' not in payload_ignored


def test_payload_change_does_not_change_fixed_model_signature():
    first, second = payload(), payload()
    second.object.primitives[0].dimensions[0] += .01
    first_camera, first_payload = LiveServer.split_attachments([camera(), first])
    second_camera, second_payload = LiveServer.split_attachments([camera(), second])
    assert LiveServer.attachment_signature(first_camera) == LiveServer.attachment_signature(second_camera)
    assert LiveServer.attachment_signature(first_payload) != LiveServer.attachment_signature(second_payload)


def test_release_motion_gen_drops_both_cuda_model_references(monkeypatch):
    server = SimpleNamespace(motion_gen=object())
    setattr(server, '_CumotionActionServer__world_collision', object())
    calls = []
    monkeypatch.setattr('live_bridge.torch.cuda.is_available', lambda: True)
    monkeypatch.setattr('live_bridge.torch.cuda.synchronize',
                        lambda: calls.append('synchronize'))
    monkeypatch.setattr('live_bridge.torch.cuda.empty_cache',
                        lambda: calls.append('empty_cache'))
    monkeypatch.setattr('live_bridge.gc.collect', lambda: calls.append('gc'))

    LiveServer.release_motion_gen(server)

    assert server.motion_gen is None
    assert getattr(server, '_CumotionActionServer__world_collision') is None
    assert calls == ['synchronize', 'gc', 'empty_cache']
