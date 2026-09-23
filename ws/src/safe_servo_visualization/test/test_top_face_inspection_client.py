import json
import time
from types import SimpleNamespace as NS
import pytest
from std_msgs.msg import String
from safe_servo_visualization.top_face_inspection_client import (
    TopFaceInspectionClient, checked_result, inspection_enabled, marker_target)
from safe_servo_visualization.top_face_automation import TopFaceAutomation


def client():
    sent = []
    node = NS(create_publisher=lambda *a: NS(publish=sent.append),
              create_subscription=lambda *a: None)
    return TopFaceInspectionClient(node, 'policy'), sent


def test_config_and_marker_identity():
    assert not inspection_enabled({})
    assert inspection_enabled({'top_face_inspection_enabled': True})
    with pytest.raises(ValueError):
        inspection_enabled({'top_face_inspection_enabled': 'false'})
    assert marker_target({'placed_marker_object_ids': {'8': 'placed_item_3'}}, 'placed_item_3') == 'placed:placed_item_visuals:8'
    with pytest.raises(ValueError):
        marker_target({}, 'placed_item_3')


@pytest.mark.parametrize('enabled', [True, False])
def test_panel_inspects_before_removing_source(enabled):
    from safe_servo_visualization.pick_place_test_node import PickPlaceTest
    calls = []
    node = NS(status={'staging': {'top_face_inspection_enabled': enabled},
                      'scene': {'placed_marker_object_ids': {'8': 'placed_item_3'}}},
              record={'obstacle_id': 'placed_item_3'}, sam_pick_result={'old': True},
              sam_inspector=NS(begin=lambda key: calls.append(('inspect', key))),
              phase=lambda state: calls.append(('phase', state)),
              remove_pallet_source=lambda: calls.append(('remove',)))
    PickPlaceTest.begin_pallet_pick(node)
    assert node.sam_pick_result is None
    if enabled:
        assert calls == [('inspect', 'placed:placed_item_visuals:8'), ('phase', 'SAM_INSPECTION')]
    else:
        assert calls == [('remove',)]


def test_test_panel_owner_allowed_only_with_matching_token():
    obj = NS(inspection=dict(owner='test', request_id='a'),
             motion_status={'test': dict(state='SAM_INSPECTION', sam_request_id='a')},
             motion_seen={'test': time.monotonic()})
    assert TopFaceAutomation._inspection_owners(obj) == {'test'}
    obj.motion_status['test']['state'] = 'FAULT'
    with pytest.raises(ValueError):
        TopFaceAutomation._inspection_owners(obj)


def test_token_binding_timeout_and_cancel():
    c, sent = client()
    c.begin('target')
    c._result(String(data=json.dumps(dict(request_id='wrong', owner='policy', target='target', success=True))))
    assert c.result is None
    c.started -= 1
    c.tick()
    assert json.loads(sent[-1].data)['action'] == 'inspect'
    c.started -= 181
    with pytest.raises(ValueError, match='timed out'):
        c.tick()
    assert c.token is None
    assert json.loads(sent[-1].data)['action'] == 'cancel'


def test_success_and_invalid_geometry():
    c, _ = client()
    c.begin('target')
    result = dict(request_id=c.token, owner='policy', target='target', success=True,
                  top_center_base_m=[0., 0., .2], grasp_rpy_rad=[0., 0., 0.],
                  size_m=[.1, .2, .3], yaw_rad=0.)
    c._result(String(data=json.dumps(result)))
    assert c.tick() == result
    assert c.token is None
    result['top_center_base_m'][0] = float('nan')
    with pytest.raises(ValueError):
        checked_result(result)


def test_owner_requires_active_matching_token():
    obj = NS(inspection=dict(owner='policy', request_id='a'),
             motion_status={'policy': dict(state='REARRANGE_INSPECT', sam_request_id='a')},
             motion_seen={'policy': time.monotonic()})
    assert TopFaceAutomation._inspection_owners(obj) == {'policy'}
    obj.motion_status['policy']['sam_request_id'] = 'b'
    with pytest.raises(ValueError):
        TopFaceAutomation._inspection_owners(obj)


def test_policy_pick_uses_refined_pose_not_observation_yaw():
    from safe_servo_visualization.policy_loading_node import PolicyLoadingNode
    result = dict(top_center_base_m=[.1, .2, .3], grasp_rpy_rad=[3.14, 0., .7],
                  size_m=[.1, .2, .15], yaw_rad=.4)
    node = NS(sam_inspection_enabled=True, sam_pallet_result=(7, result),
              pallet_unpack_approach_height=.5,
              tf_buffer=NS(lookup_transform=lambda *a: NS(transform=NS(translation=NS(z=0.)))))
    msg = PolicyLoadingNode._pallet_item_retrieval_message(node, NS(sequence_id=7))
    assert list(msg.data[1:7]) == [.1, .2, .3, 3.14, 0., .7]
    with pytest.raises(ValueError, match='required'):
        PolicyLoadingNode._pallet_item_retrieval_message(node, NS(sequence_id=8))


def test_slot_target_uses_sam_without_overwriting_inventory():
    from safe_servo_visualization.staging_slots_node import StagingSlots
    result = dict(top_center_base_m=[.1, .2, .3], grasp_rpy_rad=[3.14, 0., .7],
                  size_m=[.1, .2, .15], yaw_rad=.4)
    record = dict(release_tcp_pose=[0., 0., 100., 0., 0., 0.], size=[.1, .2, .15])
    sent = []
    node = NS(occupied={0: record}, active_slot=0, sam_inspection_enabled=True,
              sam_retrieval_result=result, clearance=.03, transfer_item_bottom_above_pallet=.5,
              _retrieval_contact_reference_z=StagingSlots._retrieval_contact_reference_z,
              _pallet_origin_z=lambda: 0., retrieve_target_pub=NS(publish=sent.append),
              PLANNING_RETRIEVAL_APPROACH='PLAN', motion_status={},
              pick_path=NS(reset=lambda *a: None), create_timer=lambda *a: None,
              _request_retrieval_plan=lambda: None)
    StagingSlots._publish_retrieval_target(node)
    assert list(sent[0].data[1:7]) == [.1, .2, .3, 3.14, 0., .7]
    assert record['release_tcp_pose'][2] == 100.


@pytest.mark.parametrize('op_id,state,info,expected', [
    (6, 'SUCCEEDED', False, False),
    (7, 'DIRECT_RETREAT', False, False),
    (7, 'SUCCEEDED', False, True),
    (7, 'SUCCEEDED', True, False),
    (8, 'SUCCEEDED', False, False),
])
def test_inspection_waits_for_own_completed_contact_retreat(op_id, state, info, expected):
    moved, faults = [], []
    node = NS(inspection={'target': 'box'}, inspection_started=time.monotonic(),
              inspection_step='contact_retreat', state='CONTACT_RETREAT',
              _inspection_owners=lambda: {'policy'}, inspection_retreat_id=7,
              motion_status={'pickup': dict(operation_id=op_id, state=state, object_info_obtained=info)},
              motion_seen={'pickup': time.monotonic()},
              inspection_retreat_future=NS(done=lambda: True, result=lambda: NS(success=True)),
              _begin_motion=lambda *args: moved.append(args),
              _motion_fault=faults.append, generation=0,
              _inspection_reply=lambda *args, **kwargs: None)
    TopFaceAutomation._automation_tick(node)
    assert bool(moved) is expected
    if info or op_id > 7:
        assert faults
