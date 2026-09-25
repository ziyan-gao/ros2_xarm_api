"""Delayed status delivery must not cause a premature transport request."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from safe_servo_visualization.continuous_transport import ContinuousTransport
from safe_servo_visualization.place_pipeline_node import PlacePipeline


def supervisor():
    return NS(pallet_locked=True, planning_scene_status={'attached_item_id': 'box_1'},
              motion_status={'operation_id': 12, 'state': 'PREPARED',
                             'target': 'transfer', 'transfer_context': 'pallet'})


def pipeline():
    h = NS(pending_motion='transfer_preparing', expected_motion_operation_id=12,
           motion_status=supervisor().motion_status.copy(), continuous_transport=True,
           return_to_observation=False, scene_status={'attached_item_id': 'box_1'},
           supervisor_status={'operation_id': 5}, state='MOVE_TRANSFER',
           LOAD_PRE_PLACE='LOAD_PRE_PLACE', _fault=Mock(), _wait_for_data=Mock(), get_logger=Mock(), _start_loading_completed=Mock())
    h.start_transport_chained = Mock()
    h.start_transport_chained.service_is_ready.return_value = True
    return h


@pytest.mark.parametrize('field,value', [('pallet_locked', False),
    ('planning_scene_status', {}), ('motion_status', {'state': 'SUCCEEDED'}),
    ('motion_status', {'operation_id': 12, 'state': 'PREPARED', 'transfer_context': 'known_pick'})])
def test_no_acknowledgement_until_all_preconditions_match(field, value):
    h = supervisor()
    setattr(h, field, value)
    assert ContinuousTransport._prepared_transport_target(h) is None


@pytest.mark.parametrize('stale', [None,
    {'operation_id': 11, 'attached_item_id': 'box_1', 'transfer_context': 'pallet'},
    {'operation_id': 12, 'attached_item_id': 'old_box', 'transfer_context': 'pallet'},
    {'operation_id': 12, 'attached_item_id': 'box_1', 'transfer_context': 'staging_store'}])
def test_waits_for_matching_ack_then_sends_one_request(stale):
    h = pipeline()
    h.supervisor_status['prepared_transport'] = stale
    PlacePipeline._tick_transfer(h)
    h.start_transport_chained.call_async.assert_not_called()
    assert h.state == 'MOVE_TRANSFER'
    h.supervisor_status['prepared_transport'] = ContinuousTransport._prepared_transport_target(supervisor())
    PlacePipeline._tick_transfer(h)
    h.start_transport_chained.call_async.assert_called_once()
    assert h.state == h.LOAD_PRE_PLACE
    assert h.expected_supervisor_operation_id == 6
    h._fault.assert_not_called()


def test_missing_ack_keeps_waiting_without_motion(monkeypatch):
    h = pipeline()
    clock = [10.]
    monkeypatch.setattr('safe_servo_visualization.place_pipeline_node.time.monotonic', lambda: clock[0])
    PlacePipeline._tick_transfer(h)
    clock[0] = 13.1
    PlacePipeline._tick_transfer(h)
    h._fault.assert_not_called()
    assert h._wait_for_data.call_count == 2
    h.start_transport_chained.call_async.assert_not_called()


def test_missing_attachment_never_starts_even_with_an_old_ack():
    h = pipeline()
    h.supervisor_status['prepared_transport'] = ContinuousTransport._prepared_transport_target(supervisor())
    h.scene_status = {}
    PlacePipeline._tick_transfer(h)
    h.start_transport_chained.call_async.assert_not_called()


def test_slot_store_waits_for_the_same_target_ack():
    from safe_servo_visualization.staging_slots_node import StagingSlots
    h = pipeline()
    h.motion_status['transfer_context'] = 'staging_store'
    h.pickup_status = h.supervisor_status
    h.TRANSFERRING_STORE = 'TRANSFERRING_STORE'
    h._request_accepted = Mock()
    StagingSlots._begin_store_continuous_transfer(h)
    h.start_transport_chained.call_async.assert_not_called()
    peer = supervisor()
    peer.motion_status['transfer_context'] = 'staging_store'
    h.pickup_status['prepared_transport'] = ContinuousTransport._prepared_transport_target(peer)
    StagingSlots._begin_store_continuous_transfer(h)
    h.start_transport_chained.call_async.assert_called_once()
    assert h.state == 'TRANSFERRING_STORE'
    h._fault.assert_not_called()
