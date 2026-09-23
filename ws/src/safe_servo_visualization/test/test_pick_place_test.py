"""Offline coordinator tests: no robot, services, or controller execution."""
from collections import deque
import math
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.client import Client as ROSClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool

from safe_servo_visualization.pick_place_test_node import (
    PickPlaceTest, pallet_record, retrieval_values, placed_ids,
)
from safe_servo_visualization.staging_slots_node import StagingSlots


class Future:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        callback(self)


class Client:
    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return Future(NS(success=True, ret=0))


TARGET = [1000001., 4., 20., 40., 0., 1., 110., 150., 160., 170., 130., 160., 10., 30., 0.]


def test_measured_slot_geometry_does_not_replace_planning_geometry():
    h = Harness()
    h.step, h.location, h.state = 'unpack', 'carried', 'STORE_SLOT_AND_HANDOFF'
    h.test_slot = 0
    h.record = dict(size_mm=[145, 150, 155], obstacle_id='placed_item_1')
    measured = [144.84838470816612, 149.3209682404995, 151.51385293280345]
    h.status['staging']['slots'] = [dict(slot=0, occupied=True,
        obstacle_id='placed_item_3', size_m=[v/1000 for v in measured])]
    h._record_slot_release()
    assert h.record['planning_size_mm'] == [145, 150, 155]
    assert h.record['size_mm'] == [145, 150, 155]
    assert h.record['real_size_mm'] == pytest.approx(measured)
    assert h.location == 'slot'


def test_fault_does_not_reenter_candidate_generation():
    h = Harness()
    h.state = 'FAULT'
    h.two_item_mode = True
    h._two_choices = Mock(side_effect=ValueError('must not be called'))
    assert h.random_choices() == {}
    h.publish_status()
    h._two_choices.assert_not_called()


def test_legacy_measured_dimensions_are_quantized_without_mutation():
    from safe_servo_visualization.two_item_test import planning_dimensions
    measured = [144.848, 149.321, 151.514]
    assert planning_dimensions(measured) == (140, 145, 155)
    assert measured == [144.848, 149.321, 151.514]


class Harness(PickPlaceTest):
    def __init__(self):
        self.status = {k: dict(state='IDLE', operation_id=0) for k in self.TOPICS}
        self.status['supervisor']['robot_error'] = 0
        self.status['supervisor']['pallet_release_rpy_rad'] = [math.pi, 0., -math.pi/2]
        self.status['scene'].update(attached_item_id='', placed_item_ids=[], placed_item_visual_ids=[])
        self.status['staging'].update(selected_store_slot=0, selected_retrieve_slot=0,
                                      slots=[dict(slot=0, occupied=False)])
        self.received = {k: time.monotonic() for k in self.TOPICS}
        self.pallet_locked, self.pallet_received = True, time.monotonic()
        self.state, self.location, self.step, self.fault = 'IDLE', 'empty', '', ''
        self.test_slot = 0
        self.random_active = self.random_step_pending = False
        self.random_pack_unpack_only = False
        self.random_max_steps, self.random_config_seed = 100, 123
        self.random_completed = 0
        self.random_counts = dict.fromkeys(('pack_new', 'unpack', 'pack_slot', 'repack'), 0)
        self.random_message, self.random_seed, self.random_next_at = '', None, 0.
        self.record = self.target = self.planning = None
        self.generation, self.sequence = 0, 1000000
        self.accepted = False
        self.future = None
        self.owner = self.expected = None
        self.phase_started = time.monotonic()
        self.events = deque(maxlen=16)
        self.height_grid, self.clearance = 5, .47
        self.container = (450, 550, 450)
        self.test_clients = {k: Client() for k in self.SERVICES}
        self.remove = Client()
        self.pick_path = NS(cancel=Mock(), reset=Mock(), prepare=Client(), tick=Mock(return_value=True))
        self.target_pub = self.source_pub = self.store_selection = self.retrieve_selection = Mock()
        self.loader = NS(pending=None, discard_pending=Mock(), reset=Mock(), plan=Mock())
        self.worker = NS(submit=Mock(return_value=Future(NS(target_values=TARGET))))
        self.tf_buffer = NS(lookup_transform=lambda *args: NS(transform=NS(
            translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.))))
        self.scene_before = set()
        self.slot_seen_active = False
        self.logger = Mock()

    def get_logger(self):
        return self.logger

    def publish_status(self):
        pass

    def advance(self, seconds=1.):
        self.phase_started -= seconds
        self.tick()

    def finish_owner(self, key, state='SUCCEEDED'):
        self.status[key].update(operation_id=self.expected, state=state)
        self.tick()

    def on_pallet(self):
        self.record = pallet_record(TARGET)
        self.record['release_tcp_rpy_rad'] = [math.pi, 0., -math.pi/2]
        self.record['obstacle_id'] = 'placed_item_1'
        self.location, self.state = 'pallet', 'READY'
        self.status['scene']['placed_item_visual_ids'] = ['placed_item_1']


def test_physical_geometry_rotated_dimensions_not_virtual_envelope():
    record = pallet_record(TARGET)
    record['release_tcp_rpy_rad'] = [math.pi, 0., -math.pi/2]
    assert record['corner_mm'] == [20, 40, 0]
    assert record['size_mm'] == [150, 110, 160]
    values = retrieval_values(record, 10, [0, 0, .1], [0, 0, 0, 1], .47)
    assert values[1:4] == pytest.approx([.095, .095, .260])
    assert values[7:10] == pytest.approx([.15, .11, .16])
    assert values[10] == .03
    assert values[12] == pytest.approx(.57)
    assert values[4:7] == pytest.approx([math.pi, 0., -math.pi/2])
    assert values[11] == 0.  # Object footprint axes, not tool yaw.


def test_missing_release_orientation_blocks_pick_before_obstacle_removal():
    h = Harness()
    h.on_pallet()
    del h.record['release_tcp_rpy_rad']
    h.begin_pallet_pick()
    assert h.state == 'FAULT'
    assert not h.remove.requests
    assert h.record['obstacle_id'] == 'placed_item_1'


@pytest.mark.parametrize('rpy', [None, [], [0., 0., float('nan')]])
def test_retrieval_never_guesses_missing_or_invalid_tool_yaw(rpy):
    record = pallet_record(TARGET)
    record['release_tcp_rpy_rad'] = rpy
    with pytest.raises(ValueError, match='orientation is missing'):
        retrieval_values(record, 10, [0, 0, .1], [0, 0, 0, 1], .47)


def test_tilted_pallet_rejected():
    with pytest.raises(ValueError, match='horizontal'):
        retrieval_values(pallet_record(TARGET), 1, [0, 0, 0],
                         [math.sin(.1), 0, 0, math.cos(.1)], .47)


@pytest.mark.parametrize('key', list(PickPlaceTest.TOPICS))
def test_no_start_with_stale_dependency(key):
    h = Harness()
    h.received[key] = 0.
    assert not h.start('pack_new', Trigger.Response()).success
    assert not any(c.requests for c in h.test_clients.values())


@pytest.mark.parametrize('change', ['held', 'occupied', 'auto', 'other_item', 'fault'])
def test_start_preconditions(change):
    h = Harness()
    if change == 'held':
        h.status['scene']['attached_item_id'] = 'held'
    elif change == 'occupied':
        h.slot()['occupied'] = True
    elif change == 'auto':
        h.status['random']['continuous_loading_enabled'] = True
    elif change == 'other_item':
        h.status['scene']['placed_item_visual_ids'] = ['placed_item_99']
    else:
        h.status['supervisor']['robot_error'] = 52
    assert not h.start('pack_new', Trigger.Response()).success


def test_pack_new_estimates_before_random_planning_and_only_uses_matching_operation():
    h = Harness()
    assert h.start('pack_new', Trigger.Response()).success
    assert not h.worker.submit.called
    h.status['pickup'].update(state='OBJECT_INFO_READY', corrected_object=dict(
        size_x_m=.149, size_y_m=.136, size_z_m=.161, box_id=4))
    h.tick()  # stale successful operation must not start planning
    assert h.state == 'ESTIMATING'
    h.finish_owner('pickup', 'OBJECT_INFO_READY')
    assert h.state == 'PLANNING_RANDOM_TARGET'
    assert h.worker.submit.call_args.kwargs['dimensions_mm'] == (145, 135, 165)
    h.tick()
    assert h.state == 'WAIT_TARGET_ACK'
    h.ack(NS(data=[99., 1.]))
    h.advance()
    assert h.state == 'WAIT_TARGET_ACK'
    h.ack(NS(data=[float(h.sequence), 1.]))
    h.advance()
    assert h.state == 'PICK_AND_PLACE_NEW'
    assert len(h.test_clients['incoming'].requests) == 1
    h.status['cycle']['operation_id'] = h.expected
    h.status['scene']['attached_item_id'] = 'held_current_item'
    h.tick()
    h.status['scene']['attached_item_id'] = ''
    h.finish_owner('cycle')
    assert h.state == 'CONFIRM_PALLET_ITEM'
    h.status['scene']['placed_item_visual_ids'] = ['placed_item_1']
    h.tick()
    assert h.state == 'READY' and h.location == 'pallet'
    h.tick()
    assert not h.test_clients['grasp'].requests  # manual next button, no auto unpack


def test_repack_excludes_old_position_in_both_orientations():
    h = Harness()
    h.on_pallet()
    assert h.start('repack', Trigger.Response()).success
    assert h.worker.submit.call_args.kwargs['excluded_placements'] == {
        (10., 30., 0., False), (10., 30., 0., True)}


def test_unpack_uses_pick_waypoints_then_grasp_then_slot0_and_overhead():
    assert PickPlaceTest.SERVICES['store'] == '/staging_slots/store_chained'
    h = Harness()
    h.on_pallet()
    assert h.start('unpack', Trigger.Response()).success
    assert h.state == 'REMOVE_SOURCE_OBSTACLE'
    h.tick()
    assert h.state == 'REMOVE_SOURCE_OBSTACLE'
    h.status['scene']['placed_item_visual_ids'] = []
    h.tick()
    assert h.state == 'PREPARE_PICK_PATH'
    assert not h.test_clients['grasp'].requests
    h.advance()
    assert h.state == 'PICK_PATH'
    h.tick()
    assert h.state == 'SETTLE_PRE_PICK'
    assert not h.test_clients['grasp'].requests
    h.advance()
    assert h.state == 'GRASP_PALLET'
    h.status['scene']['attached_item_id'] = 'held'
    h.finish_owner('supervisor')
    assert h.state == 'SELECT_STORE_SLOT'
    h.advance()
    assert h.state == 'STORE_SLOT_AND_HANDOFF'
    assert len(h.test_clients['store'].requests) == 1
    h.status['staging']['state'] = 'SUCCEEDED'
    h.tick()
    assert h.state == 'STORE_SLOT_AND_HANDOFF'  # old success, never saw active
    h.status['staging']['state'] = 'TRANSFERRING_STORE'
    h.tick()
    h.status['staging'].update(state='SUCCEEDED', return_to_observation=False)
    h.slot().update(occupied=True, obstacle_id='placed_item_2', size_m=[.15, .11, .16])
    h.status['scene']['attached_item_id'] = ''
    h.tick()
    assert h.state == 'READY' and h.location == 'slot'


def test_store_retreat_fault_records_slot_location_but_keeps_test_faulted():
    h = Harness()
    h.on_pallet()
    h.step, h.location, h.state = 'unpack', 'carried', 'STORE_SLOT_AND_HANDOFF'
    h.slot().update(occupied=True, obstacle_id='placed_item_2', size_m=[.15, .11, .16])
    h.status['staging'].update(state='FAULT', fault='retreat failed')
    h.tick()
    assert h.state == 'FAULT'
    assert h.location == 'slot'
    assert h.record['obstacle_id'] == 'placed_item_2'
    assert not h.start('pack_slot', Trigger.Response()).success


def test_late_store_inventory_updates_location_without_clearing_fault():
    import json
    h = Harness()
    h.on_pallet()
    h.step, h.location, h.state = 'unpack', 'carried', 'FAULT'
    staging = dict(h.status['staging'], slots=[dict(
        slot=0, occupied=True, obstacle_id='placed_item_2', size_m=[.15, .11, .16])])
    h.receive('staging', NS(data=json.dumps(staging)))
    assert h.state == 'FAULT' and h.location == 'slot'


def test_slot_pick_goes_directly_to_shared_placement_without_observation():
    h = Harness()
    h.location, h.state, h.step = 'slot', 'READY', 'pack_slot'
    h.record = dict(item_id=4, size_mm=[110, 150, 160], obstacle_id='placed_item_2')
    h.slot().update(occupied=True, obstacle_id='placed_item_2', size_m=[.11, .15, .16])
    assert h.start('pack_slot', Trigger.Response()).success
    h.tick()
    h.ack(NS(data=[float(h.sequence), 1.]))
    h.advance()
    h.advance()
    assert h.state == 'RETRIEVE_SLOT'
    assert h.SERVICES['retrieve'].endswith('retrieve_chained')
    h.status['staging']['state'] = 'PICKING_RETRIEVAL'
    h.tick()
    h.status['staging']['state'] = 'SUCCEEDED'
    h.slot()['occupied'] = False
    h.status['scene']['attached_item_id'] = 'held'
    h.tick()
    assert h.state == 'PLACING'
    assert len(h.test_clients['place'].requests) == 1


def test_fault_cancels_all_coordinators_without_release_and_never_commits():
    h = Harness()
    h.on_pallet()
    h.state, h.step = 'PICK_PATH', 'repack'
    h.status['supervisor'].update(state='FAULT', fault='singularity')
    h.tick()
    assert h.state == 'FAULT' and 'singularity' in h.fault
    assert h.location == 'pallet'
    assert not h.loader.discard_pending.called
    assert len(h.test_clients['abort_staging'].requests) == 1
    assert len(h.test_clients['abort_motion'].requests) == 1
    assert all('gripper' not in v for v in h.SERVICES.values())


def test_late_service_acceptance_cannot_revive_aborted_step():
    h = Harness()
    callbacks = []
    h.test_clients['estimate'].call_async = lambda req: NS(add_done_callback=callbacks.append)
    h.start('pack_new', Trigger.Response())
    h.abort(None, Trigger.Response())
    callbacks[0](Future(NS(success=True)))
    assert h.state == 'FAULT' and not h.accepted


def test_replaced_operation_faults_instead_of_accepting_unrelated_success():
    h = Harness()
    h.start('pack_new', Trigger.Response())
    h.status['pickup'].update(operation_id=h.expected+1, state='OBJECT_INFO_READY')
    h.tick()
    assert h.state == 'FAULT'


def test_reset_does_not_erase_real_inventory():
    h = Harness()
    h.on_pallet()
    assert not h.reset(None, Trigger.Response()).success
    assert not h.loader.reset.called
    h.status['scene']['placed_item_visual_ids'] = []
    assert h.reset(None, Trigger.Response()).success
    assert h.location == 'empty' and h.record is None


@pytest.mark.parametrize('method,args', [
    ('_mode_set', (None,)), ('_send_next_joint', ()), ('_restore_control', ()),
    ('_hardware_activated', (None,)), ('_restore_controllers_received', (None,)),
    ('_controllers_deactivated', (None,)), ('_restore_tick', ()),
])
def test_staging_abort_blocks_late_motion_and_mode_callbacks(method, args):
    # A callback may only inspect the latch, never future.result() or clients.
    getattr(StagingSlots, method)(NS(abort_latched=True), *args)


def test_visual_only_items_count_as_inventory():
    assert placed_ids(dict(placed_item_ids=['a'], placed_item_visual_ids=['b'])) == {'a', 'b'}


def test_foreign_target_aborts_active_test():
    h = Harness()
    h.state, h.step, h.target = 'PLACING', 'repack', list(TARGET)
    h.target_seen(NS(data=TARGET))
    assert h.state == 'PLACING'
    h.target_seen(NS(data=[9., *TARGET[1:]]))
    assert h.state == 'FAULT' and 'another publisher' in h.fault


def test_request_timeout_never_retries_motion():
    h = Harness()
    h.clients_pending = NS(add_done_callback=lambda callback: None)
    h.test_clients['estimate'].call_async = Mock(return_value=h.clients_pending)
    h.start('pack_new', Trigger.Response())
    h.advance(11)
    assert h.state == 'FAULT'
    assert h.test_clients['estimate'].call_async.call_count == 1


def test_other_buffer_items_are_retained_when_resetting_test():
    h = Harness()
    h.status['staging']['slots'].append(dict(slot=1, occupied=True, obstacle_id='placed_item_55'))
    h.status['scene']['placed_item_ids'] = ['placed_item_55']
    assert not h.check_preconditions('pack_new')
    assert h.reset(None, Trigger.Response()).success
    assert h.status['scene']['placed_item_ids'] == ['placed_item_55']


def test_staging_abort_requires_explicit_reset_before_new_store_or_retrieve():
    assert 'abort latched' in StagingSlots._busy_reason(NS(abort_latched=True))


def test_real_node_spins_idle_timers_without_motion_requests(monkeypatch):
    """Exercise the actual executor, not just construction or fake callbacks."""
    def forbid_request(*args, **kwargs):
        pytest.fail('idle test node must not issue any service request')

    monkeypatch.setattr(ROSClient, 'call_async', forbid_request)

    class SpinningTestNode(PickPlaceTest):
        def __init__(self):
            self.tick_count = self.status_count = 0
            super().__init__()

        def tick(self):
            self.tick_count += 1
            super().tick()

        def publish_status(self):
            self.status_count += 1
            super().publish_status()

    # Isolated DDS domain; run this suite in a network-disabled container.
    rclpy.init(args=[], domain_id=223)
    executor = SingleThreadedExecutor()
    node = None
    try:
        node = SpinningTestNode()
        executor.add_node(node)
        deadline = time.monotonic() + 5.
        while (node.tick_count < 2 or node.status_count < 2) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        assert node.tick_count >= 2 and node.status_count >= 2
        assert node.state == 'IDLE' and node.location == 'empty'
        assert node.future is None and node.planning is None
        assert not callable(node.guards)  # ROS owns this iterable property.
    finally:
        executor.shutdown()
        if node is not None:
            node.worker.shutdown(wait=True, cancel_futures=True)
            node.destroy_node()
        rclpy.shutdown()


def test_coordinator_does_not_override_ros_node_properties():
    reserved = {name for name, value in vars(Node).items() if isinstance(value, property)}
    assert not reserved.intersection(vars(PickPlaceTest))


def six_slots(h):
    h.status['staging']['slots'] = [dict(slot=i, occupied=False) for i in range(6)]


def arm_random(h):
    assert h.start_random(None, Trigger.Response()).success
    h.random_next_at = 0.


def test_random_operations_follow_single_item_inventory():
    h = Harness()
    six_slots(h)
    assert h.random_choices() == {'pack_new': list(range(6))}
    h.on_pallet()
    h.slot(3).update(occupied=True, obstacle_id='other')
    assert h.random_choices() == {'unpack': [0, 1, 2, 4, 5], 'repack': [0]}
    h.test_slot = 5
    h.location = 'slot'
    h.slot().update(occupied=True, obstacle_id=h.record['obstacle_id'])
    assert h.random_choices() == {'pack_slot': [5]}
    h.slot()['obstacle_id'] = 'foreign'
    assert h.random_choices() == {}


def test_pack_unpack_only_filters_repack_and_preserves_slot_and_target_randomness():
    h = Harness()
    six_slots(h)
    assert h.set_random_pack_unpack_only(SetBool.Request(data=True), SetBool.Response()).success
    assert h.random_choices() == {'pack_new': list(range(6))}
    h.on_pallet()
    h.slot(2).update(occupied=True, obstacle_id='other')
    assert h.random_choices() == {'unpack': [0, 1, 3, 4, 5]}
    assert not h.check_preconditions('repack')  # manual repack remains available
    arm_random(h)
    h.start = Mock(return_value=Trigger.Response(success=True))
    slots = set()
    for _ in range(100):
        h.random_tick()
        assert h.start.call_args.args[0] == 'unpack'
        slots.add(h.test_slot)
    assert slots == {0, 1, 3, 4, 5}
    h.random_active = False
    h.location, h.test_slot = 'slot', 5
    h.slot().update(occupied=True, obstacle_id=h.record['obstacle_id'])
    assert h.random_choices() == {'pack_slot': [5]}


@pytest.mark.parametrize('active,busy', [(True, False), (False, True)])
def test_random_mode_cannot_change_during_run_or_step(active, busy):
    h = Harness()
    h.random_active = active
    if busy:
        h.state = 'PLACING'
    result = h.set_random_pack_unpack_only(SetBool.Request(data=True), SetBool.Response())
    assert not result.success and not h.random_pack_unpack_only


def test_unchecking_mode_restores_repack_without_starting_robot():
    h = Harness()
    six_slots(h)
    h.on_pallet()
    h.random_pack_unpack_only = True
    assert h.set_random_pack_unpack_only(SetBool.Request(data=False), SetBool.Response()).success
    assert 'repack' in h.random_choices()
    assert not h.random_active
    assert all(not c.requests for c in h.test_clients.values())


def test_random_new_pack_only_starts_once_without_automatic_retry():
    h = Harness()
    six_slots(h)
    arm_random(h)
    h.tick()
    assert h.state == 'ESTIMATING'
    h.tick()
    assert len(h.test_clients['estimate'].requests) == 1
    h.advance(241)  # Request accepted, but the operation never completed.
    assert h.state == 'FAULT' and not h.random_active
    h.tick()
    assert len(h.test_clients['estimate'].requests) == 1
    assert not h.start_random(None, Trigger.Response()).success


def test_random_choice_is_seeded_and_visits_available_slots_only():
    runs = []
    for _ in range(2):
        h = Harness()
        six_slots(h)
        h.on_pallet()
        h.slot(2).update(occupied=True, obstacle_id='other')
        arm_random(h)
        h.start = Mock(return_value=Trigger.Response(success=True))
        sequence = []
        for _ in range(100):
            h.random_tick()
            step = h.start.call_args.args[0]
            sequence.append((step, h.test_slot))
        runs.append(sequence)
    assert runs[0] == runs[1]
    assert {step for step, slot in runs[0]} == {'unpack', 'repack'}
    assert {slot for step, slot in runs[0] if step == 'unpack'} == {0, 1, 3, 4, 5}


def test_graceful_stop_counts_current_step_but_never_schedules_next():
    h = Harness()
    arm_random(h)
    h.tick()
    assert h.random_step_pending
    assert h.stop_random(None, Trigger.Response()).success
    h.finish('pallet')
    assert h.random_completed == 1 and not h.random_active
    h.tick()
    assert not h.remove.requests
    assert not h.test_clients['abort_cycle'].requests


def test_random_limit_stops_at_checkpoint_and_does_not_erase_inventory():
    h = Harness()
    h.on_pallet()
    h.random_max_steps = 1
    arm_random(h)
    h.step, h.random_step_pending = 'repack', True
    h.finish('pallet')
    assert h.random_completed == 1 and h.random_counts['repack'] == 1
    assert not h.random_active and h.record['obstacle_id'] == 'placed_item_1'
    h.tick()
    assert not h.remove.requests


@pytest.mark.parametrize('condition', ['stale', 'robot_fault', 'other_loader', 'pallet_unlocked'])
def test_random_interlocks_apply_between_steps(condition):
    h = Harness()
    arm_random(h)
    if condition == 'stale': h.received['scene'] = 0.
    if condition == 'robot_fault': h.status['supervisor']['robot_error'] = 52
    if condition == 'other_loader': h.status['random']['continuous_loading_enabled'] = True
    if condition == 'pallet_unlocked': h.pallet_locked = False
    h.tick()
    assert h.state == 'FAULT' and not h.random_active
    assert not h.test_clients['estimate'].requests


def test_random_ownership_blocks_manual_steps_reset_and_duplicate_start():
    h = Harness()
    arm_random(h)
    assert not h.start('pack_new', Trigger.Response()).success
    assert not h.reset(None, Trigger.Response()).success
    assert not h.start_random(None, Trigger.Response()).success
    assert h.abort(None, Trigger.Response()).success
    h.tick()
    assert not h.random_active and not h.test_clients['estimate'].requests


def test_nonzero_slot_is_selected_and_retrieved_from_recorded_inventory():
    h = Harness()
    six_slots(h)
    h.on_pallet()
    h.test_slot = 4
    h.step, h.location = 'unpack', 'carried'
    h.begin_stage(True)
    assert h.store_selection.publish.call_args.args[0].data == 4
    h.status['staging']['selected_store_slot'] = 4
    h.advance()
    assert h.state == 'STORE_SLOT_AND_HANDOFF'
    h.status['staging']['state'] = 'TRANSFERRING_STORE'
    h.tick()
    h.status['staging'].update(state='SUCCEEDED', return_to_observation=False)
    h.slot().update(occupied=True, obstacle_id='placed_item_2', size_m=[.11, .15, .16])
    h.tick()
    assert h.location == 'slot' and h.record['slot_id'] == 4
    h.step = 'pack_slot'
    h.begin_stage(False)
    assert h.retrieve_selection.publish.call_args.args[0].data == 4
    h.status['staging']['selected_retrieve_slot'] = 4
    h.advance()
    assert h.state == 'RETRIEVE_SLOT' and h.test_clients['retrieve'].requests


def test_slot_becoming_occupied_before_store_blocks_command():
    h = Harness()
    h.on_pallet()
    h.step, h.location = 'unpack', 'carried'
    h.begin_stage(True)
    h.slot().update(occupied=True, obstacle_id='foreign')
    h.advance()
    assert h.state == 'FAULT'
    assert not h.test_clients['store'].requests
    assert h.record['obstacle_id'] == 'placed_item_1'


def placed_before_retreat(h):
    h.step, h.state, h.location = 'pack_slot', 'PLACING', 'carried'
    h.target = list(TARGET)
    h.sequence = int(TARGET[0])
    h.scene_before = set()
    h.placement_started_sequence = h.sequence
    h.placement_attachment_seen_sequence = h.sequence
    h.owner, h.expected = 'place', 0
    h.status['scene']['placed_item_visual_ids'] = ['placed_item_3']
    h.status['scene']['attached_item_id'] = ''


def test_pallet_release_survives_retreat_fault_without_counting_success():
    h = Harness()
    placed_before_retreat(h)
    h.random_active = h.random_step_pending = True
    h.status['supervisor'].update(state='FAULT', fault='upward IK failed')
    h.tick()
    assert h.location == 'pallet' and h.state == 'FAULT'
    assert h.record['obstacle_id'] == 'placed_item_3'
    assert h.record['release_tcp_rpy_rad'] == [math.pi, 0., -math.pi/2]
    assert not h.random_active and h.random_completed == 0
    assert not h.start('unpack', Trigger.Response()).success
    assert not h.reset(None, Trigger.Response()).success


def test_retrieval_fault_cannot_commit_old_slot_release_as_new_pallet_placement():
    h = Harness()
    placed_before_retreat(h)
    h.state, h.location, h.owner = 'FAULT', 'slot', 'staging'
    h.placement_started_sequence = h.sequence-1
    h._record_pallet_release()
    assert h.location == 'slot' and h.record is None


@pytest.mark.parametrize('hardware_fault', [False, True])
def test_measurement_wait_pauses_phase_timeout_but_not_hardware_faults(hardware_fault):
    h = Harness()
    h.step, h.state, h.location = 'pack_slot', 'RETRIEVE_SLOT', 'slot'
    h.phase_started = time.monotonic()-1000
    h.status['staging']['state'] = 'INSPECTION_DEPTH'
    if hardware_fault:
        h.status['supervisor'].update(state='FAULT', fault='robot fault')
    h.tick()
    assert h.state == ('FAULT' if hardware_fault else 'RETRIEVE_SLOT')
    if hardware_fault:
        assert 'robot fault' in h.fault


def test_late_scene_detach_updates_pallet_location_without_clearing_fault():
    import json
    h = Harness()
    placed_before_retreat(h)
    h.state = 'FAULT'
    scene = dict(h.status['scene'])
    h.status['scene']['attached_item_id'] = 'held'
    h._record_pallet_release()
    assert h.location == 'carried'
    h.receive('scene', NS(data=json.dumps(scene)))
    assert h.location == 'pallet' and h.state == 'FAULT'


@pytest.mark.parametrize('guard', ['stale', 'ambiguous', 'attached', 'pending', 'wrong_sequence'])
def test_pallet_release_bookkeeping_does_not_guess_inventory(guard):
    h = Harness()
    placed_before_retreat(h)
    if guard == 'stale': h.received['scene'] = 0.
    if guard == 'ambiguous': h.status['scene']['placed_item_visual_ids'].append('placed_item_4')
    if guard == 'attached': h.status['scene']['attached_item_id'] = 'held'
    if guard == 'pending': h.status['scene']['attachment_pending'] = True
    if guard == 'wrong_sequence': h.sequence += 1
    h._record_pallet_release()
    assert h.location == 'carried' and h.record is None
