from copy import deepcopy
from std_srvs.srv import SetBool, Trigger
import pytest
from packing.real_platform_loading import RealPlatformRandomLoader
from safe_servo_visualization.two_item_test import disjoint_xy
from test_pick_place_test import Harness


def harness(locations=('pallet', 'pallet')):
    h = Harness()
    h.two_item_mode, h.active_item = True, 0
    h.loader = RealPlatformRandomLoader(container_size=(450, 550, 450), clearance_mm=10,
                                        vertical_loading_filter_enabled=False)
    h.test_items = {}
    h.status['staging']['slots'] = [dict(slot=i, occupied=False) for i in range(6)]
    for key, location in enumerate(locations):
        record = dict(item_id=key, corner_mm=[key*200, 0, 0], virtual_corner_mm=[key*200, 0, 0],
                      size_mm=[100, 100, 100], virtual_size_mm=[110, 110, 100],
                      obstacle_id=f'placed_item_{key+1}', release_tcp_rpy_rad=[3.14159, 0., 0.])
        if location == 'slot':
            record['slot_id'] = key+2
            h.status['staging']['slots'][key+2].update(occupied=True, obstacle_id=record['obstacle_id'])
        h.test_items[key] = dict(location=location, record=record)
    h.status['scene']['placed_item_visual_ids'] = [v['record']['obstacle_id'] for v in h.test_items.values()]
    h.state = 'READY'
    h._select_item(0)
    return h


@pytest.mark.parametrize('locations, expected', [
    (('pallet', 'pallet'), {'unpack', 'repack'}),
    (('slot', 'slot'), {'pack_slot'}),
    (('pallet', 'slot'), {'unpack', 'repack', 'pack_slot'})])
def test_two_item_legal_operations_and_identity(locations, expected):
    h = harness(locations)
    before = deepcopy(h.test_items)
    choices = h.random_choices()
    assert set(choices) == expected
    assert h.test_items == before and h.active_item == 0
    for step, candidates in choices.items():
        for key, slot in candidates:
            assert locations[key] == ('slot' if step == 'pack_slot' else 'pallet')
            if step == 'pack_slot':
                assert slot == key+2
            if step == 'unpack':
                assert h.status['staging']['slots'][slot]['occupied'] is False


def test_bootstrap_second_item_before_rearranging():
    h = harness(('pallet',))
    choices = h.random_choices()
    assert set(choices) == {'pack_new'}
    assert all(key == 1 for key, slot in choices['pack_new'])
    assert h.active_item == 0 and h.record['item_id'] == 0


@pytest.mark.parametrize('mode', ['one_sided', 'symmetric'])
def test_real_planner_filters_other_item_and_never_stacks(mode):
    h = harness()
    h.loader = RealPlatformRandomLoader(container_size=h.container, clearance_mm=20,
        clearance_mode=mode, vertical_loading_filter_enabled=False)
    others = h._other_pallet_items()
    for _ in range(6):
        result = h.loader.plan(item_id=0, dimensions_mm=(100, 100, 100), placement_filter=h._floor_filter())
        values = result.target_values
        assert values[4] == 0 and values[14] == 0
        assert all(disjoint_xy(values[12:15], values[9:12], other) for other in others)
        h.loader.discard_pending(result.sequence_id)


def test_inventory_updates_one_item_without_overwriting_other():
    h = harness()
    other = deepcopy(h.test_items[1])
    h.location = 'slot'
    h.record['slot_id'] = 4
    h._sync_item()
    assert h.test_items[0]['location'] == 'slot'
    assert h.test_items[1] == other


def test_no_third_new_item_and_no_unpack_with_full_slots():
    h = harness()
    for s in h.status['staging']['slots']:
        s.update(occupied=True, obstacle_id='external_'+str(s['slot']))
    assert 'pack_new' not in h.random_choices()
    assert 'unpack' not in h.random_choices()


def test_pack_only_when_other_item_blocks_all_floor_is_not_offered():
    h = harness(('pallet', 'slot'))
    h.test_items[0]['record']['virtual_size_mm'] = [450, 550, 100]
    h._select_item(0)
    assert 'pack_slot' not in h.random_choices()


def test_first_item_pauses_for_operator_second_item_keeps_running():
    for locations, expected_active in [(('pallet',), False), (('pallet', 'pallet'), True)]:
        h = harness(locations)
        h.step = 'pack_new'
        h.random_active = h.random_step_pending = True
        h.finish('pallet')
        assert h.random_active is expected_active
        assert h.random_completed == 1
        assert h.state == 'READY'


def test_mode_change_and_reset_cannot_forget_tracked_items():
    h = harness(('slot', 'slot'))
    assert not h.set_two_item_mode(SetBool.Request(data=False), SetBool.Response()).success
    assert not h.reset(Trigger.Request(), Trigger.Response()).success
    assert len(h.test_items) == 2


def test_pack_unpack_only_excludes_repack_for_both_items():
    h = harness()
    h.random_pack_unpack_only = True
    assert set(h.random_choices()) == {'unpack'}
