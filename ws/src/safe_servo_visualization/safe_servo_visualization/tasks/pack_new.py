"""pack_new: task flow and tunable motion policy. Restart only this task after edits."""
MOTION = dict(pick_force_n=10.0, transport_force_n=8.0, place_force_n=4.0,
              departure_lift_m=0.100, retreat_speed_mm_s=10.0,
              sdk_pick_retreat=True, sdk_place_retreat=True)


def begin(n):
    n.phase('ESTIMATING')
    n.call('estimate_sam' if getattr(n, 'new_item_sam_enabled', False) else 'estimate', 'pickup')


def target_ready(n):
    from ..pick_place_test_node import placed_ids
    n.scene_before = placed_ids(n.status['scene'])
    n.phase('PICK_AND_PLACE_NEW')
    n.call('incoming', 'cycle')


def grasp_ready(n):
    n.begin_place()


def excluded_placements(n):
    return set()
