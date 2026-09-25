"""unpack: task flow and tunable motion policy. Restart only this task after edits."""
MOTION = dict(pick_force_n=10.0, transport_force_n=8.0, place_force_n=4.0,
              departure_lift_m=0.100, retreat_speed_mm_s=10.0,
              sdk_pick_retreat=True, sdk_place_retreat=True)


def begin(n):
    n.begin_pallet_pick()


def target_ready(n):
    n.begin_pallet_pick()


def grasp_ready(n):
    n.begin_stage(True)


def excluded_placements(n):
    return set()
