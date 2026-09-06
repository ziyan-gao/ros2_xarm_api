# Planning-scene obstacles

The unified stack adds fixed and localized workspace surfaces to MoveIt's world
through `/apply_planning_scene`. All dimensions and poses use metres in
`link_base`.

## Fixed obstacles

The supplied poses describe each top-face center. Collision-box centers are
therefore shifted downward by half the full height. The robot base is rotated
-90 degrees about Z relative to the table measurement frame, so table poses in
`link_base` use the inverse transform: `(x, y) -> (-y, x)` and +90 degrees yaw.

| ID | Size X/Y/Z | Top-face center in `link_base` | Collision-box center in `link_base` | Yaw |
| --- | --- | --- | --- | --- |
| `work_table` | 1.5 / 2.5 / 1.3 | 0.6 / 0.6 / -0.03 | 0.6 / 0.6 / -0.68 | +90 deg |
| `secondary_table` | 1.3 / 2.5 / 1.4 | 0.2 / -2.5 / 0.1 | 0.2 / -2.5 / -0.6 | +90 deg |

The `negative_x_wall` collision box represents the wall approximately 1000 mm
from the robot base in the negative-X direction. Its nearest face is at
`x = -1.000 m`; its size is `0.05 x 6.0 x 3.0 m` and its center is
`(-1.025, -1.0, 0.0) m`. It occupies the region beyond the measured wall face,
so the full 1000 mm remains available to the robot.

The `positive_y_wall` collision box represents the wall approximately 1200 mm
from the robot base in the positive-Y direction. Its nearest face is at
`y = +1.200 m`; its size is `6.0 x 0.05 x 3.0 m` and its center is
`(0.0, 1.225, 0.0) m`. The collision volume extends beyond the wall face in
positive Y, leaving the full measured 1200 mm clear on the robot side.

## Pallet surface

No pallet object exists while pallet localization is unlocalized or only in
preview. Once `/pallet_localization/status` reports `LOCKED`, the node reads the
`link_base -> pallet_frame` transform and inserts `pallet_surface` using the
configured pallet X/Y dimensions. `pallet_frame` remains at the marker corner;
the finite collision geometry is offset locally by `(size_x/2, size_y/2)` so it
extends away from that corner over the pallet deck. A 5 mm thick finite box
represents the deck plane. This is preferable to an infinite geometric plane,
which has no finite corner and would divide the entire MoveIt workspace at the
pallet height.

Clearing pallet localization removes `pallet_surface` from the planning scene.

## Placed items

Each carried-item attachment is keyed to the pickup supervisor's monotonic
`operation_id` and that ID is consumed after the planning-scene attachment
succeeds. Repeated or delayed status messages from an already-consumed pickup
are ignored. This prevents a message from the previous cycle from re-arming
attachment after release and capturing the next item's TCP-to-object transform
before its pre-grasp motion has completed. The status fields
`pending_pickup_operation_id` and `last_attached_pickup_operation_id` expose the
lifecycle guard for diagnostics.

The **Place cycle** checkbox **Add placed item as MoveIt obstacle** controls
whether the attached item is converted into a world collision box when vacuum
is released. The default is enabled. When disabled, the attached object is
removed cleanly without creating `placed_item_N`; previously created placed
objects remain until pallet localization is cleared.

Before real motion, inspect the objects in RViz and deliberately request a plan
through each table to confirm MoveIt rejects it.

During the place singularity fallback, the vacuum can release the item above
its nominal resting height. For an active stable-random-loading cycle, the
resulting `placed_item_N` is therefore reconstructed from the matching pending
`/random_stable_loading/target`: pallet-frame minimum-X/minimum-Y/bottom corner,
raw item dimensions, and clockwise-rotation flag. Normal placements continue
to derive the obstacle from the measured release-time TCP transform. The status
field `last_placed_item_pose_source` reports which source was used.
