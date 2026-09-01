# Planning-scene obstacles

The unified stack adds fixed and localized workspace surfaces to MoveIt's world
through `/apply_planning_scene`. All dimensions and poses use metres in
`link_base`.

## Fixed tables

The supplied poses describe each top-face center. Collision-box centers are
therefore shifted downward by half the full height.

| ID | Size X/Y/Z | Top-face center | Collision-box center |
| --- | --- | --- | --- |
| `work_table` | 1.5 / 2.5 / 1.3 | 0.6 / -0.5 / -0.03 | 0.6 / -0.5 / -0.68 |
| `secondary_table` | 2.5 / 1.8 / 1.9 | -1.5 / 1.2 / 1.0 | -1.5 / 1.2 / 0.05 |

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

The **Place cycle** checkbox **Add placed item as MoveIt obstacle** controls
whether the attached item is converted into a world collision box when vacuum
is released. The default is enabled. When disabled, the attached object is
removed cleanly without creating `placed_item_N`; previously created placed
objects remain until pallet localization is cleared.

Before real motion, inspect the objects in RViz and deliberately request a plan
through each table to confirm MoveIt rejects it.
