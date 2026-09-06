# Real-platform random stable loading

The `random_stable_loading` node connects incoming item localization to the
stable-loading implementation in `neuromeka_bin_packing`.

## Geometry

- Open loading volume: `450 x 550 x 450 mm`.
- The pallet-frame origin is the minimum-X, minimum-Y point on the pallet top.
- A loading target is the item's minimum-X, minimum-Y, bottom (`FLB`) corner.
- `clearance_mm` expands only the virtual item X/Y dimensions used by the
  packing model. It does not offset the published corner or shrink the open
  pallet footprint. Z always uses the physical item height.
- X/Y targets use the packer's 10 mm grid and Z targets use its 5 mm grid.

Set the virtual X/Y footprint clearance in the Compose project `.env` file:

```text
RANDOM_LOADING_CLEARANCE_MM=10
```

The value is read only when the coordinator starts. Restart the stack after
changing it, and do not change it while the physical/virtual pallet contains
placed items because all placements in one pallet run must use the same value.

## Real-platform candidate selection

Each incoming item uses this selection order:

1. Enumerate every placement accepted by the stable-loading validator.
2. Apply the existing vertical-loading height-map approach filter.
3. Randomly sample 30% of the remaining candidates without replacement
   (rounded up, with at least one candidate).
4. Select the sampled candidate minimizing `X + Y + Z`; ties prefer lower Z,
   then lower Y, then lower X.

The sample fraction can be changed in `.env` before starting Compose:

```text
RANDOM_LOADING_SAMPLE_FRACTION=0.30
```

The stability check requires a centered rectangular COM uncertainty region to
fit completely inside the support hull. Its X/Y side lengths are the configured
fraction of the real (unpadded) item footprint. The default is 20%:

```text
RANDOM_LOADING_COM_BOUND_RATIO=0.20
```

The **COM bound ratio** slider in the RViz Stable random loading panel changes
this value at runtime for subsequent candidate searches. Larger values demand
broader support and are therefore more conservative. Changes made while a
candidate search is already running are rejected, and do not alter an existing
pending target.

The status topic reports stable, vertically reachable, and sampled candidate
counts for each pending target.

## ROS interfaces

Input:

- `/item_localization/result` (`std_msgs/Float64MultiArray`): existing item
  localization result. Item localization collects the live
  `/pointcloud_detection/boxes` depth-refined marker over multiple frames.
  Elements 8, 9, and 10 are the averaged refined X/Y/Z dimensions in metres;
  the marker ID is retained only as the item identifier.
- `/random_stable_loading/config` (`std_msgs/Float64MultiArray`): element 0 is
  the COM-bound ratio in `(0, 1]`.

The current depth refinement measures the top-face X/Y footprint. Its Z value
is still the box height supplied to the depth-refinement seed because a single
top view does not independently observe the bottom face.

Output:

- `/random_stable_loading/target` (`std_msgs/Float64MultiArray`), with fields:

  ```text
  [sequence_id, item_id,
   corner_x_mm, corner_y_mm, corner_z_mm, rotate_90,
   raw_dx_mm, raw_dy_mm, raw_dz_mm,
   virtual_dx_mm, virtual_dy_mm, virtual_dz_mm]
  ```

- `/random_stable_loading/status` (`std_msgs/String`): JSON state and target
  information, including `visualization_url` and any `visualization_fault`.

## Live Three.js visualization

The coordinator starts the existing `neuromeka_bin_packing` Three.js live
server by default. Open this URL on the robot computer:

```text
http://127.0.0.1:8765
```

Committed placements are drawn using their virtual X/Y footprint, so the
configured clearance is visible. The tooltip reports both physical and virtual
dimensions. The yellow polygon is the physical support hull and should fit
inside the virtual footprint. A sampled but uncommitted loading target is shown
as a highlighted virtual item. It becomes a normal packed item only after the
matching PickAndPlace operation reports success; discarding the pending target
removes it from the view.

For the requested display orientation, the renderer swaps pallet X and Y on
screen. This affects only the Three.js view; ROS targets and pallet-frame values
remain in their original `(X, Y, Z)` order.

The server can be configured before Compose startup:

```bash
RANDOM_LOADING_VISUAL_PORT=8766 docker compose up
RANDOM_LOADING_VISUALIZE=false docker compose up
```

The pallet-localization node applies the target without replacing its marker,
offset, or pallet settings. It acknowledges the sequence on
`/random_stable_loading/target_applied` as
`[sequence_id, accepted, reason_code]`. Out-of-bounds targets are explicitly
rejected instead of leaving the coordinator waiting indefinitely.

## Robot execution and commit

Automatic robot motion is disabled by default. After the target status becomes
`TARGET_READY`, start it with:

```bash
ros2 service call /random_stable_loading/start_pick_place std_srvs/srv/Trigger '{}'
```

To enable automatic PickAndPlace after target acknowledgement and pallet lock,
start Compose with:

```bash
RANDOM_LOADING_AUTO_START=true docker compose up
```

The placement remains pending while the robot is moving. It is committed to
the virtual pallet only when the matching `/pick_place_pipeline/status` reports
`SUCCEEDED`. A failed operation leaves the target pending for operator review.

Operator services:

- `/random_stable_loading/start`: run one complete cycle: refined-item
  localization, target selection, target acknowledgement, and PickAndPlace.
- `/random_stable_loading/abort`: stop an active cycle and discard only its
  uncommitted target. Previously committed pallet state is retained.

- `/random_stable_loading/discard_pending`: discard a failed/unexecuted target
  without changing the virtual pallet.
- `/random_stable_loading/reset_pallet`: clear all committed virtual pallet
  and stability state, restart the seeded candidate sequence, and immediately
  clear the Three.js scene. Use this only when the physical pallet is also
  empty.

The RViz panel exposes these as **Stable random loading → Random loading,
Abort, Reset**. Reset requires confirmation.

## Build

The image gains the packing runtime dependencies and the Compose configuration
mounts `neuromeka_bin_packing` read-only. Rebuild once after this change:

```bash
cd /home/robot/tase2026_revision/src/ros2_xarm_api
docker compose build
docker compose up
```
