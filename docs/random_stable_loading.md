# Real-platform random stable loading

The `random_stable_loading` node connects incoming item localization to the
stable-loading implementation in `neuromeka_bin_packing`.

## Shared configuration

Random-loading geometry and its corresponding robot lift height are configured
in one file:

```text
/home/robot/tase2026_revision/neuromeka_bin_packing/configs/real_platform_random.yaml
```

The configuration contains these fields (values below are illustrative):

```yaml
container_size_mm: [450, 550, 450]
clearance_mm: 20
clearance_mode: symmetric
seed: 101
scan_downscale: 2
com_bound_ratio: 0.20
height_tolerance: 10.0
vertical_loading_filter_enabled: false
packing_height_resolution_mm: 5
robot_motion:
  transfer_corner_margin_above_container_mm: 20
```

`container_size_mm` is `(X, Y, Z)`. The robot transfer target for the carried
item's left-bottom-depth corner is derived automatically:

```text
transfer corner height = container Z + transfer margin
```

Therefore, changing container Z also changes the pickup retreat and transfer
height. For example, a 350 mm container with a 20 mm margin produces a 370 mm
transfer corner height. Keep the margin large enough to clear the physical
container rim and measurement error.

`clearance_mm` is per-side X/Y padding when `clearance_mode: symmetric`. The
virtual envelope starts one clearance distance before the physical item and
ends at least one clearance distance after it on both axes. Grid rounding can
add extra padding on the positive side. The robot target remains the physical
item corner. Use `clearance_mode: one_sided` to restore the previous behavior,
where one clearance value is added only beyond the positive-X/positive-Y side.
Clearance never changes item Z. Symmetric clearance must be a multiple of the
10 mm packing grid.

`height_tolerance` is measured in millimetres and has the same meaning as in
`real_platform_policy.yaml`: support cells up to this distance below the
candidate's nominal contact level participate in the support-region test. This
prevents small measured height differences from discarding real contact area.

`vertical_loading_filter_enabled` controls the additional top-down approach
height-map filter. When `false`, every stability-valid candidate remains
eligible. It does not disable contact/support stability validation.

The file is validated when the stack starts. Invalid dimensions or parameters
stop startup instead of letting the packing and motion nodes use inconsistent
geometry. Restart the stack after changing it. A rebuild is not required
because the repository is mounted into the container.

The container-side path defaults to:

```text
/opt/neuromeka_bin_packing/configs/real_platform_random.yaml
```

It can be changed through `RANDOM_LOADING_CONFIG_PATH` in `.env`, provided the
new file is readable inside the container.

## Geometry

- Open loading volume: the configured `container_size_mm`.
- The pallet-frame origin is the minimum-X, minimum-Y point on the pallet top.
- A loading target is the physical item's minimum-X, minimum-Y, bottom corner.
- The packer separately tracks the minimum corner of the virtual clearance
  envelope. In symmetric mode it is `physical corner - clearance` in X and Y.
- `clearance_mm` is loaded from the shared YAML described above.
- X/Y targets use the packer's 10 mm grid and Z targets use its 5 mm grid.

Do not change the geometry while the physical/virtual pallet contains placed
items because all placements in one pallet run must use the same configuration.

## Real-platform candidate selection

Each incoming item uses this selection order:

1. Enumerate every placement accepted by the stable-loading validator.
2. If `vertical_loading_filter_enabled` is true, apply the vertical-loading
   height-map approach filter; otherwise retain all stable candidates.
3. Find the minimum Z among all remaining candidates.
4. Select uniformly at random from the candidates on that minimum-Z layer.

X and Y are not part of the score. Candidate-fraction sampling and the former
`min(X + Y + Z)` rule are disabled.

After pickup and lift, but before transfer motion, the supervisor checks the
vertical path from the chosen transfer joint configuration through pre-place
to the nominal placement height. Explicit poses at intervals of at most 5 mm
are solved sequentially through MoveIt IK, using each solution as the next
seed. Equivalent periodic angles are normalized within their configured limits.
Failed IK and joint changes above 0.15 rad per spatial step are rejected;
time-resampled trajectory points are not used for this check. This
is a kinematic check; contact-path collision checking is disabled, while the
existing transfer collision checks still apply. It does not guarantee that
the UFACTORY controller will accept the path or certify a singularity margin.

For random loading, a pre-motion kinematic rejection excludes that candidate's
position and rotation for the current item. Selection then repeats uniformly
at the lowest remaining Z, and placement resumes with the already-held item.
Committed items are unchanged. If all candidates are exhausted, the cycle
faults with the item held. Controller faults, validation service failures,
and failures after motion starts do not trigger this retry. Exclusions are
cleared when a new item is estimated.

The stability check requires a centered rectangular COM uncertainty region to
fit completely inside the support hull. Its X/Y side lengths are the configured
fraction of the real (unpadded) item footprint. The YAML default is 20%.

The **COM bound ratio** slider in the RViz Stable random loading panel changes
this value at runtime for subsequent candidate searches. Larger values demand
broader support and are therefore more conservative. Changes made while a
candidate search is already running are rejected, and do not alter an existing
pending target.

The status topic reports stable, vertically reachable, and minimum-Z-pool
candidate counts for each pending target.

## ROS interfaces

Input:

- `/object_info_estimation/result` (`std_msgs/Float64MultiArray`): the
  contact-corrected item result. Before pre-grasp motion, object-information
  estimation accepts and averages 20 distinct stable depth-refined frames.
  Elements 8 and 9 are X/Y dimensions rounded downward to 5 mm; element 10 is
  the contact-derived Z dimension. The random-loading coordinator then rounds
  Z upward to its configured packing-height resolution.
- `/random_stable_loading/config` (`std_msgs/Float64MultiArray`): element 0 is
  the COM-bound ratio in `(0, 1]`.

The current depth refinement measures the top-face X/Y footprint. Its Z value
is still the box height supplied to the depth-refinement seed because a single
top view does not independently observe the bottom face.

Output:

- `/random_stable_loading/target` (`std_msgs/Float64MultiArray`), with fields:

  ```text
  [sequence_id, item_id,
   physical_corner_x_mm, physical_corner_y_mm, physical_corner_z_mm, rotate_90,
   raw_dx_mm, raw_dy_mm, raw_dz_mm,
   virtual_dx_mm, virtual_dy_mm, virtual_dz_mm,
   virtual_corner_x_mm, virtual_corner_y_mm, virtual_corner_z_mm]
  ```

  The last three fields let pallet localization validate the complete virtual
  envelope while commanding the robot to the physical corner. Consumers that
  still publish the legacy 12-field message remain supported; their physical
  and virtual corners are assumed identical.

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
