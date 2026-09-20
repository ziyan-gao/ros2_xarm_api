# Measured utilization

Policy loading retains two independent dimension triples:

- Planning dimensions: current ROS rounding (XY down to 5 mm; Z up to 5 mm).
- Measured dimensions: unrounded averaged point-cloud XY and contact-derived Z,
  recorded as floating-point millimetres in Neuromeka. These are measurements,
  not independently verified ground-truth dimensions.

The `/object_info_estimation/result` message keeps its existing first 11 values
unchanged (`id`, XYZ, quaternion, existing XYZ dimensions, lengths in metres).
Indices 11–13 optionally append the unrounded measured XYZ dimensions in metres.
Policy loading converts both triples to millimetres and passes the optional
`measured_dimensions_mm` argument to the Neuromeka loader. Robot target messages,
box rendering geometry, collision geometry and planning inputs are unchanged.

Three.js displays:

1. **Real utilization (measured)**: sum of unrounded measured XYZ products for
   successfully packed items, divided by the configured container volume.
2. **Planning utilization (rounded)**: the previous non-clearance metric.
3. **Virtual utilization (including clearance)**: virtual box volume ratio.

Pending placements and items in buffer slots are not counted as packed. Unpack
removes an item's contribution; slot retrieval and repack preserve its measured
volume. Measurement metadata belongs to the actual inventory object, not its
detector ID, since several boxes can share detector ID 0. Rotation does not change
volume. Clearing the packing inventory clears the metric too.

If any packed item lacks measurement metadata (old records or legacy producers),
the measured total is `null` and the UI shows `—`, never a partial sum or a rounded
substitute. Existing result files cannot recover measurements that were never
saved. Simulation without measurement metadata also shows `—` once items are packed.

Timestamped policy JSON now stores both triples in `planning_requested` events
and all three utilization values in event snapshots and the summary. Existing
`target_util` / MCTS reward still use the rounded planning metric; adding the
measurement report does not change search decisions.

Deploy updated ROS Python code, the Neuromeka source, and the frontend bundle,
then restart affected nodes while the robot is stopped and refresh the page.
Only new runs with the updated object-info producer can report measured utilization.
No clearance/grid or robot-motion changes are included in this feature.
