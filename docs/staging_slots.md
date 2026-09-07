# Unpacking staging slots

The `staging_slots` node provides a testable physical holding area for unpacking
and repacking. The frame is `link_base`, dimensions are metres internally, and
every staged item has zero roll, pitch, and yaw in that frame.

| Slot | Left-depth-bottom corner (mm) | Bounds (mm) |
|---|---|---|
| 0 | `(-375, 180, 0)` | X `[-375,-125]`, Y `[180,430]` |
| 1 | `(-125, 180, 0)` | X `[-125,125]`, Y `[180,430]` |
| 2 | `(125, 180, 0)` | X `[125,375]`, Y `[180,430]` |
| 3 | `(-375, 430, 0)` | X `[-375,-125]`, Y `[430,680]` |
| 4 | `(-125, 430, 0)` | X `[-125,125]`, Y `[430,680]` |
| 5 | `(125, 430, 0)` | X `[125,375]`, Y `[430,680]` |

The original, unrotated item X/Y footprint must fit within 250 x 250 mm.

## Store

1. Complete a normal pickup so the item is attached to `link_tcp` in the
   MoveIt planning scene.
2. Select a store slot in the RViz panel.
3. Press **Store carried item**.

The node asks MoveIt's `/compute_ik` service for collision-free joint solutions
at two endpoints: item bottom 480 mm above the live pallet origin and the 30 mm
pre-place pose. The second solve is seeded with the first solution so both stay
on the same nearby IK branch. It then disables Servo, deactivates the ROS
controllers and hardware, confirms xArm mode 0 from live telemetry, and sends
those joint configurations through the direct xArm joint-motion service. This
is joint-space motion, not a Cartesian linear path.

After restoring mode 1, hardware, controllers, and fresh joint states, guarded
safe-servo performs the contact descent. The supervisor releases the item,
records the actual release TCP pose, retreats vertically to the 480 mm
waypoint, restores ROS control, and the staging node returns the robot to the
saved observation pose.

## Retrieve

Select an occupied slot and press **Retrieve staged item**. The staged collision
object is removed, and MoveIt plans to 30 mm above the higher of the recorded
release TCP and geometry-predicted top-contact pose. This prevents contact
compliance or Z-calibration error from consuming the clearance. The normal
supervised pickup then performs the final descent and controller handoff. The
item is reattached to `link_tcp` before the robot returns to the observation
pose. Before pickup begins, the staging coordinator waits for fresh robot TCP
telemetry to remain within the pre-pick tolerance for three consecutive samples;
this prevents a just-finished MoveIt trajectory from being checked against an
older Servo transform sample.

Slot records are intentionally retained by **Reset staging fault**. They exist
in memory only and are cleared when the stack is restarted, so the physical
staging area must be reconciled manually after a restart.
