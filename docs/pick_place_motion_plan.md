# Collision-Aware Pick-and-Place Motion Plan

## Goal

Implement a repeatable pick-and-place cycle for the UFACTORY 850 that combines:

- MoveIt collision-aware planning for free-space motion.
- The existing safe-servo controller for short, constrained vertical motion.
- Vacuum-gripper feedback and explicit recovery behavior.
- Saved operator-taught joint configurations for repeatable observation and transit poses.

The intended cycle is:

1. Move to the item observation configuration.
2. Detect the item's pose and dimensions.
3. Move above the item, descend, and pick it up.
4. Use MoveIt to transfer the attached item above the pallet and place it.
5. Reverse the vertical loading path, then use MoveIt to return to observation.

## Design principles

### Separate transit and contact control

Use safe servo only for short vertical contact searches near an item or placement
surface. The automatic cycle uses the UFACTORY Cartesian service only for local
vertical lift, loading, and retreat segments. Cross-table transfer and return to
observation use collision-checked MoveIt plans so the planner selects a valid
joint-space branch and checks the complete swept volume.

Only one command source may control the robot at a time. Switching between MoveIt trajectory execution and SDK servo mode must be explicit, and the previous motion must be stopped and confirmed complete first.

### Unified non-servo speed control

The panel's **Non-servo motion speed** slider controls both MoveIt trajectory
velocity scaling and UFACTORY Cartesian-service velocity. It intentionally does
not change safe-servo descent speed. The slider is a percentage of the
application's commissioned envelope:

```text
MoveIt velocity scaling = slider_percent * 0.003  (1.5% .. 30%)
Cartesian service speed = slider_percent mm/s    (5 .. 100 mm/s)
```

For example, 30% selects 9% MoveIt velocity scaling and 30 mm/s linear-service
motion. MoveIt acceleration scaling remains at the conservative configured
limit.

### Save taught configurations as joint positions

Save the observation and optional intermediate waypoints primarily as six joint
positions. Define pre-place as a configurable pose relative to `pallet_frame`.

A Cartesian TCP pose can have multiple inverse-kinematics solutions. Saving joint positions preserves the intended elbow and wrist configuration and reduces the risk of returning through an undesirable self-collision-prone posture.

### Represent the carried item in the planning scene

After a successful pickup, add the detected item as an attached collision object using its measured dimensions. Remove it after placement. The planning scene must also contain the table, pallet, fixtures, and the complete wrist-mounted tool geometry.

## Motion sequence

### 1. Observation

The operator teaches and saves an observation configuration from the ROS/RViz panel. The saved data should contain:

- Joint positions.
- TCP pose and frame ID.
- Robot model/tool configuration.
- A user-facing name and timestamp.

MoveIt plans and executes a joint-space motion to this configuration. The TCP pose is used as a consistency check, not as the primary command.

### 2. Detection and grasp generation

The perception pipeline estimates the item pose and dimensions in the robot base frame. From this result, generate:

- `grasp_pose`: the desired TCP pose at pickup.
- `pre_grasp_pose`: the same orientation at a configurable vertical clearance above the grasp pose.

Before execution, validate:

- Detection age and confidence.
- TF availability and timestamp consistency.
- Item dimensions; MoveIt validates kinematic reachability and collision limits.
- Inverse-kinematics feasibility.
- Collision-free planning to `pre_grasp_pose`.
- Acceptable joint configuration and path length.

Reject the cycle if any validation fails.

### 3. Pickup

1. Use MoveIt to plan and execute to `pre_grasp_pose`.
2. Confirm that trajectory execution completed successfully.
3. Switch to safe-servo control.
4. Descend vertically while holding roll, pitch, and yaw.
5. Stop at the target height or configured force/contact condition.
6. Enable the vacuum gripper.
7. Confirm vacuum/grasp success when feedback is available.
8. Use the Cartesian service to retreat vertically to `pick_clearance`.
9. Stop safe-servo commands and switch back to trajectory control.
10. Attach the detected item geometry to the tool in the MoveIt planning scene.

No lateral movement is allowed until the tool has returned to the pre-grasp clearance.

### 4. Transfer and placement

The automatic cycle defines two elevated waypoints: `pick_clearance` above the
grasp and `place_transfer` above the final target. Both use the same configured
height for the carried item's left-bottom-depth corner:

```text
item corner in pallet_frame: Z = TRANSFER_CORNER_HEIGHT_M = 0.470 m
```

After contact, pickup converts this corner height to the corresponding absolute
TCP Z using the pallet pose, measured item height, and configured grasp offset.
It then retreats vertically to `pick_clearance`. The place-transfer target uses
the same pallet-frame corner height above the configured pre-place X/Y, making
the cross-table transfer nominally level.
After the vertical pickup retreat reaches `pick_clearance`, MoveIt plans and
executes the collision-checked cross-table motion to `place_transfer`. When
**Keep EEF perpendicular to pallet** is enabled, the transfer plan also carries
a path orientation constraint: TCP roll/pitch remain within 3 degrees of the
downward target orientation while yaw is unrestricted. The downward direction
is aligned with `link_base` Z: localized pallet yaw is retained, but measured
pallet roll/pitch is deliberately removed so localization noise cannot make the
post-pick vertical TCP an invalid constrained-planning start state. A direct
linear service move then descends vertically to the 30 mm pre-place clearance.

If MoveIt reports a planning failure before transfer execution begins, the
place pipeline can use a degraded direct-transfer fallback. It hands control
from `ros2_control` to the xArm driver and sends the already validated transfer
TCP XYZ/RPY to `/ufactory/set_position` with `motion_type=1` (prefer linear
motion, then use the controller's joint-space IK). This fallback deliberately
bypasses MoveIt collision checking. It is never started for a failed or
partially executed trajectory, and a rejected xArm motion remains a hard fault.
After the target is reached, `ros2_control` and fresh joint-state feedback are
restored before linear loading begins.

At `pre_place_pose`:

1. Switch to safe-servo control.
2. Descend vertically until guarded placement contact.
3. Disable the vacuum gripper.
4. Confirm release when feedback is available.
5. Remove the attached item from the tool and add its released box geometry to
   the world as a placed-item collision obstacle.
6. Disable safe-servo and use the Cartesian service to retreat vertically back
   to `place_transfer`.
7. Switch back to trajectory control.

If MoveIt Servo reports its specific singularity hard-stop during the guarded
place descent, the supervisor honors that stop and disables Servo. It then
hands exclusive control to the direct xArm Cartesian service and continues
downward in synchronous `PLACE_SINGULARITY_STEP_M` increments (3 mm by default,
at 10 mm/s). After every completed step it reads fresh TCP and Fz telemetry.
The next step is issued only while contact has not been detected, the configured
place floor has not been reached, and the separate
`PLACE_SINGULARITY_RECOVERY_TIMEOUT_SEC` recovery window (20 seconds by
default) has not expired. The recovery window starts when the system changes
from Servo to direct stepping, so a Servo deceleration timeout cannot consume
it before the first step. Contact, floor/timeout exhaustion, or stale telemetry
causes the item to be released and detached, followed by a vertical retreat to the
recorded `place_transfer` height and a MoveIt return to observation. Thus any
unobserved contact travel is bounded to one 3 mm step rather than one continuous
service descent. Other Servo faults remain hard failures, and a singularity
during pickup does not release the item automatically.

The direct linear move from `place_transfer` to the pre-place pose is also
guarded using a fresh transfer-waypoint Fz baseline. Two consecutive samples at
or above the configured 4 N baseline-relative threshold interrupt the linear
descent, release and detach the item, reverse vertically to `place_transfer`,
and return to observation without starting the safe-servo place descent.

The transfer-to-pre-place command uses `wait=false`, allowing the supervisor to
continue monitoring live TCP Z and Fz and to call `/ufactory/set_state`
immediately. Reaching the requested Z restores `ros2_control` and starts
safe-servo. If either this linear loading descent or the subsequent safe-servo
place descent exceeds `PLACE_DESCENT_TIMEOUT_SEC` (20 seconds by default), the
same degraded release, detach, vertical-retreat, and observation-return
sequence is used. Waiting for the direct-motion stop acknowledgement is also
bounded; if stopping cannot be confirmed, automatic release is blocked and the
pipeline faults rather than opening the gripper while motion may still be
active.

Pickup keeps its commissioned `+50 mm` TCP lower bound. Placement uses the
separate `PLACE_WORKSPACE_Z_MIN_MM` bound, currently `-100 mm`, because the
pallet-side TCP can legitimately pass below the robot-base Z origin. The
150 mm maximum place search, force-contact stop, and independent force/torque
safety caps still apply.

### 5. Retreat and return

Plan and execute a collision-checked MoveIt trajectory:

```text
place-transfer -> observation joint configuration
```

Plan the return independently after releasing the item because the planning scene and payload state have changed.

## Planning and collision requirements

Install and configure these upstream packages:

- `xarm_controller`
- `xarm_moveit_config`
- `xarm_planner`
- MoveIt 2 and the Pilz industrial motion planner

The planning model must include:

- The UF850 links and correct joint limits.
- Force sensor, vacuum gripper, camera, and other wrist hardware.
- Conservative collision padding.
- Table, pallet, boxes, fixtures, and permanent barriers.
- The detected item while it is attached to the tool.

Initial deployment should use conservative velocity and acceleration scaling. Every planned trajectory must be collision-validated immediately before execution. Cartesian path generation is reserved for short local tool motions; it must not replace general collision-aware transit planning.

## State machine

```text
IDLE
  -> MOVE_TO_OBSERVATION
  -> DETECT_ITEM
  -> PLAN_PRE_GRASP
  -> MOVE_TO_PRE_GRASP
  -> SERVO_GRASP_DESCENT
  -> VACUUM_ON
  -> SERVO_GRASP_RETREAT
  -> ATTACH_OBJECT
  -> PICK_CLEARANCE
  -> PLAN_MOVEIT_TRANSFER
  -> EXECUTE_MOVEIT_TRANSFER
  -> LINEAR_LOAD_TO_PRE_PLACE
  -> SERVO_PLACE_DESCENT
  -> VACUUM_OFF
  -> DETACH_OBJECT
  -> LINEAR_RETREAT_TO_TRANSFER
  -> PLAN_RETURN
  -> EXECUTE_RETURN
  -> MOVE_TO_OBSERVATION
```

Every state must define:

- Entry conditions.
- Success conditions.
- Timeout.
- Cancellation behavior.
- Fault transition.
- Robot mode/controller ownership.

Any uncertain outcome must transition to a safe stopped fault state rather than continuing the cycle.

## Implementation phases

### Phase 1: Install and verify the planning stack

- Extend the Docker image to install MoveIt and build the required xArm packages.
- Launch the UF850 MoveIt configuration against the real robot.
- Verify robot description, joint states, controller connection, and planning in RViz.
- Plan and execute low-speed motions without perception or servo integration.

Completion criterion: a saved joint target can be planned, visually checked, and executed safely on the robot.

### Phase 2: Taught waypoint storage

Implementation and commissioning details: [Phase 2 waypoint storage](phase2_waypoint_storage.md).

- Add panel controls to save observation and intermediate joint configurations.
- Persist configurations in a versioned YAML file.
- Add load, overwrite, validation, and display functions.
- Prevent saving while the robot is moving or joint data is stale.

Completion criterion: restarting the system preserves both configurations and MoveIt can return to them consistently.

### Phase 3: Motion orchestration foundation

Implementation and commissioning details: [Phase 3 motion coordinator](phase3_motion_coordinator.md).

- Add a pick-and-place coordinator node with the explicit state machine.
- Implement controller ownership and mode transitions.
- Add action/service interfaces for start, cancel, pause, reset, and status.
- Implement timeouts and fault propagation without enabling autonomous cycling yet.

Completion criterion: the coordinator can execute and cancel observation/intermediate test movements with deterministic state reporting.

### Phase 4: Pre-grasp generation and pickup

MoveIt Servo foundation and commissioning boundary:
[Phase 4A MoveIt Servo foundation](phase4_moveit_servo_foundation.md).

The Phase 4B supervised pickup implementation and commissioning procedure are
documented in the same guide.

The initial pre-grasp implementation plans only. It consumes one fresh refined
depth box, creates a downward-facing TCP target above its measured top face,
and previews the collision-checked plan before any execution is enabled.

- Consume the refined item pose and dimensions.
- Generate and validate grasp and pre-grasp poses.
- Plan to pre-grasp with MoveIt.
- After execution succeeds, allow 0.75 seconds for TCP telemetry to settle,
  then require the existing Cartesian pre-grasp validation before descent.
- Integrate safe-servo descent, vacuum activation and grasp verification, followed by a direct-driver vertical retreat.
- Abort safely on stale perception, planning failure, force fault, or vacuum failure.

Completion criterion: one stationary test item can be picked repeatedly at low speed without lateral servo motion.

### Phase 5: Planning-scene item attachment

Static table and localized pallet collision geometry is introduced before
Phase 5 item attachment. See
[planning-scene obstacles](planning_scene_obstacles.md).
The attachment behavior and test procedure are documented in
[Phase 5 planning-scene attachment](phase5_planning_scene_attachment.md).
The place target specifies the carried object's minimum-X/minimum-Y/bottom
corner in `pallet_frame`. The captured TCP-to-object grasp transform converts
that target into the required TCP pose, with 30 mm of automatic vertical
pre-place clearance before guarded descent. When **Rotate item 90 deg clockwise
about pallet Z** is enabled, the coordinator shifts the object's local corner
using its measured X dimension. Clockwise is defined when looking along the
positive pallet-Z axis toward the pallet. The configured XYZ remains the rotated
footprint's minimum pallet-X/minimum pallet-Y/bottom corner; the box does not
rotate around and retain its former physical corner.

When **Keep EEF perpendicular to pallet** is enabled, the target TCP is
reconstructed so its tool-Z axis follows downward `link_base` Z. Placement and
pallet yaw are preserved, but incidental roll/pitch from both the captured
grasp transform and pallet localization are not reproduced. This gives pickup,
constrained transfer, vertical loading, and guarded descent one consistent
definition of vertical.
Marker-to-pallet XYZ calibration corrects any displacement between the ArUco
origin and the physical pallet top; the current calibrated Z offset is
`+21.3 mm`. Placement contact triggers on either a debounced raw-Fz sign
reversal or 4 N of baseline-relative Z-force change. Independent 12 N force
and torque safety caps remain active.

- Add the environment collision objects.
- Create item collision geometry from detected dimensions.
- Attach it after grasp confirmation and detach it after release.
- Verify collision checking includes the carried item's swept volume.

Completion criterion: deliberately obstructed or self-colliding transfer requests are rejected before robot motion.

### Phase 6: Standard MoveIt transfer

MoveIt plans and executes the cross-table motion from pickup clearance to the
elevated place-transfer TCP pose while the item is attached in the planning
scene. The dedicated constrained-pose service applies the orientation constraint
to the complete transfer path, rather than only setting the endpoint quaternion.
The UF850 OMPL group declares a joint-space projection evaluator so constrained
RRTConnect planning can construct and explore its start tree reliably.
Only the subsequent vertical load to pre-place uses the direct Cartesian service.
After release and vertical retreat, MoveIt independently plans the return to
observation. An intermediate taught waypoint remains optional rather than
mandatory.

### Phase 7: Placement and complete cycle

Implementation and commissioning details:
[place function](phase7_place_function.md).

- Configure a pallet-relative pre-place pose and require a locked pallet frame.
- Integrate safe-servo descent, release verification, and retreat.
- Complete planning-scene updates.
- Enable repeated cycles only after all single-cycle fault cases pass.

Completion criterion: multiple supervised cycles finish successfully, and every injected fault produces a safe stop.

## Safety and validation checklist

- Use reduced speed, acceleration, and payload during commissioning.
- Keep a physical emergency stop accessible.
- Validate all frame IDs and transforms before planning.
- Reject stale joint, force, vacuum, or perception data.
- Prevent simultaneous MoveIt and SDK servo commands.
- Require vertical clearance before lateral travel.
- Check the complete trajectory, not only its endpoint.
- Confirm the attached-object state matches the physical vacuum state.
- Replan if the environment, start state, or detected object changes.
- Log state transitions, plans, execution results, force events, and vacuum events.
- Test cancellation and communication loss at every motion state.

The panel's manual operations provide **Open gripper**, **Close gripper**, and
**Clear placed obstacles**. Gripper commands are rejected while the pickup
supervisor is active. Clearing obstacles removes only accumulated placed-item
boxes; the tables, pallet surface, robot/tool geometry, and any currently
attached item remain in the planning scene.

## First implementation task

Begin with Phase 1. Update the container build to include the MoveIt/xArm planning stack and establish a collision-checked, low-speed joint-space motion to a manually specified target. Do not integrate perception or autonomous cycling until this foundation is verified.
