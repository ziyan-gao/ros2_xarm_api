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
4. Use MoveIt to transfer it to the pallet-relative pre-place pose and place it.
5. Retreat vertically and use MoveIt to return to observation.

## Design principles

### Separate transit and contact control

Use MoveIt for collision-checked movement through free space. Use safe servo only for short vertical approaches near an item or placement surface. After contact, use the existing UFACTORY driver's slow straight-line Cartesian service for the short vertical retreat. Long transfers must not be performed by Cartesian servoing.

Only one command source may control the robot at a time. Switching between MoveIt trajectory execution and SDK servo mode must be explicit, and the previous motion must be stopped and confirmed complete first.

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
8. Servo vertically back to `pre_grasp_pose`.
9. Stop safe-servo commands and switch back to trajectory control.
10. Attach the detected item geometry to the tool in the MoveIt planning scene.

No lateral movement is allowed until the tool has returned to the pre-grasp clearance.

### 4. Transfer and placement

Teach `pre_place` above the target surface with sufficient vertical clearance.
Use MoveIt to plan and execute a collision-checked trajectory:

```text
pre-grasp -> pre-place
```

MoveIt checks the attached item together with the robot and world obstacles.
The optional intermediate waypoint remains available for manual commissioning,
but is not required by the automatic place pipeline.

At `pre_place_pose`:

1. Switch to safe-servo control.
2. Descend vertically to `place_pose`.
3. Disable the vacuum gripper.
4. Confirm release when feedback is available.
5. Remove the attached item from the tool and add its released box geometry to
   the world as a placed-item collision obstacle.
6. Use the direct driver to retreat vertically back to `pre_place_pose`.
7. Switch back to trajectory control.

### 5. Retreat and return

Plan and execute a collision-checked MoveIt trajectory:

```text
pre-place -> observation joint configuration
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
  -> PLAN_TRANSFER
  -> EXECUTE_TRANSFER
  -> SERVO_PLACE_DESCENT
  -> VACUUM_OFF
  -> DETACH_OBJECT
  -> SERVO_PLACE_RETREAT
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
- Integrate safe-servo descent, vacuum activation and grasp verification, followed by a direct-driver vertical retreat.
- Abort safely on stale perception, planning failure, force fault, or vacuum failure.

Completion criterion: one stationary test item can be picked repeatedly at low speed without lateral servo motion.

### Phase 5: Planning-scene item attachment

Static table and localized pallet collision geometry is introduced before
Phase 5 item attachment. See
[planning-scene obstacles](planning_scene_obstacles.md).
The attachment behavior and test procedure are documented in
[Phase 5 planning-scene attachment](phase5_planning_scene_attachment.md).

- Add the environment collision objects.
- Create item collision geometry from detected dimensions.
- Attach it after grasp confirmation and detach it after release.
- Verify collision checking includes the carried item's swept volume.

Completion criterion: deliberately obstructed or self-colliding transfer requests are rejected before robot motion.

### Phase 6: Removed — standard MoveIt transfer

The separate continuous/blended-transfer phase is no longer required. Standard
MoveIt collision-checked plans move the attached item to pre-place and return
the released robot to observation. An intermediate waypoint is optional rather
than part of the place pipeline.

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

## First implementation task

Begin with Phase 1. Update the container build to include the MoveIt/xArm planning stack and establish a collision-checked, low-speed joint-space motion to a manually specified target. Do not integrate perception or autonomous cycling until this foundation is verified.
