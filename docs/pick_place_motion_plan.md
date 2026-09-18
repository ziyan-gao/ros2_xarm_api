# Collision-Aware Pick-and-Place Motion Plan

## Continuous new-item transport

Combined PickAndPlace now defaults to one **lift → overhead transfer →
pre-place descent** trajectory. After contact measurement and verified suction,
pickup holds at contact (without performing its old separate retreat). The
place pipeline waits for the attached-item geometry, prepares the pallet target,
then plans and validates the complete path before moving.

- Cartesian lift/descent legs have fixed orientation. Two rounded upper corners
  lie above the configured item-bottom clearance; yaw changes smoothly on the
  elevated segment. The blend adds up to 40 mm to the clearance-level TCP height,
  plus a 2 mm geometry margin. A workspace-ceiling violation rejects the path.
- MoveIt's Cartesian path service solves the entire path with collision checks;
  no sampling-based free-space planner or direct-service fallback is used here.
  Partial paths are rejected. The timed controller spline is additionally checked
  for joint limits, collision, tool tilt, and carried-item clearance. Sampled
  collision checks are not a continuous-collision guarantee.
  Continuous transport does not apply the start-relative
  `direct_transfer_max_joint_delta_rad` cap. Absolute joint-position limits,
  collision checks, and velocity/acceleration limits remain enforced. The cap
  remains unchanged for the legacy direct joint-transfer path.
  Endpoint velocities are explicitly set to zero before spline validation and
  timing adjustment; nonzero endpoint velocities returned by the Cartesian
  service no longer reject an otherwise usable path.
- One FollowJointTrajectory goal executes all three legs in ROS control/mode 1.
  The non-servo speed slider scales limits. Feasible original interval times are
  retained exactly; violations receive local timing repairs with smooth
  neighboring transitions (0.5 s neighborhood in original trajectory time).
  The worst interval no longer unconditionally stretches the whole path.
  Shared knot velocities are reduced using the local time scale; acceleration
  is recomputed and all intervals rechecked after each repair. A verified uniform
  timing candidate is used only if it is shorter than the local repair result.
  Repairs are bounded to 20 passes / a 10 s processing deadline and fail closed
  if unresolved. Logs report original/final duration, adjusted interval count,
  timing strategy and the initial limiting joint/interval/derivative.
  These changes do not relax the configured velocity/acceleration limits
  and do not guarantee the original duration for an infeasible timed path. Interior stopped
  waypoints are allowed. Shared interior accelerations are obtained with a
  tridiagonal minimum-integrated-squared-jerk solve, retaining the planner's
  knot positions. Only endpoints and full stops are constrained
  to zero acceleration, rather than artificially resetting acceleration at
  every dense waypoint. JTC uses C2 quintic interpolation, with continuous
  acceleration including at stops. This minimizes integrated jerk for the
  fixed knot data, not necessarily peak jerk or total motion time; derivative
  limits can still require slower motion on difficult paths.
  Analytic extrema of this spline are checked for joint bounds, speed,
  acceleration and (when enabled) jerk; the retimed, nanosecond-quantized curve is checked again
  and sampled for collision/FK validation at <=20 ms and <=0.02 rad intervals.
  `CONTINUOUS_TRANSPORT_ENFORCE_JERK_LIMIT=false` is the default: the recently
  added software jerk cap no longer stretches otherwise valid paths. This
  restores velocity/acceleration-only timing checks; it does not impose a
  numerical jerk ceiling. Predicted peak jerk is still logged, and acceleration
  remains continuous. Set the option to `true` to enforce
  `CONTINUOUS_TRANSPORT_MAX_JOINT_JERK_RAD_S3` (10 rad/s³ at 100% speed by default,
  scaled with the non-servo slider). This is an extra software guard, not a
  manufacturer-certified limit. Jerk may change at knots in either mode;
  this is not a guarantee of hardware tracking or absence of vibration.
  The robot still stops at pre-place to enable final safe-servo.
  Controller success enters `TRANSPORT_VERIFYING`: a complete measured joint
  sample must be received and timestamped after completion. FK from that sample
  must place the TCP within 10 mm of pre-place. Verification allows up to five
  seconds for settling/service feedback; stale samples or a persistent mismatch
  cannot authorize descent. Cached 2 Hz Servo TCP telemetry is not used here.
  Before enabling Servo, the place pipeline acknowledges the completed transport
  through the coordinator and waits for matching-operation `SUCCEEDED` telemetry,
  then observes the existing telemetry-settling interval. A failed acknowledgement
  or a five-second confirmation timeout faults without starting descent.
- Force contact on the final non-servo descent cancels that goal. Release requires
  accepted cancellation, a terminal controller result, and fresh stationary joint
  feedback for 250 ms. Failed stop confirmation keeps the gripper closed and faults.
  Confirmed contact uses the existing release, upward retreat and observation flow.
- Before-motion kinematic rejections retain the random-loading resampling path.
  Invalid timing or derivative data stops the cycle without excluding loading poses.
  Execution failures do not resample or silently switch to direct motion.
- Pick-only, staging store/retrieve, and standalone legacy place remain unchanged.

Docker `.env` settings (defaults shown):

```ini
CONTINUOUS_TRANSPORT_ENABLED=true
CONTINUOUS_RETURN_ENABLED=true
CONTINUOUS_TRANSPORT_BLEND_RADIUS_M=0.04
CONTINUOUS_TRANSPORT_ENFORCE_JERK_LIMIT=false
```

Set `CONTINUOUS_TRANSPORT_ENABLED=false` to restore the previous split sequence.
Recreate the Compose service to apply changes (`docker compose up -d --force-recreate`);
the startup script builds the mounted ROS workspace. Do this only with the robot
stopped and the cell clear. This feature requires real-hardware commissioning at
reduced speed; offline regression tests do not validate physical stopping distance.

### Connected post-release return

For continuous-transport cycles, successful pallet release/detachment now starts
two stages: **slow vertical retreat to pre-place**, then one
**vertical lift → rounded overhead transition → observation** trajectory.
The first leg uses the existing direct-mode ownership handoff and a relative
vertical command at at most `return_clearance_speed_mm_s` (default 10 mm/s,
allowed range (0, 30]), with acceleration capped at 100 mm/s². It never moves
downward if release already occurred above pre-place. Completion and live height
are checked before planning the continuous leg.
There is a stationary handoff/planning interval between the stages,
with no required stop at the elevated waypoint. Interior all-joint stops are
allowed with the same C2 interpolation and derivative checks as outbound paths.
If MoveIt returns a complete geometric return path but no usable timing
(empty velocity arrays or non-increasing timestamps), or smooth retiming
exceeds the bounded validation sample budget on return,
the supervisor instead uses the previous staged route:
confirm Servo paused, deactivate ROS writers and confirm direct mode, retreat
vertically to the saved overhead height, restore ROS control with its existing
fresh-feedback/settling gate, then request the separate observation plan.
The slow pre-place clearance leg is not repeated and the rejected trajectory is
never submitted. This fallback is one-shot and requires fresh stationary robot
feedback, no attached item, and an upward target within the workspace ceiling.
All response points are checked for joint names, array lengths and finite
positions/derivatives before any indexing. An untimed return must also match
the measured start and saved observation joints and satisfy joint-position
bounds before selecting fallback. No missing velocities are filled with zeros
to make an untimed path executable. Outbound timing failure faults while
retaining the carried item; it does not trigger release or resampling.
Collision, partial-path, malformed geometry/arrays, hardware, and execution faults
still stop the cycle; they do not trigger this fallback. This is containment,
not a guarantee of jerk continuity or singularity avoidance. The empty TCP initially retreats
at fixed orientation to at least the recorded transfer TCP height; orientation
changes occur on the elevated segment. Both upper bends stay above that level.

Servo must confirm pause before the slow leg; ROS controllers are deactivated
and direct mode 0 is confirmed before that command. After the slow leg,
the existing hardware/controller restoration
sequence then confirms mode 1, active controllers and fresh joint samples. An
already healthy mode-1 system retains ownership. Direct-mode recovery restores
the hardware and controllers before continuing. Both routes enforce
`POST_RESTORE_SETTLE_SEC` (default 0.75 s), the configured ready-sample count, and
an uninterrupted healthy interval; a readiness interruption restarts the delay.
A 30-second handoff deadline prevents an indefinite wait. No return trajectory
is sent until this gate has completed.

The saved observation joints in `config/taught_waypoints.yaml` anchor the return:
Cartesian IK is solved backward from observation, then the timed path is reversed
and fully revalidated. A mismatching release IK branch is rejected rather than
joined by an unchecked joint move. Absolute joint limits, collision checks,
clearance, endpoint speeds, and timing limits remain enforced. Completion also
requires fresh observation TCP and joint verification; the place pipeline then
finishes without commanding observation a second time.

Confirmed early-contact and singularity-recovery releases in the same cycle use
this return too. A failed release/detach does not authorize it. Return planning
errors fault; they do not resample a loading target for an item already released.
Set `CONTINUOUS_RETURN_ENABLED=false` to retain split retreat/observation behavior
without disabling continuous outbound transport. Pick-only and staging are unchanged.

### Servo-J write-stall containment

The patched hardware requests STOP and returns ERROR after any failed Servo-J
write, one write taking at least 100 ms, or two consecutive writes over 30 ms.
The fault latch blocks subsequent writes and lifecycle reactivation; automatic
controller restoration cannot clear it. Inspect the robot and communication
fault before an operator-approved hardware-process restart. A restart is not a
repair of the latency source, and must not be followed by automatic cycle replay.
This watchdog measures a blocking call after it returns: it cannot prevent the
initial stall or guarantee STOP delivery over a stalled connection.

These C++ patch changes require rebuilding the Docker image; restarting an old
image only loads the Python changes, not the hardware watchdog fix. Validate in
isolation first. No live restart or physical motion is performed by offline tests.

### Controller fault cleanup and partial-path diagnostics

`patches/ros2_control_fault_stop.patch` fixes stop-only cleanup when a hardware
error has already made every interface in the stop list unavailable. Existence
checks still apply, and every interface must belong to a component in the current
read/write cycle's failed-hardware list. Merely unconfigured hardware is not
exempted. Requests with start interfaces do not use the exception;
the hardware fault latch still blocks reactivation. The image builds a
`hardware_interface` overlay from pinned ros2_control 4.48.0 and requires the
installed version to match, avoiding a silent ABI mismatch. Rebuilding and
recreating the image is necessary; the live stack is not updated by source edits.

For partial Cartesian responses, the supervisor now probes the requested
waypoint interval identified by `fraction * waypoint_count`, seeded from the
last returned joints. This is an approximation: MoveIt does not return the exact
failed internal interpolation pose. A diagnostic IK request ignores collisions
only to separate solving from a subsequent state-validity query. Diagnostic
solutions are never executed. Logs report the target/waypoint/XYZ, IK result,
joint-bound violations, or up to eight collision pairs. A valid nearby probe
does not validate the whole path. All outcomes preserve the original rejection
and random-target resampling, with a two-second diagnostic deadline and stale
callback guards. This explains future partial paths; it does not retrospectively
prove why the earlier 60.9% path failed.

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
4. Use MoveIt to plan and execute the collision-aware transfer above the
   pallet, then place the item.
5. Reverse the vertical loading path, then use MoveIt to return to observation.

## Design principles

### Separate transit and contact control

Use safe servo only for short vertical contact searches near an item or placement
surface. The automatic cycle uses the UFACTORY Cartesian service for local
vertical lift, loading, and retreat segments. With **Keep EEF perpendicular to
pallet** selected, the transfer target keeps the configured perpendicular
orientation. Elevated free-space transfer uses MoveIt/KDL IK followed by
collision-checked joint interpolation. Periodic axes use their nearest
equivalent angle, and any IK branch requiring more than 180 degrees on one
joint is rejected. Automatic RRTConnect fallback is disabled by default; an
invalid direct path stops the cycle before robot motion. The checkbox constrains
the target orientation, not
every intermediate orientation along the joint interpolation. Return to the
observation pose uses normal MoveIt planning. The pipeline never executes an
unchecked joint interpolation or a long Cartesian transfer.

Only one command source may control the robot at a time. Switching between MoveIt trajectory execution and SDK servo mode must be explicit, and the previous motion must be stopped and confirmed complete first.

### Unified non-servo speed control

The panel's **Non-servo motion speed** slider controls both MoveIt trajectory
velocity scaling and UFACTORY Cartesian-service velocity. It intentionally does
not change safe-servo descent speed. The slider is a percentage of the
application's commissioned envelope:

```text
MoveIt velocity/acceleration scaling = slider_percent / 100  (5% .. 100%)
Cartesian service speed = 200 * slider_percent / 100 mm/s
Cartesian service acceleration = 500 * slider_percent / 100 mm/s^2
Direct joint speed limit = 2.14 * slider_percent / 100 rad/s
Direct joint acceleration = 10.0 * slider_percent / 100 rad/s^2
```

For example, with the default commissioned envelope, 30% selects 30% MoveIt
velocity and acceleration scaling, 60 mm/s and 150 mm/s^2 linear-service
motion, and a 0.642 rad/s deterministic joint-transfer limit. The Cartesian
maximums are configured by `DIRECT_CARTESIAN_MAX_SPEED_MM_S` and
`DIRECT_CARTESIAN_MAX_ACCEL_MM_S2`. Safe-servo contact descent remains
independently limited.

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
After the vertical pickup retreat reaches `pick_clearance`, MoveIt's
`/compute_ik` service runs the configured KDL solver for the desired
`link_tcp` pose, seeded with the live joint state and checked against the
current planning scene. The returned joints are normalized toward the current
configuration, rejected if the jump is excessive, and sampled at no more than
0.05 rad intervals using MoveIt's `/check_state_validity`. A valid joint line
is time-scaled with a smooth quintic profile and sent to
`/uf850_traj_controller/follow_joint_trajectory` without a controller-mode
handoff. IK or collision-validation failure before motion falls back to normal
MoveIt RRTConnect planning; failure after trajectory execution starts faults
the cycle. The direct Cartesian service is reserved for the short vertical move
from `place_transfer` to the 30 mm pre-place clearance.

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

After each direct-to-ROS controller handoff, fresh joint states alone are not
treated as sufficient readiness. The supervisor additionally requires xArm
mode 1, a motion-ready state, error code zero, and fresh joint telemetry to
remain valid for `POST_RESTORE_SETTLE_SEC` (0.75 seconds by default) across at
least five consecutive checks before the next MoveIt trajectory may start.

If MoveIt Servo reports a singularity hard-stop, or remains in singularity
deceleration for 0.75 seconds during the guarded place descent, the supervisor
honors that condition and disables Servo. It then
hands exclusive control to the direct xArm Cartesian service and continues
downward in synchronous `PLACE_SINGULARITY_STEP_M` increments (3 mm by default,
at 10 mm/s). After every completed step it reads fresh TCP and Fz telemetry.
The next step is issued only while contact has not been detected, the configured
place floor has not been reached, and the separate
`PLACE_SINGULARITY_RECOVERY_TIMEOUT_SEC` recovery window (20 seconds by
default) has not expired. Controller/mode handoff has its own timeout; the
recovery window starts only after direct control is confirmed, immediately
before the first step. Contact, floor/timeout exhaustion, or stale telemetry
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

### Force/torque controller error C52

The pickup supervisor monitors the xArm controller error field independently
of Servo status. If C52 (six-axis force/torque sensor zero-setting error)
appears during guarded descent, Servo is stopped immediately. Descent never
continues without trustworthy force feedback. When an item is attached and a
recorded elevated waypoint exists, vacuum remains enabled and the supervisor
performs the normal exclusive-controller handoff and vertical retreat. The
emergency path first disables the FT sensor, clears the controller error and
warning, and leaves the sensor disabled while the loaded retreat runs; it
never zeroes a sensor carrying an item. The handoff accepts active, inactive,
or watchdog-unconfigured ros2_control hardware, restores the controllers, and
then latches the cycle in `FAULT` with the item still attached.

Automatic motion remains blocked while `ft_recovery_required` is true. After
the operator removes the physical item and clears its planning-scene
attachment, **Recover FT sensor** performs at most two unloaded recovery
attempts: disable FT, clear controller error and warning, enable FT, wait 500
ms, zero FT, wait 500 ms, require zero controller error plus a fresh post-zero
force sample, and restore ros2_control. Failure remains latched and requires
checking sensor wiring and power. The recovery service is
`/pickup_supervisor/recover_ft_sensor`.

The ros2_control Servo-J write watchdog warns above 30 ms. It stops the robot
only after three consecutive late writes or one write of at least 250 ms. A
watchdog-generated error 999 is explicitly cleared after a verified hardware
lifecycle reactivation; genuine SDK return errors remain fatal. This prevents
a recovered synthetic latency latch from immediately shutting down the
controller manager.

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

## Optional depth-only box measurement

The RViz panel's **Box dimension estimation** section can switch from the
marker-seeded refinement to **Use depth only (no marker)**. In this mode the
aligned depth image is projected into `link_base`, cropped to Z = 70--300 mm,
and cropped in `link_tcp` to X = 50--400 mm and Y = -300--300 mm. A horizontal
RANSAC plane supplies the top face. Its minimum-area XY rectangle gives the
box X/Y dimensions, while `top_plane_z - support_z` gives its height. The
estimated X/Y/Z values and fit percentage appear numerically in the panel and
as a text marker in RViz.

The bounds and support height are configured by `DEPTH_ONLY_*` values in
`.env`. In particular, `DEPTH_ONLY_SUPPORT_Z_M` must be the physical table
surface Z in `link_base`; an error in this value produces the same error in
the raw box height. `DEPTH_ONLY_HEIGHT_OFFSET_M` is then added to that raw
height; its default is `-0.02`, so a raw 170 mm estimate is published as
150 mm. The panel exposes this correction from -30 to +30 mm. The six crop
bounds can also be adjusted live, in
millimetres, with sliders in the same RViz panel section. The orange
**Depth-only Truncated Cloud** display shows exactly which points remain after
the current crop; slider changes are runtime-only and reset to `.env` values
when the stack restarts. Depth-only mode assumes one box with a visible,
approximately horizontal top face inside the TCP-relative region of interest.
It publishes the result through the existing refined-box topic, so downstream
item localization uses it while the checkbox is selected.

The panel's **Box stability** line mirrors `/item_localization/status`.
`DETECTING` shows the number of accepted samples, `UNSTABLE` shows which pose,
angle, or dimension spread exceeded its tolerance, and only `READY` means the
configured stable-sample requirement has passed.

### Contact-corrected object information

**Estimate Object Info** uses the existing pickup motion path without enabling
the vacuum. It first guarantees the saved observation pose (the motion
coordinator skips execution when the measured joints are already within
0.02 rad), then collects 20 distinct timestamped depth-refined box estimates.
It averages their pose and dimensions and accepts them only when maximum
position, orientation, and dimension spreads remain within 5 mm, 2 degrees,
and 10 mm respectively. Re-reading one cached marker does not increase the
sample count. The averaged box is used for the 30 mm pre-grasp, after which
safe-servo touches the top face. At confirmed contact it latches:

```text
height = contact_tcp_z - empty_table_contact_tcp_z
center_z = empty_table_contact_tcp_z + height / 2
```

The depth-derived X/Y center and yaw are retained. Final X/Y dimensions are
rounded downward to the 5 mm grid (`149 -> 145`, `136 -> 135`, `131 -> 130`)
while contact-derived Z remains unmodified at this stage. The panel
shows `OBTAINED` only while the TCP remains at that measured contact pose with
the vacuum off. The corrected geometry is published on
`/object_info_estimation/result` and visualized by the **Contact Corrected
Object** RViz display.

Starting PickAndPlace in this state activates the vacuum immediately and then
retreats; it does not repeat observation, detection, or descent. Starting it
without latched information runs this estimation sequence first and continues
directly into pickup. After the supervised pickup succeeds, the latch and RViz
estimation marker are cleared. `OBJECT_CONTACT_REFERENCE_Z_M` sets the startup
reference, and **Empty-table contact Z** adjusts it live in the panel.

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
