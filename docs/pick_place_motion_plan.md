# Collision-Aware Pick-and-Place Motion Plan

[中文文档](pick_place_motion_plan_zh.md)

## Overhead waypoint retry search (current default)

Slot storage now freezes fresh `link_base -> link_tcp` orientation at the store
request. Only 441 X/Y offsets are searched; no rotation-location variants or
slot-yaw-zero requirement. Target computation bounds the rotated eight corners
and aligns their minimum XYZ to the slot corner; an oversized footprint or item
tilt over 2.5 degrees rejects storage. More than 0.5 degree orientation drift
before planning rejects the stale target. Collision checks remain enabled.
Release records retain actual object yaw and the original object-to-TCP grasp.
Retrieval transmits that grasp to scene attachment, so subsequent pallet targets
use the preserved transform rather than assuming a zero-yaw object. Legacy slot
records without the extension still use their former zero-yaw convention.

Slot storage again uses the overhead offset-rail search. The staged visit to the
actual observation pose is reverted. Adjacent translations are blended;
rotation remains a separate checked leg.

Carried-item transfers now try up to 1323 deterministic Cartesian candidates,
within a 180-second total planning/validation budget. Candidates vary the
observation-side rail X/Y from -150 to +150 mm inclusive in 15 mm steps
(21 × 21 offsets × 3 rotation locations), rotate at the first rail point,
second rail point or above the destination. No roll/pitch tilt detours are added;
the existing pickup/placement orientations are retained (normally differing in yaw).
Every rotation point stays at the computed overhead Z;
the final timed path must also pass carried-item corner clearance, collision,
joint-limit and endpoint checks. The final placement orientation is unchanged.
All candidates restart from the original measured joint seed without moving.
The nominal candidates are tried first, followed by nearby offsets before
farther ones. The time budget can stop the search before all candidates are tried.

`.env` controls: `TRANSPORT_EXPANDED_WAYPOINTS_ENABLED=true`,
`TRANSPORT_MOVEIT_FALLBACK_ENABLED=false`, and
`TRANSPORT_WAYPOINT_SEARCH_TIMEOUT_SEC=180.0` (1–600 seconds).
This disables the OMPL/GetMotionPlan fallback, not MoveIt's Cartesian IK,
collision checking or trajectory controller. Exhaustion stops with the item
held; it does not bypass validation through SDK motion. Hardware faults such as
C52 and execution failures are not retried by this search. Restart the stack
after rebuilding to apply changes; no motion is started automatically by it.

## Released-item vertical retreat

After release, collision checks are omitted only for the initial empty-tool,
upward, fixed-XY/fixed-orientation retreat through the overhead waypoint.
Return geometry is generated without collision rejection, then the final timed
trajectory is checked before execution: lateral, rotating, or descending states
remain collision checked. Once the trajectory leaves the vertical column, the
exception cannot reopen. Attached-item motion never receives this exception.
Joint limits, timing checks, feedback freshness and controller handoff guards
remain enabled. This exception does not prove that the physical retreat is clear;
verify the upward corridor before running it.

## Continuous new-item transport

Combined PickAndPlace now defaults to one **lift → overhead transfer →
pre-place descent** trajectory. After contact measurement and verified suction,
pickup holds at contact (without performing its old separate retreat). The
place pipeline waits for the attached-item geometry, prepares the pallet target,
then plans and validates the complete path before moving.

- Cartesian lift/descent legs have fixed orientation. Two rounded upper corners
  lie above the configured item-bottom clearance; yaw changes smoothly on the
  elevated segment. The blend adds up to 40 mm to the clearance-level TCP height,
  plus a 2 mm geometry margin. If enabled, the independent transfer workspace
  ceiling rejects higher paths (`TRANSPORT_WORKSPACE_Z_MAX_MM`; 0 disables it).
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

## Reusable PickAndPlace routing

`safe_servo_visualization.pick_place_workflow.pick_and_place(...)` constructs
an immutable manipulation recipe shared by the incoming-item pipeline,
policy rearrangement adapter, and staging-store adapter. It does not send robot
commands itself. Source-specific adapters still resolve/verify the grasp pose,
item dimensions, obstacle identity, and recorded item/TCP transform.

```python
recipe = pick_and_place(
    source="slot",                 # incoming / slot / pallet / carried
    destination="pallet",          # pallet / slot
    pre_pick_pose=known_source_record,
    return_to_observation=False,
)
```

Known slot/pallet sources must supply a source record; they must not fall back
to estimating a different incoming item. Incoming sources use the existing
observation/object-info flow. Pack, unpack, and repack retain their physical
commit bookkeeping: consuming a slot follows a confirmed pickup/attachment;
registering a slot or pallet placement follows the confirmed release/retreat.

The pure `transport_path.transport_waypoints()` generates the existing rounded
lift/transfer/descent path from actual grasp pose to destination pre-place.
The two upper waypoints are derived from the payload clearance, not supplied
as independent potentially inconsistent heights. Orientation interpolation,
retiming, spline validation, collision checks, controller ownership, mode
confirmation and settling stay in the shared supervisor. Short empty-tool
handoffs use `vertical_retreat_waypoints()` with fixed XY/orientation.

### Chained operations

- `/pickup_supervisor/start_for_transport`: guarded pickup of a verified known
  item, then hold at contact after vacuum/attachment confirmation; defer lift
  to the shared carry trajectory. Ordinary `/pickup_supervisor/start` retains
  its pickup-and-lift behavior.
- `/staging_slots/retrieve_chained`: overhead approach, refreshed pre-pick
  target, settled contact pickup, and hold. No observation detour. Ordinary
  `/staging_slots/retrieve` retains standalone pickup/return behavior.
- `/place_pipeline/start_continuous_chained` and
  `/pick_place_pipeline/start_chained`: after release, slowly clear to pre-place,
  then rise vertically to the saved elevated transfer clearance and confirm
  arrival. `False` means no observation, **not** no retreat.
- Staging store uses the same continuous carry executor with a
  `staging_store` destination, acknowledges the prepared coordinator operation,
  and only then arms the existing staging safe-servo placement. `store_chained`
  ends at the elevated handoff; `store` returns to observation.
- Real MCTS/A* intermediate operations use chained routes. Only the final
  operation returns to observation. Slot-to-pallet and pallet-to-pallet moves
  transfer directly from the actual grasp pose; no perception/observation stop.

Staging clearance is at least its configured lift height and the configured
container height plus 20 mm (or a larger reported transfer margin). Startup
also passes the random-loading clearance to staging; loading-node status can
raise it for taller policy containers. Clearance is not lowered during a run.
The empty handoff conservatively retains the preceding carried-item TCP
transfer height; its robot/tool geometry remains collision checked.

A failed continuous staging plan retains the item and faults. It does not
invoke the old direct-to-waypoint transfer from a low grasp pose. Existing
checked staged **empty return** fallback, C52 stop behavior, and fresh telemetry
gates remain in effect. The working incoming-item trajectory and speed/timing
defaults are unchanged by this extraction.

Validate each new chained route at reduced speed before unattended operation.
No hardware execution is performed by the offline regression suite. Restart
only with the robot stopped and reconcile physical buffer/pallet occupancy
with the in-memory records after a restart.

### Known-item pickup path (`pick_waypoints`)

Known-item approach is now separate from carried-item transport. Pallet unpack,
pallet repack pickup, and slot retrieval all use the shared empty-tool path:

```text
actual current TCP (possibly touching the measured incoming box)
  -> vertical lift above current XY
  -> overhead transfer above target XY
  -> vertical descent to known pre-pick (not contact)
  -> verified completion / telemetry settling
  -> existing safe-servo contact pickup
```

`transport_path.pick_waypoints(current_xyz, current_q, pre_pick_xyz,
pre_pick_q, clearance_z, radius=0.04, step=0.005)` is a pure geometry function.
Positions and `clearance_z` use metres in `link_base`, not pallet-relative Z.
The clearance input comes from the configured container/approach height plus
the pallet transform. The generated upper level also clears both endpoint
heights. Pallet approach targets are raised above a high item's pre-pick when
necessary. Rounded corners stay above clearance; orientation changes occur
overhead, with fixed orientation on the initial lift and final descent.
Close/coincident XY targets use vertical/overhead segments rather than skipping
clearance. The optional transfer ceiling applies to this planned approach too;
kinematic and collision checks remain mandatory with or without a ceiling.

`/motion_coordinator/prepare_pick_waypoints` prepares the known source, and
`/pickup_supervisor/start_pick_waypoints` executes its full approach using the
existing Cartesian planning, collision/FK validation and smooth retiming.
`PickPathClient` shares the prepare/execute/acknowledge handshake between policy
and staging adapters. `/motion_coordinator/accept_pick_waypoints` requires a
fresh supervisor completion for the exact target operation before exposing a
successful pre-grasp snapshot. Final measured FK checks both position and
orientation. Only after acknowledgement and telemetry settling may contact
pickup begin. No direct-to-overhead free-space fallback bypasses the lift.

The deliberate allowed start state is `AWAITING_GRASP`: if MCTS requires unpack
while the empty TCP is still touching the incoming object, the approach owns
the departure. It invalidates immediate-grasp object information and its marker,
without opening/closing the gripper or taking the old discard/observation detour.
When the incoming item is eventually packed, normal observation/estimation runs
again. Ordinary observation-to-new-item pickup is unchanged.

Servo must acknowledge pause; controller ownership, fresh samples, and the
existing uninterrupted post-restore settling gate are checked before planning.
Healthy mode 1 is retained. Handoff has a 30-second deadline. Partial geometry,
timing failure, unexpected approach contact, or unverified completion faults
without starting pickup or invoking the placement release fallback. A helper
fault requests cancellation of its active supervisor operation; stop delivery
is not guaranteed if communications have failed. Collision checking still
depends on the scene contents: this path shape is not a substitute for modelling
packed items, the tool, and the environment. Simulation display choreography
has not been rewritten by this change; commission the real pickup routes at
reduced speed before automatic operation.

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

## Single-item hardware test panel

The **Single-item PickAndPlace tests (real robot)** group in the Safe Servo
panel tests the reusable production pickup/transport/placement paths. It is
not a simulation and does not run MCTS. The existing normal loading controls
remain available; the test does not overwrite their packing inventories.

Start with a physically empty pallet, empty slot **0** (the first slot), and
an empty gripper. Lock the pallet, reset both normal loaders to IDLE, and turn
off their automatic/continuous loading options. Use an item no larger than
250 × 250 mm in XY. Enable the deliberate-real-test checkbox.

| Button | Operation | Successful endpoint |
| --- | --- | --- |
| 1. New item → random pallet | Observation/detection/contact height estimation, then random target selection, pickup and shared loaded transport | Released item on pallet; empty tool above container |
| 2. Unpack → slot 0 → overhead | Known pallet source → `pick_waypoints()` → guarded pickup → shared transport to slot 0 → guarded placement | Item in slot 0; slow retreat to pre-place, then vertical lift above the container; no observation detour |
| 3. Slot 0 → random pallet | Saved slot pre-pick → `pick_waypoints()` → pickup → shared loaded transport directly to pallet | Released item on pallet; empty tool above container; **no observation detour while carrying** |
| 4. Repack on pallet | Known pallet source → `pick_waypoints()` → pickup → shared loaded transport to a different random corner | Released item at new pallet corner; empty tool above container |

Run 1 → 2 → 3 → 4 one button at a time. After any successful pallet placement,
2 or 4 can be used again. No step automatically starts the next button.
The shared executors retain force guards, mode/controller handshakes, settling
delays, slow post-release clearance, and checked overhead travel. No new direct
robot movement implementation is introduced by this test coordinator.

Random targets use the startup-resolved `real_platform_random.yaml` settings
(container, clearance mode/amount, stability options and seed). The test's
private planner treats the pallet as empty at the next placement because it
tracks exactly one item, which is picked up before repacking. It does **not**
commit test items into the normal random/policy models. Repack excludes both
orientations at the old virtual corner. Dimensions and source poses are saved
from confirmed operations; slot retrieval uses the existing saved slot record.
RViz placed-item markers/obstacles are removed and recreated through the usual
scene node, including visual-only items. The normal loaders' Three.js inventory
is not used for this isolated test.

The panel displays the current step, phase, elapsed time, last confirmed item
location and downstream states. A disabled button's tooltip explains the failed
precondition. `/pick_place_test/status` additionally contains the target array,
item geometry, expected operation ID, recent phases and exact fault. Logs include
`TEST TARGET` and `TEST CHECKPOINT` records for reproduction. Startup is inert;
there is no automatic resume after a process/container restart.

**Abort test** requests cancellation of coordinators and motion executors and
latches a fault; it does not open the gripper, retreat, or automatically retry.
A stop request is not confirmation of a physical stop. Use the robot's emergency
stop if needed. After any interrupted operation, the physical item may no longer
be at the last confirmed location: inspect and reconcile it before recovery.

**Reset test bookkeeping** does not move the robot, release an item, delete
obstacles, or erase occupied slots. First stop/reset faulted executors using the
existing controls and physically clear the test item, reconciling its scene and
slot records. Reset is rejected while an item is attached, a test/untracked pallet item remains,
slot 0 is occupied, or another operation/planning worker is still active.
Do not operate the normal loaders or manual motion controls in parallel with a
test; the panel disables conflicting groups during a step, and the backend
rejects stale/busy state and competing loading targets.

The coordinator starts with `start_unified.sh`; rebuild
`safe_servo_visualization` and `safe_servo_rviz_panel`, then restart the stopped
stack/RViz to load changes. Services under `/pick_place_test/` are `pack_new`,
`unpack`, `pack_slot`, `repack`, `abort`, and `reset` (all `std_srvs/srv/Trigger`).

The unpack test uses `/staging_slots/store_chained`. Slot occupancy and the recorded
release TCP are committed once the matching placement operation has a release pose
and the scene confirms a new detached item, before retreat completes. A later
retreat fault retains that inventory but does not complete the test or authorize
another motion. Reconcile the physical state before resetting after any fault.
Calling the four step services directly also commands real hardware.

Pickup Servo workspace selection follows the validated source snapshot:
incoming uses `workspace_*_mm`, pallet retrieval uses the placement Z floor
(`place_workspace_z_min_mm`), and buffer retrieval uses `staging_workspace_*_mm`.
Both contact-search validation and the configuration sent to Servo use this same
selection. Force thresholds, speed and maximum descent are unchanged. Before
arming, the supervisor waits for the bridge to acknowledge the expected Z bounds
as well as speed/force settings. A pallet below the base plane therefore does not
inherit the incoming-table +50 mm floor. Retrieval messages optionally append a
14th field: `0` for pallet or `1` for buffer; legacy messages retain the pallet
retrieval floor. Restart the updated supervisor and Servo bridge together.

### Carried-item transfer alternatives

For **pallet → slot** and **slot → pallet**, the primary route is:

`pickup → vertical lift → above observation XY → above destination → pre-place`

The observation-side transfer waypoint uses a configurable offset in `link_base`:
`(observation_x, observation_y + 0.200 m, safe_transfer_z)` by default.
Set `TRANSPORT_OBSERVATION_Y_OFFSET_MM=200.0` in `.env`; `0.0` restores the
unshifted waypoint. Only the transfer waypoint is shifted, not the saved
observation pose, clearance height, target slot or pallet pose. The same offset
applies in both directions and to the observation-side fallback for other loaded
transfers. Changed `.env` values require recreating the container with
`docker compose up -d --force-recreate ros2_cv` while the robot is safely stopped.

It does not try the direct cross-body route first. If the complete waypoint
route fails validation, MoveIt overhead planning remains the **last fallback**.
The source is taken from the saved pickup snapshot; storing into a staging slot
also selects this route. New-item loading and pallet-to-pallet repacking retain
their direct-first behavior. The available candidates are:

1. The existing smooth direct lift/transfer/descent path.
2. A vertical lift, transfer via the saved observation pose's **XY at the
   transfer height**, transfer above the destination, and vertical descent.
3. A vertical lift, **MoveIt OMPL/RRTConnect** overhead transfer, and vertical
   descent. `/plan_kinematic_path` is planning-only (5 s planning budget,
   3 attempts); it does not move the robot. The overhead leg retains tool-tilt
   and height constraints and the attached-box collision geometry.

Alternatives are attempted for partial Cartesian paths or rejected collision/
clearance/tilt checks, not for hardware errors or execution failures. The
observation route does not visit the low taught observation Z, and uses the
transport orientations, not the observation tool orientation. If its saved
waypoint is missing, the supervisor proceeds to candidate 3.

**All legs are planned before any movement**. Segment seams must match joint
positions, with smooth rest boundaries; the fallback may pause at waypoints.
The complete joined trajectory goes through the same timing, sampled collision,
FK clearance/tilt, live-state and final-pose checks as the direct path. Mode-1
trajectory-controller execution and the later Servo handoff are unchanged.
No fallback is allowed after trajectory execution starts. If no candidate
passes, fault while holding the item; do not bypass collision checking.

This applies to shared loaded transport, not empty-tool pickup/return paths.
Fallbacks are enabled by default; set `CONTINUOUS_TRANSPORT_ALTERNATIVES_ENABLED=false`
at startup to disable them. Buffer transfers still require the observation-side
route with fallbacks disabled; they fault if that route cannot be validated.
Supervisor status `transport_route_attempt` is 0 (direct), 1 (observation-side),
or 2 (MoveIt) for the loaded transport. Logs identify rejected candidates and
the next planning attempt. Offline validation is not a real-robot safety test.

### Raised pre-place fallback (pallet placement)

Known-item pickup has a corresponding bounded search: `RAISED_PRE_PICK_MAX_MM`
defaults to 30 mm (0 disables, maximum 50). If its Cartesian approach is
incomplete, it replans the entire pickup path with a pre-pick endpoint raised
in 5 mm increments. All path, collision and final-pose checks remain enabled.
The original object pose and contact-search floor are unchanged. A fresh,
target-bound handoff authorizes guarded incremental pickup only after the full
approach has succeeded. Fresh force/TCP feedback is required between steps;
at least two distinct above-threshold force samples after the minimum descent
are required before suction. Unexpected early force, timeout, stale feedback or
failed steps stop without enabling suction. After grasp, deferred-lift pallet
pickups first reuse the slow direct clearance retreat to the verified pre-pick
height (including any raised approach offset), keeping suction on. They then
restore ROS ownership, pass the existing settling gate, and verify the retreat
endpoint within 2 mm Z / 3 mm XY before allowing transfer. This uses the existing
return-clearance speed (default 10 mm/s), not a singularity-threshold bypass.
Other deferred-lift sources restore ROS ownership before transfer;
non-deferred pickup first performs the direct upward retreat. This fallback
does not apply to new-object estimation. A fresh Servo singularity fault during
known pallet/buffer pickup also enters guarded incremental recovery even when
the original approach succeeded. The current enable generation, force/TCP
telemetry, healthy hardware, pickup-column alignment and remaining travel are
checked first. Existing above-threshold contact blocks rebaselining; other
faults remain hard stops. The original floor and bounded step timeout remain.

Known pallet/buffer pickup now also monitors sustained Servo singularity
deceleration, even without a Servo FAULT. It uses the same configured debounce
as placement (normally 0.75 s) and no-progress detection, then calls the same
guarded pickup recovery. Contact takes priority; transient slowdown resets the
timer. New-item estimation/probing is excluded. A singularity-related pickup
timeout is a final recovery backstop, not permission to release or enable suction
without contact. Pause, exclusive direct-mode handoff and restoration remain unchanged.

For live known-source recovery, robot state 1 (moving) is allowed only when
requesting Servo pause. A pause acknowledgment alone cannot start direct mode:
post-ack robot, force and joint samples must be fresh, robot state must be 0/2,
and joints must stay within 0.001 rad of the stop reference for at least 0.25 s.
The pause/stop gate has a five-second deadline and is bound to the operation and
pickup source. It monitors raw force against the pre-pause baseline because the
Servo bridge clears its delta on disable. Stop verification failures request
state 3 and fault with suction off; no incremental step is sent. TCP conversion
is recaptured after stopping, before the existing exclusive mode handoff.
Diagnostics identify the actual failed interlock, including mode/state/error,
generation or stale stream. This does not suppress C52 or other hardware faults.

The test panel now records live xArm TCP release RPY (the same convention used
by staging retrieval) in each pallet-item checkpoint. Unpack/repack reuse that
tool orientation rather than substituting pallet yaw. The footprint dimensions
and object yaw remain expressed in pallet axes. Checkpoints created before this
change have no release orientation and cannot authorize pickup; create a fresh
test placement instead of guessing the old grasp. This does not disable Servo
singularity detection during pickup; only verified known-source pickup singularities
can use the guarded recovery described above.

If the last fallback's final Cartesian descent is incomplete, retry that leg
at +5, +10, ... mm above the original pre-place. `RAISED_PRE_PLACE_MAX_MM` in
`.env` defaults to 30 mm (0 disables; supported maximum 50 mm). XY, orientation,
the item placement target and packing state remain unchanged. This search is
not used for pickup, return or staging-slot placement. An incomplete path is
never executed; the full combined trajectory still undergoes timed collision,
clearance and endpoint validation. Failure of those checks remains a fault.

After successful execution and a fresh endpoint check within 2 mm, an
operation/item-bound, single-use handoff starts direct guarded incremental
descent using the existing Servo-disable/controller-deactivation/mode-switch
sequence. Steps use the existing singularity recovery settings (default 3 mm,
10 mm/s, 20-second step-phase budget). Fresh force and TCP feedback are required
between steps. The travel floor stays at original pre-place Z minus the normal
descent allowance, clamped to the workspace floor; the raised start therefore
adds travel without moving the placement target down. Contact permits release.
For this fallback, timeout, stale feedback, failed motion or reaching the floor
without contact faults and keeps suction on; a direct stop is requested if
stepping was active. There is no automatic retry after execution failure.
This does not guarantee passage through a singularity. Recreate the stopped
container to apply `.env` changes; validate at low speed before automatic use.

### Bounded overhead transfer alternatives

For pallet-to-buffer and buffer-to-pallet transfers, the observation-side route
now separates translation and orientation change. Lift with the pickup
orientation, travel to the overhead observation-side waypoint with that same
orientation, rotate there, then travel above the destination and descend.
An alternative keeps the pickup orientation until above the destination and
rotates there instead. Both rotation locations remain at the computed transport
clearance height; the saved observation pose is not changed.

There are at most six Cartesian candidates: these two rotation locations at the
configured observation-side waypoint, then variants displaced by +100 or -100 mm
along **link_base X**. These offsets are search candidates, not guaranteed safe
routes. The attached item remains in the scene. Complete timing, collision,
clearance, joint-limit and endpoint checks must all pass before any execution.
MoveIt overhead planning follows the Cartesian candidates. Its overhead path has free
orientation, but its goal retains the exact placement orientation before vertical
descent. The final spline checks every sampled rotated item corner against the
clearance plane, including at the source/destination when rotated away from the
column orientation. Low lift/descent columns retain their required orientations.
The Cartesian candidates still prefer fixed-orientation translation and overhead
rotation; they are not free-orientation searches. The alternatives share a
60-second planning/validation budget, in
addition to the existing per-candidate timeout. Disabling alternatives keeps the
first required buffer route but disables subsequent retries and fallbacks.

### Checked SDK final fallback

If all Cartesian candidates and MoveIt motion planning fail **before execution**,
`sdk_transport_fallback_enabled` (supervisor ROS parameter, default `true`) allows
one final joint-space route. It uses the same two `x_o - 150 mm` waypoint positions.
Endpoint IK and collision checking still use MoveIt; the installed xArm ROS node
does not expose SDK IK. The overhead segments use straight joint interpolation,
not OMPL or constrained Cartesian interpolation. Lift/descent geometry is still
obtained from collision-checked Cartesian planning. No rejected MoveIt trajectory
is executed. Planned transport/return paths use the independent setting
`TRANSPORT_WORKSPACE_Z_MAX_MM` in `.env` (ROS: `transport_workspace_z_max_mm`).
The current configuration is `0.0`: no artificial upper TCP height restriction
on these paths. A positive value restores a ceiling in millimetres relative to
`link_base`; the code/launch fallback default remains 800 mm. MoveIt's finite
clearance box extends 10 m above the clearance floor when disabled, beyond the
fixed-base UF arm's reach; final validation imposes no upper TCP limit in this mode.
Collision checks, physical joint limits, the lower item-clearance plane, and
separate guarded-contact and bounded direct-recovery limits remain enabled.

SDK endpoint IK angles are mapped to the nearest `q + 2*pi*k` representation
within each joint's physical limits, relative to the preceding planned state.
The measured start is never wrapped. FK and full-path checks still follow this
selection. Genuine excessive joint changes remain rejected, now logging the
joint name, raw/selected angles, delta and limit. This fixes wraparound, not every
possible IK branch or unreachable route; no rejected route is forced to execute.

The complete piecewise joint path must pass joint limits, sampled collisions,
rotated-item clearance, low-column orientation and endpoint checks before any
mode switch. Validation spacing is at most 0.01 rad per joint, bounded to 4000
states. SDK commands subdivide the checked lines to at most 0.05 rad per joint.
Sampling and feedback monitoring do not constitute a continuous collision proof.

Execution uses `/ufactory/set_servo_angle` (not Servo-J) with `wait=False`, no
blending, speed capped at 0.3 rad/s and acceleration at 0.5 rad/s² (also limited
by configured speed/scaling). This is a conservative, staged last resort, not
the fast continuous path. It pauses Servo, reuses the controller/hardware
deactivation and confirmed mode-0 ownership handoff, then monitors fresh xArm
joint reports. Each command must be accepted, stopped and within 0.003 rad of
its target before the next command. Joint-line deviation above 0.01 rad stops it.

After success, the existing mode-1/hardware/controller restoration and fresh
joint-state settling gate run, followed by fresh final FK verification. Only
then may normal guarded placement begin. Feedback loss, changed attachment or
operation, unexpected force, C52/other robot errors, Abort, a 30-second stage
timeout or 180-second overall execution timeout request state 3 and hold the
item. Faults do **not** automatically clear controller errors, resume motion,
release the item or re-enable ROS writing into an unconfirmed direct motion;
operator recovery is required. Late callbacks are invalidated. SDK planning uses
the existing shared 60-second budget. This fallback can also fail safely if no
checked route exists; validate it in supervised low-speed tests before repetition.

Each rejection logs its candidate and segment, endpoint and service result.
Incomplete Cartesian legs run bounded, planning-only IK/state-validity probes;
probe success does not prove the entire path feasible, and IK failure does not
prove collision or singularity. Discarded candidates invalidate late callbacks.
No alternate route is started after execution begins. Exhaustion or timeout
faults while retaining the item; it never authorizes an unchecked route or a
release. No changes to the existing controller/mode handoff are needed here.
First validate pack/unpack/repack individually at low speed before enabling
repetitive experiments; offline tests cannot establish physical route safety.

### X-first waypoint and connected translation legs

The two intermediate waypoints are now **replaced**, not extended. After the
clearance lift, current TCP is `(x_c,y_c,z_c)` and the destination is
`(x_d,y_d,z_d)`. With saved observation X `x_o`, the nominal waypoints are
`wp1=(x_o-0.150,y_c,z_c)` and `wp2=(x_o-0.150,y_d,z_c)` in link_base metres.
Then travel above the destination and descend. Both buffer directions use this
rule. Observation Y/Z and the old `transport_observation_y_offset_mm` do not
affect these two points. Existing bounded candidate X offsets still apply to
both waypoints together. The saved observation pose is unchanged.

Consecutive fixed-orientation translation legs are planned in one Cartesian
request, with sampled quintic rounded corners. This includes lift/X-first/
observation travel and, after rotation, destination travel/descent. If no
orientation change is needed, the entire translation can be one block.
Interior translation junctions no longer receive forced zero velocities from
joining separate plans. The existing whole-trajectory C2 retimer, collision,
clearance and endpoint checks still run before one trajectory is executed.

Corners use `CONTINUOUS_TRANSPORT_BLEND_RADIUS_M` (default 0.04 m), capped by
adjacent leg lengths and clearance above the safe transport plane. Intermediate
corners are **virtual waypoints**: the rounded path passes near, not exactly
through, them. The final endpoint and low vertical approach columns remain
unchanged. In-place overhead orientation changes and the final pre-place still
have controlled stops; reversal/degenerate geometry may also require a stop.
MoveIt fallback retains its existing segment-boundary stops. Thus this removes
unnecessary translation pauses, not all possible speed reductions or stops.
No velocity/acceleration limits or mode-switch safeguards are relaxed.

### Near-start post-release upward handoff recovery

When placement used a verified raised pre-place approach, its height is retained
for the slow post-release retreat even after the single-use descent handoff is
consumed. The record is bound to the target ID and original pre-place coordinates;
stale targets cannot reuse it. New transport and faults clear it. For example,
a verified -26.4 mm approach replaces the nominal -41.4 mm retreat height.

If the empty-tool vertical handoff returns zero or near-start Cartesian progress and its
diagnostic IK returns NO_IK_SOLUTION (-31), a bounded recovery can lift 5 mm
slowly and replan from fresh measured joints. This applies only after confirmed
detachment, with `return_to_observation=False`; it is not a fallback for reverse
observation planning, collisions, distant path failures, hardware faults or failed
execution. For nonzero progress, require at most 2% and a diagnostic probe in
link_base at most 15 mm above and 3 mm sideways from the start. The failed
partial trajectory is never executed. Up to six steps / 30 mm are permitted, with a 45-second recovery
deadline. No step may exceed the existing overhead goal or workspace ceiling.

Each step pauses Servo, deactivates ROS writers, confirms direct mode, reuses
the slow clearance retreat, restores ROS control and waits through the normal
fresh-feedback/settling gate. Force changes at the placement threshold, stale
force/robot feedback, changed operation/pallet/attachment, or timeout stop the
recovery; active recovery faults request state 3. Z must verify within 2 mm and
XY within 3 mm before replanning. This is a bounded direct-service recovery,
not proof that a singularity is safe to traverse. Abort invalidates callbacks.

The test coordinator records a pallet placement when detach and exactly one new
scene item are confirmed, even if retreat subsequently faults. Release RPY is
retained when available. The operation remains faulted, does not increment the
random test success count, and cannot automatically continue. Missing or
ambiguous inventory is not guessed. Reconcile actual inventory before reset.

### Randomized single-item robustness test

The checkbox **Random test: pack / unpack only (no repack)** excludes `repack`
from automatic choices. It starts with `pack_new` if no item is tracked, then
alternates unpack to a random empty slot and pack from the recorded slot to a
random pallet target. Manual buttons are unchanged. Change the checkbox only
while idle; stop scheduling and finish the current step first. The coordinator
confirms the setting through `/pick_place_test/set_random_pack_unpack_only`
(`std_srvs/SetBool`) and publishes `random_pack_unpack_only` in its status.
It defaults to unchecked on coordinator restart; toggling it does not start motion.

The test panel also offers **Start random robustness test** and **Stop random
test after current step**. Enable the existing real-robot confirmation checkbox
before starting. This is real execution, not simulation. Start with one new
item and an empty pallet, or a confirmed item checkpoint owned by this test.
Other loaders must remain idle with automatic operation disabled.

The scheduler uses the confirmed inventory:

- No tracked item: estimate and pack the new item at a random pallet target.
- Item on pallet: uniformly select an available `unpack` or `repack` operation.
  Unpack chooses uniformly among empty slots 0–5. Repack requests a different
  random pallet target using the existing loader.
- Item in a slot: retrieve from that **recorded slot**, then pack onto the pallet.

This reuses one item; it does not fill the pallet with multiple incoming items.
The manual buttons now refer to the test/recorded slot (initially slot 0).
After a random run, they continue using the last selected slot, not necessarily
slot 0. Other occupied slots are neither selected for storage nor erased.
Each step still uses the existing production motion, force and mode-handoff
logic. Inventory availability is not a guarantee of kinematic feasibility.

`.env` settings (recreate the stopped container to apply):

```dotenv
PICK_PLACE_TEST_MAX_STEPS=100
PICK_PLACE_TEST_SEED=-1
```

The limit counts successfully completed operations, including the initial pack;
allowed range is 1–10000. Seed -1 chooses a fresh seed per run. A fixed seed
repeats operation/slot choices only when the same choices remain available;
perception, motion planning and pallet-target sampling are not replayed by this
seed. The actual seed, choices, targets, per-operation success counts and final
checkpoints are logged. Status exposes progress and the current test slot.

The next step starts after a two-second checkpoint interval, with fresh status
and interlock checks. Manual test commands/reset are blocked while automatic
scheduling is enabled. Stop-after-current disables scheduling without cancelling
the active step; **Abort test** requests cancellation and keeps gripper state.
Any fault, stale dependency, inventory mismatch or absence of valid operations
ends the run. There is no automatic fault reset, retry, release or restart on
node/container startup. Reconcile physical inventory before resetting a fault.
Validate individual operations and all intended slots at low speed before a
supervised randomized run; no real motion is performed by the offline tests.
# Policy + MCTS/A* execution wiring

Policy item-bottom transfer height is derived from its own YAML:
`container_size[2] + transfer_clearance_mm`, with a minimum/default margin of
20 mm. A 450 mm policy container uses 470 mm above the pallet, not a fixed TCP
height. Random loading retains its own configured height. Target field 16
carries this height in metres; accepted pallet config_state field 19 forwards
it to both motion consumers. Legacy targets restore the startup default.

Failed overhead routes retry the same XY candidate at successively lower heights:
10 mm steps through the full 100 mm below nominal, without clipping candidate
generation at the safe TCP plane. Clearance remains unchanged: candidates below
required clearance are still rejected before execution. Trajectories are replanned from the
original seed and revalidated within the existing overall search deadline.
`transport_lower_waypoint_search_enabled` defaults to true. Pre-place raising is
reserved for request intervals conservatively classified as final vertical descent,
not overhead travel. The fraction-based classification is not an exact failed
internal sample. Starts are blocked when policy inventory is empty but placed
scene items remain; no automatic inventory deletion or reconstruction is attempted.

The policy panel reuses the tested executors: incoming pack uses PickAndPlace;
buffer pack uses chained retrieval then placement; unpack uses the shared pickup
path then staging storage; repack uses the shared pickup path then placement.
Every policy unpack uses the returning staging store service: release into the
slot, perform the existing clearance retreat, and return to observation. The next
planner operation waits for that return to succeed; pack/repack chaining is unchanged.
The policy and test panel share physical pallet record/target generation.
Successful placement records physical dimensions, position and release TCP RPY;
later pickup does not invent a default tool yaw. Existing slot grasp transforms
and placement rotation handling remain unchanged. Planner steps commit only
after execution succeeds; faults pause the sequence. Missing release orientation
requires inventory reconciliation. Records are in-memory, not restart recovery.

### Buffer-slot inspection before retrieval

Buffer-source approaches (both inspection and corrected pre-pick) use the same
observation-side rail geometry as pallet-to-slot carrying: lift above clearance,
then `(observation_x - 0.150, current_y, high_z)`, then
`(observation_x - 0.150, target_y, high_z)`, above target, and descend vertically.
Observation XYZ is obtained from taught-joint FK. Horizontal corners may be
rounded; any orientation change occurs at the elevated second via point.
Inspection preflight and the real executor share this geometry. Pallet-source
and incoming pickup routes are unchanged. This reuses the route geometry, not
the loaded-transfer candidate search or its fallback policy.

Inspection first preflights the complete collision-checked pickup-waypoint path
without motion. If infeasible, it tries one yaw-180 alternative with unchanged
roll/pitch and observation Z, recomputing the XY camera offset. The alternative
does not require joint 6 to move closer to zero. Finite-angle checks and the
intermediate excursion guard (peak absolute angle no more than the larger
start/end magnitude plus 0.05 rad) remain. The usual
executor still replans/validates before moving; a runtime fault is not automatically
cleared or retried. A failed alternative leaves the item in its slot.
For a flipped pickup, preserve the original stored release record and re-express
both item-to-TCP translation and rotation at the same contact point. The physical
object yaw is not changed by changing the empty gripper's yaw.
Test-panel pallet bookkeeping requires the current placement phase/operation and
an observed attachment in this sequence; an inspection fault cannot commit an old
slot release as a new pallet placement.

Both standalone and chained slot retrieval now first inspect the stored item.
Copy the recorded retrieval TCP XY/orientation, use the taught observation's TCP
height (computed with FK for `link_tcp`, not the stored `link_eef` height), then
translate opposite the calibrated EEF-to-camera mounting translation projected
onto base-frame XY (normalized to the configured distance). Z remains exactly
at the observation TCP height minus 30 mm; this is not an optical-axis displacement. The default backoff
is 100 mm. Configure `.env` `STAGING_SLOT_INSPECTION_BACKOFF_M=0.100`;
`STAGING_SLOT_INSPECTION_ENABLED=false` explicitly restores recorded-pose retrieval.
These settings are passed through Compose and require recreating/restarting the stack.

The existing guarded known-item waypoint executor performs the inspection move;
the target's scene obstacle remains until inspection succeeds. At the settled
inspection pose, collect 20 distinct fresh aligned-depth frames with timestamped
camera TF. Crop to the selected slot and the recorded top height ±25 mm, check
the top is horizontal and the measured footprint agrees with saved dimensions,
and require position spread ≤6 mm and yaw spread ≤3°. The padded expected item
must fit in the camera image. Corrections exceeding 40 mm XY or 20° yaw are rejected.
Rectangle-axis ambiguity is resolved against saved dimensions and yaw; near-square
objects use the nearest recorded orientation, not semantic face recognition.

Update the recorded contact TCP and object yaw from the estimated top center/yaw,
preserving dimensions, identity and the item-to-TCP grasp transform. Then remove
the target obstacle and run the existing pre-pick approach and force-contact pickup.
No gripper action occurs during inspection. Unstable/rejected measurements keep
waiting at the inspection pose, with a warning every 15 seconds (the legacy
`slot_inspection_timeout_sec` now sets this reminder interval). Staging and test-panel
phase timeouts pause during this measurement wait; hardware faults, stale status,
planning timeouts and Abort remain active. Twenty valid stable frames automatically
resume retrieval: adjust lighting from outside the robot workspace; stop the robot
before touching the item or entering its workspace. There is no old-pose fallback.
This uses the aligned RealSense depth topics;
it does not change the new-item estimator's ROI or height offset settings.

### Two-item random test

Cross-area fallback: inspection preflight and empty-tool pickup approaches now
reuse the ±150 mm XY grid (15 mm spacing) around the observation-side rail.
Candidates retain the required final position/orientation and change only the
intermediate XY route and overhead rotation location. The empty-tool fallback
uses existing height when already clear instead of adding a rounded-path lift.
Inspection has a 5-second search budget per orientation; the executor independently
replans and validates with a 15-second grid budget. Partial paths are never executed.
Loaded transfers retain their existing bounded grid/height search and item-bottom
clearance checks. No search retries a failed physical execution or hardware fault.

Policy unpack operations now use the same chained store as the test: slow release
retreat, then vertical safe-height handoff, without returning to observation.
Empty-tool approaches select the rail using current FK position and destination
area: pallet-to-pallet and buffer-to-buffer omit it; buffer/pallet crossings use it.
The staging area is the fixed base-frame table X [-375,375], Y [180,680] mm,
expanded by 105 mm for the 100 mm inspection backoff and tracking tolerance.
Update this classification if the table layout changes. All approaches retain
safe-height lift, overhead rotation, descent and full path validation.
Cross-area approaches and carried slot/pallet transfers retain the observation-side
rail (observation X minus 150 mm), not a stop at the taught observation pose.

Slot-store transfer now allows five seconds of unresolved original-orientation
planning, then retries once with yaw +180 degrees (roll/pitch unchanged), rotating
at the high observation-side elbow. This is a planning retry, never a live rotation
command. A complete path already undergoing validation is not interrupted. The
flipped TCP XY is recomputed from attached-item geometry to preserve the slot corner;
the actual release pose/orientation remains the retrieval record. Collision,
clearance, retiming and execution checks remain active. Failed flipped searches
retain the existing bounded planning timeout and fault without releasing the item.

In the test panel, enable **Two-item floor-only test (no stacking)** before starting
with an empty pallet and reconciled inventory. Present the first new item and press
**Start random test**. After its placement the scheduler pauses; present the second
new item and press Start again. After the second placement it randomly selects a
valid unpack, pack-from-slot or repack operation, then an eligible item/slot.
Leave the pack/unpack-only checkbox unchecked to include repacking.

Both items may be on the pallet, in slots, or split between them. When both are in
slots, only packing from a slot is eligible. Pallet targets always have virtual
bottom Z=0 and cannot overlap the other item's clearance-padded footprint. This
test uses `real_platform_random.yaml`, not the policy/MCTS configuration. It reuses
the existing motion and slot-inspection executors; no additional items are sampled
after the first two. Manual single-item step buttons are disabled in this mode.

The panel reports both item locations. Stop random scheduling lets the current
operation finish; Abort retains its existing stop behavior. Faults remain latched
and do not silently change inventory. Before resetting or changing item mode,
physically reconcile both items and clear their corresponding scene/slot records.
Rebuild the RViz panel and restart the test node with the updated ROS and Neuromeka
sources before testing. This orchestration has offline tests, not hardware validation.

### Outbound Cartesian timing fallback

For an ordinary outbound transport whose Cartesian geometry is 100% complete but
whose MoveIt timing is invalid, one planning-only staged retry is allowed before
execution. The original sampled poses, rounded corners and orientations are kept;
the path is split at its overhead entry/exit and timed separately with rest seams.
Each segment uses the previous segment's joint endpoint and verifies endpoint FK.
All segments must finish planning before the joined trajectory enters the existing
full timing, joint-limit, collision and clearance validation and JTC execution.
Normal continuous transport and subsequent Servo descent are unchanged.

This narrow fallback does not replace cross-area search, inspection, cache
connectors, SDK routes, or return/pick recovery. It never retries incomplete
geometry or hardware/execution errors. A failed staged plan or timeout faults
without automatic release. Look for `planning staged transport timing once`.
Rebuild/restart the affected node while the robot is stopped; only offline tests
have been performed.
