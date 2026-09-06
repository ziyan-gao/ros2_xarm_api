# Phase 4A: MoveIt Servo Foundation

The unified stack now includes MoveIt Servo without opening a second xArm SDK
connection. Servo publishes `JointTrajectory` commands to the existing
`uf850_traj_controller`, keeping all robot commands under `ros2_control`.

## Current safety boundary

The unified real-robot stack runs the guarded Servo bridge in hardware mode:

```text
MOVEIT_SERVO_DRY_RUN=false
```

The guarded bridge accepts the existing panel interfaces but permits only
vertical Z movement in `link_base`. X, Y, and orientation fields are ignored.
It also requires:

- a fresh `link_base` to `link_tcp` transform;
- the requested Z target to remain inside the configured workspace;
- the current Z to be inside that workspace before arming;
- each enable cycle to limit the target to 10 mm from its enable-time Z;
- no active FollowJointTrajectory goal;
- the motion coordinator to be `IDLE` or `SUCCEEDED`;
- fresh target commands through the command watchdog;
- no MoveIt Servo collision, singularity, or joint-bound halt status.
- fresh external force/torque data at least every 0.6 s.

During the contact descent, reaching the panel's baseline-relative force
threshold immediately publishes a zero Twist and pauses Servo. A 0.5 Nm torque
or independent absolute-force safety limit still latches a fault.

MoveIt Servo performs self-collision, planning-scene collision, singularity,
and joint-limit checks. Pickup descent defaults to approximately 20 mm/s;
place descent uses a faster 30 mm/s profile and an independent 4 N
baseline-relative Fz contact threshold. The bridge speed ceiling can be
adjusted without restoring a panel speed control by setting
`SERVO_DESCENT_SPEED_M_S` before startup; the bridge rejects values above
50 mm/s. `SERVO_DESCENT_KP_Z` controls the vertical response and defaults to
3.0. Collision and singularity scaling may still reduce the actual speed.

The SRDF allowed-collision matrix excludes only pairs internal to the rigid
UF850 flange/force-sensor/camera-stand/vacuum assembly. Without these entries,
the 30 mm self-collision proximity threshold reports status code 4 and can
scale a valid Servo command to zero because the permanently fixed tool bodies
are necessarily close together. Collision checking against movable arm links
and planning-scene objects remains enabled.

The UF850 installation overrides the official vacuum-gripper TCP offset from
126 mm to the measured 104 mm. The macro retains 126 mm as its default, so
other xArm configurations are unaffected.

Servo publishes position-only `JointTrajectory` points. The UF850 controller's
selected interface is position and it rejects streaming trajectories whose
final point contains a nonzero velocity.

## Real-robot commissioning

Keep the physical emergency stop accessible. After controller or model changes,
recheck response, stop latency, frame direction, collision halt, singularity
halt, and action/Servo arbitration at the configured low speed.

Vacuum activation is not part of this foundation. The force stop is an
emergency contact interlock, not a validated contact-detection or grasp
criterion; it must be commissioned at low speed before use near a part.

## Phase 4B supervised pickup

The `pickup_supervisor` completes the software path from an executed pre-grasp
to a verified pickup and return to pre-grasp. It starts only when:

- the motion coordinator reports `SUCCEEDED` for a `pregrasp_box_*` target;
- the executed pre-grasp has a recent box snapshot that was validated from a
  fresh refined detection during planning;
- the live ROS TCP is aligned with that box in X/Y;
- the required descent is positive and no more than 150 mm;
- safe-servo telemetry is fresh and fault-free.

The wrist camera is not required to keep seeing the marker after it moves over
the item. The supervisor instead verifies that the live TCP matches the X/Y of
the exact snapshot associated with the successfully executed plan. Snapshots
expire after five minutes and are cleared by any unrelated waypoint plan.

The supervisor arms Servo once and continuously refreshes the measured box-top
Z target. The bridge advances downward continuously and immediately pauses when
the baseline-relative Fz increase reaches the configured threshold. Pickup uses
a 5 N default (`PICKUP_FORCE_THRESHOLD_N`), independently of the 4 N placement
threshold. Reaching the box-top safety floor without force contact is a fault.
Lateral and rotational commands remain unavailable.

At the measured box top, the supervisor calls the vacuum service on the
ros2_control hardware driver's existing connection. It requests pressure
waiting and then verifies `/ufactory/get_vacuum_gripper` before retreating.

The measured box top is not used as an exact motion endpoint. Depth and Servo
position uncertainty can otherwise stop the TCP a few millimetres before
contact. The guarded descent may continue up to 15 mm below the estimated top
(`contact_search_margin_m`) while still respecting the global maximum descent
and Servo Z floor. Force contact remains the normal stop condition.
It never launches a second xArm driver or SDK connection.

Retreat does not use MoveIt planning or segmented MoveIt Servo. After Servo is
paused, the supervisor restores robot mode/state 0 and calls the existing
`/ufactory/set_position` service once for a slow, relative base-frame +Z linear
move back to the captured pre-grasp height. Manual retreat is available
regardless of vacuum verification state. After the blocking retreat command
finishes, the supervisor restores robot mode 1 and state 0 before reporting
success. It also explicitly reactivates `uf850_traj_controller`, so later
ros2_control/MoveIt trajectories can execute normally.

An abort or fault immediately requests Servo disable. If vacuum has already
been confirmed, it is deliberately left on so a held item is not dropped.
Release belongs to the later placement phase.

Supervisor interfaces:

- Start: `/pickup_supervisor/start`
- Abort: `/pickup_supervisor/abort`
- Reset: `/pickup_supervisor/reset`
- Status: `/pickup_supervisor/status`

Commission in this order:

1. Confirm the force and vacuum services exist after the unified stack restarts.
2. Set a conservative force threshold and keep the emergency stop accessible.
3. Commission one short real continuous descent with no payload.
4. Inject stale perception, TCP misalignment, Servo fault, and abort cases.
5. Only then run the complete real descent, vacuum verification, and retreat.

Phase 4 is not physically complete until repeated supervised pickups meet the
completion criterion in the main motion plan.

## Interfaces

- Bridge enable: `/safe_servo/enable`
- Bridge reset: `/safe_servo/reset_fault`
- Target input: `/servo_command`
- Bridge status: `/safe_servo/status`
- Servo status: `/servo_node/status`
- Servo output: `/uf850_traj_controller/joint_trajectory`
