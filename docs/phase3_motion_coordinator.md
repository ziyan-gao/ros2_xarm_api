# Phase 3: Motion Coordinator Foundation

Phase 3 introduces a stateful front end for named, collision-checked MoveIt
motions. It loads Phase 2 waypoints and separates planning from execution.

## State model

```text
IDLE -> PLANNING -> PLANNED -> EXECUTING -> SUCCEEDED
          |            |           |
          +------------+-----------+-> CANCELING -> IDLE or PAUSED

Any validation, planning, execution, or cancellation failure -> FAULT
PAUSED -> resume -> PLANNING
FAULT/PAUSED/SUCCEEDED -> reset -> IDLE
```

Resume always replans from the robot's actual stopped state. It never resumes a
partially executed trajectory.

Status is published as compact JSON on `/motion_coordinator/status`, including
the state, target name, fault, operation ID, and dependency availability.

## Services

```text
/motion_coordinator/plan_observation   std_srvs/srv/Trigger
/motion_coordinator/plan_intermediate  std_srvs/srv/Trigger
/motion_coordinator/execute            std_srvs/srv/Trigger
/motion_coordinator/cancel             std_srvs/srv/Trigger
/motion_coordinator/pause              std_srvs/srv/SetBool
/motion_coordinator/resume             std_srvs/srv/Trigger
/motion_coordinator/reset              std_srvs/srv/Trigger
```

Planning requests reload and validate the YAML on every call. Execution is
accepted only from `PLANNED`. The coordinator uses the standard ROS 2 action
cancellation service on `uf850_traj_controller`; cancellation applies to every
active goal on that controller to ensure a competing trajectory cannot remain
active.

## Commissioning safety

UFACTORY's planner defaults to a fixed velocity scale. The Docker build applies
`patches/xarm_planner_low_speed.patch` as a 0.10 startup fallback and
`patches/xarm_planner_runtime_speed.patch` so the panel's unified non-servo
speed slider can update MoveIt's velocity scaling before planning. The slider
is capped at 0.30, the upstream vendor default; acceleration scaling remains at
the conservative 0.10 limit. The same slider controls local direct Cartesian
service velocity but does not alter safe-servo descent speed.

Rebuild the image before testing Phase 3:

```bash
docker compose build ros2_cv
```

Then stop any existing stack and restart the isolated MoveIt stack. Do not run
`start_all.sh` concurrently.

## RViz workflow

The **MoveIt waypoint motion** section provides:

- **Plan observation**
- **Plan intermediate**
- **Execute latest plan**
- **Cancel motion**
- **Reset motion coordinator**

For the first real test:

1. Plan observation.
2. Wait for the JSON state to become `PLANNED`.
3. Inspect the complete trajectory in RViz.
4. Execute while ready at the emergency stop.
5. Confirm `SUCCEEDED` and matching physical/RViz posture.
6. Repeat for intermediate.
7. Plan a deliberately longer safe return, execute it, then press **Cancel
   motion** early enough to verify a controlled stop.
8. Reset and replan to the intended destination from the stopped state.

Do not use the panel's safe-servo execution controls while MoveIt owns the
trajectory controller.

## UF850 planning ranges

The Docker build applies the commissioned UF850 hardware ranges from
`patches/xarm_uf850_hardware_joint_limits.patch` to both the limited robot
description used by MoveIt and the ros2_control command model:

```text
joint1: -360 .. +360 deg
joint2: -132 .. +132 deg
joint3: -242 .. +3.5 deg
joint4: -360 .. +360 deg
joint5: -124 .. +124 deg
joint6: -360 .. +360 deg
```

This avoids rejecting hardware-valid start or goal IK states at the narrower
upstream `±0.99*pi` limits. RViz, MoveIt, ros2_control, and the supervisor's
post-retreat joint-6 check use matching ranges.

## Validation

- [x] Coordinator and RViz plugin compile.
- [x] Missing planner and invalid-state requests are rejected without motion.
- [x] Waypoint YAML validation is implemented.
- [x] Planning and execution are separate operator actions.
- [x] Pause semantics require replanning rather than stale continuation.
- [ ] Observation plan inspected and executed through the coordinator.
- [ ] Intermediate plan inspected and executed through the coordinator.
- [ ] Cancel stops a real reduced-speed test trajectory safely.
- [ ] Reset/replan succeeds from the canceled state.
- [ ] Status and faults remain deterministic through all tests.

Phase 3 is complete after all real-robot checks pass.
