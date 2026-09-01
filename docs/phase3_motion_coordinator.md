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

UFACTORY's planner source sets a fixed velocity scale of 0.30, which is
independent of the RViz scaling control. The Docker build applies
`patches/xarm_planner_low_speed.patch` to set both planner velocity and
acceleration scaling to 0.10 during commissioning.

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

## UF850 joint4 planning range

The upstream limited UF850 model sets joint4 to `±0.99*pi`, while the
underlying UF850 joint definition supports `±2*pi`. A real state close to
`-pi` can therefore be valid for the hardware but rejected by MoveIt's Jazzy
`CheckStartStateBounds` adapter.

The Docker build applies `patches/xarm_uf850_joint4_pi_limit.patch` to use
`±pi` for joint4 in both the limited robot description and ros2_control
command model. The planning range remains substantially inside the underlying
`±2*pi` definition. All other joint limits remain unchanged, and both RViz
and the application planner receive the same model.

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
