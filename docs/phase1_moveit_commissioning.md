# Phase 1: MoveIt Commissioning

This phase verifies the official UF850 MoveIt planning stack and real trajectory controller in isolation. It does not start perception, the force publisher, or safe servo.

## Safety boundary

The MoveIt real-move launch creates its own robot hardware connection. Never run it at the same time as `start_all.sh`, `xarm_driver_node`, or a Python SDK motion process.

Before enabling motion:

- Clear the robot workspace and keep the emergency stop accessible.
- Confirm the selected UF850 model and vacuum-gripper model match the robot.
- Start with no payload and 10% velocity/acceleration scaling.
- Inspect every trajectory in RViz before pressing **Execute**.
- Use a small joint displacement from the current position for the first test.

The current Phase 1 model includes UFACTORY's vacuum gripper, but not yet the project's complete camera/force-sensor adapter geometry or world collision objects. Do not treat a successful plan as production-safe until Phase 5 completes the planning scene.

## Build

From `src/ros2_xarm_api` on the host:

```bash
docker compose build ros2_cv
```

Confirm the image contains the expected packages:

```bash
docker compose run --rm --no-deps ros2_cv bash -lc \
  'ros2 pkg prefix xarm_planner && ros2 pkg prefix xarm_moveit_config && ros2 pkg prefix xarm_controller'
```

## Start the isolated planning stack

Make sure the normal application container is stopped, then run:

```bash
docker compose down
docker compose run --rm --no-deps ros2_cv \
  bash /workspace/start_moveit_phase1.sh
```

The launch should bring up the hardware interface, trajectory controller, MoveIt `move_group`, the xArm planner service node, and RViz.

## Read-only verification

In another terminal:

```bash
docker exec -it ros2_xarm_api-ros2_cv-run-1 bash
```

The generated `compose run` container name can differ. If necessary, obtain it with `docker ps` and use that name. Then check:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/xarm_ws/install/setup.bash

ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
ros2 node list
ros2 topic hz /joint_states
ros2 service list | grep xarm_.*_plan
```

Expected results:

- The UF850 trajectory controller is `active`.
- A `follow_joint_trajectory` action is available.
- `/joint_states` updates continuously.
- `move_group` and the xArm planner node are present.
- Joint, pose, straight-line, and execution planning services are present.

Do not proceed if joint states are stale, the controller is inactive, the displayed robot does not match the physical posture, or MoveIt reports a model/controller mismatch.

## First motion test

Use RViz for the first test because it separates planning from execution and displays the complete trajectory:

1. Select the UF850 planning group.
2. Set velocity and acceleration scaling to no more than `0.10`.
3. Use the interactive marker or joint sliders to request a very small change from the current posture.
4. Press **Plan**, but not **Plan & Execute**.
5. Replay the planned trajectory and inspect all links for self-collision and unexpected elbow/wrist motion.
6. Confirm the physical workspace is clear.
7. Press **Execute** while ready to use the emergency stop.
8. Confirm the result reports success and `/joint_states` agrees with the final displayed state.

Repeat using a manually selected joint target that remains comfortably inside all joint limits. Do not test an observation pose until its complete swept path has been inspected.

## Phase 1 completion record

Phase 1 is complete only when all items are recorded as passing:

- [ ] Image builds with `xarm_planner`, `xarm_moveit_config`, and `xarm_controller`.
- [ ] UF850 model, DOF, namespaces, and vacuum tool are correct.
- [ ] Real joint states match the RViz posture.
- [ ] Trajectory controller is active.
- [ ] A small joint-space plan is collision-free in RViz.
- [ ] The plan executes at reduced scaling and reports success.
- [ ] Cancellation or emergency stop halts the test safely.
- [ ] No competing xArm driver or safe-servo process is running.

Record the tested robot IP, xArm ROS 2 commit, firmware version, joint target, scaling values, and result below before starting Phase 2.
