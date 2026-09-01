# Unified MoveIt Startup

The default Compose startup now runs the camera and perception application with
MoveIt as the sole owner of the UF850 robot connection and trajectory
controller.

## Start

From the project directory:

```bash
docker compose up
```

The default `START_MODE=unified` runs:

- UF850 MoveIt, `ros2_control`, `move_group`, the planner service, and RViz
- RealSense with aligned depth
- the calibrated end-effector-to-camera TF and visualization markers
- taught-waypoint storage and the MoveIt motion coordinator
- marker detection, depth refinement, pallet localization, and item localization

RViz is started by the MoveIt launch using
`/workspace/ws/src/rviz_config/rviz_scene.rviz`. Override `RVIZ_CONFIG` only if
you intentionally want a different saved layout.

The pinned xArm planner is patched to publish each successful stored plan on
`/display_planned_path`. The MotionPlanning display uses this topic to animate
the planned robot path before execution. The preview loops in cyan and retains
a lightweight sampled trail so it remains visible until another plan replaces
it without rendering every trajectory state as a full robot mesh.

The force-torque sensor is enabled and zeroed before MoveIt connects. Force
markers will appear only if the MoveIt hardware stack publishes
`/ufactory/uf_ftsensor_ext_states`; the old continuous force publisher is not
started because it opens a second direct SDK connection.

The real-move launch is patched to spawn ros2_control's
`joint_state_broadcaster`. The pinned hardware plugin intentionally suppresses
the old `/ufactory/joint_states` driver topic, so feeding that topic into
`joint_state_publisher` leaves RViz and TF stale after real motion. The standard
`/joint_states` output is the authoritative state for MoveIt, RViz, TF, and
waypoint storage.

## Motion ownership

The legacy `xarm_driver_node` and direct-SDK `safe_servo_controller` are not
started in unified mode. Running them alongside the MoveIt hardware interface
would create competing owners of the physical robot. Cartesian safe-servo
motion must therefore wait for a controller handoff or a MoveIt Servo based
implementation.

Do not run `start_all.sh`, `start_moveit_phase1.sh`, or another xArm driver in a
second terminal while unified mode is active.

## Alternate modes

Modes are mutually exclusive and are intended for commissioning or rollback:

```bash
# Isolated MoveIt commissioning stack
START_MODE=moveit docker compose up

# Original direct-SDK stack (no MoveIt)
START_MODE=legacy docker compose up
```

The launchers remain directly callable inside the container as
`/workspace/start_unified.sh`, `/workspace/start_moveit_phase1.sh`, and
`/workspace/start_all.sh`.
