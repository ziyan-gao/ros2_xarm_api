# Phase 2: Taught Waypoint Storage

Phase 2 adds persistent, operator-taught joint configurations for:

- `observation`: the item observation configuration.
- `intermediate`: an optional manually commissioned transit configuration.

The waypoint store observes the robot but never sends a motion command.

## Stored data

The default file is:

```text
/workspace/config/taught_waypoints.yaml
```

Because `/workspace` is the bind-mounted project directory, the file persists
when the container is replaced. The YAML contains a format version, robot type,
ordered joint names, positions in radians, UTC save time, and the corresponding
`link_base -> link_eef` TCP pose.

Saving an existing name intentionally overwrites that waypoint using an atomic
file replacement. A failed write cannot leave a partially written target file.

## Save requirements

A save request is rejected unless:

- All six `joint1` through `joint6` positions are present and finite.
- Joint feedback is no more than 0.5 seconds old.
- The robot has remained below 0.01 rad/s for the validation window.
- The `link_base -> link_eef` transform is available and finite.
- Any existing waypoint file has the supported format.

The stationary check uses both reported joint velocity and position changes, so
it remains effective when the driver omits the velocity array.

## RViz operation

Restart the isolated MoveIt stack so the new package and panel are rebuilt:

```bash
/workspace/start_moveit_phase1.sh
```

In RViz, open **Panels -> Add New Panel** and select
`safe_servo_rviz_panel/SafeServoPanel` if it is not already visible. Use the
**Taught motion waypoints** section at the bottom of the panel.

To teach a waypoint:

1. Move the robot to the desired configuration using a collision-checked,
   reduced-speed MoveIt plan.
2. Wait until trajectory execution has completed and the robot is stationary.
3. Visually confirm the physical posture and clearance.
4. Click **Save observation** or **Save intermediate**.
5. Confirm the panel reports the saved file path.
6. Click **Reload waypoint file** and confirm both saved names are listed.

The same services can be checked from a terminal:

```bash
ros2 service call /taught_waypoints/save_observation std_srvs/srv/Trigger '{}'
ros2 service call /taught_waypoints/save_intermediate std_srvs/srv/Trigger '{}'
ros2 service call /taught_waypoints/reload std_srvs/srv/Trigger '{}'
```

## Validation

- [x] ROS packages compile.
- [x] A stationary synthetic UF850 joint state and TCP transform can be saved.
- [x] The saved YAML reloads successfully.
- [x] A moving synthetic robot state is rejected without creating a file.
- [x] Observation configuration saved from the real robot.
- [x] Intermediate configuration saved from the real robot.
- [ ] Both configurations persist after a container restart.
- [ ] RViz and physical TCP postures agree for both saved configurations.

Phase 2 is complete after the four real-robot checks above pass.
