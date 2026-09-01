# Pallet-relative place function

The placement target is a Cartesian pose relative to `pallet_frame`, not a
fixed robot joint waypoint. If the pallet pose is not `LOCKED`, planning and
the automatic place cycle are rejected.

## Persistent configuration

The pallet localization node stores configuration in:

```text
/workspace/config/pallet_place_config.yaml
```

The file includes the pallet X/Y dimensions and surface thickness, marker
localization settings, the last accepted pallet pose, and the pallet-relative
pre-place pose. Pallet dimensions and localization fields are restored into
the RViz panel when the stack restarts.

The RViz pallet panel contains editable fields for:

- pre-place position in `pallet_frame`: X, Y and Z;
- a checkbox that rotates the item 90 degrees about the pallet Z axis;
- a checkbox that preserves the current TCP roll and pitch while applying the
  pallet-relative target yaw;
- a checkbox that controls whether a released item is retained as a MoveIt
  world obstacle;
- pallet dimensions and marker-localization settings.

Linear values use millimetres and angles use degrees in the panel. The
pre-place fields are shown in the **Place cycle** group. Click
**Apply/save place target** to persist them. Use **Apply/save values** for the
pallet dimensions and localization settings. The pallet pose itself
is not exposed as six manual fields. A successful marker detection followed by
**Accept, save & lock** writes the measured `link_base -> pallet_frame` pose to
the same configuration file.

On restart, the saved values are loaded into the panel, but place remains
disabled until the pallet is explicitly determined by either:

- detecting it and clicking **Accept, save & lock**; or
- clicking **Load saved pallet & lock**.

This explicit lock prevents automatic motion from using an unnoticed stale
pallet pose.

## Real placement sequence

After pickup, click **Plan pre-place** to inspect the pallet-relative MoveIt
plan, or click **Start place** to run the complete sequence:

1. require `pallet_localization/status == LOCKED`;
2. transform the configured pre-place pose from `pallet_frame` to `link_base`;
3. plan and execute to pre-place with MoveIt and the attached item;
4. zero the force sensor and descend continuously with MoveIt Servo;
5. stop when the baseline-relative Fz increase reaches the dedicated 4 N
   placement threshold (independent of the pickup threshold shown in the
   panel);
6. disable vacuum and detach the carried collision box; if **Add placed item as
   MoveIt obstacle** is selected, add it back to the world at its measured
   release pose;
7. retreat vertically to the original pre-place height;
8. restore trajectory control and use MoveIt to return to observation.

The placement threshold defaults to 4 N and can be changed at startup with
`PLACE_FORCE_THRESHOLD_N`. The panel force field is intentionally labelled as
the pickup threshold so it cannot silently overwrite the placement value.

Monitor:

```bash
ros2 topic echo /pallet_localization/status
ros2 topic echo /pallet_localization/pre_place_pose
ros2 topic echo /place_pipeline/status
ros2 topic echo /planning_scene_obstacles/status
```

Placed boxes use the dimensions captured during pickup and receive unique IDs
(`placed_item_1`, `placed_item_2`, ...). They remain collision obstacles for
later transfers during the current locked-pallet session. Unlocking the pallet
removes the pallet surface and all associated placed-item obstacles so stale
geometry is not retained after pallet relocalization.

Both checkboxes are applied automatically before **Plan pre-place** and
**Start place**, and are persisted in `pallet_place_config.yaml`. Disabling the
placed-item option affects future releases only; it does not remove obstacles
that were already inserted earlier in the locked-pallet session.
