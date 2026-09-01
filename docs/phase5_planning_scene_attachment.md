# Phase 5: Planning-scene item attachment

After a real pickup confirms force contact and vacuum, the detected item is
added to MoveIt as a collision box attached to `link_tcp`. Subsequent MoveIt
plans therefore check the swept volume of both the robot and the carried item.

The pre-grasp snapshot supplies the selected box's center, dimensions and yaw.
Attachment is disabled in dry-run mode and requires both `contact_detected`
and `vacuum_verified`. Only the tool contact links are allowed to touch it;
collisions against other robot links and world objects remain active.

## Test after a real pickup

```bash
ros2 topic echo /planning_scene_obstacles/status
ros2 topic echo /monitored_planning_scene --once
```

`attached_item_id` must become non-empty, for example `carried_item_3`. In
RViz, enable the MotionPlanning scene geometry to inspect the attached box.
Plan, but do not execute, an intentionally obstructed transfer. Planning must
fail when the carried box intersects a table even if the arm itself would fit.

After the vacuum has physically released the item, detach it with:

```bash
ros2 service call /planning_scene_obstacles/detach_item std_srvs/srv/Trigger '{}'
```

Do not detach while the robot still holds the item. Phase 7 will invoke this
service as part of the placement/release sequence.
