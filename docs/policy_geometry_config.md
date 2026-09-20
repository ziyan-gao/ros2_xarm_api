# Policy clearance and XY resolution

The real-platform policy YAML is authoritative:

```yaml
clearance: 30          # mm, current one-sided XY inflation
xy_resolution_mm: 5   # supported: 5 or 10; Z grid remains 5 mm
```

The ROS policy node copies YAML clearance into its shared `clearance_mm` parameter
and passes that YAML value to the loader. An old launch override such as
`-p clearance_mm:=20` no longer overrides YAML. The unified launcher no longer
supplies the hard-coded value.

Neuromeka's geometry/map constants are fixed when a Python process imports them.
`start_unified.sh` validates the policy YAML before starting robot components and
sets `NEUROMEKA_XY_RESOLUTION_MM` **only for the policy process**. This keeps the
height map, item geometry, EMS positions, MCTS and A* on the same grid. Other
processes, including random loading and training, retain the default 10 mm grid.
The loader refuses a YAML/process-grid mismatch; do not change grid constants in
an already running process or reuse packing state saved on a different grid.

For manual startup inside the configured ROS environment, set the grid before
any packing imports (it must match the YAML):

```bash
NEUROMEKA_XY_RESOLUTION_MM=5 ros2 run safe_servo_visualization policy_loading --ros-args \
  -p policy_config_path:=/opt/neuromeka_bin_packing/configs/real_platform_policy.yaml
```

At 5 mm, a rounded planning size of `145 x 135 x 155 mm`, with clearance 30 mm,
has virtual size `175 x 165 x 155 mm`. At 10 mm it was `180 x 170 x 155 mm`.
ROS dimension estimation/rounding, robot motion, measured utilization, and the
meaning of `target_util` are unchanged. The policy network still receives
normalized physical planning dimensions, EMS coordinates, and a feasibility mask;
the action tensor sizes do not change. A finer grid can change candidate poses,
policy choices and search cost even with the same checkpoint.

Startup logs and `/policy_loading/status` report the effective clearance and
XY resolution. New result JSON files include `effective_clearance_mm` and
`effective_xy_resolution_mm` in their configuration.

Deploy the updated ROS and Neuromeka sources and restart the stack while the
robot is stopped. The unified launcher rebuilds ROS packages. Start with a fresh,
physically reconciled inventory, not the old 10 mm packing state. Offline tests
do not constitute validation of the new loading poses on hardware.
