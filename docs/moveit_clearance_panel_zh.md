# RViz 净空路径约束对比测试

配置文件：`config/moveit_test_constraints.yaml`，不控制自动装箱流程。

- `clearance_constraint_enabled`：启用/关闭路径高度约束。
- `tcp_min_z_m` / `tcp_max_z_m`：`link_base` 下 `link_tcp` 的绝对 Z 范围（米），不是物体底部高度。
- `tilt_tolerance_rad`：旋转向量 X/Y 容差，默认 π/6（30°）；yaw 自由。
- 保存名字：`tcp_down_yaw_free_clearance`，保留原 `tcp_down_yaw_free` 用于对比。

当前 0.4364 m 是上次日志的示例下界，不是所有物体通用的安全高度。
上下界必须覆盖起点和目标；建议不要让二者恰好处于边界。
4 m × 4 m 的 XY 区域与自动流程一致，较大的 Z 上界不是物理可达高度。
关闭高度约束不关闭碰撞检查，但规划只能保护场景中正确建模的障碍。

首次编译并保存（不产生运动）：

```bash
docker exec -it ros2_cv bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/xarm_ws/install/setup.bash && cd /workspace/ws && colcon build --packages-select moveit_test_constraints && source install/setup.bash && ros2 run moveit_test_constraints save_constraints --ros-args --params-file /workspace/config/moveit_test_constraints.yaml'
```

以后修改 YAML 后重新保存：

```bash
docker exec -it ros2_cv bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/ws/install/setup.bash && ros2 run moveit_test_constraints save_constraints --ros-args --params-file /workspace/config/moveit_test_constraints.yaml'
```

在 RViz 中连接同一个 SQLite 数据库 `/workspace/results/moveit_warehouse.sqlite`，
刷新/重新连接后，在 Path Constraints 选择 `tcp_down_yaw_free_clearance`。
修改 YAML 不会自动刷新数据库或已选中的约束；保存后重新选择。
先使用 Plan 预览，不要直接 Plan & Execute。
