# RViz 命名约束：tcp_down_yaw_free

用于手动规划测试，不改变 test panel / policy 的规划参数。

- 模型 `UF_ROBOT`，规划组 `uf850`。
- `link_tcp` 相对 `link_base` 保持工具朝下（参考四元数 XYZW = `[1, 0, 0, 0]`）。
- 使用 ROTATION_VECTOR 参数化，X/Y 容差各 π/6 rad（30°），Z 容差 π，允许 yaw 改变。这是相对朝下参考姿态的旋转向量分量容差，并非世界欧拉角 roll/pitch 的绝对值限制，也不等于总倾角小于 30°。
- 仅为姿态约束，不含高于容器的净空约束，也不保证避开奇异位形。起点和终点必须满足约束。

## 在当前 RViz 中选择

1. MotionPlanning → Context → Warehouse / Database。
2. Host 填 `/workspace/results/moveit_warehouse.sqlite`，Port 保持 `33829`（SQLite 忽略端口）。
3. 点击 Connect。如果已经连接，先 Disconnect 再 Connect。不要点击 Reset。
4. Planning Group 选择 `uf850`，在 Planning → Path Constraints 中选择 `tcp_down_yaw_free`。
5. 取消 `Use Cartesian Path`，先只点击 **Plan** 查看动画；不要与自动流程同时发起执行。

若列表未刷新，可在数据库连接完成后重新选择 planning group。

## 重启后的连接参数

当前运行中的 RViz 已设置 SQLite 插件，但动态参数不会自动保存进启动配置。重启后在容器中运行：

```bash
source /opt/ros/jazzy/setup.bash
ros2 param set /rviz2 warehouse_plugin warehouse_ros_sqlite::DatabaseConnection
ros2 param set /rviz2 warehouse_host /workspace/results/moveit_warehouse.sqlite
```

然后按上述 Context 页步骤连接。数据库位于宿主机仓库 `results/moveit_warehouse.sqlite`，容器重建不丢失。Dockerfile 已加入 SQLite 插件依赖；当前容器也已安装。

## 重建或重新生成约束

在容器工作区构建 `moveit_test_constraints` 后：

```bash
source /workspace/ws/install/setup.bash
ros2 run moveit_test_constraints save_constraints
```

明确更新已有同名约束为当前默认的 30° 容差时，使用 `ros2 run moveit_test_constraints save_constraints --ros-args -p overwrite:=true`。更新后在面板先选 None，再重新选择该约束；不会改变已经提交的规划请求或自动搬运参数。

工具不发送运动请求，保存后通过 MoveIt warehouse API 读回验证。已有同名约束不会被覆盖。首次创建 SQLite 空表时，插件可能打印 `no such column: M_constraints_id`；应以写入后读回成功为准，第二次运行应正常读到已有记录。
