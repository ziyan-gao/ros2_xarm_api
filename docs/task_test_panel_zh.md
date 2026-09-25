# 独立任务调试

RViz：**Panels → Add New Panel → safe_servo_rviz_panel/TaskTestPanel**。

| 按钮 | 进程 | 任务源文件（safe_servo_visualization/tasks/） |
|---|---|---|
| Pack new | test_task_pack_new | pack_new.py |
| Unpack | test_task_unpack | unpack.py |
| Pack from slot | test_task_pack_slot | pack_slot.py |
| Repack | test_task_repack | repack.py |

每个文件定义自己的 begin / target_ready / grasp_ready 流程、目标排除规则和 MOTION 策略。
MOTION 控制抓取、搬运、放置力阈值，出发抬升高度、接触退离速度、托盘抓后与放置后的 SDK 退离开关。修改某个任务文件后，在该任务行点 Restart task node，即可重新导入。共享任务帮助代码修改也由相应任务进程重启加载。

共享的 pickup_supervisor 是运动执行器：实现控制器交接、轨迹执行、力监控等动作，执行任务下发的策略。每次任务启动先通过 ROS 原子参数服务提交完整策略和唯一 token，收到服务成功及带同一 token 的新状态后才启动动作；配置不完整、越界、过期、所有权不匹配、执行器正在执行时拒绝。任务结束后回到默认策略，避免影响后续非测试流程。

会话节点 pick_place_test 保留库存、互斥、随机调度和任务进程管理。四个任务不同时控制机器人。旧 test panel 与新 panel 使用同一会话；不要再另行启动旧 pick_place_test 可执行文件。任务进程退出不再触发统一启动脚本关闭整套系统。

共享的规划器、硬件、Servo、场景库存及执行器实现仍是公共能力。修改它们的底层实现需要重启对应公共节点；修改四个任务文件的流程或策略不需要。首次升级到这套配置接口仍需要加载新版执行器与会话，后续任务修改只重启该任务。新 C++ panel 首次加载也需要重建并重新加载 RViz。

## Reset

**Reset task state (keep items)**：确认停稳、反馈新鲜、无附着物和硬件/FT 故障；复位故障 pipeline，并把 motion 的 PREPARED 待执行路径清掉；等待新状态确认后解除任务故障。复位过程中显示进度并禁止新任务。

空夹爪、空测试库存、依赖就绪时，复位后 Pack new 重新可用。保留已有箱子记录，不承诺任意物理状态下所有按钮都可用。仍持物或库存不一致时，面板给出实际原因；不会通过删除箱子记录来强行启用 Pack new。

**Reset motion coordinator**：直接清除 PREPARED 路径，不移动机器人、不松夹、不清库存。

**Open gripper** 位于原 SafeServoPanel 的 Manual operations。先确认物体有支撑。该动作关闭真空，成功后请求清除 attached item。它与 Reset 分开。

确实需要清空测试账本时，保留显式 `/test_tasks/clear_bookkeeping` 服务及原有空场景检查；日常 Reset 不调用它。会话本身重启后的库存持久化不在此实现范围。

## 验证

离线测试覆盖真实子进程启动与单独重启、旧 token 和重复命令隔离、任务策略确认前不执行、配置重载而执行器不重启、执行中拒绝改策略、Reset 清理 PREPARED 后 Pack new 允许条件恢复、故障和重启保留库存。真机运动未用于这些测试。


## 共享应用节点 Restart（本次新增）

Task Test Panel 的滚动区域列出共享节点，显示 PID、进程是否存活、退出状态、
重启阶段和阻塞原因。RUNNING 仅表示进程存活，任务启动仍须通过原有 ROS 就绪检查。

已接通：pickup_supervisor、motion_coordinator、pickup_pipeline、place_pipeline、
pick_place_pipeline、random_stable_loading、policy_loading、item_localization、
waypoint_store、visualization_node、box_marker_detector、depth_box_refinement。

start_unified.sh 用固定白名单监控器直接启动这些应用可执行文件，保留启动参数。
应用退出不再使监控器退出；Restart 只向自己启动的应用发送 SIGINT，
确认旧进程退出后才启动替代进程。5 秒仍未退出就报告超时，不强杀、不启动第二份。
要求任务停止、自动运行关闭、关节反馈新鲜且静止。重启不重放任务或操作夹爪。
首次使用需要加载新版面板与启动脚本；当前会话中的旧进程无法凭名称被接管。

未接通的按钮明确显示 Restart backend unavailable：top_face_debug、staging_slots、
planning_scene_obstacles、pallet_localization、task_test、servo_node、safe_servo、
move_group、xarm_planner_node、ros2_control_node、robot_state_publisher、
realsense2_camera_node、cumotion_action_server。
库存所有者须先实现可靠 checkpoint 恢复。底层进程管理方案被自动审批拦截，
这些按钮当前没有发送终止信号的后端。

待批准的底层方案：仅管理启动器实际创建并记录的固定子进程；RViz 不参与重启。
点选对应按钮后结束该子进程及其所属启动组，确认退出后使用原参数启动，
停止或退出超时明确报告；不按节点名搜索/杀进程，不清库存、不自动恢复任务或释放夹爪。
ros2_control、Servo、MoveIt 重启会短暂中断相应控制/规划服务，必须在任务已停后进行。
