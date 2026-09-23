# 顶面定位与抓取调试面板

目的：在接入 unpack、repack、slot pickup 之前，独立验证“已知箱体位姿 → 图像投影 → SAM 分割 → 使用记录顶面高度定位”。不用反复执行完整装箱流程来调试相机。

当前算法**不使用实时深度图**：对 SAM 顶面外轮廓像素去畸变，将相机射线变换到 base 坐标系，与 `Z = 记录顶面高度` 求交，再在 XY 平面拟合矩形中心和 yaw。Z 保持记录值，不是重新测得的高度；相机标定或记录高度的误差仍会影响 XY 定位。

采集、SAM 和保存仍不产生运动。新增的 **Move to / Pick 会控制真实机械臂**，必须单独点击并确认；请先关闭 policy/random/test 自动循环。

这是独立调试入口，不属于 policy/MCTS 操作序列：抓取成功会移除原场景物体并由现有 pickup 流程附着到工具；slot 来源还会清空对应 slot。**不会自动改写 policy/test 的装箱账本，测试后须核对实物并重置实验状态，不能直接恢复旧实验。** 若移除源物体后抓取失败，不猜测物体位置、不自动释放真空；停机核对场景和实物。

## 面板按钮

RViz 面板名称：`Top-face Inspection Test`。如果恢复的 RViz 配置里没有它，使用 `Panels → Add New Panel → safe_servo_rviz_panel/TopFaceDebugPanel`。

| 控件 | 功能 |
| --- | --- |
| 目标下拉框 | 选择场景中已经放置的箱子，或当前检测框 |
| `Move to` | 用记录顶面、CameraInfo、相机到 TCP 的 TF 计算居中位置，正常 MoveIt 碰撞检查规划并执行；保持当前 EEF 姿态 |
| `Pick` | 使用本目标通过检查且未过期的 SAM XY/yaw、记录顶面 Z，生成 pre-pick，复用已知物体 pickup 路径与力控下降、吸取、抬升 |
| `Stop motion` | 请求取消规划/运动及停止 pickup，不自动关闭真空；不是硬件急停的替代品 |
| `1. Capture / project` | 冻结 RGB 帧，按采集时间查询 TF，投影已知顶面中心和四角；无需深度帧 |
| `2. Run SAM once` | 对冻结的图像做一次 SAM2 分割，将轮廓反投影至记录高度平面，估计 XY 和 yaw |
| `3. Save debug data` | 保存这次采集、投影、分割和拟合结果 |
| `Clear capture` | 清空显示和结果；正在运行的推理不会强制中断，但其晚到结果会被丢弃 |

目标名 `placed:placed_item_visuals:N` 中的 `N` 是场景 marker ID，**不是 slot 编号**。场景记录包括 pallet 和 slot 中已释放的箱子；目标移除后会从列表消失。`detected:...` 来自现有箱体检测器，可用于首次调试。记录只是预测先验，不代表新的视觉确认。

图像颜色：绿色为记录尺寸投影出的预测顶面，紫色为 SAM mask，黄色为 SAM 拟合中心、yaw 和拟合宽长构成的矩形，白色十字为图像中心。青色菱形是目标 pre-pick TCP（接触点上方 30 mm）在该冻结图像中的预测投影，并非实时 TCP 检测。由于相机斜视和高度差，它通常不与黄色接触点重合。

图像显示 Prior XY / SAM XY 两组尺寸，帮助判断绿框偏大来自记录尺寸还是投影几何。SAM 尺寸同样依赖记录顶面高度和手眼标定，不是独立真实尺寸；这里只改变黄色框显示，不自动修改规划场景或抓取尺寸。

## 两按钮调试顺序

当前本地 `.env` 设置 `PICKUP_FORCE_THRESHOLD_N=15.0`，用于新物体及再次抓取的相对 Z 向接触力阈值；放置阈值独立，未随之修改。该值在 pickup supervisor 启动时读取并缓存，不支持仅用 `ros2 param set` 热更新。修改 `.env` 后需在机械臂停止、物体妥善安置后重建容器以载入新环境；仅重启 top-face debugger 不会生效。

`Move to` 不做顶面水平性检查，直接使用记录的中心和四角（保留倾角）计算观察位置。SAM 的水平面拟合检查、入镜检查和运动规划检查仍保留。

1. 保持工具为空、相机朝下，停止其他自动流程，选中目标。
2. 点击 `Move to` 并确认。默认 `inspection_tcp_z_m=0` 表示自动选择高度：从现有容器安全高度与顶面上方 100 mm 两者较高处开始，只有顶面无法完整入镜才每次升高 10 mm，最多额外 200 mm。正参数可指定更高的 TCP 高度（base 坐标，米），不会降低容器安全下限。使用完整手眼变换让顶面中心位于图像中心；目标 EEF 朝向保持当前值。无法完整入镜或规划失败会停止，不执行部分路径。
3. 到位后点击 Capture，再 Run SAM。Move to 会清空旧结果，不能用移动前的图像直接抓取。
4. 确认黄色框正确后点击 Pick。接受命令时要求图像不超过 30 秒（`refine_max_age_sec`）；接受后为当前最长 180 秒操作冻结目标，不因规划/移动耗时超过 30 秒而在 pre-pick 处退出。到位仍检查记录目标和尺寸没有改变。pre-pick 为顶面中心上方 30 mm。抓手朝向由保存的箱体相对 TCP 旋转恢复：`R_base_tcp = R_base_object(refine) × inverse(R_tcp_object记录)`，不使用观察姿态作为基准。箱子未转动时，抓手恢复原来的抓取方向；箱子转动后则保持原有抓手—箱体相对关系。已知物体 approach 先走安全高度，随后复用 `/pickup_supervisor/start`：力达到阈值后吸取、抬升。到下降下限但没有力确认会报错，不能视作拾取成功。
5. 成功后保持真空，不会自动放置。没有真空反馈硬件，力确认只证明接触，仍须目视确认物体确实吸牢。

可以在启动节点时通过 ROS 参数调整观察高度和结果有效期；需要重启 debugger、motion coordinator、planning scene、staging slots 及 RViz 才能使用新增接口。

抓取朝向记录由新版 planning scene 节点在释放时保存，并按 marker ID 发布。升级前已经放下的箱子没有这项记录，不能从观察姿态补猜：需在更新场景节点后重新抓取、放置生成记录。此调试 Pick 只支持具有记录的 placed 目标；新检测物体使用原有新物体拾取流程。记录目前随场景驻留内存，不是跨节点重启的持久化库存。

## 构建和启动

先停止自动实验，机械臂保持静止。修改 RViz 插件后需要重新启动 RViz 才能加载新库。

在宿主机进入容器终端：

```bash
docker exec -it ros2_cv bash
```

在容器中构建：

```bash
source /opt/ros/jazzy/setup.bash
source /opt/xarm_ws/install/setup.bash
cd /workspace/ws
colcon build --symlink-install --packages-select safe_servo_visualization safe_servo_rviz_panel
source install/setup.bash
```

统一启动脚本现在默认自动启动调试节点（`TOP_FACE_DEBUG_ENABLED=true`），**不需要每次手动运行**。它不加入主流程的必需进程列表：调试器退出不会停止机器人主流程，统一关闭时则会一并结束调试器。模型推理只在按按钮后开始。

若当前容器是在修改启动脚本前启动的，不需要为此重启机器人。先在原来的调试终端按 Ctrl+C，仅停止旧调试节点，再在宿主机新终端启动新版：

```bash
docker exec -it ros2_cv bash /workspace/start_top_face_debug.sh
```

这时不需要 SAM2，就可以使用投影、采集、保存功能。相机、标定 TF 和场景记录的发布节点需要已经运行。场景节点新增了每 0.5 秒的只读 marker 快照发布，方便调试器晚启动后收到已有箱子；这个修改不改变碰撞物或箱体状态。

### 启用 SAM2

实现使用官方 [SAM2ImagePredictor](https://github.com/facebookresearch/sam2/blob/main/sam2/sam2_image_predictor.py) 的点提示和框提示接口。需要在**运行这个 ROS 节点的 Python 环境**安装兼容的 `torch`、`sam2`，并准备本地 checkpoint。模型配置必须与 checkpoint 匹配；程序不会自动下载模型。

已在工作区的 `.venv-top-face` 独立环境安装 SAM2，并下载 SAM2.1 tiny checkpoint 到 `results/models/sam2.1_hiera_tiny.pt`。基础机器人/策略环境的 PyTorch 保持不变。该虚拟环境用于当前 ROS 镜像的 Python 3.12；换 Python 或基础镜像后需重新准备兼容环境。

在其他相同环境的机器首次安装时，容器内运行（下载官方依赖和约 149 MiB 模型；不要在机器人执行任务时安装）：

```bash
bash /workspace/setup_top_face_sam.sh
```

启动包装脚本会优先使用独立环境，否则退回系统 Python（仍可投影、保存）。不要再用裸 `ros2 run` 来启动 SAM 测试，它的解释器可能仍然是没有 SAM2 的系统 Python。

`.env` 可配置 `TOP_FACE_DEBUG_ENABLED`、`TOP_FACE_SAM_CHECKPOINT`、`TOP_FACE_SAM_MODEL_CONFIG`、`TOP_FACE_SAM_DEVICE` 和 `TOP_FACE_SAM_THREADS`。默认 CPU、2 个推理线程。只有运行环境确实支持 CUDA 时才改成 `cuda`。Compose 环境变量修改在下次安全重建容器时生效，不要为本调试强制中断正在进行的实验。

SAM2 代码固定到 `2b90b9f5ceec907a1c18123530e92e794ad901a4`，PyTorch 为 2.5.1 CPU，TorchVision 为 0.20.1 CPU。模型下载后的 SHA-256 校验值固定在安装脚本内。自动启动不会进行安装、下载或依赖升级。

## 最快的调试顺序

### 本机 GPU 配置

本机 NVIDIA runtime 已通过隔离容器检查，可访问 RTX 3080。重新安装后的真实 SAM2 CUDA 合成图像测试通过，三次推理为 0.267、0.073、0.065 秒（不是实际箱体精度或端到端耗时）。19 项调试模块测试通过，CUDA 调试节点的隔离启动检查通过。

**GPU 环境已安装，默认配置已切换为 CUDA**。用户清理磁盘后重新安装成功，原 CPU 环境、模型和实验数据保留。安装器保留至少 12 GiB 可用空间检查。验证只使用隔离容器，没有自动启动机器人主栈；下次正常启动会启用 GPU 调试器。

GPU 模式使用 `compose.gpu.yaml` 作为增量配置；基础 `compose.yaml` 不强制要求 GPU。本机 `.env` 已设置：

```env
COMPOSE_FILE=compose.yaml:compose.gpu.yaml
TOP_FACE_SAM_DEVICE=cuda
```

CUDA 环境与 CPU 环境分开：`.venv-top-face-cuda` 使用 PyTorch 2.5.1 / CUDA 12.4 和 TorchVision 0.20.1，原 `.venv-top-face` 保留。准备 CUDA 环境的命令是：

```bash
bash /workspace/setup_top_face_sam.sh cuda
```

当前机器的 NVIDIA runtime 使用 CDI 模式，因此 GPU 增量配置同时指定 `runtime: nvidia` 和 GPU device reservation。此配置已由 `docker compose config` 检查。

实验结束、确认可以启动机器人主栈后，在宿主机仓库目录执行 `docker compose up -d --force-recreate ros2_cv`，让新容器获得 GPU 设备访问。**这个命令会启动机器人主栈，不是纯分割测试命令。** 仅重启旧容器内的调试节点无法新增 GPU 设备访问。若命令里显式使用 `-f`，必须同时包含 `-f compose.yaml -f compose.gpu.yaml`。

新容器会自动用 CUDA 环境启动调试器，终端输出 `SAM GPU: NVIDIA GeForce RTX 3080`。若 CUDA 环境或设备不可用，调试器会明确退出，不会默默改用 CPU，也不会停止主装箱流程。

回退 CPU 推理：将 `.env` 的 `TOP_FACE_SAM_DEVICE` 改为 `cpu`，在下次安全重建容器时生效。若还要取消 GPU 容器配置，将 `COMPOSE_FILE` 改成 `compose.yaml`。策略使用的系统 Python/PyTorch 不变。

实际模型的无机器人 smoke test 在 `tests/top_face_sam_smoke.py`，只使用合成图像；它不代表真实箱体定位精度已经验证。

### 按钮操作

1. 暂停自动装箱，只选一个箱子，人工通过已有的安全控制方式把相机移到可观察位置，等机械臂静止。
2. 点击 `Capture / project`。先检查绿色轮廓是不是对应所选箱子的顶面。如果不对，先排查目标记录、坐标系、TF 标定和相机内参，不要用 SAM 掩盖投影问题。
3. 确认整个顶面在画面中后运行 SAM。提示点为预测中心及向四角方向内缩的四个点，框为顶面投影的包围框。
4. 看紫色 mask 是否只包含正确顶面，黄色中心/yaw 是否合理。若 mask 包含侧面，投影到顶面高度后会畸变；仍检查已知尺寸和目标关联，不再用深度平面过滤侧面。
5. 轻微移动箱子或改变光照，再次采集、分割、保存。比较结果里的 `delta_center_m`、`delta_yaw_deg` 和 `image_center_error_px`。整个过程无需实际抓取。
6. 先用这些固定场景数据验证误差，再接入“预览下一步 → 单步移动 → 重新采集”的相机对准闭环。不要直接把单帧结果接入自动抓取。

## 当前诊断条件与限制

- RGB 帧采集时不超过 800 ms；查询 RGB 时间戳对应的 TF。仅在机器人静止时采集，帧的新鲜度不代表物体或机器人稳定。节点不订阅深度图。
- RGB 与 CameraInfo 的图像尺寸、光学坐标系必须匹配。支持 `plumb_bob` / `rational_polynomial` 内参。
- 完整预测顶面需处于图像内部，留 12 px 边缘；投影太小或被裁剪时仍能保存采集，但不允许开始分割。
- 至少 120 个分割像素；主连通区域至少占 mask 的 90%，mask 不得触及图像边界。保留完整轮廓，不按先验箱体范围裁剪，以免人为缩小尺寸。
- 使用水平顶面假设，记录顶面角点高度与中心差不超过 5 mm；拒绝近乎平行于平面的射线，以及交点在相机后方或相机处的情况。没有深度平面拟合或深度内点比例判据。
- 顶面 XY 尺寸与记录相差不超过 25 mm，拟合中心 XY 偏移不超过 50 mm。它们只是本调试模块的关联检查，不改变现有抓取判据。
- 矩形的 180° 对称性无法单靠顶面区分，因此选择与记录 yaw 接近的等价解；接近正方形时朝向也可能不确定。
- 这是单帧定位，**没有实现 20 帧稳定确认或闭环增量居中**。Move to 是按记录几何一次计算。调试期间不要操作其他控制面板；本面板检查其他流程状态，但不是全系统排他控制锁。
- 力达到阈值只能确认接触，不能证明真空已建立。本次没有更改现有吸取/力反馈流程。

## 保存位置和内容

默认容器路径：`/workspace/results/top_face_debug/<UTC时间戳_唯一ID>/`。

对应宿主机：`src/ros2_xarm_api/results/top_face_debug/`。

- `frames.npz`：RGB、SAM mask（未运行时为空），新采集不再包含 `depth_m`。
- `metadata.json`：目标 ID、真实记录尺寸、图像时间戳、内参、采集时变换、提示点、投影和拟合结果。
- `preview.png`：叠加后的调试图。

新记录为 `schema_version: 2`，包含 `estimation_method: rgb_mask_recorded_top_plane`、`uses_measured_depth: false` 和 `recorded_top_z_m`。历史采集文件保持不变。

可以通过 `results_directory` ROS 参数改变目录。结果带 `diagnostic_only: true`；不会写入 policy 实验 JSON，也不会自动覆盖抓取位姿。保存的数据可用于离线复现，但第一版面板尚无加载旧采集的按钮。

ROS 接口仅为 `/top_face_debug/command`（JSON 命令）、`/top_face_debug/status`（状态）、`/top_face_debug/preview`（图像），不包含任何机器人控制客户端。
