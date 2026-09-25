# cuMotion isolated experiment

## Real-state / real-scene commissioning profile (2026-09-24)

Optional integration now exists in `compose.cumotion.yaml`. It adds cuMotion to
the existing MoveIt pipeline list, selects it as the default for unqualified
MoveIt requests, and uses the existing driver, controller and joint feedback.
It does not start a second controller or fake joint publisher. This live test
profile explicitly enables MoveIt trajectory execution so observation and
pre-grasp stages used by the test panel can run. Pose path constraints,
octomaps, non-box attachments and unsupported collision exemptions are rejected.

The camera and box payload use cuRobo's standard volume/surface sphere
approximation. The fixed camera is baked into the robot model once. Twenty-four
disabled collision spheres are preallocated on `link_tcp`; attaching a box updates
those spheres from the box dimensions and object-to-TCP transform, and detaching it
disables them. Payload attach/detach therefore does not rebuild `MotionGen` or
repeat CUDA warm-up. MoveIt's exact-geometry response validation remains enabled.
Changing the fixed camera geometry still requires a model rebuild. World primitives
are taken from each complete MoveIt scene request.
Actual dimensions/placements must still be checked in RViz. This is not a general
mesh/constraint implementation or an execution-certified backend.

The live bridge publishes the exact configured robot/camera spheres and the
current 24-sphere payload fit on `/cumotion/collision_spheres`. The repository
RViz configuration enables this MarkerArray by default: cyan is the robot,
purple is the EEF camera, and orange is the carried payload. World obstacles
remain the PlanningScene boxes/voxels and are not converted into marker spheres.

The live profile also adds two **cuMotion-only** clearance barriers to each
planning request. One follows the measured `pallet_surface` footprint from its
top to the pallet transfer clearance; the other covers only the six-slot
footprint from the slot surface to the slot transfer clearance. They are not
added to MoveIt's global PlanningScene, so the existing Cartesian approach and
descent phases remain possible. RViz displays them as translucent red/orange
boxes on `/cumotion/clearance_barriers`. The complete measured work-table
footprint is deliberately not extended upward because it contains `link_base`
and would make every cuMotion start state collide.
The configured pallet/slot clearances are pallet-relative distances; the
bridge converts them to absolute `link_base` Z using the localized pallet
surface. A 3 mm top margin avoids treating an exact planned endpoint as
penetrating contact with the virtual floor.
The slot barrier uses `CUMOTION_SLOT_CLEARANCE_Z_M=0.480`. Pallet and
slot barriers are injected into every cuMotion request, including empty-tool
inspection and return motions. Payload sphere collision remains enabled. For
empty-tool cuMotion endpoints use `TRANSPORT_EMPTY_TOOL_DROP_M=0.030`, based
on the measured 24.5 mm vacuum-gripper sphere extent below `link_tcp`; the
existing 10 mm endpoint margin remains additional. For
carried transfers, `TRANSPORT_MAX_PAYLOAD_DROP_M=0.300` raises both clearance
endpoints by at least 300 mm plus the existing 10 mm endpoint margin; larger
measured downward extents still win. This keeps boxes up to 300 mm high clear
of the fixed virtual floors without rebuilding the GPU model or repeating
warm-up. The duplicate local low-Z/column rejection is disabled with
`TRANSPORT_LOCAL_CLEARANCE_VALIDATION_ENABLED=false`; workspace, exact endpoint,
PlanningScene collision, Cartesian vertical approach, and Servo force checks
remain active.
The markers remain visible continuously and the boxes are injected into every
cuMotion collision world. Low final approach and descent motions remain separate
Cartesian/Servo phases and do not add these cuMotion-only boxes to MoveIt's
global PlanningScene.

Build from ROS repository root:

```bash
docker build -t uf850-cumotion-live:4.0 -f experiments/cumotion/Dockerfile.live experiments/cumotion
```

When the robot is stationary, no operation is active and any held object is secured,
replace the current service using the same project and optional override:

```bash
docker compose -f compose.yaml -f compose.gpu.yaml -f compose.cumotion.yaml up -d --no-build ros2_cv
```

This restarts the existing ROS service; it must be an operator-controlled step.
In the existing RViz select group `uf850`, pipeline `isaac_ros_cumotion`, and **Plan**.
Leave path constraints empty for this backend; keep OMPL for constrained planning.
The GPU backend takes time to warm up. The same profile also enables RViz Execute;
use it only as an intentional real-robot command after reviewing the planned path.
The isolated panel demo described below remains execution-disabled.

The application test panel can select the same backend with:

```dotenv
TRANSPORT_MOVEIT_PIPELINE_ID=isaac_ros_cumotion
TRANSPORT_MOVEIT_PLANNER_ID=cuMotion
```

This selection replaces every sampling-planner request made by the test-panel
transport code. Cartesian lift/descent, force-controlled Servo contact and the
existing trajectory execution/validation remain unchanged. For cuMotion the
request intentionally contains no MoveIt path constraints; exact endpoint and
planning-scene collision constraints are retained. Start the stack with
`compose.cumotion.yaml`; the base image does not provide this pipeline.

The live model can narrow selected physical joint intervals without changing
the xArm driver or ros2_control limits. The current commissioning values are
configured in `.env` as signed MoveIt joint angles:

```dotenv
CUMOTION_JOINT3_MIN_DEG=-130.0
CUMOTION_JOINT5_MAX_DEG=40.0
CUMOTION_PARALLEL_FINETUNE=false
```

They apply to the entire cuMotion trajectory, not only its endpoint. Startup
fails closed if an override expands a physical URDF limit, creates an empty
interval, or excludes the taught observation configuration. Restart the
`ros2_cv` service to rebuild the temporary GPU model after changing them.

New-item pre-grasp requests follow the same effective route as an RViz
interactive-marker goal: the motion coordinator calls collision-aware MoveIt
IK with an explicit fresh real-joint seed, then sends the validated joint goal
to cuMotion. This avoids cuMotion pose-IK false negatives without relying on a
second internal current-state monitor. A failed or malformed IK response is not
passed to the planner. Other pose and orientation-constrained services are not
silently converted or relaxed.

To restore the previous image/stack, omit the cuMotion override:

```bash
docker compose -f compose.yaml -f compose.gpu.yaml up -d --no-build ros2_cv
```

Validation: image built and Compose/shell syntax checked. Offline import of the
captured actual scene included five world obstacles and the attached D435i camera.
After replacing the over-conservative covers with cuRobo's standard fitting, the
captured all-zero start to a joint1 +0.03 rad target planned successfully (32 points).
That recorded check was plan-only; it does not validate later live execution.

Status: GPU planning, ROS Action, live planning-scene integration and isolated
MoveIt pipeline tests passed. The live profile is connected to the application
test panel and real controller. A separate execution-disabled RViz/MoveIt demo
is also available. Orientation path-constraint integration is not complete.

## 独立 MoveIt 面板测试（不执行）

在本机桌面终端、ROS 仓库根目录运行：

```bash
bash experiments/cumotion/run_panel_demo.sh
```

在工作区根目录则运行：

```bash
bash src/ros2_xarm_api/experiments/cumotion/run_panel_demo.sh
```

等待 GPU 预热和 RViz 模型加载，然后在新窗口的 MotionPlanning 中：

1. Planning Group 选择 `uf850`。
2. Pipeline 选择 `isaac_ros_cumotion`，Planner 选择 `cuMotion`。
3. Path Constraints 保持为空。本版不支持 ±30° 路径约束，有约束会拒绝。
4. Start State 使用 current，拖动目标交互标记，然后点 **Plan** 预览轨迹。
5. 不使用 Execute / Plan & Execute。MoveGroup 设置了
   `allow_trajectory_execution=false`，并移除了 ExecuteTrajectory capability。
   没有机器人驱动、控制器或真实 ROS 网络连接。

模型起点来自 `taught_waypoints.yaml` 的 observation 关节角，不是实时机器人状态。
只有 UF850/tool 和合成地面，不含实际相机、负载、桌面及 pallet。
此入口只用于测试规划速度/路径形状，不能作为真实场景安全验证。
关闭测试时在启动终端按 Ctrl+C；现有生产窗口不受影响。
无需连接 warehouse 数据库，也不需要启动 `ros2_cv`。

该镜像从固定官方提交重新编译纯 C++ MoveIt 插件，以匹配已安装的
MoveIt 2.12.4（官方预编译库依赖 2.12.3）。仅对该纯 C++ 插件跳过
`isaac_ros_common` 的 CUDA 开发工具探测；GPU 运行依赖不变。
面板专用 bridge 校验默认 SRDF 碰撞豁免、转换对象级位姿，拒绝附着物和
未支持的场景特性。不要把它放到生产 ROS graph。

已验证 headless MoveIt pipeline：返回成功，规划 0.338 s，MoveIt
`ValidateSolution` 和独立原始几何采样检查通过；仍是 joint1 +0.1 rad 小范围用例。
另已验证本机 RViz X11/OpenGL 4.6 启动及同一后端规划；15 项回归测试通过。
启动脚本自动适配本机 Snap Docker 的 hostfs X11 socket 路径，不使用 `xhost +`。
测试不配置控制器和深度地图，因此启动日志可能出现 `No controller_names`
和 `No 3D sensor plugin(s)`；这不代表本次规划失败，也不要为消除这些日志接入真机控制器。

## ROS Action integration test

From the ROS repository root, run the server/client pair inside one isolated
container (the production ROS stack need not be started):

```bash
docker run --rm --network none --runtime nvidia --gpus all \
  -v "$PWD/experiments/cumotion:/experiment:ro" \
  -v "$PWD/config/taught_waypoints.yaml:/waypoints.yaml:ro" \
  uf850-cumotion-test:4.0 -lc 'timeout 150s bash /experiment/test_ros_action.sh'
```

This exports the UF850 model, starts the official cuMotion server through a
plan-only adapter, and sends a `moveit_msgs/action/MoveGroup` query to
`/cumotion_test/cumotion/move_group` in ROS domain 87. The client validates the
returned trajectory against original URDF geometry and the diagnostic tilt
bound. The server is stopped when the test ends. No drivers, controllers,
trajectory execution actions or production launch files are started/modified.

Currently accepts explicit six-joint start/goal states and primitive world
geometry in `link_base`. The adapter now calls `plan_single_js` directly:
it preserves the requested joint goal instead of converting it to a TCP pose
and solving IK again. Returned joint endpoints are checked against the supplied
tolerances. Planning failures abort the Action, and exceptions release the busy
flag. Logs include ordered start/goal joints and world object IDs for reproduction.
Execution requests, path constraints, attached objects, scene diffs, custom
ACM, padding/scaling and unsupported geometry are **rejected**, not discarded.
The server response by itself is not an execution-approved trajectory; the
independent checker is currently in the smoke client, not a production gate.

After the joint-goal fix, the 0.1 rad MoveIt pipeline test passed in 0.0842 s
(single run); 19 regression tests passed. This does not reproduce the user's
previous failed targets because the old server did not log their joint values.
Restart the isolated panel to load this Python change; no image rebuild needed.

The packing-scene issue is **not fixed by the joint-goal change**. The launcher
now prints an explicit synthetic-scene warning. Real-scene testing requires a
live planning-scene snapshot including attached camera/payload and subsequent
attachment-model import. Do not infer that this demo reflects the packing scene.

This is ROS transport integration only. Do not wire this endpoint to the
production MoveIt plugin yet: camera/payload import, real scene validation,
constraint-aware planning and production-side trajectory validation are still
required. The default production planner remains unchanged.

Verified 2026-09-24: the isolated ROS Action test exited successfully. Both
negative requests (execution and unsupported path constraint) were rejected.
The valid synthetic-floor query returned SUCCESS, reported planning time
0.3275 s, and passed independent checks at 38 samples (0.0092 s check time,
0.04984 m TCP path). This remains the small observation/joint1+0.1 rad case.
The static-scene service warning about no file is expected in this test:
the synthetic floor is supplied explicitly in the action request instead.

## Verified results (2026-09-24)

- Independent image `uf850-cumotion-test:4.0` built successfully.
- RTX 3080, PyTorch `2.9.0+cu130`, CUDA 13.0: GPU matrix operation passed.
- `MotionGen` import including native dependencies passed.
- Bundled Franka model, one small reachable goal, floor collision geometry:
  warmup 5.033 s; one planning call 0.06434 s; success true.
- This is NOT a UF850 benchmark, a constrained-transfer test, or a claim of
  comparable speed for the production scene. No robot motion was executed.

Run the bundled-model test (120-second process timeout):

```bash
docker run --rm --network none --runtime nvidia --gpus all \
  -v "$PWD/experiments/cumotion:/experiment:ro" uf850-cumotion-test:4.0 \
  -lc 'source /opt/ros/jazzy/setup.bash && timeout 120s python3 /experiment/plan_smoke.py'
```

## Environment

### UF850 offline model progress

Latest experiment uses `--sphere-model slabs --sphere-pitch 0.04` (now the
defaults). Closed link meshes are cut into slabs, then every slab's clipped
vertices are enclosed by a sphere with 1 mm numerical/geometric padding.
Open meshes retain AABB coverage. The base/wrist and thin sensor retain
local AABB covers because large slab spheres overlap nearby fixed geometry.
No self-collision exclusions were added.

Latest one-pair test: **179 spheres**, initialization/warmup 6.754 s,
known-joint-goal planning **0.0743 s**, Cartesian-pose-goal planning
**0.2509 s**, both successful. The test changes joint1 by only 0.1 rad from
the stored observation configuration. These are single-run measurements,
NOT a representative transfer benchmark or approval for execution.

Earlier 259-sphere AABB model also passed known-joint-goal planning (1.517 s)
while its pose-goal query failed IK. Thus that IK failure did not establish
that the target was physically unreachable.

`uf850_smoke.py` expands the patched UF850 xacro with the vacuum tool and
sensor stack, uses `link_base -> link_tcp`, retains URDF joint limits and
only the existing SRDF self-collision exclusions. It strips control/Gazebo
declarations from the temporary URDF. No ROS nodes are created.

```bash
docker run --rm --network none --runtime nvidia --gpus all \
  -v "$PWD/experiments/cumotion:/experiment:ro" \
  -v "$PWD/config/taught_waypoints.yaml:/waypoints.yaml:ro" \
  uf850-cumotion-test:4.0 -lc \
  'source /opt/ros/jazzy/setup.bash && source /opt/xarm_ws/install/setup.bash && timeout 60s python3 /experiment/uf850_smoke.py --waypoints /waypoints.yaml --plan'
```

Omit `--plan` to run FK and initial sphere-overlap diagnostics only.

Measured results:

- 21 configurations (saved observation plus 20 reproducible random joint
  samples): independent URDF-chain FK vs cuRobo TCP FK maximum position error
  5.763e-7 m and rotation error 9.837e-7 rad. This verifies conversion
  consistency, not physical calibration or reachability of random samples.
- Uniform 40 mm AABB covering cells produced 842 spheres; cuRobo selected its
  slow self-collision kernel; initialization/warmup exceeded 120 seconds.
- Uniform 80 mm cells produced 163 spheres and fast warmup (~2.72 s), but
  conservative sphere overlap between link4 and the force sensor rejected
  the start state. Do NOT add a collision exclusion to hide this.
- Local sensor refinement to 20 mm gives 259 spheres and no detected initial
  sphere overlaps. Warmup took ~23.57 s; the small Cartesian goal still failed
  with IK_FAIL (~0.65 s). This is retained via `--sphere-model aabb --sphere-pitch 0.08`.
- A 30 mm sensor refinement gave 186 spheres, but reintroduced overlap;
  rejected as a candidate.

Remaining work is model review and comparison against MoveIt mesh collision
results over representative configurations before integrating. Runtime camera body and payload are deliberately
marked **absent**; the scene only includes a synthetic floor. No production
collision-free or +/-30-degree compliance claim can be made from this test.

Verified on 2026-09-24: RTX 3080 10 GiB, host NVIDIA driver 580.126.09;
`nvidia-smi` works inside a separate GPU container. Host is Ubuntu 20.04;
the existing ROS image provides Ubuntu 24.04/Jazzy userspace. This is not
the complete officially tested Ubuntu 24.04 host setup.

The Dockerfile adds the **release-4.0** Isaac ROS repository to a separate
image. It does not edit the production image, compose configuration, or host
APT sources. Package installation success and CUDA planning must be tested
separately; a successful `nvidia-smi` is not a CUDA computation test.

From the ROS repository root:

```bash
docker build -t uf850-cumotion-test:4.0 -f experiments/cumotion/Dockerfile experiments/cumotion
docker run --rm --network none --runtime nvidia --gpus all uf850-cumotion-test:4.0
```

Do not mount robot devices or use host networking. Domain 87 is reserved for
this experiment; network isolation is the actual barrier to the production
ROS graph. The image starts an import/device probe, not a robot driver.

## Important constraint compatibility finding

Inspected official `release-4.0` commit
`1acd5d89d0b32b8972e614ff149996803a2d3f45`:

- `isaac_ros_cumotion_moveit` forwards the MoveIt request to
  `cumotion/move_group`.
- `cumotion_planner.py` reads goal position/orientation or joint constraints,
  but does not consume MoveIt `path_constraints` when calling `plan_single`.
- The separate goal-set action has `hold_partial_pose` support. This is NOT
  proof of equivalent support for MoveIt's roll/pitch +/-30 degree tolerance.

Do not select this plugin in production on the assumption that existing
orientation constraints are honored. Do not silently discard them.

## Remaining acceptance gates

### Capturing the actual scene (read-only)

`capture_scene.py` requests `/get_planning_scene` and reads TF. It never starts
the stack, plans, executes, or modifies the scene. Capture while the robot is
stationary, with the pallet localized and camera collision body present.
An attached payload is included only if the live scene actually contains it.
Do not pick an item merely to run this command; use an already safe stationary
state from your normal workflow. The scene and latest TF are not atomic.

With the normal `ros2_cv` stack already running, from the ROS repository root:

```bash
docker exec ros2_cv bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/ws/install/setup.bash && python3 /workspace/experiments/cumotion/capture_scene.py --output /workspace/results/cumotion/scene_snapshot.json'
```

Choose a new output filename for each capture; existing files are not overwritten.
The snapshot includes full collision geometry, attached objects/touch links,
joint state, allowed-collision matrix, padding/scaling, octomap and base-frame
transforms. Missing camera TF or object frames cause capture to fail explicitly.
Inspect `camera_included`, `world_object_ids`, and `attached_object_ids` before
using it. A saved snapshot alone does not prove the scene matches reality.

**Not yet wired into `uf850_smoke.py`:** the current smoke command still uses
the synthetic floor. The next adapter must import these geometries and attachment
transforms, and either implement or explicitly reject unsupported scene features.
Do not interpret the current smoke benchmark as a full-scene validation.

### Independent sampled trajectory validation (2026-09-24)

`uf850_smoke.py --plan` now checks both returned trajectories using
`validate_path.py`: original URDF collision meshes/analytic primitives via
FCL, existing SRDF exclusions only, joint bounds, start/goal consistency,
and a +/-30 degree X/Y rotation-vector diagnostic relative to the goal TCP.
It checks every interpolated sample and subdivides joint steps larger than
0.005 rad. Failed validation makes the command fail. This is not continuous
collision detection or proof of MoveIt path-constraint compatibility.

The observation-to-joint1+0.1 rad smoke case passed both independent checks:

| Goal | GPU planning wall time | Checked samples | FCL/check time | EEF length |
| --- | --- | --- | --- | --- |
| Joint | 0.0726 s | 51 | 0.0118 s | 0.04985 m |
| Pose | 0.2484 s | 44 | 0.0100 s | 0.04984 m |

Initialization/warmup was 6.779 s. These are single-run measurements of a
small movement, not representative pallet/slot transfers. Scene is still
only a synthetic floor; camera, payload, pallet and real table are absent.
No robot motion was executed. Orientation checking rejects an invalid path;
it does not make cuMotion generate a constrained path.

CPU regression tests (including intentional collision, joint-bound and
orientation violations):

```bash
docker run --rm --network none \
  -v "$PWD/experiments/cumotion:/experiment:ro" \
  uf850-cumotion-test:4.0 -lc \
  'source /opt/ros/jazzy/setup.bash && cd /experiment && PYTHONDONTWRITEBYTECODE=1 python3 -m unittest -v test_sphere_cover test_validate_path'
```

### Gates before production integration

1. DONE: install dependencies and run a real CUDA/cuRobo planning smoke test.
2. Export the patched UF850 URDF and create/review XRDF collision spheres,
   including tool/camera; compare FK and joint ordering with MoveIt.
3. Sync pallet, table, packed boxes and attached payload geometry.
4. Implement compatible orientation handling and independently validate the
   returned path against the MoveIt model, limits, collisions and tilt bound.
5. Benchmark identical start/goal pairs: success, planning time, EEF length,
   joint continuity and tilt. No trajectory execution during these tests.
6. Only after review, enable a separately selected production pipeline;
   retain existing Cartesian lift/descent and force-contact handling.

## References

- https://nvidia-isaac-ros.github.io/v/release-4.0/getting_started/index.html
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion/tree/1acd5d89d0b32b8972e614ff149996803a2d3f45
# Logged joint-goal failure: timing reproduction (2026-09-24)

The isolated plan-only adapter now exposes `joint_trajectory_max_dt` (seconds,
default `0.30`). This controls trajectory optimization timing, not request timeout
and not a joint displacement cap. Joint limits and dynamics/collision checking
remain enabled. Restart the experimental process to load it; no image rebuild
is needed for the bind-mounted Python files.

For the logged six-joint goal, the previous default maximum timestep of 0.15 s
gave `TRAJOPT_FAIL`, although graph-only diagnostic search succeeded. With 0.30 s,
normal joint trajectory optimization succeeded in approximately 0.08 s offline.
`reproduce_joint_failure.py --model MODEL_DIRECTORY [--dt 0.3]` reproduces this
comparison without execution. Graph-only results are diagnostic, not an execution
fallback.

The same target also succeeded through the experimental MoveIt pipeline. Independent
original-mesh validation sampled 931 states without collision in the synthetic floor
scene. However, the trajectory failed the separate 30-degree tilt diagnostic
(approximately 58/44 degrees). This is **not** a passed end-to-end packing test:
the experimental adapter still rejects orientation path constraints, and neither
the actual packing scene nor carried payload was validated. Robot execution remains
disabled. `panel_smoke.py` accepts `CUMOTION_TEST_GOAL_JSON` for reproducing a six-angle
joint target and still fails validation if the tilt check is violated.
