# Offline joint-planning benchmark — 2026-09-25

Historical comparison before the parameter wiring fix. The adapter now applies
`trajopt_finetune_iters` to JS finetune too, and the benchmark no longer needs
an experimental override. For current replay, omit `--js-finetune-iters` and
set the ROS parameter directly; old baseline results remain historical.

Scene captured from the running RViz/MoveIt environment. All replay ran in a separate GPU container with `--network none`; no trajectories were executed and the live service was not restarted.

## Result

Three representative joint-space pairs (pallet return to observation, observation to raised pallet target, observation to pre-pick reference), three measured repetitions per pair. One initial repetition per pair excluded. Values below are medians, seconds.

| Setting | Actual JS finetune iterations | Success | Full callback | Finetune |
|---|---:|---:|---:|---:|
| baseline300 | 400 | 9/9 | 2.994 | 2.286 |
| baseline50 | 400 | 9/9 | 2.988 | 2.290 |
| js100 | 100 | 9/9 | 1.281 | 0.564 |
| js50 | 50 | 9/9 | 1.003 | 0.279 |

`baseline300`/`baseline50` change the existing `trajopt_finetune_iters` ROS parameter. `js100`/`js50` keep that parameter at 50 and override only construction of the joint-space finetune solver inside the benchmark process. Main trajectory optimization and collision geometry/checks are unchanged.

The installed cuRobo `motion_gen.py` passes `finetune_trajopt_iters` to the pose finetune solver (line 807), but `grad_trajopt_iters` to the joint finetune solver (line 849). The application only sets the former. The measured optimizer therefore remains at 400 iterations when the ROS parameter changes 300 → 50. This explains the unchanged latency.

Actual JS finetune 100 cuts the measured callback latency about 57%; 50 cuts it about 66%. A separate JS finetune setting, applied to the correct solver, is the useful next production change. Start evaluation at 100; consider 50 after more loaded-scene cases. No production tuning was applied by this experiment.

## Scope and trajectory differences

Captured world: pallet surface, work table, secondary table, two walls; attached fixed camera; no carried box or placed items. Existing virtual pallet/slot barriers were included. The same shared GPU also hosted the idle live stack. Pose CUDA warmup was skipped only in the benchmark to fit GPU memory; measured joint cases were warmed.

This times the planning backend callback, not application Cartesian generation, combined trajectory retiming, MoveIt response validation, controller settling, or execution. It does not reproduce the entire reported 5–10 second delay. Successful results mean backend acceptance; no hardware or full application path commissioning was performed. Three pairs and nine measured samples per setting do not establish general reliability.

Generated trajectories differ between settings/runs; timing improvements do not establish shorter motion or cycle times. Median trajectory durations:

| Setting | Return | Transfer | Pre-pick reference |
|---|---:|---:|---:|
| baseline300 | 3.527 | 4.013 | 3.888 |
| baseline50 | 4.963 | 4.224 | 4.477 |
| js100 | 4.629 | 3.730 | 3.894 |
| js50 | 4.971 | 4.226 | 5.278 |

## Artifacts and replay

- Script: `benchmark_finetune.py` (local process instrumentation only).
- Scene: `results/planning_speed_scene.json`.
- Exported model: `results/planning_speed_model/`.
- Measurements: `results/finetune_{baseline300,baseline50,js100,js50}.json`.
- Raw artifacts are under the existing ignored `results/` directory.

Inside a **separate `--network none` GPU container**, mount the repo at `/workspace`, source ROS and xArm, set `ISAAC_ROS_WS=/tmp` and `CUMOTION_TEST_MODEL=/workspace/experiments/cumotion/results/planning_speed_model`, and run the static scene service (`ros2 run isaac_ros_cumotion static_planning_scene`). Then:

```bash
python3 /workspace/experiments/cumotion/benchmark_finetune.py \
  --scene /workspace/experiments/cumotion/results/planning_speed_scene.json \
  --output /tmp/js100.json \
  --ros-args -p yml_file_path:=$CUMOTION_TEST_MODEL/uf850.yml \
  -p tool_frame:=link_tcp -p max_attempts:=2 \
  -p trajopt_finetune_iters:=100 -p parallel_finetune:=false \
  -p read_esdf_world:=false -p add_ground_plane:=false
```

Keep the container isolated: the adapter also publishes scene/visualization data.

## Verification after wiring fix

Replayed the same scene using the production adapter initialization, with no
experimental solver override. All 9 measured plans succeeded, actual optimizer
iterations stayed at 50 (2 × 25); callback median 1.010 s.
Raw results: `results/finetune_wired50.json`. The live process was not restarted.
