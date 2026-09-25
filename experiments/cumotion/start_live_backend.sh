#!/usr/bin/env bash
# Only GPU planning server: no hardware connection, controllers or execution.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/xarm_ws/install/setup.bash
source /opt/cumotion_plugin_overlay/setup.bash
export ISAAC_ROS_WS=/tmp PYTHONDONTWRITEBYTECODE=1
live_model_dir=$(mktemp -d /tmp/cumotion-live.XXXXXX)
export CUMOTION_TEST_MODEL="$live_model_dir/model"
python3 /workspace/experiments/cumotion/uf850_smoke.py \
  --waypoints /workspace/config/taught_waypoints.yaml --sphere-model curobo \
  --sphere-pitch 0.025 \
  --joint3-min-deg "${CUMOTION_JOINT3_MIN_DEG:--130.0}" \
  --joint5-max-deg "${CUMOTION_JOINT5_MAX_DEG:-40.0}" \
  --export-model "$CUMOTION_TEST_MODEL"
ros2 run isaac_ros_cumotion static_planning_scene &
static_pid=$!
trap 'kill "$static_pid" 2>/dev/null || true' EXIT
python3 /workspace/experiments/cumotion/live_bridge.py --ros-args \
  -p yml_file_path:="$CUMOTION_TEST_MODEL/uf850.yml" -p tool_frame:=link_tcp \
  -p max_attempts:=2 \
  -p speed_multiplier:="${CUMOTION_SPEED_MULTIPLIER:-2.0}" \
  -p collision_cache_cuboid:="${CUMOTION_COLLISION_CACHE_CUBOID:-128}" \
  -p trajopt_finetune_iters:="${CUMOTION_TRAJOPT_FINETUNE_ITERS:-50}" \
  -p parallel_finetune:="${CUMOTION_PARALLEL_FINETUNE:-false}" \
  -p clearance_barriers_enabled:="${CUMOTION_CLEARANCE_BARRIERS_ENABLED:-false}" \
  -p pallet_clearance_z_m:="${CUMOTION_PALLET_CLEARANCE_Z_M:-0.470}" \
  -p slot_clearance_z_m:="${CUMOTION_SLOT_CLEARANCE_Z_M:-0.480}" \
  -p slot_surface_z_m:="${CUMOTION_SLOT_SURFACE_Z_M:-0.0}" \
  -p slot_x_min_m:="${CUMOTION_SLOT_X_MIN_M:--0.375}" \
  -p slot_x_max_m:="${CUMOTION_SLOT_X_MAX_M:-0.375}" \
  -p slot_y_min_m:="${CUMOTION_SLOT_Y_MIN_M:-0.180}" \
  -p slot_y_max_m:="${CUMOTION_SLOT_Y_MAX_M:-0.680}" \
  -p barrier_top_margin_m:="${CUMOTION_BARRIER_TOP_MARGIN_M:-0.003}" \
  -p read_esdf_world:=false -p add_ground_plane:=false
