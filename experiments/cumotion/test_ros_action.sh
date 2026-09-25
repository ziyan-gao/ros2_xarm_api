#!/usr/bin/env bash
# Run INSIDE the isolated GPU image. No controller/driver is launched.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/xarm_ws/install/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=87
export ISAAC_ROS_WS=/tmp
model_dir=$(mktemp -d /tmp/uf850-ros-model.XXXXXX)
python3 /experiment/uf850_smoke.py --waypoints /waypoints.yaml --export-model "$model_dir/model"
ros2 run isaac_ros_cumotion static_planning_scene &
scene_pid=$!
trap 'kill "$scene_pid" 2>/dev/null || true; wait "$scene_pid" 2>/dev/null || true' EXIT
python3 /experiment/ros_plan_only.py --ros-args -r __ns:=/cumotion_test \
  -p yml_file_path:="$model_dir/model/uf850.yml" -p tool_frame:=link_tcp \
  -p max_attempts:=2 -p read_esdf_world:=false -p add_ground_plane:=false &
planner_pid=$!
trap 'kill "$scene_pid" "$planner_pid" 2>/dev/null || true; wait "$scene_pid" "$planner_pid" 2>/dev/null || true' EXIT
python3 /experiment/ros_client_smoke.py --model "$model_dir/model" &
client_pid=$!
trap 'kill "$scene_pid" "$client_pid" "$planner_pid" 2>/dev/null || true; wait "$scene_pid" "$client_pid" "$planner_pid" 2>/dev/null || true' EXIT
wait -n -p completed_pid "$scene_pid" "$planner_pid" "$client_pid"
test "$completed_pid" = "$client_pid"
