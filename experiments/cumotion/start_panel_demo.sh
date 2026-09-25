#!/usr/bin/env bash
# Container-only entrypoint. Host must use --network none and no device mounts.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/xarm_ws/install/setup.bash
source /opt/cumotion_plugin_overlay/setup.bash
set -u
export ROS_DOMAIN_ID=87 ISAAC_ROS_WS=/tmp PYTHONDONTWRITEBYTECODE=1
echo 'SYNTHETIC DEMO ONLY: saved observation start + synthetic floor; NOT the packing scene.'
echo 'Camera, payload, pallet and actual tables are NOT loaded. Robot execution is disabled.'
demo_dir=$(mktemp -d /tmp/cumotion-panel.XXXXXX)
export CUMOTION_TEST_MODEL="$demo_dir/model"
python3 /experiment/uf850_smoke.py --waypoints /waypoints.yaml \
  --joint3-min-deg "${CUMOTION_JOINT3_MIN_DEG:--130.0}" \
  --joint5-max-deg "${CUMOTION_JOINT5_MAX_DEG:-40.0}" \
  --export-model "$CUMOTION_TEST_MODEL"
pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT
ros2 run isaac_ros_cumotion static_planning_scene &
pids+=("$!")
python3 /experiment/panel_bridge.py --ros-args \
    -p yml_file_path:="$CUMOTION_TEST_MODEL/uf850.yml" -p tool_frame:=link_tcp \
    -p max_attempts:=2 -p read_esdf_world:=false -p add_ground_plane:=false &
pids+=("$!")
# Do not expose MoveIt until the GPU action server is ready.
python3 /experiment/wait_planner.py
ros2 launch /experiment/panel_demo.launch.py rviz:="${CUMOTION_TEST_RVIZ:-true}" &
pids+=("$!")
if [ "${CUMOTION_TEST_HEADLESS:-false}" = true ]; then
    python3 /experiment/panel_smoke.py
else
    python3 /experiment/panel_smoke.py --scene-only
    wait -n "${pids[@]}"
fi
