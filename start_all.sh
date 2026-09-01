#!/usr/bin/env bash

set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/xarm_ws/install/setup.bash

cd /workspace/ws
# The source workspace is bind-mounted, so rebuild incrementally on startup to
# ensure newly added nodes and RViz plugins are always available.
colcon build --symlink-install
source install/setup.bash
set -u

child_pids=()

stop_children() {
  if ((${#child_pids[@]})); then
    kill -TERM "${child_pids[@]}" 2>/dev/null || true
    wait "${child_pids[@]}" 2>/dev/null || true
  fi
}
trap stop_children EXIT INT TERM

echo "Enabling UFACTORY force-torque sensor"
ros2 run ft_sensor_publish ft_sensor_initializer

echo "Starting UFACTORY 850 driver at ${ROBOT_IP} (report_type=${XARM_REPORT_TYPE})"
ros2 run xarm_api xarm_driver_node --ros-args \
  --remap __node:=ufactory_driver \
  --params-file /opt/xarm_ws/install/xarm_api/share/xarm_api/config/xarm_params.yaml \
  -p robot_ip:="${ROBOT_IP}" \
  -p report_type:="${XARM_REPORT_TYPE}" \
  -p dof:=6 \
  -p hw_ns:=ufactory &
child_pids+=("$!")

echo "Starting RealSense camera"
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true &
child_pids+=("$!")

echo "Starting robot, camera, and force visualization TF"
ros2 launch safe_servo_visualization visualization.launch.py &
child_pids+=("$!")

echo "Starting safe servo controller (dry_run=${SAFE_SERVO_DRY_RUN})"
ros2 run safe_servo_package safe_servo_controller --ros-args \
  -p dry_run:="${SAFE_SERVO_DRY_RUN}" \
  -p robot_ip:="${ROBOT_IP}" &
child_pids+=("$!")

echo "Starting taught-waypoint storage"
ros2 run safe_servo_visualization waypoint_store &
child_pids+=("$!")

echo "Starting ArUco box marker detector"
ros2 run box_marker_detection box_marker_detector &
child_pids+=("$!")

echo "Starting aligned-depth box geometry refinement"
ros2 run box_marker_detection depth_box_refinement &
child_pids+=("$!")

echo "Starting pallet localization supervisor"
ros2 run safe_servo_visualization pallet_localization &
child_pids+=("$!")

echo "Starting incoming-item localization supervisor"
ros2 run safe_servo_visualization item_localization &
child_pids+=("$!")

echo "Starting RViz with ${RVIZ_CONFIG}"
# Avoid loading the panel while its ROS services and TF publishers are still
# being constructed during the same scheduler burst.
sleep 3
rviz2 -d "${RVIZ_CONFIG}" &
child_pids+=("$!")

# Treat every process, including the safety UI, as required.
wait -n "${child_pids[@]}"
