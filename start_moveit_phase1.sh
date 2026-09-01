#!/usr/bin/env bash

# Phase 1 commissioning entry point. This launch owns the robot connection via
# ros2_control and must not run at the same time as start_all.sh.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/xarm_ws/install/setup.bash

cd /workspace/ws
colcon build --symlink-install \
  --packages-select safe_servo_visualization safe_servo_rviz_panel
source install/setup.bash
set -u

: "${ROBOT_IP:=192.168.1.232}"
: "${XARM_REPORT_TYPE:=rich}"
: "${RVIZ_CONFIG:=/workspace/ws/src/rviz_config/rviz_scene.rviz}"

if [[ ! -r "${RVIZ_CONFIG}" ]]; then
  echo "RViz configuration is not readable: ${RVIZ_CONFIG}" >&2
  exit 2
fi

echo "Starting isolated UF850 MoveIt commissioning stack"
echo "Robot: ${ROBOT_IP}; set RViz velocity and acceleration scaling to 0.10 before planning"
echo "Do not start start_all.sh or another robot driver concurrently."

# Waypoint storage is read-only with respect to the robot. It captures only
# fresh, stationary joint states and a matching TCP transform.
ros2 run safe_servo_visualization waypoint_store &
waypoint_store_pid=$!
ros2 run safe_servo_visualization motion_coordinator &
motion_coordinator_pid=$!
cleanup() {
  kill -TERM "${waypoint_store_pid}" "${motion_coordinator_pid}" \
    2>/dev/null || true
  wait "${waypoint_store_pid}" "${motion_coordinator_pid}" \
    2>/dev/null || true
}
trap cleanup EXIT INT TERM

# The xarm_planner real-move launch includes the UF850 hardware interface,
# trajectory controller, robot_state_publisher, move_group, RViz, and planner
# service node. The upstream launch has no startup argument for scaling, so it
# must be set explicitly in the RViz MotionPlanning panel before planning.
ros2 launch xarm_planner uf850_planner_realmove.launch.py \
  robot_ip:="${ROBOT_IP}" \
  report_type:="${XARM_REPORT_TYPE}" \
  add_vacuum_gripper:=true \
  rviz_config:="${RVIZ_CONFIG}"
