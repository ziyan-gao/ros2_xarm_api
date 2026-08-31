#!/usr/bin/env bash

# Phase 1 commissioning entry point. This launch owns the robot connection via
# ros2_control and must not run at the same time as start_all.sh.
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/xarm_ws/install/setup.bash

: "${ROBOT_IP:=192.168.1.232}"
: "${XARM_REPORT_TYPE:=rich}"
: "${MOVEIT_VELOCITY_SCALING:=0.10}"
: "${MOVEIT_ACCELERATION_SCALING:=0.10}"

case "${MOVEIT_VELOCITY_SCALING}" in
  0|0.*|1|1.0) ;;
  *) echo "MOVEIT_VELOCITY_SCALING must be between 0 and 1" >&2; exit 2 ;;
esac
case "${MOVEIT_ACCELERATION_SCALING}" in
  0|0.*|1|1.0) ;;
  *) echo "MOVEIT_ACCELERATION_SCALING must be between 0 and 1" >&2; exit 2 ;;
esac

echo "Starting isolated UF850 MoveIt commissioning stack"
echo "Robot: ${ROBOT_IP}; velocity scale: ${MOVEIT_VELOCITY_SCALING}; acceleration scale: ${MOVEIT_ACCELERATION_SCALING}"
echo "Do not start start_all.sh or another robot driver concurrently."

# The xarm_planner real-move launch includes the UF850 hardware interface,
# trajectory controller, robot_state_publisher, move_group, RViz, and planner
# service node. Scaling defaults are supplied to move_group; the RViz panel
# should also be kept at or below these values during commissioning.
exec ros2 launch xarm_planner uf850_planner_realmove.launch.py \
  robot_ip:="${ROBOT_IP}" \
  report_type:="${XARM_REPORT_TYPE}" \
  add_vacuum_gripper:=true \
  velocity_scaling:="${MOVEIT_VELOCITY_SCALING}" \
  acceleration_scaling:="${MOVEIT_ACCELERATION_SCALING}"
