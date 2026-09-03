#!/usr/bin/env bash

# Unified real-robot stack. MoveIt/ros2_control exclusively owns the robot
# connection and trajectory controller. Do not add xarm_driver_node or the
# direct-SDK safe_servo_controller here without explicit controller handoff.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/xarm_ws/install/setup.bash

cd /workspace/ws
colcon build --symlink-install
source install/setup.bash
set -u

: "${ROBOT_IP:=192.168.1.232}"
: "${XARM_REPORT_TYPE:=dev}"
: "${RVIZ_CONFIG:=/workspace/ws/src/rviz_config/rviz_scene.rviz}"
: "${MOVEIT_SERVO_DRY_RUN:=false}"
: "${SERVO_DESCENT_SPEED_M_S:=0.05}"
: "${SERVO_DESCENT_KP_Z:=3.0}"
: "${PLACE_FORCE_THRESHOLD_N:=4.0}"

if [[ "${XARM_REPORT_TYPE}" != "dev" ]]; then
  echo "FATAL: XARM_REPORT_TYPE must be 'dev' for non-blocking 100 Hz joint feedback." >&2
  exit 4
fi

if [[ ! -r "${RVIZ_CONFIG}" ]]; then
  echo "RViz configuration is not readable: ${RVIZ_CONFIG}" >&2
  exit 2
fi

# A non-real-time controller_manager previously missed tens of Servo-J write
# cycles and then caught up abruptly.  Refuse to connect to the real robot if
# the container cannot create a FIFO thread; compose.yaml grants this narrowly
# through SYS_NICE and RLIMIT_RTPRIO.
if ! chrt --fifo 1 true >/dev/null 2>&1; then
  echo "FATAL: FIFO real-time scheduling is unavailable; refusing to start robot control." >&2
  echo "Start this stack through compose.yaml with SYS_NICE and rtprio enabled." >&2
  exit 3
fi
echo "Real-time scheduling preflight passed."

child_pids=()

start_required() {
  echo "Starting $1"
  shift
  "$@" &
  child_pids+=("$!")
}

stop_children() {
  if ((${#child_pids[@]})); then
    kill -TERM "${child_pids[@]}" 2>/dev/null || true
    wait "${child_pids[@]}" 2>/dev/null || true
  fi
}
trap stop_children EXIT INT TERM

# This is a short-lived connection made before ros2_control connects. It does
# not remain as a second robot owner once MoveIt starts.
echo "Enabling and zeroing the UFACTORY force-torque sensor"
ros2 run ft_sensor_publish ft_sensor_initializer

start_required "UF850 MoveIt/ros2_control stack (${ROBOT_IP})" \
  ros2 launch xarm_planner uf850_planner_realmove.launch.py \
    robot_ip:="${ROBOT_IP}" \
    report_type:="${XARM_REPORT_TYPE}" \
    add_vacuum_gripper:=true \
    rviz_config:="${RVIZ_CONFIG}"

start_required "RealSense camera" \
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

# Run only the application node. visualization.launch.py also creates a robot
# state publisher whose model and TF would conflict with MoveIt's publisher.
start_required "camera calibration TF and force visualization" \
  ros2 run safe_servo_visualization visualization_node

start_required "taught-waypoint storage" \
  ros2 run safe_servo_visualization waypoint_store
start_required "MoveIt motion coordinator" \
  ros2 run safe_servo_visualization motion_coordinator
start_required "MoveIt planning-scene obstacles" \
  ros2 run safe_servo_visualization planning_scene_obstacles
start_required "pickup pipeline orchestrator" \
  ros2 run safe_servo_visualization pickup_pipeline
start_required "place pipeline orchestrator" \
  ros2 run safe_servo_visualization place_pipeline
start_required "supervised Phase 4 pickup coordinator" \
  ros2 run safe_servo_visualization pickup_supervisor --ros-args \
    -p place_force_contact_threshold_n:="${PLACE_FORCE_THRESHOLD_N}"
start_required "MoveIt Servo and guarded vertical bridge (dry_run=${MOVEIT_SERVO_DRY_RUN}, descent_speed=${SERVO_DESCENT_SPEED_M_S}m/s)" \
  ros2 launch safe_servo_package uf850_moveit_servo.launch.py \
    dry_run:="${MOVEIT_SERVO_DRY_RUN}" \
    max_linear_speed:="${SERVO_DESCENT_SPEED_M_S}" \
    kp_z:="${SERVO_DESCENT_KP_Z}"
start_required "ArUco box marker detector" \
  ros2 run box_marker_detection box_marker_detector
start_required "aligned-depth box geometry refinement" \
  ros2 run box_marker_detection depth_box_refinement
start_required "pallet localization supervisor" \
  ros2 run safe_servo_visualization pallet_localization
start_required "incoming-item localization supervisor" \
  ros2 run safe_servo_visualization item_localization

echo "Unified stack is running; MoveIt is the sole motion owner."
echo "Direct-SDK safe servo is disabled; guarded descent uses the real MoveIt Servo bridge."

# Every component is required. If one exits, stop the stack instead of leaving
# a partially functioning robot application running.
wait -n "${child_pids[@]}"
