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
: "${PICKUP_FORCE_THRESHOLD_N:=5.0}"
: "${PLACE_FORCE_THRESHOLD_N:=4.0}"
: "${PLACE_DESCENT_TIMEOUT_SEC:=20.0}"
: "${PLACE_SINGULARITY_RECOVERY_TIMEOUT_SEC:=20.0}"
: "${PLACE_SINGULARITY_STEP_M:=0.003}"
: "${PLACE_SINGULARITY_STEP_SPEED_MM_S:=10.0}"
: "${DIRECT_CARTESIAN_MAX_SPEED_MM_S:=200.0}"
: "${DIRECT_CARTESIAN_MAX_ACCEL_MM_S2:=500.0}"
: "${CONTINUOUS_TRANSPORT_ENABLED:=true}"
: "${CONTINUOUS_RETURN_ENABLED:=true}"
: "${CONTINUOUS_TRANSPORT_BLEND_RADIUS_M:=0.04}"
: "${CONTINUOUS_TRANSPORT_MAX_JOINT_JERK_RAD_S3:=10.0}"
: "${CONTINUOUS_TRANSPORT_ENFORCE_JERK_LIMIT:=false}"
: "${RANDOM_LOADING_AUTO_START:=false}"
: "${PICK_PLACE_TEST_MAX_STEPS:=100}"
: "${PICK_PLACE_TEST_SEED:=-1}"
: "${RANDOM_LOADING_CONFIG_PATH:=/opt/neuromeka_bin_packing/configs/real_platform_random.yaml}"
: "${STAGING_TRANSFER_BOTTOM_ABOVE_PALLET_M:=0.480}"
: "${PLACE_WORKSPACE_Z_MIN_MM:=-100.0}"
: "${TRANSPORT_WORKSPACE_Z_MAX_MM:=800.0}"
: "${RANDOM_LOADING_VISUALIZE:=true}"
: "${RANDOM_LOADING_VISUAL_PORT:=8765}"
: "${POLICY_LOADING_CHECKPOINT:=/opt/neuromeka_bin_packing/train_outputs/cardboard_xy_random_clearance20/policy_step.pth}"
: "${POLICY_LOADING_DEVICE:=cpu}"
: "${POLICY_LOADING_CONFIG_PATH:=/opt/neuromeka_bin_packing/configs/real_platform_policy.yaml}"
: "${POLICY_LOADING_VISUALIZE:=true}"
: "${POLICY_LOADING_VISUAL_PORT:=8766}"
: "${DEPTH_ONLY_SUPPORT_Z_M:=0.0}"
: "${DEPTH_ONLY_HEIGHT_OFFSET_M:=-0.02}"
: "${OBJECT_CONTACT_REFERENCE_Z_M:=0.0}"
: "${OBJECT_INFO_STABLE_SAMPLES:=20}"
: "${DEPTH_ONLY_BASE_Z_MIN_M:=0.07}"
: "${DEPTH_ONLY_BASE_Z_MAX_M:=0.30}"
: "${DEPTH_ONLY_TCP_X_MIN_M:=0.05}"
: "${DEPTH_ONLY_TCP_X_MAX_M:=0.40}"
: "${DEPTH_ONLY_TCP_Y_MIN_M:=-0.30}"
: "${DEPTH_ONLY_TCP_Y_MAX_M:=0.30}"

if [[ "${XARM_REPORT_TYPE}" != "dev" ]]; then
  echo "FATAL: XARM_REPORT_TYPE must be 'dev' for non-blocking 100 Hz joint feedback." >&2
  exit 4
fi

if [[ ! -r "${RVIZ_CONFIG}" ]]; then
  echo "RViz configuration is not readable: ${RVIZ_CONFIG}" >&2
  exit 2
fi

if [[ ! -r "${RANDOM_LOADING_CONFIG_PATH}" ]]; then
  echo "Random-loading configuration is not readable: ${RANDOM_LOADING_CONFIG_PATH}" >&2
  exit 2
fi

# Load the random-packing geometry once and pass the same validated values to
# both the packing node and robot-motion nodes. This keeps a changed container
# height synchronized with the collision-clear transfer/lift height.
random_loading_fields="$(
  python3 -m packing.real_platform_random_config \
    --config "${RANDOM_LOADING_CONFIG_PATH}"
)"
IFS=$'\t' read -r \
  RANDOM_LOADING_CONTAINER_CSV \
  RANDOM_LOADING_CLEARANCE_MM \
  RANDOM_LOADING_CLEARANCE_MODE \
  RANDOM_LOADING_SEED \
  RANDOM_LOADING_SCAN_DOWNSCALE \
  RANDOM_LOADING_COM_BOUND_RATIO \
  RANDOM_LOADING_HEIGHT_TOLERANCE_MM \
  RANDOM_LOADING_VERTICAL_FILTER_ENABLED \
  RANDOM_LOADING_HEIGHT_RESOLUTION_MM \
  TRANSFER_CORNER_HEIGHT_M \
  RANDOM_LOADING_USE_FM <<< "${random_loading_fields}"

if [[ -z "${RANDOM_LOADING_CONTAINER_CSV}" ||
      -z "${RANDOM_LOADING_CLEARANCE_MM}" ||
      -z "${RANDOM_LOADING_CLEARANCE_MODE}" ||
      -z "${RANDOM_LOADING_SEED}" ||
      -z "${RANDOM_LOADING_SCAN_DOWNSCALE}" ||
      -z "${RANDOM_LOADING_COM_BOUND_RATIO}" ||
      -z "${RANDOM_LOADING_HEIGHT_TOLERANCE_MM}" ||
      -z "${RANDOM_LOADING_VERTICAL_FILTER_ENABLED}" ||
      -z "${RANDOM_LOADING_HEIGHT_RESOLUTION_MM}" ||
      -z "${TRANSFER_CORNER_HEIGHT_M}" ||
      -z "${RANDOM_LOADING_USE_FM}" ]]; then
  echo "FATAL: random-loading configuration returned incomplete startup values." >&2
  exit 2
fi
RANDOM_LOADING_CONTAINER_ROS="[${RANDOM_LOADING_CONTAINER_CSV//,/, }]"
echo "Random loading feasibility-map mask: use_fm=${RANDOM_LOADING_USE_FM}"
echo "Random loading config: container=${RANDOM_LOADING_CONTAINER_ROS} mm, clearance=${RANDOM_LOADING_CLEARANCE_MM} mm (${RANDOM_LOADING_CLEARANCE_MODE}), height tolerance=${RANDOM_LOADING_HEIGHT_TOLERANCE_MM} mm, vertical filter=${RANDOM_LOADING_VERTICAL_FILTER_ENABLED}, transfer corner height=${TRANSFER_CORNER_HEIGHT_M} m"

# Resolve the policy's grid before importing any Neuromeka geometry in that
# process. Do not export this override to random loading or other ROS nodes.
policy_geometry_fields="$(python3 -m packing.real_platform_policy_config --config "${POLICY_LOADING_CONFIG_PATH}")"
IFS=$'\t' read -r POLICY_CLEARANCE_MM POLICY_XY_RESOLUTION_MM <<< "${policy_geometry_fields}"
if [[ -z "${POLICY_CLEARANCE_MM}" || -z "${POLICY_XY_RESOLUTION_MM}" ]]; then
  echo "FATAL: policy geometry configuration is incomplete." >&2
  exit 2
fi
echo "Policy config: clearance=${POLICY_CLEARANCE_MM} mm, XY resolution=${POLICY_XY_RESOLUTION_MM} mm"

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

start_required "observation waypoint storage" \
  ros2 run safe_servo_visualization waypoint_store

start_required "MoveIt motion coordinator" \
  ros2 run safe_servo_visualization motion_coordinator --ros-args \
    -p transfer_corner_height_m:="${TRANSFER_CORNER_HEIGHT_M}"
start_required "MoveIt planning-scene obstacles" \
  ros2 run safe_servo_visualization planning_scene_obstacles
start_required "six-slot unpacking staging coordinator" \
  ros2 run safe_servo_visualization staging_slots --ros-args \
    -p slot_inspection_policy_config_path:="${POLICY_LOADING_CONFIG_PATH}" \
    -p slot_inspection_enabled:="${STAGING_SLOT_INSPECTION_ENABLED:-true}" \
    -p slot_inspection_backoff_m:="${STAGING_SLOT_INSPECTION_BACKOFF_M:-0.100}" \
    -p configured_container_clearance_m:="${TRANSFER_CORNER_HEIGHT_M}" \
    -p transfer_item_bottom_above_pallet_m:="${STAGING_TRANSFER_BOTTOM_ABOVE_PALLET_M}"
start_required "pickup pipeline orchestrator" \
  ros2 run safe_servo_visualization pickup_pipeline --ros-args \
    -p detection_stable_frames:="${OBJECT_INFO_STABLE_SAMPLES}"
start_required "place pipeline orchestrator" \
  ros2 run safe_servo_visualization place_pipeline
start_required "combined PickAndPlace orchestrator" \
  ros2 run safe_servo_visualization pick_place_pipeline --ros-args \
    -p continuous_transport_enabled:="${CONTINUOUS_TRANSPORT_ENABLED}"
start_required "supervised Phase 4 pickup coordinator" \
  ros2 run safe_servo_visualization pickup_supervisor --ros-args \
    -p continuous_return_enabled:="${CONTINUOUS_RETURN_ENABLED}" \
    -p continuous_transport_blend_radius_m:="${CONTINUOUS_TRANSPORT_BLEND_RADIUS_M}" \
    -p continuous_transport_alternatives_enabled:="${CONTINUOUS_TRANSPORT_ALTERNATIVES_ENABLED:-true}" \
    -p transport_moveit_fallback_enabled:="${TRANSPORT_MOVEIT_FALLBACK_ENABLED:-false}" \
    -p transport_expanded_waypoints_enabled:="${TRANSPORT_EXPANDED_WAYPOINTS_ENABLED:-true}" \
    -p transport_waypoint_search_timeout_sec:="${TRANSPORT_WAYPOINT_SEARCH_TIMEOUT_SEC:-180.0}" \
    -p transport_observation_y_offset_mm:="${TRANSPORT_OBSERVATION_Y_OFFSET_MM:-200.0}" \
    -p raised_pre_place_max_mm:="${RAISED_PRE_PLACE_MAX_MM:-30.0}" \
    -p raised_pre_pick_max_mm:="${RAISED_PRE_PICK_MAX_MM:-30.0}" \
    -p continuous_transport_max_joint_jerk_rad_s3:="${CONTINUOUS_TRANSPORT_MAX_JOINT_JERK_RAD_S3}" \
    -p continuous_transport_enforce_jerk_limit:="${CONTINUOUS_TRANSPORT_ENFORCE_JERK_LIMIT}" \
    -p force_contact_threshold_n:="${PICKUP_FORCE_THRESHOLD_N}" \
    -p place_force_contact_threshold_n:="${PLACE_FORCE_THRESHOLD_N}" \
    -p place_descent_timeout_sec:="${PLACE_DESCENT_TIMEOUT_SEC}" \
    -p direct_transfer_max_joint_delta_rad:=3.141592653589793 \
    -p singularity_place_recovery_timeout_sec:="${PLACE_SINGULARITY_RECOVERY_TIMEOUT_SEC}" \
    -p singularity_place_step_m:="${PLACE_SINGULARITY_STEP_M}" \
    -p singularity_place_step_speed_mm_s:="${PLACE_SINGULARITY_STEP_SPEED_MM_S}" \
    -p singularity_deceleration_grace_sec:="${PLACE_SINGULARITY_DECELERATION_GRACE_SEC}" \
    -p singularity_no_progress_sec:="${PLACE_SINGULARITY_NO_PROGRESS_SEC}" \
    -p post_restore_settle_sec:="${POST_RESTORE_SETTLE_SEC}" \
    -p direct_transfer_ik_timeout_sec:="${DIRECT_TRANSFER_IK_TIMEOUT_SEC}" \
    -p direct_cartesian_max_speed_mm_s:="${DIRECT_CARTESIAN_MAX_SPEED_MM_S}" \
    -p direct_cartesian_max_acc_mm_s2:="${DIRECT_CARTESIAN_MAX_ACCEL_MM_S2}" \
    -p transfer_corner_height_m:="${TRANSFER_CORNER_HEIGHT_M}" \
    -p contact_reference_z_m:="${OBJECT_CONTACT_REFERENCE_Z_M}" \
    -p transport_workspace_z_max_mm:="${TRANSPORT_WORKSPACE_Z_MAX_MM}" \
    -p place_workspace_z_min_mm:="${PLACE_WORKSPACE_Z_MIN_MM}"
start_required "MoveIt Servo and guarded vertical bridge (dry_run=${MOVEIT_SERVO_DRY_RUN}, descent_speed=${SERVO_DESCENT_SPEED_M_S}m/s)" \
  ros2 launch safe_servo_package uf850_moveit_servo.launch.py \
    dry_run:="${MOVEIT_SERVO_DRY_RUN}" \
    max_linear_speed:="${SERVO_DESCENT_SPEED_M_S}" \
    kp_z:="${SERVO_DESCENT_KP_Z}"
start_required "ArUco box marker detector" \
  ros2 run box_marker_detection box_marker_detector
start_required "aligned-depth box geometry refinement" \
  ros2 run box_marker_detection depth_box_refinement --ros-args \
    -p depth_only_support_z_m:="${DEPTH_ONLY_SUPPORT_Z_M}" \
    -p depth_only_height_offset_m:="${DEPTH_ONLY_HEIGHT_OFFSET_M}" \
    -p depth_only_base_z_min_m:="${DEPTH_ONLY_BASE_Z_MIN_M}" \
    -p depth_only_base_z_max_m:="${DEPTH_ONLY_BASE_Z_MAX_M}" \
    -p depth_only_tcp_x_min_m:="${DEPTH_ONLY_TCP_X_MIN_M}" \
    -p depth_only_tcp_x_max_m:="${DEPTH_ONLY_TCP_X_MAX_M}" \
    -p depth_only_tcp_y_min_m:="${DEPTH_ONLY_TCP_Y_MIN_M}" \
    -p depth_only_tcp_y_max_m:="${DEPTH_ONLY_TCP_Y_MAX_M}"
start_required "pallet localization supervisor" \
  ros2 run safe_servo_visualization pallet_localization --ros-args \
    -p auto_load_saved_pose:=true
start_required "incoming-item localization supervisor" \
  ros2 run safe_servo_visualization item_localization
start_required "real-platform random stable-loading coordinator" \
  ros2 run safe_servo_visualization random_stable_loading --ros-args \
    -p container_size_mm:="${RANDOM_LOADING_CONTAINER_ROS}" \
    -p packing_height_resolution_mm:="${RANDOM_LOADING_HEIGHT_RESOLUTION_MM}" \
    -p clearance_mm:="${RANDOM_LOADING_CLEARANCE_MM}" \
    -p clearance_mode:="${RANDOM_LOADING_CLEARANCE_MODE}" \
    -p seed:="${RANDOM_LOADING_SEED}" \
    -p scan_downscale:="${RANDOM_LOADING_SCAN_DOWNSCALE}" \
    -p com_bound_ratio:="${RANDOM_LOADING_COM_BOUND_RATIO}" \
    -p height_tolerance:="${RANDOM_LOADING_HEIGHT_TOLERANCE_MM}" \
    -p use_fm:="${RANDOM_LOADING_USE_FM}" \
    -p vertical_loading_filter_enabled:="${RANDOM_LOADING_VERTICAL_FILTER_ENABLED}" \
    -p transfer_corner_height_m:="${TRANSFER_CORNER_HEIGHT_M}" \
    -p random_loading_config_path:="${RANDOM_LOADING_CONFIG_PATH}" \
    -p auto_start_pick_place:="${RANDOM_LOADING_AUTO_START}" \
    -p visualization_enabled:="${RANDOM_LOADING_VISUALIZE}" \
    -p visualization_port:="${RANDOM_LOADING_VISUAL_PORT}"
start_required "real-platform learned-policy coordinator" \
  env NEUROMEKA_XY_RESOLUTION_MM="${POLICY_XY_RESOLUTION_MM}" \
  ros2 run safe_servo_visualization policy_loading --ros-args \
    -p policy_config_path:="${POLICY_LOADING_CONFIG_PATH}" \
    -p packing_height_resolution_mm:=5 \
    -p checkpoint_path:="${POLICY_LOADING_CHECKPOINT}" \
    -p policy_device:="${POLICY_LOADING_DEVICE}" \
    -p auto_start_pick_place:=false \
    -p visualization_enabled:="${POLICY_LOADING_VISUALIZE}" \
    -p visualization_port:="${POLICY_LOADING_VISUAL_PORT}"

# Inert until an explicit test-panel button is pressed. Uses the same random
# loading geometry configuration, without changing either loader's inventory.
start_required "single-item pack/unpack/repack test coordinator" \
  ros2 run safe_servo_visualization pick_place_test --ros-args \
    -p random_test_max_steps:="${PICK_PLACE_TEST_MAX_STEPS}" \
    -p random_test_seed:="${PICK_PLACE_TEST_SEED}" \
    -p container_size_mm:="${RANDOM_LOADING_CONTAINER_ROS}" \
    -p packing_height_resolution_mm:="${RANDOM_LOADING_HEIGHT_RESOLUTION_MM}" \
    -p clearance_mm:="${RANDOM_LOADING_CLEARANCE_MM}" \
    -p clearance_mode:="${RANDOM_LOADING_CLEARANCE_MODE}" \
    -p seed:="${RANDOM_LOADING_SEED}" \
    -p scan_downscale:="${RANDOM_LOADING_SCAN_DOWNSCALE}" \
    -p com_bound_ratio:="${RANDOM_LOADING_COM_BOUND_RATIO}" \
    -p height_tolerance:="${RANDOM_LOADING_HEIGHT_TOLERANCE_MM}" \
    -p use_fm:="${RANDOM_LOADING_USE_FM}" \
    -p vertical_loading_filter_enabled:="${RANDOM_LOADING_VERTICAL_FILTER_ENABLED}" \
    -p transfer_corner_height_m:="${TRANSFER_CORNER_HEIGHT_M}"

echo "Unified stack is running; MoveIt is the sole motion owner."
echo "Direct-SDK safe servo is disabled; guarded descent uses the real MoveIt Servo bridge."

# Every component is required. If one exits, stop the stack instead of leaving
# a partially functioning robot application running.
wait -n "${child_pids[@]}"
