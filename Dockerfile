FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ARG XARM_ROS2_REF=57be2f40d4d198d1e552973b15fc26de6ebeed20

# 基础工具
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-rosdep \
    git \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-realsense2-camera \
    ros-jazzy-moveit \
    ros-jazzy-moveit-servo \
    ros-jazzy-pilz-industrial-motion-planner \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# 安装基础 Python 包
RUN python3 -m pip install --break-system-packages build pyproject_hooks && \
    python3 -m pip install --break-system-packages --no-deps opencv-python-headless

# 安装 xArm Python SDK（按官方 source code 方式）
RUN git clone https://github.com/xArm-Developer/xArm-Python-SDK.git /opt/xArm-Python-SDK && \
    cd /opt/xArm-Python-SDK && \
    python3 -m pip install --break-system-packages .

COPY patches/xarm_moveit_config_no_gazebo.patch /tmp/xarm_moveit_config_no_gazebo.patch
COPY patches/xarm_planner_low_speed.patch /tmp/xarm_planner_low_speed.patch
COPY patches/xarm_planner_display_path.patch /tmp/xarm_planner_display_path.patch
COPY patches/xarm_uf850_joint4_pi_limit.patch /tmp/xarm_uf850_joint4_pi_limit.patch
COPY patches/xarm_uf850_sensor_stack.patch /tmp/xarm_uf850_sensor_stack.patch
COPY patches/xarm_vacuum_services.patch /tmp/xarm_vacuum_services.patch
COPY patches/xarm_realmove_joint_states.patch /tmp/xarm_realmove_joint_states.patch
COPY patches/xarm_control_write_watchdog.patch /tmp/xarm_control_write_watchdog.patch
COPY patches/xarm_nonblocking_report_states.patch /tmp/xarm_nonblocking_report_states.patch

# Official UFACTORY ROS 2 driver and collision-aware planning stack. Gazebo is
# excluded from this real-robot image by the manifest-only patch copied above.
RUN mkdir -p /opt/xarm_ws/src && \
    git clone --branch jazzy \
      https://github.com/xArm-Developer/xarm_ros2.git \
      /opt/xarm_ws/src/xarm_ros2 && \
    cd /opt/xarm_ws/src/xarm_ros2 && \
    git checkout "${XARM_ROS2_REF}" && \
    git submodule update --init --recursive && \
    git apply /tmp/xarm_moveit_config_no_gazebo.patch && \
    git apply /tmp/xarm_planner_low_speed.patch && \
    git apply /tmp/xarm_planner_display_path.patch && \
    git apply /tmp/xarm_uf850_joint4_pi_limit.patch && \
    git apply /tmp/xarm_uf850_sensor_stack.patch && \
    git apply /tmp/xarm_vacuum_services.patch && \
    git apply /tmp/xarm_realmove_joint_states.patch && \
    git apply /tmp/xarm_control_write_watchdog.patch && \
    git apply /tmp/xarm_nonblocking_report_states.patch && \
    source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths \
      /opt/xarm_ws/src/xarm_ros2/uf_ros_lib \
      /opt/xarm_ws/src/xarm_ros2/xarm_api \
      /opt/xarm_ws/src/xarm_ros2/xarm_controller \
      /opt/xarm_ws/src/xarm_ros2/xarm_description \
      /opt/xarm_ws/src/xarm_ros2/xarm_moveit_config \
      /opt/xarm_ws/src/xarm_ros2/xarm_msgs \
      /opt/xarm_ws/src/xarm_ros2/xarm_planner \
      /opt/xarm_ws/src/xarm_ros2/xarm_sdk \
      --ignore-src --rosdistro jazzy \
      --skip-keys "joint_state_publisher robot_state_publisher xacro urdf" \
      -y && \
    cd /opt/xarm_ws && \
    colcon build --packages-up-to xarm_planner xarm_moveit_config \
      --packages-skip xarm_gazebo

# 进入容器自动 source ROS 2 和工作区环境
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo 'source /opt/xarm_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /workspace/ws/install/setup.bash ]; then source /workspace/ws/install/setup.bash; fi' >> /root/.bashrc

CMD ["bash"]
