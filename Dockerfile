FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# 基础工具
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-rosdep \
    git \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-realsense2-camera \
    ros-jazzy-moveit \
    ros-jazzy-pilz-industrial-motion-planner \
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

# Official UFACTORY ROS 2 driver and collision-aware planning stack. Gazebo and
# MoveIt Servo are intentionally not built for the real-robot Phase 1 setup.
RUN mkdir -p /opt/xarm_ws/src && \
    git clone --recursive --branch jazzy \
      https://github.com/xArm-Developer/xarm_ros2.git \
      /opt/xarm_ws/src/xarm_ros2 && \
    git clone --depth 1 --branch jazzy \
      https://github.com/ros-controls/control_msgs.git \
      /opt/xarm_ws/src/control_msgs && \
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
      /opt/xarm_ws/src/control_msgs \
      --ignore-src --rosdistro jazzy \
      --skip-keys "joint_state_publisher robot_state_publisher xacro urdf" \
      -y && \
    cd /opt/xarm_ws && \
    colcon build --packages-up-to xarm_planner xarm_moveit_config

# 进入容器自动 source ROS 2 和工作区环境
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo 'source /opt/xarm_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /workspace/ws/install/setup.bash ]; then source /workspace/ws/install/setup.bash; fi' >> /root/.bashrc

CMD ["bash"]
