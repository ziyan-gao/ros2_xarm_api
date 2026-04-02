FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# 基础工具
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    git \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# 安装基础 Python 包
RUN python3 -m pip install --break-system-packages build pyproject_hooks && \
    python3 -m pip install --break-system-packages --no-deps opencv-python-headless

# 安装 xArm Python SDK（按官方 source code 方式）
RUN git clone https://github.com/xArm-Developer/xArm-Python-SDK.git /opt/xArm-Python-SDK && \
    cd /opt/xArm-Python-SDK && \
    python3 -m pip install --break-system-packages .

# 进入容器自动 source ROS 2 和工作区环境
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo 'if [ -f /workspace/ws/install/setup.bash ]; then source /workspace/ws/install/setup.bash; fi' >> /root/.bashrc

CMD ["bash"]
