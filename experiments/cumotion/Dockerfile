# Isolated dependency probe. Never used by the production compose stack.
FROM ros2_xarm_api-ros2_cv:latest
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg ca-certificates && \
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | \
    gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg
# Same CUDA Python dependency mechanism as Isaac ROS CLI release-4.0.
RUN python3 -m pip install --break-system-packages --no-cache-dir cuda-python==13.0.1
COPY cuda-python-placeholder /tmp/cuda-python-placeholder
RUN dpkg-deb --root-owner-group --build /tmp/cuda-python-placeholder /tmp/cuda-python-placeholder.deb && \
    dpkg -i /tmp/cuda-python-placeholder.deb
RUN echo 'deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.0 noble main' > /etc/apt/sources.list.d/nvidia-isaac-ros.list && \
    apt-get update && apt-get install -y --no-install-recommends ros-jazzy-isaac-ros-cumotion ros-jazzy-isaac-ros-cumotion-moveit
RUN python3 -m pip install --break-system-packages --no-cache-dir torch==2.9.0+cu130 --index-url https://download.pytorch.org/whl/cu130 && \
    python3 -m pip install --break-system-packages --no-cache-dir warp-lang==1.9.0 trimesh yourdfpy
# No production entrypoint, robot address, host network, or execution node.
RUN python3 -m pip install --break-system-packages --no-cache-dir python-fcl==0.7.0.8
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-isaac-manipulator-ros-python-utils
RUN git init /opt/cumotion_plugin_src && \
    git -C /opt/cumotion_plugin_src fetch --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git 1acd5d89d0b32b8972e614ff149996803a2d3f45 && \
    git -C /opt/cumotion_plugin_src checkout --detach FETCH_HEAD && \
    source /opt/ros/jazzy/setup.bash && \
    cd /opt/cumotion_plugin_src && colcon build --packages-select isaac_ros_cumotion_moveit \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DCMAKE_DISABLE_FIND_PACKAGE_isaac_ros_common=TRUE \
      --install-base /opt/cumotion_plugin_overlay
ENV ROS_DOMAIN_ID=87
COPY smoke.py /opt/cumotion-experiment/smoke.py
ENTRYPOINT ["/bin/bash"]
CMD ["-lc", "source /opt/ros/jazzy/setup.bash && python3 /opt/cumotion-experiment/smoke.py"]
