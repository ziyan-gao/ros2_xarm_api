#!/usr/bin/env bash

set -euo pipefail

ROS_DISTRO="jazzy"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
ROS_ENV_SNIPPET="${HOME}/.ros2_container_topic_env.sh"

if [[ "${EUID}" -eq 0 ]]; then
  echo "Please run this script as a normal user, not root."
  echo "It will use sudo when package installation is needed."
  exit 1
fi

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot detect host OS: /etc/os-release is missing."
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release

if [[ "${ID:-}" != "ubuntu" ]]; then
  echo "This script currently supports Ubuntu hosts only."
  exit 1
fi

if [[ "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "ROS 2 Jazzy targets Ubuntu 24.04 (noble)."
  echo "Detected: ${PRETTY_NAME:-unknown}"
  exit 1
fi

sudo apt-get update
sudo apt-get install -y locales curl gnupg2 ca-certificates lsb-release software-properties-common

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi

ROS_APT_SOURCE="/etc/apt/sources.list.d/ros2.list"
ROS_REPO_LINE="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main"

if [[ ! -f "${ROS_APT_SOURCE}" ]] || ! grep -Fq "${ROS_REPO_LINE}" "${ROS_APT_SOURCE}"; then
  echo "${ROS_REPO_LINE}" | sudo tee "${ROS_APT_SOURCE}" >/dev/null
fi

sudo apt-get update
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-ros-base" \
  "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"

cat > "${ROS_ENV_SNIPPET}" <<'EOF'
#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
EOF

chmod +x "${ROS_ENV_SNIPPET}"

if ! grep -Fq "${ROS_ENV_SNIPPET}" "${HOME}/.bashrc"; then
  {
    echo ""
    echo "# ROS 2 host settings for talking to the ros2_cv container"
    echo "source \"${ROS_ENV_SNIPPET}\""
  } >> "${HOME}/.bashrc"
fi

echo ""
echo "Install complete."
echo "Open a new shell or run:"
echo "  source \"${ROS_ENV_SNIPPET}\""
echo ""
echo "Then verify with:"
echo "  ros2 topic list"
echo "  ros2 topic echo /topic"
