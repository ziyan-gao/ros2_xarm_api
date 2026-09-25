#!/usr/bin/env bash
# Host launcher: separate X11 RViz, isolated network, no robot device mounts.
set -euo pipefail
experiment_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$experiment_dir/../.." && pwd)
if [ -z "${DISPLAY:-}" ]; then
    echo 'DISPLAY is unset. Run this from a terminal in the local graphical session.' >&2
    exit 1
fi
auth_file=${XAUTHORITY:-${HOME}/.Xauthority}
if [ ! -r "$auth_file" ]; then
    echo 'No readable Xauthority file. Set XAUTHORITY to your desktop session file.' >&2
    exit 1
fi
x11_socket_dir=/tmp/.X11-unix
case "$(docker info --format '{{.DockerRootDir}}')" in
    /var/snap/docker/*) x11_socket_dir=/var/lib/snapd/hostfs/tmp/.X11-unix ;;
esac
exec docker run --rm --init --network none --runtime nvidia --gpus all \
    --hostname "$(hostname)" \
    -e DISPLAY -e XAUTHORITY=/tmp/cumotion.xauth \
    -e QT_X11_NO_MITSHM=1 \
    -e CUMOTION_TEST_HEADLESS="${CUMOTION_TEST_HEADLESS:-false}" \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
    --mount "type=bind,src=$x11_socket_dir,dst=/tmp/.X11-unix,readonly" \
    --mount "type=bind,src=$auth_file,dst=/tmp/cumotion.xauth,readonly" \
    -v "$experiment_dir:/experiment:ro" \
    -v "$repo_dir/config/taught_waypoints.yaml:/waypoints.yaml:ro" \
    uf850-cumotion-test:4.0 -lc 'bash /experiment/start_panel_demo.sh'
