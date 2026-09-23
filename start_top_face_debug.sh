#!/usr/bin/env bash
# Passive process only: never launch/restart the robot stack here.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /workspace/ws/install/setup.bash
set -u
debug_python=/usr/bin/python3
debug_device="${TOP_FACE_SAM_DEVICE:-cpu}"
if [[ "$debug_device" == cuda* ]]; then
  debug_python=/workspace/.venv-top-face-cuda/bin/python
  if [[ ! -x "$debug_python" ]]; then
    echo 'SAM CUDA environment missing: run bash /workspace/setup_top_face_sam.sh cuda' >&2
    exit 2
  fi
  "$debug_python" -c 'import torch; assert torch.cuda.is_available(), "CUDA unavailable: check NVIDIA runtime/GPU device access"; print("SAM GPU:", torch.cuda.get_device_name(0))'
elif [[ "$debug_device" != cpu ]]; then
  echo "Unsupported SAM device: $debug_device" >&2
  exit 2
elif [[ -x /workspace/.venv-top-face/bin/python ]]; then
  debug_python=/workspace/.venv-top-face/bin/python
fi
# Keep background model inference from monopolizing the robot host CPU.
export OMP_NUM_THREADS="${TOP_FACE_SAM_THREADS:-2}"
export MKL_NUM_THREADS="${TOP_FACE_SAM_THREADS:-2}"
exec "$debug_python" -c 'from safe_servo_visualization.top_face_debug_node import main; main()' \
  --ros-args \
  -p sam_checkpoint:="${TOP_FACE_SAM_CHECKPOINT:-/workspace/results/models/sam2.1_hiera_tiny.pt}" \
  -p sam_model_config:="${TOP_FACE_SAM_MODEL_CONFIG:-configs/sam2.1/sam2.1_hiera_t.yaml}" \
  -p sam_device:="$debug_device" "$@"
