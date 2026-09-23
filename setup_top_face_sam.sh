#!/usr/bin/env bash
# Run inside the ROS image/container. Only writes this dedicated environment
# and ignored results directory; never upgrades the robot/policy interpreter.
set -euo pipefail
cd /workspace
case "${1:-cpu}" in
  cpu) debug_env=/workspace/.venv-top-face; wheel_index=https://download.pytorch.org/whl/cpu ;;
  cuda) debug_env=/workspace/.venv-top-face-cuda; wheel_index=https://download.pytorch.org/whl/cu124 ;;
  *) echo 'Usage: bash setup_top_face_sam.sh [cpu|cuda]' >&2; exit 2 ;;
esac
if [[ "${1:-cpu}" == cuda ]]; then
  python3 -c 'import os; s=os.statvfs("/workspace"); free=s.f_bavail*s.f_frsize; assert free >= 12*1024**3, f"CUDA installation requires 12 GiB free (including download/extraction headroom); available: {free/1024**3:.1f} GiB"'
fi
python3 -m venv --system-site-packages "$debug_env"
debug_python="$debug_env/bin/python"
"$debug_python" -m pip install --no-cache-dir 'torch==2.5.1' 'torchvision==0.20.1' \
  --index-url "$wheel_index"
SAM2_BUILD_CUDA=0 "$debug_python" -m pip install --no-cache-dir --no-build-isolation \
  'git+https://github.com/facebookresearch/sam2.git@2b90b9f5ceec907a1c18123530e92e794ad901a4'
mkdir -p /workspace/results/models
checkpoint=/workspace/results/models/sam2.1_hiera_tiny.pt
if [[ ! -s "$checkpoint" ]]; then
  curl --fail --location --retry 3 \
    https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt \
    --output "${checkpoint}.download"
  mv "${checkpoint}.download" "$checkpoint"
fi
"$debug_python" -c 'import hashlib; from pathlib import Path; p=Path("/workspace/results/models/sam2.1_hiera_tiny.pt"); assert hashlib.sha256(p.read_bytes()).hexdigest() == "7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69", "SAM2 checkpoint checksum mismatch"'
"$debug_python" -c 'import torch; from sam2.build_sam import build_sam2; print("SAM2 ready, torch:", torch.__version__)'
