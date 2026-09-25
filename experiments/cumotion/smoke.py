"""Offline GPU and cuRobo kernel probe; never creates a ROS node or executes motion."""
import json
import torch

assert torch.cuda.is_available(), 'CUDA unavailable'
x = torch.eye(8, device='cuda')
assert torch.equal(x @ x, x)
torch.cuda.synchronize()

from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig  # noqa: E402
from curobo.types.base import TensorDeviceType  # noqa: E402

print(json.dumps({
    'gpu': torch.cuda.get_device_name(0),
    'torch': torch.__version__,
    'cuda': torch.version.cuda,
    'tensor_test': 'passed',
    'motion_gen_import': 'passed',
    'robot_execution': False,
}), flush=True)
