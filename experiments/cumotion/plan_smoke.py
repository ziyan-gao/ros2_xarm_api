"""Plan for the bundled Franka model only, without ROS or trajectory execution.

This validates the GPU planner installation, not UF850 geometry or constraints.
"""
import json
import time
import torch
from curobo.types.base import TensorDeviceType
from curobo.types.state import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

torch.manual_seed(0)
args = TensorDeviceType()
t0 = time.monotonic()
config = MotionGenConfig.load_from_robot_config(
    'franka.yml',
    {'cuboid': {'floor': {'dims': [2.0, 2.0, 0.1],
                           'pose': [0, 0, -0.1, 1, 0, 0, 0]}}},
    args, interpolation_dt=0.02,
)
planner = MotionGen(config)
planner.warmup(enable_graph=True)
torch.cuda.synchronize()
warmup_s = time.monotonic() - t0
q = planner.get_retract_config().view(1, -1)
start = JointState.from_position(q, joint_names=planner.kinematics.joint_names)
goal_q = q.clone()
goal_q[0, 0] += 0.15
goal = planner.compute_kinematics(
    JointState.from_position(goal_q, joint_names=planner.kinematics.joint_names)
).ee_pose.clone()
t0 = time.monotonic()
result = planner.plan_single(start, goal, MotionGenPlanConfig(max_attempts=2))
torch.cuda.synchronize()
success = bool(result.success.item())
print(json.dumps({'model': 'bundled_franka_NOT_uf850', 'success': success,
                  'status': str(result.status), 'warmup_s': warmup_s,
                  'planning_wall_s': time.monotonic() - t0,
                  'robot_execution': False}), flush=True)
if not success:
    raise SystemExit(1)
