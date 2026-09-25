"""Offline reproduction of the logged failed joint target. Never executes."""
import argparse
import json
from pathlib import Path
import time
import yaml
import torch
from curobo.types.base import TensorDeviceType
from curobo.types.state import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig


def summarize(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().tolist()
    return str(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--dt', type=float)
    args = parser.parse_args()
    folder = Path(args.model)
    cfg = yaml.safe_load((folder/'uf850.yml').read_text())['robot_cfg']
    cfg['kinematics']['urdf_path'] = str(folder/'uf850.urdf')
    cspace = cfg['kinematics']['cspace']
    target = [2.0394086600248884, .604404010119571, -.9274115747808079,
              -3.4630660756369354, 1.5389186620064559, 4.605106583444039]
    device = TensorDeviceType()
    options = {} if args.dt is None else {'js_trajopt_dt': args.dt,
                                          'maximum_trajectory_dt': max(.15, args.dt)}
    planner = MotionGen(MotionGenConfig.load_from_robot_config(cfg,
        {'cuboid': {'floor': {'dims': [2., 2., .1], 'pose': [0, 0, -.15, 1, 0, 0, 0]}}},
        device, trajopt_tsteps=32, num_graph_seeds=6, num_trajopt_seeds=6,
        store_debug_in_result=True, **options))
    planner.warmup(enable_graph=True)
    start = JointState.from_position(device.to_device([cspace['retract_config']]),
                                    joint_names=cspace['joint_names'])
    goal = JointState.from_position(device.to_device([target]), joint_names=cspace['joint_names'])
    for label, config in [
            ('normal', MotionGenPlanConfig(max_attempts=2, enable_graph_attempt=1)),
            ('graph_only_diagnostic', MotionGenPlanConfig(max_attempts=2, enable_graph=True, enable_opt=False))]:
        planner.reset(reset_seed=True)
        t0 = time.monotonic()
        result = planner.plan_single_js(start, goal, config)
        torch.cuda.synchronize()
        report = {'case': label, 'dt_override': args.dt, 'success': summarize(result.success),
                  'status': str(result.status), 'valid_query': result.valid_query,
                  'wall_s': time.monotonic()-t0, 'graph_time': result.graph_time,
                  'attempts': result.attempts, 'debug': {}}
        for key, val in (result.debug_info or {}).items():
            report['debug'][key] = {k: summarize(getattr(val, k)) for k in
                ('success', 'feasible', 'cspace_error', 'position_error', 'optimized_dt',
                 'smooth_error', 'smooth_label') if hasattr(val, k)}
        print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
