"""Replay a captured scene offline. No execution clients or controller requests.
Run in a --network none GPU container with an exported CUMOTION_TEST_MODEL.
"""
import argparse
import copy
import json
import time
from pathlib import Path

import rclpy
import torch
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningScene, Constraints, JointConstraint
from rosidl_runtime_py.set_message import set_message_fields
from curobo.wrap.reacher.motion_gen import MotionGen
from live_bridge import LiveServer
from live_snapshot_smoke import Handle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--speed-scale', type=float, default=.8)
    args, ros_args = parser.parse_known_args()
    if not 0 < args.speed_scale <= 1:
        parser.error('--speed-scale must be in (0, 1]')
    latest = {}
    plan = MotionGen.plan_single_js
    def measured(self, *a, **kw):
        result = plan(self, *a, **kw)
        torch.cuda.synchronize()
        opt = self.finetune_js_trajopt_solver.solver.newton_optimizer
        latest.clear()
        latest.update({key: float(getattr(result, key)) for key in
                       ('finetune_time', 'trajopt_time', 'total_time')})
        latest['optimizer'] = {k: getattr(opt, k, None) for k in ('outer_iters', 'inner_iters', 'n_iters')}
        latest['success'] = bool(result.success.item())
        latest['status'] = str(result.status)
        return result
    MotionGen.plan_single_js = measured
    # Warm each measured joint-space case below; avoid allocating unused pose
    # CUDA graphs alongside the live planner on the shared GPU.
    MotionGen.warmup = lambda self, *a, **kw: None
    rclpy.init(args=ros_args)
    node = LiveServer()
    rows = []
    try:
        snapshot = json.loads(Path(args.scene).read_text())
        for obj in snapshot['scene']['world']['collision_objects'] + [a['object'] for a in snapshot['scene']['robot_state']['attached_collision_objects']]:
            if isinstance(obj['operation'], str):
                obj['operation'] = obj['operation'].encode('latin1')
        scene = PlanningScene()
        set_message_fields(scene, snapshot['scene'])
        observation = [.03502044, -.13850302, -1.22992897, .00022201, -1.10141492, .03496121]
        pallet = [-1.17431797, -.62052577, -2.02929079, .00000352, -1.40876502, -1.14244754]
        prepick = [.05814797, -.48335290, -1.19919291, .00001418, -.71583994, 1.48971485]
        cases = [('return', pallet, observation), ('transfer', observation, pallet), ('prepick_reference', observation, prepick)]
        for label, start, end in cases:
            for repeat in range(args.repeats+1):
                goal = MoveGroup.Goal()
                goal.planning_options.plan_only = True
                goal.planning_options.planning_scene_diff = copy.deepcopy(scene)
                state = goal.planning_options.planning_scene_diff.robot_state
                state.joint_state.name = [f'joint{i}' for i in range(1,7)]
                state.joint_state.position = start
                state.joint_state.velocity = [0.]*6
                state.joint_state.effort = []
                goal.request.start_state = copy.deepcopy(state)
                goal.request.group_name = 'uf850'
                goal.request.max_velocity_scaling_factor = args.speed_scale
                goal.request.max_acceleration_scaling_factor = args.speed_scale
                goal.request.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
                    joint_name=n, position=q, tolerance_above=1e-5, tolerance_below=1e-5, weight=1.)
                    for n,q in zip(state.joint_state.name,end)])]
                handle = Handle(goal)
                latest.clear()
                before=time.monotonic()
                result = node.execute_callback(handle)
                row = dict(case=label, repeat=repeat, warmup=repeat==0,
                           wall_s=time.monotonic()-before, error_code=result.error_code.val, **latest)
                points = result.planned_trajectory.joint_trajectory.points
                if points:
                    row['duration_s'] = points[-1].time_from_start.sec+points[-1].time_from_start.nanosec*1e-9
                    row['goal_error_rad'] = max(abs(x-y) for x,y in zip(points[-1].positions,end))
                    row['peak_velocity'] = max((abs(v) for p in points for v in p.velocities), default=0.)
                    row['peak_acceleration'] = max((abs(v) for p in points for v in p.accelerations), default=0.)
                rows.append(row)
                print('BENCH '+json.dumps(row), flush=True)
        Path(args.output).write_text(json.dumps(rows, indent=2)+'\n')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
