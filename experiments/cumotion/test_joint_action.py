import threading
from types import SimpleNamespace
import unittest
import torch
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ros_plan_only import PlanOnlyServer, cumotion_speed_scale


class JointActionTests(unittest.TestCase):
    def run_case(self, success=True, raises=False, wrong_end=False):
        names = [f'joint{i}' for i in range(1, 7)]
        goal = MoveGroup.Goal()
        goal.planning_options.plan_only = True
        req = goal.request
        req.group_name = 'uf850'
        req.max_velocity_scaling_factor = .4
        req.max_acceleration_scaling_factor = .4
        req.start_state.joint_state.name = names
        req.start_state.joint_state.position = [0.]*6
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=.1, tolerance_above=.001,
                            tolerance_below=.001) for n in names])]
        handle = SimpleNamespace(request=goal, aborted=False, succeeded=False)
        handle.abort = lambda: setattr(handle, 'aborted', True)
        handle.succeed = lambda: setattr(handle, 'succeeded', True)
        calls = []
        def plan(start, target, config):
            calls.append(target.position.tolist())
            self.assertFalse(config.parallel_finetune)
            self.assertAlmostEqual(config.time_dilation_factor, .8)
            if raises:
                raise RuntimeError('injected failure')
            return SimpleNamespace(success=torch.tensor([success]), status='TEST',
                optimized_plan=None, optimized_dt=torch.tensor(.02), total_time=.01)
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = names
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[.2 if wrong_end else .1]*6)]
        logger = SimpleNamespace(info=lambda _: None, error=lambda _: None)
        server = SimpleNamespace(lock=threading.Lock(), planner_busy=False,
            parallel_finetune=False, speed_multiplier=2.,
            update_world_objects=lambda _: True,
            tensor_args=SimpleNamespace(to_device=lambda v: torch.tensor(v)),
            motion_gen=SimpleNamespace(get_active_js=lambda x: x,
                reset=lambda **kw: None, plan_single_js=plan),
            get_logger=lambda: logger,
            get_parameter=lambda key: SimpleNamespace(value=2 if key == 'max_attempts' else .2),
            get_joint_trajectory=lambda *args: trajectory)
        result = PlanOnlyServer.execute_callback(server, handle)
        self.assertFalse(server.planner_busy)
        self.assertEqual(len(calls), 1)
        return result, handle

    def test_speed_multiplier_caps_at_model_limits(self):
        self.assertAlmostEqual(cumotion_speed_scale(.4, 2.), .8)
        self.assertEqual(cumotion_speed_scale(.66, 2.), 1.)
        self.assertEqual(cumotion_speed_scale(.96, 2.), 1.)
        for bad in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                cumotion_speed_scale(.4, bad)

    def test_joint_goal_no_pose_ik(self):
        result, handle = self.run_case()
        self.assertEqual(result.error_code.val, 1)
        self.assertTrue(handle.succeeded)
        self.assertFalse(handle.aborted)
        endpoint = result.planned_trajectory.joint_trajectory.points[-1]
        self.assertEqual(list(endpoint.velocities), [0.0] * 6)
        self.assertEqual(list(endpoint.accelerations), [0.0] * 6)

    def test_failure_aborts(self):
        result, handle = self.run_case(success=False)
        self.assertNotEqual(result.error_code.val, 1)
        self.assertTrue(handle.aborted)
        self.assertFalse(handle.succeeded)

    def test_exception_releases_busy(self):
        result, handle = self.run_case(raises=True)
        self.assertTrue(handle.aborted)
        self.assertNotEqual(result.error_code.val, 1)

    def test_wrong_joint_endpoint_rejected(self):
        result, handle = self.run_case(wrong_end=True)
        self.assertTrue(handle.aborted)
        self.assertFalse(result.planned_trajectory.joint_trajectory.points)


if __name__ == '__main__':
    unittest.main()
