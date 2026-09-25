import unittest
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import AttachedCollisionObject, LinkPadding, AllowedCollisionEntry, CollisionObject
from geometry_msgs.msg import Pose
from panel_bridge import normalize


class PanelBridgeTests(unittest.TestCase):
    def test_object_pose_is_composed(self):
        goal = MoveGroup.Goal()
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.pose.orientation.w = 1.
        obj.pose.position.x = .3
        p = Pose()
        p.orientation.w = 1.
        p.position.x = .2
        obj.primitive_poses = [p]
        goal.planning_options.planning_scene_diff.world.collision_objects = [obj]
        result = normalize(goal, set()).planning_options.planning_scene_diff.world.collision_objects[0]
        self.assertEqual(result.header.frame_id, 'link_base')
        self.assertAlmostEqual(result.primitive_poses[0].position.x, .5)
        self.assertEqual(result.pose.position.x, 0.)

    def test_does_not_mutate_original(self):
        original = MoveGroup.Goal()
        self.assertTrue(normalize(original, set()).planning_options.plan_only)
        self.assertFalse(original.planning_options.plan_only)

    def test_rejects_attachment(self):
        goal = MoveGroup.Goal()
        goal.planning_options.planning_scene_diff.robot_state.attached_collision_objects = [AttachedCollisionObject()]
        with self.assertRaisesRegex(ValueError, 'attachments'):
            normalize(goal, set())

    def test_rejects_padding(self):
        goal = MoveGroup.Goal()
        goal.planning_options.planning_scene_diff.link_padding = [LinkPadding(link_name='link1', padding=.1)]
        with self.assertRaisesRegex(ValueError, 'padding'):
            normalize(goal, set())

    def test_rejects_custom_acm(self):
        goal = MoveGroup.Goal()
        acm = goal.planning_options.planning_scene_diff.allowed_collision_matrix
        acm.entry_names = ['link1', 'link2']
        acm.entry_values = [AllowedCollisionEntry(enabled=[True, True]),
                            AllowedCollisionEntry(enabled=[True, True])]
        with self.assertRaisesRegex(ValueError, 'exemption'):
            normalize(goal, set())
        self.assertTrue(normalize(goal, {frozenset(('link1', 'link2'))}).planning_options.plan_only)


if __name__ == '__main__':
    unittest.main()
