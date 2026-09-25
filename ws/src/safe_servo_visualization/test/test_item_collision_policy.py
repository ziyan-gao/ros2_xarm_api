from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
from safe_servo_visualization.planning_scene_obstacles_node import PlanningSceneObstacles


def test_exact_pairs_preserve_robot_camera_and_environment_checks():
    names = ['link1', 'link2', 'camera', 'pallet_surface']
    matrix = AllowedCollisionMatrix(entry_names=names)
    matrix.entry_values = [AllowedCollisionEntry(enabled=[False]*4) for _ in names]
    matrix.entry_values[0].enabled[1] = matrix.entry_values[1].enabled[0] = True
    result = PlanningSceneObstacles._allow_item_contacts(matrix, ['placed_item_1', 'carried_item_0'])
    def allowed(a, b):
        return result.entry_values[result.entry_names.index(a)].enabled[result.entry_names.index(b)]
    assert allowed('link1', 'link2')
    assert allowed('placed_item_1', 'carried_item_0')
    assert allowed('carried_item_0', 'placed_item_1')
    assert allowed('xarm_vacuum_gripper_link', 'placed_item_1')
    for other in ('camera', 'link1', 'pallet_surface'):
        assert not allowed(other, 'carried_item_0')
        assert not allowed(other, 'placed_item_1')
    PlanningSceneObstacles._allow_item_contacts(result, ['placed_item_2'])
    assert allowed('placed_item_2', 'carried_item_0')
    assert not allowed('camera', 'placed_item_2')
