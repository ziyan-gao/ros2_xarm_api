import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup

rclpy.init()
node = rclpy.create_node('wait_cumotion_panel_planner')
try:
    client = ActionClient(node, MoveGroup, '/cumotion/move_group')
    if not client.wait_for_server(timeout_sec=60.):
        raise RuntimeError('cuMotion did not become ready within 60 seconds')
finally:
    node.destroy_node()
    rclpy.try_shutdown()
