"""Isolated MoveIt/RViz planning demo: no control manager or robot driver."""
import os
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model = Path(os.environ['CUMOTION_TEST_MODEL'])
    share = Path(get_package_share_directory('xarm_moveit_config'))
    urdf = ET.parse(model/'uf850.urdf').getroot()
    for mesh in urdf.findall('.//mesh'):
        filename = mesh.get('filename')
        if filename.startswith('/'):
            mesh.set('filename', Path(filename).as_uri())
    description = {'robot_description': ET.tostring(urdf, encoding='unicode'),
                   'robot_description_semantic': (model/'uf850.srdf').read_text(),
                   'robot_description_kinematics': yaml.safe_load(
                       (share/'config/uf850/kinematics.yaml').read_text()),
                   'robot_description_planning': yaml.safe_load(
                       (share/'config/uf850/joint_limits.yaml').read_text())}
    pipeline = {
        'planning_pipelines': ['isaac_ros_cumotion'],
        'default_planning_pipeline': 'isaac_ros_cumotion',
        'isaac_ros_cumotion': {
            'planning_plugins': ['isaac_ros_cumotion_moveit/CumotionPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision'],
            'response_adapters': [
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath']}}
    cspace = yaml.safe_load((model/'uf850.yml').read_text())['robot_cfg']['kinematics']['cspace']
    rviz_config = yaml.safe_load((share/'rviz/moveit.rviz').read_text())
    def set_group(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'Planning Group':
                    value[key] = 'uf850'
                else:
                    set_group(child)
        elif isinstance(value, list):
            for child in value:
                set_group(child)
    set_group(rviz_config)
    rviz_file = model/'panel.rviz'
    rviz_file.write_text(yaml.safe_dump(rviz_config))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description['robot_description']}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'robot_description': description['robot_description'],
                          'zeros': dict(zip(cspace['joint_names'], cspace['retract_config'])),
                          'rate': 20}]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[description, pipeline, {
                 'allow_trajectory_execution': False,
                 'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction',
                 'publish_robot_description': True,
                 'publish_robot_description_semantic': True,
                 'publish_planning_scene': True,
                 'publish_geometry_updates': True,
                 'publish_state_updates': True,
                 'publish_transforms_updates': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', str(rviz_file)],
             parameters=[description, pipeline], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
