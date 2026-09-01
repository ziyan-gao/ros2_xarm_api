import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
import yaml


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    servo_config_path = os.path.join(
        get_package_share_directory('safe_servo_package'),
        'config', 'uf850_servo.yaml')
    with open(servo_config_path, encoding='utf-8') as config_file:
        servo_config = yaml.safe_load(config_file)

    moveit_config = MoveItConfigsBuilder(
        context=context,
        dof='6',
        robot_type='uf850',
        hw_ns=LaunchConfiguration('hw_ns', default='ufactory'),
        add_vacuum_gripper='true',
    ).to_moveit_configs()

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            {'moveit_servo': servo_config},
            {'update_period': 0.02},
            {'planning_group_name': 'uf850'},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    bridge_node = Node(
        package='safe_servo_package',
        executable='moveit_servo_bridge',
        name='safe_servo',
        output='screen',
        parameters=[{
            'dry_run': ParameterValue(
                LaunchConfiguration('dry_run'), value_type=bool),
            'max_linear_speed': ParameterValue(
                LaunchConfiguration('max_linear_speed'), value_type=float),
            'kp_z': ParameterValue(
                LaunchConfiguration('kp_z'), value_type=float),
        }],
    )
    return [servo_node, bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='Simulate guarded Servo commands without robot motion'),
        DeclareLaunchArgument(
            'max_linear_speed', default_value='0.03',
            description='Maximum guarded vertical speed in m/s'),
        DeclareLaunchArgument(
            'kp_z', default_value='3.0',
            description='Proportional gain for guarded vertical motion'),
        OpaqueFunction(function=launch_setup),
    ])
