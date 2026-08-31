from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution([
        FindPackageShare('safe_servo_visualization'), 'urdf',
        'uf850_sensor_stack.urdf.xacro'
    ])
    description = Command([FindExecutable(name='xacro'), ' ', model])
    return LaunchDescription([
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='uf850_state_publisher',
            parameters=[{'robot_description': description}],
            remappings=[('/joint_states', '/ufactory/joint_states')],
            output='screen',
        ),
        Node(
            package='safe_servo_visualization', executable='visualization_node',
            output='screen',
        ),
    ])
