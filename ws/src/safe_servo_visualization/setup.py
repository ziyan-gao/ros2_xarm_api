from glob import glob

from setuptools import find_packages, setup

package_name = 'safe_servo_visualization'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='robot', maintainer_email='robot@example.com',
    description='UF850 visualization helpers', license='Apache-2.0',
    entry_points={'console_scripts': [
        'visualization_node = safe_servo_visualization.visualization_node:main',
        'pallet_localization = safe_servo_visualization.pallet_localization_node:main',
        'item_localization = safe_servo_visualization.item_localization_node:main',
        'waypoint_store = safe_servo_visualization.waypoint_store_node:main',
        'motion_coordinator = safe_servo_visualization.motion_coordinator_node:main',
        'pickup_supervisor = safe_servo_visualization.pickup_supervisor_node:main',
        'pickup_pipeline = safe_servo_visualization.pickup_pipeline_node:main',
        'place_pipeline = safe_servo_visualization.place_pipeline_node:main',
        'planning_scene_obstacles = safe_servo_visualization.planning_scene_obstacles_node:main',
    ]},
)
