from setuptools import find_packages, setup

package_name = 'box_marker_detection'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='robot', maintainer_email='robot@example.com',
    description='ArUco box detection and visualization', license='Apache-2.0',
    entry_points={'console_scripts': [
        'box_marker_detector = box_marker_detection.detector_node:main',
        'depth_box_refinement = box_marker_detection.depth_refinement_node:main',
    ]},
)
