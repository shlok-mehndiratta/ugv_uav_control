import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ugv_uav_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models/turtlebot3_burger_aruco'), glob('models/turtlebot3_burger_aruco/model.*')),
        (os.path.join('share', package_name, 'models/turtlebot3_burger_aruco/materials/scripts'), glob('models/turtlebot3_burger_aruco/materials/scripts/*')),
        (os.path.join('share', package_name, 'models/turtlebot3_burger_aruco/materials/textures'), glob('models/turtlebot3_burger_aruco/materials/textures/*')),
        (os.path.join('share', package_name, 'models/path_ground_plane'), glob('models/path_ground_plane/model.*')),
        (os.path.join('share', package_name, 'models/path_ground_plane/materials/scripts'), glob('models/path_ground_plane/materials/scripts/*')),
        (os.path.join('share', package_name, 'models/path_ground_plane/materials/textures'), glob('models/path_ground_plane/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shlok-mehndiratta',
    maintainer_email='shlokm23@iiserb.ac.in',
    description='UGV UAV Control Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'click_to_drive = ugv_uav_control.click_to_drive:main',
            'line_follower = ugv_uav_control.line_follower:main',
            'middle_path_follower = ugv_uav_control.middle_path_follower:main',
            'waypoint_follower = ugv_uav_control.waypoint_follower:main',
            'pure_pursuit_node = ugv_uav_control.pure_pursuit_node:main',
            'stanley_node = ugv_uav_control.stanley_node:main',
        ],
    },
)
