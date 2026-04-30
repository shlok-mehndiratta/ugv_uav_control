"""
Launch file for Standard MPC controller.

Usage:
    ros2 launch mpc_controllers standard_mpc.launch.py
    
Optional arguments:
    control_rate:=20.0          # Control loop rate (Hz)
    waypoint_threshold:=0.1     # Waypoint reached threshold (m)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='20.0',
        description='Control loop rate in Hz'
    )
    
    waypoint_threshold_arg = DeclareLaunchArgument(
        'waypoint_threshold',
        default_value='0.1',
        description='Distance threshold to consider waypoint reached (m)'
    )
    
    # Standard MPC Node
    standard_mpc_node = Node(
        package='lcs',
        executable='standard_mpc_node',
        name='standard_mpc_node',
        output='screen',
        parameters=[{
            'horizon': 20,
            'dt': 0.05,
            'control_rate': LaunchConfiguration('control_rate'),
            'tube_data_path': '/home/ptc/IROS_paper/tube_data.npz',
            'waypoint_threshold': LaunchConfiguration('waypoint_threshold'),
        }],
        remappings=[
            ('/drone/odom', '/drone/odom'),
            ('/waypoint', '/waypoint'),
            ('/drone/cmd_accel', '/drone/cmd_accel'),
            ('/drone/mpc_status', '/drone/mpc_status'),
        ]
    )
    
    return LaunchDescription([
        control_rate_arg,
        waypoint_threshold_arg,
        standard_mpc_node,
    ])
