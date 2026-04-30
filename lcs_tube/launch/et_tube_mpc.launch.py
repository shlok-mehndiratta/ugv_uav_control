"""
Launch file for Event-Triggered Tube MPC controller.

Usage:
    ros2 launch mpc_controllers et_tube_mpc.launch.py
    
Optional arguments:
    control_rate:=20.0              # Control loop rate (Hz)
    waypoint_threshold:=0.1         # Waypoint reached threshold (m)
    trigger_threshold:=0.15           # Position error trigger threshold (m)
    consecutive_threshold:=3          # Consecutive steps to trigger
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
    
    trigger_threshold_arg = DeclareLaunchArgument(
        'trigger_threshold',
        default_value='0.15',
        description='Position error threshold to trigger re-optimization (m)'
    )
    
    consecutive_threshold_arg = DeclareLaunchArgument(
        'consecutive_threshold',
        default_value='3',
        description='Consecutive steps above threshold to trigger'
    )
    
    # ET-Tube MPC Node
    et_tube_mpc_node = Node(
        package='lcs',
        executable='et_tube_mpc_node',
        name='et_tube_mpc_node',
        output='screen',
        parameters=[{
            'horizon': 20,
            'dt': 0.05,
            'control_rate': LaunchConfiguration('control_rate'),
            'tube_data_path': '/home/ptc/IROS_paper/tube_data.npz',
            'waypoint_threshold': LaunchConfiguration('waypoint_threshold'),
            'trigger_threshold': LaunchConfiguration('trigger_threshold'),
            'consecutive_threshold': LaunchConfiguration('consecutive_threshold'),
        }],
        remappings=[
            ('/drone/odom', '/drone/odom'),
            ('/waypoint', '/waypoint'),
            ('/drone/cmd_accel', '/drone/cmd_accel'),
            ('/drone/mpc_status', '/drone/mpc_status'),
            ('/drone/et_trigger', '/drone/et_trigger'),
        ]
    )
    
    return LaunchDescription([
        control_rate_arg,
        waypoint_threshold_arg,
        trigger_threshold_arg,
        consecutive_threshold_arg,
        et_tube_mpc_node,
    ])
