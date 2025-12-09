import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def set_track_texture(context, *args, **kwargs):
    pkg_ugv_uav = get_package_share_directory('ugv_uav_control')
    track_name = LaunchConfiguration('track_name').perform(context)
    
    texture_dir = os.path.join(pkg_ugv_uav, 'models', 'path_ground_plane', 'materials', 'textures')
    source_file = os.path.join(texture_dir, f'{track_name}.png')
    target_file = os.path.join(texture_dir, 'Track.png')
    
    if os.path.exists(source_file):
        print(f"Setting track texture to {track_name}")
        shutil.copyfile(source_file, target_file)
    else:
        print(f"Warning: Track texture {track_name} not found at {source_file}. Using default.")

def generate_launch_description():
    pkg_ugv_uav = get_package_share_directory('ugv_uav_control')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch Arguments
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pure_pursuit',
        description='Controller to use: pure_pursuit, waypoint_follower, or stanley'
    )
    
    track_name_arg = DeclareLaunchArgument(
        'track_name',
        default_value='track1',
        description='Name of the track texture to use (e.g., track1, track2)'
    )

    # Set Track Texture Function
    set_track_texture_action = OpaqueFunction(function=set_track_texture)

    # Set GZ_SIM_RESOURCE_PATH
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_ugv_uav, 'models'),
            ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    world_file = os.path.join(pkg_ugv_uav, 'worlds', 'path_following.world')

    # Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # Spawn Robot (at known good position)
    custom_model_path = os.path.join(pkg_ugv_uav, 'models', 'turtlebot3_burger_aruco', 'model.sdf')
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger',
            '-file', custom_model_path,
            '-x', '0.0',
            '-y', '2.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    # Bridge
    bridge_params = os.path.join(
        pkg_turtlebot3_gazebo,
        'params',
        'turtlebot3_burger_bridge.yaml'
    )

    robot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Camera Bridge
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/overhead_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # Perception Node (Middle Path Follower)
    perception_node = Node(
        package='ugv_uav_control',
        executable='middle_path_follower',
        output='screen'
    )
    
    # Control Node (Waypoint Follower)
    waypoint_follower_node = Node(
        package='ugv_uav_control',
        executable='waypoint_follower',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('controller'), "' == 'waypoint_follower'"])
        )
    )

    # Control Node (Pure Pursuit)
    pure_pursuit_node = Node(
        package='ugv_uav_control',
        executable='pure_pursuit_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('controller'), "' == 'pure_pursuit'"])
        )
    )

    # Control Node (Stanley)
    stanley_node = Node(
        package='ugv_uav_control',
        executable='stanley_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('controller'), "' == 'stanley'"])
        )
    )

    # Control Node (Simple P)
    simple_p_node = Node(
        package='ugv_uav_control',
        executable='simple_p_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('controller'), "' == 'simple_p'"])
        )
    )

    return LaunchDescription([
        controller_arg,
        track_name_arg,
        set_track_texture_action,
        gz_sim_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        robot_bridge,
        camera_bridge,
        perception_node,
        waypoint_follower_node,
        pure_pursuit_node,
        stanley_node,
        simple_p_node
    ])
