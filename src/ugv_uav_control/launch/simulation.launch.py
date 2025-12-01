import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ugv_uav = get_package_share_directory('ugv_uav_control')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set GZ_SIM_RESOURCE_PATH to include our package share directory so Gazebo can find the models
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

    # Spawn Custom TurtleBot3 with ArUco
    # We use our custom model path
    custom_model_path = os.path.join(pkg_ugv_uav, 'models', 'turtlebot3_burger_aruco', 'model.sdf')
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger',
            '-file', custom_model_path,
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    # Bridge for Robot Topics (cmd_vel, odom, joint_states, tf)
    # Reusing the standard bridge config from turtlebot3_gazebo
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

    # Bridge for overhead camera
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/overhead_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # Control Node
    control_node = Node(
        package='ugv_uav_control',
        executable='line_follower',
        output='screen'
    )

    return LaunchDescription([
        gz_sim_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        robot_bridge,
        camera_bridge,
        control_node
    ])
