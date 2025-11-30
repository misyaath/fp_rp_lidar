import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Process URDF
    xacro_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'my_robot.urdf.xacro'
    )
    doc = xacro.process_file(xacro_path)
    robot_description_config = doc.toxml()

    # Paths
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('my_robot_description'),
            'maps',
            'my_map.yaml'))
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('my_robot_description'),
            'config',
            'nav_param.yaml'))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Include Nav2 bringup (planner, controller, AMCL, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': 'true',
                'use_respawn': 'true',
                'log_level': 'error'
            }.items(),
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': False
            }]
        ),

        # Joint State Publisher (optional, ensures TF completeness)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_gui': False,
                'source_list': ['base_joint', 'laser_mount']
            }]
        ),

        # Odometry Publisher
        Node(
            package='fp_odom',
            executable='fp_odom_receiver',
            name='fp_odom_receiver',
            output='screen',
            parameters=[{'publish_rate_hz': 100}]
        ),

        # RPLIDAR Driver
        Node(
            package='fp_rplidar_ros2',
            executable='fp_rplidar_publisher',
            name='fp_rplidar_publisher',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'frame_id': 'laser_frame'},
                {'scan_mode': 'Standard'}
            ]
        ),

        # Motor Controller
        Node(
            package='nav_motor_controller',
            executable='nav_motor_controller_node',
            name='nav_motor_controller_node',
            output='screen'
        ),
    ])
