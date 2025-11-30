from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch_ros.actions import LifecycleNode
from launch.actions import ExecuteProcess
import xacro
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'my_robot.urdf.xacro'
    )

    doc = xacro.process_file(xacro_path)
    robot_description_config = doc.toxml()

    slam_node_action = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[{
            'use_sim_time': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',  # CRITICAL parameter
            'mode': 'mapping',
            'use_map_saver': True,
            'use_lifecycle_manager': True,
            'loop_closure_enabled': True,
            'minimum_loop_closure_score': 0.8,
            'use_pose_graph': True,
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': '',
            'debug_logging': False,
            'throttle_scans': 3,
            'transform_publish_period': 0.02,  # if 0 never publishes odometry
            'map_update_interval': 5.0,
            'resolution': 0.05,
            'max_laser_range': 12.0,  # for rastering images
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,  # // program needs a larger stack size to serialize large maps
            'enable_interactive_mode': True,

            # General Parameters
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.5,
            'minimum_travel_heading': 0.5,
            'scan_buffer_size': 10,
            'scan_buffer_maximum_scan_distance': 10.0,
            'link_match_minimum_response_fine': 0.1,
            'link_scan_maximum_distance': 1.5,
            'loop_search_maximum_distance': 3.0,
            'do_loop_closing': True,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_coarse': 0.35,
            'loop_match_minimum_response_fine': 0.45,

            # Correlation Parameters - Correlation Parameters
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1,

            # Correlation Parameters - Loop Closure Parameters
            'loop_search_space_dimension': 8.0,
            'loop_search_space_resolution': 0.05,
            'loop_search_space_smear_deviation': 0.03,

            # Scan Matcher Parameters
            'distance_variance_penalty': 0.5,
            'angle_variance_penalty': 1.0,

            'fine_search_angle_offset': 0.00349,
            'coarse_search_angle_offset': 0.349,
            'coarse_angle_resolution': 0.0349,
            'minimum_angle_penalty': 0.9,
            'minimum_distance_penalty': 0.5,
            'use_response_expansion': True,
            'min_pass_through': 2,
            'occupancy_threshold': 0.1,

        }]
    )

    return LaunchDescription([

        # 🚗 Odometry publisher
        Node(
            package='fp_odom',
            executable='fp_odom_receiver',
            name='fp_odom_receiver',
            output='screen',
            parameters=[
                {'publish_rate_hz': 100}
            ]
        ),

        # 🧠 Robot State Publisher with xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_description_config
            }]
        ),

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

        # 🌀 RPLIDAR C1 driver
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

        # ⚙️ Motor controller
        Node(
            package='motor_controller',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),

        # 🗺️ SLAM Toolbox in mapping mode
        TimerAction(period=5.0, actions=[slam_node_action]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['slam_toolbox']
            }]
        )

    ])
