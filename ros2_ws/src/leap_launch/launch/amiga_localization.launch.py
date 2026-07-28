import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_leap_control = get_package_share_directory('leap_control')

    ekf_local_config = os.path.join(pkg_leap_control, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_leap_control, 'config', 'ekf_global.yaml')
    map_path = os.path.join(pkg_leap_control, 'maps', 'cmu.ply')
    terrain_path = os.path.join(pkg_leap_control, 'maps', 'cmu_dtm.ply')
    transform_path = os.path.join(pkg_leap_control, 'config', 'T_world_utm.txt')

    use_map_arg = DeclareLaunchArgument(
        'use_map',
        default_value='true',
        description='Whether to use the map frame'
    )

    map_ply_arg = DeclareLaunchArgument(
        'map_ply',
        default_value=map_path,
        description='Path to the map point cloud file'
    )

    terrain_ply_arg = DeclareLaunchArgument(
        'terrain_ply',
        default_value=terrain_path,
        description='Path to the terrain dtm point cloud file'
    )

    initial_yaw_deg_arg = DeclareLaunchArgument(
        'initial_yaw_deg',
        default_value='0.0',
        description='Initial yaw in degrees'
    )

    transform_file_arg = DeclareLaunchArgument(
        'transform_file',
        default_value=transform_path,
        description='Path to the T_world_utm.txt rigid transform file',
    )

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_filter_node',
        parameters=[ekf_local_config],
        output='screen',
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_filter_node',
        parameters=[ekf_global_config],
        output='screen',
        remappings=[
            ('odometry/filtered', '/odometry/global')
        ],
        condition=IfCondition(LaunchConfiguration('use_map'))
    )

    gps_transform_node = Node(
        package='leap_control',
        executable='gps_transform',
        name='gps_transform',
        output='screen',
        parameters=[{
            'transform_file': LaunchConfiguration('transform_file')
        }],
    )

    gps_filter_node = Node(
        package='leap_control',
        executable='gps_filter',
        name='gps_filter',
        output='screen',
    )

    icp_node = Node(
        package='leap_icp',
        executable='map_odom_localizer',
        name='map_odom_localizer',
        output='screen',
        parameters=[{
            'base_frame': 'base_footprint',

            'map_ply_path': LaunchConfiguration('map_ply'),
            'terrain_ply_path': LaunchConfiguration('terrain_ply'),
            'voxel_leaf_map': 0.3,
            'voxel_leaf_scan': 0.3,
            'vgicp_resolution': 1,
            'vgicp_max_iterations': 64,
            'vgicp_max_corresp_dist': 3.0,

            # --- Initialization Parameters ---
            'init_mode': 'position_only',
            'init_x': 0.0,   # 22
            'init_y': 2.0,  # 121
            
            # Heading tuning parameters
            'init_heading_candidates': 16,
            'init_search_max_iter': 20,
        }],
        condition=IfCondition(LaunchConfiguration('use_map'))
    )

    static_tf_map_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=UnlessCondition(LaunchConfiguration('use_map'))
    )

    ld = LaunchDescription()
    ld.add_action(use_map_arg)
    ld.add_action(map_ply_arg)
    ld.add_action(terrain_ply_arg)
    ld.add_action(initial_yaw_deg_arg)
    ld.add_action(transform_file_arg)
    ld.add_action(ekf_local_node)
    ld.add_action(ekf_global_node)
    ld.add_action(gps_transform_node)
    ld.add_action(gps_filter_node)
    ld.add_action(icp_node)
    ld.add_action(static_tf_map_odom_node)
    return ld