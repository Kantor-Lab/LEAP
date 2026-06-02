import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_leap_control = get_package_share_directory('leap_control')

    control_launch_path = os.path.join(pkg_leap_control, 'launch', 'amiga_control.launch.py')
    ekf_local_config = os.path.join(pkg_leap_control, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_leap_control, 'config', 'ekf_global.yaml')
    map_path = os.path.join(pkg_leap_control, 'maps', 'cmu.ply')

    map_ply_arg = DeclareLaunchArgument(
        'map_ply',
        default_value=map_path,
        description='Path to the map point cloud file'
    )

    use_gps_init_arg = DeclareLaunchArgument(
        'use_gps_init',
        default_value='true',
        description='Use GPS for initial localization'
    )

    initial_yaw_deg_arg = DeclareLaunchArgument(
        'initial_yaw_deg',
        default_value='0.0',
        description='Initial yaw in degrees'
    )

    control_included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_path)
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
        ]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            'frequency': 30.0,
            'delay': 3.0,
            'yaw_offset': 0.0,
            'magnetic_declination_radians': -0.164,
            'zero_altitude': False,
            'publish_filtered_gps': True,
            'broadcast_cartesian_transform': False,
            'use_odometry_yaw': False
        }],
        remappings=[
            ('odometry/filtered', '/odometry/global')
        ]
    )

    icp_node = Node(
        package='leap_icp',
        executable='map_odom_localizer',
        name='map_odom_localizer',
        output='screen',
        parameters=[{
            'map_ply_path': LaunchConfiguration('map_ply'),
            'voxel_leaf_map': 0.3,
            'voxel_leaf_scan': 0.1,
            'vgicp_resolution': 1,
            'vgicp_max_iterations': 64,
            'vgicp_max_corresp_dist': 1.5,

            # --- Initialization Parameters ---
            'init_mode': 'position_only',
            'init_x': 0.0,
            'init_y': 2.0,
            
            # Heading tuning parameters
            'init_heading_candidates': 16,
            'init_search_max_iter': 20,
        }]
    )

    ply_pub_node = Node(
        package='leap_control',
        executable='ply_publisher',
        name='ply_publisher',
        output='screen',
        parameters=[{
            'map_ply_path': LaunchConfiguration('map_ply')
        }]
    )

    ld = LaunchDescription()
    ld.add_action(map_ply_arg)
    ld.add_action(use_gps_init_arg)
    ld.add_action(initial_yaw_deg_arg)
    ld.add_action(control_included_launch)
    ld.add_action(ekf_local_node)
    ld.add_action(ekf_global_node)
    ld.add_action(navsat_transform_node)
    ld.add_action(icp_node)
    # ld.add_action(ply_pub_node)

    return ld