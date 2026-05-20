import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_leap_control = get_package_share_directory('leap_control')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # 1. Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    map_file = LaunchConfiguration('map')
    declare_map_file = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(pkg_leap_control, 'maps', 'sim_map'),
        description='Full path to map yaml file to load'
    )

    # 2. Local State Estimation (EKF)
    ekf_config_path = os.path.join(pkg_leap_control, 'config', 'sim_ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    # 3. Global Localization (SLAM Toolbox)
    # Note: Use the mapper_params_localization.yaml file we fixed earlier, 
    # but rename it to mapper_params_sim.yaml to keep things organized.
    slam_loc_config_path = os.path.join(pkg_leap_control, 'config', 'sim_mapper_params_localization.yaml')
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_loc_config_path
        }.items()
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_file)
    ld.add_action(ekf_node)
    ld.add_action(slam_node)
    
    return ld