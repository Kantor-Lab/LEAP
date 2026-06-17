import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    pkg_leap_control = get_package_share_directory('leap_control')
    map_yaml_path = os.path.join(pkg_leap_control, 'maps', 'cmu_occ.yaml')

    leap_nav = get_package_share_directory('leap_nav')
    nav2_launch_path = os.path.join(leap_nav, 'launch', 'navigation_launch.py')

    nav2_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_path}
        ]
    )

    nav2_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    nav2_navigation_group = GroupAction(
        actions=[
            # Catch the default /cmd_vel and push it to /cmd_vel_nav2 (/cmd_vel_nav is an internal nav2 topic)
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav2'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                # launch_arguments={
                #     # 'params_file': nav2_params_path
                #     'use_velocity_smoother': 'false'
                # }.items()
            )
        ]
    )

    ld = LaunchDescription()

    ld.add_action(nav2_server_node)
    ld.add_action(nav2_manager_node)
    ld.add_action(nav2_navigation_group)

    return ld