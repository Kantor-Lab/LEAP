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
    nav2_params_path = os.path.join(leap_nav, 'config', 'nav2_params.yaml')

    nav2_navigation_group = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav2_raw'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={
                    'params_file': nav2_params_path,
                    'use_sim_time': 'False',
                    'map_yaml_path': map_yaml_path  # <-- Passing the map path down
                }.items()
            )
        ]
    )

    cmdvelnav2_relay_node = Node(
        package='leap_nav',
        executable='cmdvelnav2_relay',
        name='cmdvelnav2_relay',
        output='screen'
    )

    row_mission_node = Node(
        package='leap_nav',
        executable='row_mission_node',
        name='row_mission_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(nav2_navigation_group)
    ld.add_action(cmdvelnav2_relay_node)
    ld.add_action(row_mission_node)

    return ld