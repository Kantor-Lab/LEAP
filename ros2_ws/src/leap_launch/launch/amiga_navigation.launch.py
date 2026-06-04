import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_leap_control = get_package_share_directory('leap_control')
    map_yaml_path = os.path.join(pkg_leap_control, 'maps', 'cmu_occ.yaml')

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
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(nav2_server_node)
    ld.add_action(nav2_manager_node)

    return ld