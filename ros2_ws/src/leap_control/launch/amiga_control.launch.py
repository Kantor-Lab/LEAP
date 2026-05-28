#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_leap_control = get_package_share_directory('leap_control')

    teleop_launch_path = os.path.join(pkg_leap_control, 'launch', 'teleop.launch.py')
    ekf_config = os.path.join(pkg_leap_control, 'config', 'ekf.yaml')

    teleop_included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch_path)
    )

    control_node = Node(
        package='leap_control',
        executable='amiga_control',
        name='amiga_control',
        output='screen'
    )

    cmdvel_relay_node = Node(
        package='leap_control',
        executable='cmdvel_relay',
        name='cmdvel_relay',
        output='screen'
    )

    imu_relay_node = Node(
        package='leap_control',
        executable='imu_relay',
        name='imu_relay',
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        output='screen',
    )

    ld = LaunchDescription()
    

    ld.add_action(teleop_included_launch)     
    ld.add_action(control_node)
    ld.add_action(cmdvel_relay_node)
    ld.add_action(imu_relay_node)
    ld.add_action(ekf_node)

    return ld