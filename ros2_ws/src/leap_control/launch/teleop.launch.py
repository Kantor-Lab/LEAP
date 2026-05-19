import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_leap_control = get_package_share_directory('leap_control')

    # Use_sim_time argument so that we can switch between real time (on the real
    #   robot) and simulation time (in Gazebo)
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false', 
        description='Use simulation (Gazebo) clock if true'
    )

    twist_mux_config = os.path.join(pkg_leap_control, 'config', 'twist_mux_params.yaml')
    xbox_config = os.path.join(pkg_leap_control, 'config', 'xbox_joy_params.yaml')

    # Reads hardware inputs from the Xbox controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[xbox_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Translates button presses into velocity commands (cmd_vel_joy)
    teleop_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_node',
        parameters=[xbox_config, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        output='screen'
    )

    # Multiplexes velocity commands (e.g., joystick overrides autonomy) and 
    #   outputs to the differential drive controller
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', 'diff_controller/cmd_vel_unstamped')],
        output='screen',
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(twist_mux_node)

    return ld