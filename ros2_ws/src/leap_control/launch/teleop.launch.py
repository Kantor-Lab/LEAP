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

    xbox_config = os.path.join(pkg_leap_control, 'config', 'xbox_params.yaml')
    twist_mux_config = os.path.join(pkg_leap_control, 'config', 'mux_params.yaml')

    # Reads hardware inputs from the Xbox controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            xbox_config,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Translates button presses into velocity commands (cmd_vel_joy)
    joy_teleop_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            xbox_config,
            {
                'use_sim_time': use_sim_time,
                'publish_stamped_twist': False  # The mux expects unstamped messages
            }
        ],
        remappings=[('/cmd_vel', 'cmd_vel_joy')],
        output='screen'
    )

    # Multiplexes velocity commands (e.g., joystick overrides autonomy) and 
    #   outputs to the differential drive controller
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[
            twist_mux_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/cmd_vel_out', '/cmd_vel_raw')],
        output='screen',
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(twist_mux_node)

    return ld