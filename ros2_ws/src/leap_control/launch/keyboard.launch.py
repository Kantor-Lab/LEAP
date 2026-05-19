"""
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key

When using a launch file, nodes lose direct access to the physical layer. To avoid
teleop_twist_keyboard panicking when it doesn't detect a keyboard, we launch it with xterm
which gives it its own terminal and direct access to the keyboard. If you don't want a
separate terminal, you can use the ros2 run command above.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Use_sim_time argument so that we can switch between real time (on the real
    #   robot) and simulation time (in Gazebo)
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch the main teleop controller
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leap_control, 'launch', 'teleop.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Intercepts keystrokes and outputs them to the twist_mux
    keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_key')],
        output='screen',
        prefix=['xterm -e']  # See comment at top of file
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(keyboard_node)

    return ld