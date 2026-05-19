import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_leap_desc = get_package_share_directory('leap_desc')
    
    # Use_sim_time argument so that we can switch between real time (on the real
    #   robot) and simulation time (in Gazebo)
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false', 
        description='Use simulation (Gazebo) clock if true'
    )

    # We can directly use the xacro file instead of having to generate the urdf first
    robot_xacro_path = os.path.join(pkg_leap_desc, 'urdf', 'top_level_amiga.urdf.xacro')
    robot_desc = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', robot_xacro_path]),
        value_type=str
    )
    # robot_urdf_path = os.path.join(pkg_leap_sim, 'urdf', 'top_level_amiga.urdf')
    # with open(robot_urdf_path, 'r') as infp:
    #     robot_desc = infp.read()

    # Publish the robot state to the /tf topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(robot_state_publisher)

    return ld