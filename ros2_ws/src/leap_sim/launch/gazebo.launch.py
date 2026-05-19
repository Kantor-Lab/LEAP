import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_leap_sim = get_package_share_directory('leap_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_leap_sim, 'worlds', 'tree_rows.world')
    models_path = os.path.join(pkg_leap_sim, 'models')

    # We can directly use the xacro file instead of having to generate the urdf first
    robot_xacro_path = os.path.join(pkg_leap_sim, 'urdf', 'top_level_amiga.urdf.xacro')
    robot_desc = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', robot_xacro_path
        ]),
        value_type=str # Ensure the result is treated as a string
    )
    # robot_urdf_path = os.path.join(pkg_leap_sim, 'urdf', 'top_level_amiga.urdf')
    # with open(robot_urdf_path, 'r') as infp:
    #     robot_desc = infp.read()
    
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )

    # Launch the gazebo server with the specified world
    # -r starts it unpaused; -v4 sets verbosity to level 4 (helps catch missing
    #   model errors)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v4 {world_path}', 
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Publish the robot state to the /tf topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=[ 
            '-name', 'top_level_amiga', 
            '-topic', 'robot_description', 
            '-x', '-8.0', '-y', '0.0', '-z', '0.0'
        ], 
        output='screen', 
    )

    # Bridge the /clock topic from Gazebo to ROS 2 so that robot_state_publisher
    #   and other nodes can use simulation time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(clock_bridge)

    return ld