import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_leap_sim = get_package_share_directory('leap_sim')
    pkg_leap_desc = get_package_share_directory('leap_desc')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_leap_sim, 'worlds', 'tree_rows.world')
    models_path = os.path.join(pkg_leap_sim, 'models')
    sim_urdf_path = os.path.join(pkg_leap_sim, 'urdf', 'amiga_sim.xacro')
    rviz_config_path = os.path.join(pkg_leap_sim, 'rviz', 'amiga_config.rviz')
    ekf_config_path = os.path.join(pkg_leap_sim, 'config', 'ekf_config.yaml')
    
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )

    # Whether to open RViz or not
    open_rviz = DeclareLaunchArgument(
        'open_rviz', 
        default_value='true', 
        description='Open RViz.'
    )

    # Launch the gazebo server with the specified world
    # -r starts it unpaused; -v4 sets verbosity to level 4 (helps catch missing
    #   model errors)
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v4 {world_path}', 
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Launch the robot state publisher to publish the robot's state to the /tf
    #   topic via the leap_desc package
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leap_desc, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'urdf_file': sim_urdf_path  # Overrides the default hardware-only URDF
        }.items()
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=[ 
            '-name', 'amiga_sim', 
            '-topic', 'robot_description', 
            '-x', '-8.0', '-y', '0.0', '-z', '0.0'
        ], 
        output='screen', 
    )

    # Bridge the clock and lase scan (lidar) topics from Gazebo to ROS 2 so we
    #   can use them in other nodes and RViz
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    # Bridges the camera image topic from Gazebo to ROS 2
    # This uses a different package which is why it's not included in the previous
    #   bridge node
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Reads the wheel positions from Gazebo and publishes them to /joint_states
    #   so the robot_state_publisher knows exactly where the wheels are pointing
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen",
    )

    # Listens to /diff_controller/cmd_vel_unstamped and actually spins the wheels
    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"],
        output="screen",
    )

    # RViz visualization of the robot and its sensors
    rviz_launch = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', rviz_config_path],
       output='screen',
       parameters=[{'use_sim_time' : True}],
       condition=IfCondition(LaunchConfiguration('open_rviz'))
    )

    # EKF node to fuse the wheel odometry and IMU data to get a better estimate of 
    #   the robot's pose
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_path, {'use_sim_time' : True}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_launch)
    ld.add_action(rsp_launch)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(image_bridge)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_diff_drive_controller)
    ld.add_action(open_rviz)
    ld.add_action(rviz_launch)
    ld.add_action(robot_localization)

    return ld