import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_leap_sim = get_package_share_directory('leap_sim')
    pkg_leap_control = get_package_share_directory('leap_control')

    rviz_config_path = os.path.join(pkg_leap_sim, 'rviz', 'amiga_config.rviz')

    # Launch the simulator
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leap_sim, 'launch', 'gazebo.launch.py')
        )
    )

    # Launch localization within the simulator
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leap_control, 'launch', 'sim_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # For visualization
    rviz_launch = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', rviz_config_path],
       output='screen',
       parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(localization_launch)
    ld.add_action(rviz_launch)

    return ld