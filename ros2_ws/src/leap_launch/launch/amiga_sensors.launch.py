import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    ======================================================================
    USEFUL COMMANDS & REFERENCE CHEAT SHEET
    ======================================================================
    [Network Configuration]
    Reach IP: http://192.168.2.15
    Ouster IP: http://169.254.239.217/
    Linux memory setting: sysctl net.core.rmem_max (should be 2147483647)

    [Common Commands]
    Reach RTK driver:      ros2 run reach_ros_node nmea_tcp_driver --ros-args -p host:="192.168.2.15" -p port:="9001"

    [YOLO Utilities]
    Launch YOLO:           ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/flir_camera/image_raw use_tracking:=False model:=yolov8n.pt
    Launch YOLO (Seg):     ros2 launch yolo_bringup yoloe.launch.py input_image_topic:=/flir_camera/image_raw use_tracking:=False model:=yoloe-26n-seg.pt
    Set YOLO Classes:      ros2 service call /yolo/set_classes yolo_msgs/srv/SetClasses "{classes: ['person', 'potted tree']}"

    [Calibration Commands]
    Camera calibration:    ros2 run camera_calibration cameracalibrator --size 6x8 --square .025 --pattern 'chessboard' --ros-args -r image:=/flir_camera/image_raw -p camera:=/flir_camera
    ======================================================================
    """

    # --- Package Directories ---
    pkg_leap_sensors = get_package_share_directory('leap_sensors')
    pkg_leap_desc = get_package_share_directory('leap_desc')
    ouster_pkg = get_package_share_directory('ouster_ros')

    # --- File Paths ---
    flir_calibration_path = 'file://' + os.path.join(pkg_leap_sensors, 'config', 'flir_calib.yaml')
    cam_ext_calibration_path = os.path.join(pkg_leap_sensors, 'config', 'ouster_flir_ext_cal2.txt')
    blackfly_settings_path = os.path.join(pkg_leap_sensors, 'config', 'blackfly_s_trigger.yaml')
    ouster_settings_path = os.path.join(pkg_leap_sensors, 'config', 'ouster_params.yaml')
    rviz_config_path = os.path.join(pkg_leap_sensors, 'rviz', 'sensor_module.rviz')

    with open(cam_ext_calibration_path, 'r') as f:
        cam_ext_cal_args = f.read().split()

    # --- Launch Configurations ---
    use_yolo = LaunchConfiguration('yolo')
    
    declare_yolo = DeclareLaunchArgument(
        'yolo', 
        default_value='false', 
        description='Launch YOLO node'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz', 
        default_value='false', 
        description='Open RViz.'
    )

    # --- Nodes & Launch Descriptions ---

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_leap_desc, 'launch', 'rsp.launch.py')),
    )

    cal_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='both',
        arguments=cam_ext_cal_args
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    start_blackfly_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leap_sensors, 'launch', 'blackfly_s.launch.py')
        ),
        launch_arguments={
            'camera_type': 'blackfly_s',
            'serial': "'25483480'",
            'config_yaml': blackfly_settings_path,
            'calib_url': flir_calibration_path
        }.items()
    )

    start_ouster_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ouster_pkg, 'launch', 'driver.launch.py')
        ),
        launch_arguments={
            'params_file': ouster_settings_path,
            'viz': 'False'
        }.items()
    )

    reach_m2_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 38400,
            'frame_id': 'reach',
            'use_GNSS_time': False, 
            'time_ref_source': "gps", 
            'useRMC': False
        }], 
        output='screen'
    )

    start_yolo_ros_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leap_sensors, 'launch', 'yolo.launch.py')
        ),
        condition=IfCondition(use_yolo)
    )

    # --- Assemble Launch Description ---
    ld = LaunchDescription()
    
    # Declarations
    ld.add_action(declare_rviz)
    ld.add_action(declare_yolo)
    
    # Actions
    ld.add_action(rsp_launch)
    ld.add_action(cal_cam_tf)
    ld.add_action(rviz_node)
    ld.add_action(start_blackfly_cmd)
    ld.add_action(start_ouster_cmd)
    ld.add_action(reach_m2_node)
    ld.add_action(start_yolo_ros_cmd)

    return ld