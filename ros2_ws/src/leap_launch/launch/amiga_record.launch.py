import os
import time
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Define the absolute path to your bags directory. 
    # NOTE: os.path.expanduser('~') grabs your home directory (e.g., /home/username). 
    # If LEAP is not in your home folder, change this to your exact path (e.g., '/absolute/path/to/LEAP/bags')
    bag_directory = os.path.expanduser('~/LEAP/bags')
    
    # 2. Make sure the LEAP/bags directory actually exists before trying to save to it
    os.makedirs(bag_directory, exist_ok=True)

    # 3. Combine the path with the timestamped bag name
    timestamp = time.strftime('%Y_%m_%d-%H_%M_%S')
    default_bag_path = os.path.join(bag_directory, f"{timestamp}")

    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value=default_bag_path,
        description='Name/path of the output bag directory'
    )

    # Your topic list
    topics_to_record = [
        '/clicked_point',
        '/diagnostics',
        '/fix',
        '/flir_camera/camera_info',
        '/flir_camera/image_raw',
        # '/flir_camera/image_raw/compressed',
        # '/flir_camera/image_raw/compressedDepth',
        # '/flir_camera/image_raw/theora',
        '/flir_camera/meta',
        '/goal_pose',
        '/heading',
        '/initialpose',
        '/joint_states',
        '/ouster/imu',
        # '/ouster/imu_packets',
        '/ouster/lidar_packets',
        '/ouster/metadata',
        # '/ouster/nearir_image',
        '/ouster/os_driver/transition_event',
        # '/ouster/points',
        # '/ouster/range_image',
        # '/ouster/reflec_image',
        # '/ouster/scan',
        # '/ouster/signal_image',
        '/ouster/telemetry',
        '/parameter_events',
        '/robot_description',
        '/rosout',
        '/tf',
        '/tf_static',
        '/time_reference',
        # '/vel',
        # '/yolo/dbg_image',
        # '/yolo/debug_node/transition_event',
        '/yolo/detections',
        # '/yolo/dgb_bb_markers',
        # '/yolo/dgb_kp_markers',
        # '/yolo/yolo_node/transition_event',

        '/bond',
        '/clock',
        '/cmd_vel',
        '/cmd_vel_joy',
        '/cmd_vel_key',
        '/cmd_vel_nav',
        '/cmd_vel_raw',
        '/gps/filtered',
        '/gps/fix',
        '/imu',
        '/joy',
        '/joy/set_feedback',
        '/localizer/pose',
        '/map',
        '/map_server/transition_event',
        '/move_base_simple/goal',
        '/odom',
        '/odometry/filtered',
        '/odometry/global',
        '/odometry/gps',
        '/set_pose',
        '/vel',
    ]

    # Construct the ros2 bag record command
    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', LaunchConfiguration('bag_name')] + topics_to_record,
        output='screen'
    )

    return LaunchDescription([
        bag_name_arg,
        record_process
    ])