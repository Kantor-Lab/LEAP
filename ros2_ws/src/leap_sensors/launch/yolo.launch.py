from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    """
    ======================================================================
    USEFUL COMMANDS & REFERENCE CHEAT SHEET
    ======================================================================
    [YOLO Utilities]
    Launch YOLO:           ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/flir_camera/image_raw use_tracking:=False model:=yolov8n.pt
    Launch YOLO (Seg):     ros2 launch yolo_bringup yoloe.launch.py input_image_topic:=/flir_camera/image_raw use_tracking:=False model:=yoloe-26n-seg.pt
    Set YOLO Classes:      ros2 service call /yolo/set_classes yolo_msgs/srv/SetClasses "{classes: ['person', 'potted tree']}"
    ======================================================================
    """

    # --- Configuration Variables ---
    namespace = 'yolo'
    input_image_topic = '/flir_camera/image_raw'
    debug_detections_topic = "detections"
    use_debug = 'True'

    # --- YOLO Parameters ---
    yolo_params = {
        "model_type": 'YOLOE',
        "model": 'yoloe-26n-seg.pt',
        "device": 'cuda:0',
        "fuse_model": False,
        "yolo_encoding": 'bgr8',
        "enable": True,
        "threshold": 0.5,
        "iou": 0.7,
        "imgsz_height": 480,
        "imgsz_width": 640,
        "half": False,
        "max_det": 300,
        "augment": False,
        "agnostic_nms": False,
        "retina_masks": False,
        "image_reliability": 1,
    }

    # --- Nodes ---
    yolo_node_cmd = LifecycleNode(
        package="yolo_ros",
        executable="yolo_node",
        name="yolo_node",
        namespace=namespace,
        parameters=[yolo_params],
        remappings=[("image_raw", input_image_topic)],
    )
    
    debug_node_cmd = LifecycleNode(
        package="yolo_ros",
        executable="debug_node",
        name="debug_node",
        namespace=namespace,
        parameters=[{"image_reliability": yolo_params["image_reliability"]}],
        remappings=[
            ("image_raw", input_image_topic),
            ("detections", debug_detections_topic),
        ],
        condition=IfCondition(use_debug),
    )

    # --- Event Handlers & Execution Processes ---
    
    # Command to set classes once the node is active
    set_class_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', 
            '/yolo/set_classes', 
            'yolo_msgs/srv/SetClasses', 
            "{classes: ['person', 'potted tree']}"
        ],
        output='both'
    )

    # Trigger the service call when yolo_node reaches the 'active' state
    on_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=yolo_node_cmd,
            goal_state='active',
            entities=[set_class_cmd]
        )
    )

    # --- Assemble Launch Description ---
    ld = LaunchDescription()
    
    ld.add_action(yolo_node_cmd)
    ld.add_action(debug_node_cmd)
    ld.add_action(on_active)

    return ld