import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
import open3d as o3d
import numpy as np

class PlyPublisher(Node):
    def __init__(self):
        super().__init__('ply_publisher')
        
        # 1. Declare parameters
        self.declare_parameter('map_ply_path', 'map.ply')
        self.declare_parameter('frame_id', 'map')
        
        self.ply_path = self.get_parameter('map_ply_path').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # 2. Setup Publisher
        self.publisher_ = self.create_publisher(PointCloud2, 'map_pointcloud', 10)
        
        # 3. Load the map once into memory
        self.cloud_msg = self.load_ply_to_ros_msg()

        if self.cloud_msg is not None:
            # Publish every 1 second so RViz catches it whenever it opens
            self.timer = self.create_timer(1.0, self.timer_callback)
            self.get_logger().info(f"Publishing {self.ply_path} on '/map_pointcloud'")

    def load_ply_to_ros_msg(self):
        self.get_logger().info(f"Loading PLY file: {self.ply_path}...")
        try:
            # Read the PLY file using Open3D
            pcd = o3d.io.read_point_cloud(self.ply_path)
            
            if pcd.is_empty():
                self.get_logger().error("Point cloud is empty or file not found!")
                return None
                
            # Convert Open3D point cloud to an Nx3 numpy array
            points = np.asarray(pcd.points)
            
            # Create the ROS Header
            header = Header()
            header.frame_id = self.frame_id
            
            # Use ROS 2 native tools to convert the XYZ numpy array to PointCloud2
            # (We leave the timestamp empty here, and update it right before publishing)
            msg = point_cloud2.create_cloud_xyz32(header, points)
            
            self.get_logger().info(f"Successfully cached {len(points)} points.")
            return msg
            
        except Exception as e:
            self.get_logger().error(f"Failed to load PLY: {e}")
            return None

    def timer_callback(self):
        if self.cloud_msg is not None:
            # Update the timestamp to 'now' so RViz doesn't complain about old TF data
            self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlyPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()