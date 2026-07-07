#!/usr/bin/env python3

import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class AngularRampTestNode(Node):
    def __init__(self):
        super().__init__('angular_ramp_test_node')

        # --- Configuration Parameters ---
        self.max_angular_vel = 1.0  # Target max angular velocity in rad/s
        self.ramp_duration = 600.0   # How many seconds it takes to reach max velocity
        self.timer_period = 0.1     # Control loop period in seconds (10 Hz)
        
        # Calculate the increment needed per timer tick
        self.increment = (self.max_angular_vel / self.ramp_duration) * self.timer_period

        # --- State Variables ---
        self.current_cmd_w = 0.0
        self.odom_filtered_w = 0.0
        self.odom_global_w = 0.0

        # --- CSV Setup ---
        self.csv_filename = 'angular_vel_test_log.csv'
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write the header row
        self.csv_writer.writerow(['timestamp_sec', 'cmd_vel_w', 'filtered_odom_w', 'global_odom_w'])

        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav2_raw', 10)
        
        self.filtered_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_cb, 10)
            
        self.global_sub = self.create_subscription(
            Odometry, '/odometry/global', self.global_cb, 10)

        # --- Timer ---
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        self.start_time = self.get_clock().now()

        self.get_logger().info(f"Starting angular ramp test. Ramping to {self.max_angular_vel} rad/s over {self.ramp_duration} seconds.")

    def filtered_cb(self, msg):
        """Extract angular.z from /odometry/filtered"""
        self.odom_filtered_w = msg.twist.twist.angular.z

    def global_cb(self, msg):
        """Extract angular.z from /odometry/global"""
        self.odom_global_w = msg.twist.twist.angular.z

    def timer_cb(self):
        """Main control and logging loop"""
        now = self.get_clock().now()
        timestamp = now.nanoseconds / 1e9

        # 1. Ramp up logic (cap it at max_angular_vel)
        if self.current_cmd_w < self.max_angular_vel:
            self.current_cmd_w = min(self.max_angular_vel, self.current_cmd_w + self.increment)

        # 2. Publish the command
        twist_msg = Twist()
        twist_msg.angular.z = self.current_cmd_w
        self.cmd_pub.publish(twist_msg)

        # 3. Log the current states to CSV
        self.csv_writer.writerow([
            timestamp, 
            self.current_cmd_w, 
            self.odom_filtered_w, 
            self.odom_global_w
        ])

    def destroy_node(self):
        """Ensure the CSV file is safely closed when the node is destroyed"""
        self.csv_file.close()
        self.get_logger().info(f"Data saved to {self.csv_filename}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AngularRampTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user. Stopping the robot...")
    finally:
        # Safety feature: send a 0 velocity command before exiting
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()