#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.twist_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/cmd_vel',
            10
        )
        
        # --- Watchdog Timer Setup ---
        self.last_msg_time = self.get_clock().now()
        self.timeout_duration = 0.25  # seconds before assuming we are stopped
        
        # Check for timeouts at 10Hz
        self.timer = self.create_timer(0.1, self.watchdog_callback)
        
        # Cache a zero-velocity message so we don't have to rebuild it in the loop
        self.zero_msg = Twist() 
        self.get_logger().info("Cmdvel relay with watchdog started.")

    def get_covariance(self):
        # Order: [x, y, z, roll, pitch, yaw]
        covariance = [0.0] * 36
        covariance[0]  = 0.01  # X linear
        covariance[7]  = 0.01  # Y linear
        covariance[35] = 0.5   # Z angular
        return covariance

    def publish_stamped_twist(self, twist_msg):
        stamped_msg = TwistWithCovarianceStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link' 
        
        stamped_msg.twist.twist = twist_msg
        stamped_msg.twist.covariance = self.get_covariance()
        
        self.publisher.publish(stamped_msg)

    def twist_callback(self, msg):
        # Update the watchdog timer
        self.last_msg_time = self.get_clock().now()
        
        # Pass the commanded twist through
        self.publish_stamped_twist(msg)

    def watchdog_callback(self):
        time_since_last_msg = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        
        if time_since_last_msg > self.timeout_duration:
            # We haven't heard from the controller. Publish 0.0 to stop coasting
            # and act as a Zero Velocity Update for the EKF.
            self.publish_stamped_twist(self.zero_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()