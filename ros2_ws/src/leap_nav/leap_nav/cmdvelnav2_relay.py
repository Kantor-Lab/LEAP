"""
This is from a pretty old issue I was having where the robot would reach the goal but then
continue moving forwards from the last thing nav2 sent. I added this watchdog to monitor the
path topic and stop the robot if it hasn't received a new path in a while. You can see that I
essentially disabled it by setting timeout_duration to something ridiculous.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

class TwistToTwist(Node):
    def __init__(self):
        super().__init__('twist_to_twist')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav2_raw',
            self.twist_callback,
            10
        )

        self.checker = self.create_subscription(
            Path,
            '/received_global_plan',
            self.path_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_nav2',
            10
        )
        
        # --- Watchdog Timer Setup ---
        self.triggered = False
        self.last_msg_time = self.get_clock().now()
        # self.timeout_duration = 0.25  # seconds before assuming we are stopped
        self.timeout_duration = 10000000000
        
        # Check for timeouts at 10Hz
        self.timer = self.create_timer(0.1, self.watchdog_callback)
        
        self.zero_msg = Twist() 
        self.get_logger().info("Cmdvel_nav2 relay with watchdog started.")

    def twist_callback(self, msg):
        if self.triggered: return
        self.publisher.publish(msg)

    def path_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.triggered = False

    def watchdog_callback(self):
        time_since_last_msg = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if time_since_last_msg > self.timeout_duration:
            self.triggered = True
            self.publisher.publish(self.zero_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwist()
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