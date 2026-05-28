#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')
        
        self.subscription = self.create_subscription(
            Imu,
            '/ouster/imu',
            self.imu_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Imu,
            '/imu',
            10
        )
        
        # --- Calibration Constants from Logs ---
        self.linear_accel_bias = [-0.144932, 0.112021, 0.198726]
        self.angular_vel_bias = [-0.004937, 0.000345, -0.013345]
        
        self.linear_accel_cov = [
            0.00135564, 0.0, 0.0, 
            0.0, 0.00146361, 0.0, 
            0.0, 0.0, 0.00039451
        ]
        
        self.angular_vel_cov = [
            0.00000078, 0.0, 0.0, 
            0.0, 0.00000063, 0.0, 
            0.0, 0.0, 0.00000195
        ]

        self.get_logger().info("IMU Relay started. Publishing clean data to /imu.")

    def imu_callback(self, msg):
        calibrated_msg = Imu()
        
        # 1. Pass through the header and orientation data
        calibrated_msg.header = msg.header
        calibrated_msg.orientation = msg.orientation
        calibrated_msg.orientation_covariance = msg.orientation_covariance
        
        # 2. Subtract Biases from Linear Acceleration
        calibrated_msg.linear_acceleration.x = msg.linear_acceleration.x - self.linear_accel_bias[0]
        calibrated_msg.linear_acceleration.y = msg.linear_acceleration.y - self.linear_accel_bias[1]
        calibrated_msg.linear_acceleration.z = msg.linear_acceleration.z - self.linear_accel_bias[2]
        
        # 3. Subtract Biases from Angular Velocity
        calibrated_msg.angular_velocity.x = msg.angular_velocity.x - self.angular_vel_bias[0]
        calibrated_msg.angular_velocity.y = msg.angular_velocity.y - self.angular_vel_bias[1]
        calibrated_msg.angular_velocity.z = msg.angular_velocity.z - self.angular_vel_bias[2]
        
        # 4. Apply Calculated Covariance Matrices
        calibrated_msg.linear_acceleration_covariance = self.linear_accel_cov
        calibrated_msg.angular_velocity_covariance = self.angular_vel_cov
        
        # 5. Publish the cleaned message
        self.publisher.publish(calibrated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRelay()
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