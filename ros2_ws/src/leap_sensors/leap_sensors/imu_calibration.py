#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        
        self.subscription = self.create_subscription(
            Imu,
            '/ouster/imu',
            self.imu_callback,
            10
        )
        
        self.get_logger().info("Starting IMU calibration. KEEP THE ROBOT COMPLETELY STILL...")
        
        # Adjust this depending on your IMU's publish rate
        self.num_samples_target = 5000
        self.samples_collected = 0
        
        self.linear_accel_data = []
        self.angular_vel_data = []
        
    def imu_callback(self, msg):
        if self.samples_collected < self.num_samples_target:
            self.linear_accel_data.append([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
            self.angular_vel_data.append([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])
            self.samples_collected += 1
            
            if self.samples_collected % 100 == 0:
                self.get_logger().info(f"Collected {self.samples_collected}/{self.num_samples_target} samples...")
                
        elif self.samples_collected == self.num_samples_target:
            self.calculate_and_print_results()
            self.samples_collected += 1 # Increment once to prevent re-triggering
            
    def calculate_and_print_results(self):
        accel_arr = np.array(self.linear_accel_data)
        gyro_arr = np.array(self.angular_vel_data)
        
        # Calculate means (biases)
        accel_means = np.mean(accel_arr, axis=0)
        gyro_means = np.mean(gyro_arr, axis=0)
        
        # Standard gravity (m/s^2)
        GRAVITY = 9.80665
        
        # The Z bias is the reading minus actual gravity
        z_bias = accel_means[2] - GRAVITY
        
        # Calculate variances (diagonal of the covariance matrix)
        accel_vars = np.var(accel_arr, axis=0)
        gyro_vars = np.var(gyro_arr, axis=0)
        
        self.get_logger().info("\n\n--- CALIBRATION COMPLETE ---")
        
        self.get_logger().info("\n# 1. BIASES")
        self.get_logger().info("# Subtract these from your raw /ouster/imu readings in an IMU relay node:")
        self.get_logger().info(f"linear_acceleration_bias: [{accel_means[0]:.6f}, {accel_means[1]:.6f}, {z_bias:.6f}]")
        self.get_logger().info(f"angular_velocity_bias:    [{gyro_means[0]:.6f}, {gyro_means[1]:.6f}, {gyro_means[2]:.6f}]")
        
        self.get_logger().info("\n# 2. COVARIANCES")
        self.get_logger().info("# Inject these 9-element arrays into your Imu message covariance fields:")
        self.get_logger().info(f"linear_acceleration_covariance: [{accel_vars[0]:.8f}, 0.0, 0.0, 0.0, {accel_vars[1]:.8f}, 0.0, 0.0, 0.0, {accel_vars[2]:.8f}]")
        self.get_logger().info(f"angular_velocity_covariance:    [{gyro_vars[0]:.8f}, 0.0, 0.0, 0.0, {gyro_vars[1]:.8f}, 0.0, 0.0, 0.0, {gyro_vars[2]:.8f}]")
        
        self.get_logger().info("\nShutting down calibrator...")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImuCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Check if context is still valid before shutting down
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()