#ifndef IMU_COVARIANCE_INJECTOR_HPP_
#define IMU_COVARIANCE_INJECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuCovarianceInjector : public rclcpp::Node
{
public:
  explicit ImuCovarianceInjector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  double angular_variance_;
  double linear_variance_;
};

#endif  // IMU_COVARIANCE_INJECTOR_HPP_