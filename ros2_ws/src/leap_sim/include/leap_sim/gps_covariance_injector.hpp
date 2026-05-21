#ifndef GPS_COVARIANCE_INJECTOR_HPP_
#define GPS_COVARIANCE_INJECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GpsCovarianceInjector : public rclcpp::Node
{
public:
  explicit GpsCovarianceInjector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;

  double horizontal_variance_;
  double vertical_variance_;
};

#endif  // GPS_COVARIANCE_INJECTOR_HPP_