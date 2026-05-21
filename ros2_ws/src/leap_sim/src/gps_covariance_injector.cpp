#include "leap_sim/gps_covariance_injector.hpp"
#include <memory>

GpsCovarianceInjector::GpsCovarianceInjector(const rclcpp::NodeOptions & options)
: Node("gps_covariance_injector", options)
{
  // Horizontal (stddev = 0.01)
  horizontal_variance_ = 0.01 * 0.01;
  
  // Vertical (stddev = 0.01)
  vertical_variance_ = 0.01 * 0.01;

  // Subscribe to the bridged topic (that has 0 covariance)
  sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "gps/fix_raw", 10,
    std::bind(&GpsCovarianceInjector::gps_callback, this, std::placeholders::_1));

  // Publish the corrected topic for GTSAM
  pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
  
  RCLCPP_INFO(this->get_logger(), "GPS Covariance Injector Node started.");
}

void GpsCovarianceInjector::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  auto corrected_msg = *msg;

  // Inject 3x3 diagonal covariance matrix (East, North, Up)
  corrected_msg.position_covariance.fill(0.0);
  corrected_msg.position_covariance[0] = horizontal_variance_; // East variance
  corrected_msg.position_covariance[4] = horizontal_variance_; // North variance
  corrected_msg.position_covariance[8] = vertical_variance_;   // Up variance

  // Set the type flag to indicate the diagonal is populated
  corrected_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  pub_->publish(corrected_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsCovarianceInjector>());
  rclcpp::shutdown();
  return 0;
}