#include "leap_sim/imu_covariance_injector.hpp"
#include <memory>

ImuCovarianceInjector::ImuCovarianceInjector(const rclcpp::NodeOptions & options)
: Node("imu_covariance_injector", options)
{
    // Angular Velocity (stddev = 0.0002)
    angular_variance_ = 0.0002 * 0.0002;
    
    // Linear Acceleration (stddev = 0.017)
    linear_variance_ = 0.017 * 0.017;

    // Subscribe to the bridged topic (that has 0 covariance)
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_raw", 10,
        std::bind(&ImuCovarianceInjector::imu_callback, this, std::placeholders::_1));

    // Publish the corrected topic for GTSAM
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/ouster/imu", 10);
    
    RCLCPP_INFO(this->get_logger(), "IMU Covariance Injector Node started.");
}

void ImuCovarianceInjector::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    auto corrected_msg = *msg;

    // Inject Orientation Covariance 
    // Setting the first element to -1.0 means "orientation is unknown/unestimated"
    // Just changing the first element is sufficient for GTSAM to ignore orientation
    corrected_msg.orientation_covariance.fill(0.0);
    corrected_msg.orientation_covariance[0] = -1.0;

    // Inject Angular Velocity Covariance
    corrected_msg.angular_velocity_covariance.fill(0.0);
    corrected_msg.angular_velocity_covariance[0] = angular_variance_; // x
    corrected_msg.angular_velocity_covariance[4] = angular_variance_; // y
    corrected_msg.angular_velocity_covariance[8] = angular_variance_; // z

    // Inject Linear Acceleration Covariance
    corrected_msg.linear_acceleration_covariance.fill(0.0);
    corrected_msg.linear_acceleration_covariance[0] = linear_variance_; // x
    corrected_msg.linear_acceleration_covariance[4] = linear_variance_; // y
    corrected_msg.linear_acceleration_covariance[8] = linear_variance_; // z

    pub_->publish(corrected_msg);
}

// Standard main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuCovarianceInjector>());
    rclcpp::shutdown();
    return 0;
}