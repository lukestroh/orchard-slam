#include "orchard_slam_sensors/imu_covariance_fixer_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

ImuCovFixer::ImuCovFixer()
: Node("imu_cov_fixer")
{
  pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/fixed", 10);
  sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu",
    qos_imu,
    std::bind(&ImuCovFixer::cb_fix_imu_msg_covariance, this, std::placeholders::_1));
}

void ImuCovFixer::cb_fix_imu_msg_covariance(sensor_msgs::msg::Imu::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received IMU message with timestamp: %u.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
  // orientation variance (rad^2)
  msg->orientation_covariance[0] = 0.01;
  msg->orientation_covariance[4] = 0.01;
  msg->orientation_covariance[8] = 0.01;

  // angular velocity variance = stddev^2 (0.009^2)
  msg->angular_velocity_covariance[0] = 8.1e-5;
  msg->angular_velocity_covariance[4] = 8.1e-5;
  msg->angular_velocity_covariance[8] = 8.1e-5;

  // linear acceleration variance = stddev^2 (0.021^2)
  msg->linear_acceleration_covariance[0] = 4.41e-4;
  msg->linear_acceleration_covariance[4] = 4.41e-4;
  msg->linear_acceleration_covariance[8] = 4.41e-4;

  pub_->publish(*msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCovFixer>());
  rclcpp::shutdown();
  return 0;
}