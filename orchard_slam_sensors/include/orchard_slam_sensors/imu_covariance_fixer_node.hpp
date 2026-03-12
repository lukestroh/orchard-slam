#ifndef ORCHARD_SLAM_SENSORS__IMU_COVARIANCE_FIXER_NODE_HPP_
#define ORCHARD_SLAM_SENSORS__IMU_COVARIANCE_FIXER_NODE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuCovFixer : public rclcpp::Node
{
public:
  ImuCovFixer();
  rclcpp::QoS qos_imu = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

private:
  void cb_fix_imu_msg_covariance(sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

#endif  // ORCHARD_SLAM_SENSORS__IMU_COVARIANCE_FIXER_NODE_HPP_