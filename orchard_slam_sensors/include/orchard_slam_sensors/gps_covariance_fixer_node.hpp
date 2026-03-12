#ifndef ORCHARD_SLAM_SENSORS__GPS_COVARIANCE_FIXER_NODE_HPP_
#define ORCHARD_SLAM_SENSORS__GPS_COVARIANCE_FIXER_NODE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp> 

class GpsCovFixer : public rclcpp::Node
{
public:
    GpsCovFixer();
    rclcpp::QoS qos_gps = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

private:
    void cb_fix_gps_msg_covariance(sensor_msgs::msg::NavSatFix::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

#endif  // ORCHARD_SLAM_SENSORS__GPS_COVARIANCE_FIXER_NODE_HPP_