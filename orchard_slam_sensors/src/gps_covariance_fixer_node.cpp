#include <orchard_slam_sensors/gps_covariance_fixer_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

GpsCovFixer::GpsCovFixer()
: Node("gps_cov_fixer")
{
    pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fixed", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix",
        qos_gps,
        std::bind(&GpsCovFixer::cb_fix_gps_msg_covariance, this, std::placeholders::_1));
} 

void GpsCovFixer::cb_fix_gps_msg_covariance(sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received GPS message with timestamp: %u.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
    // position variance (m^2)
    msg->position_covariance[0] = 1.0;
    msg->position_covariance[4] = 1.0;
    msg->position_covariance[8] = 100.0;

    pub_->publish(*msg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsCovFixer>());
    rclcpp::shutdown();
    return 0;
}