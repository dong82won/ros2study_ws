#include "topic_exercise_nh/laser_subscriber.hpp"

LaserSubscriber::LaserSubscriber(const std::shared_ptr<rclcpp::Node> &node) {
    subscriber_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&LaserSubscriber::laser_callback, this, std::placeholders::_1)
    );
}

void LaserSubscriber::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("LaserSubscriber"), "Received laser data: %f", msg->ranges[0]);
}
