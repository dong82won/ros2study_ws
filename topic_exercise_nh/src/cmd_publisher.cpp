#include "topic_exercise_nh/cmd_publisher.hpp"

CmdPublisher::CmdPublisher(const std::shared_ptr<rclcpp::Node> &node) {
    publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void CmdPublisher::publish_command(double linear, double angular) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    publisher_->publish(msg);
}
