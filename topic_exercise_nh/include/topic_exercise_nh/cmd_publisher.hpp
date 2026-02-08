#ifndef CMD_PUBLISHER_HPP
#define CMD_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class CmdPublisher {
public:
    explicit CmdPublisher(const std::shared_ptr<rclcpp::Node> &node);

    void publish_command(double linear, double angular);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

#endif // CMD_PUBLISHER_HPP
