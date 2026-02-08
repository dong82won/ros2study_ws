#ifndef LASER_SUBSCRIBER_HPP
#define LASER_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserSubscriber {
public:
    explicit LaserSubscriber(const std::shared_ptr<rclcpp::Node> &node);

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};

#endif // LASER_SUBSCRIBER_HPP
