#ifndef EXERCISE_NH_HPP
#define EXERCISE_NH_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Exercise
{
public:
    Exercise();
    std::shared_ptr<rclcpp::Node> get_node_handle() { return nh_; }

private:

    std::shared_ptr<rclcpp::Node> nh_;  // 노드 핸들

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void motion();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double laser_forward_;
};

#endif // EXERCISE_NH_HPP

