#ifndef EXERCISE_HPP
#define EXERCISE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Exercise : public rclcpp::Node
{
public:
    Exercise();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void motion();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    double laser_forward_;
};

#endif // EXERCISE_HPP
