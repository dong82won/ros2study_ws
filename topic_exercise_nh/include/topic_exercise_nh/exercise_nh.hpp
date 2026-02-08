#ifndef EXERCISE_NH_HPP
#define EXERCISE_NH_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Exercise 클래스: 기본 노드 핸들을 제공
class Exercise {
public:
    explicit Exercise(const std::string &node_name);

    // 노드 핸들을 반환하는 함수
    std::shared_ptr<rclcpp::Node> get_node_handle() const { return nh_; }

private:
    std::shared_ptr<rclcpp::Node> nh_;
};

#endif // EXERCISE_NH_HPP
