#include "topic_exercise_nh/exercise_nh.hpp"

Exercise::Exercise(const std::string &node_name) {
    nh_ = std::make_shared<rclcpp::Node>(node_name);
}
