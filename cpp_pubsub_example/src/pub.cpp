#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node {
public:
    Talker() : Node("talker") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Talker::publish_message, this));
    }

private:
    void publish_message() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello ROS2!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto talker = std::make_shared<Talker>();
    rclcpp::spin(talker);
    rclcpp::shutdown();
    return 0;
}
