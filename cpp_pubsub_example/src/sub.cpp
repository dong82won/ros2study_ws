#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node
{
public:
    Listener() : Node("listener")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&Listener::topic_callback, this, std::placeholders::_1));
    }

private:
    // 이슈!! 확인!!
    // void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    void topic_callback(const std::shared_ptr<const std_msgs::msg::String> &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto listener = std::make_shared<Listener>();
    rclcpp::spin(listener);
    rclcpp::shutdown();
    return 0;
}
