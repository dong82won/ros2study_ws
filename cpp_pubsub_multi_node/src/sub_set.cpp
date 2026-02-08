// subscriber_member_function.cpp (여러 노드 생성)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node
{
public:
    Listener(const std::string &node_name, const std::string &topic_name) : Node(node_name)
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(topic_name, 10, std::bind(&Listener::topic_callback,
            this, std::placeholders::_1));
    }

private:
    //void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    void topic_callback(const std::shared_ptr<const std_msgs::msg::String> &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 여러 노드 생성
    auto listener1 = std::make_shared<Listener>("listener1", "/topic1");
    auto listener2 = std::make_shared<Listener>("listener2", "/topic2");

    // 각각의 노드를 별도의 스레드에서 실행
    std::thread thread1([&]() { rclcpp::spin(listener1); });
    std::thread thread2([&]() { rclcpp::spin(listener2); });

    thread1.join();
    thread2.join();

    rclcpp::shutdown();
    return 0;
}
