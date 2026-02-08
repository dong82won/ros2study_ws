// subscriber_member_function.cpp (여러 노드 생성)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener1 : public rclcpp::Node
{
public:
    Listener1() : Node("listener1")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>
                        ("topic", 10, std::bind(&Listener1::topic_callback, this, std::placeholders::_1));
    }

private:
    //void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    void topic_callback(const std::shared_ptr<const std_msgs::msg::String> &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received listener1: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


class Listener2 : public rclcpp::Node
{
public:
    Listener2() : Node("listener2")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>
                        ("topic", 10, std::bind(&Listener2::topic_callback, this, std::placeholders::_1));
    }

private:
    //void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    void topic_callback(const std::shared_ptr<const std_msgs::msg::String> &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received listener2: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 여러 노드 생성
    auto listener1 = std::make_shared<Listener1>();
    auto listener2 = std::make_shared<Listener2>();

    // // 각각의 구독자를 별도의 스레드에서 실행
    // std::thread thread1([&]()
    //                     { rclcpp::spin(listener1); });
    // std::thread thread2([&]()
    //                     { rclcpp::spin(listener2); });

    // thread1.join();
    // thread2.join();

    // MultiThreadedExecutor 사용
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(listener1);
    executor.add_node(listener2);

    // Executor 실행
    executor.spin();


    rclcpp::shutdown();
    return 0;
}
