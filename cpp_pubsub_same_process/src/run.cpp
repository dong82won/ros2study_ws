#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node
{
public:
    Talker(const std::string &node_name) : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic_name", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Talker::publish_message, this));
    }

private:
    void publish_message()
    {
        auto msg = std_msgs::msg::String();
        msg.data = std::string("Hello from ") + this->get_name();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class Listener : public rclcpp::Node
{
public:
    Listener(const std::string &node_name) : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>("topic_name", 10, std::bind(&Listener::topic_callback, this, std::placeholders::_1));
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

    auto talker = std::make_shared<Talker>("talker");
    auto listener = std::make_shared<Listener>("listener");

    // SingleThreadedExecutor 사용
    // 동일한 프로세스 내 실행: talker와 listener가 동일한 프로세스 내에서 실행되도록 SingleThreadedExecutor를 사용합니다.
    // 인트라프로세스 통신 활성화: NodeOptions::use_intra_process_comms(true)를 통해 인트라프로세스 통신을 활성화하여 성능을 향상시킵니다.
    // ROS2에서는 여러 노드를 동일한 프로세스 내에서 실행할 수 있으며, 이는 성능과 메모리 효율성을 높이는 데 유용함.


    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(talker);
    executor.add_node(listener);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
