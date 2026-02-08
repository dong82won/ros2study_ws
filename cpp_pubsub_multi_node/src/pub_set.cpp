// publisher_member_function.cpp (여러 노드 생성)
// 멀티스레딩 사용
// 멀티스레드를 사용하여 두 노드를 동시에 실행할 수 있습니다.
// 이를 위해 std::thread를 활용합니다.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <sstream>
#include <thread>

class Talker : public rclcpp::Node
{
public:
    Talker(const std::string &node_name, const std::string &topic_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 여러 노드 생성
    auto talker1 = std::make_shared<Talker>("talker1", "/topic1");
    auto talker2 = std::make_shared<Talker>("talker2", "/topic2");

    // rclcpp::spin(talker1);
    // rclcpp::spin(talker2);

    // 각각의 노드를 별도의 스레드에서 실행
    std::thread thread1([&]() { rclcpp::spin(talker1); });
    std::thread thread2([&]() { rclcpp::spin(talker2); });

    thread1.join();
    thread2.join();

    rclcpp::shutdown();
    return 0;
}
