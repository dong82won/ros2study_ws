#include "topic_exercise_nh/exercise_nh.hpp"
#include "topic_exercise_nh/laser_subscriber.hpp"
#include "topic_exercise_nh/cmd_publisher.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Exercise 객체 생성 및 노드 핸들 공유
    auto exercise = std::make_shared<Exercise>("exercise_node");
    auto node_handle = exercise->get_node_handle();

    // 구독자 및 발생자 구성
    auto laser_subscriber = std::make_shared<LaserSubscriber>(node_handle);
    auto cmd_publisher = std::make_shared<CmdPublisher>(node_handle);

    // 명령 메시지 퍼블리시 예제 실행 (타이머 없이 단순 호출)
    cmd_publisher->publish_command(0.5, 0.0);

    // Executor 실행
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_handle);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
