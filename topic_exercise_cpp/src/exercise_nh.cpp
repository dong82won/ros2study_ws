#include "topic_exercise_cpp/exercise_nh.hpp"

Exercise::Exercise()
    : laser_forward_(0.0)
{
    // 노드 생성
    nh_ = std::make_shared<rclcpp::Node>("topic_exercise_node");

    // cmd_vel 퍼블리셔 생성
    publisher_ = nh_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // /scan 서브스크라이버 생성
    subscriber_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)),
        std::bind(&Exercise::laser_callback, this, std::placeholders::_1));

    // 주기적인 모션 업데이트를 위한 타이머 생성
    timer_ = nh_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Exercise::motion, this));
}

void Exercise::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 정면(0도) 방향의 거리 저장
    laser_forward_ = msg->ranges[359];
}

void Exercise::motion()
{
    // 레이저 데이터 로깅
    RCLCPP_INFO(nh_->get_logger(), "I receive: '%f'", laser_forward_);

    // 로봇 움직임을 위한 Twist 메시지 정의
    geometry_msgs::msg::Twist cmd;

    // 라이다 데이터를 기반으로 로봇 움직임 결정
    if (laser_forward_ > 5.0)
    {
        cmd.linear.x = 0.5;
        cmd.angular.z = 0.5;
    }
    else if (laser_forward_ < 5.0 && laser_forward_ >= 0.5)
    {
        cmd.linear.x = 0.2;
        cmd.angular.z = 0.0;
    }
    else
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }

    // cmd_vel 토픽에 Twist 메시지 퍼블리시
    publisher_->publish(cmd);
}

int main(int argc, char **argv)
{
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // Exercise 객체 생성
    auto exercise_node = std::make_shared<Exercise>();

    // Executor를 사용하여 노드 실행
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(exercise_node->get_node_handle());
    executor.spin();

    // ROS2 종료
    rclcpp::shutdown();
    return 0;
}
