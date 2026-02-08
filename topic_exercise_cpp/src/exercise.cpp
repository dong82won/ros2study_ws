#include "topic_exercise_cpp/exercise.hpp"

Exercise::Exercise()
    : Node("topic_exercise_node"), laser_forward_(0.0)
{
    // Create publisher for cmd_vel topic
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create subscriber for /scan topic
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)),
        std::bind(&Exercise::laser_callback, this, std::placeholders::_1));

    // Create timer for periodic motion updates
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Exercise::motion, this));
}

void Exercise::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Save the frontal laser scan info at 0°
    laser_forward_ = msg->ranges[359];
}

void Exercise::motion()
{
    // Log the received laser data
    RCLCPP_INFO(this->get_logger(), "I receive: '%f'", laser_forward_);

    // Define the Twist message for robot movement
    geometry_msgs::msg::Twist cmd;

    // Logic for robot movement based on laser scan data
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

    // Publish the Twist message to cmd_vel topic
    publisher_->publish(cmd);
}

int main(int argc, char **argv)
{
    // Initialize ROS2 communication
    rclcpp::init(argc, argv);

    // Create and spin the Exercise node
    auto exercise_node = std::make_shared<Exercise>();

    if (0)
    {
        // Executor를 사용하여 노드 실행
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(exercise_node);
        executor.spin();
    }
    else
    {
        rclcpp::spin(exercise_node);
    }

    // Shutdown ROS2 communication
    rclcpp::shutdown();
    return 0;
}
