
#include <rclcpp/rclcpp.hpp>
#include <csignal>

#include "Y3SpaceDriver.h"

// This ensures that the node shuts down properly on SIGINT
void signal_handler(int signal)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down...");
    rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);

	auto driver = std::make_shared<Y3SpaceDriver>();
    driver->run();

    rclcpp::shutdown();
    return 0;
}
