#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "C3iroboticsLidar.h"
#include "../sdk/include/CSerialConnection.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0)

typedef struct _rslidar_data
{
    _rslidar_data() : signal(0), angle(0.0), distance(0.0) {}
    uint8_t signal;
    float angle;
    float distance;
} RslidarDataComplete;

using namespace std;
using namespace everest::hwdrivers;

class Delta2bLidarNode : public rclcpp::Node
{
public:
    Delta2bLidarNode()
        : Node("delta_2b_lidar_node"), serial_connect_(), robotics_lidar_()
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<std::string>("frame_id", "laser");

        serial_port_ = this->get_parameter("serial_port").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();

        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        serial_connect_.setBaud(115200);
        serial_connect_.setPort(serial_port_.c_str());

        if (serial_connect_.openSimple())
        {
            RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", serial_port_.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "3iRoboticsLidar connected");
        robotics_lidar_.initilize(&serial_connect_);

        start_scan_time_ = this->now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(67), // 약 15Hz
            [this]() { this->scan_callback(); });
    }

private:
    void scan_callback()
    {
        TLidarGrabResult result = robotics_lidar_.getScanData();
        switch (result)
        {
        case LIDAR_GRAB_ING:
            break;
        case LIDAR_GRAB_SUCESS:
        {
            TLidarScan lidar_scan = robotics_lidar_.getLidarScan();
            size_t lidar_scan_size = lidar_scan.getSize();
            std::vector<RslidarDataComplete> send_lidar_scan_data(lidar_scan_size);

            for (size_t i = 0; i < lidar_scan_size; i++)
            {
                send_lidar_scan_data[i].signal = lidar_scan.signal[i];
                send_lidar_scan_data[i].angle = lidar_scan.angle[i];
                send_lidar_scan_data[i].distance = lidar_scan.distance[i];
            }

            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);

            rclcpp::Time end_scan_time = this->now();
            double scan_duration = (end_scan_time - start_scan_time_).nanoseconds() / 1e9;

            RCLCPP_INFO(this->get_logger(), "Received Lidar data count: %zu", lidar_scan_size);

            publish_scan(scan_publisher_, send_lidar_scan_data, lidar_scan_size,
                         start_scan_time_, scan_duration, angle_min, angle_max, frame_id_);

            start_scan_time_ = end_scan_time;
            break;
        }
        case LIDAR_GRAB_ERRO:
            RCLCPP_WARN(this->get_logger(), "Lidar grab error!");
            break;
        case LIDAR_GRAB_ELSE:
            RCLCPP_WARN(this->get_logger(), "Lidar grab else case triggered!");
            break;
        }
    }

    void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub,
                      std::vector<RslidarDataComplete> &nodes,
                      size_t node_count, rclcpp::Time start,
                      double scan_time,
                      float angle_min, float angle_max,
                      std::string frame_id)
    {
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = frame_id;
        scan_msg->angle_min = angle_min;
        scan_msg->angle_max = angle_max;
        scan_msg->angle_increment = (angle_max - angle_min) / (360.0f - 1.0f);
        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / static_cast<double>(node_count - 1);
        scan_msg->range_min = 0.15;
        scan_msg->range_max = 5.0;

        scan_msg->ranges.resize(360, std::numeric_limits<float>::infinity());
        scan_msg->intensities.resize(360, 0.0);

        for (size_t i = 0; i < node_count; i++)
        {
            size_t current_angle = static_cast<size_t>(floor(nodes[i].angle));
            if (current_angle >= 360)
            {
                RCLCPP_WARN(this->get_logger(), "Lidar angle out of range: %zu", current_angle);
                continue;
            }

            float read_value = nodes[i].distance;
            if (read_value < scan_msg->range_min || read_value > scan_msg->range_max)
            {
                scan_msg->ranges[359 - current_angle] = std::numeric_limits<float>::infinity();
            }
            else
            {
                scan_msg->ranges[359 - current_angle] = read_value;
            }

            scan_msg->intensities[359 - current_angle] = static_cast<float>(nodes[i].signal);
        }

        pub->publish(*scan_msg);
    }

    std::string serial_port_;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_scan_time_;
    CSerialConnection serial_connect_;
    C3iroboticsLidar robotics_lidar_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Delta2bLidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
