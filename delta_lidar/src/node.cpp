/*
 *  3iRoboticsLIDAR System II
 *  Driver Interface
 *
 *  Copyright 2017 3iRobotics
 *  All rights reserved.
 *
 *	Author: 3iRobotics, Data:2017-09-15
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "C3iroboticsLidar.h"
#include "../sdk/include/CSerialConnection.h"

#define DEG2RAD(x) ((x) * M_PI / 180.)
 
typedef struct _rslidar_data
{
    _rslidar_data() : signal(0), angle(0.0), distance(0.0) {}
    uint8_t signal;
    float angle;
    float distance;
} RslidarDataComplete;

using namespace std;
using namespace everest::hwdrivers;

void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub,
                  _rslidar_data *nodes,
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
    scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (360.0f - 1.0f);

    scan_msg->scan_time = scan_time;
    scan_msg->time_increment = scan_time / (double)(node_count - 1);
    scan_msg->range_min = 0.15;
    scan_msg->range_max = 8.0;

    scan_msg->ranges.resize(360, std::numeric_limits<float>::infinity());
    scan_msg->intensities.resize(360, 0.0);

    // Unpack data
    for (size_t i = 0; i < node_count; i++)
    {
        size_t current_angle = floor(nodes[i].angle);
        if (current_angle > 360.0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Lidar angle is out of range %d", (int)current_angle);
            continue;
        }
        float read_value = (float)nodes[i].distance;
        if (read_value < scan_msg->range_min || read_value > scan_msg->range_max)
            scan_msg->ranges[360 - 1 - current_angle] = std::numeric_limits<float>::infinity();
        else
            scan_msg->ranges[360 - 1 - current_angle] = read_value;

        float intensities = (float)nodes[i].signal;
        scan_msg->intensities[360 - 1 - current_angle] = intensities;
    }

    pub->publish(*scan_msg);
}

class Delta2BLidarNode : public rclcpp::Node
{
public:
    Delta2BLidarNode() : Node("delta_2b_lidar_node")
    {
        // Read parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<std::string>("frame_id", "laser");

        serial_port_ = this->get_parameter("serial_port").as_string();
        frame_id = this->get_parameter("frame_id").as_string();

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable();
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

        serial_connect.setBaud(serial_port_baudrate);
        serial_connect.setPort(serial_port_.c_str());
        if (serial_connect.openSimple())
        {
            RCLCPP_INFO(this->get_logger(), "Open serial port successful!");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Open serial port %s failed!", serial_port_.c_str());
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "3iRoboticsLidar connected");
        robotics_lidar.initilize(&serial_connect);
        start_scan_time = this->now();
    }

    void run()
    {
        while (rclcpp::ok())
        {   

            TLidarGrabResult result = robotics_lidar.getScanData();
            switch(result)
            {
                case LIDAR_GRAB_ING:
                {
                    break;
                }
                case LIDAR_GRAB_SUCESS:
                {
                    TLidarScan lidar_scan = robotics_lidar.getLidarScan();
                    size_t lidar_scan_size = lidar_scan.getSize();
                    
                    std::vector<RslidarDataComplete> send_lidar_scan_data;
                    send_lidar_scan_data.resize(lidar_scan_size);
                    RslidarDataComplete one_lidar_data;

                    for(size_t i = 0; i < lidar_scan_size; i++)
                    {
                        one_lidar_data.signal = lidar_scan.signal[i];
                        one_lidar_data.angle = lidar_scan.angle[i];
                        one_lidar_data.distance = lidar_scan.distance[i];
                        send_lidar_scan_data[i] = one_lidar_data;
                    }

                    float angle_min = DEG2RAD(0.0f);
                    float angle_max = DEG2RAD(359.0f);

                    end_scan_time = this->now();
                    scan_duration = (end_scan_time - start_scan_time).seconds() * 1e-3;
                    RCLCPP_INFO(this->get_logger(), "Receive Lidar count %u,  %f", lidar_scan_size, scan_duration);

                    // If successful, publish lidar scan
                    publish_scan(scan_pub, &send_lidar_scan_data[0], lidar_scan_size,
                                 start_scan_time, scan_duration,
                                 angle_min, angle_max,
                                 frame_id);

                    start_scan_time = end_scan_time;

                    break;
                }
                case LIDAR_GRAB_ERRO:
                {
                    break;
                }
                case LIDAR_GRAB_ELSE:
                {
                    RCLCPP_WARN(this->get_logger(), "LIDAR_GRAB_ELSE!");
                    break;
                }
            }

 
            
            usleep(50);
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }


private:
    int serial_port_baudrate = 115200;
    std::string serial_port_;
    std::string frame_id;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    CSerialConnection serial_connect;
    C3iroboticsLidar robotics_lidar;
    rclcpp::Time start_scan_time;
    rclcpp::Time end_scan_time;
    double scan_duration;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Delta2BLidarNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}