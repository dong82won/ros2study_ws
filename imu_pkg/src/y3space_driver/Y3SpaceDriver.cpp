// #include <string>
// #include <csignal>
// #include <memory>
// #include <chrono>
// #include <vector>
// #include <sstream>


#include <rclcpp/rclcpp.hpp>
#include "Y3SpaceDriver.h"
using namespace std::chrono_literals;

Y3SpaceDriver::Y3SpaceDriver()
    : Node("y3_space_driver")
{
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<int>("timeout", 60000); // 60000
    //this->declare_parameter<std::string>("mode", "ABSOLUTE_MAG");
    this->declare_parameter<std::string>("mode", "absolute");
    this->declare_parameter<std::string>("frame", "imu");

    m_port = this->get_parameter("port").as_string();
    m_baudrate = this->get_parameter("baudrate").as_int();
    m_timeout = this->get_parameter("timeout").as_int();
    m_mode = this->get_parameter("mode").as_string();
    m_frame = this->get_parameter("frame").as_string();

    // baud rates: 1200, 2400, 4800, 9600, 19200, 28800, 38400, 57600, 115200, 230400, 460800, 921600. The factory default baud rate is 115200.

    RCLCPP_INFO(this->get_logger(), "Port: %s", m_port.c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate: %d", m_baudrate);
    RCLCPP_INFO(this->get_logger(), "Timeout: %d", m_timeout);
    RCLCPP_INFO(this->get_logger(), "Mode: %s", m_mode.c_str());
    RCLCPP_INFO(this->get_logger(), "Frame: %s", m_frame.c_str());

    serial_interface_ = std::make_unique<SerialInterface>(m_port, m_baudrate, m_timeout);

    try
    {
        serial_interface_->serialConnect();
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to serial port: %s", e.what());
        rclcpp::shutdown();
        throw;
    }

    serial_interface_->serialWriteString(SET_AXIS_DIRECTIONS_X_Forward_Y_Right_Z_Up); // change axis directions
    rclcpp::sleep_for(1500ms);

    // IMU QoS (sensor_data)
    // rclcpp::QoS imu_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    sensor_qos.best_effort();
    sensor_qos.durability_volatile();
    sensor_qos.keep_last(10);

    m_imuPub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", sensor_qos);

    m_magPub = this->create_publisher<sensor_msgs::msg::MagneticField>("/mag/data_raw", sensor_qos);

    m_tempPub = this->create_publisher<std_msgs::msg::Float32>("/imu/temp", 10);

    RCLCPP_INFO(this->get_logger(), "Y3SpaceDriver done ");
}

Y3SpaceDriver::~Y3SpaceDriver() {}

void Y3SpaceDriver::restoreFactorySettings()
{
    serial_interface_->serialWriteString(RESTORE_FACTORY_SETTINGS);
}

const std::string Y3SpaceDriver::getSoftwareVersion()
{
    serial_interface_->serialWriteString(GET_FIRMWARE_VERSION_STRING);

    const std::string buf = serial_interface_->serialReadLine();
    RCLCPP_INFO(this->get_logger(), "Software version: %s", buf.c_str());
    return buf;
}

const std::string Y3SpaceDriver::getAxisDirection()
{
    serial_interface_->serialWriteString(GET_AXIS_DIRECTION);

    const std::string buf = serial_interface_->serialReadLine();
    const std::string ret = [&]()
    {
        if (buf == "0\r\n")
        {
            return "X: Right, Y: Up, Z: Forward";
        }
        else if (buf == "1\r\n")
        {
            return "X: Right, Y: Forward, Z: Up";
        }
        else if (buf == "2\r\n")
        {
            return "X: Up, Y: Right, Z: Forward";
        }
        else if (buf == "3\r\n")
        {
            return "X: Forward, Y: Right, Z: Up";
        }
        else if (buf == "4\r\n")
        {
            return "X: Up, Y: Forward, Z: Right";
        }
        else if (buf == "5\r\n")
        {
            return "X: Forward, Y: Up, Z: Right";
        }
        else if (buf == "19\r\n")
        {
            return "X: Forward, Y: Left, Z: Up";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Buffer indicates: %s", buf.c_str());
            return "Unknown";
        }
    }();

    RCLCPP_INFO(this->get_logger(), "Axis Direction:  %s", ret.c_str());
    return ret;
}

void Y3SpaceDriver::startGyroCalibration(void)
{
    RCLCPP_INFO(this->get_logger(), "Starting Auto Gyro Calibration...");
    serial_interface_->serialWriteString(BEGIN_GYRO_AUTO_CALIB);

    rclcpp::sleep_for(std::chrono::seconds(5)); // ROS2에서 권장하는 sleep 사용 ;
    RCLCPP_INFO(this->get_logger(), "Proceeding");
}

void Y3SpaceDriver::setMIMode(bool on)
{
    if (on)
    {
        serial_interface_->serialWriteString(SET_MI_MODE_ENABLED);
    }
    else
    {
        serial_interface_->serialWriteString(SET_MI_MODE_DISABLED);
    }
}

const std::string Y3SpaceDriver::getCalibMode()
{
    serial_interface_->serialWriteString(GET_CALIB_MODE);

    const std::string buf = serial_interface_->serialReadLine();
    const std::string ret = [&]()
    {
        if (buf == "0\r\n")
        {
            return "Bias";
        }
        else if (buf == "1\r\n")
        {
            return "Scale and Bias";
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Buffer indicates: %s", buf.c_str());
            return "Unknown";
        }
    }();

    RCLCPP_INFO(this->get_logger(), "Calibration Mode:  %s", ret.c_str());
    return ret;
}

const std::string Y3SpaceDriver::getMIMode()
{
    serial_interface_->serialWriteString(GET_MI_MODE_ENABLED);

    const std::string buf = serial_interface_->serialReadLine();
    const std::string ret = [&]()
    {
        if (buf == "0\r\n")
        {
            return "Disabled";
        }
        else if (buf == "1\r\n")
        {
            return "Enabled";
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Buffer indicates: %s", buf.c_str());
            return "Unknown";
        }
    }();

    RCLCPP_INFO(this->get_logger(), "MI Mode: %s", ret.c_str());
    return ret;
}

//! Run the serial sync
// https://github.com/cagataysari/yostlab_imu/blob/master/src/YostLabDriver.cpp
void Y3SpaceDriver::run()
{
    RCLCPP_INFO(this->get_logger(), "Y3SpaceDriver run start ");

    std::vector<double> parsedVals;

    sensor_msgs::msg::Imu imuMsg;
    sensor_msgs::msg::MagneticField magMsg;
    std_msgs::msg::Float32 tempMsg;

    rclcpp::sleep_for(300ms); // usleep(300000);

    this->startGyroCalibration();
    this->getSoftwareVersion();
    this->getAxisDirection();
    this->getCalibMode();
    this->getMIMode();

    if (m_mode == MODE_ABSOLUTE)
    {
        RCLCPP_INFO(this->get_logger(), "Using absolute driver stream configuration");
        serial_interface_->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_ABSOLUTE);
    }
    else if (m_mode == MODE_RELATIVE)
    {
        RCLCPP_INFO(this->get_logger(), "Using relative driver stream configuration");
        serial_interface_->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_RELATIVE);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Unknown driver mode set... Defaulting to relative");
        serial_interface_->serialWriteString(SET_STREAMING_SLOTS_ROS_IMU_ABSOLUTE_MAG);
    }

    serial_interface_->serialWriteString(TARE_WITH_CURRENT_ORIENTATION);
    serial_interface_->serialWriteString(TARE_WITH_CURRENT_QUATERNION);
    // 100ms -> 10ms
    serial_interface_->serialWriteString(SET_STREAMING_TIMING_10_MS); // SET_STREAMING_TIMING_10_MS  SET_STREAMING_TIMING_100_MS
    serial_interface_->serialWriteString(START_STREAMING);

    RCLCPP_INFO(this->get_logger(), "Y3SpaceDriver run - Ready\n");

    // 다음은 각 모드에서 권장하는 ROS 발행 주기입니다:
    // 모드	센서 샘플링 주기 (최대)	권장 ROS 발행 주기	설명
    // Kalman AHRS 모드	250Hz	50Hz ~ 100Hz =>
    // QCOMP AHRS 모드	850Hz	100Hz ~ 250Hz
    // IMU 모드	1350Hz	250Hz ~ 500Hz (최대)
    // up to 250Hz with Kalman AHRS(higher with oversampling)
    rclcpp::Rate rate(50); // 20ms
    int line = 0;

    while (rclcpp::ok())
    {
        while (serial_interface_->available() > 0)
        {
            line += 1;
            std::string buf = serial_interface_->serialReadLine();
            // std::string parse;
            std::stringstream ss(buf);
            double i;

            // Parse data from the line
            while (ss >> i)
            {
                parsedVals.push_back(i);
                if (ss.peek() == ',')
                    ss.ignore();
            }
            // Should stop reading when line == number of tracked streams
            if (line == 4)
            {
                // Reset line tracker
                line = 0;

                // Prepare IMU message
                // ROS 2 timestamp this->now();
                imuMsg.header.stamp = this->get_clock()->now();
                imuMsg.header.frame_id = m_frame;

                magMsg.header.stamp = this->get_clock()->now();
                magMsg.header.frame_id = m_frame;
                
                imuMsg.orientation.x = parsedVals[0];
                imuMsg.orientation.y = parsedVals[1];
                imuMsg.orientation.z = parsedVals[2];
                imuMsg.orientation.w = parsedVals[3];

                imuMsg.angular_velocity.x = parsedVals[4];
                imuMsg.angular_velocity.y = parsedVals[5];
                imuMsg.angular_velocity.z = parsedVals[6];

                imuMsg.linear_acceleration.x = parsedVals[7];
                imuMsg.linear_acceleration.y = parsedVals[8];
                imuMsg.linear_acceleration.z = parsedVals[9];


                // Magnetic field (Tesla, Gauss → Tesla 변환)
                magMsg.magnetic_field.x = parsedVals[10];//* 1e-4;
                magMsg.magnetic_field.y = parsedVals[11];// * 1e-4;
                magMsg.magnetic_field.z = parsedVals[12];// * 1e-4;

                // Prepare temperature message
                tempMsg.data = static_cast<float>(parsedVals[13]);

                // Clear parsed values
                parsedVals.clear();
                m_imuPub->publish(imuMsg);
                m_magPub->publish(magMsg);
                m_tempPub->publish(tempMsg);
            }
        }
        rate.sleep();
        // ROS 2 equivalent of ros::spinOnce()
        rclcpp::spin_some(this->get_node_base_interface());
    }
}
