#include <SerialInterface.h>

SerialInterface::SerialInterface(std::string port, int baudrate, int timeout)
    : m_port(std::move(port)), m_baudrate(baudrate), m_timeout(timeout),
    m_logger(rclcpp::get_logger("SerialInterface"))
{
}

SerialInterface::~SerialInterface()
{
    if (m_connection && m_connection->isOpen())
    {
        RCLCPP_INFO(m_logger, "[SerialInterface] Closing the serial port");
        m_connection->close();
    }
}

// void SerialInterface::serialConnect()
// {
//     try
//     {
//         m_connection = std::make_unique<Serial>(m_port, static_cast<uint32_t>(m_baudrate), Timeout::simpleTimeout(m_timeout));
//         if (m_connection->isOpen())
//         {
//             RCLCPP_INFO(m_logger, "Connection Established with Port: %s with baudrate: %d", m_port.c_str(), m_baudrate);
//         }
//     }
//     catch (const IOException &e)
//     {
//         RCLCPP_ERROR(m_logger, "Unable to connect port: %s", m_port.c_str());
//         RCLCPP_ERROR(m_logger, "Is the device plugged in? Is the serial port open?\nError: %s", e.what());
//     }
// }

void SerialInterface::serialConnect()
{
    int retries = 3;
    while (retries > 0)
    {
        try
        {
            m_connection = std::make_unique<serial::Serial>(m_port, static_cast<uint32_t>(m_baudrate), serial::Timeout::simpleTimeout(m_timeout));
            if (m_connection->isOpen())
            {
                RCLCPP_INFO(m_logger, "[SerialInterface] Connection established on port: %s at baudrate: %d", m_port.c_str(), m_baudrate);
                return;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(m_logger, "[SerialInterface] Connection failed. Retries left: %d", retries - 1);
        }
        retries--;
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    throw std::runtime_error("[SerialInterface] Failed to establish connection after multiple attempts");
}

void SerialInterface::serialWrite(uint8_t *buf, size_t len)
{
    size_t written = m_connection->write(buf, len);
    if (written != len)
    {
        //RCLCPP_WARN(m_logger, "Len: %ld .. Written: %ld", size_t(len), size_t(written));
        RCLCPP_WARN(m_logger, "[SerialInterface] Expected to write %zu bytes but wrote %zu bytes", len, written);
    }
}

void SerialInterface::serialWriteString(const std::string &str)
{
    size_t written = m_connection->write(str);
    if (written != str.size())
    {
        //RCLCPP_WARN(m_logger, "Expected to write %zu bytes, but wrote %zu bytes", str.size(), written);
        RCLCPP_WARN(m_logger, "[SerialInterface] Expected to write %zu bytes but wrote %zu bytes", str.size(), written);
    }
}

std::string SerialInterface::serialReadLine()
{
    return m_connection->readline();
}


uint8_t SerialInterface::serialReadByte()
{
    uint8_t buf;
    if (m_connection->available() > 0)
    {
        size_t bytes = m_connection->read(&buf, 1);
        if (bytes != 1)
        {
            RCLCPP_WARN(m_logger, "Unable to read");
        }
    }
    return buf;
}
 

// uint8_t *SerialInterface::serialReadBytes(size_t nbytes)
// {
//     uint8_t *buf = (uint8_t *)malloc(sizeof(uint8_t) * nbytes);
//     if (m_connection->available() > 0)
//     {
//         size_t bytes = m_connection->read(buf, nbytes);
//         if (bytes != nbytes)
//         {
//             RCLCPP_WARN(m_logger, "Unable to read");
//         }
//     }
//     return buf;
// }

std::vector<uint8_t> SerialInterface::serialReadBytes(size_t nbytes)
{
    std::vector<uint8_t> buf(nbytes);
    if (m_connection->available() > 0)
    {
        size_t bytes = m_connection->read(buf.data(), nbytes);
        if (bytes != nbytes)
        {
            RCLCPP_WARN(m_logger, "[SerialInterface] Unable to read the expected number of bytes");
        }
    }
    return buf;
}

size_t SerialInterface::available()
{
    return m_connection->available();
}
