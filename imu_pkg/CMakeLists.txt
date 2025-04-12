cmake_minimum_required(VERSION 3.5)
project(imu_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 테스트 할 경우 Debug 옵션으로 빌드하세요
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

# Include directories
include_directories(
  include
  include/y3space_driver
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

# Find libraries
find_library(SERIAL_LIBRARY NAMES serial PATHS /usr/local/lib REQUIRED)

add_library(serial_interface
            src/y3space_driver/SerialInterface.cpp
            )

# Add executable
add_executable(imu_node
              src/y3space_driver/main.cpp
              src/y3space_driver/Y3SpaceDriver.cpp
              )

target_link_libraries(imu_node
  serial_interface
  ${SERIAL_LIBRARY}
)

# Link libraries
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  std_msgs
  serial
)

# Install targets
# serial_interface
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Add ament package
ament_package()