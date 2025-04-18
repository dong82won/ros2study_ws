cmake_minimum_required(VERSION 3.8)
project(delta_lidar)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# SDK 경로 설정
set(delta_lidar_SDK_PATH "./sdk/")

# SDK 소스 파일 추가
file(GLOB delta_lidar_SDK_SRC
  "${delta_lidar_SDK_PATH}/src/*.cpp"
)

# Include directories
include_directories(
  ${delta_lidar_SDK_PATH}/include
  ${delta_lidar_SDK_PATH}/src
)

# delta_lidar_node 생성
add_executable(delta_lidar_node src/node.cpp ${delta_lidar_SDK_SRC})
add_executable(delta_lidar_client src/client.cpp ${delta_lidar_SDK_SRC})

target_include_directories(delta_lidar_node PRIVATE
  ${delta_lidar_SDK_PATH}/include
  ${delta_lidar_SDK_PATH}/src
)

# 올바른 방식으로 target_link_libraries 설정
ament_target_dependencies(delta_lidar_node
  rclcpp
  sensor_msgs
  std_srvs
)

ament_target_dependencies(delta_lidar_client
  rclcpp
  sensor_msgs
  std_srvs
)

# Install targets
install(TARGETS
  delta_lidar_node
  delta_lidar_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and other files
install(DIRECTORY
  #launch
  sdk
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp sensor_msgs std_srvs)

# Install the package
ament_package()
