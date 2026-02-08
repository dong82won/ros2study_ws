cmake_minimum_required(VERSION 3.8)
project(cpp_pub1sub2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/pub.cpp)
add_executable(talker_count src/pub_count.cpp)
add_executable(listener src/sub.cpp)
add_executable(listener_count src/sub_count.cpp)
add_executable(listener_node src/sub_node.cpp)


target_include_directories(talker PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(talker ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

target_include_directories(talker_count PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(talker_count ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

target_include_directories(listener PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(listener ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

target_include_directories(listener_count PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(listener_count ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

target_include_directories(listener_node PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(listener_node ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

install(TARGETS talker talker_count
        listener listener_count listener_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
