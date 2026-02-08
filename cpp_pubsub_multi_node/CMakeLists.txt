cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub_multi_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(talker src/pub.cpp)
add_executable(talker_set src/pub_set.cpp)
add_executable(listener src/sub.cpp)
add_executable(listener_set src/sub_set.cpp)


target_include_directories(talker PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(talker ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

target_include_directories(listener PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(listener ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

install(TARGETS talker listener
        DESTINATION lib/${PROJECT_NAME})


target_include_directories(talker_set PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(talker_set ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

target_include_directories(listener_set PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(listener_set ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

install(TARGETS talker_set listener_set
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
