cmake_minimum_required(VERSION 3.20)
project(bagger_execution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(bagger_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rcl_action REQUIRED)

add_executable(container_unloader
  src/container_unloader.cpp
)

add_executable(dumping_executor
  src/dumping_executor.cpp
)

add_executable(target_recovery
  src/target_recovery.cpp
)

target_link_libraries(container_unloader PUBLIC
  ${bagger_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${control_msgs_TARGETS}
  "rclcpp::rclcpp"
  "rclcpp_action::rclcpp_action"
)

target_link_libraries(dumping_executor PUBLIC
  ${bagger_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  "rclcpp::rclcpp"
  "rclcpp_action::rclcpp_action"
)

target_link_libraries(target_recovery PUBLIC
  ${bagger_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  "rclcpp::rclcpp"
  "rclcpp_action::rclcpp_action"
  "tf2_ros::tf2_ros"
  "tf2::tf2"
)

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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS container_unloader
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS dumping_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS target_recovery
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
