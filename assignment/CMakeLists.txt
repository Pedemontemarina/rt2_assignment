cmake_minimum_required(VERSION 3.8)
project(assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)


# Server component
add_library(nav_server SHARED
  src/nav_server.cpp
)
ament_target_dependencies(nav_server
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  custom_interfaces
)
rclcpp_components_register_nodes(nav_server "assignment::NavServer")

# Client component
add_library(nav_client SHARED
  src/nav_client.cpp
)
ament_target_dependencies(nav_client
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  std_msgs
  custom_interfaces
  tf2
  tf2_ros
)
rclcpp_components_register_nodes(nav_client "assignment::NavClient")


install(PROGRAMS
  scripts/nav_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the component libraries
install(
  TARGETS nav_server nav_client
  EXPORT export_assignment
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
