cmake_minimum_required(VERSION 3.8)
project(robot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(utils REQUIRED)
find_package(robot_control REQUIRED)
find_package(robot_localization REQUIRED)




install(
  DIRECTORY include/
  DESTINATION include
)


install(
  DIRECTORY launch maps config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  nav2_msgs
  utils
  robot_control
  robot_localization
)

ament_package()
