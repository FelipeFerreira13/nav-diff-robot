cmake_minimum_required(VERSION 3.8)
project(robot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(utils REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetData.srv"
  "srv/GetAngle.srv"
  "srv/GetButton.srv"
  "srv/SensorDriver.srv"
  "action/SensorCommand.action"
  DEPENDENCIES
)

rosidl_get_typesupport_target(typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# --------------------------------------------------
# Sensors library
# --------------------------------------------------
add_library(sensor src/sensors.cpp)

target_include_directories(sensor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sensor 
  rclcpp
  rclcpp_action
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  utils
)

target_link_libraries( sensor "${typesupport_target}" )


# --------------------------------------------------
# Main executable
# --------------------------------------------------
add_executable(main src/main.cpp)

ament_target_dependencies(main 
  rclcpp
)

target_link_libraries(main
  sensor
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  sensor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the executable separately
install(
  TARGETS main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  utils
)

ament_package()
